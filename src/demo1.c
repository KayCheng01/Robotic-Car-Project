// main_imu_outer_constant_speed_slew_MATCH.c
// Constant-speed drive WITH IMU OUTER LOOP.
// - Outer loop: IMU heading PID -> small turn-rate trim (delta_w)
// - Inner loops: per-wheel speed PID (both wheels track cm/s targets)
// - Straightness: encoder-based PI (uses speed mismatch R-L)
// - Base PWM (stiction breaker) + PWM slew limiter
//
// Uses your exact constants from the no-IMU version.

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"

// ========================= Robot geometry (added for IMU trim) =================
#define TRACK_WIDTH_M        0.115f   // wheel center-to-center distance (meters)

// ========================= User-tunable trims & limits (UNCHANGED) =============
#ifndef WHEEL_TRIM_LEFT
#define WHEEL_TRIM_LEFT      (-2)
#endif
#ifndef WHEEL_TRIM_RIGHT
#define WHEEL_TRIM_RIGHT     (+8)
#endif

// ========================= Target speed (UNCHANGED) ============================
#define V_TARGET_MPS         0.20f
#define V_TARGET_CMPS        (V_TARGET_MPS * 100.0f)
#define START_RAMP_MS        0        // unchanged

// ========================= Heading PID (added for IMU) =========================
#ifndef KP_HEADING
#define KP_HEADING           0.30f
#endif
#ifndef KI_HEADING
#define KI_HEADING           0.00f
#endif
#ifndef KD_HEADING
#define KD_HEADING           0.18f
#endif
#define HEADING_DEADBAND_DEG 2.0f
#define HDG_EMA_ALPHA        0.20f
#define DEG2RAD              (float)(M_PI / 180.0f)
#define HEADING_OFFSET_DEG   20.0f
#define HEADING_RATE_SCALE   0.02f    // gentle scale

// ========================= Wheel speed PID (UNCHANGED) =========================
// PID output = delta PWM in [0..PWM_MAX], base PWM added later
#define SPID_OUT_MIN         0.0f
#define SPID_OUT_MAX         (float)(PWM_MAX_RIGHT)
#define SPID_KP              6.0f
#define SPID_KI              0.8f
#define SPID_KD              0.0f

// ========================= Encoder straightness PI (UNCHANGED) =================
#define STRAIGHT_KP          1.2f
#define STRAIGHT_KI          0.30f
#define STRAIGHT_I_CLAMP     50.0f

// ========================= Base & Slew (UNCHANGED) =============================
#define BASE_PWM_L           (PWM_MIN_LEFT)
#define BASE_PWM_R           (PWM_MIN_RIGHT)
#define MAX_PWM_STEP         6

// ========================= Timing (UNCHANGED) ==================================
#define LOOP_DT_MS           10
#define DT_S                 ((float)LOOP_DT_MS / 1000.0f)

// ========================= Helpers =============================================
static inline float clampf(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline int clampi(int v, int lo, int hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float ema(float prev, float x, float a){
    return prev*(1.f - a) + x*a;
}
static inline float wrap_deg_pm180(float e){ while (e > 180.f) e -= 360.f; while (e < -180.f) e += 360.f; return e; }
static inline float wrap_deg_0_360(float e){ while (e < 0.f) e += 360.f; while (e >= 360.f) e -= 360.f; return e; }

// ========================= Simple PID ==========================================
typedef struct {
    float kp, ki, kd;
    float integ, prev_err;
    float out_min, out_max;
    float integ_min, integ_max;
} pid_ctrl_t;

static inline void pid_init(pid_ctrl_t* p, float kp, float ki, float kd,
                            float out_min, float out_max, float integ_min, float integ_max){
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integ = 0.f; p->prev_err = 0.f;
    p->out_min = out_min; p->out_max = out_max;
    p->integ_min = integ_min; p->integ_max = integ_max;
}
static inline float pid_update(pid_ctrl_t* p, float err, float dt){
    p->integ += err * dt;
    p->integ = clampf(p->integ, p->integ_min, p->integ_max);
    float deriv = (err - p->prev_err) / dt;
    p->prev_err = err;
    float u = p->kp * err + p->ki * p->integ + p->kd * deriv;
    return clampf(u, p->out_min, p->out_max);
}

// ========================= Globals =============================================
static imu_t g_imu;
static pid_ctrl_t g_pid_speed_L, g_pid_speed_R;

// ========================= Main ================================================
int main(void){
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(800);

    printf("\n[CTRL:IMU-OUTER] Boot. HeadingPID(%.2f/%.2f/%.2f)  SpeedPID(%.2f/%.2f/%.2f)\n",
           (double)KP_HEADING, (double)KI_HEADING, (double)KD_HEADING,
           (double)SPID_KP, (double)SPID_KI, (double)SPID_KD);

    // ---- IMU ----
    g_imu.i2c      = i2c1;
    g_imu.i2c_baud = IMU_I2C_BAUD;
    g_imu.pin_sda  = IMU_SDA_PIN;
    g_imu.pin_scl  = IMU_SCL_PIN;
    g_imu.mx_off = g_imu.my_off = g_imu.mz_off = 0.f;
    if (!imu_init(&g_imu)) { printf("[CTRL] IMU init failed\n"); while (1) { sleep_ms(500); } }
    printf("[CTRL] IMU OK\n");

    // ---- Motors & encoders ----
    motor_init();
    disable_pid_control(); // we drive PWM manually here
    printf("[CTRL] Motors OK (PWM L[%d..%d] R[%d..%d])\n",
           PWM_MIN_LEFT, PWM_MAX_LEFT, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

    encoder_init();
    printf("[CTRL] Encoders OK\n");

    // ---- Speed PIDs ----
    const float iwind = 300.0f;
    pid_init(&g_pid_speed_L, SPID_KP, SPID_KI, SPID_KD, SPID_OUT_MIN, SPID_OUT_MAX, -iwind, +iwind);
    pid_init(&g_pid_speed_R, SPID_KP, SPID_KI, SPID_KD, SPID_OUT_MIN, SPID_OUT_MAX, -iwind, +iwind);

    // ---- Capture target heading ----
    float filt_hdg = 0.f;
    for (int i = 0; i < 20; ++i){
        float h = imu_update_and_get_heading(&g_imu);
        h += HEADING_OFFSET_DEG;
        h = wrap_deg_0_360(h);
        filt_hdg = ema(filt_hdg, h, 0.20f);
        sleep_ms(10);
    }
    const float target_heading_deg = filt_hdg;
    printf("[CTRL] Target heading = %.1f deg (corrected)\n", (double)target_heading_deg);

    // ---- Timing & states ----
    absolute_time_t t0 = get_absolute_time();
    absolute_time_t t_next = make_timeout_time_ms(LOOP_DT_MS);
    float h_integ = 0.f, h_prev_err = 0.f;   // outer PID
    float s_int = 0.0f;                      // straightness PI
    int lastL = PWM_MIN_LEFT, lastR = PWM_MIN_RIGHT;

    while (true){
        // === 1) IMU heading ===
        float raw_hdg = imu_update_and_get_heading(&g_imu);
        raw_hdg += HEADING_OFFSET_DEG;
        raw_hdg = wrap_deg_0_360(raw_hdg);
        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);

        float h_err = wrap_deg_pm180(target_heading_deg - filt_hdg);
        if (fabsf(h_err) < HEADING_DEADBAND_DEG) h_err = 0.f;

        float h_deriv = (h_err - h_prev_err) / DT_S;
        h_prev_err = h_err;

        if (KI_HEADING > 0.f){
            float h_iw = 150.0f / fmaxf(KI_HEADING, 1e-6f);
            h_integ += h_err * DT_S;
            h_integ = clampf(h_integ, -h_iw, +h_iw);
        }

        float delta_heading_rate_deg_s =
            KP_HEADING * h_err + KI_HEADING * h_integ + KD_HEADING * h_deriv;

        // small turn-rate in rad/s, scaled gently
        float delta_w = delta_heading_rate_deg_s * DEG2RAD * HEADING_RATE_SCALE;

        // === 2) Command v (cm/s) with optional ramp (unchanged) ===
        float v_cmd_cmps = V_TARGET_CMPS;
        if (START_RAMP_MS > 0){
            int64_t ms_since = absolute_time_diff_us(t0, get_absolute_time()) / 1000;
            if (ms_since < START_RAMP_MS){
                float frac = (float)ms_since / (float)START_RAMP_MS;
                if (frac < 0.f) frac = 0.f; 
                if (frac > 1.f) frac = 1.f;
                v_cmd_cmps = V_TARGET_CMPS * frac;
            }
        }

        // === 3) (v, delta_w) -> left/right linear targets (cm/s) ===
        // vL = v - 0.5 * TRACK * delta_w ; vR = v + 0.5 * TRACK * delta_w   (m/s)
        // convert the differential term to cm/s by *100
        float diff_cmps = (0.5f * TRACK_WIDTH_M * delta_w) * 100.0f;
        float vL_target = v_cmd_cmps - diff_cmps;
        float vR_target = v_cmd_cmps + diff_cmps;

        // === 4) Measure speeds (cm/s) ===
        float vL_meas = get_left_speed();
        float vR_meas = get_right_speed();
        static float vL_prev = 0.0f, vR_prev = 0.0f;
        if (vL_meas < 0.0f) vL_meas = vL_prev; else vL_prev = vL_meas;
        if (vR_meas < 0.0f) vR_meas = vR_prev; else vR_prev = vR_meas;

        // === 5) Inner speed PIDs -> delta-PWM ===
        float uL = pid_update(&g_pid_speed_L, (vL_target - vL_meas), DT_S);
        float uR = pid_update(&g_pid_speed_R, (vR_target - vR_meas), DT_S);

        // === 6) Straightness PI (UNCHANGED gains) ===
        float s_err = (vR_meas - vL_meas);    // cm/s mismatch
        s_int += s_err * DT_S;
        s_int = clampf(s_int, -STRAIGHT_I_CLAMP, STRAIGHT_I_CLAMP);
        float s_trim = STRAIGHT_KP * s_err + STRAIGHT_KI * s_int;

        // === 7) Compose PWM = BASE + PID delta + trims, then clamp ===
        int pwmL = (int)lroundf(BASE_PWM_L + uL + WHEEL_TRIM_LEFT  + s_trim);
        int pwmR = (int)lroundf(BASE_PWM_R + uR + WHEEL_TRIM_RIGHT - s_trim);
        pwmL = clampi(pwmL, PWM_MIN_LEFT,  PWM_MAX_LEFT);
        pwmR = clampi(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

        // === 8) Slew limit (UNCHANGED) ===
        int dL = pwmL - lastL, dR = pwmR - lastR;
        if (dL >  MAX_PWM_STEP) pwmL = lastL + MAX_PWM_STEP;
        if (dL < -MAX_PWM_STEP) pwmL = lastL - MAX_PWM_STEP;
        if (dR >  MAX_PWM_STEP) pwmR = lastR + MAX_PWM_STEP;
        if (dR < -MAX_PWM_STEP) pwmR = lastR - MAX_PWM_STEP;

        forward_motor_manual(pwmL, pwmR);
        lastL = pwmL; lastR = pwmR;

        // === 9) Telemetry (~5 Hz) ===
        static int div = 0;
        if ((div++ % (1000/LOOP_DT_MS/5)) == 0) {
            printf("[CTRL:IMU-OUTER] hdg=%.1f(raw=%.1f) herr=%.2f dW=%.3f  v=%.1fcm/s  "
                   "vL[t/m]=%.2f/%.2f  vR[t/m]=%.2f/%.2f  s_err=%.2f s_int=%.2f str=%.2f  PWM[L=%d R=%d]\n",
                   (double)filt_hdg, (double)raw_hdg, (double)h_err, (double)delta_w,
                   (double)v_cmd_cmps,
                   (double)vL_target, (double)vL_meas,
                   (double)vR_target, (double)vR_meas,
                   (double)s_err, (double)s_int, (double)s_trim,
                   pwmL, pwmR);
        }

        // === 10) Loop timing (UNCHANGED) ===
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }
    return 0;
}
