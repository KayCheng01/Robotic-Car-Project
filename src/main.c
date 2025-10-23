// main.c — Outer-loop IMU heading PID (trim) + Inner-loop wheel-speed PID (per wheel)
// Week-10 Demo 1 skeleton tailored for encoder API returning cm/s

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"

// ========================= Robot geometry (EDIT THESE) =========================
#define TRACK_WIDTH_M        0.115f   // distance between wheel contact patches (meters)
#define WHEEL_RADIUS_M       0.032f   // wheel radius (meters)

// ========================= User-tunable trims & limits =========================
#ifndef WHEEL_TRIM_LEFT
#define WHEEL_TRIM_LEFT   (0.0f)    // PWM offset to balance mechanical bias (L)
#endif
#ifndef WHEEL_TRIM_RIGHT
#define WHEEL_TRIM_RIGHT  (0.0f)    // PWM offset to balance mechanical bias (R)
#endif

// Base speed ramp (on linear speed v), not on PWM
#define V_START            0.00f      // m/s at start
#define V_TARGET           0.20f      // m/s target for straight test
#define RAMP_TIME_MS       1500       // time to ramp v

// ========================= Heading PID (outer loop) =========================
// Output is a small angular velocity correction (rad/s) applied to wheel speed targets
#ifndef KP_HEADING
#define KP_HEADING         0.30f
#endif
#ifndef KI_HEADING
#define KI_HEADING         0.00f
#endif
#ifndef KD_HEADING
#define KD_HEADING         0.18f
#endif

#define HEADING_DEADBAND_DEG   2.0f
#define HDG_EMA_ALPHA          0.20f

// Manual offset to align IMU heading to "physical" forward
static const float HEADING_OFFSET_DEG = +20.0f;   // adjust this after compass vs IMU check

// ========================= Wheel speed PIDs (inner loops) ======================
// These regulate *wheel speeds* to targets (units = rad/s)
#define SPID_KP   0.90f
#define SPID_KI   0.15f
#define SPID_KD   0.00f

// Safety/output limits for inner loops (PWM range clamp done later)
#define SPID_OUT_MIN   (float)(PWM_MIN_LEFT)      // rough mapping to PWM range
#define SPID_OUT_MAX   (float)(PWM_MAX_RIGHT)

// ========================= Control loop timing =================================
#define LOOP_DT_MS      10                        // 100 Hz for inner/outer loops
#define DT_S            ((float)LOOP_DT_MS / 1000.0f)

// ========================= Helpers =============================================
static inline float clampf(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float ema(float prev, float x, float a){
    return prev*(1.f - a) + x*a;
}
static inline float wrap_deg_pm180(float e){       // [-180,180]
    while (e > 180.f) e -= 360.f;
    while (e < -180.f) e += 360.f;
    return e;
}
static inline float wrap_deg_0_360(float e){       // [0,360)
    while (e < 0.f)   e += 360.f;
    while (e >= 360.f) e -= 360.f;
    return e;
}
// Convert encoder linear speed from cm/s → rad/s
static inline float cmps_to_rads(float cmps){
    float mps = cmps / 100.0f;
    return mps / WHEEL_RADIUS_M;
}

// ========================= Simple PID struct ===================================
typedef struct {
    float kp, ki, kd;
    float integ;
    float prev_err;
    float out_min, out_max;     // saturate output
    float integ_min, integ_max; // anti-windup on I-term
} pid_ctrl_t;

static inline void pid_init(pid_ctrl_t* p, float kp, float ki, float kd,
                            float out_min, float out_max, float integ_min, float integ_max){
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integ = 0.f; p->prev_err = 0.f;
    p->out_min = out_min; p->out_max = out_max;
    p->integ_min = integ_min; p->integ_max = integ_max;
}

static inline float pid_update(pid_ctrl_t* p, float err, float dt){
    // I-term (with anti-windup clamp)
    p->integ += err * dt;
    p->integ = clampf(p->integ, p->integ_min, p->integ_max);

    float deriv = (err - p->prev_err) / dt;
    p->prev_err = err;

    float u = p->kp * err + p->ki * p->integ + p->kd * deriv;
    return clampf(u, p->out_min, p->out_max);
}

// ========================= Globals =============================================
static imu_t g_imu;
static pid_ctrl_t g_pid_speed_L, g_pid_speed_R;    // inner loops (per wheel)

// ========================= Main ================================================
int main(void){
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(800);

    printf("\n[CTRL] Boot. HeadingPID(Kp=%.2f Ki=%.2f Kd=%.2f), SpeedPID(Kp=%.2f Ki=%.2f Kd=%.2f)\n",
           (double)KP_HEADING, (double)KI_HEADING, (double)KD_HEADING,
           (double)SPID_KP, (double)SPID_KI, (double)SPID_KD);

    // ---- Init IMU ----
    g_imu.i2c      = i2c1;
    g_imu.i2c_baud = IMU_I2C_BAUD;
    g_imu.pin_sda  = IMU_SDA_PIN;
    g_imu.pin_scl  = IMU_SCL_PIN;
    g_imu.mx_off = g_imu.my_off = g_imu.mz_off = 0.f;
    if (!imu_init(&g_imu)) {
        printf("[CTRL] IMU init failed\n");
        while (1) { sleep_ms(500); }
    }
    printf("[CTRL] IMU OK\n");

    // ---- Init motors ----
    motor_init();
    printf("[CTRL] Motors OK (PWM L[%d..%d] R[%d..%d])\n",
           PWM_MIN_LEFT, PWM_MAX_LEFT, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

    // ---- Init encoders ----
    encoder_init();
    printf("[CTRL] Encoders OK\n");

    // ---- Heading reference = current IMU yaw (average a few samples) ----
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

    // ---- PIDs ----
    // Inner wheel-speed PIDs: set output roughly in PWM scale; adjust if you prefer "delta PWM" style.
    const float iwind = 300.0f;  // integral windup clamp (tune)
    pid_init(&g_pid_speed_L, SPID_KP, SPID_KI, SPID_KD,
             SPID_OUT_MIN, SPID_OUT_MAX, -iwind, +iwind);
    pid_init(&g_pid_speed_R, SPID_KP, SPID_KI, SPID_KD,
             SPID_OUT_MIN, SPID_OUT_MAX, -iwind, +iwind);

    // ---- Timebase ----
    absolute_time_t t_next = make_timeout_time_ms(LOOP_DT_MS);
    const absolute_time_t t_ramp_end = make_timeout_time_ms(RAMP_TIME_MS);

    // ---- Outer (heading) PID state ----
    float h_integ = 0.f;
    float h_prev_err = 0.f;

    // ---- Main loop ----
    while (true){
        // --- 1) IMU heading (deg), filtered & offset-corrected ---
        float raw_hdg = imu_update_and_get_heading(&g_imu);
        raw_hdg += HEADING_OFFSET_DEG;
        raw_hdg = wrap_deg_0_360(raw_hdg);
        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);

        // --- 2) Heading error + deadband ---
        float h_err = wrap_deg_pm180(target_heading_deg - filt_hdg);
        if (fabsf(h_err) < HEADING_DEADBAND_DEG) h_err = 0.f;

        // --- 3) Heading PID → delta_w (rad/s) ---
        float h_deriv = (h_err - h_prev_err) / DT_S;
        h_prev_err = h_err;

        if (KI_HEADING > 0.f) {
            // anti-windup for outer loop (keep modest)
            float h_iw = 150.0f / fmaxf(KI_HEADING, 1e-6f);
            h_integ += h_err * DT_S;
            h_integ = clampf(h_integ, -h_iw, +h_iw);
        }

        // Heading PID output in "deg/s equivalent"
        float delta_heading_rate_deg_s =
            KP_HEADING * h_err + KI_HEADING * h_integ + KD_HEADING * h_deriv;

        // Map small deg/s → rad/s trim (tune this scaling!)
        const float DEG2RAD = (float)M_PI / 180.0f;
        float delta_w = delta_heading_rate_deg_s * DEG2RAD * 0.02f; // 0.02 is a gentle scale

        // --- 4) Motion command (straight) + ramp v ---
        float v_cmd = V_TARGET;
        if (!time_reached(t_ramp_end)) {
            int64_t ms_left  = absolute_time_diff_us(get_absolute_time(), t_ramp_end)/1000;
            float frac = 1.f - (float)ms_left / (float)RAMP_TIME_MS;
            if (frac < 0.f) {
                frac = 0.f; if (frac > 1.f) frac = 1.f;
            }
            v_cmd = V_START + frac * (V_TARGET - V_START);
        }

        float w_cmd = 0.0f + delta_w;     // straight, trimmed by IMU

        // --- 5) Convert (v, w) → wheel speed targets (rad/s) ---
        float wL_target = (v_cmd - 0.5f * TRACK_WIDTH_M * w_cmd) / WHEEL_RADIUS_M;
        float wR_target = (v_cmd + 0.5f * TRACK_WIDTH_M * w_cmd) / WHEEL_RADIUS_M;

        // --- 6) Measure wheel speeds from encoders (cm/s) → rad/s ---
        float L_cmps = get_left_speed();
        float R_cmps = get_right_speed();

        // Handle invalid (-1) gracefully: treat as zero (or hold last)
        static float L_cmps_prev = 0.0f, R_cmps_prev = 0.0f;
        if (L_cmps < 0.0f) L_cmps = L_cmps_prev;
        if (R_cmps < 0.0f) R_cmps = R_cmps_prev;
        L_cmps_prev = L_cmps; R_cmps_prev = R_cmps;

        float wL_meas = cmps_to_rads(L_cmps);
        float wR_meas = cmps_to_rads(R_cmps);

        // --- 7) Inner PIDs (per wheel) → PWM commands ---
        float uL = pid_update(&g_pid_speed_L, (wL_target - wL_meas), DT_S);
        float uR = pid_update(&g_pid_speed_R, (wR_target - wR_meas), DT_S);

        // Apply trims and clamp to motor legal bounds
        int pwmL = (int)lroundf(clampf(uL + WHEEL_TRIM_LEFT,  PWM_MIN_LEFT,  PWM_MAX_LEFT));
        int pwmR = (int)lroundf(clampf(uR + WHEEL_TRIM_RIGHT, PWM_MIN_RIGHT, PWM_MAX_RIGHT));

        forward_motor_manual(pwmL, pwmR);

        // --- 8) Telemetry (rate-limited) ---
        static int div = 0;
        if ((div++ % (1000/LOOP_DT_MS/5)) == 0) { // ~5 Hz
            printf("[CTRL] hdg=%.1f(raw=%.1f) herr=%.2f dW=%.3f  v=%.2f w=%.3f  "
                   "wL[t/m]=%.2f/%.2f  wR[t/m]=%.2f/%.2f  PWM[L=%d R=%d]\n",
                   (double)filt_hdg, (double)raw_hdg, (double)h_err, (double)delta_w,
                   (double)v_cmd, (double)w_cmd,
                   (double)wL_target, (double)wL_meas,
                   (double)wR_target, (double)wR_meas,
                   pwmL, pwmR);
        }

        // --- 9) Loop timing ---
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }

    return 0;
}
