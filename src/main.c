#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "motor.h"

// ========================= User-tunable params =========================

// Per-wheel trims found in Step 2 (PWM units). Edit if you re-trim later.
#ifndef WHEEL_TRIM_LEFT
#define WHEEL_TRIM_LEFT   (-6.0f)
#endif
#ifndef WHEEL_TRIM_RIGHT
#define WHEEL_TRIM_RIGHT  (+5.0f)
#endif

// Base speed ramp (relative to PWM_MIN_*)
#define BASE_START_OFFSET   5     // start at PWM_MIN + 5
#define BASE_TARGET_OFFSET  15    // ramp to PWM_MIN + 15
#define RAMP_TIME_MS        1500  // time to ramp base speed

// Heading control
#ifndef KP_HEADING
#define KP_HEADING   0.30f
#endif
#ifndef KI_HEADING
#define KI_HEADING   0.00f
#endif
#ifndef KD_HEADING
#define KD_HEADING   0.18f
#endif

#define HEADING_DEADBAND_DEG   2.0f     // ignore tiny errors
#define LOOP_DT_MS             50       // control loop period
#define HDG_EMA_ALPHA          0.20f    // heading low-pass

// Limit how much PID can bend each wheel away from the base
#define MAX_CORR_LEFT   ((PWM_MAX_LEFT  - PWM_MIN_LEFT ) * 0.20f)  // 20% range
#define MAX_CORR_RIGHT  ((PWM_MAX_RIGHT - PWM_MIN_RIGHT) * 0.20f)

// Heading offset calibration (manual)
const float HEADING_OFFSET = +20.0f;   // adjust this value based on compass vs IMU

// ========================= helpers =========================
static inline float clampf(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float ema(float prev, float x, float a){
    return prev*(1.f - a) + x*a;
}
// wrap to [-180, 180]
static inline float wrap_deg_pm180(float e){
    if (e > 180.f) e -= 360.f;
    if (e < -180.f) e += 360.f;
    return e;
}

// wrap to [0, 360)
static inline float wrap_deg_0_360(float e){
    if (e < 0) e += 360.f;
    if (e >= 360.f) e -= 360.f;
    return e;
}

// ========================= globals =========================
static imu_t g_imu;

int main(void){
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(1200);
    printf("\n[STEP4] Boot. PID(Kp=%.2f Ki=%.2f Kd=%.2f)  trims(L=%.1f R=%.1f)\n",
           (double)KP_HEADING, (double)KI_HEADING, (double)KD_HEADING,
           (double)WHEEL_TRIM_LEFT, (double)WHEEL_TRIM_RIGHT);

    // ---- IMU init ----
    g_imu.i2c      = i2c1;
    g_imu.i2c_baud = IMU_I2C_BAUD;
    g_imu.pin_sda  = IMU_SDA_PIN;
    g_imu.pin_scl  = IMU_SCL_PIN;
    g_imu.mx_off = g_imu.my_off = g_imu.mz_off = 0.f;
    if (!imu_init(&g_imu)) { 
        printf("IMU init failed\n"); 
        while(1){sleep_ms(500);} 
    }
    printf("[STEP4] IMU OK\n");

    // ---- Motors ----
    motor_init();
    printf("[STEP4] Motors OK (Lmin=%d Rmin=%d)\n", PWM_MIN_LEFT, PWM_MIN_RIGHT);

    // ---- Pick the target heading from current yaw (average a few samples) ----
    float hdg = 0.f;
    for (int i=0;i<12;i++){
        float temp = imu_update_and_get_heading(&g_imu);
        temp += HEADING_OFFSET;
        temp = wrap_deg_0_360(temp);
        hdg = temp;
        sleep_ms(20);
    }
    const float target_heading = hdg;
    printf("[STEP4] Target heading = %.1f deg (corrected)\n", (double)target_heading);

    // ---- Base PWM ramp setup ----
    const int BASE_START_L = PWM_MIN_LEFT  + BASE_START_OFFSET;
    const int BASE_START_R = PWM_MIN_RIGHT + BASE_START_OFFSET;
    const int BASE_TGT_L   = PWM_MIN_LEFT  + BASE_TARGET_OFFSET;
    const int BASE_TGT_R   = PWM_MIN_RIGHT + BASE_TARGET_OFFSET;

    int baseL = BASE_START_L;
    int baseR = BASE_START_R;

    // ---- PID state ----
    float filt_hdg = target_heading;   // filtered heading
    float err_prev = 0.f;
    float integ    = 0.f;
    const float dt = (float)LOOP_DT_MS / 1000.f;
    const float integ_limit = 200.0f / fmaxf(KI_HEADING, 0.0001f); // anti-windup

    // ---- Timebase ----
    absolute_time_t t_next = make_timeout_time_ms(LOOP_DT_MS);
    const absolute_time_t t_ramp_end = make_timeout_time_ms(RAMP_TIME_MS);

    // ---- Loop ----
    while (true){
        // 1) Get heading (low-pass + offset correction)
        float raw_hdg = imu_update_and_get_heading(&g_imu);
        raw_hdg += HEADING_OFFSET;
        raw_hdg = wrap_deg_0_360(raw_hdg);

        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);

        // 2) Error (wrap) + deadband
        float err = wrap_deg_pm180(target_heading - filt_hdg);
        if (fabsf(err) < HEADING_DEADBAND_DEG) err = 0.f;

        // 3) PID
        if (KI_HEADING > 0.f){
            integ += err * dt;
            integ = clampf(integ, -integ_limit, +integ_limit);
        }
        float deriv = (err - err_prev) / dt;
        err_prev = err;
        float corr = KP_HEADING*err + KI_HEADING*integ + KD_HEADING*deriv;

        // 4) Ramp base toward target during first RAMP_TIME_MS
        if (!time_reached(t_ramp_end)) {
            int64_t ms_left = absolute_time_diff_us(get_absolute_time(), t_ramp_end)/1000;
            int64_t ms_total = RAMP_TIME_MS;
            float frac = 1.f - (float)ms_left / (float)ms_total;
            if (frac < 0.f) frac = 0.f;
            if (frac > 1.f) frac = 1.f;
            baseL = BASE_START_L + (int)lroundf(frac * (BASE_TGT_L - BASE_START_L));
            baseR = BASE_START_R + (int)lroundf(frac * (BASE_TGT_R - BASE_START_R));
        } else {
            baseL = BASE_TGT_L;
            baseR = BASE_TGT_R;
        }

        // 5) Apply correction around base (L slower when corr>0, R faster)
        float pwmLf = (float)baseL - corr;
        float pwmRf = (float)baseR + corr;

        // limit correction excursion
        float dL = pwmLf - baseL;
        float dR = pwmRf - baseR;
        if (dL >  MAX_CORR_LEFT)  pwmLf = baseL + MAX_CORR_LEFT;
        if (dL < -MAX_CORR_LEFT)  pwmLf = baseL - MAX_CORR_LEFT;
        if (dR >  MAX_CORR_RIGHT) pwmRf = baseR + MAX_CORR_RIGHT;
        if (dR < -MAX_CORR_RIGHT) pwmRf = baseR - MAX_CORR_RIGHT;

        // apply trims last + clamp to legal bounds
        int outL = (int)lroundf(clampf(pwmLf + WHEEL_TRIM_LEFT,  PWM_MIN_LEFT,  PWM_MAX_LEFT));
        int outR = (int)lroundf(clampf(pwmRf + WHEEL_TRIM_RIGHT, PWM_MIN_RIGHT, PWM_MAX_RIGHT));

        // 6) Drive
        forward_motor_manual(outL, outR);

        // 7) Log
        static int print_div = 0;
        if ((print_div++ % 4) == 0) {
            printf("[STEP4] HDG=%.1f (raw=%.1f) ERR=%.2f  corr=%.2f  base[L=%d R=%d]  OUT[L=%d R=%d]\n",
                   (double)filt_hdg, (double)raw_hdg, (double)err, (double)corr,
                   baseL, baseR, outL, outR);
        }

        // 8) Loop timing
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }

    return 0;
}