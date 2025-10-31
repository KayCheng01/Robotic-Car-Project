// main.c — Drive straight + decode Code39 while moving
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "config.h"
#include "barcode.h"   // IR_SENSOR_PIN + barcode APIs

// ===== Barcode event bridge (from your barcode.c) =====
extern char decoded_barcode_char;         // set when *X* completes
extern volatile bool is_scanning_allowed; // scan gate

// Mirror a clean line when a char is decoded (optional hook for actions)
static void vBarcodeEventTask(void *pv) {
    char last = 0;
    for (;;) {
        if (decoded_barcode_char != 0 && decoded_barcode_char != last) {
            last = decoded_barcode_char;
            printf("[BARCODE] Decoded: %c\n", last);
            // TODO: if (last=='S') slow/stop; etc.
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ================= Drive task (your testdemo2 logic) =================
   IMU outer-loop heading hold + inner per-wheel speed PID + straightness PI.
   Uses your config.h (KP/ KI/ KD, V_TARGET_CMPS, slew, PWM limits, etc.)
*/
typedef struct { float kp,ki,kd, integ, prev_err, out_min,out_max, integ_min,integ_max; } pid_ctrl_t;
static inline float clampf(float v,float lo,float hi){ return v<lo?lo:(v>hi?hi:v); }
static inline int   clampi(int v,int lo,int hi){ return v<lo?lo:(v>hi?hi:v); }
static inline float ema(float p,float x,float a){ return p*(1.f-a)+x*a; }
static inline float wrap_deg_pm180(float e){ while(e>180.f)e-=360.f; while(e<-180.f)e+=360.f; return e; }
static inline float wrap_deg_0_360(float e){ while(e<0.f)e+=360.f; while(e>=360.f)e-=360.f; return e; }
static inline void  pid_init(pid_ctrl_t*P,float kp,float ki,float kd,float omin,float omax,float imin,float imax){
    P->kp=kp;P->ki=ki;P->kd=kd;P->integ=0;P->prev_err=0;P->out_min=omin;P->out_max=omax;P->integ_min=imin;P->integ_max=imax;
}
static inline float pid_update(pid_ctrl_t*P,float err,float dt){
    P->integ = clampf(P->integ + err*dt, P->integ_min, P->integ_max);
    float u = P->kp*err + P->ki*P->integ + P->kd*((err - P->prev_err)/dt); P->prev_err = err;
    return clampf(u, P->out_min, P->out_max);
}

static void vDriveTask(void *pv) {
    // ---- IMU bring-up (same as your testdemo2) ----
    imu_t imu; imu.i2c=i2c1; imu.i2c_baud=IMU_I2C_BAUD; imu.pin_sda=IMU_SDA_PIN; imu.pin_scl=IMU_SCL_PIN;
    if (!imu_init(&imu)) { printf("[CTRL] IMU init failed\n"); vTaskDelete(NULL); }

    // ---- Inner wheel speed PIDs ----
    pid_ctrl_t pidL, pidR;
    pid_init(&pidL, SPID_KP, SPID_KI, SPID_KD, SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);
    pid_init(&pidR, SPID_KP, SPID_KI, SPID_KD, SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);

    // ---- Lock current heading ----
    float filt_hdg = 0.f;
    for (int i=0;i<20;i++){ float h=imu_update_and_get_heading(&imu); h+=HEADING_OFFSET_DEG;
        h=wrap_deg_0_360(h); filt_hdg=ema(filt_hdg,h,0.20f); vTaskDelay(pdMS_TO_TICKS(10)); }
    const float target_hdg = filt_hdg;

    float s_int=0.f; int lastL=BASE_PWM_L, lastR=BASE_PWM_R;
    float h_integ=0.f, h_prev_err=0.f;

    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        // (1) Heading filter
        float raw = imu_update_and_get_heading(&imu) + HEADING_OFFSET_DEG;
        raw = wrap_deg_0_360(raw); filt_hdg = ema(filt_hdg, raw, HDG_EMA_ALPHA);

        // (2) Outer heading PID -> delta_w
        float h_err = wrap_deg_pm180(target_hdg - filt_hdg);
        if (fabsf(h_err) < HEADING_DEADBAND_DEG) h_err = 0.f;
        float h_deriv = (h_err - h_prev_err) / DT_S; h_prev_err = h_err;
        if (KI_HEADING > 0.f) {
            float clamp = 150.0f / (KI_HEADING > 1e-6f ? KI_HEADING : 1e-6f);
            h_integ = clampf(h_integ + h_err*DT_S, -clamp, +clamp);
        }
        float d_heading_rate_deg_s = KP_HEADING*h_err + KI_HEADING*h_integ + KD_HEADING*h_deriv;
        float delta_w = d_heading_rate_deg_s * DEG2RAD * HEADING_RATE_SCALE;

        // (3) Split v, w to L/R targets (cm/s)
        float v_cmd = V_TARGET_CMPS;                       // from config.h
        float diff_cmps = (0.5f * TRACK_WIDTH_M * delta_w) * 100.0f;
        float vL_t = v_cmd - diff_cmps, vR_t = v_cmd + diff_cmps;

        // (4) Measured speeds (cm/s)
        float vL_m = get_left_speed(), vR_m = get_right_speed();

        // (5) Inner speed PIDs
        float uL = pid_update(&pidL, vL_t - vL_m, DT_S);
        float uR = pid_update(&pidR, vR_t - vR_m, DT_S);

        // (6) Straightness PI (encoder mismatch)
        float s_err = (vR_m - vL_m);
        s_int = clampf(s_int + s_err*DT_S, -STRAIGHT_I_CLAMP, STRAIGHT_I_CLAMP);
        float s_trim = STRAIGHT_KP*s_err + STRAIGHT_KI*s_int + GLOBAL_S_TRIM_OFFSET;

        // (7) Final PWM + clamp + slew
        int pwmL = (int)lroundf(BASE_PWM_L + uL + s_trim);
        int pwmR = (int)lroundf(BASE_PWM_R + uR - s_trim);
        pwmL = clampi(pwmL, PWM_MIN_LEFT,  PWM_MAX_LEFT);
        pwmR = clampi(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

        int dL = pwmL - lastL, dR = pwmR - lastR;
        if (dL >  MAX_PWM_STEP) pwmL = lastL + MAX_PWM_STEP;
        if (dL < -MAX_PWM_STEP) pwmL = lastL - MAX_PWM_STEP;
        if (dR >  MAX_PWM_STEP) pwmR = lastR + MAX_PWM_STEP;
        if (dR < -MAX_PWM_STEP) pwmR = lastR - MAX_PWM_STEP;

        forward_motor_manual(pwmL, pwmR);
        lastL = pwmL; lastR = pwmR;

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_DT_MS));
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(600);

    // ---- Drive stack (from your testdemo2/demo1) ----
    motor_init();
    encoder_init();

    // ---- Barcode stack: semaphore + task + IRQ on IR_SENSOR_PIN ----
    barcode_init();     // creates semaphore, starts scanner task, enables scan gate
    init_barcode_irq(); // attaches ISR on rising+falling edges for IR_SENSOR_PIN

    is_scanning_allowed = true;  // explicit

    // ---- Tasks ----
    xTaskCreate(vDriveTask,        "DriveTask",     2048, NULL, 1, NULL);
    xTaskCreate(vBarcodeEventTask, "BarcodeEvent",  1024, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) { printf("Scheduler exited!\n"); sleep_ms(1000); }
}
