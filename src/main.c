// main.c (excerpt)

#include "ir.h"
#include "motor.h"
#include "encoder.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Tune these
#define BASE_L        PWM_MID_LEFT
#define BASE_R        PWM_MID_RIGHT
#define LOOP_DT_MS    20
#define OFF_DEBOUNCE_MS 100   // set 0 for instant stop
#define SPEED_PRINT_MS 500 

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void move_when_both_black_task(void *pv)
{
    (void)pv;
    uint32_t off_ms = 0;
    uint32_t print_ms = 0;

    const float TRIM_K = 0.12f;

    for (;;) {
    const int left_black  = ir_left_is_black();
    const int right_black = ir_right_is_black();

        if (left_black && right_black) {
            // both on black -> go forward with trim correction from encoders
            off_ms = 0;

            // read speeds (may be zero briefly after start)
            float vL = get_left_speed();
            float vR = get_right_speed();

            // proportional correction to reduce speed difference (push slower wheel up)
            float err = vL - vR;
            float corr = TRIM_K * err;

            // apply correction: subtract from faster side, add to slower
            float pwmL = BASE_L - corr;
            float pwmR = BASE_R + corr;

            // clamp to allowed PWM ranges from motor.h
            pwmL = clampf(pwmL, PWM_MIN_LEFT, PWM_MAX_LEFT);
            pwmR = clampf(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

            forward_motor_manual(pwmL, pwmR);
        } else {
            // one or both off black -> stop (with small debounce)
            off_ms += LOOP_DT_MS;
            if (off_ms >= OFF_DEBOUNCE_MS) {
                stop_motor_manual();
            }
        }
        
        print_ms += LOOP_DT_MS;
        if (print_ms >= SPEED_PRINT_MS) {
            print_ms = 0;
            float vL = get_left_speed();
            float vR = get_right_speed();
            float avg = (vL + vR) / 2.0f;
            printf("[ENC] L: %.2f  R: %.2f  AVG: %.2f  off_ms: %u\n", vL, vR, avg, (unsigned)off_ms);
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS));
    }
    }

int main(void)
{
    stdio_init_all();

    ir_init();        // uses IR_BLACK_IS_LOW in IR.h
    encoder_init();   // optional, for telemetry
    motor_init();     // your 2-pin PWM motor init

    xTaskCreate(move_when_both_black_task, "BothBlack",
                configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);

    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
}
