#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#include "ir.h"
#include "motor.h"

// ====== Tunables ======
#define LOOP_MS           10          // 100 Hz loop
#define BASE_PWM          160         // cruising speed
#define LINE_KP           40.0f       // proportional gain for steering
#define DIFF_CLAMP        60.0f       // max steering correction (+/-)
#define SLOW_ON_TURN      12.0f       // reduce base when |err| large (bigger = more slowdown)

#define LOST_TIMEOUT_MS   300         // after this long off the line, start recovery
#define RECOVER_PWM       80          // spin speed while searching
#define RECOVER_DIR       +1          // +1 spin right, -1 spin left

// ====== Helpers ======
static inline float clampf(float v, float lo, float hi){ if(v<lo)v=lo; if(v>hi)v=hi; return v; }
static inline int   clampi(int v, int lo, int hi){ if(v<lo)v=lo; if(v>hi)v=hi; return v; }

// R and L return 1 on black, 0 on white.
// error = R - L ∈ {-1,0,+1}  (right black → steer left; left black → steer right)
static inline float line_error(void){
    int L = ir_left_is_black();
    int R = ir_right_is_black();
    return (float)R - (float)L;
}

static void line_task(void* arg){
    // --- init subsystems ---
    motor_init();
    disable_pid_control();

    ir_init();

    printf("[LINE] Ready. BASE=%d KP=%.1f CLAMP=%d\n", BASE_PWM, LINE_KP, DIFF_CLAMP);

    // --- state for “lost line” detection ---
    absolute_time_t last_seen = get_absolute_time();

    for(;;){
        int L = ir_left_is_black();
        int R = ir_right_is_black();

        if (L || R){
            // we see the line on at least one sensor → normal follow
            last_seen = get_absolute_time();

            // proportional steering
            float err  = (float)R - (float)L;               // -1,0,+1 (can be fractional if you later add analog)
            float diff = clampf(LINE_KP * err, -DIFF_CLAMP, DIFF_CLAMP);

            // optional slowdown in tighter turns: lower base when |err| is big
            float base = (float)BASE_PWM - SLOW_ON_TURN * (float)fabsf(err);
            if (base < 60) base = 60;                       // don't stall

            int Lp = (int)(base - diff);
            int Rp = (int)(base + diff);

            forward_motor_manual(Lp, Rp);
        } else {
            // both sensors see white → maybe off the line
            int64_t ms_since = to_ms_since_boot(get_absolute_time()) -
                               to_ms_since_boot(last_seen);

            if (ms_since < LOST_TIMEOUT_MS) {
                // brief gap (e.g., crossing)—coast straight a bit
                forward_motor_manual(BASE_PWM, BASE_PWM);
            } else {
                // line lost → spin to search (fixed direction)
                int s = RECOVER_PWM;
                if (RECOVER_DIR > 0) {
                    forward_motor_manual(+s, -s);           // spin right
                } else {
                    forward_motor_manual(-s, +s);           // spin left
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_MS));
    }
}

int main(void){
    stdio_init_all();
    sleep_ms(800);
    printf("\n[DEMO2-IR] Line-follow only (no barcode, no IMU)\n");

    xTaskCreate(line_task, "line", 1024, NULL, 2, NULL);
    vTaskStartScheduler();

    while(1){}
    return 0;
}
