// ==============================
// main.c — Clean version (Demo 2)
// ==============================
// - Slow PID line following (6 cm/s)
// - One IR sensor (ADC)
// - Hysteresis line detection
// - "Remember last turn" recovery
// - No ultrasonic / obstacle detection
// ==============================

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

// === Your existing motor + encoder API headers ===
#include "motor.h"
#include "encoder.h"

// ================== CONFIG ==================

// IR line sensor pin: GPIO 28 (ADC2)
#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 28
#endif

// Hysteresis thresholds (tuned for your data: floor ~170, black ~4070)
#ifndef TH_LO
#define TH_LO   600     // definitely OFF line (white floor)
#endif
#ifndef TH_HI
#define TH_HI   3000    // definitely ON line (black tape)
#endif

#ifndef LEFT
#define LEFT  0
#endif
#ifndef RIGHT
#define RIGHT 1
#endif

// Slow crawl speed (PID target, in cm/s)
#ifndef SLOW_SPEED_CMPS
#define SLOW_SPEED_CMPS 2.0f
#endif

// Base steering window for turn search (ms)
#ifndef STEER_DURATION
#define STEER_DURATION 220
#endif

// Debounce between decisions (ms)
#ifndef DEBOUNCE_DELAY_MS
#define DEBOUNCE_DELAY_MS 500
#endif

// Pivot PWM while searching
#ifndef SEARCH_PWM
#define SEARCH_PWM 50
#endif

// ================== Helpers ==================
static inline bool on_line_hys(uint16_t v) {
    static bool state = false;
    if (v >= TH_HI)      state = true;    // clearly black
    else if (v <= TH_LO) state = false;   // clearly white
    return state;
}

static void init_line_sensor(void) {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2); // GPIO28 -> ADC2
}

// Turn and poll IR; exit early if the line is seen again.
static bool line_follow_turn_motor(int direction_0isLeft_1isRight, uint32_t steer_duration_ms) {
    disable_pid_control();
    const int motor_dir = (direction_0isLeft_1isRight == 0) ? LEFT : RIGHT;

    turn_motor_manual(motor_dir, CONTINUOUS_TURN, SEARCH_PWM, SEARCH_PWM);

    const uint64_t start = time_us_64();
    const uint64_t max_us = (uint64_t)steer_duration_ms * 1000ULL;

    while ((time_us_64() - start) < max_us) {
        uint16_t v = adc_read();
        if (on_line_hys(v)) {
            stop_motor_pid();
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    stop_motor_pid();
    vTaskDelay(pdMS_TO_TICKS(50));
    return false;
}

// ================== Interrupts ==================
void driver_callbacks(uint gpio, uint32_t events) {
    switch (gpio) {
        case L_ENCODER_OUT: read_encoder_pulse(L_ENCODER_OUT, events); break;
        case R_ENCODER_OUT: read_encoder_pulse(R_ENCODER_OUT, events); break;
        default: break;
    }
}

static void init_interrupts(void) {
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true,  &driver_callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true,  &driver_callbacks);
}

// ================== Line-follow Task ==================
static void lineFollowTask(void *pvParameters) {
    uint64_t last_decide_time = 0;

    int last_turn_dir = 0;   // 0 = Left, 1 = Right
    int initial_turn_direction   = last_turn_dir;
    int alternate_turn_direction = 1 - initial_turn_direction;

    bool needs_second_turn       = false;

    uint32_t initial_steer_duration = STEER_DURATION;
    uint32_t second_steer_duration  = initial_steer_duration + 60;

    int consecutive_reversals = 0;
    int non_reversal_turns    = 0;

    printf("[LF] Slow-PID line following started. Preferring %s first.\n",
           (initial_turn_direction==0) ? "LEFT" : "RIGHT");

    for (;;) {
        const uint64_t now = time_us_64();
        if (now - last_decide_time < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decide_time = now;

        const uint16_t v = adc_read();

        if (on_line_hys(v)) {
            // ON LINE: crawl smoothly using PID
            forward_motor_pid(SLOW_SPEED_CMPS);

            initial_turn_direction   = last_turn_dir;
            alternate_turn_direction = 1 - initial_turn_direction;
            initial_steer_duration   = STEER_DURATION;
            second_steer_duration    = initial_steer_duration + 60;
            consecutive_reversals    = 0;
            non_reversal_turns       = 0;
            needs_second_turn        = false;
        } else {
            // LOST LINE: reverse a little
            if (consecutive_reversals < 3 || non_reversal_turns >= 3) {
                stop_motor_pid();
                reverse_motor_manual(130, 130);
                vTaskDelay(pdMS_TO_TICKS(90));
                if (consecutive_reversals < 3) consecutive_reversals++;
            }

            if (!needs_second_turn) {
                // First try
                bool found = line_follow_turn_motor(initial_turn_direction, initial_steer_duration);
                needs_second_turn = !found;
                if (found) last_turn_dir = initial_turn_direction;
            } else {
                // Second try (opposite side)
                bool found = line_follow_turn_motor(alternate_turn_direction, second_steer_duration);
                needs_second_turn = false;
                if (found) last_turn_dir = alternate_turn_direction;
            }

            // Escalate steer duration to cope with near-90° bends
            if (consecutive_reversals >= 3) {
                initial_steer_duration = (initial_steer_duration + 70 > 500)
                                           ? 500 : (initial_steer_duration + 70);
                second_steer_duration  = (initial_steer_duration + 60 > 520)
                                           ? 520 : (initial_steer_duration + 60);
                non_reversal_turns++;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ================== Main ==================
int main(void) {
    stdio_init_all();
    sleep_ms(400);

    init_line_sensor();
    encoder_init();
    motor_init();
    init_interrupts();

    xTaskCreate(lineFollowTask, "LineFollow", configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] Slow-PID line follow ready. TH_LO=%d, TH_HI=%d, speed=%.1f cm/s\n",
           TH_LO, TH_HI, SLOW_SPEED_CMPS);

    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}
