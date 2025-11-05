// ==============================
// main.c — IR Line Follow (ADC only, no Guard DO)
// ==============================
// - Primary sensor: ADC on GPIO28 (ADC2)
// - Hysteresis thresholds: TH_LO, TH_HI
// - Follow BLACK by default; set FOLLOW_WHITE=1 to follow white
// - Uses your motor PID for forward crawl
// - Remembers last successful turn direction; 2-stage search with escalation
// ==============================

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "encoder.h"

// ======== CONFIG ========
#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 28           // GPIO28 -> ADC2
#endif

// Follow target color: 0 = BLACK on white floor (default), 1 = WHITE on black
#ifndef FOLLOW_WHITE
#define FOLLOW_WHITE 0
#endif

// Hysteresis thresholds (tune to your floor/tape)
#ifndef TH_LO
#define TH_LO   600      // definitely WHITE
#endif
#ifndef TH_HI
#define TH_HI   3000     // definitely BLACK
#endif

// Crawl speed for PID
#ifndef SLOW_SPEED_CMPS
#define SLOW_SPEED_CMPS 2.0f
#endif

// Search behaviour
#ifndef SEARCH_PWM
#define SEARCH_PWM 50              // pivot PWM while searching
#endif
#ifndef STEER_DURATION
#define STEER_DURATION 70         // first sweep (ms)
#endif
#ifndef DEBOUNCE_DELAY_MS
#define DEBOUNCE_DELAY_MS 120      // decision cadence (ms)
#endif
#ifndef REVERSE_MS
#define REVERSE_MS 90              // back off before pivot (ms)
#endif

// Require a few consecutive "on-track" readings to accept reacquisition
#ifndef REACQUIRE_GOOD_SAMPLES
#define REACQUIRE_GOOD_SAMPLES 3
#endif

// ======== ADC init & helpers ========
static inline void init_line_adc(void) {
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2); // GPIO28 -> ADC2
}

// Hysteresis: return true if main sensor is on the target color
static inline bool adc_on_track(uint16_t v) {
    static bool state = false;
#if FOLLOW_WHITE
    if (v <= TH_LO)      state = true;    // on white
    else if (v >= TH_HI) state = false;   // off (black)
#else
    if (v >= TH_HI)      state = true;    // on black
    else if (v <= TH_LO) state = false;   // off (white)
#endif
    return state;
}

// Pivot + poll ADC; exit early when we’ve seen N consecutive "on-track"
static bool search_pivot_and_probe(bool want_left, uint32_t ms) {
    disable_pid_control(); // manual control during search
    turn_motor_manual(want_left ? 0 /*LEFT*/ : 1 /*RIGHT*/,
                      CONTINUOUS_TURN, SEARCH_PWM, SEARCH_PWM);

    const uint64_t start = time_us_64();
    const uint64_t limit = (uint64_t)ms * 1000ULL;
    int good = 0;

    while ((time_us_64() - start) < limit) {
        uint16_t raw = adc_read();
        if (adc_on_track(raw)) {
            if (++good >= REACQUIRE_GOOD_SAMPLES) {
                stop_motor_pid();
                return true;
            }
        } else {
            good = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    stop_motor_pid();
    vTaskDelay(pdMS_TO_TICKS(40));
    return false;
}

// ======== Line-follow Task (ADC-only) ========
static void lineFollowTask(void *pvParameters) {
    (void)pvParameters;

    int last_turn_dir = 0;        // 0 = Left, 1 = Right (remember last success)
    uint32_t first_ms  = STEER_DURATION;
    uint32_t second_ms = STEER_DURATION + 60;
    bool needs_second  = false;

    printf("[LF] ADC-only line-follow. Speed=%.1f cm/s, TH_LO=%d, TH_HI=%d, FOLLOW_%s\n",
           SLOW_SPEED_CMPS, TH_LO, TH_HI, FOLLOW_WHITE ? "WHITE" : "BLACK");

    uint64_t last_decide = 0;

    for (;;) {
        const uint64_t now = time_us_64();
        if (now - last_decide < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decide = now;

        uint16_t raw = adc_read();
        bool on_track = adc_on_track(raw);

        if (on_track) {
            // Normal follow via your PID
            forward_motor_pid(SLOW_SPEED_CMPS);

            // Reset search parameters on solid reacquisition
            first_ms  = STEER_DURATION;
            second_ms = STEER_DURATION + 60;
            needs_second = false;

        } else {
            // LOST: back off a bit then pivot search
            stop_motor_pid();
            reverse_motor_manual(130, 130);
            vTaskDelay(pdMS_TO_TICKS(REVERSE_MS));

            // Try last successful direction first, then the opposite
            int prefer_dir = last_turn_dir;

            bool found = false;
            if (!needs_second) {
                found = search_pivot_and_probe(prefer_dir == 0, first_ms);
                needs_second = !found;
                if (found) last_turn_dir = prefer_dir;
            } else {
                int alt_dir = 1 - prefer_dir;
                found = search_pivot_and_probe(alt_dir == 0, second_ms);
                needs_second = false;
                if (found) last_turn_dir = alt_dir;
            }

            // Escalate sweep windows gradually to handle sharp corners
            if (!found) {
                if (first_ms  < 500) first_ms  += 70;
                if (second_ms < 520) second_ms += 70;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// ======== Main ========
#ifndef INTEGRATED_BUILD
int main(void) {
    stdio_init_all();
    sleep_ms(5000);
    printf("[MAIN] DEBUG\n");

    init_line_adc();
    encoder_init();  // your encoders already register IRQs inside here
    motor_init();    // starts PID task, jump-start logic lives inside motor.c

    xTaskCreate(lineFollowTask, "LineFollow",
                configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] IR line-follow (ADC-only) ready.\n");
    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}
#endif // INTEGRATED_BUILD
