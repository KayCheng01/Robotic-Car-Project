// ==============================
// main.c — Method 2: Straight-drive calibration (plain floor)
// ==============================
// Purpose:
//   Drive straight at a slow, constant speed and log encoder drift (L, R, Δ).
//   Use the Δ trend to tune wheel trim / straightness PI/PID.
//
// Notes:
// - No line following / search logic is active in this build.
// - Optional guard IR (DO) is read only for polarity/wiring confirmation.
// - Uses FreeRTOS; single task runs the calibration loop and then parks.
//
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

// Crawl speed for calibration (cm/s)
#ifndef SLOW_SPEED_CMPS
#define SLOW_SPEED_CMPS 2.0f
#endif

// Calibration duration (seconds)
#ifndef CAL_SECONDS
#define CAL_SECONDS 6
#endif

// Optional: guard IR (DO) just to sanity-check polarity/wiring during the run
// Active-LOW on black for most modules; pull-up enabled below.
#ifndef IR_GUARD_DO_GPIO
#define IR_GUARD_DO_GPIO 1   // set to your DO pin or -1 to disable
#endif

// If you still want to peek the ADC value while rolling (e.g., noise level),
// you can keep using GPIO28 (ADC2). Otherwise this can stay compiled but unused.
#ifndef LINE_SENSOR_PIN
#define LINE_SENSOR_PIN 28   // GPIO28 / ADC2
#endif

// ================== Helpers & Init ==================

static inline void init_optional_sensors(void) {
    // ADC line sensor (optional observability)
    adc_init();
    adc_gpio_init(LINE_SENSOR_PIN);
    adc_select_input(2); // GPIO28 -> ADC2

#if (IR_GUARD_DO_GPIO >= 0)
    // Guard IR (DO) for polarity/wiring check
    gpio_init(IR_GUARD_DO_GPIO);
    gpio_set_dir(IR_GUARD_DO_GPIO, GPIO_IN);
    gpio_pull_up(IR_GUARD_DO_GPIO);
#endif
}

// Encoder IRQ fan-out; relies on your encoder.h symbols
static void driver_callbacks(uint gpio, uint32_t events) {
    switch (gpio) {
        case L_ENCODER_OUT: read_encoder_pulse(L_ENCODER_OUT, events); break;
        case R_ENCODER_OUT: read_encoder_pulse(R_ENCODER_OUT, events); break;
        default: break;
    }
}

static void init_interrupts(void) {
    // Rising-edge only is typical for single-channel tick ISR
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true,  &driver_callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true,  &driver_callbacks);
}

// ================== Calibration Task ==================

static void straightCalTask(void *pvParameters) {
    (void)pvParameters;

    // Brief settle
    vTaskDelay(pdMS_TO_TICKS(200));

    // Reset encoders, start forward motion
    reset_encoder_counts();
    stop_motor_pid(); // ensure clean start
    forward_motor_pid(SLOW_SPEED_CMPS);

    printf("[CAL] Straight-drive mode\n");
    printf("      Target speed = %.2f cm/s, duration = %d s\n", SLOW_SPEED_CMPS, CAL_SECONDS);
#if (IR_GUARD_DO_GPIO >= 0)
    printf("      Guard DO on GP%d (LOW=black for most modules). Printing state while rolling.\n", IR_GUARD_DO_GPIO);
#endif
    printf("      Columns: time_ms, L, R, Δ=L-R, adc_raw(optional), guard(optional)\n");

    const uint32_t total_ticks = (CAL_SECONDS * 1000) / 50; // 50 ms per sample
    uint32_t t_ms = 0;

    for (uint32_t i = 0; i < total_ticks; ++i) {
        int l = get_left_encoder_count();
        int r = get_right_encoder_count();
        int d = l - r;

        // Optional probes
        uint16_t adc_raw = adc_read(); // harmless even on plain floor

#if (IR_GUARD_DO_GPIO >= 0)
        int guard = gpio_get(IR_GUARD_DO_GPIO); // 0=LOW(black), 1=HIGH(white)
        printf("[CAL] %6u  %7d %7d %7d  %5u  %d\n", t_ms, l, r, d, adc_raw, guard);
#else
        printf("[CAL] %6u  %7d %7d %7d  %5u\n", t_ms, l, r, d, adc_raw);
#endif

        vTaskDelay(pdMS_TO_TICKS(50));
        t_ms += 50;
    }

    stop_motor_pid();

    printf("[CAL] Completed.\n");
    printf("      If Δ grows positive steadily -> left wheel faster (trim right / reduce left bias).\n");
    printf("      If Δ grows negative steadily -> right wheel faster (trim left / reduce right bias).\n");
    printf("      If Δ oscillates -> tune straightness PI/PID or lower speed for calibration.\n");

    // Park forever
    for(;;) vTaskDelay(pdMS_TO_TICKS(1000));
}

// ================== Main ==================

int main(void) {
    stdio_init_all();
    sleep_ms(10000);

    init_optional_sensors();
    encoder_init();
    motor_init();
    init_interrupts();

    xTaskCreate(straightCalTask, "StraightCal", configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] Straight-drive calibration ready.\n");

    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}
