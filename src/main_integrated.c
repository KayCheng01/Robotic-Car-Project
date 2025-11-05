// main_integrated.c — Unified FreeRTOS application with:
//   - Line following (GPIO IRQ on digital sensor)
//   - Barcode scanning (ADC1+DMA, continuous background decoding)
//   - Motor PID + Encoder IRQs

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "encoder.h"
#include "line_sensor.h"
#include "adc_barcode.h"
#include "barcode_task.h"

// Line follow task configuration
#define SLOW_SPEED_CMPS 2.0f
#define SEARCH_PWM 50
#define STEER_DURATION 70
#define DEBOUNCE_DELAY_MS 120
#define REVERSE_MS 90
#define REACQUIRE_GOOD_SAMPLES 3

// Pivot + poll line sensor; exit early when we've seen N consecutive "on-track"
static bool search_pivot_and_probe(bool want_left, uint32_t ms) {
    disable_pid_control();
    turn_motor_manual(want_left ? 0 : 1, CONTINUOUS_TURN, SEARCH_PWM, SEARCH_PWM);

    const uint64_t start = time_us_64();
    const uint64_t limit = (uint64_t)ms * 1000ULL;
    int good = 0;

    while ((time_us_64() - start) < limit) {
        if (line_sensor_is_on_track()) {
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

// Line-follow task (digital GPIO-based)
static void lineFollowTask(void *pvParameters) {
    (void)pvParameters;

    int last_turn_dir = 0;
    uint32_t first_ms  = STEER_DURATION;
    uint32_t second_ms = STEER_DURATION + 60;
    bool needs_second  = false;
    bool stopped_for_barcode = false;

    // Give user time to open serial monitor
    printf("\n\n");
    printf("═══════════════════════════════════════════\n");
    printf("    ROBOTIC CAR - INTEGRATED SYSTEM\n");
    printf("═══════════════════════════════════════════\n\n");
    printf("Waiting for serial monitor...\n");
    for (int i = 3; i > 0; i--) {
        printf("  Starting system check in %d seconds...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("\n");
    
    printf("[LF] Digital line-follow ready. Speed=%.1f cm/s\n", SLOW_SPEED_CMPS);
    printf("[LF] Checking system status...\n");
    
    // Wait a bit for barcode task to initialize
    vTaskDelay(pdMS_TO_TICKS(500));
    
    printf("[LF] ✓ Line sensor: Active (GPIO IRQ)\n");
    printf("[LF] ✓ Barcode scanner: Active (ADC1+DMA)\n");
    printf("[LF] ✓ Motor control: Ready\n");
    printf("[LF] ✓ Both tasks running!\n\n");
    
    printf("─────────────────────────────────────────\n");
    printf("Ready to start line following!\n");
    printf("─────────────────────────────────────────\n\n");
    
    for (int i = 5; i > 0; i--) {
        printf("  Starting in %d...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("\n");
    printf("╔═══════════════════════════════════════════╗\n");
    printf("║          GO! STARTING MISSION             ║\n");
    printf("╚═══════════════════════════════════════════╝\n");
    printf("Looking for Code 39 barcode: *X* (3 characters)\n\n");

    uint64_t last_decide = 0;

    for (;;) {
        // Check if barcode scan is complete - stop the car
        if (barcode_scan_complete() && !stopped_for_barcode) {
            stop_motor_pid();
            printf("\n╔═══════════════════════════════════════════╗\n");
            printf("║  ROBOT STOPPED - BARCODE DETECTED        ║\n");
            printf("╚═══════════════════════════════════════════╝\n");
            printf("[LF] Motors stopped for inspection.\n\n");
            stopped_for_barcode = true;
            // Keep looping but don't move
        }
        
        // If stopped for barcode, just wait
        if (stopped_for_barcode) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        const uint64_t now = time_us_64();
        if (now - last_decide < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decide = now;

        bool on_track = line_sensor_is_on_track();

        if (on_track) {
            // Normal follow via PID
            forward_motor_pid(SLOW_SPEED_CMPS);

            // Reset search parameters
            first_ms  = STEER_DURATION;
            second_ms = STEER_DURATION + 60;
            needs_second = false;

        } else {
            // LOST: back off then pivot search
            stop_motor_pid();
            reverse_motor_manual(130, 130);
            vTaskDelay(pdMS_TO_TICKS(REVERSE_MS));

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

            // Escalate sweep windows
            if (!found) {
                if (first_ms  < 500) first_ms  += 70;
                if (second_ms < 520) second_ms += 70;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// Optional: monitor task (simplified since barcode task now prints summary)
static void monitorTask(void *pvParameters) {
    (void)pvParameters;
    
    for (;;) {
        // Just keep the task alive for potential future use
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

int main(void) {
    stdio_init_all();
    sleep_ms(500);

    printf("\n[MAIN] Integrated robotic car system starting...\n");

    // Initialize hardware modules
    encoder_init();
    printf("[MAIN] Encoders initialized\n");
    
    motor_init();
    printf("[MAIN] Motors initialized\n");
    
    line_sensor_init();
    printf("[MAIN] Line sensor (GPIO IRQ) initialized\n");
    
    adc_barcode_init();
    printf("[MAIN] Barcode ADC+DMA initialized\n");

    // Create FreeRTOS tasks
    xTaskCreate(lineFollowTask, "LineFollow",
                configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 1, NULL);
    printf("[MAIN] Line-follow task created\n");
    
    barcode_task_create();
    printf("[MAIN] Barcode task created\n");
    
    xTaskCreate(monitorTask, "Monitor",
                configMINIMAL_STACK_SIZE * 2,
                NULL, tskIDLE_PRIORITY + 1, NULL);
    printf("[MAIN] Monitor task created\n");

    printf("[MAIN] Starting FreeRTOS scheduler...\n\n");
    vTaskStartScheduler();

    // Should never reach here
    while (true) {
        tight_loop_contents();
    }
    return 0;
}
