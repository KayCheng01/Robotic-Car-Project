// ==============================
// barcodedemo2.c — Barcode Detection with Movement Control
// ==============================
// - IR sensor on GP27 for barcode detection (Code39)
// - Decodes barcodes and interprets A-Z characters
// - Movement patterns based on decoded character:
//   A-C: Forward, D-F: Left turn, G-I: Right turn, etc.
// - Uses FreeRTOS tasks for barcode scanning and motor control
// ==============================

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// === Module headers ===
#include "motor.h"
#include "encoder.h"
#include "barcode.h"

// ================== CONFIG ==================

// IR barcode sensor pin (GP27)
#define BARCODE_SENSOR_PIN 27

// Movement parameters
#define FORWARD_SPEED_CMPS    3.0f   // Forward speed
#define TURN_SPEED_CMPS       2.0f   // Turn speed
#define TURN_DURATION_MS      300    // How long to turn
#define FORWARD_DURATION_MS   500    // How long to move forward

// Debounce for barcode scanning (microseconds)
#define BARCODE_DEBOUNCE_US   5000

// ================== Global Variables ==================

volatile bool barcode_scan_ready = false;
volatile uint64_t last_barcode_transition = 0;

// Barcode decoding variables
uint64_t barcode_timing[9] = {0};
uint16_t barcode_bar_count = 0;
bool barcode_scan_started = false;

char current_decoded_char = '#';
SemaphoreHandle_t barcode_decode_semaphore;
SemaphoreHandle_t motor_control_semaphore;

// ================== Barcode Decoding Helpers ==================

typedef struct {
    uint16_t index;
    uint64_t timing;
} BarTiming;

static int compare_bar_timings(const void *a, const void *b) {
    BarTiming *barA = (BarTiming *)a;
    BarTiming *barB = (BarTiming *)b;
    return (barB->timing > barA->timing) - (barB->timing < barA->timing);
}

/**
 * Decode Code39 barcode from timing information
 * Returns the decoded character (A-Z, 0-9, etc.) or '#' if invalid
 */
static char decode_code39_character(void) {
    // Create array of bar timings with indices
    BarTiming bar_timings[9];
    for (uint16_t i = 0; i < 9; i++) {
        bar_timings[i].index = i;
        bar_timings[i].timing = barcode_timing[i];
    }

    // Sort by timing (descending)
    qsort(bar_timings, 9, sizeof(BarTiming), compare_bar_timings);

    // Generate binary code: top 3 timings are '1' (wide bars), rest are '0'
    char binary_code[10];
    for (uint16_t i = 0; i < 9; i++) {
        binary_code[i] = '0';
    }
    binary_code[9] = '\0';

    for (uint16_t i = 0; i < 3; i++) {
        binary_code[bar_timings[i].index] = '1';
    }

    // Code39 character mapping (binary -> character)
    // Characters A-Z mapped to Code39 patterns
    const char *code39_chars = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
    const char *code39_patterns[] = {
        "000110100", // 0
        "100100001", // 1
        "001100001", // 2
        "101100000", // 3
        "000110001", // 4
        "100110000", // 5
        "001110000", // 6
        "000100101", // 7
        "100100100", // 8
        "001100100", // 9
        "100001001", // A
        "001001001", // B
        "101001000", // C
        "000011001", // D
        "100011000", // E
        "001011000", // F
        "000001101", // G
        "100001100", // H
        "001001100", // I
        "000011100", // J
        "100000011", // K
        "001000011", // L
        "101000010", // M
        "000010011", // N
        "100010010", // O
        "001010010", // P
        "000000111", // Q
        "100000110", // R
        "001000110", // S
        "000010110", // T
        "110000001", // U
        "011000001", // V
        "111000000", // W
        "010010001", // X
        "110010000", // Y
        "011010000", // Z
        "010000101", // -
        "110000100", // .
        "010101000", // $
        "010100010", // /
        "010001010", // +
        "000101010", // %
        "011000100"  // *
    };

    // Match binary code to character
    for (int i = 0; i < 43; i++) {
        if (strcmp(binary_code, code39_patterns[i]) == 0) {
            return code39_chars[i];
        }
    }

    return '#'; // Invalid character
}

// ================== Barcode Interrupt Handler ==================

static void barcode_irq_handler_demo2(uint gpio, uint32_t events) {
    if (gpio != BARCODE_SENSOR_PIN) return;

    uint64_t now = time_us_64();

    // Debounce
    if ((now - last_barcode_transition) < BARCODE_DEBOUNCE_US) {
        return;
    }

    last_barcode_transition = now;

    if (!barcode_scan_started) {
        // First transition: mark as started
        barcode_scan_started = true;
    } else {
        // Capture timing between transitions
        barcode_timing[barcode_bar_count] = now - last_barcode_transition;
        barcode_bar_count++;

        // When we've captured 9 bars, decode the character
        if (barcode_bar_count >= 9) {
            current_decoded_char = decode_code39_character();
            barcode_bar_count = 0;
            barcode_scan_started = false;

            // Signal the motor control task
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(barcode_decode_semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

// ================== Motor Control Based on Character ==================

/**
 * Execute movement based on decoded barcode character
 * Alternating turns based on letter position:
 * A, C, E, G, I, K, M, O, Q, S, U, W, Y: Turn LEFT
 * B, D, F, H, J, L, N, P, R, T, V, X, Z: Turn RIGHT
 */
static void execute_character_movement(char ch) {
    printf("[BARCODE] Decoded character: %c\n", ch);

    // Disable PID for manual control
    disable_pid_control();

    if (ch >= 'A' && ch <= 'Z') {
        // Calculate position in alphabet (0-25)
        int pos = ch - 'A';
        
        // Determine if it's odd or even position
        // Even position (0, 2, 4, ...) = A, C, E, ... = LEFT TURN
        // Odd position (1, 3, 5, ...) = B, D, F, ... = RIGHT TURN
        if (pos % 2 == 0) {
            // Even position: Turn LEFT (A, C, E, G, ...)
            printf("[MOVE] Letter %c (position %d) - Turning LEFT\n", ch, pos);
            turn_motor_manual(0, CONTINUOUS_TURN, 100, 100);
            vTaskDelay(pdMS_TO_TICKS(TURN_DURATION_MS));
            stop_motor_manual();
        } else {
            // Odd position: Turn RIGHT (B, D, F, H, ...)
            printf("[MOVE] Letter %c (position %d) - Turning RIGHT\n", ch, pos);
            turn_motor_manual(1, CONTINUOUS_TURN, 100, 100);
            vTaskDelay(pdMS_TO_TICKS(TURN_DURATION_MS));
            stop_motor_manual();
        }
    }
    else {
        printf("[MOVE] Invalid character: %c, stopping\n", ch);
        stop_motor_manual();
    }
}

// ================== Interrupt Setup ==================

static void init_barcode_gpio(void) {
    gpio_init(BARCODE_SENSOR_PIN);
    gpio_set_dir(BARCODE_SENSOR_PIN, GPIO_IN);
    // Optional: pull-up to handle sensor output
    gpio_pull_up(BARCODE_SENSOR_PIN);
}

static void init_barcode_irq_demo2(void) {
    gpio_set_irq_enabled_with_callback(
        BARCODE_SENSOR_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &barcode_irq_handler_demo2
    );
}

// ================== FreeRTOS Tasks ==================

/**
 * Task to process decoded barcode characters and control motor
 */
static void barcode_motor_control_task(void *pvParameters) {
    printf("[TASK] Barcode motor control task started\n");

    for (;;) {
        // Wait for barcode decode semaphore
        if (xSemaphoreTake(barcode_decode_semaphore, portMAX_DELAY)) {
            // Execute movement based on decoded character
            if (current_decoded_char != '#') {
                execute_character_movement(current_decoded_char);
                current_decoded_char = '#';
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * Optional: Monitor barcode status
 */
static void barcode_monitor_task(void *pvParameters) {
    printf("[TASK] Barcode monitor task started\n");

    for (;;) {
        printf("[MONITOR] Current char: %c, Bar count: %d\n", 
               current_decoded_char, barcode_bar_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ================== Encoder Interrupt Handler ==================

void driver_callbacks(uint gpio, uint32_t events) {
    switch (gpio) {
        case L_ENCODER_OUT: read_encoder_pulse(L_ENCODER_OUT, events); break;
        case R_ENCODER_OUT: read_encoder_pulse(R_ENCODER_OUT, events); break;
        default: break;
    }
}

static void init_interrupts(void) {
    gpio_set_irq_enabled_with_callback(L_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &driver_callbacks);
    gpio_set_irq_enabled_with_callback(R_ENCODER_OUT, GPIO_IRQ_EDGE_RISE, true, &driver_callbacks);
}

// ================== Main ==================

int main(void) {
    stdio_init_all();
    sleep_ms(500);

    printf("\n========== Barcode Demo 2 ==========\n");
    printf("IR Sensor on GP27 (Code39 Barcode Decoder)\n");
    printf("Character A-Z maps to different movements\n");
    printf("====================================\n\n");

    // Initialize components
    init_barcode_gpio();
    encoder_init();
    motor_init();
    init_interrupts();
    init_barcode_irq_demo2();

    // Create semaphores
    barcode_decode_semaphore = xSemaphoreCreateBinary();
    if (barcode_decode_semaphore == NULL) {
        printf("[ERROR] Failed to create barcode decode semaphore\n");
        return -1;
    }

    // Create FreeRTOS tasks
    xTaskCreate(barcode_motor_control_task, "BarcodeMotor", 
                configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);
    
    xTaskCreate(barcode_monitor_task, "BarcodeMonitor", 
                configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    printf("[MAIN] All systems initialized. Waiting for barcode...\n");

    vTaskStartScheduler();

    // Should never reach here
    while (true) {
        tight_loop_contents();
    }

    return 0;
}
