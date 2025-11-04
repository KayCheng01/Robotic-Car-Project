// =============================================
// testdemo2_merged.c
// - Keeps your existing Demo2v4 line-follow from testdemo2.c (GP28 DO)
// - Adds EXACT Code-39 scanner logic from barcode_scanner.c on GP27/ADC1
// =============================================

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"
#include "encoder.h"

// ===============================
// LINE-FOLLOW (from your testdemo2.c)
// (unchanged logic; using DO on GP28)
// ===============================
#define LINE_SENSOR_DO_PIN     28  // If you were using DO; keep as in your testdemo2.c
#ifndef DO_ACTIVE_ON_BLACK
#define DO_ACTIVE_ON_BLACK     0   // 1 if DO=LOW on black; 0 if DO=HIGH on black
#endif

#ifndef SLOW_SPEED_CMPS
#define SLOW_SPEED_CMPS        2.0f
#endif
#ifndef SEARCH_PWM
#define SEARCH_PWM             50
#endif
#ifndef STEER_DURATION
#define STEER_DURATION         70
#endif
#ifndef DEBOUNCE_DELAY_MS
#define DEBOUNCE_DELAY_MS      120
#endif
#ifndef REVERSE_MS
#define REVERSE_MS             90
#endif
#ifndef REACQUIRE_GOOD_SAMPLES
#define REACQUIRE_GOOD_SAMPLES 3
#endif

static inline void init_line_digital(void) {
    gpio_init(LINE_SENSOR_DO_PIN);
    gpio_set_dir(LINE_SENSOR_DO_PIN, GPIO_IN);
}

static inline bool do_on_track_raw(void) {
    bool level = gpio_get(LINE_SENSOR_DO_PIN);
    return DO_ACTIVE_ON_BLACK ? (level == 0) : (level == 1);
}

static bool do_on_track_filtered(void) {
    int good = 0;
    for (int i = 0; i < 6; i++) {
        if (do_on_track_raw()) {
            if (++good >= 4) return true;
        } else {
            good = 0;
        }
        sleep_ms(2);
    }
    return false;
}

static bool search_pivot_and_probe_do(bool want_left, uint32_t ms) {
    disable_pid_control();
    turn_motor_manual(want_left ? 0 : 1, CONTINUOUS_TURN, SEARCH_PWM, SEARCH_PWM);

    const uint64_t start = time_us_64();
    const uint64_t limit = (uint64_t)ms * 1000ULL;
    int good = 0;

    while ((time_us_64() - start) < limit) {
        if (do_on_track_raw()) {
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

static void lineFollowTask(void *pvParameters) {
    (void)pvParameters;

    int last_turn_dir = 0;           // 0=Left, 1=Right
    uint32_t first_ms  = STEER_DURATION;
    uint32_t second_ms = STEER_DURATION + 60;
    bool needs_second  = false;

    printf("[LF/DO] Start on GP%d. Speed=%.2f, DO_ACTIVE_ON_BLACK=%d\n",
           LINE_SENSOR_DO_PIN, SLOW_SPEED_CMPS, DO_ACTIVE_ON_BLACK);

    uint64_t last_decide = 0;

    for (;;) {
        const uint64_t now = time_us_64();
        if (now - last_decide < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        last_decide = now;

        bool on_track = do_on_track_filtered();

        if (on_track) {
            forward_motor_pid(SLOW_SPEED_CMPS);
            first_ms  = STEER_DURATION;
            second_ms = STEER_DURATION + 60;
            needs_second = false;

        } else {
            stop_motor_pid();
            reverse_motor_manual(130, 130);
            vTaskDelay(pdMS_TO_TICKS(REVERSE_MS));

            int prefer_dir = last_turn_dir;
            bool found = false;

            if (!needs_second) {
                found = search_pivot_and_probe_do(prefer_dir == 0, first_ms);
                needs_second = !found;
                if (found) last_turn_dir = prefer_dir;
            } else {
                int alt_dir = 1 - prefer_dir;
                found = search_pivot_and_probe_do(alt_dir == 0, second_ms);
                needs_second = false;
                if (found) last_turn_dir = alt_dir;
            }

            if (!found) {
                if (first_ms  < 500) first_ms  += 70;
                if (second_ms < 520) second_ms += 70;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// ======================================================================
// BARCODE SCANNER — EXACT logic lifted from barcode_scanner.c (GP27/ADC1)
// (unchanged tables, thresholding, 9-element timing, and prints)
// ======================================================================

#ifndef IR_SENSOR_ADC_PIN
#define IR_SENSOR_ADC_PIN 27
#endif
#ifndef IR_SENSOR_ADC_INPUT
#define IR_SENSOR_ADC_INPUT 1   // ADC1 is GPIO27
#endif
#ifndef ADC_THRESHOLD
#define ADC_THRESHOLD 2000
#endif

#define TOTAL_CHAR 44
#define BARCODE_ELEMENTS 9
#define BARCODE_BUFFER_SIZE 100
#define END_OF_BARCODE_TIMEOUT_MS 3000

static char char_array[TOTAL_CHAR] = {
    '0','1','2','3','4','5','6','7','8','9',
    'A','B','C','D','E','F','G','H','I','J',
    'K','L','M','N','O','P','Q','R','S','T',
    'U','V','W','X','Y','Z','-','.', ' ', '*', '$','/','+','%'
};

static char *code_array[TOTAL_CHAR] = {
    "000110100","100100001","001100001","101100000","000110001",
    "100110000","001110000","000100101","100100100","001100100",
    "100001001","001001001","101001000","000011001","100011000",
    "001011000","000001101","100001100","001001100","000011100",
    "100000011","001000011","101000010","000010011","100010010",
    "001010010","000000111","100000110","001000110","000010110",
    "110000001","011000001","111000000","010010001","110010000",
    "011010000","010000101","110000100","011000100","010010100",
    "010101000","010100010","010001010","000101010"
};

static char *reverse_code_array[TOTAL_CHAR] = {
    "001011000","100001001","100001100","000001101","100011000",
    "000011001","000011100","101001000","001001001","001001100",
    "100100001","100100100","000100101","100110000","000110001",
    "000110100","101100000","001100001","001100100","001110000",
    "110000001","110000100","010000101","110010000","010010001",
    "010010100","111000000","011000001","011000100","011010000",
    "100000011","100000110","000000111","100010010","000010011",
    "000010110","101000010","001000011","001000110","001010010",
    "000101010","010001010","010100010","010101000"
};

static inline uint16_t read_ir_adc(void) {
    adc_select_input(IR_SENSOR_ADC_INPUT);
    return adc_read();
}

static char decode_barcode(const char *binary_code) {
    for (int i = 0; i < TOTAL_CHAR; ++i)
        if (strcmp(binary_code, code_array[i]) == 0) return char_array[i];
    for (int i = 0; i < TOTAL_CHAR; ++i)
        if (strcmp(binary_code, reverse_code_array[i]) == 0) return char_array[i];
    return '?';
}

static void scan_and_print_full_barcode(void) {
    char decoded_string[BARCODE_BUFFER_SIZE];
    int char_count = 0;

    printf("Waiting for barcode start (black bar after white pre-element)...\n");

    // Wait for the initial white pre-element to pass — YIELD here
    while (read_ir_adc() > ADC_THRESHOLD) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Now, wait for the first black bar of the first character — YIELD here
    while (read_ir_adc() < ADC_THRESHOLD) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    // Loop to scan multiple characters - continues until end of barcode is detected
    while (char_count < BARCODE_BUFFER_SIZE - 1) {
        uint32_t durations[BARCODE_ELEMENTS];
        char binary_code[BARCODE_ELEMENTS + 1];
        uint32_t min_duration = 0xFFFFFFFF, max_duration = 0;

        printf("\n--- Scanning Character %d ---\n", char_count + 1);

        // The first bar is black, which we already detected.
        // Scan the 9 elements of the current character.
        for (int i = 0; i < BARCODE_ELEMENTS; ++i) {
            uint16_t current_adc = read_ir_adc();
            bool is_black = current_adc > ADC_THRESHOLD;
            const char* color_str = is_black ? "Black" : "White";

            uint32_t start_time = time_us_32();

            // IMPORTANT: keep this inner wait tight for timing accuracy (NO yield)
            while ((read_ir_adc() > ADC_THRESHOLD) == is_black) {
                tight_loop_contents();
            }

            uint32_t end_time = time_us_32();
            durations[i] = end_time - start_time;

            if (durations[i] < min_duration) min_duration = durations[i];
            if (durations[i] > max_duration) max_duration = durations[i];

            printf("Element %d: Duration=%-5lu us, Color=%s\n", i, durations[i], color_str);
        }

        // --- Decode the scanned character ---
        uint32_t width_threshold = min_duration + (max_duration - min_duration) / 2;
        for (int i = 0; i < BARCODE_ELEMENTS; ++i) {
            binary_code[i] = (durations[i] > width_threshold) ? '1' : '0';
        }
        binary_code[BARCODE_ELEMENTS] = '\0';

        char decoded_char = decode_barcode(binary_code);
        printf("Binary: %s -> Decoded: %c\n", binary_code, decoded_char);

        if (decoded_char != '?') {
            decoded_string[char_count++] = decoded_char;
        }

        // Inter-character white gap or end — YIELD here
        printf("Waiting for inter-character white space or end of barcode...\n");
        uint32_t white_start_time = time_us_32();
        while (read_ir_adc() < ADC_THRESHOLD) {
            if (time_us_32() - white_start_time > END_OF_BARCODE_TIMEOUT_MS * 1000) {
                printf("End of barcode detected (timeout on white space).\n");
                goto end_of_scan;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        printf("Next character detected.\n");
    }

end_of_scan:
    decoded_string[char_count] = '\0';

    printf("\n==================================\n");
    if (char_count > 0) {
        printf("Final Decoded String: %s\n", decoded_string);
    } else {
        printf("No barcode was successfully decoded.\n");
    }
    printf("==================================\n\n");
}

// ===============================
// BARCODE TASK (calls your exact scanner)
// ===============================
#ifndef DRIVE_WHILE_SCANNING
#define DRIVE_WHILE_SCANNING 1
#endif
#ifndef TEST_SPEED_CMPS
#define TEST_SPEED_CMPS 1.6f
#endif

static void barcodeTask(void *arg) {
    (void)arg;
    printf("[BC] Using EXACT scanner (GP27/ADC1, thr=%d)\n", ADC_THRESHOLD);

    for (;;) {
#if DRIVE_WHILE_SCANNING
        // Optional: crawl gently while scanning so timing is stable
        forward_motor_pid(TEST_SPEED_CMPS);
#endif
        scan_and_print_full_barcode();   // ← exact logic from barcode_scanner.c
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

// ===============================
// MAIN
// ===============================
int main(void) {
    stdio_init_all();
    sleep_ms(400);

    // Line sensor (digital DO on GP28)
    init_line_digital();

    // Barcode ADC (GP27 / ADC1)
    adc_init();
    adc_gpio_init(IR_SENSOR_ADC_PIN);
    adc_select_input(IR_SENSOR_ADC_INPUT);

    encoder_init();
    motor_init();

    xTaskCreate(lineFollowTask, "LineFollow",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 1, NULL);

    xTaskCreate(barcodeTask, "Barcode",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 2, NULL);

    printf("[MAIN] LF (DO on GP28) + Barcode (ADC1 on GP27) ready.\n");
    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}
