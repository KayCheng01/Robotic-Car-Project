// =============================================
// testdemo2_merged.c
// - Keeps your existing Demo2v4 line-follow from testdemo2.c (GP28 DO)
// - Adds robust Code-39 scanner on GP27/ADC1 (auto-cal + hysteresis + ratio)
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
// LINE-FOLLOW (your existing DO on GP28)
// ===============================
#define LINE_SENSOR_DO_PIN     28  // keep as in your testdemo2.c
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

// Task handle for interrupt notification
static TaskHandle_t lineFollowTaskHandle = NULL;

// GPIO interrupt handler for line sensor
static void line_sensor_isr(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    
    // Notify the line follow task that sensor state changed
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(lineFollowTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

    printf("[LF/DO] Start on GP%d. Speed=%.2f, DO_ACTIVE_ON_BLACK=%d (INTERRUPT MODE)\n",
           LINE_SENSOR_DO_PIN, SLOW_SPEED_CMPS, DO_ACTIVE_ON_BLACK);

    // Enable GPIO interrupt for line sensor (both edges)
    gpio_set_irq_enabled_with_callback(LINE_SENSOR_DO_PIN, 
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &line_sensor_isr);

    uint64_t last_decide = 0;

    for (;;) {
        // Wait for interrupt notification (blocks until sensor change or timeout)
        // Timeout every 100ms to handle cases where we're already off-track
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        const uint64_t now = time_us_64();
        // Debounce: ignore changes too close together
        if (now - last_decide < (uint64_t)DEBOUNCE_DELAY_MS * 1000ULL) {
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

        // Small delay to allow motor commands to take effect
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ======================================================================
// BARCODE SCANNER — robust Code 39 (GP27/ADC1)
// (keeps your tables; swaps in auto-cal + hysteresis + ratio classification)
// ======================================================================

#ifndef IR_SENSOR_ADC_PIN
#define IR_SENSOR_ADC_PIN 27
#endif
#ifndef IR_SENSOR_ADC_INPUT
#define IR_SENSOR_ADC_INPUT 1   // ADC1 is GPIO27
#endif
#ifndef ADC_THRESHOLD
#define ADC_THRESHOLD 2000      // still used in logs; hysteresis handles edges
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

// ---------- Robust helpers: hysteresis + voting, ratio widths ----------
// Using pre-calibrated threshold values (auto-calibration disabled)
static uint16_t g_bc_thr_lo = 1700, g_bc_thr_hi = 2300; // pre-calibrated fixed values
static bool     g_bc_state_black = false;               // last debounced state

static inline bool bc_read_is_black(void) {
    // Single sample with Schmitt trigger (hysteresis) for narrow lines
    adc_select_input(IR_SENSOR_ADC_INPUT);
    uint16_t v = adc_read();
    bool sample_black = g_bc_state_black
                        ? (v > g_bc_thr_lo)    // to leave black, must drop below TH_LO
                        : (v > g_bc_thr_hi);   // to enter black, must rise above TH_HI
    g_bc_state_black = sample_black;
    return sample_black;
}

// ========== Classification Methods ==========

// Method 1: K-means clustering (most robust for varying speeds)
static void classify_widths_kmeans(uint32_t *dur, char *bits9) {
    // Use k-means with k=2 to find narrow and wide clusters
    uint32_t narrow_center = dur[0];
    uint32_t wide_center = dur[0];
    
    // Find initial centers (min and max)
    for (int i = 0; i < 9; i++) {
        if (dur[i] < narrow_center) narrow_center = dur[i];
        if (dur[i] > wide_center) wide_center = dur[i];
    }
    
    // Iterate k-means (3 iterations usually sufficient)
    for (int iter = 0; iter < 3; iter++) {
        uint32_t narrow_sum = 0, wide_sum = 0;
        int narrow_count = 0, wide_count = 0;
        
        for (int i = 0; i < 9; i++) {
            uint32_t dist_narrow = (dur[i] > narrow_center) ? (dur[i] - narrow_center) : (narrow_center - dur[i]);
            uint32_t dist_wide = (dur[i] > wide_center) ? (dur[i] - wide_center) : (wide_center - dur[i]);
            
            if (dist_narrow < dist_wide) {
                narrow_sum += dur[i];
                narrow_count++;
            } else {
                wide_sum += dur[i];
                wide_count++;
            }
        }
        
        if (narrow_count > 0) narrow_center = narrow_sum / narrow_count;
        if (wide_count > 0) wide_center = wide_sum / wide_count;
    }
    
    // Classify using threshold between centers
    uint32_t threshold = (narrow_center + wide_center) / 2;
    for (int i = 0; i < 9; i++)
        bits9[i] = (dur[i] > threshold) ? '1' : '0';
    bits9[9] = '\0';
    
    printf("K-means: narrow_center=%lu, wide_center=%lu, threshold=%lu\n", 
           narrow_center, wide_center, threshold);
}

// Method 2: Simple min/max midpoint (fastest, good for stable speeds)
static void classify_widths_minmax(uint32_t *dur, char *bits9) {
    uint32_t min_dur = 0xFFFFFFFF, max_dur = 0;
    
    for (int i = 0; i < 9; i++) {
        if (dur[i] < min_dur) min_dur = dur[i];
        if (dur[i] > max_dur) max_dur = dur[i];
    }
    
    uint32_t threshold = min_dur + (max_dur - min_dur) / 2;
    for (int i = 0; i < 9; i++)
        bits9[i] = (dur[i] > threshold) ? '1' : '0';
    bits9[9] = '\0';
    
    printf("MinMax: min=%lu, max=%lu, threshold=%lu\n", min_dur, max_dur, threshold);
}

// Method 3: Ratio-based (original method, tunable K)
static void classify_widths_ratio(uint32_t *dur, char *bits9, float K) {
    uint32_t d[9];
    for (int i = 0; i < 9; i++) d[i] = dur[i];
    // insertion sort
    for (int i = 1; i < 9; i++) {
        uint32_t x = d[i]; int j = i - 1;
        while (j >= 0 && d[j] > x) { d[j+1] = d[j]; j--; }
        d[j+1] = x;
    }
    uint32_t narrow_base = (d[0] + d[1] + d[2] + d[3]) / 4;
    uint32_t threshold = (uint32_t)(K * narrow_base);
    
    for (int i = 0; i < 9; i++)
        bits9[i] = (dur[i] > threshold) ? '1' : '0';
    bits9[9] = '\0';
    
    printf("Ratio (K=%.2f): narrow_base=%lu, threshold=%lu\n", K, narrow_base, threshold);
}

// Method 4: Median-based (robust to outliers)
static void classify_widths_median(uint32_t *dur, char *bits9) {
    uint32_t d[9];
    for (int i = 0; i < 9; i++) d[i] = dur[i];
    // insertion sort
    for (int i = 1; i < 9; i++) {
        uint32_t x = d[i]; int j = i - 1;
        while (j >= 0 && d[j] > x) { d[j+1] = d[j]; j--; }
        d[j+1] = x;
    }
    
    // Use median as threshold (5th element when sorted)
    uint32_t threshold = d[4];
    for (int i = 0; i < 9; i++)
        bits9[i] = (dur[i] > threshold) ? '1' : '0';
    bits9[9] = '\0';
    
    printf("Median: threshold=%lu\n", threshold);
}

// Select which method to use (change this to test different methods)
#define CLASSIFICATION_METHOD 1  // 1=K-means, 2=MinMax, 3=Ratio, 4=Median

static void classify_widths(uint32_t *dur, char *bits9) {
#if CLASSIFICATION_METHOD == 1
    classify_widths_kmeans(dur, bits9);
#elif CLASSIFICATION_METHOD == 2
    classify_widths_minmax(dur, bits9);
#elif CLASSIFICATION_METHOD == 3
    classify_widths_ratio(dur, bits9, 1.65f);  // Adjust K value (1.5-2.0)
#elif CLASSIFICATION_METHOD == 4
    classify_widths_median(dur, bits9);
#else
    classify_widths_kmeans(dur, bits9);  // Default to K-means
#endif
}

// ---------- Decoder ----------
static inline uint16_t read_ir_adc(void) {
    adc_select_input(IR_SENSOR_ADC_INPUT);
    return adc_read();
}

static char decode_barcode(const char *binary_code) {
    // First check forward direction
    for (int i = 0; i < TOTAL_CHAR; ++i) {
        if (strcmp(binary_code, code_array[i]) == 0) {
            printf("  -> Matched code_array[%d] = '%s' => Character '%c'\n", i, code_array[i], char_array[i]);
            return char_array[i];
        }
    }
    
    // Then check reverse direction
    for (int i = 0; i < TOTAL_CHAR; ++i) {
        if (strcmp(binary_code, reverse_code_array[i]) == 0) {
            printf("  -> Matched reverse_code_array[%d] = '%s' => Character '%c' (reversed scan)\n", i, reverse_code_array[i], char_array[i]);
            return char_array[i];
        }
    }
    
    // No match found - print what we tried to decode
    printf("  -> NO MATCH FOUND for binary '%s'\n", binary_code);
    printf("  -> Closest matches in code_array:\n");
    for (int i = 0; i < TOTAL_CHAR && i < 5; ++i) {
        int diff_count = 0;
        for (int j = 0; j < 9; j++) {
            if (binary_code[j] != code_array[i][j]) diff_count++;
        }
        if (diff_count <= 2) {
            printf("     [%d] '%c': %s (differs by %d bits)\n", i, char_array[i], code_array[i], diff_count);
        }
    }
    
    return '?';
}

// ---------- Scanner (continuous multi-character scan) ----------
#define MAX_CHARS_TO_SCAN 3  // Number of characters to scan in one go

static void scan_and_print_full_barcode(void) {
    char decoded_string[BARCODE_BUFFER_SIZE];
    int char_count = 0;

    printf("Waiting for barcode start (black bar after white pre-element)...\n");

    // Wait for initial white - check periodically to not hog CPU
    while (bc_read_is_black()) { 
        vTaskDelay(pdMS_TO_TICKS(5));  // Check every 5ms while waiting
    }
    
    // Wait for first black - check periodically
    while (!bc_read_is_black()) { 
        vTaskDelay(pdMS_TO_TICKS(5));  // Check every 5ms while waiting
    }

    printf("\n--- Scanning up to %d characters ---\n", MAX_CHARS_TO_SCAN);

    // Loop to scan multiple characters - same logic as barcode_scanner.c
    while (char_count < MAX_CHARS_TO_SCAN && char_count < BARCODE_BUFFER_SIZE - 1) {
        uint32_t durations[BARCODE_ELEMENTS];
        uint32_t min_duration = 0xFFFFFFFF, max_duration = 0;

        printf("\n--- Scanning Character %d ---\n", char_count + 1);

        // The first bar is black, which we already detected.
        // Scan the 9 elements of the current character.
        for (int i = 0; i < BARCODE_ELEMENTS; ++i) {
            bool is_black = bc_read_is_black();
            const char* color_str = is_black ? "Black" : "White";
            
            uint32_t start_time = time_us_32();
            
            // Wait for the color to change
            while (bc_read_is_black() == is_black) {
                tight_loop_contents();
            }
            
            uint32_t end_time = time_us_32();
            durations[i] = end_time - start_time;

            if (durations[i] < min_duration) min_duration = durations[i];
            if (durations[i] > max_duration) max_duration = durations[i];

            printf("Element %d: Duration=%-5lu us, Color=%s\n", i, durations[i], color_str);
        }

        // --- Decode the scanned character ---
        char bits9[BARCODE_ELEMENTS + 1];
        classify_widths(durations, bits9);

        char decoded_char = decode_barcode(bits9);
        printf("Binary: %s -> Decoded: %c\n", bits9, decoded_char);

        if (decoded_char != '?') {
            decoded_string[char_count++] = decoded_char;
        }

        // --- Check for end of barcode or next character ---
        // After scanning 9 elements, we need to find the inter-character gap (white space)
        printf("Waiting for inter-character white space or end of barcode...\n");
        
        // First, ensure we're on a white space (skip any remaining black)
        bool currently_black = bc_read_is_black();
        if (currently_black) {
            printf("Still on black, waiting for white...\n");
            while (bc_read_is_black()) {
                tight_loop_contents();
            }
        }
        
        // Now we're on white space - wait for next black (next char) or timeout (end of barcode)
        uint32_t white_start_time = time_us_32();
        while (!bc_read_is_black()) {
            if (time_us_32() - white_start_time > END_OF_BARCODE_TIMEOUT_MS * 1000) {
                printf("End of barcode detected (timeout on white space).\n");
                goto end_of_scan; // End of barcode detected
            }
            tight_loop_contents();
        }
        printf("Next character detected (black bar found).\n");
    }

end_of_scan:
    decoded_string[char_count] = '\0';
    printf("\n==================================\n");
    printf("BARCODE SCAN COMPLETE\n");
    printf("==================================\n");
    if (char_count > 0) {
        printf("Characters decoded: %d\n", char_count);
        printf("Final Decoded String: '%s'\n", decoded_string);
        printf("\nDetailed breakdown:\n");
        for (int i = 0; i < char_count; i++) {
            // Find the character in char_array
            for (int j = 0; j < TOTAL_CHAR; j++) {
                if (decoded_string[i] == char_array[j]) {
                    printf("  [%d] '%c' -> code_array[%d] = %s\n", 
                           i, decoded_string[i], j, code_array[j]);
                    break;
                }
            }
        }
    } else {
        printf("No barcode was successfully decoded.\n");
    }
    printf("==================================\n\n");
}

// ===============================
// BARCODE TASK
// ===============================
#ifndef DRIVE_WHILE_SCANNING
#define DRIVE_WHILE_SCANNING 0  // Set to 0 - let line following control motors
#endif
#ifndef TEST_SPEED_CMPS
#define TEST_SPEED_CMPS 1.6f
#endif

static void barcodeTask(void *arg) {
    (void)arg;
    printf("[BC] Robust scanner (GP27/ADC1). Using pre-calibrated thresholds: TH_LO=%u TH_HI=%u\n", 
           g_bc_thr_lo, g_bc_thr_hi);

    // Let motors/PID settle before starting scan
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (;;) {
#if DRIVE_WHILE_SCANNING
        // Gentle crawl while scanning
        forward_motor_pid(TEST_SPEED_CMPS);
#endif
        scan_and_print_full_barcode();
        
        // Long delay between scans to let line following task run
        printf("[BC] Scan complete. Waiting 3 seconds before next scan...\n");
        vTaskDelay(pdMS_TO_TICKS(3000));  // Wait 3 seconds between scans
    }
}

// ===============================
// MAIN
// ===============================
int main(void) {
    stdio_init_all();
    sleep_ms(6000);

    // Line sensor (digital DO on GP28)
    init_line_digital();

    // Barcode ADC (GP27 / ADC1)
    adc_init();
    adc_gpio_init(IR_SENSOR_ADC_PIN);
    adc_select_input(IR_SENSOR_ADC_INPUT);

    encoder_init();
    motor_init();

    // Create line follow task and store its handle for ISR notification
    xTaskCreate(lineFollowTask, "LineFollow",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 2, &lineFollowTaskHandle);  // Higher priority for line following

    xTaskCreate(barcodeTask, "Barcode",
                configMINIMAL_STACK_SIZE * 4, NULL,
                tskIDLE_PRIORITY + 1, NULL);  // Lower priority for barcode

    printf("[MAIN] LF (Interrupt on GP28) + Barcode (Polling ADC1 on GP27) ready.\n");
    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}
