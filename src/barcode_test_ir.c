// ==============================
// barcode_test_ir.c — IR Sensor Barcode Testing & Characterization
// ==============================
// - Tests IR sensor on GP27 for barcode detection
// - Captures and displays raw timing data for bars
// - Analyzes bar width distribution (narrow vs wide)
// - Reads ADC intensity levels (black/white)
// - Helps calibrate thresholds
// ==============================

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "motor.h"
#include "encoder.h"

// ================== CONFIG ==================

// IR barcode sensor pin (GPIO 27 ONLY - focused on barcode black/white detection)
#define BARCODE_SENSOR_PIN 27

// ADC for intensity reading (GPIO 26 = ADC0)
#define BARCODE_ADC_PIN 26
#define BARCODE_ADC_CHANNEL 0

// Debounce for barcode scanning (microseconds)
// Prevents noise from being detected as real edges
#define BARCODE_DEBOUNCE_US 5000

// Maximum bars to capture per scan
#define MAX_BARS_TO_CAPTURE 20

// HARDWARE MAP:
// - GPIO 27: Barcode IR sensor (digital edge detection)
// - GPIO 26: ADC0 (intensity reading for black/white detection)
// - GPIO 28: RESERVED for line-following sensor in demo2.c (ADC2)
// This test focuses ONLY on GPIO 27 and uses GPIO 26 ADC (NOT GPIO 28)

// ================== Global Variables ==================

volatile bool barcode_scan_started = false;
volatile uint64_t last_transition_time = 0;
volatile uint64_t current_element_start_time = 0;

// Timing and color data storage
typedef struct {
    uint64_t duration_us;   // Time duration of this element
    bool is_black;          // true = black bar, false = white space
    uint16_t adc_value;     // ADC reading when captured
} BarElement;

#define NUM_ELEMENTS 9  // 5 black bars + 4 white spaces
BarElement captured_elements[NUM_ELEMENTS];
uint16_t num_elements_captured = 0;
bool scan_complete = false;
bool is_currently_black = false;

// Two-threshold system with dead zone
// BLACK (high): ADC > 2500
// WHITE (low):  ADC < 2000
// DEAD ZONE:    2000 <= ADC <= 2500 (discard/ignore)
#define WHITE_THRESHOLD_MAX 1000  // Values below this are WHITE
#define BLACK_THRESHOLD_MIN 1500  // Values above this are BLACK

SemaphoreHandle_t scan_semaphore;

// Current detected state (ignoring dead zone values)
typedef enum {
    STATE_UNKNOWN,  // In dead zone or not yet determined
    STATE_WHITE,    // Confirmed white (ADC < 1500)
    STATE_BLACK     // Confirmed black (ADC > 2500)
} BarState;

// ================== ADC Helpers ==================

static uint16_t read_barcode_adc(void) {
    adc_select_input(BARCODE_ADC_CHANNEL);
    return adc_read();
}

static void init_barcode_adc(void) {
    adc_init();
    adc_gpio_init(BARCODE_ADC_PIN);
    adc_select_input(BARCODE_ADC_CHANNEL);
}

// ================== Timing Analysis ==================

static void print_element_timings(void) {
    if (num_elements_captured == 0) {
        printf("\n[ERROR] No elements captured!\n");
        return;
    }

    printf("\n========== BARCODE TIMING DATA (9 ELEMENTS) ==========\n");
    printf("Element # | Type  | Duration (µs) | ADC Value | Width\n");
    printf("----------|-------|---------------|-----------|--------\n");

    uint64_t min_time = captured_elements[0].duration_us;
    uint64_t max_time = captured_elements[0].duration_us;

    // Find min and max timing
    for (uint16_t i = 0; i < num_elements_captured; i++) {
        if (captured_elements[i].duration_us < min_time) min_time = captured_elements[i].duration_us;
        if (captured_elements[i].duration_us > max_time) max_time = captured_elements[i].duration_us;
    }

    // Calculate threshold between narrow and wide (roughly)
    uint64_t threshold = (min_time + max_time) / 2;

    // Print each element
    for (uint16_t i = 0; i < num_elements_captured; i++) {
        const char *color = captured_elements[i].is_black ? "BLACK" : "WHITE";
        const char *width = captured_elements[i].duration_us > threshold ? "WIDE" : "NARROW";
        printf("%6d    | %5s | %13llu | %9d | %s\n",
               i,
               color,
               captured_elements[i].duration_us,
               captured_elements[i].adc_value,
               width);
    }

    printf("\n========== STATISTICS ==========\n");
    printf("Total elements: %d (should be 9 for Code39)\n", num_elements_captured);
    printf("Min timing: %llu µs\n", min_time);
    printf("Max timing: %llu µs\n", max_time);
    printf("Estimated threshold: %llu µs\n", threshold);
    printf("  NARROW <= %llu µs\n", threshold);
    printf("  WIDE   >  %llu µs\n", threshold);

    // Generate binary representation (0=narrow, 1=wide)
    printf("\nBinary pattern (0=narrow, 1=wide):\n");
    for (uint16_t i = 0; i < num_elements_captured; i++) {
        printf("%c", captured_elements[i].duration_us > threshold ? '1' : '0');
    }
    printf("\n");

    // Count wide vs narrow
    uint16_t wide_count = 0;
    uint16_t narrow_count = 0;
    for (uint16_t i = 0; i < num_elements_captured; i++) {
        if (captured_elements[i].duration_us > threshold) wide_count++;
        else narrow_count++;
    }
    printf("\nWide elements: %d, Narrow elements: %d\n", wide_count, narrow_count);
    printf("Expected for Code39: 3 wide + 6 narrow\n");
}

static void analyze_intensity_levels(void) {
    if (num_elements_captured < 2) {
        printf("[ERROR] Need at least 2 elements for analysis\n");
        return;
    }

    printf("\n========== BLACK/WHITE INTENSITY ANALYSIS ==========\n");
    
    uint16_t black_min = 4095, black_max = 0;
    uint16_t white_min = 4095, white_max = 0;
    uint32_t black_sum = 0, white_sum = 0;
    uint16_t black_count = 0, white_count = 0;

    // Separate black and white statistics
    for (uint16_t i = 0; i < num_elements_captured; i++) {
        if (captured_elements[i].is_black) {
            // Black bar statistics
            if (captured_elements[i].adc_value < black_min) black_min = captured_elements[i].adc_value;
            if (captured_elements[i].adc_value > black_max) black_max = captured_elements[i].adc_value;
            black_sum += captured_elements[i].adc_value;
            black_count++;
        } else {
            // White space statistics
            if (captured_elements[i].adc_value < white_min) white_min = captured_elements[i].adc_value;
            if (captured_elements[i].adc_value > white_max) white_max = captured_elements[i].adc_value;
            white_sum += captured_elements[i].adc_value;
            white_count++;
        }
    }

    uint16_t black_avg = black_count > 0 ? black_sum / black_count : 0;
    uint16_t white_avg = white_count > 0 ? white_sum / white_count : 0;

    printf("BLACK bars: Min=%d, Max=%d, Avg=%d, Count=%d\n", 
           black_min, black_max, black_avg, black_count);
    printf("WHITE bars: Min=%d, Max=%d, Avg=%d, Count=%d\n", 
           white_min, white_max, white_avg, white_count);

    // Suggest threshold for black/white detection
    uint16_t suggested_threshold = (black_avg + white_avg) / 2;
    printf("\n========== RECOMMENDED THRESHOLD ==========\n");
    printf("Threshold Value: %d\n", suggested_threshold);
    printf("  - ADC >= %d: BLACK bar\n", suggested_threshold);
    printf("  - ADC <  %d: WHITE space\n", suggested_threshold);
    
    printf("\nUpdate code with:\n");
    printf("  #define BLACK_THRESHOLD %d\n", suggested_threshold);
}

// ================== Continuous ADC Monitoring Task ==================

/**
 * Task that continuously monitors ADC to detect black bar start
 * and times each element (black/white) duration
 */
static void barcode_timing_task(void *pvParameters) {
    printf("[TASK] Barcode timing monitor started\n");
    printf("[INFO] WHITE: ADC < %d | DEAD ZONE: %d-%d | BLACK: ADC > %d\n",
           WHITE_THRESHOLD_MAX, WHITE_THRESHOLD_MAX, BLACK_THRESHOLD_MIN, BLACK_THRESHOLD_MIN);
    
    const TickType_t sample_delay = pdMS_TO_TICKS(1); // 1ms sampling rate
    BarState last_valid_state = STATE_UNKNOWN;
    
    for (;;) {
        uint16_t adc_val = read_barcode_adc();
        uint64_t now = time_us_64();
        
        // Determine current state using two-threshold system
        BarState current_state;
        if (adc_val > BLACK_THRESHOLD_MIN) {
            current_state = STATE_BLACK;  // BLACK (high)
        } else if (adc_val < WHITE_THRESHOLD_MAX) {
            current_state = STATE_WHITE;  // WHITE (low)
        } else {
            current_state = STATE_UNKNOWN; // DEAD ZONE - discard
        }
        
        // Ignore readings in dead zone
        if (current_state == STATE_UNKNOWN) {
            vTaskDelay(sample_delay);
            continue;
        }
        
        if (!barcode_scan_started) {
            // Wait for black bar to start scan (RISING EDGE - transition to HIGH)
            if (current_state == STATE_BLACK) {
                printf("\n[SCAN START] RISING EDGE - Black bar detected! ADC=%d (>%d)\n", 
                       adc_val, BLACK_THRESHOLD_MIN);
                printf("[TIMING] Started timing BLACK element...\n");
                barcode_scan_started = true;
                is_currently_black = true;      // We are now in BLACK state
                current_element_start_time = now; // Start timing this BLACK element
                num_elements_captured = 0;
                scan_complete = false;
                last_valid_state = STATE_BLACK;
            }
        } else {
            // Scanning in progress - detect transitions (only if state changed)
            bool state_changed = false;
            
            if (current_state == STATE_BLACK && last_valid_state == STATE_WHITE) {
                state_changed = true;  // RISING EDGE (WHITE → BLACK)
            } else if (current_state == STATE_WHITE && last_valid_state == STATE_BLACK) {
                state_changed = true;  // FALLING EDGE (BLACK → WHITE)
            }
            
            if (state_changed) {
                // *** TRANSITION DETECTED ***
                // Calculate duration of the COMPLETED element (the one that just ended)
                uint64_t duration = now - current_element_start_time;
                
                // Store the COMPLETED element (the one we were just timing)
                if (num_elements_captured < NUM_ELEMENTS) {
                    captured_elements[num_elements_captured].duration_us = duration;
                    captured_elements[num_elements_captured].is_black = is_currently_black; // The state we WERE in
                    captured_elements[num_elements_captured].adc_value = adc_val;
                    
                    const char *color = is_currently_black ? "BLACK" : "WHITE";
                    const char *edge = (current_state == STATE_BLACK) ? "RISING EDGE (→BLACK)" : "FALLING EDGE (→WHITE)";
                    printf("[Element %d] %s finished: %llu µs | ADC: %d | %s\n",
                           num_elements_captured, color, duration, adc_val, edge);
                    
                    num_elements_captured++;
                    
                    // Check if we've captured all 9 elements
                    if (num_elements_captured >= NUM_ELEMENTS) {
                        printf("\n[SCAN COMPLETE] Captured all %d elements!\n", NUM_ELEMENTS);
                        barcode_scan_started = false;  // Stop scanning
                        scan_complete = true;
                        last_valid_state = STATE_UNKNOWN;  // Reset state
                        
                        // Signal analysis task
                        xSemaphoreGive(scan_semaphore);
                        
                        printf("[RESET] Waiting for next black bar to start new scan...\n");
                        // Don't update timing or state after completion
                        vTaskDelay(sample_delay);
                        continue;  // Go back to waiting for black bar
                    }
                }
                
                // Update state for next element (only if scan still in progress)
                is_currently_black = (current_state == STATE_BLACK);
                current_element_start_time = now;
            }
            
            last_valid_state = current_state;
        }
        
        vTaskDelay(sample_delay);
    }
}

// ================== GPIO Setup ==================
// Note: We don't use interrupts anymore - using continuous ADC polling instead

// ================== FreeRTOS Tasks ==================

/**
 * Task to handle scan completion and display analysis
 */
static void barcode_analysis_task(void *pvParameters) {
    printf("[TASK] Barcode analysis task started\n");
    printf("[INFO] Ready for barcode scan. Move barcode at FIXED SPEED over IR sensor...\n\n");

    for (;;) {
        // Wait for scan completion
        if (xSemaphoreTake(scan_semaphore, portMAX_DELAY)) {
            if (scan_complete) {
                vTaskDelay(pdMS_TO_TICKS(100)); // Small delay
                
                // Display analysis
                print_element_timings();
                analyze_intensity_levels();

                scan_complete = false;
                num_elements_captured = 0;
                printf("\n[READY] Ready for next scan...\n\n");
            }
        }
    }
}

/**
 * Optional: Status monitor task
 */
static void barcode_monitor_task(void *pvParameters) {
    for (;;) {
        if (!barcode_scan_started) {
            uint16_t adc = read_barcode_adc();
            const char *state;
            if (adc > BLACK_THRESHOLD_MIN) {
                state = "BLACK (HIGH)";
            } else if (adc < WHITE_THRESHOLD_MAX) {
                state = "WHITE (LOW)";
            } else {
                state = "DEAD ZONE";
            }
            printf("[MONITOR] ADC=%4d | %s | Waiting for black bar (>%d)...\n",
                   adc, state, BLACK_THRESHOLD_MIN);
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            vTaskDelay(pdMS_TO_TICKS(100)); // Don't print during scan
        }
    }
}

// ================== Main ==================

int main(void) {
    stdio_init_all();
    sleep_ms(500);

    printf("\n");
    printf("╔══════════════════════════════════════════════════════════════╗\n");
    printf("║          GPIO 27 Barcode Black/White Detection Test           ║\n");
    printf("║                                                              ║\n");
    printf("║  Primary Sensor:                                             ║\n");
    printf("║    - GPIO 27: Barcode IR sensor                             ║\n");
    printf("║    - Uses: Digital edge detection + ADC1 (analog intensity) ║\n");
    printf("║                                                              ║\n");
    printf("║  Reserved (NOT used by this test):                           ║\n");
    printf("║    - GPIO 28: Line-following sensor (demo2.c) uses ADC2    ║\n");
    printf("║                                                              ║\n");
    printf("║  Captures:                                                   ║\n");
    printf("║    - Bar widths (narrow vs wide)                            ║\n");
    printf("║    - ADC1 intensity levels (black/white differentiation)    ║\n");
    printf("║    - Auto-generates black/white detection threshold         ║\n");
    printf("╚══════════════════════════════════════════════════════════════╝\n\n");

    // Initialize ADC only (no GPIO interrupts needed)
    init_barcode_adc();

    // Create semaphore
    scan_semaphore = xSemaphoreCreateBinary();
    if (scan_semaphore == NULL) {
        printf("[ERROR] Failed to create semaphore\n");
        return -1;
    }

    printf("[INIT] All systems initialized\n");
    printf("[INFO] WHITE (low):  ADC < %d\n", WHITE_THRESHOLD_MAX);
    printf("[INFO] DEAD ZONE:    ADC %d - %d (discarded)\n", WHITE_THRESHOLD_MAX, BLACK_THRESHOLD_MIN);
    printf("[INFO] BLACK (high): ADC > %d\n", BLACK_THRESHOLD_MIN);
    printf("[INFO] Move your barcode at FIXED SPEED over the sensor\n");
    printf("[INFO] Scan starts automatically when black bar detected\n");
    printf("[INFO] Will capture 9 elements: 5 black bars + 4 white spaces\n\n");

    // Create FreeRTOS tasks
    xTaskCreate(barcode_timing_task, "BarcodeTiming", 
                configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 3, NULL);
    
    xTaskCreate(barcode_analysis_task, "BarcodeAnalysis", 
                configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL);
    
    xTaskCreate(barcode_monitor_task, "BarcodeMonitor", 
                configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);

    vTaskStartScheduler();

    // Should never reach here
    while (true) {
        tight_loop_contents();
    }

    return 0;
}
