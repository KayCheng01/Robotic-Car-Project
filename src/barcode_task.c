#include "barcode_task.h"
#include "adc_barcode.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "semphr.h"

#define BARCODE_ELEMENTS 9
#define BARCODE_BUFFER_SIZE 100
#define TOTAL_CHAR 44
#define INTER_CHAR_TIMEOUT_MS 300  // 300ms timeout for end of barcode
#define MAX_ELEMENT_DURATION_US 500000  // 500ms max per element (for slow car speed)

// Code 39 lookup tables (from original barcode_scanner.c)
static const char char_array[TOTAL_CHAR] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z',
    '-', '.', ' ', '*', '$', '/', '+', '%'
};

static const char *code_array[TOTAL_CHAR] = {
    "000110100", "100100001", "001100001", "101100000", "000110001",
    "100110000", "001110000", "000100101", "100100100", "001100100",
    "100001001", "001001001", "101001000", "000011001", "100011000",
    "001011000", "000001101", "100001100", "001001100", "000011100",
    "100000011", "001000011", "101000010", "000010011", "100010010",
    "001010010", "000000111", "100000110", "001000110", "000010110",
    "110000001", "011000001", "111000000", "010010001", "110010000",
    "011010000", "010000101", "110000100", "011000100", "010010100",
    "010101000", "010100010", "010001010", "000101010"
};

static const char *reverse_code_array[TOTAL_CHAR] = {
    "001011000", "100001001", "100001100", "000001101", "100011000",
    "000011001", "000011100", "101001000", "001001001", "001001100",
    "100100001", "100100100", "000100101", "100110000", "000110001",
    "000110100", "101100000", "001100001", "001100100", "001110000",
    "110000001", "110000100", "010000101", "110010000", "010010001",
    "010010100", "111000000", "011000001", "011000100", "011010000",
    "100000011", "100000110", "000000111", "100010010", "000010011",
    "000010110", "101000010", "001000011", "001000110", "001010010",
    "000101010", "010001010", "010100010", "010101000"
};

// Shared result buffer (protected by mutex)
static char decoded_result[BARCODE_BUFFER_SIZE];
static bool new_result_available = false;
static bool scan_complete = false;
static SemaphoreHandle_t result_mutex = NULL;

// Summary tracking
typedef struct {
    char binary_code[BARCODE_ELEMENTS + 1];
    char decoded_char;
    uint32_t min_duration;
    uint32_t max_duration;
    uint32_t threshold;
} CharacterSummary;

static CharacterSummary char_summaries[BARCODE_BUFFER_SIZE];
static int summary_count = 0;

// Decode a 9-bit binary string to character
static char decode_barcode(const char *binary_code) {
    for (int i = 0; i < TOTAL_CHAR; ++i) {
        if (strcmp(binary_code, code_array[i]) == 0) {
            return char_array[i];
        }
    }
    for (int i = 0; i < TOTAL_CHAR; ++i) {
        if (strcmp(binary_code, reverse_code_array[i]) == 0) {
            return char_array[i];
        }
    }
    return '?';
}

// Barcode decoder task
static void barcode_task_func(void *pvParameters) {
    (void)pvParameters;
    
    QueueHandle_t edge_queue = adc_barcode_get_queue();
    barcode_edge_t edge;
    
    char decoded_string[BARCODE_BUFFER_SIZE];
    int char_count = 0;
    
    // State machine for multi-character decoding
    enum {
        WAITING_PRE_WHITE,      // Wait for white before barcode
        WAITING_FIRST_BLACK,    // Wait for first black bar
        SCANNING_CHARACTER      // Collecting 9 elements (alternating black/white)
    } state = WAITING_PRE_WHITE;
    
    uint32_t durations[BARCODE_ELEMENTS];
    int element_idx = 0;
    uint32_t last_edge_time = 0;
    uint32_t last_white_time = 0;  // Track when we entered white space after characters
    
    printf("[BC] Task started, waiting for edges...\n");
    
    adc_barcode_start();
    printf("[BC] ADC+DMA started\n");
    
    for (;;) {
        // Wait for edge event with timeout
        if (xQueueReceive(edge_queue, &edge, pdMS_TO_TICKS(100)) != pdTRUE) {
            // Check if we're done scanning (3 chars complete + white timeout)
            if (char_count >= 3 && last_white_time > 0) {
                uint32_t now_us = (uint32_t)time_us_64();
                uint32_t white_duration = now_us - last_white_time;
                
                if (white_duration > (INTER_CHAR_TIMEOUT_MS * 1000)) {
                    // End of barcode - print summary
                    decoded_string[char_count] = '\0';
                    
                    // Check if this is a valid Code 39 barcode (should be 3 chars: *X*)
                    bool is_valid_code39 = (summary_count == 3 && 
                                           char_summaries[0].decoded_char == '*' &&
                                           char_summaries[2].decoded_char == '*');
                    
                    // Print detailed summary
                    printf("\n");
                    printf("╔════════════════════════════════════════════════╗\n");
                    printf("║         BARCODE SCAN COMPLETE                  ║\n");
                    printf("╚════════════════════════════════════════════════╝\n\n");
                    
                    printf("Characters Detected: %d\n", summary_count);
                    printf("Format: %s\n\n", is_valid_code39 ? "✓ Valid Code 39" : "✗ Invalid format");
                    
                    for (int i = 0; i < summary_count; i++) {
                        printf("──────────────────────────────────────────────\n");
                        printf("Character %d:\n", i + 1);
                        printf("  Binary Code:  %s\n", char_summaries[i].binary_code);
                        printf("  Decoded Char: '%c'\n", char_summaries[i].decoded_char);
                        printf("  Duration Range: %lu - %lu us\n", 
                               char_summaries[i].min_duration, 
                               char_summaries[i].max_duration);
                        printf("  Width Threshold: %lu us\n", char_summaries[i].threshold);
                    }
                    printf("──────────────────────────────────────────────\n\n");
                    
                    if (is_valid_code39) {
                        printf("✓ DECODED RESULT: '%c'\n", char_summaries[1].decoded_char);
                    } else {
                        printf("Raw String: [%s]\n", decoded_string);
                    }
                    printf("\n╔════════════════════════════════════════════════╗\n");
                    printf("║         READY TO STOP CAR                      ║\n");
                    printf("╚════════════════════════════════════════════════╝\n\n");
                    
                    // Store result only if valid Code 39 (3 chars)
                    if (summary_count == 3) {
                        if (xSemaphoreTake(result_mutex, portMAX_DELAY) == pdTRUE) {
                            strncpy(decoded_result, decoded_string, BARCODE_BUFFER_SIZE);
                            decoded_result[BARCODE_BUFFER_SIZE - 1] = '\0';
                            new_result_available = true;
                            scan_complete = true;
                            xSemaphoreGive(result_mutex);
                        }
                    }
                    
                    // Reset for next scan
                    char_count = 0;
                    summary_count = 0;
                    last_white_time = 0;
                    state = WAITING_PRE_WHITE;
                }
            }
            continue;
        }
        
        // Edge received - print during active scanning
        if (state == SCANNING_CHARACTER && last_edge_time > 0) {
            uint32_t duration = edge.timestamp_us - last_edge_time;
            printf("[BC] Edge: elem=%d/%d, duration=%lu us (%.1f ms), is_black=%d\n", 
                   element_idx, BARCODE_ELEMENTS, 
                   duration, duration / 1000.0f, edge.is_black);
        }
        
        // If we see white after completing characters, start timeout timer
        if (char_count > 0 && !edge.is_black && state != SCANNING_CHARACTER) {
            if (last_white_time == 0) {
                last_white_time = edge.timestamp_us;
                printf("[BC] White detected after %d chars, starting end-of-barcode timer\n", char_count);
            }
        }
        
        // If we see black after white, reset the white timer (not end yet)
        if (edge.is_black && last_white_time > 0 && state != SCANNING_CHARACTER) {
            last_white_time = 0;
            printf("[BC] Black detected, continuing scan\n");
        }
        
        // Process edge based on state
        switch (state) {
            case WAITING_PRE_WHITE:
                // Looking for white space before barcode
                if (!edge.is_black) {
                    state = WAITING_FIRST_BLACK;
                    last_edge_time = edge.timestamp_us;
                    printf("[BC] Pre-white detected\n");
                }
                break;
                
            case WAITING_FIRST_BLACK:
                // Looking for first black bar to start scanning
                if (edge.is_black) {
                    printf("\n[BC] ═══ BARCODE SCAN STARTED ═══\n");
                    printf("[BC] First element MUST be black\n");
                    element_idx = 0;
                    last_edge_time = edge.timestamp_us;
                    state = SCANNING_CHARACTER;
                }
                break;
                
            case SCANNING_CHARACTER: {
                // Measure duration of element that just ended (before this edge)
                uint32_t duration = edge.timestamp_us - last_edge_time;
                
                // Sanity check duration
                if (duration > MAX_ELEMENT_DURATION_US) {
                    printf("[BC] Element %d timeout (%.1f ms > %.1f ms max), resetting\n\n", 
                           element_idx, duration / 1000.0f, MAX_ELEMENT_DURATION_US / 1000.0f);
                    state = WAITING_PRE_WHITE;
                    break;
                }
                
                // Store the duration
                durations[element_idx] = duration;
                printf("[BC] Stored element %d: %.1f ms\n", element_idx, duration / 1000.0f);
                element_idx++;
                last_edge_time = edge.timestamp_us;
                
                // Check if we have all 9 elements
                if (element_idx == BARCODE_ELEMENTS) {
                    printf("[BC] Got all 9 elements, decoding...\n");
                    
                    // Decode character
                    uint32_t min_dur = 0xFFFFFFFF, max_dur = 0;
                    for (int i = 0; i < BARCODE_ELEMENTS; i++) {
                        if (durations[i] < min_dur) min_dur = durations[i];
                        if (durations[i] > max_dur) max_dur = durations[i];
                    }
                    
                    uint32_t threshold = min_dur + (max_dur - min_dur) / 2;
                    printf("[BC] Duration range: %.1f - %.1f ms, threshold: %.1f ms\n",
                           min_dur / 1000.0f, max_dur / 1000.0f, threshold / 1000.0f);
                    char binary_code[BARCODE_ELEMENTS + 1];
                    for (int i = 0; i < BARCODE_ELEMENTS; i++) {
                        binary_code[i] = (durations[i] > threshold) ? '1' : '0';
                    }
                    binary_code[BARCODE_ELEMENTS] = '\0';
                    
                    char decoded_char = decode_barcode(binary_code);
                    
                    // Store summary for this character
                    if (summary_count < BARCODE_BUFFER_SIZE) {
                        strncpy(char_summaries[summary_count].binary_code, binary_code, BARCODE_ELEMENTS + 1);
                        char_summaries[summary_count].decoded_char = decoded_char;
                        char_summaries[summary_count].min_duration = min_dur;
                        char_summaries[summary_count].max_duration = max_dur;
                        char_summaries[summary_count].threshold = threshold;
                        summary_count++;
                    }
                    
                    printf("[BC] Char %d: '%c'\n", char_count + 1, decoded_char);
                    
                    if (decoded_char != '?' && char_count < BARCODE_BUFFER_SIZE - 1) {
                        decoded_string[char_count++] = decoded_char;
                    }
                    
                    // Continue scanning next character immediately
                    // Next edge will be either:
                    // - White space (inter-character gap)
                    // - Black bar (next character starts)
                    element_idx = 0;
                    
                    // If we have 3 chars and see white, we'll timeout and print summary
                    if (char_count >= 3) {
                        printf("[BC] Got 3 characters, waiting for white timeout to finalize\n");
                    }
                }
                
                last_edge_time = edge.timestamp_us;
                break;
            }
        }
    }
}

void barcode_task_create(void) {
    result_mutex = xSemaphoreCreateMutex();
    configASSERT(result_mutex != NULL);
    
    xTaskCreate(barcode_task_func, "Barcode",
                configMINIMAL_STACK_SIZE * 4,
                NULL, tskIDLE_PRIORITY + 2, NULL);
}

bool barcode_get_result(char *buf, size_t buf_size) {
    bool result = false;
    if (xSemaphoreTake(result_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (new_result_available) {
            strncpy(buf, decoded_result, buf_size);
            buf[buf_size - 1] = '\0';
            new_result_available = false;
            result = true;
        }
        xSemaphoreGive(result_mutex);
    }
    return result;
}

void barcode_reset(void) {
    if (xSemaphoreTake(result_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        scan_complete = false;
        new_result_available = false;
        summary_count = 0;
        xSemaphoreGive(result_mutex);
    }
}

bool barcode_scan_complete(void) {
    bool complete = false;
    if (xSemaphoreTake(result_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        complete = scan_complete;
        xSemaphoreGive(result_mutex);
    }
    return complete;
}
