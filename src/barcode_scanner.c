/**
 * @file barcode_scanner.c
 * @brief Barcode 39 scanning and decoding logic.
 *
 * This file implements the logic to scan a Code 39 barcode using an IR sensor
 * connected to an ADC pin. It is designed to be resistant to variations in speed
 * by dynamically determining the threshold between narrow and wide bars.
 *
 * The process is as follows:
 * 1. Wait for a preliminary white space (low ADC reading).
 * 2. Once the barcode starts (transition to black), measure the duration of 9 consecutive
 *    black and white elements.
 * 3. Analyze the 9 collected durations to classify them as 'narrow' or 'wide'.
 * 4. Convert the sequence of narrow/wide elements into a 9-bit binary string (0 for narrow, 1 for wide).
 * 5. Compare the binary string against a lookup table for Code 39 to find the corresponding character.
 * 6. The decoded character is returned.
 *
 * Detailed debugging information is printed to the console during the scan.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

// Define the GPIO pin for the IR sensor's ADC input
#define IR_SENSOR_ADC_PIN 27
#define IR_SENSOR_ADC_INPUT 1 // ADC1 is on GPIO27

// ADC threshold to distinguish between black (high ADC) and white (low ADC)
#define ADC_THRESHOLD 2000 // This value may need tuning based on sensor calibration

#define TOTAL_CHAR 44
#define BARCODE_ELEMENTS 9

// Hardcoded Code 39 character set
char char_array[TOTAL_CHAR] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G',
                               'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X',
                               'Y', 'Z', '_', '.', '$', '/', '+', '%', ' '};

// Binary representations for Code 39 characters (0=narrow, 1=wide)
char *code_array[TOTAL_CHAR] = {"000110100", "100100001", "001100001", "101100000", "000110001", "100110000", "001110000",
                                "000100101", "100100100", "001100100", "100001001", "001001001", "101001000", "000011001",
                                "100011000", "001011000", "000001101", "100001100", "001001100", "000011100", "100000011",
                                "001000011", "101000010", "000010011", "100010010", "001010010", "000000111", "100000110",
                                "001000110", "000010110", "110000001", "011000001", "111000000", "010010001", "110010000",
                                "011010000", "010000101", "110000100", "010101000", "010100010", "010001010", "000101010",
                                "011000100"};

// Reversed binary representations for scanning in the opposite direction
char *reverse_code_array[TOTAL_CHAR] = {"001011000", "100001001", "100001100", "000001101", "100011000", "000011001",
                                        "000011100", "101001000", "001001001", "001001100", "100100001", "100100100",
                                        "000100101", "100110000", "000110001", "000110100", "101100000", "001100001",
                                        "001100100", "001110000", "110000001", "110000100", "010000101", "110010000",
                                        "010010001", "010010100", "111000000", "011000001", "011000100", "011010000",
                                        "100000011", "100000110", "000000111", "100010010", "000010011", "000010110",
                                        "101000010", "001000011", "000101010", "010001010", "010100010", "010101000",
                                        "001000110"};

/**
 * @brief Reads the current ADC value from the IR sensor.
 * @return The 12-bit ADC value.
 */
uint16_t read_ir_adc() {
    adc_select_input(IR_SENSOR_ADC_INPUT);
    return adc_read();
}

/**
 * @brief Decodes a 9-bit binary string into a character using the Code 39 tables.
 * @param binary_code The 9-bit string representing the barcode.
 * @return The decoded character, or '?' if not found.
 */
char decode_barcode(const char *binary_code) {
    // First, check for a direct match in the forward direction
    for (int i = 0; i < TOTAL_CHAR; ++i) {
        if (strcmp(binary_code, code_array[i]) == 0) {
            return char_array[i];
        }
    }
    // If no direct match, check for a reversed match
    for (int i = 0; i < TOTAL_CHAR; ++i) {
        if (strcmp(binary_code, reverse_code_array[i]) == 0) {
            // Optional: could return a special indicator for reversed scan
            return char_array[i];
        }
    }
    return '?'; // Return '?' if no match is found
}

/**
 * @brief Scans and decodes a single Code 39 character.
 *
 * This function waits for the start of a barcode, measures the width of the 9
 * elements, classifies them as narrow or wide, and returns the decoded character.
 *
 * @return The decoded character, or '?' on failure.
 */
char scan_barcode_char() {
    uint32_t durations[BARCODE_ELEMENTS];
    char binary_code[BARCODE_ELEMENTS + 1];
    uint32_t min_duration = 0xFFFFFFFF, max_duration = 0;

    printf("Waiting for pre-element (white bar)...\n");
    // Wait until the sensor sees white (low ADC reading)
    while (read_ir_adc() > ADC_THRESHOLD) {
        tight_loop_contents();
    }

    printf("Waiting for barcode start (black bar)...\n");
    // Wait until the sensor sees the first black bar
    while (read_ir_adc() < ADC_THRESHOLD) {
        tight_loop_contents();
    }

    printf("--- Starting Barcode Scan ---\n");

    // Scan the 9 elements of the barcode
    for (int i = 0; i < BARCODE_ELEMENTS; ++i) {
        uint16_t current_adc = read_ir_adc();
        bool is_black = current_adc > ADC_THRESHOLD;
        const char* color_str = is_black ? "Black" : "White";
        
        uint32_t start_time = time_us_32();
        
        // Wait for the color to change
        while ((read_ir_adc() > ADC_THRESHOLD) == is_black) {
            tight_loop_contents();
        }
        
        uint32_t end_time = time_us_32();
        durations[i] = end_time - start_time;

        // Update min and max durations for later classification
        if (durations[i] < min_duration) min_duration = durations[i];
        if (durations[i] > max_duration) max_duration = durations[i];

        // The 'speed' would be inversely proportional to duration, but we don't need to calculate it
        // as the ratio-based decoding handles speed variations.
        printf("Element %d: Duration=%lu us, Color=%s, ADC=%u\n", i, durations[i], color_str, current_adc);
    }

    // Dynamically determine the threshold between narrow and wide bars
    uint32_t width_threshold = min_duration + (max_duration - min_duration) / 2;
    printf("\n--- Analysis ---\n");
    printf("Min Duration: %lu us, Max Duration: %lu us\n", min_duration, max_duration);
    printf("Width Threshold: %lu us\n", width_threshold);

    // Classify elements and build the binary string
    for (int i = 0; i < BARCODE_ELEMENTS; ++i) {
        if (durations[i] > width_threshold) {
            binary_code[i] = '1'; // Wide
            printf("Element %d is WIDE\n", i);
        } else {
            binary_code[i] = '0'; // Narrow
            printf("Element %d is NARROW\n", i);
        }
    }
    binary_code[BARCODE_ELEMENTS] = '\0'; // Null-terminate the string

    printf("\nBinary Code: %s\n", binary_code);

    // Decode the binary string
    char decoded_char = decode_barcode(binary_code);
    
    printf("--- Result ---\n");
    if (decoded_char != '?') {
        printf("Decoded Character: %c\n", decoded_char);
    } else {
        printf("Decoding failed. No match found.\n");
    }
    printf("----------------\n\n");

    return decoded_char;
}

// Main function to set up and run the scanner
int main() {
    stdio_init_all();
    sleep_ms(2000); // Wait for console to connect

    printf("Initializing ADC for Barcode Scanner...\n");
    adc_init();
    adc_gpio_init(IR_SENSOR_ADC_PIN);
    adc_select_input(IR_SENSOR_ADC_INPUT);

    while (1) {
        printf("Press a key to start scanning...\n");
        getchar(); // Wait for user input to start a scan
        scan_barcode_char();
        // The main application logic would use the decoded character here.
        // For this demo, we just loop and wait for the next scan.
    }

    return 0;
}
