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

char char_array[TOTAL_CHAR] = {
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
    'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
    'U', 'V', 'W', 'X', 'Y', 'Z',
    '-', '.', ' ', '*', '$', '/', '+', '%'
};

char *code_array[TOTAL_CHAR] = {
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
    "011000100", // (space)
    "010010100", // *
    "010101000", // $
    "010100010", // /
    "010001010", // +
    "000101010"  // %
};

char *reverse_code_array[TOTAL_CHAR] = {
    "001011000", // reverse of "000110100" → '0'
    "100001001", // reverse of "100100001" → '1'
    "100001100", // reverse of "001100001" → '2'
    "000001101", // reverse of "101100000" → '3'
    "100011000", // reverse of "000110001" → '4'
    "000011001", // reverse of "100110000" → '5'
    "000011100", // reverse of "001110000" → '6'
    "101001000", // reverse of "000100101" → '7'
    "001001001", // reverse of "100100100" → '8'
    "001001100", // reverse of "001100100" → '9'
    "100100001", // reverse of "100001001" → 'A'
    "100100100", // reverse of "001001001" → 'B'
    "000100101", // reverse of "101001000" → 'C'
    "100110000", // reverse of "000011001" → 'D'
    "000110001", // reverse of "100011000" → 'E'
    "000110100", // reverse of "001011000" → 'F'
    "101100000", // reverse of "000001101" → 'G'
    "001100001", // reverse of "100001100" → 'H'
    "001100100", // reverse of "001001100" → 'I'
    "001110000", // reverse of "000011100" → 'J'
    "110000001", // reverse of "100000011" → 'K'
    "110000100", // reverse of "001000011" → 'L'
    "010000101", // reverse of "101000010" → 'M'
    "110010000", // reverse of "000010011" → 'N'
    "010010001", // reverse of "100010010" → 'O'
    "010010100", // reverse of "001010010" → 'P'
    "111000000", // reverse of "000000111" → 'Q'
    "011000001", // reverse of "100000110" → 'R'
    "011000100", // reverse of "001000110" → 'S'
    "011010000", // reverse of "000010110" → 'T'
    "100000011", // reverse of "110000001" → 'U'
    "100000110", // reverse of "011000001" → 'V'
    "000000111", // reverse of "111000000" → 'W'
    "100010010", // reverse of "010010001" → 'X'
    "000010011", // reverse of "110010000" → 'Y'
    "000010110", // reverse of "011010000" → 'Z'
    "101000010", // reverse of "010000101" → '-'
    "001000011", // reverse of "110000100" → '.'
    "001000110", // reverse of "011000100" → ' ' (space)
    "001010010", // reverse of "010010100" → '*'  ← note: this is different!
    "000101010", // reverse of "010101000" → '$'
    "010001010", // reverse of "010100010" → '/'
    "010100010", // reverse of "010001010" → '+'
    "010101000"  // reverse of "000101010" → '%'
};

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
