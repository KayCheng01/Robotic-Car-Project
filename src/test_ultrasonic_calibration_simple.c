#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../ultrasonic/ultrasonic.h"

// ============== Calibration Configuration ==============
#define NUM_SAMPLES_PER_DISTANCE    5   // Number of measurements per distance
#define MEASUREMENT_DELAY_MS        10  // Delay between measurements (ms)

int main(void) {
    stdio_init_all();
    
    // Wait a bit for USB to initialize
    sleep_ms(3000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║         ULTRASONIC SENSOR CALIBRATION TEST - SIMPLE VERSION       ║\n");
    printf("╚════════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Initialize ultrasonic sensor
    printf("[START] Initializing ultrasonic sensor...\n");
    fflush(stdout);
    
    ultrasonic_init();
    
    printf("[OK] Ultrasonic sensor initialized\n");
    printf("\n");
    fflush(stdout);
    
    sleep_ms(2000);
    
    printf("═══════════════════════════════════════════════════════════════════════\n");
    printf("Starting continuous distance measurements...\n");
    printf("═══════════════════════════════════════════════════════════════════════\n");
    printf("\n");
    fflush(stdout);
    
    // Simple continuous monitoring loop
    int measurement_count = 0;
    
    while (1) {
        measurement_count++;
        
        float distance = ultrasonic_get_distance_cm();
        
        // Print with timestamp and status
        printf("[%4d] Distance: %7.2f cm", measurement_count, distance);
        
        // Add status
        if (distance < 0) {
            printf(" | Status: TIMEOUT");
        } else if (distance < 2.0f) {
            printf(" | Status: TOO_CLOSE");
        } else if (distance <= 30.0f) {
            printf(" | Status: OBSTACLE !!!");
        } else if (distance <= 50.0f) {
            printf(" | Status: CAUTION");
        } else if (distance <= 200.0f) {
            printf(" | Status: CLEAR");
        } else {
            printf(" | Status: OUT_OF_RANGE");
        }
        
        printf("\n");
        fflush(stdout);
        
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        // Optional: stop after 30 measurements for testing
        if (measurement_count >= 100) {
            printf("\n[COMPLETE] Test finished after %d measurements\n", measurement_count);
            fflush(stdout);
            break;
        }
    }
    
    printf("\n═══════════════════════════════════════════════════════════════════════\n");
    printf("Test complete! Waiting for input...\n");
    printf("═══════════════════════════════════════════════════════════════════════\n");
    fflush(stdout);
    
    while (1) {
        sleep_ms(1000);
    }
    
    return 0;
}
