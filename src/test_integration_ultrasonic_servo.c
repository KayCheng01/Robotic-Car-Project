#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "servo.h"
#include "ultrasonic.h"

/**
 * Ultrasonic + Servo Integration Test
 * 
 * Scans obstacle distances while sweeping servo from left to right and back.
 * Prints single-line data: Angle | Distance | Pulse | Status
 * 
 * Expected serial output (115200 baud):
 *   [SERVO] Init OK ...
 *   [ULTRASONIC] Init OK ...
 *   [INTEGRATION] Starting ultrasonic + servo integration test
 *   [SCAN] 90° | 25.5 cm | 1485 µs | OK
 *   [SCAN] 95° | 28.2 cm | 1642 µs | OK
 *   [SCAN] 100° | 30.1 cm | 1750 µs | OK
 *   ...
 */

// Status strings for ultrasonic reading
typedef enum {
    STATUS_OK = 0,
    STATUS_TOO_CLOSE = 1,
    STATUS_NO_OBJECT = 2,
    STATUS_ERROR = 3
} ultrasonic_status_t;

/**
 * Get status string based on distance
 */
static const char* get_status_string(float distance_cm) {
    if (distance_cm < 0.0f) {
        return "OUT_OF_RANGE";
    } else if (distance_cm <= 20.0f) {
        return "OBSTACLE_FOUND";
    } else if (distance_cm <= 50.0f) {
        return "CLEAR_TO_PROCEED";
    } else {
        return "OUT_OF_RANGE";
    }
}

/**
 * Measure distance and return pulse width for debugging
 * Note: ultrasonic.h doesn't expose pulse width directly,
 * so we'll estimate it from distance: pulse = distance / 0.0343
 */
static uint16_t estimate_pulse_width(float distance_cm) {
    if (distance_cm < 0.0f) return 0;
    // Sound travels ~0.0343 cm per microsecond
    // Round trip: distance = (pulse_width_us / 2) * 0.0343
    // pulse_width_us = (distance * 2) / 0.0343
    uint16_t pulse_us = (uint16_t)((distance_cm * 2.0f) / 0.0343f);
    return pulse_us;
}

int main(void) {
    stdio_init_all();
    sleep_ms(500);
    
    printf("\n========================================\n");
    printf("  ULTRASONIC + SERVO INTEGRATION TEST\n");
    printf("========================================\n");
    printf("Scanning with ultrasonic while sweeping servo\n");
    printf("Pattern: LEFT (5° steps) → CENTER → RIGHT (5° steps) → CENTER\n");
    printf("Output format: [SCAN] Angle | Distance | Pulse | Status\n");
    printf("========================================\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(500);
    
    // Initialize ultrasonic
    ultrasonic_init();
    sleep_ms(500);
    
    printf("[INTEGRATION] Starting ultrasonic + servo integration test\n\n");
    printf("[SCAN] Format: Angle | Distance | Pulse | Status\n");
    printf("========================================\n\n");
    
    uint32_t cycle = 0;
    
    while (1) {
        cycle++;
        
        // ============ LEFT SWEEP (90° → 170° → 90°) ============
        // Start at center
        servo_set_angle(90.0f);
        sleep_ms(300);  // Settle
        
        // Sweep left: 90 → 170 in 5° steps
        for (float angle = 90.0f; angle <= 170.0f; angle += 5.0f) {
            servo_set_angle(angle);
            sleep_ms(300);  // Wait for servo and ultrasonic to settle
            
            // Measure distance
            float distance_cm = ultrasonic_get_distance_cm();
            uint16_t pulse_us = estimate_pulse_width(distance_cm);
            const char* status = get_status_string(distance_cm);
            
            // Print single-line data
            printf("[SCAN] %.0f° | %6.1f cm | %4u µs | %s\n",
                   (double)angle,
                   (double)distance_cm,
                   pulse_us,
                   status);
            fflush(stdout);
        }
        
        // Return to center
        servo_set_angle(90.0f);
        sleep_ms(300);
        printf("[SCAN] --- CENTER RETURN ---\n\n");
        
        // ============ RIGHT SWEEP (90° → 10° → 90°) ============
        // Sweep right: 90 → 10 in 5° steps
        for (float angle = 90.0f; angle >= 10.0f; angle -= 5.0f) {
            servo_set_angle(angle);
            sleep_ms(300);  // Wait for servo and ultrasonic to settle
            
            // Measure distance
            float distance_cm = ultrasonic_get_distance_cm();
            uint16_t pulse_us = estimate_pulse_width(distance_cm);
            const char* status = get_status_string(distance_cm);
            
            // Print single-line data
            printf("[SCAN] %.0f° | %6.1f cm | %4u µs | %s\n",
                   (double)angle,
                   (double)distance_cm,
                   pulse_us,
                   status);
            fflush(stdout);
        }
        
        // Return to center
        servo_set_angle(90.0f);
        sleep_ms(300);
        printf("[SCAN] --- CENTER RETURN ---\n");
        printf("[SCAN] === CYCLE %u COMPLETE ===\n\n", (unsigned int)cycle);
    }
    
    return 0;
}
