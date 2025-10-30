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
    } else if (distance_cm <= 30.0f) {
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
    printf("Continuous scanning mode:\n");
    printf("  CLEAR (>30cm): Servo stops at 90°,\n");
    printf("                 Ultrasonic continues scanning\n");
    printf("  OBSTACLE (<=30cm): Delay 2 seconds,\n");
    printf("                     Servo + Ultrasonic scan\n");
    printf("                     for clear path,\n");
    printf("                     Return to 90° when clear\n");
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
    
    // Initial state: servo at center, in normal scan mode
    servo_set_angle(90.0f);
    bool in_obstacle_mode = false;
    
    while (1) {
        // Measure distance at current position
        float distance_cm = ultrasonic_get_distance_cm();
        uint16_t pulse_us = estimate_pulse_width(distance_cm);
        const char* status = get_status_string(distance_cm);
        
        // ============ NORMAL SCANNING MODE (distance > 20cm) ============
        if (distance_cm > 30.0f && distance_cm < 400.0f) {
            if (in_obstacle_mode) {
                // Transitioning back to normal mode
                printf("[STATUS] Obstacle cleared! Returning to NORMAL SCAN mode\n");
                printf("[STATUS] Servo parking at CENTER (90°)\n\n");
                servo_set_angle(90.0f);
                in_obstacle_mode = false;
                sleep_ms(500);
            }
            
            // In normal mode: servo stays at 90°, ultrasonic keeps scanning
            printf("[NORMAL] %.0f° | %6.1f cm | %4u µs | %s (Servo IDLE at 90°)\n",
                   90.0f,
                   (double)distance_cm,
                   pulse_us,
                   status);
            fflush(stdout);
            sleep_ms(500);  // Continuous ultrasonic scanning
        }
        
        // ============ OBSTACLE DETECTED MODE (distance <= 20cm) ============
        else if (distance_cm <= 30.0f && distance_cm > 0.0f) {
            if (!in_obstacle_mode) {
                // First detection: print alert and delay
                printf("[ALERT] *** OBSTACLE DETECTED at 90° ***\n");
                printf("[ALERT] Distance: %.1f cm\n", (double)distance_cm);
                printf("[ALERT] Delaying 2 seconds before avoidance scan...\n");
                sleep_ms(2000);
                
                printf("[ALERT] Triggering AVOIDANCE SCAN MODE\n");
                in_obstacle_mode = true;
            }
            
            // Active avoidance scanning
            printf("[OBSTACLE] %.0f° | %6.1f cm | %4u µs | %s\n",
                   90.0f,
                   (double)distance_cm,
                   pulse_us,
                   status);
            fflush(stdout);
            
            // Scan left side for clear path
            printf("[AVOIDANCE] Scanning LEFT side...\n");
            float best_angle = 90.0f;
            float best_distance = distance_cm;
            
            for (float check_angle = 95.0f; check_angle <= 170.0f; check_angle += 5.0f) {
                servo_set_angle(check_angle);
                sleep_ms(250);
                float check_distance = ultrasonic_get_distance_cm();
                uint16_t check_pulse = estimate_pulse_width(check_distance);
                
                printf("[AVOIDANCE] LEFT: %.0f° | %6.1f cm | %4u µs\n", 
                       (double)check_angle, (double)check_distance, check_pulse);
                fflush(stdout);
                
                // Track best clear angle (>30cm preferred)
                if (check_distance > 30.0f && check_distance > best_distance) {
                    best_distance = check_distance;
                    best_angle = check_angle;
                }
            }
            
            // Scan right side for clear path
            printf("[AVOIDANCE] Scanning RIGHT side...\n");
            for (float check_angle = 85.0f; check_angle >= 10.0f; check_angle -= 5.0f) {
                servo_set_angle(check_angle);
                sleep_ms(250);
                float check_distance = ultrasonic_get_distance_cm();
                uint16_t check_pulse = estimate_pulse_width(check_distance);
                
                printf("[AVOIDANCE] RIGHT: %.0f° | %6.1f cm | %4u µs\n", 
                       (double)check_angle, (double)check_distance, check_pulse);
                fflush(stdout);
                
                // Track best clear angle (>30cm preferred)
                if (check_distance > 30.0f && check_distance > best_distance) {
                    best_distance = check_distance;
                    best_angle = check_angle;
                }
            }
            
            // Move to best angle or stay at center if no clear path
            if (best_distance > 30.0f && best_angle != 90.0f) {
                printf("[AVOIDANCE] ✓ Clear path found at %.0f° with %.1f cm\n", 
                       (double)best_angle, (double)best_distance);
                servo_set_angle(best_angle);
                sleep_ms(1000);
            } else {
                printf("[AVOIDANCE] ✗ No clear path found, staying at CENTER (90°)\n");
                servo_set_angle(90.0f);
                sleep_ms(1000);
            }
            printf("[AVOIDANCE] === SCAN COMPLETE ===\n\n");
        }
        
        // ============ OUT OF RANGE ============
        else {
            printf("[OUT_RANGE] %.0f° | %6.1f cm | %4u µs | %s\n",
                   90.0f,
                   (double)distance_cm,
                   pulse_us,
                   status);
            fflush(stdout);
            sleep_ms(500);
        }
    }
    
    return 0;
}
