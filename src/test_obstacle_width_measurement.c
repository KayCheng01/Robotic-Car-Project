#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"
#include "servo.h"
#include "ultrasonic.h"

/**
 * Ultrasonic + Servo Obstacle Width Measurement
 * 
 * Advanced obstacle detection with width calculation:
 * 1. Detect obstacle at ≤30cm
 * 2. Reconfirm distance (perpendicular distance stored)
 * 3. Sweep servo left/right until transition to CLEAR/OUT_OF_RANGE
 * 4. Store hypotenuse distances at transition points
 * 5. Calculate obstacle width using Pythagorean theorem
 * 
 * Geometry:
 *   perpendicular distance (d) = distance at 90°
 *   hypotenuse left (h_left) = distance at left transition angle
 *   hypotenuse right (h_right) = distance at right transition angle
 *   
 *   Using Pythagorean theorem: a² + b² = c²
 *   horizontal_distance = sqrt(hypotenuse² - perpendicular²)
 *   
 *   obstacle_width = left_horizontal + right_horizontal
 */

typedef struct {
    float perpendicular_distance;      // Distance at 90° (confirmed)
    float hypotenuse_left;             // Distance at left transition angle
    float hypotenuse_right;            // Distance at right transition angle
    float left_transition_angle;       // Angle where left side clears obstacle
    float right_transition_angle;      // Angle where right side clears obstacle
    float left_horizontal_distance;    // Calculated from left side
    float right_horizontal_distance;   // Calculated from right side
    float total_obstacle_width;        // Sum of both sides
} obstacle_measurement_t;

obstacle_measurement_t obstacle_data;

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
 * Estimate pulse width from distance
 */
static uint16_t estimate_pulse_width(float distance_cm) {
    if (distance_cm < 0.0f) return 0;
    uint16_t pulse_us = (uint16_t)((distance_cm * 2.0f) / 0.0343f);
    return pulse_us;
}

/**
 * Calculate horizontal distance using Pythagorean theorem
 * horizontal = sqrt(hypotenuse² - perpendicular²)
 */
static float calculate_horizontal_distance(float hypotenuse, float perpendicular) {
    if (hypotenuse <= perpendicular) return 0.0f;
    float horizontal_sq = (hypotenuse * hypotenuse) - (perpendicular * perpendicular);
    return sqrt(horizontal_sq);
}

int main(void) {
    stdio_init_all();
    sleep_ms(500);
    
    printf("\n========================================\n");
    printf("  OBSTACLE WIDTH MEASUREMENT SYSTEM\n");
    printf("========================================\n");
    printf("Advanced obstacle detection with width:\n");
    printf("1. Detect obstacle at ≤30cm\n");
    printf("2. Reconfirm perpendicular distance\n");
    printf("3. Sweep to find transition points\n");
    printf("4. Calculate obstacle width\n");
    printf("========================================\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(500);
    
    // Initialize ultrasonic
    ultrasonic_init();
    sleep_ms(500);
    
    printf("[INTEGRATION] Starting obstacle measurement system\n\n");
    
    // Initial state
    servo_set_angle(90.0f);
    bool obstacle_detected = false;
    
    while (1) {
        // Measure distance at current position
        float distance_cm = ultrasonic_get_distance_cm();
        uint16_t pulse_us = estimate_pulse_width(distance_cm);
        const char* status = get_status_string(distance_cm);
        
        // ============ MONITORING FOR OBSTACLE ============
        if (!obstacle_detected) {
            printf("[MONITOR] %.0f° | %6.1f cm | %4u µs | %s\n",
                   90.0f,
                   (double)distance_cm,
                   pulse_us,
                   status);
            fflush(stdout);
            sleep_ms(300);
            
            // Check if obstacle detected at ≤30cm
            if (distance_cm <= 30.0f && distance_cm > 0.0f) {
                printf("\n");
                printf("╔════════════════════════════════════╗\n");
                printf("║    OBSTACLE DETECTED AT ≤30cm!     ║\n");
                printf("╚════════════════════════════════════╝\n\n");
                
                printf("[MEASURE] Step 1: Confirming perpendicular distance\n");
                sleep_ms(500);
                
                // Reconfirm the perpendicular distance (multiple measurements)
                float confirmed_distance = 0.0f;
                printf("[MEASURE] Taking 3 measurements for confirmation:\n");
                for (int i = 0; i < 3; i++) {
                    sleep_ms(100);
                    float measure = ultrasonic_get_distance_cm();
                    printf("[MEASURE]   Measurement %d: %.1f cm\n", i+1, (double)measure);
                    confirmed_distance += measure;
                }
                confirmed_distance /= 3.0f;
                obstacle_data.perpendicular_distance = confirmed_distance;
                
                printf("[MEASURE] ✓ Perpendicular distance confirmed: %.1f cm\n\n", 
                       (double)obstacle_data.perpendicular_distance);
                
                printf("[MEASURE] Step 2: LEFT SWEEP - Finding transition point\n");
                printf("[MEASURE] Scanning from 90° → 170° for clear path\n");
                
                // LEFT SWEEP: Find transition point where obstacle clears
                bool left_transition_found = false;
                for (float angle = 95.0f; angle <= 170.0f; angle += 5.0f) {
                    servo_set_angle(angle);
                    sleep_ms(300);
                    
                    float sweep_distance = ultrasonic_get_distance_cm();
                    uint16_t sweep_pulse = estimate_pulse_width(sweep_distance);
                    const char* sweep_status = get_status_string(sweep_distance);
                    
                    printf("[LEFT_SWEEP] %.0f° | %6.1f cm | %4u µs | %s\n",
                           (double)angle,
                           (double)sweep_distance,
                           sweep_pulse,
                           sweep_status);
                    fflush(stdout);
                    
                    // Detect transition: from OBSTACLE to CLEAR/OUT_OF_RANGE
                    if (sweep_distance > 30.0f && !left_transition_found) {
                        printf("[LEFT_TRANSITION] ✓ Found at %.0f° with %.1f cm\n", 
                               (double)angle, (double)sweep_distance);
                        obstacle_data.left_transition_angle = angle;
                        obstacle_data.hypotenuse_left = sweep_distance;
                        left_transition_found = true;
                    }
                }
                
                if (left_transition_found) {
                    printf("[CALCULATE] LEFT: hypotenuse=%.1f cm, perpendicular=%.1f cm\n",
                           (double)obstacle_data.hypotenuse_left,
                           (double)obstacle_data.perpendicular_distance);
                    
                    obstacle_data.left_horizontal_distance = calculate_horizontal_distance(
                        obstacle_data.hypotenuse_left,
                        obstacle_data.perpendicular_distance
                    );
                    
                    printf("[CALCULATE] LEFT: horizontal_distance = √(%.1f² - %.1f²) = %.1f cm\n",
                           (double)obstacle_data.hypotenuse_left,
                           (double)obstacle_data.perpendicular_distance,
                           (double)obstacle_data.left_horizontal_distance);
                } else {
                    printf("[LEFT_SWEEP] ✗ No clear path found on left side\n");
                    obstacle_data.left_horizontal_distance = 0.0f;
                }
                
                printf("\n[MEASURE] Step 3: RIGHT SWEEP - Finding transition point\n");
                printf("[MEASURE] Scanning from 90° → 10° for clear path\n");
                
                // Return to center first
                servo_set_angle(90.0f);
                sleep_ms(300);
                
                // RIGHT SWEEP: Find transition point where obstacle clears
                bool right_transition_found = false;
                for (float angle = 85.0f; angle >= 10.0f; angle -= 5.0f) {
                    servo_set_angle(angle);
                    sleep_ms(300);
                    
                    float sweep_distance = ultrasonic_get_distance_cm();
                    uint16_t sweep_pulse = estimate_pulse_width(sweep_distance);
                    const char* sweep_status = get_status_string(sweep_distance);
                    
                    printf("[RIGHT_SWEEP] %.0f° | %6.1f cm | %4u µs | %s\n",
                           (double)angle,
                           (double)sweep_distance,
                           sweep_pulse,
                           sweep_status);
                    fflush(stdout);
                    
                    // Detect transition: from OBSTACLE to CLEAR/OUT_OF_RANGE
                    if (sweep_distance > 30.0f && !right_transition_found) {
                        printf("[RIGHT_TRANSITION] ✓ Found at %.0f° with %.1f cm\n", 
                               (double)angle, (double)sweep_distance);
                        obstacle_data.right_transition_angle = angle;
                        obstacle_data.hypotenuse_right = sweep_distance;
                        right_transition_found = true;
                    }
                }
                
                if (right_transition_found) {
                    printf("[CALCULATE] RIGHT: hypotenuse=%.1f cm, perpendicular=%.1f cm\n",
                           (double)obstacle_data.hypotenuse_right,
                           (double)obstacle_data.perpendicular_distance);
                    
                    obstacle_data.right_horizontal_distance = calculate_horizontal_distance(
                        obstacle_data.hypotenuse_right,
                        obstacle_data.perpendicular_distance
                    );
                    
                    printf("[CALCULATE] RIGHT: horizontal_distance = √(%.1f² - %.1f²) = %.1f cm\n",
                           (double)obstacle_data.hypotenuse_right,
                           (double)obstacle_data.perpendicular_distance,
                           (double)obstacle_data.right_horizontal_distance);
                } else {
                    printf("[RIGHT_SWEEP] ✗ No clear path found on right side\n");
                    obstacle_data.right_horizontal_distance = 0.0f;
                }
                
                // ============ CALCULATE TOTAL OBSTACLE WIDTH ============
                obstacle_data.total_obstacle_width = obstacle_data.left_horizontal_distance + 
                                                    obstacle_data.right_horizontal_distance;
                
                printf("\n╔════════════════════════════════════╗\n");
                printf("║    OBSTACLE MEASUREMENT RESULT     ║\n");
                printf("╠════════════════════════════════════╣\n");
                printf("║ Perpendicular Distance: %6.1f cm ║\n", 
                       (double)obstacle_data.perpendicular_distance);
                printf("║                                    ║\n");
                printf("║ LEFT SIDE:                         ║\n");
                printf("║   Transition Angle: %5.0f°         ║\n", 
                       (double)obstacle_data.left_transition_angle);
                printf("║   Hypotenuse: %6.1f cm            ║\n", 
                       (double)obstacle_data.hypotenuse_left);
                printf("║   Horizontal: %6.1f cm            ║\n", 
                       (double)obstacle_data.left_horizontal_distance);
                printf("║                                    ║\n");
                printf("║ RIGHT SIDE:                        ║\n");
                printf("║   Transition Angle: %5.0f°         ║\n", 
                       (double)obstacle_data.right_transition_angle);
                printf("║   Hypotenuse: %6.1f cm            ║\n", 
                       (double)obstacle_data.hypotenuse_right);
                printf("║   Horizontal: %6.1f cm            ║\n", 
                       (double)obstacle_data.right_horizontal_distance);
                printf("║                                    ║\n");
                printf("║ ★ OBSTACLE WIDTH: %6.1f cm ★    ║\n", 
                       (double)obstacle_data.total_obstacle_width);
                printf("╚════════════════════════════════════╝\n\n");
                
                // Return to center
                servo_set_angle(90.0f);
                obstacle_detected = true;
                
                // Wait before resuming monitoring
                printf("[MONITOR] Waiting 5 seconds before resuming...\n");
                sleep_ms(5000);
            }
        }
        
        // If obstacle was detected and now cleared, reset
        if (obstacle_detected && distance_cm > 30.0f) {
            printf("[MONITOR] Obstacle cleared! Resuming monitoring...\n\n");
            obstacle_detected = false;
        }
    }
    
    return 0;
}
