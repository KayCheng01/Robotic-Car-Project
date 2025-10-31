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
 * 4. Store distances and angles at transition points
 * 5. Calculate obstacle width using Cosine Rule
 * 
 * Geometry (using Cosine Rule):
 *   c² = a² + b² - 2ab·cos(C)
 *   
 *   For LEFT side:
 *     a = distance at 90°
 *     b = distance at left_angle
 *     C = angle between them (left_angle - 90°)
 *     c = left_edge_distance (obstacle edge distance)
 *   
 *   For RIGHT side:
 *     a = distance at 90°
 *     b = distance at right_angle
 *     C = angle between them (90° - right_angle)
 *     c = right_edge_distance (obstacle edge distance)
 *   
 *   obstacle_width = left_edge_distance + right_edge_distance
 */

typedef struct {
    float center_distance;             // Distance at 90° (center, confirmed)
    float distance_left;               // Distance at left transition angle
    float distance_right;              // Distance at right transition angle
    float left_transition_angle;       // Angle where left side clears obstacle
    float right_transition_angle;      // Angle where right side clears obstacle
    float left_angle_delta;            // Angle difference for left: left_angle - 90°
    float right_angle_delta;           // Angle difference for right: 90° - right_angle
    float left_edge_distance;          // Calculated using cosine rule (left side)
    float right_edge_distance;         // Calculated using cosine rule (right side)
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
 * Calculate edge distance using Cosine Rule
 * c² = a² + b² - 2ab·cos(C)
 * 
 * @param distance_at_90: distance measured at 90° (side a)
 * @param distance_at_angle: distance measured at sweep angle (side b)
 * @param angle_degrees: angle difference from 90° (angle C)
 * @return: edge distance (side c)
 */
static float calculate_edge_distance_cosine_rule(float distance_at_90, float distance_at_angle, float angle_degrees) {
    if (distance_at_90 <= 0.0f || distance_at_angle <= 0.0f) return 0.0f;
    
    // Convert angle to radians
    float angle_rad = angle_degrees * (M_PI / 180.0f);
    
    // Apply cosine rule: c² = a² + b² - 2ab·cos(C)
    float a = distance_at_90;
    float b = distance_at_angle;
    float cos_c = cosf(angle_rad);
    
    float c_squared = (a * a) + (b * b) - (2.0f * a * b * cos_c);
    
    if (c_squared < 0.0f) return 0.0f;
    
    return sqrtf(c_squared);
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
                obstacle_data.center_distance = confirmed_distance;
                
                printf("[MEASURE] ✓ Center distance confirmed: %.1f cm\n\n", 
                       (double)obstacle_data.center_distance);
                
                printf("[MEASURE] Step 2: LEFT SWEEP - Finding transition point\n");
                printf("[MEASURE] Scanning from 90° → 170° for clear path (1° increments)\n");
                
                // LEFT SWEEP: Find transition point where obstacle clears
                bool left_transition_found = false;
                for (float angle = 91.0f; angle <= 170.0f; angle += 1.0f) {
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
                        obstacle_data.distance_left = sweep_distance;
                        obstacle_data.left_angle_delta = angle - 90.0f;
                        left_transition_found = true;
                    }
                }
                
                if (left_transition_found) {
                    printf("[CALCULATE] LEFT: distance_at_90°=%.1f cm, distance_at_%.0f°=%.1f cm, angle_delta=%.0f°\n",
                           (double)obstacle_data.center_distance,
                           (double)obstacle_data.left_transition_angle,
                           (double)obstacle_data.distance_left,
                           (double)obstacle_data.left_angle_delta);
                    
                    obstacle_data.left_edge_distance = calculate_edge_distance_cosine_rule(
                        obstacle_data.center_distance,
                        obstacle_data.distance_left,
                        obstacle_data.left_angle_delta
                    );
                    
                    printf("[CALCULATE] LEFT: Using cosine rule c²=a²+b²-2ab·cos(C)\n");
                    printf("[CALCULATE] LEFT: c² = %.1f² + %.1f² - 2·%.1f·%.1f·cos(%.0f°)\n",
                           (double)obstacle_data.center_distance,
                           (double)obstacle_data.distance_left,
                           (double)obstacle_data.center_distance,
                           (double)obstacle_data.distance_left,
                           (double)obstacle_data.left_angle_delta);
                    printf("[CALCULATE] LEFT: edge_distance = %.1f cm\n",
                           (double)obstacle_data.left_edge_distance);
                } else {
                    printf("[LEFT_SWEEP] ✗ No clear path found on left side\n");
                    obstacle_data.left_edge_distance = 0.0f;
                }
                
                printf("\n[MEASURE] Step 3: RIGHT SWEEP - Finding transition point\n");
                printf("[MEASURE] Scanning from 90° → 10° for clear path (1° increments)\n");
                
                // Return to center first
                servo_set_angle(90.0f);
                sleep_ms(300);
                
                // RIGHT SWEEP: Find transition point where obstacle clears
                bool right_transition_found = false;
                for (float angle = 89.0f; angle >= 10.0f; angle -= 1.0f) {
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
                        obstacle_data.distance_right = sweep_distance;
                        obstacle_data.right_angle_delta = 90.0f - angle;
                        right_transition_found = true;
                    }
                }
                
                if (right_transition_found) {
                    printf("[CALCULATE] RIGHT: distance_at_90°=%.1f cm, distance_at_%.0f°=%.1f cm, angle_delta=%.0f°\n",
                           (double)obstacle_data.center_distance,
                           (double)obstacle_data.right_transition_angle,
                           (double)obstacle_data.distance_right,
                           (double)obstacle_data.right_angle_delta);
                    
                    obstacle_data.right_edge_distance = calculate_edge_distance_cosine_rule(
                        obstacle_data.center_distance,
                        obstacle_data.distance_right,
                        obstacle_data.right_angle_delta
                    );
                    
                    printf("[CALCULATE] RIGHT: Using cosine rule c²=a²+b²-2ab·cos(C)\n");
                    printf("[CALCULATE] RIGHT: c² = %.1f² + %.1f² - 2·%.1f·%.1f·cos(%.0f°)\n",
                           (double)obstacle_data.center_distance,
                           (double)obstacle_data.distance_right,
                           (double)obstacle_data.center_distance,
                           (double)obstacle_data.distance_right,
                           (double)obstacle_data.right_angle_delta);
                    printf("[CALCULATE] RIGHT: edge_distance = %.1f cm\n",
                           (double)obstacle_data.right_edge_distance);
                } else {
                    printf("[RIGHT_SWEEP] ✗ No clear path found on right side\n");
                    obstacle_data.right_edge_distance = 0.0f;
                }
                
                // ============ CALCULATE TOTAL OBSTACLE WIDTH ============
                obstacle_data.total_obstacle_width = obstacle_data.left_edge_distance + 
                                                    obstacle_data.right_edge_distance;
                
                printf("\n╔════════════════════════════════════╗\n");
                printf("║    OBSTACLE MEASUREMENT RESULT     ║\n");
                printf("╠════════════════════════════════════╣\n");
                printf("║ Center Distance: %6.1f cm        ║\n", 
                       (double)obstacle_data.center_distance);
                printf("║                                    ║\n");
                printf("║ LEFT SIDE (Cosine Rule):          ║\n");
                printf("║   Transition Angle: %5.0f°         ║\n", 
                       (double)obstacle_data.left_transition_angle);
                printf("║   Distance at angle: %6.1f cm     ║\n", 
                       (double)obstacle_data.distance_left);
                printf("║   Angle delta: %5.0f°              ║\n", 
                       (double)obstacle_data.left_angle_delta);
                printf("║   Edge distance: %6.1f cm         ║\n", 
                       (double)obstacle_data.left_edge_distance);
                printf("║                                    ║\n");
                printf("║ RIGHT SIDE (Cosine Rule):         ║\n");
                printf("║   Transition Angle: %5.0f°         ║\n", 
                       (double)obstacle_data.right_transition_angle);
                printf("║   Distance at angle: %6.1f cm     ║\n", 
                       (double)obstacle_data.distance_right);
                printf("║   Angle delta: %5.0f°              ║\n", 
                       (double)obstacle_data.right_angle_delta);
                printf("║   Edge distance: %6.1f cm         ║\n", 
                       (double)obstacle_data.right_edge_distance);
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
