#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../ultrasonic/ultrasonic.h"
#include "../servo/servo.h"

// ============== Configuration ==============
#define OBSTACLE_THRESHOLD_CM           25.0f   // Obstacle detection threshold
#define SERVO_CENTER_ANGLE              90.0f   // Center/perpendicular angle
#define SERVO_RIGHT_ANGLE               150.0f  // Max right angle
#define SERVO_LEFT_ANGLE                35.0f   // Max left angle
#define SERVO_SCAN_STEP_DEGREE          1.0f    // Fast scanning: 1 degree increments
#define SERVO_REFINE_STEP_DEGREE        0.5f    // Fine scanning: 0.5 degree increments
#define MEASUREMENT_DELAY_MS            50      // Fast delay: 50ms between measurements
#define REFINEMENT_DELAY_MS             2000    // 2 second delay for refinement
#define MAX_DETECTION_DISTANCE_CM       100.0f  // Maximum valid echo distance
#define EDGE_CONFIRMATION_SAMPLES       5       // Number of samples to confirm edge detection
#define CONSISTENCY_THRESHOLD_CM        10.0f   // Maximum variation for consistent samples

// ============== Data Structures ==============
typedef struct {
    float angle;
    float distance_cm;
    bool echo_detected;
} edge_point_t;

// ============== Sensor Control Functions ==============

/**
 * Calculate width component using Pythagorean theorem
 * width = sqrt(hypotenuse² - adjacent²)
 * 
 * @param adjacent: perpendicular distance (initial stopping distance at 90°)
 * @param hypotenuse: measured distance at edge angle
 * @return: width component in cm
 */
float calculate_width_component(float adjacent, float hypotenuse) {
    // Pythagorean theorem: width = sqrt(hypotenuse² - adjacent²)
    float h_squared = hypotenuse * hypotenuse;
    float a_squared = adjacent * adjacent;
    
    if (h_squared <= a_squared) {
        printf("    [WARNING] Hypotenuse (%.2f) <= adjacent (%.2f), returning 0\n", 
               hypotenuse, adjacent);
        return 0.0f;
    }
    
    float width = sqrtf(h_squared - a_squared);
    
    return width;
}

/**
 * Take samples and check consistency
 */
bool take_consistent_samples(float *avg_distance_out) {
    float samples[EDGE_CONFIRMATION_SAMPLES];
    int valid_samples = 0;
    
    for (int i = 0; i < EDGE_CONFIRMATION_SAMPLES; i++) {
        sleep_ms(100);
        float sample = ultrasonic_get_distance_cm();
        
        if (sample > 0 && sample <= MAX_DETECTION_DISTANCE_CM) {
            samples[valid_samples] = sample;
            valid_samples++;
            printf("      Sample %d: %.2f cm\n", i + 1, sample);
        } else {
            printf("      Sample %d: invalid\n", i + 1);
        }
    }
    
    if (valid_samples < 3) {
        return false;
    }
    
    // Check consistency
    float min_sample = 999.0f;
    float max_sample = 0.0f;
    float sum = 0.0f;
    
    for (int i = 0; i < valid_samples; i++) {
        sum += samples[i];
        if (samples[i] < min_sample) min_sample = samples[i];
        if (samples[i] > max_sample) max_sample = samples[i];
    }
    
    float variation = max_sample - min_sample;
    *avg_distance_out = sum / valid_samples;
    
    return (variation <= CONSISTENCY_THRESHOLD_CM);
}

/**
 * Scan LEFT SIDE: Start at 35°, scan toward 90° until echo detected
 * Then refine by moving back and confirming edge
 * 
 * @param adjacent: perpendicular distance (adjacent length)
 * @return: left width component
 */
float scan_left_side(float adjacent) {
    printf("\n[LEFT_SCAN] Waiting 2 seconds before scanning...\n");
    sleep_ms(2000);
    
    printf("[LEFT_SCAN] Starting from 35° scanning toward 90°...\n");
    printf("[LEFT_SCAN] Looking for ECHO (edge detection)...\n");
    printf("[LEFT_SCAN] Angle │ Status\n");
    printf("[LEFT_SCAN] ──────┼────────────────\n");
    
    float current_angle = SERVO_LEFT_ANGLE;
    float left_width = 0.0f;
    bool edge_found = false;
    float edge_distance = 0.0f;
    
    // Fast scan to find edge
    while (current_angle <= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        float distance = ultrasonic_get_distance_cm();
        
        if (distance > 0 && distance <= MAX_DETECTION_DISTANCE_CM) {
            printf("[LEFT_SCAN] %.1f° │ ✓ ECHO! Confirming...\n", current_angle);
            
            float avg_distance;
            if (take_consistent_samples(&avg_distance)) {
                printf("[LEFT_SCAN] ✓✓ EDGE FOUND at %.1f°! Starting refinement...\n", current_angle);
                edge_found = true;
                edge_distance = avg_distance;
                break;
            } else {
                printf("[LEFT_SCAN] ✗ Inconsistent, continuing...\n");
            }
        } else {
            printf("[LEFT_SCAN] %.1f° │ No echo\n", current_angle);
        }
        
        current_angle += SERVO_SCAN_STEP_DEGREE;
    }
    
    if (!edge_found) {
        printf("[LEFT_SCAN] No edge found!\n");
        return 0.0f;
    }
    
    // Refinement: Move back and check continuously
    printf("\n[LEFT_REFINE] Starting edge refinement (moving back toward 35°)...\n");
    
    while (current_angle >= SERVO_LEFT_ANGLE) {
        current_angle -= SERVO_REFINE_STEP_DEGREE;
        servo_set_angle(current_angle);
        
        printf("[LEFT_REFINE] Testing %.1f° (waiting 2s)...\n", current_angle);
        sleep_ms(REFINEMENT_DELAY_MS);
        
        float avg_distance;
        if (take_consistent_samples(&avg_distance)) {
            printf("[LEFT_REFINE] ✓ Still edge at %.1f°, avg: %.2f cm\n", current_angle, avg_distance);
            edge_distance = avg_distance;
        } else {
            printf("[LEFT_REFINE] ✗ Edge lost! Final edge at %.1f°\n", current_angle + SERVO_REFINE_STEP_DEGREE);
            current_angle += SERVO_REFINE_STEP_DEGREE;
            break;
        }
    }
    
    // Calculate width
    left_width = calculate_width_component(adjacent, edge_distance);
    printf("[LEFT_REFINE] Final edge angle: %.1f°\n", current_angle);
    printf("[LEFT_REFINE] Final distance: %.2f cm\n", edge_distance);
    printf("[LEFT_REFINE] LEFT width: %.2f cm\n", left_width);
    
    if (!edge_found) {
        printf("[LEFT_SCAN] WARNING: No edge found, reached 90°\n");
    }
    
    return left_width;
}

/**
 * Scan RIGHT SIDE: Start at 150°, scan toward 90° until echo detected
 * Then refine by moving back and confirming edge
 * 
 * @param adjacent: perpendicular distance (adjacent length)
 * @return: right width component
 */
float scan_right_side(float adjacent) {
    printf("\n[RIGHT_SCAN] Waiting 2 seconds before scanning...\n");
    sleep_ms(2000);
    
    printf("[RIGHT_SCAN] Starting from 150° scanning toward 90°...\n");
    printf("[RIGHT_SCAN] Looking for ECHO (edge detection)...\n");
    printf("[RIGHT_SCAN] Angle │ Status\n");
    printf("[RIGHT_SCAN] ──────┼────────────────\n");
    
    float current_angle = SERVO_RIGHT_ANGLE;
    float right_width = 0.0f;
    bool edge_found = false;
    float edge_distance = 0.0f;
    
    // Fast scan to find edge
    while (current_angle >= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        float distance = ultrasonic_get_distance_cm();
        
        if (distance > 0 && distance <= MAX_DETECTION_DISTANCE_CM) {
            printf("[RIGHT_SCAN] %.1f° │ ✓ ECHO! Confirming...\n", current_angle);
            
            float avg_distance;
            if (take_consistent_samples(&avg_distance)) {
                printf("[RIGHT_SCAN] ✓✓ EDGE FOUND at %.1f°! Starting refinement...\n", current_angle);
                edge_found = true;
                edge_distance = avg_distance;
                break;
            } else {
                printf("[RIGHT_SCAN] ✗ Inconsistent, continuing...\n");
            }
        } else {
            printf("[RIGHT_SCAN] %.1f° │ No echo\n", current_angle);
        }
        
        current_angle -= SERVO_SCAN_STEP_DEGREE;
    }
    
    if (!edge_found) {
        printf("[RIGHT_SCAN] No edge found!\n");
        return 0.0f;
    }
    
    // Refinement: Move back and check continuously
    printf("\n[RIGHT_REFINE] Starting edge refinement (moving back toward 150°)...\n");
    
    while (current_angle <= SERVO_RIGHT_ANGLE) {
        current_angle += SERVO_REFINE_STEP_DEGREE;
        servo_set_angle(current_angle);
        
        printf("[RIGHT_REFINE] Testing %.1f° (waiting 2s)...\n", current_angle);
        sleep_ms(REFINEMENT_DELAY_MS);
        
        float avg_distance;
        if (take_consistent_samples(&avg_distance)) {
            printf("[RIGHT_REFINE] ✓ Still edge at %.1f°, avg: %.2f cm\n", current_angle, avg_distance);
            edge_distance = avg_distance;
        } else {
            printf("[RIGHT_REFINE] ✗ Edge lost! Final edge at %.1f°\n", current_angle - SERVO_REFINE_STEP_DEGREE);
            current_angle -= SERVO_REFINE_STEP_DEGREE;
            break;
        }
    }
    
    // Calculate width
    right_width = calculate_width_component(adjacent, edge_distance);
    printf("[RIGHT_REFINE] Final edge angle: %.1f°\n", current_angle);
    printf("[RIGHT_REFINE] Final distance: %.2f cm\n", edge_distance);
    printf("[RIGHT_REFINE] RIGHT width: %.2f cm\n", right_width);
    
    if (!edge_found) {
        printf("[RIGHT_SCAN] WARNING: No edge found, reached 90°\n");
    }
    
    return right_width;
}

// ============== Main Program ==============
int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║      OBJECT WIDTH DETECTION WITH ECHO-BASED EDGE SCANNING        ║\n");
    printf("║                                                                    ║\n");
    printf("║  Servo Range: 35° (left) to 150° (right), center at 90°          ║\n");
    printf("║  Obstacle Detection Threshold: %.1f cm                             ║\n", OBSTACLE_THRESHOLD_CM);
    printf("║  Scan: %.0f° steps, Refine: %.1f° steps                            ║\n", SERVO_SCAN_STEP_DEGREE, SERVO_REFINE_STEP_DEGREE);
    printf("║                                                                    ║\n");
    printf("║  Method: PYTHAGOREAN THEOREM with Edge Refinement                ║\n");
    printf("║  1. Fast scan until edge found                                   ║\n");
    printf("║  2. Move back 0.5° with 2s delay to refine edge                  ║\n");
    printf("║  3. Continue until edge lost                                     ║\n");
    printf("║  4. Calculate width: √(hypotenuse² - adjacent²)                  ║\n");
    printf("║  5. Total width = left_width + right_width                       ║\n");
    printf("║                                                                    ║\n");
    printf("╚════════════════════════════════════════════════════════════════════╝\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(500);
    
    // Initialize ultrasonic
    ultrasonic_init();
    sleep_ms(500);
    
    printf("[INIT] System initialized. Moving servo to center (90°)...\n\n");
    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(500);
    
    while (1) {
        // Monitor distance at center
        float distance_cm = ultrasonic_get_distance_cm();
        
        printf("[MONITOR] Center (90°): ");
        if (distance_cm < 0.0f) {
            printf("timeout/out of range");
        } else {
            printf("%.2f cm", distance_cm);
        }
        printf("\n");
        fflush(stdout);
        sleep_ms(500);
        
        // Check if obstacle detected
        if (distance_cm <= OBSTACLE_THRESHOLD_CM && distance_cm > 0.0f) {
            printf("\n");
            printf("╔════════════════════════════════════════════════════════════════════╗\n");
            printf("║                 *** OBSTACLE DETECTED ***                          ║\n");
            printf("║           Perpendicular Distance: %.2f cm (adjacent)               ║\n", 
                   distance_cm);
            printf("╚════════════════════════════════════════════════════════════════════╝\n");
            
            // Step 1: Use current distance as adjacent (perpendicular distance)
            float adjacent = distance_cm;
            printf("\n[ADJACENT] Using perpendicular distance: %.2f cm\n", adjacent);
            
            // Step 2: Scan LEFT side (35° → 90°) - find first echo
            float left_width = scan_left_side(adjacent);
            
            sleep_ms(500);
            
            // Step 3: Scan RIGHT side (150° → 90°) - find first echo
            float right_width = scan_right_side(adjacent);
            
            // Step 4: Calculate total object width
            float total_width = left_width + right_width;
            
            printf("\n");
            printf("╔════════════════════════════════════════════════════════════════════╗\n");
            printf("║                    OBJECT WIDTH RESULTS                           ║\n");
            printf("╠════════════════════════════════════════════════════════════════════╣\n");
            printf("║  Perpendicular distance:    %.2f cm                                ║\n", adjacent);
            printf("║  Left edge width:           %.2f cm                                ║\n", left_width);
            printf("║  Right edge width:          %.2f cm                                ║\n", right_width);
            printf("║  ──────────────────────────────────────────────────────────────   ║\n");
            printf("║                                                                    ║\n");
            printf("║  ██████  OBJECT WIDTH: %.2f cm  ██████                            ║\n", total_width);
            printf("║                                                                    ║\n");
            printf("╚════════════════════════════════════════════════════════════════════╝\n");
            
            // Return servo to center
            printf("\n[RETURN] Moving servo back to center (90°)...\n");
            servo_set_angle(SERVO_CENTER_ANGLE);
            sleep_ms(500);
            
            printf("\n[READY] Ready for next obstacle detection. Waiting 2 seconds...\n\n");
            sleep_ms(2000);
        }
    }
    
    return 0;
}
