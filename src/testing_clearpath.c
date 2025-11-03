#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../ultrasonic/ultrasonic.h"
#include "../servo/servo.h"

// ============== Configuration ==============
#define OBSTACLE_THRESHOLD_CM           25.0f   // Obstacle detection threshold
#define SERVO_CENTER_ANGLE              90.0f   // Center/perpendicular angle
#define SERVO_LEFT_ANGLE                10.0f   // Max left angle
#define SERVO_RIGHT_ANGLE               170.0f  // Max right angle
#define SERVO_SCAN_STEP_DEGREE          1.0f    // Fast scanning: 1 degree increments
#define SERVO_REFINE_STEP_DEGREE        0.5f    // Fine scanning: 0.5 degree increments
#define MEASUREMENT_DELAY_MS            50      // Fast delay: 50ms between measurements
#define REFINEMENT_DELAY_MS             2000    // 2 second delay for refinement
#define MAX_DETECTION_DISTANCE_CM       100.0f  // Maximum valid echo distance
#define EDGE_CONFIRMATION_SAMPLES       5       // Number of samples to confirm edge detection
#define CONSISTENCY_THRESHOLD_CM        10.0f   // Maximum variation for consistent samples
#define MAX_REFINEMENT_ITERATIONS       5       // Maximum refinement iterations

// ============== Data Structures ==============
typedef struct {
    float perpendicular_distance_cm;
    float left_edge_distance_cm;
    float left_width_cm;
    float right_edge_distance_cm;
    float right_width_cm;
    float object_width_cm;
    float clearpath_distance_cm;
    float clearpath_angle;
    int clearpath_side;  // 0 = left, 1 = right
} detection_result_t;

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
 * Calculate clearing angle using trigonometry (COSINE)
 * cos(angle) = adjacent / hypotenuse
 * angle = acos(adjacent / hypotenuse)
 * 
 * @param adjacent: perpendicular distance (the side we know)
 * @param hypotenuse: measured distance at edge
 * @return: clearing angle in degrees
 */
float calculate_clearing_angle(float adjacent, float hypotenuse) {
    if (hypotenuse <= 0.0f || adjacent <= 0.0f) return 0.0f;
    
    float ratio = adjacent / hypotenuse;
    
    // Clamp ratio to valid range for acos
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;
    
    float angle_rad = acosf(ratio);
    float angle_deg = angle_rad * (180.0f / 3.14159265359f);
    
    printf("      [ANGLE_CALC] adjacent=%.2f, hypotenuse=%.2f, ratio=%.4f, angle=%.2f°\n", 
           adjacent, hypotenuse, ratio, angle_deg);
    
    return angle_deg;
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
 * Scan LEFT SIDE: Start at 10°, scan toward 90°
 */
bool scan_left_side(float adjacent, detection_result_t *result) {
    printf("\n[LEFT_SCAN] Waiting 2 seconds before scanning...\n");
    sleep_ms(2000);
    
    printf("[LEFT_SCAN] Starting left scan...\n");
    
    float current_angle = SERVO_LEFT_ANGLE;
    bool edge_found = false;
    float edge_distance = 0.0f;
    
    // Fast scan to find edge
    while (current_angle <= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        float distance = ultrasonic_get_distance_cm();
        
        if (distance > 0 && distance <= MAX_DETECTION_DISTANCE_CM) {
            float avg_distance;
            if (take_consistent_samples(&avg_distance)) {
                printf("[LEFT_SCAN] ✓ EDGE FOUND! Starting refinement...\n");
                edge_found = true;
                edge_distance = avg_distance;
                break;
            }
        }
        
        current_angle += SERVO_SCAN_STEP_DEGREE;
    }
    
    if (!edge_found) {
        printf("[LEFT_SCAN] No edge found!\n");
        return false;
    }
    
    // Refinement: Move back and check continuously (max 5 iterations)
    printf("[LEFT_SCAN] Refining edge...\n");
    
    int refine_count = 0;
    bool refining = true;
    while (refining && refine_count < MAX_REFINEMENT_ITERATIONS && current_angle >= 0.0f) {
        current_angle -= SERVO_REFINE_STEP_DEGREE;
        servo_set_angle(current_angle);
        
        sleep_ms(REFINEMENT_DELAY_MS);
        
        float avg_distance;
        if (take_consistent_samples(&avg_distance)) {
            printf("[LEFT_SCAN] Refine %d: Still edge detected\n", refine_count + 1);
            edge_distance = avg_distance;
            refine_count++;
        } else {
            printf("[LEFT_SCAN] Refine complete after %d iterations\n", refine_count);
            refining = false;
        }
    }
    
    // Store results
    result->left_edge_distance_cm = edge_distance;
    result->left_width_cm = calculate_width_component(adjacent, edge_distance);
    
    return true;
}

/**
 * Scan RIGHT SIDE: Start at 170°, scan toward 90°
 */
bool scan_right_side(float adjacent, detection_result_t *result) {
    printf("\n[RIGHT_SCAN] Waiting 2 seconds before scanning...\n");
    sleep_ms(2000);
    
    printf("[RIGHT_SCAN] Starting right scan...\n");
    
    float current_angle = SERVO_RIGHT_ANGLE;
    bool edge_found = false;
    float edge_distance = 0.0f;
    
    // Fast scan to find edge
    while (current_angle >= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        float distance = ultrasonic_get_distance_cm();
        
        if (distance > 0 && distance <= MAX_DETECTION_DISTANCE_CM) {
            float avg_distance;
            if (take_consistent_samples(&avg_distance)) {
                printf("[RIGHT_SCAN] ✓ EDGE FOUND! Starting refinement...\n");
                edge_found = true;
                edge_distance = avg_distance;
                break;
            }
        }
        
        current_angle -= SERVO_SCAN_STEP_DEGREE;
    }
    
    if (!edge_found) {
        printf("[RIGHT_SCAN] No edge found!\n");
        return false;
    }
    
    // Refinement: Move back and check continuously (max 5 iterations)
    printf("[RIGHT_SCAN] Refining edge...\n");
    
    int refine_count = 0;
    bool refining = true;
    while (refining && refine_count < MAX_REFINEMENT_ITERATIONS && current_angle <= 180.0f) {
        current_angle += SERVO_REFINE_STEP_DEGREE;
        servo_set_angle(current_angle);
        
        sleep_ms(REFINEMENT_DELAY_MS);
        
        float avg_distance;
        if (take_consistent_samples(&avg_distance)) {
            printf("[RIGHT_SCAN] Refine %d: Still edge detected\n", refine_count + 1);
            edge_distance = avg_distance;
            refine_count++;
        } else {
            printf("[RIGHT_SCAN] Refine complete after %d iterations\n", refine_count);
            refining = false;
        }
    }
    
    // Store results
    result->right_edge_distance_cm = edge_distance;
    result->right_width_cm = calculate_width_component(adjacent, edge_distance);
    
    return true;
}

/**
 * Determine clear path based on narrower width
 */
void determine_clear_path(detection_result_t *result) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║                  CLEAR PATH ANALYSIS                              ║\n");
    printf("╠════════════════════════════════════════════════════════════════════╣\n");
    printf("║  Left edge distance:  %.2f cm                                     ║\n", result->left_edge_distance_cm);
    printf("║  Right edge distance: %.2f cm                                     ║\n", result->right_edge_distance_cm);
    printf("║  ──────────────────────────────────────────────────────────────   ║\n");
    
    if (result->left_edge_distance_cm < result->right_edge_distance_cm) {
        // Left side has shorter distance - clearer path
        result->clearpath_side = 0;
        result->clearpath_distance_cm = result->left_width_cm;
        result->clearpath_angle = calculate_clearing_angle(result->perpendicular_distance_cm, 
                                                           result->left_edge_distance_cm);
        
        printf("║  ✓ CLEAR PATH: LEFT (shorter distance)                         ║\n");
    } else {
        // Right side has shorter distance - clearer path
        result->clearpath_side = 1;
        result->clearpath_distance_cm = result->right_width_cm;
        result->clearpath_angle = calculate_clearing_angle(result->perpendicular_distance_cm, 
                                                           result->right_edge_distance_cm);
        
        printf("║  ✓ CLEAR PATH: RIGHT (shorter distance)                        ║\n");
    }
    
    printf("╚════════════════════════════════════════════════════════════════════╝\n");
}

/**
 * Print final detection results
 */
void print_detection_results(detection_result_t *result) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║              OBSTACLE DETECTION RESULTS                           ║\n");
    printf("╠════════════════════════════════════════════════════════════════════╣\n");
    printf("║                                                                    ║\n");
    printf("║  Perpendicular distance:  %.2f cm                                 ║\n", result->perpendicular_distance_cm);
    printf("║  Left edge distance:      %.2f cm                                 ║\n", result->left_edge_distance_cm);
    printf("║  Right edge distance:     %.2f cm                                 ║\n", result->right_edge_distance_cm);
    printf("║  Full obstacle width:     %.2f cm                                 ║\n", result->object_width_cm);
    printf("║                                                                    ║\n");
    printf("║  Clear path turning angle: %.2f°                                  ║\n", result->clearpath_angle);
    printf("║                                                                    ║\n");
    printf("╚════════════════════════════════════════════════════════════════════╝\n\n");
}

// ============== Main Program ==============
int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║        CLEAR PATH DETECTION WITH OBJECT WIDTH MEASUREMENT        ║\n");
    printf("║                                                                    ║\n");
    printf("║  Features:                                                         ║\n");
    printf("║  • Edge refinement with 0.5° increments                           ║\n");
    printf("║  • Pythagorean theorem for width calculation                      ║\n");
    printf("║  • Trigonometry for clearing angle calculation                    ║\n");
    printf("║  • Clear path = narrower width                                    ║\n");
    printf("║                                                                    ║\n");
    printf("║  Stored Data:                                                      ║\n");
    printf("║  - Perpendicular distance                                         ║\n");
    printf("║  - Object width (left + right)                                    ║\n");
    printf("║  - Clear path distance                                            ║\n");
    printf("║  - Clear path angle                                               ║\n");
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
            printf("║           Perpendicular Distance: %.2f cm                          ║\n", 
                   distance_cm);
            printf("╚════════════════════════════════════════════════════════════════════╝\n");
            
            // Initialize detection result structure
            detection_result_t result = {0};
            result.perpendicular_distance_cm = distance_cm;
            
            // Step 1: Scan LEFT side
            if (!scan_left_side(distance_cm, &result)) {
                printf("[ERROR] Failed to detect left edge\n");
                servo_set_angle(SERVO_CENTER_ANGLE);
                sleep_ms(500);
                sleep_ms(2000);
                continue;
            }
            
            // Step 2: Scan RIGHT side
            if (!scan_right_side(distance_cm, &result)) {
                printf("[ERROR] Failed to detect right edge\n");
                servo_set_angle(SERVO_CENTER_ANGLE);
                sleep_ms(500);
                sleep_ms(2000);
                continue;
            }
            
            // Step 3: Calculate total object width
            result.object_width_cm = result.left_width_cm + result.right_width_cm;
            
            // Step 4: Determine clear path
            determine_clear_path(&result);
            
            // Step 5: Print all results
            print_detection_results(&result);
            
            // Return servo to center
            printf("[RETURN] Moving servo back to center (90°)...\n");
            servo_set_angle(SERVO_CENTER_ANGLE);
            sleep_ms(500);
            
            printf("[READY] Ready for next obstacle detection. Waiting 2 seconds...\n\n");
            sleep_ms(2000);
        }
    }
    
    return 0;
}
