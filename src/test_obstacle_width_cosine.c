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
#define SERVO_STEP_DEGREE               1.0f    // Fast scanning: 1 degree increments
#define SERVO_FINE_STEP_DEGREE          0.5f    // Fine scanning after edge found
#define MEASUREMENT_DELAY_MS            50      // Fast delay: 50ms between measurements
#define EDGE_CONFIRM_DELAY_MS           2000    // 2 second delay for edge confirmation
#define MAX_DETECTION_DISTANCE_CM       100.0f  // Maximum valid echo distance
#define EDGE_CONFIRMATION_SAMPLES       5       // Number of samples to confirm edge detection

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
 * Scan LEFT SIDE: Start at 35°, scan toward 90° until echo detected
 * When echo found, move back in 0.5° increments until edge no longer detected
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
    float edge_angle = 0.0f;
    float edge_distance = 0.0f;
    
    while (current_angle <= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        float distance = ultrasonic_get_distance_cm();
        
        // Check if echo detected (valid distance reading)
        if (distance > 0 && distance <= MAX_DETECTION_DISTANCE_CM) {
            // ECHO FOUND - Take multiple readings to confirm
            printf("[LEFT_SCAN] %.1f° │ ✓ ECHO FOUND! Taking %d confirmation samples...\n", 
                   current_angle, EDGE_CONFIRMATION_SAMPLES);
            
            float samples[EDGE_CONFIRMATION_SAMPLES];
            int valid_samples = 0;
            
            for (int i = 0; i < EDGE_CONFIRMATION_SAMPLES; i++) {
                sleep_ms(100);
                float sample = ultrasonic_get_distance_cm();
                
                if (sample > 0 && sample <= MAX_DETECTION_DISTANCE_CM) {
                    samples[valid_samples] = sample;
                    valid_samples++;
                    printf("[LEFT_SCAN]   Sample %d: %.2f cm\n", i + 1, sample);
                } else {
                    printf("[LEFT_SCAN]   Sample %d: invalid\n", i + 1);
                }
            }
            
            // Check consistency: samples shouldn't vary by more than 10cm
            bool consistent = true;
            float min_sample = 999.0f;
            float max_sample = 0.0f;
            float sum = 0.0f;
            
            for (int i = 0; i < valid_samples; i++) {
                sum += samples[i];
                if (samples[i] < min_sample) min_sample = samples[i];
                if (samples[i] > max_sample) max_sample = samples[i];
            }
            
            float variation = max_sample - min_sample;
            if (variation > 10.0f) {
                consistent = false;
            }
            
            if (valid_samples >= 3 && consistent) {  // Need at least 3 valid samples AND consistency
                float avg_distance = sum / valid_samples;
                printf("[LEFT_SCAN] ✓✓✓ INITIAL EDGE FOUND! Average: %.2f cm (%d valid samples, variation: %.2f cm)\n", 
                       avg_distance, valid_samples, variation);
                
                edge_angle = current_angle;
                edge_distance = avg_distance;
                
                // Now move back in 0.5° increments to find precise edge
                printf("\n[LEFT_SCAN] Moving back in %.1f° increments to refine edge...\n", SERVO_FINE_STEP_DEGREE);
                
                while (current_angle >= SERVO_LEFT_ANGLE) {
                    current_angle -= SERVO_FINE_STEP_DEGREE;
                    servo_set_angle(current_angle);
                    
                    printf("[LEFT_SCAN] Testing at %.1f°, waiting %d seconds...\n", current_angle, EDGE_CONFIRM_DELAY_MS/1000);
                    sleep_ms(EDGE_CONFIRM_DELAY_MS);
                    
                    // Take samples
                    float test_sum = 0.0f;
                    int test_valid = 0;
                    printf("[LEFT_SCAN] Taking %d samples...\n", EDGE_CONFIRMATION_SAMPLES);
                    
                    for (int i = 0; i < EDGE_CONFIRMATION_SAMPLES; i++) {
                        sleep_ms(100);
                        float sample = ultrasonic_get_distance_cm();
                        if (sample > 0 && sample <= MAX_DETECTION_DISTANCE_CM) {
                            test_sum += sample;
                            test_valid++;
                            printf("[LEFT_SCAN]   Sample %d: %.2f cm\n", i + 1, sample);
                        }
                    }
                    
                    if (test_valid >= 3) {
                        float test_avg = test_sum / test_valid;
                        printf("[LEFT_SCAN] Average: %.2f cm - EDGE STILL DETECTED, updating edge position\n", test_avg);
                        edge_angle = current_angle;
                        edge_distance = test_avg;
                    } else {
                        printf("[LEFT_SCAN] ✗ EDGE NO LONGER DETECTED at %.1f°\n", current_angle);
                        printf("[LEFT_SCAN] Final edge angle: %.1f°\n", edge_angle);
                        break;
                    }
                }
                
                float angle_from_center = SERVO_CENTER_ANGLE - edge_angle;
                left_width = calculate_width_component(adjacent, edge_distance);
                
                printf("\n[LEFT_SCAN] FINAL EDGE RESULTS:\n");
                printf("[LEFT_SCAN] Edge angle: %.1f°\n", edge_angle);
                printf("[LEFT_SCAN] Angle from center: %.1f°\n", angle_from_center);
                printf("[LEFT_SCAN] Hypotenuse (measured): %.2f cm\n", edge_distance);
                printf("[LEFT_SCAN] Adjacent (perpendicular): %.2f cm\n", adjacent);
                printf("[LEFT_SCAN] LEFT width component: %.2f cm\n", left_width);
                
                edge_found = true;
                break;
            } else {
                if (!consistent) {
                    printf("[LEFT_SCAN] ✗ Samples inconsistent (variation: %.2f cm > 10cm), continuing scan...\n", variation);
                } else {
                    printf("[LEFT_SCAN] ✗ Not enough valid samples, continuing scan...\n");
                }
            }
        } else {
            printf("[LEFT_SCAN] %.1f° │ No echo (scanning...)\n", current_angle);
        }
        
        current_angle += SERVO_STEP_DEGREE;
    }
    
    if (!edge_found) {
        printf("[LEFT_SCAN] WARNING: No edge found, reached 90°\n");
    }
    
    return left_width;
}

/**
 * Scan RIGHT SIDE: Start at 150°, scan toward 90° until echo detected
 * When echo found, move back in 0.5° increments until edge no longer detected
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
    float edge_angle = 0.0f;
    float edge_distance = 0.0f;
    
    while (current_angle >= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        float distance = ultrasonic_get_distance_cm();
        
        // Check if echo detected (valid distance reading)
        if (distance > 0 && distance <= MAX_DETECTION_DISTANCE_CM) {
            // ECHO FOUND - Take multiple readings to confirm
            printf("[RIGHT_SCAN] %.1f° │ ✓ ECHO FOUND! Taking %d confirmation samples...\n", 
                   current_angle, EDGE_CONFIRMATION_SAMPLES);
            
            float samples[EDGE_CONFIRMATION_SAMPLES];
            int valid_samples = 0;
            
            for (int i = 0; i < EDGE_CONFIRMATION_SAMPLES; i++) {
                sleep_ms(100);
                float sample = ultrasonic_get_distance_cm();
                
                if (sample > 0 && sample <= MAX_DETECTION_DISTANCE_CM) {
                    samples[valid_samples] = sample;
                    valid_samples++;
                    printf("[RIGHT_SCAN]   Sample %d: %.2f cm\n", i + 1, sample);
                } else {
                    printf("[RIGHT_SCAN]   Sample %d: invalid\n", i + 1);
                }
            }
            
            // Check consistency: samples shouldn't vary by more than 10cm
            bool consistent = true;
            float min_sample = 999.0f;
            float max_sample = 0.0f;
            float sum = 0.0f;
            
            for (int i = 0; i < valid_samples; i++) {
                sum += samples[i];
                if (samples[i] < min_sample) min_sample = samples[i];
                if (samples[i] > max_sample) max_sample = samples[i];
            }
            
            float variation = max_sample - min_sample;
            if (variation > 10.0f) {
                consistent = false;
            }
            
            if (valid_samples >= 3 && consistent) {  // Need at least 3 valid samples AND consistency
                float avg_distance = sum / valid_samples;
                printf("[RIGHT_SCAN] ✓✓✓ INITIAL EDGE FOUND! Average: %.2f cm (%d valid samples, variation: %.2f cm)\n", 
                       avg_distance, valid_samples, variation);
                
                edge_angle = current_angle;
                edge_distance = avg_distance;
                
                // Now move back in 0.5° increments to find precise edge
                printf("\n[RIGHT_SCAN] Moving back in %.1f° increments to refine edge...\n", SERVO_FINE_STEP_DEGREE);
                
                while (current_angle <= SERVO_RIGHT_ANGLE) {
                    current_angle += SERVO_FINE_STEP_DEGREE;
                    servo_set_angle(current_angle);
                    
                    printf("[RIGHT_SCAN] Testing at %.1f°, waiting %d seconds...\n", current_angle, EDGE_CONFIRM_DELAY_MS/1000);
                    sleep_ms(EDGE_CONFIRM_DELAY_MS);
                    
                    // Take samples
                    float test_sum = 0.0f;
                    int test_valid = 0;
                    printf("[RIGHT_SCAN] Taking %d samples...\n", EDGE_CONFIRMATION_SAMPLES);
                    
                    for (int i = 0; i < EDGE_CONFIRMATION_SAMPLES; i++) {
                        sleep_ms(100);
                        float sample = ultrasonic_get_distance_cm();
                        if (sample > 0 && sample <= MAX_DETECTION_DISTANCE_CM) {
                            test_sum += sample;
                            test_valid++;
                            printf("[RIGHT_SCAN]   Sample %d: %.2f cm\n", i + 1, sample);
                        }
                    }
                    
                    if (test_valid >= 3) {
                        float test_avg = test_sum / test_valid;
                        printf("[RIGHT_SCAN] Average: %.2f cm - EDGE STILL DETECTED, updating edge position\n", test_avg);
                        edge_angle = current_angle;
                        edge_distance = test_avg;
                    } else {
                        printf("[RIGHT_SCAN] ✗ EDGE NO LONGER DETECTED at %.1f°\n", current_angle);
                        printf("[RIGHT_SCAN] Final edge angle: %.1f°\n", edge_angle);
                        break;
                    }
                }
                
                float angle_from_center = edge_angle - SERVO_CENTER_ANGLE;
                right_width = calculate_width_component(adjacent, edge_distance);
                
                printf("\n[RIGHT_SCAN] FINAL EDGE RESULTS:\n");
                printf("[RIGHT_SCAN] Edge angle: %.1f°\n", edge_angle);
                printf("[RIGHT_SCAN] Angle from center: %.1f°\n", angle_from_center);
                printf("[RIGHT_SCAN] Hypotenuse (measured): %.2f cm\n", edge_distance);
                printf("[RIGHT_SCAN] Adjacent (perpendicular): %.2f cm\n", adjacent);
                printf("[RIGHT_SCAN] RIGHT width component: %.2f cm\n", right_width);
                
                edge_found = true;
                break;
            } else {
                if (!consistent) {
                    printf("[RIGHT_SCAN] ✗ Samples inconsistent (variation: %.2f cm > 10cm), continuing scan...\n", variation);
                } else {
                    printf("[RIGHT_SCAN] ✗ Not enough valid samples, continuing scan...\n");
                }
            }
        } else {
            printf("[RIGHT_SCAN] %.1f° │ No echo (scanning...)\n", current_angle);
        }
        
        current_angle -= SERVO_STEP_DEGREE;
    }
    
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
    printf("║  Fast Scanning: %.0f° steps, %dms delay                            ║\n", SERVO_STEP_DEGREE, MEASUREMENT_DELAY_MS);
    printf("║                                                                    ║\n");
    printf("║  Method: PYTHAGOREAN THEOREM                                      ║\n");
    printf("║  1. Measure perpendicular distance (adjacent) at 90°             ║\n");
    printf("║  2. Scan LEFT (35°→90°) until ECHO detected (hypotenuse)         ║\n");
    printf("║  3. Scan RIGHT (150°→90°) until ECHO detected (hypotenuse)       ║\n");
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
            printf("║  Adjacent (perpendicular):  %.2f cm                                ║\n", adjacent);
            printf("║  Left width component:      %.2f cm                                ║\n", left_width);
            printf("║  Right width component:     %.2f cm                                ║\n", right_width);
            printf("║  ──────────────────────────────────────────────────────────────   ║\n");
            printf("║  TOTAL OBJECT WIDTH:        %.2f cm                                ║\n", total_width);
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
