#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../ultrasonic/ultrasonic.h"
#include "../servo/servo.h"

// ============== Configuration ==============
#define OBSTACLE_THRESHOLD_CM           20.0f   // Obstacle detection threshold
#define PERPENDICULAR_SAMPLES           3       // Number of samples to average perpendicular distance
#define SERVO_CENTER_ANGLE              90.0f   // Center/perpendicular angle
#define SERVO_RIGHT_ANGLE               10.0f   // Max right angle
#define SERVO_LEFT_ANGLE                170.0f  // Max left angle
#define SERVO_STEP_DEGREE               1.0f    // 1-degree increments
#define MEASUREMENT_DELAY_MS            200     // Delay between measurements (200ms)
#define EDGE_DETECTION_THRESHOLD_CM     5.0f    // When distance is within this of perpendicular, it's considered edge

// ============== Data Structures ==============
typedef struct {
    float perpendicular_distance_cm;
    float right_edge_angle;         // Angle at right edge
    float right_edge_distance_cm;   // Distance at right edge
    float left_edge_angle;          // Angle at left edge
    float left_edge_distance_cm;    // Distance at left edge
    float object_width_cm;          // Calculated object width
} obstacle_measurement_t;

/**
 * Get perpendicular distance (at 90°)
 */
float get_perpendicular_distance(void) {
    printf("\n[PERPENDICULAR] Taking %d samples to confirm distance...\n", PERPENDICULAR_SAMPLES);
    
    float sum = 0.0f;
    float min_dist = 999.0f;
    float max_dist = 0.0f;
    
    for (int i = 0; i < PERPENDICULAR_SAMPLES; i++) {
        float distance = ultrasonic_get_distance_cm();
        sum += distance;
        
        if (distance < min_dist) min_dist = distance;
        if (distance > max_dist) max_dist = distance;
        
        printf("  Sample %d: %.2f cm\n", i + 1, distance);
        sleep_ms(MEASUREMENT_DELAY_MS);
    }
    
    float average = sum / PERPENDICULAR_SAMPLES;
    printf("[PERPENDICULAR] Average: %.2f cm (Min: %.2f, Max: %.2f)\n", average, min_dist, max_dist);
    
    return average;
}

/**
 * Calculate object width using cosine rule
 * c² = a² + b² - 2ab·cos(C)
 * 
 * @param perp_dist: perpendicular distance at 90°
 * @param angle_dist: distance at sweep angle
 * @param angle_deg: angle difference from 90°
 * @return: object width component
 */
float calculate_width_cosine_rule(float perp_dist, float angle_dist, float angle_deg) {
    if (perp_dist <= 0.0f || angle_dist <= 0.0f) return 0.0f;
    
    // Convert angle to radians
    float angle_rad = angle_deg * (M_PI / 180.0f);
    
    // Apply cosine rule: c² = a² + b² - 2ab·cos(C)
    float a = perp_dist;
    float b = angle_dist;
    float cos_c = cosf(angle_rad);
    
    float c_squared = (a * a) + (b * b) - (2.0f * a * b * cos_c);
    
    if (c_squared < 0.0f) return 0.0f;
    
    return sqrtf(c_squared);
}

/**
 * Find right edge: start from max right (10°) and sweep toward center (90°)
 * Stop when distance transitions to value close to perpendicular distance
 */
void find_right_edge(float perpendicular_distance, obstacle_measurement_t *measurement) {
    printf("\n[RIGHT_SWEEP] Starting from MAX RIGHT (%.0f°) sweeping toward CENTER (90°)...\n", SERVO_RIGHT_ANGLE);
    printf("[RIGHT_SWEEP] Looking for edge (transition to distance close to %.2f cm)\n", perpendicular_distance);
    
    float current_angle = SERVO_RIGHT_ANGLE;
    bool edge_found = false;
    float previous_distance = 999.0f;
    bool previous_was_far = true;  // Track if previous reading was far/timeout
    
    printf("[RIGHT_SWEEP] Angle  │ Distance │ Status\n");
    printf("[RIGHT_SWEEP] ───────┼──────────┼──────────────────\n");
    
    while (current_angle <= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        float distance = ultrasonic_get_distance_cm();
        const char *status = "";
        bool is_close = false;
        
        if (distance < 0) {
            status = "timeout";
        } else if (distance <= (perpendicular_distance + EDGE_DETECTION_THRESHOLD_CM)) {
            status = "✓ close to perp";
            is_close = true;
        } else {
            status = "far";
        }
        
        // Detect edge: transition from far/timeout to close
        if (is_close && previous_was_far) {
            status = "✓✓✓ EDGE FOUND!";
            edge_found = true;
            measurement->right_edge_angle = current_angle;
            measurement->right_edge_distance_cm = distance;
            printf("[RIGHT_SWEEP] %7.1f° │ %8.2f │ %s\n", current_angle, distance, status);
            break;
        }
        
        printf("[RIGHT_SWEEP] %7.1f° │ %8.2f │ %s\n", current_angle, distance, status);
        previous_distance = distance;
        previous_was_far = !is_close;  // Update if we were far
        current_angle += SERVO_STEP_DEGREE;
    }
    
    if (!edge_found) {
        printf("[RIGHT_SWEEP] No edge found - using last measurement\n");
        measurement->right_edge_angle = SERVO_CENTER_ANGLE;
        measurement->right_edge_distance_cm = previous_distance;
    }
}

/**
 * Find left edge: move to max left (170°), wait 1s, then sweep toward center (90°)
 * Stop when distance transitions to value close to perpendicular distance
 */
void find_left_edge(float perpendicular_distance, obstacle_measurement_t *measurement) {
    printf("\n[LEFT_SETUP] Moving to MAX LEFT (%.0f°)...\n", SERVO_LEFT_ANGLE);
    servo_set_angle(SERVO_LEFT_ANGLE);
    sleep_ms(1000);  // Wait 1 second
    
    printf("[LEFT_SWEEP] Starting from MAX LEFT (%.0f°) sweeping toward CENTER (90°)...\n", SERVO_LEFT_ANGLE);
    printf("[LEFT_SWEEP] Looking for edge (transition to distance close to %.2f cm)\n", perpendicular_distance);
    
    float current_angle = SERVO_LEFT_ANGLE;
    bool edge_found = false;
    float previous_distance = 999.0f;
    bool previous_was_far = true;  // Track if previous reading was far/timeout
    
    printf("[LEFT_SWEEP] Angle  │ Distance │ Status\n");
    printf("[LEFT_SWEEP] ───────┼──────────┼──────────────────\n");
    
    while (current_angle >= SERVO_CENTER_ANGLE && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms(MEASUREMENT_DELAY_MS);
        
        float distance = ultrasonic_get_distance_cm();
        const char *status = "";
        bool is_close = false;
        
        if (distance < 0) {
            status = "timeout";
        } else if (distance <= (perpendicular_distance + EDGE_DETECTION_THRESHOLD_CM)) {
            status = "✓ close to perp";
            is_close = true;
        } else {
            status = "far";
        }
        
        // Detect edge: transition from far/timeout to close
        if (is_close && previous_was_far) {
            status = "✓✓✓ EDGE FOUND!";
            edge_found = true;
            measurement->left_edge_angle = current_angle;
            measurement->left_edge_distance_cm = distance;
            printf("[LEFT_SWEEP] %7.1f° │ %8.2f │ %s\n", current_angle, distance, status);
            break;
        }
        
        printf("[LEFT_SWEEP] %7.1f° │ %8.2f │ %s\n", current_angle, distance, status);
        previous_distance = distance;
        previous_was_far = !is_close;  // Update if we were far
        current_angle -= SERVO_STEP_DEGREE;
    }
    
    if (!edge_found) {
        printf("[LEFT_SWEEP] No edge found - using last measurement\n");
        measurement->left_edge_angle = SERVO_CENTER_ANGLE;
        measurement->left_edge_distance_cm = previous_distance;
    }
}

/**
 * Print final measurement results
 */
void print_measurement_results(obstacle_measurement_t *measurement) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║                 OBSTACLE WIDTH MEASUREMENT RESULTS                ║\n");
    printf("╚════════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("PERPENDICULAR DISTANCE (at 90°):\n");
    printf("  Distance: %.2f cm\n\n", measurement->perpendicular_distance_cm);
    
    printf("RIGHT EDGE (from max right 10°):\n");
    printf("  Angle: %.1f°\n", measurement->right_edge_angle);
    printf("  Distance: %.2f cm\n", measurement->right_edge_distance_cm);
    printf("  Angle difference from 90°: %.1f°\n\n", 
           90.0f - measurement->right_edge_angle);
    
    printf("LEFT EDGE (from max left 170°):\n");
    printf("  Angle: %.1f°\n", measurement->left_edge_angle);
    printf("  Distance: %.2f cm\n", measurement->left_edge_distance_cm);
    printf("  Angle difference from 90°: %.1f°\n\n", 
           measurement->left_edge_angle - 90.0f);
    
    printf("CALCULATED OBJECT WIDTH (using Cosine Rule):\n");
    printf("  Formula: c² = a² + b² - 2ab·cos(C)\n");
    printf("  Where a = perp dist, b = edge dist, C = angle diff\n\n");
    
    // Calculate right width component
    float right_angle_diff = 90.0f - measurement->right_edge_angle;
    float right_width = calculate_width_cosine_rule(
        measurement->perpendicular_distance_cm,
        measurement->right_edge_distance_cm,
        right_angle_diff
    );
    
    // Calculate left width component
    float left_angle_diff = measurement->left_edge_angle - 90.0f;
    float left_width = calculate_width_cosine_rule(
        measurement->perpendicular_distance_cm,
        measurement->left_edge_distance_cm,
        left_angle_diff
    );
    
    printf("  RIGHT component: %.2f cm (angle: %.1f°)\n", right_width, right_angle_diff);
    printf("  LEFT component:  %.2f cm (angle: %.1f°)\n", left_width, left_angle_diff);
    printf("  ═══════════════════════════════════\n");
    printf("  TOTAL WIDTH:     %.2f cm\n", right_width + left_width);
    printf("╚════════════════════════════════════════════════════════════════════╝\n\n");
    
    measurement->object_width_cm = right_width + left_width;
}

// ============== Main Program ==============
int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════════════════════════════╗\n");
    printf("║         OBSTACLE WIDTH MEASUREMENT - COSINE RULE METHOD           ║\n");
    printf("║                                                                    ║\n");
    printf("║  This program will:                                                ║\n");
    printf("║  1. Detect obstacle at ≤20cm (perpendicular)                       ║\n");
    printf("║  2. From MAX RIGHT (10°), find first edge → record angle & dist   ║\n");
    printf("║  3. Move to MAX LEFT (170°), wait 1s                              ║\n");
    printf("║  4. From MAX LEFT, find first edge → record angle & dist          ║\n");
    printf("║  5. Calculate object width using cosine rule                      ║\n");
    printf("║                                                                    ║\n");
    printf("║  Edge detection: transition from timeout/far to close distance    ║\n");
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
        // Measure distance at center
        float distance_cm = ultrasonic_get_distance_cm();
        
        printf("[MONITOR] Center (90°): %.1f cm | Status: ", (double)distance_cm);
        
        if (distance_cm < 0.0f) {
            printf("timeout/out of range\n");
        } else if (distance_cm <= OBSTACLE_THRESHOLD_CM) {
            printf("✓ OBSTACLE DETECTED\n");
        } else {
            printf("clear\n");
        }
        
        fflush(stdout);
        sleep_ms(500);
        
        // Check if obstacle detected
        if (distance_cm <= OBSTACLE_THRESHOLD_CM && distance_cm > 0.0f) {
            printf("\n");
            printf("╔════════════════════════════════════════════════════════════════════╗\n");
            printf("║                  *** OBSTACLE DETECTED ***                         ║\n");
            printf("║              Distance: %.2f cm (≤ %.1f cm threshold)               ║\n", 
                   distance_cm, OBSTACLE_THRESHOLD_CM);
            printf("╚════════════════════════════════════════════════════════════════════╝\n\n");
            
            // Initialize measurement structure
            obstacle_measurement_t measurement = {0};
            
            // Step 1: Get perpendicular distance
            servo_set_angle(SERVO_CENTER_ANGLE);
            sleep_ms(500);
            measurement.perpendicular_distance_cm = get_perpendicular_distance();
            
            // Step 2: Find right edge (from max right to center)
            find_right_edge(measurement.perpendicular_distance_cm, &measurement);
            
            // Step 3: Find left edge (to max left, wait 1s, then toward center)
            find_left_edge(measurement.perpendicular_distance_cm, &measurement);
            
            // Step 4: Print results
            print_measurement_results(&measurement);
            
            // Return servo to center
            servo_set_angle(SERVO_CENTER_ANGLE);
            sleep_ms(500);
            
            printf("[INFO] Measurement complete. Clearing cache and waiting for next obstacle...\n");
            
            // Clear cache: take a few measurements to flush old data
            printf("[CACHE_CLEAR] Flushing sensor cache...\n");
            for (int i = 0; i < 5; i++) {
                ultrasonic_get_distance_cm();
                sleep_ms(100);
            }
            printf("[CACHE_CLEAR] Cache cleared. Ready for next detection.\n");
            sleep_ms(2000);
        }
    }
    
    return 0;
}
