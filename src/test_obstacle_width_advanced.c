#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../ultrasonic/ultrasonic.h"
#include "../servo/servo.h"

// ============== Configuration ==============
#define OBSTACLE_THRESHOLD_CM       20.0f   // Obstacle detection threshold
#define PERPENDICULAR_SAMPLES       3       // Number of samples to average perpendicular distance
#define SERVO_CENTER_ANGLE          90.0f   // Center/perpendicular angle
#define SERVO_STEP_DEGREE           0.2f    // 0.2-degree increments for precise measurement
#define SERVO_MIN_ANGLE             10.0f   // Servo minimum angle
#define SERVO_MAX_ANGLE             170.0f  // Servo maximum angle
#define MEASUREMENT_DELAY_US        100000  // Delay between measurements (100ms)
#define MAX_DISTANCE_CHANGE_CM      3.0f    // Maximum expected distance change per 1° step
#define DISTANCE_JUMP_THRESHOLD_CM  10.0f   // Threshold for sudden jump (indicates edge)

// ============== Data Structures ==============
typedef struct {
    float perpendicular_distance_cm;
    float angle_left_edge;          // Angle where left side transitions to clear
    float angle_right_edge;         // Angle where right side transitions to clear
    float hypotenuse_left_cm;       // Distance measurement at left transition
    float hypotenuse_right_cm;      // Distance measurement at right transition
    float horizontal_distance_left; // Width component from left: sqrt(h_left^2 - p^2)
    float horizontal_distance_right;// Width component from right: sqrt(h_right^2 - p^2)
    float total_obstacle_width_cm;  // Total width: left + right
} obstacle_measurement_t;

// ============== Helper Functions ==============

/**
 * Get average perpendicular distance (3 samples)
 */
float get_perpendicular_distance(void) {
    printf("\n  [PERPENDICULAR] Taking %d samples to confirm distance...\n", PERPENDICULAR_SAMPLES);
    
    float sum = 0.0f;
    float min_dist = 999.0f;
    float max_dist = 0.0f;
    
    for (int i = 0; i < PERPENDICULAR_SAMPLES; i++) {
        float distance = ultrasonic_get_distance_cm();
        sum += distance;
        
        if (distance < min_dist) min_dist = distance;
        if (distance > max_dist) max_dist = distance;
        
        printf("    Sample %d: %.2f cm\n", i + 1, distance);
        sleep_us(MEASUREMENT_DELAY_US);
    }
    
    float average = sum / PERPENDICULAR_SAMPLES;
    printf("  [PERPENDICULAR] Average: %.2f cm (Min: %.2f, Max: %.2f)\n", average, min_dist, max_dist);
    
    return average;
}

/**
 * Calculate horizontal distance using Pythagorean theorem
 * horizontal = sqrt(hypotenuse^2 - perpendicular^2)
 */
float calculate_horizontal_distance(float hypotenuse_cm, float perpendicular_cm) {
    if (hypotenuse_cm < perpendicular_cm) {
        printf("    [WARNING] Hypotenuse (%.2f) < perpendicular (%.2f), returning 0\n", 
               hypotenuse_cm, perpendicular_cm);
        return 0.0f;
    }
    
    float h_sq = hypotenuse_cm * hypotenuse_cm;
    float p_sq = perpendicular_cm * perpendicular_cm;
    float horizontal = sqrtf(h_sq - p_sq);
    
    return horizontal;
}

/**
 * Sweep from center (90°) to the left (170°) until obstacle clears
 * Uses dynamic edge detection: monitors distance increases and detects sudden jumps
 * Stores both angle and distance in the measurement structure
 */
void sweep_left_to_find_edge(float perpendicular_distance, obstacle_measurement_t *measurement) {
    printf("\n  [LEFT SWEEP] Starting from 90° sweeping toward 170°...\n");
    printf("  Dynamic detection: looking for sudden jump (>%.1f cm) or timeout\n", DISTANCE_JUMP_THRESHOLD_CM);
    
    float current_angle = SERVO_CENTER_ANGLE;
    float previous_distance = perpendicular_distance;
    measurement->angle_left_edge = SERVO_CENTER_ANGLE;
    measurement->hypotenuse_left_cm = perpendicular_distance;
    bool found_edge = false;
    
    printf("  Angle  │ Distance │ Change  │ Status\n");
    printf("  ───────┼──────────┼─────────┼─────────────────────\n");
    
    while (current_angle <= SERVO_MAX_ANGLE && !found_edge) {
        servo_set_angle(current_angle);
        sleep_us(MEASUREMENT_DELAY_US);
        
        float distance = ultrasonic_get_distance_cm();
        float distance_change = distance - previous_distance;
        
        // Determine status
        const char *status = "";
        bool is_edge = false;
        
        if (distance < 0) {
            // Timeout/invalid reading
            status = "TIMEOUT - EDGE!";
            is_edge = true;
        } else if (distance_change > DISTANCE_JUMP_THRESHOLD_CM) {
            // Sudden jump indicates edge
            status = "JUMP DETECTED - EDGE!";
            is_edge = true;
        } else if (distance_change < -5.0f) {
            // Sudden decrease (shouldn't happen in normal sweep)
            status = "UNEXPECTED DROP";
        } else if (distance_change > MAX_DISTANCE_CHANGE_CM) {
            status = "INCREASING (normal)";
        } else if (distance >= 0 && distance_change >= 0) {
            status = "NORMAL";
        } else {
            status = "ANOMALY";
        }
        
        printf("  %6.1f°│ %8.2f │ %+6.2f  │ %s\n", 
               current_angle, distance, distance_change, status);
        fflush(stdout);
        
        if (is_edge) {
            found_edge = true;
            // Store the PREVIOUS valid measurement (before edge)
            measurement->angle_left_edge = current_angle - SERVO_STEP_DEGREE;
            measurement->hypotenuse_left_cm = previous_distance;
            printf("  \n  ✓ EDGE DETECTED!\n");
            printf("    Edge at angle: %.1f° (last valid before jump)\n", measurement->angle_left_edge);
            printf("    Distance at edge: %.2f cm\n", measurement->hypotenuse_left_cm);
        } else {
            previous_distance = distance;
        }
        
        current_angle += SERVO_STEP_DEGREE;
    }
    
    if (!found_edge) {
        printf("\n  [LEFT SWEEP] Reached maximum angle without clear edge detection.\n");
        measurement->angle_left_edge = SERVO_MAX_ANGLE;
        measurement->hypotenuse_left_cm = previous_distance;
    }
}

/**
 * Sweep from center (90°) to the right (10°) until obstacle clears
 * Uses dynamic edge detection: monitors distance increases and detects sudden jumps
 * Stores both angle and distance in the measurement structure
 */
void sweep_right_to_find_edge(float perpendicular_distance, obstacle_measurement_t *measurement) {
    printf("\n  [RIGHT SWEEP] Starting from 90° sweeping toward 10°...\n");
    printf("  Dynamic detection: looking for sudden jump (>%.1f cm) or timeout\n", DISTANCE_JUMP_THRESHOLD_CM);
    
    float current_angle = SERVO_CENTER_ANGLE;
    float previous_distance = perpendicular_distance;
    measurement->angle_right_edge = SERVO_CENTER_ANGLE;
    measurement->hypotenuse_right_cm = perpendicular_distance;
    bool found_edge = false;
    
    printf("  Angle  │ Distance │ Change  │ Status\n");
    printf("  ───────┼──────────┼─────────┼─────────────────────\n");
    
    while (current_angle >= SERVO_MIN_ANGLE && !found_edge) {
        servo_set_angle(current_angle);
        sleep_us(MEASUREMENT_DELAY_US);
        
        float distance = ultrasonic_get_distance_cm();
        float distance_change = distance - previous_distance;
        
        // Determine status
        const char *status = "";
        bool is_edge = false;
        
        if (distance < 0) {
            // Timeout/invalid reading
            status = "TIMEOUT - EDGE!";
            is_edge = true;
        } else if (distance_change > DISTANCE_JUMP_THRESHOLD_CM) {
            // Sudden jump indicates edge
            status = "JUMP DETECTED - EDGE!";
            is_edge = true;
        } else if (distance_change < -5.0f) {
            // Sudden decrease (shouldn't happen in normal sweep)
            status = "UNEXPECTED DROP";
        } else if (distance_change > MAX_DISTANCE_CHANGE_CM) {
            status = "INCREASING (normal)";
        } else if (distance >= 0 && distance_change >= 0) {
            status = "NORMAL";
        } else {
            status = "ANOMALY";
        }
        
        printf("  %6.1f°│ %8.2f │ %+6.2f  │ %s\n", 
               current_angle, distance, distance_change, status);
        fflush(stdout);
        
        if (is_edge) {
            found_edge = true;
            // Store the PREVIOUS valid measurement (before edge)
            measurement->angle_right_edge = current_angle + SERVO_STEP_DEGREE;
            measurement->hypotenuse_right_cm = previous_distance;
            printf("  \n  ✓ EDGE DETECTED!\n");
            printf("    Edge at angle: %.1f° (last valid before jump)\n", measurement->angle_right_edge);
            printf("    Distance at edge: %.2f cm\n", measurement->hypotenuse_right_cm);
        } else {
            previous_distance = distance;
        }
        
        current_angle -= SERVO_STEP_DEGREE;
    }
    
    if (!found_edge) {
        printf("\n  [RIGHT SWEEP] Reached minimum angle without clear edge detection.\n");
        measurement->angle_right_edge = SERVO_MIN_ANGLE;
        measurement->hypotenuse_right_cm = previous_distance;
    }
}

/**
 * Print final measurement results in a formatted box
 */
void print_measurement_results(obstacle_measurement_t *measurement) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                      OBSTACLE WIDTH MEASUREMENT RESULTS                       ║\n");
    printf("╚════════════════════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    printf("┌─ PERPENDICULAR DISTANCE (at 90°) ──────────────────────────────────────────────┐\n");
    printf("│  Perpendicular Distance: %.2f cm                                                 │\n", 
           measurement->perpendicular_distance_cm);
    printf("└────────────────────────────────────────────────────────────────────────────────┘\n\n");
    
    printf("┌─ LEFT SWEEP MEASUREMENTS ───────────────────────────────────────────────────────┐\n");
    printf("│  Edge Found At:          %.1f° (swept from 90° to 170°)                       ║\n", 
           measurement->angle_left_edge);
    printf("│  Degrees from center:    %.1f° (left side)                                    ║\n", 
           measurement->angle_left_edge - 90.0f);
    printf("│  Hypotenuse Distance:    %.2f cm                                                │\n", 
           measurement->hypotenuse_left_cm);
    printf("│  Horizontal Component:   %.2f cm (width from center to left edge)              │\n", 
           measurement->horizontal_distance_left);
    printf("└────────────────────────────────────────────────────────────────────────────────┘\n\n");
    
    printf("┌─ RIGHT SWEEP MEASUREMENTS ──────────────────────────────────────────────────────┐\n");
    printf("│  Edge Found At:          %.1f° (swept from 90° to 10°)                        ║\n", 
           measurement->angle_right_edge);
    printf("│  Degrees from center:    %.1f° (right side)                                   ║\n", 
           90.0f - measurement->angle_right_edge);
    printf("│  Hypotenuse Distance:    %.2f cm                                                │\n", 
           measurement->hypotenuse_right_cm);
    printf("│  Horizontal Component:   %.2f cm (width from center to right edge)             │\n", 
           measurement->horizontal_distance_right);
    printf("└────────────────────────────────────────────────────────────────────────────────┘\n\n");
    
    printf("╔════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                         CALCULATED OBSTACLE WIDTH                             ║\n");
    printf("╠════════════════════════════════════════════════════════════════════════════════╣\n");
    printf("║                                                                                ║\n");
    printf("║  Width Formula: Left_Horizontal + Right_Horizontal                            ║\n");
    printf("║  Calculation:   %.2f cm + %.2f cm = %.2f cm                                    ║\n", 
           measurement->horizontal_distance_left, 
           measurement->horizontal_distance_right, 
           measurement->total_obstacle_width_cm);
    printf("║                                                                                ║\n");
    printf("║  *** TOTAL OBSTACLE WIDTH: %.2f cm ***                                        ║\n", 
           measurement->total_obstacle_width_cm);
    printf("║                                                                                ║\n");
    printf("╚════════════════════════════════════════════════════════════════════════════════╝\n\n");
}

// ============== Main Program ==============
int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║              OBSTACLE WIDTH MEASUREMENT SYSTEM - TRIGONOMETRIC                ║\n");
    printf("║                                                                                ║\n");
    printf("║  This program will:                                                            ║\n");
    printf("║  1. Detect obstacle at %5.1f cm                                              ║\n", OBSTACLE_THRESHOLD_CM);
    printf("║  2. Confirm perpendicular distance (at 90°)                                   ║\n");
    printf("║  3. Sweep left (90°→170°) to find edge at 1° increments                      ║\n");
    printf("║  4. Sweep right (90°→10°) to find edge at 1° increments                      ║\n");
    printf("║  5. Calculate obstacle width using: sqrt(hyp² - perp²) × 2                  ║\n");
    printf("║                                                                                ║\n");
    printf("╚════════════════════════════════════════════════════════════════════════════════╝\n\n");
    
    // Initialize hardware
    printf("[INIT] Initializing servo motor...\n");
    servo_init();
    printf("[INIT] Servo initialized at 90° (center position)\n");
    servo_set_angle(SERVO_CENTER_ANGLE);
    sleep_ms(500);
    
    printf("[INIT] Initializing ultrasonic sensor...\n");
    ultrasonic_init();
    printf("[INIT] Ultrasonic sensor ready\n");
    sleep_ms(500);
    
    printf("\n[READY] System ready for obstacle measurement\n");
    printf("Position an obstacle in front of the sensor.\n");
    printf("Press ENTER to start monitoring...\n");
    fflush(stdout);
    
    // Monitoring loop
    int detection_count = 0;
    
    while (1) {
        float distance = ultrasonic_get_distance_cm();
        
        // Continuously monitor for obstacle
        if (distance > 0 && distance <= OBSTACLE_THRESHOLD_CM) {
            detection_count++;
            
            printf("\n╔════════════════════════════════════════════════════════════════════════════════╗\n");
            printf("║                    *** OBSTACLE DETECTED ***                                  ║\n");
            printf("║  Detection #%d | Distance: %.2f cm (≤ %.1f cm threshold)                    ║\n", 
                   detection_count, distance, OBSTACLE_THRESHOLD_CM);
            printf("╚════════════════════════════════════════════════════════════════════════════════╝\n");
            
            // Initialize measurement structure
            obstacle_measurement_t measurement = {0};
            
            // Step 1: Get perpendicular distance (at 90 degrees)
            servo_set_angle(SERVO_CENTER_ANGLE);
            sleep_ms(500);
            measurement.perpendicular_distance_cm = get_perpendicular_distance();
            
            // Step 2: Sweep left to find edge
            sweep_left_to_find_edge(measurement.perpendicular_distance_cm, &measurement);
            
            // Step 3: Return to center and sweep right to find edge
            servo_set_angle(SERVO_CENTER_ANGLE);
            sleep_ms(500);
            sweep_right_to_find_edge(measurement.perpendicular_distance_cm, &measurement);
            
            // Step 4: Calculate horizontal distances using Pythagorean theorem
            measurement.horizontal_distance_left = calculate_horizontal_distance(
                measurement.hypotenuse_left_cm, 
                measurement.perpendicular_distance_cm
            );
            
            measurement.horizontal_distance_right = calculate_horizontal_distance(
                measurement.hypotenuse_right_cm, 
                measurement.perpendicular_distance_cm
            );
            
            // Step 5: Calculate total width
            measurement.total_obstacle_width_cm = measurement.horizontal_distance_left + 
                                                   measurement.horizontal_distance_right;
            
            // Print results
            print_measurement_results(&measurement);
            
            // Return servo to center
            servo_set_angle(SERVO_CENTER_ANGLE);
            sleep_ms(500);
            
            printf("[INFO] Measurement complete. Waiting for next obstacle...\n");
            fflush(stdout);
            
            sleep_ms(2000);
        }
        
        sleep_us(100000);  // Check every 100ms
    }
    
    return 0;
}
