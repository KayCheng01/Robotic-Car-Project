#include "demo3_obstacle.h"
#include "ultrasonic.h"
#include "servo.h"
#include <stdio.h>
#include <math.h>

// ============== Initialization ==============

void demo3_obstacle_init(void) {
    ultrasonic_init();
    servo_init();
    printf("[DEMO3] Obstacle detection initialized\n");
}

// ============== Obstacle Scanning ==============

void demo3_scan_obstacle(obstacle_data_t *obstacle) {
    if (!obstacle) return;
    
    printf("[DEMO3] ========== OBSTACLE SCAN START ==========\n");
    
    // Measure distance straight ahead (servo at center)
    printf("[DEMO3] Scanning: CENTER (90°)...\n");
    servo_move_center(300);  // Move and stabilize
    obstacle->distance_center_cm = ultrasonic_get_distance_cm();
    printf("[DEMO3]   Center distance: %.1f cm\n", (double)obstacle->distance_center_cm);
    
    // Measure distance to the left
    printf("[DEMO3] Scanning: LEFT (0°)...\n");
    servo_move_left(300);
    obstacle->distance_left_cm = ultrasonic_get_distance_cm();
    printf("[DEMO3]   Left distance: %.1f cm\n", (double)obstacle->distance_left_cm);
    
    // Measure distance to the right
    printf("[DEMO3] Scanning: RIGHT (180°)...\n");
    servo_move_right(300);
    obstacle->distance_right_cm = ultrasonic_get_distance_cm();
    printf("[DEMO3]   Right distance: %.1f cm\n", (double)obstacle->distance_right_cm);
    
    // Return servo to center
    servo_move_center(300);
    
    // Estimate obstacle width
    // Simple heuristic: max distance on sides - center distance
    float max_side = fmaxf(obstacle->distance_left_cm, obstacle->distance_right_cm);
    obstacle->width_cm = fmaxf(0.0f, max_side - obstacle->distance_center_cm);
    
    printf("[DEMO3] ========== OBSTACLE SCAN COMPLETE ==========\n");
    printf("[DEMO3] Estimated width: %.1f cm\n", (double)obstacle->width_cm);
}

// ============== Avoidance Side Selection ==============

int demo3_choose_avoidance_side(const obstacle_data_t *obstacle) {
    if (!obstacle) return 0;
    
    // Check if left side is clear (distance > threshold + margin)
    bool left_clear = (obstacle->distance_left_cm > 
                       OBSTACLE_DETECTION_THRESHOLD_CM + OBSTACLE_CLEARANCE_MARGIN_CM);
    
    // Check if right side is clear
    bool right_clear = (obstacle->distance_right_cm > 
                        OBSTACLE_DETECTION_THRESHOLD_CM + OBSTACLE_CLEARANCE_MARGIN_CM);
    
    // Prefer the side with more clearance
    if (left_clear && right_clear) {
        // Both clear: choose side with more space
        if (obstacle->distance_left_cm > obstacle->distance_right_cm) {
            printf("[DEMO3] Both sides clear, choosing LEFT (%.1f > %.1f)\n",
                   (double)obstacle->distance_left_cm, 
                   (double)obstacle->distance_right_cm);
            return -1;
        } else {
            printf("[DEMO3] Both sides clear, choosing RIGHT (%.1f > %.1f)\n",
                   (double)obstacle->distance_right_cm,
                   (double)obstacle->distance_left_cm);
            return 1;
        }
    }
    
    // One side clear
    if (left_clear) {
        printf("[DEMO3] Choosing LEFT side (left=%.1f, right=%.1f)\n",
               (double)obstacle->distance_left_cm,
               (double)obstacle->distance_right_cm);
        return -1;
    }
    if (right_clear) {
        printf("[DEMO3] Choosing RIGHT side (left=%.1f, right=%.1f)\n",
               (double)obstacle->distance_left_cm,
               (double)obstacle->distance_right_cm);
        return 1;
    }
    
    // Both blocked
    printf("[DEMO3] ERROR: Both sides blocked! (left=%.1f, right=%.1f)\n",
           (double)obstacle->distance_left_cm,
           (double)obstacle->distance_right_cm);
    return 0;
}

// ============== Avoidance Path Planning ==============

void demo3_plan_avoidance(const obstacle_data_t *obstacle,
                          float *turn_angle_deg,
                          float *forward_dist_cm,
                          float *parallel_offset_cm) {
    if (!obstacle || !turn_angle_deg || !forward_dist_cm || !parallel_offset_cm) {
        return;
    }
    
    // ---- Determine turn direction and magnitude ----
    int side = demo3_choose_avoidance_side(obstacle);
    
    // Turn roughly 45° toward clear side (adjust based on testing)
    // and move forward by obstacle width + margin
    if (side == -1) {
        // Left is clear: turn left ~45°
        *turn_angle_deg = 45.0f;
    } else if (side == 1) {
        // Right is clear: turn right ~-45°
        *turn_angle_deg = -45.0f;
    } else {
        // Blocked: default to right turn (conservative)
        *turn_angle_deg = -45.0f;
    }
    
    // Forward distance: obstacle width + clearance margin + a bit extra
    *forward_dist_cm = obstacle->width_cm + OBSTACLE_CLEARANCE_MARGIN_CM + 10.0f;
    
    // Parallel offset: distance to move parallel to original path after clearing obstacle
    // This helps get back in line with the original track
    *parallel_offset_cm = 15.0f;  // Tune based on track width
    
    printf("[DEMO3] Avoidance plan: turn=%.1f°, forward=%.1f cm, parallel=%.1f cm\n",
           (double)*turn_angle_deg, (double)*forward_dist_cm, (double)*parallel_offset_cm);
}

// ============== Logging ==============

void demo3_log_obstacle(const obstacle_data_t *obstacle) {
    if (!obstacle) return;
    
    printf("[DEMO3:LOG] Obstacle Data:\n");
    printf("[DEMO3:LOG]   Center: %.1f cm\n", (double)obstacle->distance_center_cm);
    printf("[DEMO3:LOG]   Left:   %.1f cm\n", (double)obstacle->distance_left_cm);
    printf("[DEMO3:LOG]   Right:  %.1f cm\n", (double)obstacle->distance_right_cm);
    printf("[DEMO3:LOG]   Width:  %.1f cm\n", (double)obstacle->width_cm);
    printf("[DEMO3:LOG]   Side:   %s\n",
           obstacle->clear_side == -1 ? "LEFT" :
           obstacle->clear_side == 1  ? "RIGHT" : "UNKNOWN");
}
