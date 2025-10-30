#ifndef DEMO3_OBSTACLE_H
#define DEMO3_OBSTACLE_H

#include <stdint.h>
#include <stdbool.h>

// ============== State Machine ==============
typedef enum {
    STATE_LINE_FOLLOW,      // Following line with IR sensors
    STATE_OBSTACLE_DETECT,  // Obstacle found, stopping
    STATE_SCANNING,         // Servo scanning left/center/right
    STATE_PLANNING,         // Deciding avoidance path
    STATE_AVOIDING,         // Executing turn + movement
    STATE_SEARCHING,        // Looking for line after avoidance
    STATE_RESUME            // Resuming line following
} demo3_state_t;

// ============== Obstacle Data ==============
typedef struct {
    float distance_cm;          // Distance to obstacle (straight ahead)
    float distance_left_cm;     // Distance when servo pointing left
    float distance_center_cm;   // Distance when servo pointing center
    float distance_right_cm;    // Distance when servo pointing right
    float width_cm;             // Estimated obstacle width
    int clear_side;             // -1 = left, +1 = right, 0 = blocked
} obstacle_data_t;

// ============== Configuration ==============
#define OBSTACLE_DETECTION_THRESHOLD_CM   20.0f  // Obstacle within 20cm
#define OBSTACLE_CLEARANCE_MARGIN_CM      5.0f   // Need at least 5cm clearance on sides

// ============== Function Prototypes ==============

/**
 * Initialize obstacle detection system.
 */
void demo3_obstacle_init(void);

/**
 * Perform obstacle scan: move servo to measure distance at left/center/right.
 * Fills obstacle_data_t with measurements.
 * Blocking call (~2 seconds for full scan).
 */
void demo3_scan_obstacle(obstacle_data_t *obstacle);

/**
 * Determine which side is clear for avoidance.
 * Returns: -1 (left clear), +1 (right clear), 0 (blocked/error)
 */
int demo3_choose_avoidance_side(const obstacle_data_t *obstacle);

/**
 * Calculate avoidance maneuver parameters.
 * Returns: turn angle (deg), forward distance (cm), reverse distance (cm)
 */
void demo3_plan_avoidance(const obstacle_data_t *obstacle, 
                          float *turn_angle_deg, 
                          float *forward_dist_cm, 
                          float *parallel_offset_cm);

/**
 * Log obstacle data to stdout.
 */
void demo3_log_obstacle(const obstacle_data_t *obstacle);

#endif // DEMO3_OBSTACLE_H
