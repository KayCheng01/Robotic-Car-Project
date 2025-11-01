#include "object_width_detection.h"
#include "../ultrasonic/ultrasonic.h"
#include "../servo/servo.h"
#include <stdio.h>
#include <math.h>

// ============== Default Configuration ==============
static object_width_config_t default_config = {
    .stopping_distance_cm = 30.0f,
    .perpendicular_samples = 3,
    .servo_left_angle_deg = 170.0f,
    .servo_right_angle_deg = 10.0f,
    .servo_center_angle_deg = 90.0f,
    .servo_step_degree = 0.5f,
    .measurement_delay_ms = 200,
    .edge_detection_threshold_cm = 5.0f,
    .method = WIDTH_METHOD_COSINE_RULE
};

// ============== Implementation ==============

object_width_config_t object_width_get_default_config(void) {
    return default_config;
}

void object_width_init(void) {
    // Servo and ultrasonic are expected to be initialized before this
    printf("[WIDTH_DETECTION] Initialized with method: %d\n", default_config.method);
}

// ============== Method 1: Pythagorean Theorem ==============
float width_pythagorean(float perpendicular_dist_cm, float sweep_distance_cm) {
    if (perpendicular_dist_cm <= 0.0f || sweep_distance_cm <= 0.0f) {
        return 0.0f;
    }
    
    // width = sqrt(hypotenuse² - perpendicular²)
    float h_sq = sweep_distance_cm * sweep_distance_cm;
    float p_sq = perpendicular_dist_cm * perpendicular_dist_cm;
    
    if (h_sq < p_sq) {
        // Hypotenuse should never be less than perpendicular
        return 0.0f;
    }
    
    return sqrtf(h_sq - p_sq);
}

// ============== Method 2: Cosine Rule (Law of Cosines) ==============
float width_cosine_rule(float perpendicular_dist_cm, float sweep_distance_cm, float angle_delta_deg) {
    if (perpendicular_dist_cm <= 0.0f || sweep_distance_cm <= 0.0f) {
        return 0.0f;
    }
    
    // Convert angle to radians
    float angle_rad = angle_delta_deg * 3.14159265358979323846f / 180.0f;
    float cos_angle = cosf(angle_rad);
    
    // Law of cosines: c² = a² + b² - 2ab*cos(C)
    // Where:
    //   a = perpendicular distance
    //   b = sweep distance
    //   C = angle between them
    //   c = object width (what we want)
    
    float a_sq = perpendicular_dist_cm * perpendicular_dist_cm;
    float b_sq = sweep_distance_cm * sweep_distance_cm;
    float width_sq = a_sq + b_sq - 2.0f * perpendicular_dist_cm * sweep_distance_cm * cos_angle;
    
    if (width_sq < 0.0f) {
        return 0.0f;
    }
    
    return sqrtf(width_sq);
}

// ============== Method 3: Triangulation ==============
float width_triangulation(float left_angle_deg, float left_distance_cm,
                         float right_angle_deg, float right_distance_cm,
                         float perpendicular_dist_cm) {
    if (left_distance_cm <= 0.0f || right_distance_cm <= 0.0f) {
        return 0.0f;
    }
    
    // Convert angles to radians
    float left_angle_rad = left_angle_deg * 3.14159265358979323846f / 180.0f;
    float right_angle_rad = right_angle_deg * 3.14159265358979323846f / 180.0f;
    
    // Calculate horizontal positions of edges relative to robot center
    // Assuming servo rotates around a fixed point
    float left_x = left_distance_cm * sinf(left_angle_rad - 90.0f * 3.14159265358979323846f / 180.0f);
    float right_x = right_distance_cm * sinf(right_angle_rad - 90.0f * 3.14159265358979323846f / 180.0f);
    
    // Width is the distance between the two edges
    float width = fabsf(left_x - right_x);
    
    return width;
}

// ============== Edge Detection ==============
static void find_right_edge(object_width_config_t *config, object_width_measurement_t *measurement) {
    printf("\n[RIGHT_SWEEP] Finding right edge from %.0f° toward center...\n", config->servo_right_angle_deg);
    
    float current_angle = config->servo_right_angle_deg;
    bool edge_found = false;
    bool previous_was_far = true;
    
    while (current_angle <= config->servo_center_angle_deg && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms((uint16_t)config->measurement_delay_ms);
        
        float distance = ultrasonic_get_distance_cm();
        bool is_close = (distance > 0.0f) && 
                       (distance <= (measurement->perpendicular_distance_cm + config->edge_detection_threshold_cm));
        
        if (is_close && previous_was_far) {
            edge_found = true;
            measurement->right_edge_angle_deg = current_angle;
            measurement->right_edge_distance_cm = distance;
            printf("[RIGHT_SWEEP] Edge found at %.1f°, distance %.2f cm\n", current_angle, distance);
        }
        
        previous_was_far = !is_close;
        current_angle += config->servo_step_degree;
    }
    
    if (!edge_found) {
        measurement->right_edge_angle_deg = config->servo_center_angle_deg;
        measurement->right_edge_distance_cm = measurement->perpendicular_distance_cm;
        printf("[RIGHT_SWEEP] No edge found, using center measurement\n");
    }
}

static void find_left_edge(object_width_config_t *config, object_width_measurement_t *measurement) {
    printf("\n[LEFT_SWEEP] Moving to max left and finding edge...\n");
    servo_set_angle(config->servo_left_angle_deg);
    sleep_ms(1000);
    
    float current_angle = config->servo_left_angle_deg;
    bool edge_found = false;
    bool previous_was_far = true;
    
    while (current_angle >= config->servo_center_angle_deg && !edge_found) {
        servo_set_angle(current_angle);
        sleep_ms((uint16_t)config->measurement_delay_ms);
        
        float distance = ultrasonic_get_distance_cm();
        bool is_close = (distance > 0.0f) && 
                       (distance <= (measurement->perpendicular_distance_cm + config->edge_detection_threshold_cm));
        
        if (is_close && previous_was_far) {
            edge_found = true;
            measurement->left_edge_angle_deg = current_angle;
            measurement->left_edge_distance_cm = distance;
            printf("[LEFT_SWEEP] Edge found at %.1f°, distance %.2f cm\n", current_angle, distance);
        }
        
        previous_was_far = !is_close;
        current_angle -= config->servo_step_degree;
    }
    
    if (!edge_found) {
        measurement->left_edge_angle_deg = config->servo_center_angle_deg;
        measurement->left_edge_distance_cm = measurement->perpendicular_distance_cm;
        printf("[LEFT_SWEEP] No edge found, using center measurement\n");
    }
}

// ============== Main Detection Functions ==============

bool object_width_find_edges(object_width_config_t *config, object_width_measurement_t *measurement) {
    if (!config || !measurement) {
        return false;
    }
    
    // Get perpendicular distance at center
    printf("\n[PERPENDICULAR] Taking %.0f samples at center (90°)...\n", config->perpendicular_samples);
    servo_set_angle(config->servo_center_angle_deg);
    sleep_ms(500);
    
    float sum = 0.0f;
    for (int i = 0; i < (int)config->perpendicular_samples; i++) {
        float distance = ultrasonic_get_distance_cm();
        if (distance > 0.0f) {
            sum += distance;
            printf("  Sample %d: %.2f cm\n", i + 1, distance);
        }
        sleep_ms((uint16_t)config->measurement_delay_ms);
    }
    
    measurement->perpendicular_distance_cm = sum / config->perpendicular_samples;
    printf("[PERPENDICULAR] Average: %.2f cm\n", measurement->perpendicular_distance_cm);
    
    // Find right and left edges
    find_right_edge(config, measurement);
    find_left_edge(config, measurement);
    
    // Calculate width components
    float right_angle_diff = config->servo_center_angle_deg - measurement->right_edge_angle_deg;
    float left_angle_diff = measurement->left_edge_angle_deg - config->servo_center_angle_deg;
    
    switch (config->method) {
        case WIDTH_METHOD_PYTHAGOREAN:
            measurement->right_width_component_cm = width_pythagorean(
                measurement->perpendicular_distance_cm,
                measurement->right_edge_distance_cm
            );
            measurement->left_width_component_cm = width_pythagorean(
                measurement->perpendicular_distance_cm,
                measurement->left_edge_distance_cm
            );
            break;
            
        case WIDTH_METHOD_COSINE_RULE:
            measurement->right_width_component_cm = width_cosine_rule(
                measurement->perpendicular_distance_cm,
                measurement->right_edge_distance_cm,
                right_angle_diff
            );
            measurement->left_width_component_cm = width_cosine_rule(
                measurement->perpendicular_distance_cm,
                measurement->left_edge_distance_cm,
                left_angle_diff
            );
            break;
            
        case WIDTH_METHOD_TRIANGULATION:
            measurement->object_width_cm = width_triangulation(
                measurement->left_edge_angle_deg,
                measurement->left_edge_distance_cm,
                measurement->right_edge_angle_deg,
                measurement->right_edge_distance_cm,
                measurement->perpendicular_distance_cm
            );
            measurement->right_width_component_cm = measurement->object_width_cm / 2.0f;
            measurement->left_width_component_cm = measurement->object_width_cm / 2.0f;
            break;
            
        default:
            break;
    }
    
    if (config->method != WIDTH_METHOD_TRIANGULATION) {
        measurement->object_width_cm = measurement->right_width_component_cm + measurement->left_width_component_cm;
    }
    
    measurement->center_angle_deg = left_angle_diff + right_angle_diff;
    measurement->method_used = config->method;
    
    return true;
}

bool object_width_measure(object_width_config_t *config, object_width_measurement_t *measurement) {
    if (!config || !measurement) {
        return false;
    }
    
    // Check for obstacle
    if (!object_width_obstacle_detected(config)) {
        printf("[MEASURE] No obstacle detected\n");
        return false;
    }
    
    // Find edges and measure
    bool success = object_width_find_edges(config, measurement);
    
    if (success) {
        object_width_print_results(measurement);
    }
    
    return success;
}

bool object_width_obstacle_detected(object_width_config_t *config) {
    if (!config) {
        return false;
    }
    
    servo_set_angle(config->servo_center_angle_deg);
    sleep_ms(100);
    
    float distance = ultrasonic_get_distance_cm();
    return (distance > 0.0f);
}

// ============== Printing & Comparison ==============

void object_width_print_results(object_width_measurement_t *measurement) {
    if (!measurement) {
        return;
    }
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════╗\n");
    printf("║              OBJECT WIDTH MEASUREMENT RESULTS                 ║\n");
    printf("╚════════════════════════════════════════════════════════════════╝\n\n");
    
    const char *method_name = "UNKNOWN";
    switch (measurement->method_used) {
        case WIDTH_METHOD_PYTHAGOREAN:
            method_name = "PYTHAGOREAN THEOREM";
            break;
        case WIDTH_METHOD_COSINE_RULE:
            method_name = "COSINE RULE (Law of Cosines)";
            break;
        case WIDTH_METHOD_TRIANGULATION:
            method_name = "TRIANGULATION";
            break;
        default:
            break;
    }
    
    printf("METHOD USED: %s\n\n", method_name);
    
    printf("PERPENDICULAR DISTANCE (at 90°):\n");
    printf("  Distance: %.2f cm\n\n", measurement->perpendicular_distance_cm);
    
    printf("RIGHT EDGE:\n");
    printf("  Angle: %.1f°\n", measurement->right_edge_angle_deg);
    printf("  Distance: %.2f cm\n", measurement->right_edge_distance_cm);
    printf("  Width component: %.2f cm\n\n", measurement->right_width_component_cm);
    
    printf("LEFT EDGE:\n");
    printf("  Angle: %.1f°\n", measurement->left_edge_angle_deg);
    printf("  Distance: %.2f cm\n", measurement->left_edge_distance_cm);
    printf("  Width component: %.2f cm\n\n", measurement->left_width_component_cm);
    
    printf("TOTAL ANGLE SPAN: %.1f°\n", measurement->center_angle_deg);
    printf("═══════════════════════════════════════════════════════════════\n");
    printf("CALCULATED OBJECT WIDTH: %.2f cm\n", measurement->object_width_cm);
    printf("╚════════════════════════════════════════════════════════════════╝\n\n");
}

void object_width_compare_methods(object_width_measurement_t *measurement) {
    if (!measurement) {
        return;
    }
    
    float angle_delta_right = 90.0f - measurement->right_edge_angle_deg;
    float angle_delta_left = measurement->left_edge_angle_deg - 90.0f;
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════╗\n");
    printf("║          COMPARING ALL DETECTION METHODS                       ║\n");
    printf("╚════════════════════════════════════════════════════════════════╝\n\n");
    
    // Method 1: Pythagorean
    float right_pyth = width_pythagorean(
        measurement->perpendicular_distance_cm,
        measurement->right_edge_distance_cm
    );
    float left_pyth = width_pythagorean(
        measurement->perpendicular_distance_cm,
        measurement->left_edge_distance_cm
    );
    float total_pyth = right_pyth + left_pyth;
    
    printf("1. PYTHAGOREAN THEOREM: %.2f cm\n", total_pyth);
    printf("   Right: %.2f cm, Left: %.2f cm\n\n", right_pyth, left_pyth);
    
    // Method 2: Cosine Rule
    float right_cos = width_cosine_rule(
        measurement->perpendicular_distance_cm,
        measurement->right_edge_distance_cm,
        angle_delta_right
    );
    float left_cos = width_cosine_rule(
        measurement->perpendicular_distance_cm,
        measurement->left_edge_distance_cm,
        angle_delta_left
    );
    float total_cos = right_cos + left_cos;
    
    printf("2. COSINE RULE (Best Accuracy): %.2f cm\n", total_cos);
    printf("   Right: %.2f cm, Left: %.2f cm\n\n", right_cos, left_cos);
    
    // Method 3: Triangulation
    float total_tri = width_triangulation(
        measurement->left_edge_angle_deg,
        measurement->left_edge_distance_cm,
        measurement->right_edge_angle_deg,
        measurement->right_edge_distance_cm,
        measurement->perpendicular_distance_cm
    );
    
    printf("3. TRIANGULATION (Most Robust): %.2f cm\n\n", total_tri);
    
    // Differences
    printf("DIFFERENCE ANALYSIS:\n");
    printf("  Cosine vs Pythagorean: %.2f cm (%.1f%%)\n", 
           fabsf(total_cos - total_pyth), 
           fabsf(total_cos - total_pyth) / total_pyth * 100.0f);
    printf("  Triangulation vs Cosine: %.2f cm (%.1f%%)\n", 
           fabsf(total_tri - total_cos),
           fabsf(total_tri - total_cos) / total_cos * 100.0f);
    printf("╚════════════════════════════════════════════════════════════════╝\n\n");
}
