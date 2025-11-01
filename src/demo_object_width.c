#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "object_width_detection.h"
#include "../ultrasonic/ultrasonic.h"
#include "../servo/servo.h"

// ============== Program State ==============
typedef enum {
    STATE_IDLE,              // Waiting for obstacle
    STATE_DETECTED,          // Obstacle detected, about to measure
    STATE_MEASURING,         // Measuring object width
    STATE_DECISION,          // Deciding what to do with result
    STATE_STOP               // Stop and wait
} program_state_t;

// ============== Main Program ==============
int main(void) {
    stdio_init_all();
    sleep_ms(3000);
    
    printf("\n\n");
    printf("╔════════════════════════════════════════════════════════════════╗\n");
    printf("║        FLEXIBLE OBJECT WIDTH DETECTION SYSTEM                 ║\n");
    printf("║                                                                ║\n");
    printf("║  Features:                                                     ║\n");
    printf("║  • Multiple detection methods (Pythagorean, Cosine, Triangulation)\n");
    printf("║  • Flexible obstacle width handling                            ║\n");
    printf("║  • 30cm stopping distance (configurable)                       ║\n");
    printf("║  • Servo + Ultrasonic edge detection                           ║\n");
    printf("║                                                                ║\n");
    printf("╚════════════════════════════════════════════════════════════════╝\n\n");
    
    // Initialize servo
    printf("[INIT] Initializing servo...\n");
    servo_init();
    sleep_ms(500);
    
    // Initialize ultrasonic
    printf("[INIT] Initializing ultrasonic...\n");
    ultrasonic_init();
    sleep_ms(500);
    
    // Initialize object width detection
    printf("[INIT] Initializing object width detection...\n");
    object_width_init();
    sleep_ms(500);
    
    // Get default config and customize
    object_width_config_t config = object_width_get_default_config();
    
    // Customize for your needs
    config.stopping_distance_cm = 30.0f;           // Stop at 30cm
    config.method = WIDTH_METHOD_COSINE_RULE;       // Use Cosine Rule for best accuracy
    config.perpendicular_samples = 3;
    config.edge_detection_threshold_cm = 5.0f;
    
    printf("[CONFIG] Detection method: COSINE RULE (highest accuracy)\n");
    printf("[CONFIG] Stopping distance: %.1f cm\n", config.stopping_distance_cm);
    
    printf("\n[READY] System ready. Move obstacle in front of robot...\n\n");
    
    program_state_t state = STATE_IDLE;
    int detection_count = 0;
    
    while (1) {
        switch (state) {
            // ===== IDLE STATE: Monitor for obstacles =====
            case STATE_IDLE: {
                servo_set_angle(90.0f);
                float distance = ultrasonic_get_distance_cm();
                
                if (distance > 0.0f) {
                    printf("\n[DETECT] Obstacle detected at %.1f cm\n", distance);
                    state = STATE_DETECTED;
                    detection_count++;
                } else {
                    sleep_ms(200);
                }
                break;
            }
            
            // ===== DETECTED STATE: Prepare for measurement =====
            case STATE_DETECTED: {
                printf("\n[PREPARE] Detection #%d - Preparing measurement...\n", detection_count);
                printf("═══════════════════════════════════════════════════════════════\n");
                sleep_ms(500);
                state = STATE_MEASURING;
                break;
            }
            
            // ===== MEASURING STATE: Find edges and measure width =====
            case STATE_MEASURING: {
                object_width_measurement_t measurement = {0};
                
                if (object_width_find_edges(&config, &measurement)) {
                    printf("[MEASURE] Measurement complete\n");
                    object_width_print_results(&measurement);
                    
                    // Show comparison of methods
                    printf("\n[ANALYSIS] Comparing all available methods...\n");
                    object_width_compare_methods(&measurement);
                    
                    state = STATE_DECISION;
                } else {
                    printf("[ERROR] Failed to measure width\n");
                    state = STATE_IDLE;
                }
                break;
            }
            
            // ===== DECISION STATE: Process result =====
            case STATE_DECISION: {
                printf("[DECISION] Measurement complete. Ready for next obstacle.\n");
                printf("═══════════════════════════════════════════════════════════════\n");
                
                // Clear sensor cache
                printf("[CACHE] Flushing sensor cache...\n");
                for (int i = 0; i < 5; i++) {
                    ultrasonic_get_distance_cm();
                    sleep_ms(100);
                }
                
                sleep_ms(2000);
                state = STATE_IDLE;
                break;
            }
            
            case STATE_STOP: {
                printf("[STOP] System stopped\n");
                while (1) {
                    sleep_ms(1000);
                }
                break;
            }
            
            default:
                state = STATE_IDLE;
                break;
        }
    }
    
    return 0;
}
