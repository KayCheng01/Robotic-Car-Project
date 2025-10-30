#include "demo3_line_recovery.h"
#include "ir.h"
#include "motor.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <stdbool.h>

// ============== Helper: IR Line Detection ==============

bool demo3_line_detected(void) {
    // Line detected if BOTH IR sensors see black, or configured logic
    int left = ir_left_is_black();
    int right = ir_right_is_black();
    
    // Line found if at least one sensor detects it
    // (more lenient) or both (stricter)
    // For demo, use: at least one sensor
    return (left || right);
}

// ============== Line Search Pattern ==============

/**
 * Spiral search pattern:
 * 1. Forward + right turn
 * 2. Forward + left turn
 * 3. Increase search radius
 */
bool demo3_search_for_line(void) {
    printf("[DEMO3:RECOVERY] Starting line search...\n");
    
    absolute_time_t search_start = get_absolute_time();
    absolute_time_t search_timeout = make_timeout_time_ms(LINE_RECOVERY_TIMEOUT_MS);
    
    int search_phase = 0;  // 0=right, 1=left, 2=back/escalate
    
    while (!time_reached(search_timeout)) {
        // Quick check: found line?
        if (demo3_line_detected()) {
            printf("[DEMO3:RECOVERY] Line FOUND! Detected after %.0f ms\n",
                   (double)absolute_time_diff_us(search_start, get_absolute_time()) / 1000.0);
            stop_motor_manual();  // Stop all motion
            sleep_ms(200);
            return true;
        }
        
        // Search pattern
        switch (search_phase % 3) {
            case 0:
                // Turn right and move forward
                printf("[DEMO3:RECOVERY] Phase: RIGHT SWEEP\n");
                turn_motor_manual(1, 30.0f, LINE_SEARCH_SPEED_PWM, LINE_SEARCH_SPEED_PWM);  // Right 30°
                forward_motor_manual(LINE_SEARCH_SPEED_PWM, LINE_SEARCH_SPEED_PWM);
                sleep_ms(800);
                break;
                
            case 1:
                // Turn left and move forward
                printf("[DEMO3:RECOVERY] Phase: LEFT SWEEP\n");
                turn_motor_manual(0, 60.0f, LINE_SEARCH_SPEED_PWM, LINE_SEARCH_SPEED_PWM);  // Left 60°
                forward_motor_manual(LINE_SEARCH_SPEED_PWM, LINE_SEARCH_SPEED_PWM);
                sleep_ms(800);
                break;
                
            case 2:
                // Back to center and escalate
                printf("[DEMO3:RECOVERY] Phase: CENTER + ESCALATE\n");
                turn_motor_manual(1, 30.0f, LINE_SEARCH_SPEED_PWM, LINE_SEARCH_SPEED_PWM);  // Back to center
                reverse_motor_manual(LINE_SEARCH_SPEED_PWM / 2, LINE_SEARCH_SPEED_PWM / 2);
                sleep_ms(600);
                break;
        }
        
        search_phase++;
    }
    
    printf("[DEMO3:RECOVERY] Line search TIMEOUT after %.0f ms\n",
           (double)LINE_RECOVERY_TIMEOUT_MS);
    stop_motor_manual();
    return false;
}

void demo3_resume_line_following(void) {
    printf("[DEMO3:RECOVERY] Resuming line following...\n");
    // Note: The main loop's line following logic will re-engage automatically
    // This function is a placeholder for any special re-init needed
    sleep_ms(500);  // Brief stabilization delay
}
