#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "servo.h"

/**
 * Servo Symmetric Scan Test
 * 
 * Tests symmetric scanning pattern:
 * - Start at center (90°)
 * - Sweep LEFT in 5° increments (90° → 170° → 90°)
 * - Sweep RIGHT in 5° increments (90° → 10° → 90°)
 * - Repeat continuously
 * - Print angle at each position
 * 
 * Expected serial output (115200 baud):
 *   [SERVO] Init OK (pin=15, slice=7, ch=3, offset=7.0°)
 *   [SERVO_SCAN] Starting symmetric scan test
 *   
 *   === LEFT SWEEP ===
 *   [SERVO_SCAN] Angle: 90° (center) - ready to scan
 *   [SERVO_SCAN] Angle: 95° (left 1) - 0.5 sec
 *   [SERVO_SCAN] Angle: 100° (left 2) - 0.5 sec
 *   ...
 *   [SERVO_SCAN] Angle: 170° (left max) - 0.5 sec
 *   [SERVO_SCAN] Angle: 90° (back to center)
 *   
 *   === RIGHT SWEEP ===
 *   [SERVO_SCAN] Angle: 90° (center) - ready to scan
 *   [SERVO_SCAN] Angle: 85° (right 1) - 0.5 sec
 *   [SERVO_SCAN] Angle: 80° (right 2) - 0.5 sec
 *   ...
 *   [SERVO_SCAN] Angle: 10° (right max) - 0.5 sec
 *   [SERVO_SCAN] Angle: 90° (back to center)
 *   
 *   === LEFT SWEEP ===
 *   (repeat...)
 */

int main(void) {
    stdio_init_all();
    sleep_ms(500);
    
    printf("\n========================================\n");
    printf("  SERVO SYMMETRIC SCAN TEST\n");
    printf("========================================\n");
    printf("Pattern: LEFT (5° steps) → CENTER\n");
    printf("         RIGHT (5° steps) → CENTER\n");
    printf("         REPEAT continuously\n");
    printf("Range: 10° (right) to 170° (left)\n");
    printf("Center: 90° (parallel to car)\n");
    printf("========================================\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(1000);
    
    uint32_t cycle = 0;
    
    while (1) {
        cycle++;
        printf("\n========== CYCLE %u ==========\n\n", (unsigned int)cycle);
        
        // ============ LEFT SWEEP (90° → 170° → 90°) ============
        printf("=== LEFT SWEEP ===\n");
        printf("[SERVO_SCAN] Angle: 90° (center) - ready to scan left\n");
        servo_set_angle(90.0f);
        sleep_ms(500);
        
        // Sweep left: 90 → 170 in 5° steps
        for (float angle = 95.0f; angle <= 170.0f; angle += 5.0f) {
            printf("[SERVO_SCAN] Angle: %.0f° (left step) - 0.5 sec\n", (double)angle);
            servo_set_angle(angle);
            fflush(stdout);
            sleep_ms(500);
        }
        
        // Return to center
        printf("[SERVO_SCAN] Angle: 90° (returning to center)\n");
        servo_set_angle(90.0f);
        sleep_ms(500);
        
        // ============ RIGHT SWEEP (90° → 10° → 90°) ============
        printf("\n=== RIGHT SWEEP ===\n");
        printf("[SERVO_SCAN] Angle: 90° (center) - ready to scan right\n");
        servo_set_angle(90.0f);
        sleep_ms(500);
        
        // Sweep right: 90 → 10 in 5° steps
        for (float angle = 85.0f; angle >= 10.0f; angle -= 5.0f) {
            printf("[SERVO_SCAN] Angle: %.0f° (right step) - 0.5 sec\n", (double)angle);
            servo_set_angle(angle);
            fflush(stdout);
            sleep_ms(500);
        }
        
        // Return to center
        printf("[SERVO_SCAN] Angle: 90° (returning to center)\n");
        servo_set_angle(90.0f);
        sleep_ms(500);
    }
    
    return 0;
}
