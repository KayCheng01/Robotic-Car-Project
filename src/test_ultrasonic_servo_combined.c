/**
 * test_ultrasonic_servo_combined.c
 * Test ultrasonic + servo together: servo sweeps left/center/right while measuring distances
 *
 * Procedure:
 *   1. Flash this file (swap in CMakeLists.txt)
 *   2. Open serial monitor at 115200 baud
 *   3. Place obstacle ~20cm ahead
 *   4. Watch servo sweep and distance readings update
 *
 * Expected output:
 *   === Ultrasonic + Servo Combined Test ===
 *   [SERVO] Init OK...
 *   [ULTRASONIC] Init OK...
 *   
 *   --- Scan 1 ---
 *   Moving to CENTER...
 *   Center distance: 19.5 cm
 *   Moving to LEFT...
 *   Left distance: 35.2 cm
 *   Moving to RIGHT...
 *   Right distance: 22.1 cm
 *   Estimated width: 15.7 cm
 *   [repeats every 8 seconds]
 *
 * Success criteria:
 *   - Servo moves smoothly through 3 positions
 *   - Center distance is smallest (obstacle ahead)
 *   - One side shows significantly larger distance (clear path)
 *   - Width estimation is reasonable
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "servo.h"
#include "ultrasonic.h"

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(800);
    
    printf("\n=== Ultrasonic + Servo Combined Test ===\n");
    printf("This test coordinates servo positioning with ultrasonic measurements.\n");
    printf("Place an obstacle ~15-25cm ahead, with clear space on at least one side.\n\n");
    
    printf("Initializing servo...\n");
    servo_init();
    
    printf("Initializing ultrasonic...\n");
    ultrasonic_init();
    
    printf("\nReady. Starting scan cycles...\n");
    printf("Each cycle takes ~8 seconds (300ms move + 300ms settle + 1s per position + 500ms interval)\n\n");
    
    int scan_num = 0;
    while (true) {
        scan_num++;
        printf("--- Scan %d ---\n", scan_num);
        
        // CENTER measurement
        printf("  Moving to CENTER (90°)...\n");
        servo_move_center(300);
        sleep_ms(300);  // Stabilize
        float dist_center = ultrasonic_get_distance_cm();
        printf("  Center distance: %.1f cm\n", (double)dist_center);
        sleep_ms(500);
        
        // LEFT measurement
        printf("  Moving to LEFT (0°)...\n");
        servo_move_left(300);
        sleep_ms(300);  // Stabilize
        float dist_left = ultrasonic_get_distance_cm();
        printf("  Left distance: %.1f cm\n", (double)dist_left);
        sleep_ms(500);
        
        // RIGHT measurement
        printf("  Moving to RIGHT (180°)...\n");
        servo_move_right(300);
        sleep_ms(300);  // Stabilize
        float dist_right = ultrasonic_get_distance_cm();
        printf("  Right distance: %.1f cm\n", (double)dist_right);
        sleep_ms(500);
        
        // Return to center
        printf("  Returning to CENTER...\n");
        servo_move_center(300);
        
        // Analyze
        float max_side = fmaxf(dist_left, dist_right);
        float width = fmaxf(0.0f, max_side - dist_center);
        
        // Determine clear side
        const char *clear_side = "BLOCKED";
        if (dist_left > 25.0f && dist_left > dist_right + 5.0f) {
            clear_side = "LEFT (more clearance)";
        } else if (dist_right > 25.0f && dist_right > dist_left + 5.0f) {
            clear_side = "RIGHT (more clearance)";
        } else if (dist_left > 25.0f && dist_right > 25.0f) {
            clear_side = "BOTH (choose best)";
        }
        
        printf("  Estimated obstacle width: %.1f cm\n", (double)width);
        printf("  Clear side: %s\n", clear_side);
        printf("  [Raw: L=%.1f, C=%.1f, R=%.1f]\n\n", 
               (double)dist_left, (double)dist_center, (double)dist_right);
        
        sleep_ms(1000);  // Pause between scans
    }
    
    return 0;
}
