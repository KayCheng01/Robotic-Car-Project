#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "servo.h"

/**
 * Servo Manual Control Test
 * 
 * Allows manual control of servo via serial input.
 * Enter angles (0-180) to move servo and track calibration.
 * 
 * Usage:
 *   Send: 0     (servo moves to 0°)
 *   Send: 90    (servo moves to 90°)
 *   Send: 180   (servo moves to 180°)
 *   Send: 45    (servo moves to 45°)
 *   etc.
 * 
 * Expected serial output (115200 baud):
 *   [SERVO] Init OK (pin=15, slice=7, ch=3, offset=-10.0°)
 *   [SERVO_MANUAL] Ready for manual control
 *   [SERVO_MANUAL] Enter angle (0-180): 
 *   Enter: 0
 *   [SERVO_MANUAL] Setting angle to 0.0°
 *   [SERVO_MANUAL] Enter angle (0-180): 
 */

int main(void) {
    stdio_init_all();
    sleep_ms(500);
    
    printf("\n========================================\n");
    printf("  SERVO MANUAL CONTROL TEST\n");
    printf("========================================\n");
    printf("Enter angles (0-180) via serial\n");
    printf("Servo will move to your commanded angle\n");
    printf("Track the physical angle for calibration\n");
    printf("========================================\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(500);
    
    printf("[SERVO_MANUAL] Ready for manual control\n");
    printf("[SERVO_MANUAL] Current offset: -10.0°\n\n");
    
    char input_buffer[32];
    int buffer_index = 0;
    
    while (1) {
        printf("[SERVO_MANUAL] Enter angle (0-180): ");
        fflush(stdout);
        
        // Read input line
        buffer_index = 0;
        memset(input_buffer, 0, sizeof(input_buffer));
        
        while (1) {
            int ch = getchar();
            
            if (ch == '\n' || ch == '\r') {
                // End of line
                if (buffer_index > 0) {
                    break;
                }
            } else if (ch >= '0' && ch <= '9') {
                // Digit
                if (buffer_index < sizeof(input_buffer) - 1) {
                    input_buffer[buffer_index++] = ch;
                    putchar(ch);  // Echo
                }
            }
        }
        putchar('\n');
        
        if (buffer_index == 0) {
            printf("[SERVO_MANUAL] No input, skipping\n\n");
            continue;
        }
        
        // Parse angle
        float angle = (float)atoi(input_buffer);
        
        // Validate range
        if (angle < 0.0f || angle > 180.0f) {
            printf("[SERVO_MANUAL] ERROR: Angle out of range (0-180)\n\n");
            continue;
        }
        
        // Set angle
        printf("[SERVO_MANUAL] Setting angle to %.0f°\n", (double)angle);
        servo_set_angle(angle);
        
        // Get current angle (after offset applied internally)
        float current = servo_get_angle();
        printf("[SERVO_MANUAL] Commanded: %.0f° | Current: %.0f°\n", 
               (double)angle, (double)current);
        printf("[SERVO_MANUAL] Note: Internal offset (-10.0°) already applied\n\n");
        
        sleep_ms(100);  // Brief pause
    }
    
    return 0;
}
