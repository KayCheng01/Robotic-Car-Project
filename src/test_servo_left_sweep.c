#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "servo.h"

/**
 * Servo Left Sweep Test
 * 
 * Sweeps from 90° (center) to 0° (leftmost) to find maximum left angle.
 * Each degree displayed for 0.5 seconds with angle printed.
 * 
 * Expected serial output (115200 baud):
 *   [SERVO] Init OK (pin=15, slice=7, ch=3, offset=7.0°)
 *   [SERVO_LEFT] Starting left sweep from 90° to 0°
 *   [SERVO_LEFT] Angle: 90° (center)
 *   [SERVO_LEFT] Angle: 89° - Check position (0.5 sec)
 *   [SERVO_LEFT] Angle: 88° - Check position (0.5 sec)
 *   ...
 *   [SERVO_LEFT] Angle: 0° - Check position (0.5 sec)
 *   [SERVO_LEFT] Sweep complete! Note the maximum left angle.
 */

int main(void) {
    stdio_init_all();
    sleep_ms(500);
    
    printf("\n========================================\n");
    printf("  SERVO LEFT SWEEP TEST\n");
    printf("========================================\n");
    printf("Sweeping from 90° (center) to 0° (left)\n");
    printf("Each degree displayed for 0.5 seconds\n");
    printf("Watch and note maximum left angle\n");
    printf("before servo hits mechanical limit\n");
    printf("========================================\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(1000);
    
    printf("[SERVO_LEFT] Starting left sweep from 90° to 0°...\n\n");
    sleep_ms(500);
    
    // Start at center
    printf("[SERVO_LEFT] Angle: 90° (center) - parking here first\n");
    servo_set_angle(90.0f);
    sleep_ms(1000);
    
    // Sweep left from 89° to 0°
    for (float angle = 89.0f; angle >= 0.0f; angle -= 1.0f) {
        printf("[SERVO_LEFT] Angle: %.0f° - Check position (0.5 sec)\n", (double)angle);
        servo_set_angle(angle);
        fflush(stdout);
        sleep_ms(500);
    }
    
    printf("\n[SERVO_LEFT] Sweep complete!\n");
    printf("========================================\n");
    printf("Maximum left angle before limit?\n");
    printf("Did servo hit a physical stop?\n");
    printf("Report the maximum safe left angle\n");
    printf("and I'll configure scanning range.\n");
    printf("========================================\n\n");
    
    // Park at center
    printf("[SERVO_LEFT] Parking at 90° (center)...\n");
    servo_set_angle(90.0f);
    sleep_ms(500);
    printf("[SERVO_LEFT] Ready for your feedback\n");
    
    // Loop forever at center
    while (1) {
        sleep_ms(1000);
    }
    
    return 0;
}
