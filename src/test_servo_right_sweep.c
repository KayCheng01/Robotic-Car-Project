#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "servo.h"

/**
 * Servo Right Sweep Test
 * 
 * Sweeps from 90° (center) to 180° (rightmost) to find maximum right angle.
 * Each degree displayed for 0.5 seconds with angle printed.
 * 
 * Expected serial output (115200 baud):
 *   [SERVO] Init OK (pin=15, slice=7, ch=3, offset=7.0°)
 *   [SERVO_RIGHT] Starting right sweep from 90° to 180°
 *   [SERVO_RIGHT] Angle: 90° (center)
 *   [SERVO_RIGHT] Angle: 91° - Check position (0.5 sec)
 *   [SERVO_RIGHT] Angle: 92° - Check position (0.5 sec)
 *   ...
 *   [SERVO_RIGHT] Angle: 180° - Check position (0.5 sec)
 *   [SERVO_RIGHT] Sweep complete! Note the maximum right angle.
 */

int main(void) {
    stdio_init_all();
    sleep_ms(500);
    
    printf("\n========================================\n");
    printf("  SERVO RIGHT SWEEP TEST\n");
    printf("========================================\n");
    printf("Sweeping from 90° (center) to 180° (right)\n");
    printf("Each degree displayed for 0.5 seconds\n");
    printf("Watch and note maximum right angle\n");
    printf("before servo hits mechanical limit\n");
    printf("========================================\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(1000);
    
    printf("[SERVO_RIGHT] Starting right sweep from 90° to 180°...\n\n");
    sleep_ms(500);
    
    // Start at center
    printf("[SERVO_RIGHT] Angle: 90° (center) - parking here first\n");
    servo_set_angle(90.0f);
    sleep_ms(1000);
    
    // Sweep right from 91° to 180°
    for (float angle = 91.0f; angle <= 180.0f; angle += 1.0f) {
        printf("[SERVO_RIGHT] Angle: %.0f° - Check position (0.5 sec)\n", (double)angle);
        servo_set_angle(angle);
        fflush(stdout);
        sleep_ms(500);
    }
    
    printf("\n[SERVO_RIGHT] Sweep complete!\n");
    printf("========================================\n");
    printf("Maximum right angle before limit?\n");
    printf("Did servo hit a physical stop?\n");
    printf("Report the maximum safe right angle\n");
    printf("and I'll configure scanning range.\n");
    printf("========================================\n\n");
    
    // Park at center
    printf("[SERVO_RIGHT] Parking at 90° (center)...\n");
    servo_set_angle(90.0f);
    sleep_ms(500);
    printf("[SERVO_RIGHT] Ready for your feedback\n");
    
    // Loop forever at center
    while (1) {
        sleep_ms(1000);
    }
    
    return 0;
}
