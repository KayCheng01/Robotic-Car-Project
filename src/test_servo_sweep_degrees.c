#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "servo.h"

/**
 * Servo Degree-by-Degree Sweep Test
 * 
 * Automatically sweeps through every degree (0-180) with print statements.
 * Pauses at each degree so you can physically check and identify the center position.
 * 
 * Expected serial output (115200 baud):
 *   [SERVO] Init OK (pin=15, slice=7, ch=3, offset=-10.0°)
 *   [SERVO_SWEEP] Starting degree-by-degree sweep
 *   [SERVO_SWEEP] Angle: 0° - Check position (1/2 sec)
 *   [SERVO_SWEEP] Angle: 1° - Check position (1/2 sec)
 *   [SERVO_SWEEP] Angle: 2° - Check position (1/2 sec)
 *   ...
 *   [SERVO_SWEEP] Angle: 180° - Check position (1/2 sec)
 *   [SERVO_SWEEP] Sweep complete! Note which angle looked centered/parallel.
 */

int main(void) {
    stdio_init_all();
    sleep_ms(500);
    
    printf("\n========================================\n");
    printf("  SERVO DEGREE-BY-DEGREE SWEEP\n");
    printf("========================================\n");
    printf("Sweeping 0° to 180° (every 1°)\n");
    printf("Each degree displayed for 0.5 seconds\n");
    printf("Watch physically and note which angle\n");
    printf("is centered/parallel to your car\n");
    printf("========================================\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(1000);
    
    printf("[SERVO_SWEEP] Starting degree-by-degree sweep...\n\n");
    sleep_ms(500);
    
    // Sweep every degree from 0 to 180
    for (float angle = 0.0f; angle <= 180.0f; angle += 1.0f) {
        printf("[SERVO_SWEEP] Angle: %.0f° - Check position (0.5 sec)\n", (double)angle);
        servo_set_angle(angle);
        fflush(stdout);
        sleep_ms(500);
    }
    
    printf("\n[SERVO_SWEEP] Sweep complete!\n");
    printf("========================================\n");
    printf("Which angle looked centered/parallel?\n");
    printf("Report that angle and I'll update the\n");
    printf("calibration offset accordingly.\n");
    printf("Current offset: -10.0°\n");
    printf("========================================\n\n");
    
    // Park at center for final inspection
    printf("[SERVO_SWEEP] Parking at 90° for reference...\n");
    servo_set_angle(90.0f);
    sleep_ms(500);
    printf("[SERVO_SWEEP] Ready for your feedback\n");
    
    // Loop forever at center
    while (1) {
        sleep_ms(1000);
    }
    
    return 0;
}
