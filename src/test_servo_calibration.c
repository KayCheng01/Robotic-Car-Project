#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "servo.h"

/**
 * Servo Calibration Test
 * 
 * Tests servo angles incrementally to verify calibration offset.
 * Sweeps from 0° to 180° in 10° steps with 1-second hold at each position.
 * 
 * Expected output on serial (115200 baud):
 *   [SERVO] Init OK (pin=15, slice=7, ch=3, offset=-10.0°)
 *   [SERVO_CAL] Testing angle: 0°
 *   [SERVO_CAL] Testing angle: 10°
 *   ...
 *   [SERVO_CAL] Testing angle: 180°
 *   [SERVO_CAL] Calibration test complete!
 */

int main(void) {
    stdio_init_all();
    sleep_ms(500);  // Give serial time to initialize
    
    printf("\n========================================\n");
    printf("  SERVO CALIBRATION TEST (Manual Check)\n");
    printf("========================================\n");
    printf("Sweeping 0° → 180° in 10° steps\n");
    printf("Hold each position for 1 second\n");
    printf("Physically measure angle with protractor\n");
    printf("========================================\n\n");
    
    // Initialize servo
    servo_init();
    sleep_ms(500);
    
    // Sweep from 0° to 180° in 10° increments
    for (float angle = 0.0f; angle <= 180.0f; angle += 10.0f) {
        printf("[SERVO_CAL] Testing angle: %.0f°\n", (double)angle);
        servo_set_angle(angle);
        fflush(stdout);
        sleep_ms(1000);  // Hold for 1 second
    }
    
    printf("\n[SERVO_CAL] Calibration test complete!\n");
    printf("========================================\n");
    printf("Servo should be centered/parallel at 90°\n");
    printf("If not, adjust SERVO_CALIBRATION_OFFSET\n");
    printf("========================================\n\n");
    
    // Park at center for final inspection
    printf("[SERVO_CAL] Parking at CENTER (90°)...\n");
    servo_set_angle(90.0f);
    sleep_ms(500);
    printf("[SERVO_CAL] Ready for final check\n");
    
    // Loop forever at center
    while (1) {
        sleep_ms(1000);
    }
    
    return 0;
}
