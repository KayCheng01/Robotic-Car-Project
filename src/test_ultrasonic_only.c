/**
 * test_ultrasonic_only.c
 * Minimal test: initialize ultrasonic sensor and print distance every 500ms
 * 
 * Procedure:
 *   1. Flash this file (swap main() in CMakeLists.txt)
 *   2. Open serial monitor at 115200 baud
 *   3. Place objects at known distances (5cm, 10cm, 20cm, 50cm, 100cm)
 *   4. Observe distance readings and verify they match object position
 *
 * Expected output:
 *   Ultrasonic Sensor Test
 *   Initialized OK
 *   Distance: 150.5 cm
 *   Distance: 125.3 cm
 *   Distance: 25.2 cm  (when you move hand close)
 *   ...
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(800);
    
    printf("\n=== Ultrasonic Sensor Test ===\n");
    printf("Initializing ultrasonic sensor...\n");
    
    ultrasonic_init();
    
    printf("Initialized OK\n");
    printf("Place objects at known distances and observe readings.\n");
    printf("Expected: distance decreases as you move object closer\n\n");
    
    int count = 0;
    while (true) {
        float dist = ultrasonic_get_distance_cm();
        
        // Print with timestamp and validation
        if (dist < 0) {
            printf("[%4d] Distance: ERROR (timeout or out of range)\n", count);
        } else if (dist < 2.0f) {
            printf("[%4d] Distance: %.1f cm (TOO CLOSE)\n", count, (double)dist);
        } else if (dist > 400.0f) {
            printf("[%4d] Distance: %.1f cm (NO OBJECT)\n", count, (double)dist);
        } else {
            printf("[%4d] Distance: %.1f cm ✓\n", count, (double)dist);
        }
        
        count++;
        sleep_ms(500);
    }
    
    return 0;
}
