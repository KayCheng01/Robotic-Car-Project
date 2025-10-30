/**
 * test_servo_only.c
 * Minimal test: initialize servo and sweep through positions repeatedly
 *
 * Procedure:
 *   1. Flash this file (swap main() in CMakeLists.txt)
 *   2. Open serial monitor at 115200 baud
 *   3. Observe servo physically move: LEFT → CENTER → RIGHT → repeat
 *   4. Servo should settle in ~300ms per position
 *
 * Expected output:
 *   Servo Motor Test
 *   [SERVO] Init OK (pin=5, slice=X, ch=X)
 *   Moving to LEFT (0°)
 *   Moving to CENTER (90°)
 *   Moving to RIGHT (180°)
 *   [repeats]
 *
 * Success criteria:
 *   - Servo arm moves smoothly to each position
 *   - No grinding or buzzing
 *   - Movement is complete before next command
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "servo.h"

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(800);
    
    printf("\n=== Servo Motor Test ===\n");
    printf("Initializing servo on GPIO5 (PWM)...\n");
    
    servo_init();
    
    printf("Servo initialized. Starting sweep test.\n");
    printf("Observe physical servo movement: should move to LEFT → CENTER → RIGHT smoothly.\n\n");
    
    int cycle = 0;
    while (true) {
        cycle++;
        
        printf("\n--- Cycle %d ---\n", cycle);
        
        printf("Moving to LEFT (0°)...\n");
        servo_move_left(300);
        printf("LEFT position reached. Holding 1 second...\n");
        sleep_ms(1000);
        
        printf("Moving to CENTER (90°)...\n");
        servo_move_center(300);
        printf("CENTER position reached. Holding 1 second...\n");
        sleep_ms(1000);
        
        printf("Moving to RIGHT (180°)...\n");
        servo_move_right(300);
        printf("RIGHT position reached. Holding 1 second...\n");
        sleep_ms(1000);
        
        printf("Cycle %d complete.\n", cycle);
    }
    
    return 0;
}
