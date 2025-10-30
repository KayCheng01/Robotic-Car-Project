// src/ir_line_test.c

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include "ir.h" // Include ir header to access sensor functions
// Include motor header if you plan to actually control motors during test
// #include "motor.h"

// Include necessary constants from demo2.c or define them here for test consistency
// Use test-specific names to avoid conflicts if demo2.c is also included elsewhere
#define TEST_LOOP_MS           100 // Shorter loop interval for better timing precision
#define TEST_BASE_PWM          160
#define TEST_LINE_KP           40.0f
#define TEST_DIFF_CLAMP        60.0f
#define TEST_SLOW_ON_TURN      12.0f
#define TEST_LOST_TIMEOUT_MS   300 // Note: This timeout logic might be affected by the new print timing
#define TEST_RECOVER_PWM       80
#define TEST_RECOVER_DIR       +1

#define PRINT_INTERVAL_MS      5000 // Print every 5 seconds

// Helper function to simulate sensor states or read actual sensors
// For initial testing, you might just read the actual sensors
// Or use a more complex mechanism for controlled input if needed
int get_current_sensor_state(void) {
    int L = ir_left_is_black();
    int R = ir_right_is_black();
    return (L << 1) | R; // Pack into a single int: L in bit 1, R in bit 0
}

// Static inline helper for clamping (replicating demo2.c logic)
static inline float test_clampf(float v, float lo, float hi){
    if(v<lo) v=lo;
    if(v>hi) v=hi;
    return v;
}

static void ir_line_test_task(void* arg) {
    printf("[TEST] Starting IR Line Following Test...\n");

    // Initialize subsystems needed for the test
    // ir_init() should already be called in main if needed globally,
    // or call it here if this task is responsible for it.
    // ir_init(); // Call if necessary for this task's context

    // If testing motor control, initialize motor here
    // motor_init();
    // disable_pid_control(); // Ensure direct PWM control if needed

    int test_step = 0; // Counter for test progression or state

    // Rate limiting variables - Track time for printing
    absolute_time_t last_print_time = get_absolute_time();

    while (test_step < 1000) { // Run for a set number of steps or until condition met
        absolute_time_t current_time = get_absolute_time();

        // Check if it's time to print (based on elapsed time)
        if (absolute_time_diff_us(last_print_time, current_time) >= PRINT_INTERVAL_MS * 1000ULL) {
            // Get current sensor state *at print time*
            int sensor_state = get_current_sensor_state();
            int L = (sensor_state >> 1) & 1; // Extract left sensor bit
            int R = sensor_state & 1;        // Extract right sensor bit

            // --- Core Logic Under Test (Replicated from demo2.c logic) ---
            // This logic now runs only when printing
            if (L || R) {
                // Normal follow path
                // Note: last_seen_test logic might need adjustment if print interval is much longer than loop time
                // For this test, we might just print the instantaneous state based on L||R
                // Or, maintain a separate state for line loss based on the shorter loop time if needed.

                float err = (float)R - (float)L;
                float diff = TEST_LINE_KP * err;
                diff = test_clampf(diff, -TEST_DIFF_CLAMP, TEST_DIFF_CLAMP);

                float base = (float)TEST_BASE_PWM - TEST_SLOW_ON_TURN * (float)fabsf(err);
                if (base < 60) base = 60; // Don't stall

                int Lp = (int)(base - diff);
                int Rp = (int)(base + diff);

                printf("[TEST-STEP %d] Sensors: L=%d, R=%d\n", test_step, L, R);
                printf("[TEST] Normal Follow: err=%.2f, diff=%.2f, base=%.2f, Lp=%d, Rp=%d\n",
                       (double)err, (double)diff, (double)base, Lp, Rp);

            } else {
                // Both sensors see white path - print immediately when detected during print check
                // Note: The LOST_TIMEOUT_MS logic is based on the *print* time now, not a fast internal loop.
                // This changes the behavior significantly compared to the original demo2.c.
                // If you need the original timeout behavior, the logic becomes more complex.

                // For this simplified version, just print the state when it occurs at print time
                printf("[TEST-STEP %d] Sensors: L=%d, R=%d\n", test_step, L, R);
                printf("[TEST] Both Sensors White (Check for Recovery Logic Separately)\n");
                // Recovery logic based on print-time would require tracking time since *last* L||R was true
                // at the print interval, which is different from the original loop-based timeout.
                // If you strictly need the original timeout behavior, consider keeping the fast loop
                // and only rate-limiting the *printing* part of the output, not the entire logic block.
            }
            // --- End Core Logic Under Test ---

            last_print_time = current_time; // Update the time of the last print
            test_step++; // Only increment step when we print
        }

        // Short delay to prevent tight looping and allow other tasks to run
        vTaskDelay(pdMS_TO_TICKS(TEST_LOOP_MS));
    }

    printf("[TEST] IR Line Following Test Completed.\n");
    vTaskDelete(NULL); // Delete this test task
}

// Optional: Function to start the test task, potentially called from main
void start_ir_line_test_task(void) {
    xTaskCreate(ir_line_test_task, "ir_line_test", 1024, NULL, 2, NULL);
}