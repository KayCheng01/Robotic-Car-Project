// src/main_ir_test.c

#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
// Include headers for other subsystems if needed for initialization
#include "ir.h" // Include ir header for ir_init
// #include "motor.h" // Include motor header if needed for motor_init in main

// Declare the function to start the test task (defined in ir_line_test.c)
extern void start_ir_line_test_task(void);

int main(void){
    stdio_init_all();
    sleep_ms(10000);
    printf("\n[TEST-IR] Starting IR Line Following Unit Test (New Main)\n");

    // Initialize necessary subsystems globally if needed before tasks start
    // Example: Initialize IR sensors globally
    ir_init();
    printf("[TEST-IR] IR Sensors Initialized\n");

    // Example: Initialize Motor globally (if motor init is centralized)
    // motor_init();
    // disable_pid_control(); // If needed globally
    // printf("[TEST-IR] Motor System Initialized\n");

    // Create the IR Line Following Test Task
    start_ir_line_test_task(); // This calls xTaskCreate for ir_line_test_task

    // Start the FreeRTOS Scheduler
    vTaskStartScheduler();

    // If all is well, the scheduler will now be running the tasks.
    // The following line should only execute if there was insufficient heap
    // memory available to create the idle task.
    while(1){
        // Error state - scheduler failed to start
        printf("[TEST-IR] FATAL: Failed to start FreeRTOS scheduler!\n");
        sleep_ms(1000);
    }

    return 0; // Should not reach here
}