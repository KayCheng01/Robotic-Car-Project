#include "ultrasonic.h"
#include <stdio.h>

// ============== Static State ==============
static volatile float g_distance_cm = ULTRASONIC_INVALID_READING;
static volatile uint64_t g_echo_start_us = 0;
static volatile uint64_t g_echo_end_us = 0;
static volatile bool g_echo_captured = false;
static volatile bool g_measurement_pending = false;

// ============== Echo Pin Interrupt Handler ==============
static void ultrasonic_echo_handler(uint gpio, uint32_t events) {
    if (gpio != ULTRASONIC_ECHO_PIN) return;
    
    uint64_t now_us = time_us_64();
    
    // Rising edge: start of echo pulse
    if (events & GPIO_IRQ_EDGE_RISE) {
        g_echo_start_us = now_us;
    }
    
    // Falling edge: end of echo pulse
    if (events & GPIO_IRQ_EDGE_FALL) {
        g_echo_end_us = now_us;
        g_echo_captured = true;
    }
}

// ============== Public API ==============

void ultrasonic_init(void) {
    // Initialize trigger pin (output)
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
    
    // Initialize echo pin (input)
    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
    gpio_pull_down(ULTRASONIC_ECHO_PIN);
    
    // Set up interrupt on echo pin (both edges)
    gpio_set_irq_enabled_with_callback(
        ULTRASONIC_ECHO_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &ultrasonic_echo_handler
    );
    
    printf("[ULTRASONIC] Init OK (trig=%d, echo=%d)\n", 
           ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
}

/**
 * Blocking measurement: send 10µs trigger pulse and wait for echo.
 * Returns distance in cm, or -1.0f if timeout.
 */
float ultrasonic_get_distance_cm(void) {
    // Reset state
    g_echo_start_us = 0;
    g_echo_end_us = 0;
    g_echo_captured = false;
    
    // Send trigger pulse (10 µs)
    gpio_put(ULTRASONIC_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
    
    // Wait for echo to complete with timeout
    uint64_t timeout_at = time_us_64() + ULTRASONIC_TIMEOUT_US;
    while (!g_echo_captured) {
        if (time_us_64() > timeout_at) {
            printf("[ULTRASONIC] Timeout waiting for echo\n");
            return ULTRASONIC_INVALID_READING;
        }
        sleep_us(10);  // Yield to other tasks
    }
    
    // Calculate distance: d = (duration_us / 2) * speed_of_sound_cm_per_us
    // Speed of sound ≈ 343 m/s = 0.0343 cm/µs
    uint64_t pulse_width_us = g_echo_end_us - g_echo_start_us;
    float distance_cm = (float)pulse_width_us * 0.0343f / 2.0f;
    
    // Clamp to valid range
    if (distance_cm < ULTRASONIC_MIN_DISTANCE_CM || 
        distance_cm > ULTRASONIC_MAX_DISTANCE_CM) {
        return ULTRASONIC_INVALID_READING;
    }
    
    g_distance_cm = distance_cm;
    return distance_cm;
}

bool ultrasonic_update(void) {
    // Non-blocking version (for future use)
    // For now, return false as this requires async state machine
    return false;
}

float ultrasonic_get_last_distance_cm(void) {
    return g_distance_cm;
}

bool ultrasonic_obstacle_detected(float threshold_cm) {
    return (g_distance_cm > 0 && g_distance_cm < threshold_cm);
}
