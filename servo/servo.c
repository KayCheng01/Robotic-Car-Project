#include "servo.h"
#include <stdio.h>

// ============== Static State ==============
static float g_current_angle_deg = SERVO_ANGLE_CENTER;
static uint8_t g_servo_slice = 0;
static uint16_t g_servo_channel = 0;

// ============== PWM Timing Constants ==============
// Standard servo: 50 Hz (20ms period)
// Pulse duration: 1ms (0°) to 2ms (180°)
#define PWM_FREQ_HZ         50
#define PWM_PERIOD_US       20000   // 50 Hz = 20ms period
#define PWM_MIN_US          1000    // 1ms = 0°
#define PWM_MAX_US          2000    // 2ms = 180°

// ============== Helper Functions ==============

/**
 * Convert angle (0-180°) to PWM pulse width (microseconds).
 * Linear interpolation: 1ms @ 0°, 2ms @ 180°
 * Applies calibration offset for mechanical alignment.
 */
static inline uint16_t angle_to_pwm_us(float angle_deg) {
    // Apply calibration offset
    float calibrated_angle = angle_deg - SERVO_CALIBRATION_OFFSET;
    
    // Clamp calibrated angle to valid range
    if (calibrated_angle < SERVO_ANGLE_MIN) calibrated_angle = SERVO_ANGLE_MIN;
    if (calibrated_angle > SERVO_ANGLE_MAX) calibrated_angle = SERVO_ANGLE_MAX;
    
    // Linear map: 0° → 1000µs, 180° → 2000µs
    float frac = calibrated_angle / (SERVO_ANGLE_MAX - SERVO_ANGLE_MIN);
    uint16_t pwm_us = (uint16_t)(PWM_MIN_US + frac * (PWM_MAX_US - PWM_MIN_US));
    return pwm_us;
}

/**
 * Convert PWM pulse width to duty cycle level (for Pico PWM counter).
 * PWM counter runs at clk_sys / PWM_DIV. We calculate to achieve the desired pulse.
 */
static inline uint16_t pwm_us_to_level(uint16_t pwm_us) {
    // Typical: clk_sys = 125 MHz, PWM_DIV = 125 → 1 MHz counter
    // PWM_PERIOD_US = 20000µs → wrap = 20000 (counts)
    // level = (pwm_us / PWM_PERIOD_US) * wrap
    return (uint16_t)((pwm_us * 20000) / PWM_PERIOD_US);
}

// ============== Public API ==============

void servo_init(void) {
    // Get PWM details for SERVO_PIN
    g_servo_slice = pwm_gpio_to_slice_num(SERVO_PIN);
    g_servo_channel = pwm_gpio_to_channel(SERVO_PIN);
    
    // Initialize GPIO for PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    
    // Set PWM frequency to 50 Hz
    // clk_sys = 125 MHz (typical), so to achieve 50 Hz with 16-bit wrap:
    // target_freq = clk_sys / (clk_div * (wrap + 1))
    // 50 = 125MHz / (clk_div * 20000) → clk_div ≈ 125
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);  // Divide 125MHz → 1MHz
    pwm_config_set_wrap(&cfg, PWM_PERIOD_US - 1);  // Wrap at 20000-1
    pwm_init(g_servo_slice, &cfg, true);  // Enable
    
    // Start at center position
    servo_set_angle(SERVO_ANGLE_CENTER);
    
    printf("[SERVO] Init OK (pin=%d, slice=%d, ch=%d, offset=%.1f°)\n", 
           SERVO_PIN, g_servo_slice, g_servo_channel, (double)SERVO_CALIBRATION_OFFSET);
}

bool servo_set_angle(float angle_deg) {
    // Clamp and validate
    if (angle_deg < SERVO_ANGLE_MIN || angle_deg > SERVO_ANGLE_MAX) {
        printf("[SERVO] Invalid angle %.1f (range %.0f-%.0f)\n", 
               (double)angle_deg, (double)SERVO_ANGLE_MIN, (double)SERVO_ANGLE_MAX);
        return false;
    }
    
    g_current_angle_deg = angle_deg;
    
    // Convert angle to PWM pulse width in microseconds
    uint16_t pwm_us = angle_to_pwm_us(angle_deg);
    
    // Convert to PWM duty level
    uint16_t pwm_level = pwm_us_to_level(pwm_us);
    
    // Set PWM level on the appropriate channel
    pwm_set_chan_level(g_servo_slice, g_servo_channel, pwm_level);
    
    return true;
}

float servo_get_angle(void) {
    return g_current_angle_deg;
}

void servo_move_left(uint16_t delay_ms) {
    servo_set_angle(SERVO_ANGLE_LEFT);
    sleep_ms(delay_ms);
}

void servo_move_center(uint16_t delay_ms) {
    servo_set_angle(SERVO_ANGLE_CENTER);
    sleep_ms(delay_ms);
}

void servo_move_right(uint16_t delay_ms) {
    servo_set_angle(SERVO_ANGLE_RIGHT);
    sleep_ms(delay_ms);
}
