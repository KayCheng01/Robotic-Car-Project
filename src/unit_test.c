// UT-Motor-01 — Verify PWM configuration (20–30 kHz, 8-bit TOP)
// Built for RP2040 (Pico SDK v1.5.x). Run on device, watch USB CDC logs.

#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include "motor.h"  // provides motor_pwm_init() and pin defines
                    // L_MOTOR_IN1/IN2, R_MOTOR_IN3/IN4  

// --------- Config ---------
#define FREQ_MIN_HZ  20000.0f
#define FREQ_MAX_HZ  30000.0f
#ifndef TEST_PWM_PIN
#define TEST_PWM_PIN L_MOTOR_IN1
#endif

static inline const char* passfail(bool b){ return b ? "PASS" : "FAIL"; }

// Wait for a rising edge on 'pin' with timeout (ms); returns nil_time on timeout
static absolute_time_t wait_for_rising_edge(uint pin, uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    bool last = gpio_get(pin);

    // ensure we see a falling edge first (avoid steady-high false positive)
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        bool cur = gpio_get(pin);
        if (last && !cur) break;
        last = cur;
        tight_loop_contents();
    }
    // then wait for rising
    while (absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        bool cur = gpio_get(pin);
        if (!last && cur) return get_absolute_time();
        last = cur;
        tight_loop_contents();
    }
    return nil_time;
}

// Measure PWM frequency by timing two successive rising edges
static bool measure_pwm_frequency(uint pin, float *out_hz) {
    absolute_time_t t1 = wait_for_rising_edge(pin, 200); // ms
    if (is_nil_time(t1)) return false;
    absolute_time_t t2 = wait_for_rising_edge(pin, 200);
    if (is_nil_time(t2)) return false;
    int64_t dt_us = absolute_time_diff_us(t1, t2);
    if (dt_us <= 0) return false;
    *out_hz = 1e6f / (float)dt_us;
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(10000);  // allow USB CDC to enumerate
    printf("\n=== UT-Motor-01: Verify PWM configuration unit ===\n");

    // 1) Call the function under test (your existing PWM init)
    motor_pwm_init();  // sets GPIO functions, TOP, clkdiv, enables slices
                       // and calls coast()  

    // 2) Verify pins mapped to PWM
    bool pins_ok =
        gpio_get_function(L_MOTOR_IN1) == GPIO_FUNC_PWM &&
        gpio_get_function(L_MOTOR_IN2) == GPIO_FUNC_PWM &&
        gpio_get_function(R_MOTOR_IN3) == GPIO_FUNC_PWM &&
        gpio_get_function(R_MOTOR_IN4) == GPIO_FUNC_PWM;
    printf("[Pins mapped to PWM] %s\n", passfail(pins_ok));

    // 3) Verify 8-bit resolution: TOP must be 255 on the slice used by TEST_PWM_PIN
    uint slice = pwm_gpio_to_slice_num(TEST_PWM_PIN);
    uint16_t top = pwm_hw->slice[slice].top;   // read TOP directly (SDK v1.5.x)
    bool wrap_ok = (top == 255);
    printf("[8-bit resolution (TOP==255)] %s — TOP=%u\n", passfail(wrap_ok), top);

    // 4) Force a stable duty and measure frequency at TEST_PWM_PIN
    pwm_set_gpio_level(TEST_PWM_PIN, 128); // ~50% duty for clean edges
    float hz = 0.0f;
    bool measured = measure_pwm_frequency(TEST_PWM_PIN, &hz);
    bool freq_ok = measured && (hz >= FREQ_MIN_HZ) && (hz <= FREQ_MAX_HZ);
    printf("[Frequency 20–30 kHz] %s — %s%.1f Hz\n",
           passfail(freq_ok), measured ? "" : "no signal / ", hz);

    bool all_ok = pins_ok && wrap_ok && freq_ok;
    printf("\nUT-Motor-01 %s\n", all_ok ? "✅ PASS" : "❌ FAIL");

    while (true) tight_loop_contents();
    return 0;
}
