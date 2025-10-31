#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

// ============================================================
//                CONFIGURATION SECTION
// ============================================================

// === Servo configuration ===
#define SERVO_PIN        2       // PWM pin connected to the servo signal wire
#define SERVO_CENTER_US  1500    // Center pulse width in microseconds (1.5 ms = 0°)
#define SERVO_SPAN_US     500    // Pulse span: ±500 µs → covers ~±90° movement range
#define SERVO_SETTLE_MS    70    // Delay (ms) for servo to physically settle after each move

// === Ultrasonic sensor configuration ===
#define TRIG_PIN         3       // GPIO pin connected to HC-SR04 TRIG (output from Pico)
#define ECHO_PIN         4       // GPIO pin connected to HC-SR04 ECHO (input to Pico)
                                 // IMPORTANT: Use a voltage divider on ECHO (5V → 3.3V)
#define D_MIN_CM          6.0f   // Minimum valid distance reading (cm)
#define D_MAX_CM         80.0f   // Maximum valid distance reading (cm)

// === Scanning sweep parameters ===
#define SWEEP_MIN_DEG   -30.0f   // Minimum sweep angle (leftmost)
#define SWEEP_MAX_DEG    30.0f   // Maximum sweep angle (rightmost)
#define SWEEP_STEP_DEG    1.0f   // Step size for each servo move during sweep (smaller = smoother)
#define EDGE_JUMP_CM     15.0f   // Distance difference threshold (cm) to detect object edges

// === Mount and direction tuning ===
#define YAW_BIAS_DEG      10.0f   // Adjust if "0°" does not face straight ahead (positive = rotate right)
#define YAW_DIR_SIGN     +1.0f   // Set to -1.0 if servo sweeps reversed (left↔right flipped)

// ============================================================


// ============================================================
//                  SERVO CONTROL FUNCTIONS
// ============================================================

/*
 * Initializes the servo by configuring the PWM hardware.
 * - Uses 50 Hz frequency (20 ms period)
 * - 1 µs resolution
 * - Starts centered
 */
static void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);          // Set pin function to PWM
    uint slice = pwm_gpio_to_slice_num(SERVO_PIN);        // Get PWM slice for this pin

    pwm_config c = pwm_get_default_config();
    pwm_config_set_clkdiv(&c, 125.0f);                    // 125 MHz / 125 = 1 MHz → 1 µs per tick
    pwm_config_set_wrap(&c, 20000 - 1);                   // 20,000 ticks = 20 ms period (50 Hz)
    pwm_init(slice, &c, true);                            // Initialize PWM
    pwm_set_gpio_level(SERVO_PIN, SERVO_CENTER_US);       // Move servo to center (0°)
    sleep_ms(300);                                        // Wait for servo to settle
}

/*
 * Rotates the servo to a given logical angle (in degrees).
 * - logical_deg is the desired angle (e.g., -45 to +45)
 * - The function applies mounting bias and direction sign
 * - Converts degrees to PWM pulse width (µs)
 * - Waits briefly for servo movement to complete
 */
static void servo_write_deg(float logical_deg) {
    // Apply bias and direction correction
    float mech_deg = YAW_BIAS_DEG + (YAW_DIR_SIGN * logical_deg);

    // Clamp mechanical angle to safe range
    if (mech_deg < -90) mech_deg = -90;
    if (mech_deg >  90) mech_deg =  90;

    // Convert angle to pulse width (µs)
    uint16_t pulse = SERVO_CENTER_US + (int16_t)((mech_deg / 90.0f) * SERVO_SPAN_US);

    // Constrain pulse range (most servos accept 500–2500 µs)
    if (pulse < 500)  pulse = 500;
    if (pulse > 2500) pulse = 2500;

    // Send PWM pulse and wait for servo to move
    pwm_set_gpio_level(SERVO_PIN, pulse);
    sleep_ms(SERVO_SETTLE_MS);
}

// ============================================================
//                  ULTRASONIC SENSOR FUNCTIONS
// ============================================================

/*
 * Initializes the ultrasonic sensor (HC-SR04 type)
 * - TRIG: output (Pico → sensor)
 * - ECHO: input (sensor → Pico)
 */
static void ultrasonic_init(void) {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);       // ensure TRIG starts LOW

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

/*
 * Sends one trigger pulse and measures the echo pulse width.
 * Converts pulse duration to distance (cm).
 * Returns NaN if no valid echo is received.
 */
static float ultrasonic_read_cm_once(void) {
    // Send 10 µs trigger pulse
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    // Wait for echo HIGH (start)
    absolute_time_t t_to = make_timeout_time_us(30000); // 30 ms timeout
    while (!gpio_get(ECHO_PIN))
        if (absolute_time_diff_us(get_absolute_time(), t_to) > 0) return NAN;
    absolute_time_t t1 = get_absolute_time();

    // Wait for echo LOW (end)
    while (gpio_get(ECHO_PIN))
        if (absolute_time_diff_us(get_absolute_time(), t1) > 30000) return NAN;
    absolute_time_t t2 = get_absolute_time();

    // Calculate duration (µs) and convert to distance (cm)
    float us = (float)absolute_time_diff_us(t1, t2);
    float cm = us / 58.0f;  // ~58 µs per cm at 20–25°C
    if (cm < D_MIN_CM || cm > D_MAX_CM) return NAN; // filter out invalid
    return cm;
}

/*
 * Utility: returns the median of three values.
 * Used to smooth out noisy ultrasonic readings.
 */
static inline float median3(float a, float b, float c) {
    if (a > b){ float t=a;a=b;b=t; }
    if (b > c){ float t=b;b=c;c=t; }
    if (a > b){ float t=a;a=b;b=t; }
    return b;
}

/*
 * Takes three ultrasonic readings and returns their median.
 * This improves reliability by removing outliers.
 */
static float ultrasonic_read_cm(void) {
    float a = ultrasonic_read_cm_once(); sleep_ms(5);
    float b = ultrasonic_read_cm_once(); sleep_ms(5);
    float c = ultrasonic_read_cm_once();

    // Handle all-invalid case
    if (isnan(a) && isnan(b) && isnan(c)) return NAN;

    // Substitute missing values if only 1–2 failed
    if (isnan(a)) a = (!isnan(b)) ? b : c;
    if (isnan(b)) b = (!isnan(c)) ? c : a;
    if (isnan(c)) c = (!isnan(a)) ? a : b;

    return median3(a,b,c);
}

// ============================================================
//                OBSTACLE WIDTH SCANNING LOGIC
// ============================================================

/*
 * Main loop:
 * 1. Sweep servo from left (-45°) to right (+45°)
 * 2. Measure distance at each angle
 * 3. Detect left and right edges (where distance changes sharply)
 * 4. Compute obstacle width using simple trigonometry
 */
int main() {
    stdio_init_all();
    sleep_ms(300);

    // Initialize hardware
    servo_init();
    ultrasonic_init();

    while (true) {
        float d_prev = NAN;                // stores previous distance
        float left_angle = 0, right_angle = 0;
        float left_dist = 0, right_dist = 0;
        bool found_left = false, found_right = false;

        printf("\n=== Scanning... ===\n");

        // Sweep from left to right
        for (float deg = SWEEP_MIN_DEG; deg <= SWEEP_MAX_DEG; deg += SWEEP_STEP_DEG) {
            servo_write_deg(deg);              // move servo
            float d = ultrasonic_read_cm();    // measure distance
            printf("Angle %.1f°, Distance %.1f cm\n", deg, d);

            // Detect large change (jump) → possible edge
            if (!isnan(d_prev) && !isnan(d)) {
                float jump = d_prev - d; // negative jump = object closer

                // Detect entering edge (free → object)
                if (!found_left && jump > EDGE_JUMP_CM) {
                    left_angle = deg;
                    left_dist = d;
                    found_left = true;
                }

                // Detect exiting edge (object → free)
                if (found_left && !found_right && -jump > EDGE_JUMP_CM) {
                    right_angle = deg;
                    right_dist = d;
                    found_right = true;
                }
            }
            d_prev = d;
        }

        // If both edges detected, compute width
        if (found_left && found_right) {
            // Convert polar (θ, d) to Cartesian (x, y)
            float thL = left_angle  * (M_PI / 180.0f);
            float thR = right_angle * (M_PI / 180.0f);
            float xL = left_dist * cosf(thL);
            float yL = left_dist * sinf(thL);
            float xR = right_dist * cosf(thR);
            float yR = right_dist * sinf(thR);

            // Distance between edges = width
            float width = sqrtf((xR - xL)*(xR - xL) + (yR - yL)*(yR - yL));
            printf("\nEstimated obstacle width: %.1f cm\n", width);
        } else {
            printf("\nNo clear edges detected.\n");
        }

        sleep_ms(1500); // wait before next full scan
    }
}
