#include "motor.h"

// =================== Config tweaks ===================
#define RIGHT_INVERTED 0   // set to 1 if your right wheel is reversed in hardware

// Use the larger of the two maxes as the PWM counter top
#ifndef PWM_TOP
#define PWM_TOP ((PWM_MAX_LEFT > PWM_MAX_RIGHT) ? PWM_MAX_LEFT : PWM_MAX_RIGHT)
#endif

static bool use_pid_control = false;
static PIDState pid_state = DISABLED;
static float target_speed = MIN_SPEED;
static float target_turn_angle = CONTINUOUS_TURN;

// =================== Tiny helpers ====================
static inline void set_pwm_level(uint gpio, uint16_t level) {
    pwm_set_gpio_level(gpio, level);
}

static inline uint16_t clamp_left(float v) {
    if (v < PWM_MIN_LEFT)  v = PWM_MIN_LEFT;
    if (v > PWM_MAX_LEFT)  v = PWM_MAX_LEFT;
    return (uint16_t)v;
}
static inline uint16_t clamp_right(float v) {
    if (v < PWM_MIN_RIGHT) v = PWM_MIN_RIGHT;
    if (v > PWM_MAX_RIGHT) v = PWM_MAX_RIGHT;
    return (uint16_t)v;
}

// Per-wheel raw drive (so turns don’t overwrite the other side)
static inline void left_forward_raw(uint16_t pwm) {  // A = PWM, B = 0
    set_pwm_level(L_MOTOR_IN1, pwm);
    set_pwm_level(L_MOTOR_IN2, 0);
}
static inline void left_reverse_raw(uint16_t pwm) {  // A = 0, B = PWM
    set_pwm_level(L_MOTOR_IN1, 0);
    set_pwm_level(L_MOTOR_IN2, pwm);
}

// Right side supports optional inversion
#if RIGHT_INVERTED
// "Forward" means B = PWM, A = 0 (inverted wiring)
static inline void right_forward_raw(uint16_t pwm) {
    set_pwm_level(R_MOTOR_IN3, 0);
    set_pwm_level(R_MOTOR_IN4, pwm);
}
static inline void right_reverse_raw(uint16_t pwm) {
    set_pwm_level(R_MOTOR_IN3, pwm);
    set_pwm_level(R_MOTOR_IN4, 0);
}
#else
// Normal: A = PWM, B = 0 for forward
static inline void right_forward_raw(uint16_t pwm) {
    set_pwm_level(R_MOTOR_IN3, pwm);
    set_pwm_level(R_MOTOR_IN4, 0);
}
static inline void right_reverse_raw(uint16_t pwm) {
    set_pwm_level(R_MOTOR_IN3, 0);
    set_pwm_level(R_MOTOR_IN4, pwm);
}
#endif

static void brake_coast(void) {
    set_pwm_level(L_MOTOR_IN1, 0);
    set_pwm_level(L_MOTOR_IN2, 0);
    set_pwm_level(R_MOTOR_IN3, 0);
    set_pwm_level(R_MOTOR_IN4, 0);
}

// =================== API (manual / PID use) ===================
void forward_motor(float new_pwm_left, float new_pwm_right) {
    uint16_t pl = clamp_left(new_pwm_left);
    uint16_t pr = clamp_right(new_pwm_right);
    left_forward_raw(pl);
    right_forward_raw(pr);
}

void reverse_motor(float new_pwm_left, float new_pwm_right) {
    uint16_t pl = clamp_left(new_pwm_left);
    uint16_t pr = clamp_right(new_pwm_right);
    left_reverse_raw(pl);
    right_reverse_raw(pr);
}

void turn_motor(int direction, float new_pwm_left, float new_pwm_right) {
    // Simple differential turn: one side forward, the other reverse (or lower duty)
    uint16_t pl = clamp_left(new_pwm_left);
    uint16_t pr = clamp_right(new_pwm_right);

    if (direction == LEFT) {
        // left reverse, right forward
        left_reverse_raw(pl);
        right_forward_raw(pr);
    } else {
        // RIGHT: left forward, right reverse
        left_forward_raw(pl);
        right_reverse_raw(pr);
    }
}

void stop_motor(void) {
    brake_coast();
}

// =================== Manual wrappers ===================
void disable_pid_control(void) { use_pid_control = false; }

void forward_motor_manual(float new_pwm_left, float new_pwm_right) {
    disable_pid_control();
    forward_motor(new_pwm_left, new_pwm_right);
}

void reverse_motor_manual(float new_pwm_left, float new_pwm_right) {
    disable_pid_control();
    reverse_motor(new_pwm_left, new_pwm_right);
}

void turn_motor_manual(int direction, float angle, float new_pwm_left, float new_pwm_right) {
    disable_pid_control();
    turn_motor(direction, new_pwm_left, new_pwm_right);
    if (angle != CONTINUOUS_TURN) {
        reset_encoders();
        while (turn_until_angle(angle)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

void stop_motor_manual(void) {
    disable_pid_control();
    stop_motor();
}

void offset_move_motor(int direction, int turn, float offset) {
    if (offset < 0.0f) offset = 0.0f;
    if (offset > 1.0f) offset = 1.0f;

    int pwm_left = PWM_MID_LEFT;
    int pwm_right = PWM_MID_RIGHT;
    int pwm_left_offset_range  = (PWM_MAX_LEFT  - PWM_MIN_LEFT ) / 2;
    int pwm_right_offset_range = (PWM_MAX_RIGHT - PWM_MIN_RIGHT) / 2;

    if (turn == LEFT) {
        pwm_left  -= pwm_left_offset_range  * offset;
        pwm_right += pwm_right_offset_range * offset;
    } else if (turn == RIGHT) {
        pwm_left  += pwm_left_offset_range  * offset;
        pwm_right -= pwm_right_offset_range * offset;
    }

    if (direction == FORWARDS)      forward_motor_manual(pwm_left, pwm_right);
    else if (direction == BACKWARDS) reverse_motor_manual(pwm_left, pwm_right);
}

// =================== PID interface ===================
void enable_pid_control(void) { use_pid_control = true; }

void forward_motor_pid(float new_target_speed) {
    enable_pid_control();
    target_speed = new_target_speed;
    pid_state = FORWARD;
}

void reverse_motor_pid(float new_target_speed) {
    enable_pid_control();
    target_speed = new_target_speed;
    pid_state = REVERSE;
}

void turn_motor_pid(int direction, float new_target_speed, float new_target_turn_angle) {
    enable_pid_control();
    target_speed = new_target_speed;
    target_turn_angle = new_target_turn_angle;
    pid_state = (direction == LEFT) ? LEFT_TURN : RIGHT_TURN;
}

void stop_motor_pid(void) {
    disable_pid_control();
    stop_motor();
    target_speed = 0.0f;
    pid_state = STOP;
    enable_pid_control();
}

// Must call turn_motor and reset encoders before calling this function
bool turn_until_angle(float angle) {
    if (angle == CONTINUOUS_TURN) return true;
    if (angle < 0.0f || angle > FULL_CIRCLE) return false;

    float target_distance = (angle / FULL_CIRCLE) * (PI * WHEEL_TO_WHEEL_DISTANCE);
    if (target_distance - get_average_distance() <= 0.05f) {
        if (use_pid_control) stop_motor_pid();
        else                 stop_motor_manual();
        return false;
    }
    return true;
}

// =================== PID math & task ===================
float compute_pid_pwm(float target, float current, float *integral, float *prev_error) {
    float error = target - current;
    *integral += error;
    float derivative = error - *prev_error;
    float u = Kp * error + Ki * (*integral) + Kd * derivative;
    *prev_error = error;
    return u;
}

void pid_task(void *params) {
    float iL = 0.0f, iR = 0.0f;
    float eL = 0.0f, eR = 0.0f;
    float pwmL = PWM_MIN_LEFT, pwmR = PWM_MIN_RIGHT;
    bool jumpstarted = false;

    for (;;) {
        if (!use_pid_control) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }

        if (target_speed < MIN_SPEED) pid_state = STOP;
        if (target_speed > MAX_SPEED) target_speed = MAX_SPEED;

        float vL = get_left_speed();
        float vR = get_right_speed();

        float straight_err = (vR - vL);
        float trim = Kp_heading * straight_err;
        pwmL += trim;
        pwmR -= trim;

        // --- compute PID per wheel ---
        pwmL += compute_pid_pwm(target_speed, vL, &iL, &eL);
        pwmR += compute_pid_pwm(target_speed, vR, &iR, &eR);

        pwmL += compute_pid_pwm(target_speed, vL, &iL, &eL);
        pwmR += compute_pid_pwm(target_speed, vR, &iR, &eR);

        if (vL < JUMPSTART_SPEED_THRESHOLD || vR < JUMPSTART_SPEED_THRESHOLD) {
            pwmL = PWM_JUMPSTART; pwmR = PWM_JUMPSTART; jumpstarted = true;
        } else {
            if (pwmL < PWM_MIN_LEFT || jumpstarted)  pwmL = PWM_MIN_LEFT;
            else if (pwmL > PWM_MAX_LEFT)            pwmL = PWM_MAX_LEFT;

            if (pwmR < PWM_MIN_RIGHT || jumpstarted) pwmR = PWM_MIN_RIGHT;
            else if (pwmR > PWM_MAX_RIGHT)           pwmR = PWM_MAX_RIGHT;

            jumpstarted = false;
        }

        switch (pid_state) {
            case FORWARD:    forward_motor(pwmL, pwmR); break;
            case REVERSE:    reverse_motor(pwmL, pwmR); break;
            case LEFT_TURN:  turn_motor(LEFT,  pwmL, pwmR); reset_encoders(); pid_state = TURNING; break;
            case RIGHT_TURN: turn_motor(RIGHT, pwmL, pwmR); reset_encoders(); pid_state = TURNING; break;
            case TURNING:    turn_until_angle(target_turn_angle); break;
            case STOP:       stop_motor(); break;
            case DISABLED:
            default:         disable_pid_control(); break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =================== Hardware init ===================
void motor_pwm_init(void) {
    // Put all IN pins into PWM mode (so we can PWM either A or B)
    gpio_set_function(L_MOTOR_IN1, GPIO_FUNC_PWM);
    gpio_set_function(L_MOTOR_IN2, GPIO_FUNC_PWM);
    gpio_set_function(R_MOTOR_IN3, GPIO_FUNC_PWM);
    gpio_set_function(R_MOTOR_IN4, GPIO_FUNC_PWM);

    // Configure their slices uniformly
    uint slices[4] = {
        pwm_gpio_to_slice_num(L_MOTOR_IN1),
        pwm_gpio_to_slice_num(L_MOTOR_IN2),
        pwm_gpio_to_slice_num(R_MOTOR_IN3),
        pwm_gpio_to_slice_num(R_MOTOR_IN4)
    };
    for (int i = 0; i < 4; i++) {
        pwm_set_wrap(slices[i], PWM_TOP);
        pwm_set_clkdiv(slices[i], 125);
        pwm_set_enabled(slices[i], true);
    }

    brake_coast();
}

void motor_init(void) {
#ifdef MOTOR_STBY
    gpio_init(MOTOR_STBY);
    gpio_set_dir(MOTOR_STBY, GPIO_OUT);
    gpio_put(MOTOR_STBY, 1);
#endif

    motor_pwm_init();

    // Start PID task (safe: yields when disabled)
    xTaskCreate(pid_task, "PID Task", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
}

// Optional conditioning
void motor_conditioning(void) {
    printf("[MOTOR/CONDITIONING] Running motor conditioning.\n");
    stop_motor();
    forward_motor(PWM_JUMPSTART, PWM_JUMPSTART);
    sleep_ms(15000);
    printf("[MOTOR/CONDITIONING] Reversing motor conditioning.\n");
    reverse_motor(PWM_JUMPSTART, PWM_JUMPSTART);
    sleep_ms(15000);
    stop_motor();
    printf("[MOTOR/CONDITIONING] Motor conditioning complete.\n");
}
