#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "encoder.h"   // for LEFT/RIGHT, distances, etc.

// ---------------- Pin Map (your request) ----------------
#define L_MOTOR_IN1 11   // M1A (Left A)   <-- PWM
#define L_MOTOR_IN2 10   // M1B (Left B)   <-- PWM
#define R_MOTOR_IN3 8    // M2A (Right A)  <-- PWM
#define R_MOTOR_IN4 9    // M2B (Right B)  <-- PWM

// Optional: if your board has a standby pin, define and wire it; otherwise leave undefined
// #define MOTOR_STBY 22

// ---------------- PID & PWM constants (tune later) ----------------
#define Kp 0.80f
#define Ki 0.00f
#define Kd 0.00f

#define PWM_MIN_LEFT     3
#define PWM_MIN_RIGHT    3
#define PWM_MID_LEFT     5
#define PWM_MID_RIGHT    5
#define PWM_MAX_LEFT     10
#define PWM_MAX_RIGHT    10
#define PWM_JUMPSTART    6  // a bit higher so wheels can kickstart

#define MIN_SPEED                 2.0f
#define MAX_SPEED                 5.0f
#define TURN_SPEED                5.0f
#define JUMPSTART_SPEED_THRESHOLD 2.0f

#define PI 3.14159265358979323846
#define FULL_CIRCLE 360.0f
#define CONTINUOUS_TURN -1.0f

typedef enum {
    STOP,
    FORWARD,
    REVERSE,
    LEFT_TURN,
    RIGHT_TURN,
    TURNING,
    DISABLED
} PIDState;

// Init
void motor_init(void);
void motor_pwm_init(void);

// Movement (used by manual + PID)
void forward_motor(float new_pwm_left, float new_pwm_right);
void reverse_motor(float new_pwm_left, float new_pwm_right);
void turn_motor(int direction, float new_pwm_left, float new_pwm_right);
void stop_motor(void);
bool turn_until_angle(float angle);

// Manual wrappers (what you call in main.c)
void disable_pid_control(void);
void forward_motor_manual(float new_pwm_left, float new_pwm_right);
void reverse_motor_manual(float new_pwm_left, float new_pwm_right);
void turn_motor_manual(int direction, float angle, float new_pwm_left, float new_pwm_right);
void stop_motor_manual(void);
void offset_move_motor(int direction, int turn, float offset);

// PID interface (optional)
void enable_pid_control(void);
void forward_motor_pid(float new_target_speed);
void reverse_motor_pid(float new_target_speed);
void turn_motor_pid(int direction, float new_target_speed, float new_target_turn_angle);
void stop_motor_pid(void);
float compute_pid_pwm(float target_speed, float current_value, float *integral, float *prev_error);
void pid_task(void *params);

void motor_conditioning(void);

#endif // MOTOR_H
