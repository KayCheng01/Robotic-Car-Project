/**
 * Demo 3: Obstacle Detection + Avoidance + Telemetry
 *
 * Purpose:
 *   Validates obstacle detection (ultrasonic), servo-based width measurement,
 *   intelligent path selection, avoidance maneuver, and line recovery.
 *
 * State Machine:
 *   1. LINE_FOLLOW      → Normal line following with IR sensors + IMU
 *   2. OBSTACLE_DETECT  → Ultrasonic detects obstacle ahead, stop motors
 *   3. SCANNING         → Servo sweeps left/center/right, measure distances
 *   4. PLANNING         → Decide avoidance path (left or right)
 *   5. AVOIDING         → Execute turn + forward movement
 *   6. SEARCHING        → Look for line again with IR
 *   7. RESUME           → Resume line following
 *
 * Telemetry logged: speed, distance, IMU heading, line events, obstacle encounters
 */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/timer.h"

#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "ir.h"
#include "ultrasonic.h"
#include "servo.h"

// Internal demo3 modules
#include "demo3_obstacle.h"
#include "demo3_line_recovery.h"

// ============== Configuration ==============

// Robot geometry (from main.c)
#define TRACK_WIDTH_M        0.115f
#define WHEEL_RADIUS_M       0.032f

// Heading PID (outer loop)
#define KP_HEADING           0.30f
#define KI_HEADING           0.00f
#define KD_HEADING           0.18f
#define HEADING_DEADBAND_DEG 2.0f
#define HDG_EMA_ALPHA        0.20f
#define HEADING_OFFSET_DEG   +20.0f

// Wheel speed PIDs (inner loops)
#define SPID_KP              0.90f
#define SPID_KI              0.15f
#define SPID_KD              0.00f

// Speed command
#define V_TARGET             0.20f   // m/s
#define V_START              0.00f
#define RAMP_TIME_MS         1500

// Control timing
#define LOOP_DT_MS           10
#define DT_S                 ((float)LOOP_DT_MS / 1000.0f)

// Obstacle detection
#define OBSTACLE_THRESHOLD_CM 25.0f

// ============== Utility Functions ==============

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float ema(float prev, float x, float a) {
    return prev * (1.f - a) + x * a;
}

static inline float wrap_deg_pm180(float e) {
    while (e > 180.f) e -= 360.f;
    while (e < -180.f) e += 360.f;
    return e;
}

static inline float wrap_deg_0_360(float e) {
    while (e < 0.f) e += 360.f;
    while (e >= 360.f) e -= 360.f;
    return e;
}

static inline float cmps_to_rads(float cmps) {
    float mps = cmps / 100.0f;
    return mps / WHEEL_RADIUS_M;
}

// ============== PID Controller ==============

typedef struct {
    float kp, ki, kd;
    float integ;
    float prev_err;
    float out_min, out_max;
    float integ_min, integ_max;
} pid_ctrl_t;

static inline void pid_init(pid_ctrl_t* p, float kp, float ki, float kd,
                            float out_min, float out_max, float integ_min, float integ_max) {
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integ = 0.f; p->prev_err = 0.f;
    p->out_min = out_min; p->out_max = out_max;
    p->integ_min = integ_min; p->integ_max = integ_max;
}

static inline float pid_update(pid_ctrl_t* p, float err, float dt) {
    p->integ += err * dt;
    p->integ = clampf(p->integ, p->integ_min, p->integ_max);

    float deriv = (err - p->prev_err) / dt;
    p->prev_err = err;

    float u = p->kp * err + p->ki * p->integ + p->kd * deriv;
    return clampf(u, p->out_min, p->out_max);
}

// ============== Telemetry Context ==============
// Note: demo3_state_t is defined in demo3_obstacle.h

typedef struct {
    demo3_state_t state;
    demo3_state_t prev_state;
    uint32_t state_entry_time_ms;
    
    // Telemetry
    float target_heading_deg;
    float heading_deg;
    float speed_left_cmps;
    float speed_right_cmps;
    float speed_avg_cmps;
    float distance_traveled_cm;
    uint32_t obstacle_count;
    
    // Obstacle data
    obstacle_data_t obstacle;
    int avoidance_side;
    float avoidance_turn_angle_deg;
    float avoidance_forward_dist_cm;
} demo3_context_t;

static demo3_context_t g_ctx = {0};
static imu_t g_imu = {0};
static pid_ctrl_t g_pid_speed_L, g_pid_speed_R;

// ============== State Transitions ==============

static void demo3_change_state(demo3_state_t new_state) {
    if (g_ctx.state != new_state) {
        g_ctx.prev_state = g_ctx.state;
        g_ctx.state = new_state;
        g_ctx.state_entry_time_ms = to_ms_since_boot(get_absolute_time());
        
        const char *state_names[] = {
            "LINE_FOLLOW", "OBSTACLE_DETECT", "SCANNING", 
            "PLANNING", "AVOIDING", "SEARCHING", "RESUME"
        };
        printf("[DEMO3:STATE] %s → %s\n",
               state_names[g_ctx.prev_state],
               state_names[new_state]);
    }
}

// ============== Main Control Loop Logic ==============

static void demo3_line_follow_step(void) {
    // Standard line following with IMU heading correction
    // Check for obstacle
    float dist = ultrasonic_get_distance_cm();
    
    if (dist > 0 && dist < OBSTACLE_THRESHOLD_CM) {
        printf("[DEMO3] Obstacle detected at %.1f cm!\n", (double)dist);
        g_ctx.obstacle_count++;
        demo3_change_state(STATE_OBSTACLE_DETECT);
        stop_motor_manual();
        sleep_ms(200);
    } else {
        // Continue line following: use IR sensors for lateral control
        // and IMU for heading stability
        int ir_left = ir_left_is_black();
        int ir_right = ir_right_is_black();
        
        // Simple steering: if both sensors see line, go straight
        // If one sensor lost line, correct
        float steering_correction = 0.0f;
        if (ir_left && !ir_right) {
            steering_correction = -0.1f;  // Correct left
        } else if (!ir_left && ir_right) {
            steering_correction = +0.1f;  // Correct right
        }
        
        // Apply speed command with steering
        float v_cmd = V_TARGET;
        float w_cmd = steering_correction;  // Angular velocity
        
        float wL_target = (v_cmd - 0.5f * TRACK_WIDTH_M * w_cmd) / WHEEL_RADIUS_M;
        float wR_target = (v_cmd + 0.5f * TRACK_WIDTH_M * w_cmd) / WHEEL_RADIUS_M;
        
        // Get measured speeds
        float L_cmps = get_left_speed();
        float R_cmps = get_right_speed();
        if (L_cmps < 0.0f) L_cmps = 0.0f;
        if (R_cmps < 0.0f) R_cmps = 0.0f;
        
        float wL_meas = cmps_to_rads(L_cmps);
        float wR_meas = cmps_to_rads(R_cmps);
        
        // Inner PIDs
        float uL = pid_update(&g_pid_speed_L, (wL_target - wL_meas), DT_S);
        float uR = pid_update(&g_pid_speed_R, (wR_target - wR_meas), DT_S);
        
        int pwmL = (int)lroundf(clampf(uL, PWM_MIN_LEFT, PWM_MAX_LEFT));
        int pwmR = (int)lroundf(clampf(uR, PWM_MIN_RIGHT, PWM_MAX_RIGHT));
        
        forward_motor_manual(pwmL, pwmR);
        
        g_ctx.speed_left_cmps = L_cmps;
        g_ctx.speed_right_cmps = R_cmps;
        g_ctx.speed_avg_cmps = (L_cmps + R_cmps) / 2.0f;
    }
}

static void demo3_obstacle_detect_step(void) {
    // Already stopped, now scan
    sleep_ms(200);
    demo3_change_state(STATE_SCANNING);
}

static void demo3_scanning_step(void) {
    demo3_scan_obstacle(&g_ctx.obstacle);
    g_ctx.avoidance_side = demo3_choose_avoidance_side(&g_ctx.obstacle);
    demo3_log_obstacle(&g_ctx.obstacle);
    demo3_change_state(STATE_PLANNING);
}

static void demo3_planning_step(void) {
    demo3_plan_avoidance(&g_ctx.obstacle,
                        &g_ctx.avoidance_turn_angle_deg,
                        &g_ctx.avoidance_forward_dist_cm,
                        NULL);
    demo3_change_state(STATE_AVOIDING);
}

static void demo3_avoiding_step(void) {
    printf("[DEMO3:AVOID] Executing avoidance: turn=%.1f°, forward=%.1f cm\n",
           (double)g_ctx.avoidance_turn_angle_deg,
           (double)g_ctx.avoidance_forward_dist_cm);
    
    // 1. Turn (estimated)
    // Using simple timing-based turn (tune based on robot)
    float turn_time_ms = fabs(g_ctx.avoidance_turn_angle_deg) / 90.0f * 1000.0f;  // ~1 sec per 90°
    if (g_ctx.avoidance_turn_angle_deg > 0) {
        turn_motor_manual(0, g_ctx.avoidance_turn_angle_deg, 120, 120);  // Left turn
    } else {
        turn_motor_manual(1, -g_ctx.avoidance_turn_angle_deg, 120, 120);  // Right turn
    }
    sleep_ms((uint32_t)turn_time_ms);
    
    // 2. Move forward (estimated)
    // Using timing-based distance (0.20 m/s = 20 cm/s → 100ms per 20cm)
    float move_time_ms = (g_ctx.avoidance_forward_dist_cm / 20.0f) * 1000.0f;
    forward_motor_manual(120, 120);
    sleep_ms((uint32_t)move_time_ms);
    
    stop_motor_manual();
    sleep_ms(300);
    
    printf("[DEMO3:AVOID] Avoidance complete, searching for line...\n");
    demo3_change_state(STATE_SEARCHING);
}

static void demo3_searching_step(void) {
    if (demo3_search_for_line()) {
        printf("[DEMO3:SEARCH] Line found!\n");
        demo3_change_state(STATE_RESUME);
    } else {
        printf("[DEMO3:SEARCH] Line NOT found, attempting recovery...\n");
        // In a more robust system, we'd have additional strategies here
        demo3_change_state(STATE_RESUME);
    }
}

static void demo3_resume_step(void) {
    demo3_resume_line_following();
    demo3_change_state(STATE_LINE_FOLLOW);
}

// ============== Main Function ==============

int main(void) {
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(10000);
    
    printf("\n[DEMO3] ========== DEMO 3: OBSTACLE AVOIDANCE ==========\n");
    printf("[DEMO3] Version: 1.0 (IMU + IR + Ultrasonic + Servo)\n");
    printf("[DEMO3] Compiled: " __DATE__ " " __TIME__ "\n\n");
    
    // ---- Initialize IMU ----
    g_imu.i2c = i2c1;
    g_imu.i2c_baud = IMU_I2C_BAUD;
    g_imu.pin_sda = IMU_SDA_PIN;
    g_imu.pin_scl = IMU_SCL_PIN;
    g_imu.mx_off = g_imu.my_off = g_imu.mz_off = 0.f;
    
    if (!imu_init(&g_imu)) {
        printf("[DEMO3] IMU init failed!\n");
        while (1) sleep_ms(500);
    }
    printf("[DEMO3] IMU OK\n");
    
    // ---- Initialize motors ----
    motor_init();
    printf("[DEMO3] Motors OK\n");
    
    // ---- Initialize encoders ----
    encoder_init();
    printf("[DEMO3] Encoders OK\n");
    
    // ---- Initialize IR sensors ----
    ir_init();
    printf("[DEMO3] IR sensors OK\n");
    
    // ---- Initialize obstacle detection ----
    demo3_obstacle_init();
    printf("[DEMO3] Obstacle detection OK\n");
    
    // ---- Set target heading ----
    float filt_hdg = 0.f;
    for (int i = 0; i < 20; ++i) {
        float h = imu_update_and_get_heading(&g_imu);
        h += HEADING_OFFSET_DEG;
        h = wrap_deg_0_360(h);
        filt_hdg = ema(filt_hdg, h, 0.20f);
        sleep_ms(10);
    }
    g_ctx.target_heading_deg = filt_hdg;
    printf("[DEMO3] Target heading = %.1f°\n", (double)g_ctx.target_heading_deg);
    
    // ---- Initialize PIDs ----
    const float iwind = 300.0f;
    pid_init(&g_pid_speed_L, SPID_KP, SPID_KI, SPID_KD,
             PWM_MIN_LEFT, PWM_MAX_LEFT, -iwind, +iwind);
    pid_init(&g_pid_speed_R, SPID_KP, SPID_KI, SPID_KD,
             PWM_MIN_RIGHT, PWM_MAX_RIGHT, -iwind, +iwind);
    
    printf("[DEMO3] Ready! Starting in 3 seconds...\n");
    sleep_ms(3000);
    
    // ---- Main state machine loop ----
    g_ctx.state = STATE_LINE_FOLLOW;
    g_ctx.state_entry_time_ms = to_ms_since_boot(get_absolute_time());
    g_ctx.obstacle_count = 0;
    
    absolute_time_t t_next = make_timeout_time_ms(LOOP_DT_MS);
    static int telem_div = 0;
    
    while (true) {
        // Update IMU
        float raw_hdg = imu_update_and_get_heading(&g_imu);
        raw_hdg += HEADING_OFFSET_DEG;
        raw_hdg = wrap_deg_0_360(raw_hdg);
        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);
        g_ctx.heading_deg = filt_hdg;
        
        // State machine
        switch (g_ctx.state) {
            case STATE_LINE_FOLLOW:
                demo3_line_follow_step();
                break;
            case STATE_OBSTACLE_DETECT:
                demo3_obstacle_detect_step();
                break;
            case STATE_SCANNING:
                demo3_scanning_step();
                break;
            case STATE_PLANNING:
                demo3_planning_step();
                break;
            case STATE_AVOIDING:
                demo3_avoiding_step();
                break;
            case STATE_SEARCHING:
                demo3_searching_step();
                break;
            case STATE_RESUME:
                demo3_resume_step();
                break;
        }
        
        // Telemetry (5 Hz)
        if ((telem_div++ % (1000 / LOOP_DT_MS / 5)) == 0) {
            const char *state_names[] = {
                "LINE_FOLLOW", "OBSTACLE_DETECT", "SCANNING",
                "PLANNING", "AVOIDING", "SEARCHING", "RESUME"
            };
            printf("[DEMO3:TELEM] State=%s Hdg=%.1f° Speed=%.1f cm/s Obstacles=%u\n",
                   state_names[g_ctx.state],
                   (double)g_ctx.heading_deg,
                   (double)g_ctx.speed_avg_cmps,
                   g_ctx.obstacle_count);
        }
        
        // Loop timing
        sleep_until(t_next);
        t_next = delayed_by_ms(t_next, LOOP_DT_MS);
    }
    
    return 0;
}
