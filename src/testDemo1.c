// demo1_freertos_mqtt_single.c
// One-file Demo 1: FreeRTOS control loop + Wi-Fi + MQTT telemetry
// - Keeps testDemo1 behavior (V=0.20 m/s, +20° IMU offset, wheel trims)
// - 10 ms control loop, ~5 Hz telemetry (USB + MQTT JSON)
//
// Based on your testDemo1 control logic and telemetry cadence, adapted to FreeRTOS
// and integrated with a lightweight MQTT reconnect loop.

/* ============================ Includes ============================ */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"

#include "imu.h"
#include "motor.h"
#include "encoder.h"

/* ============================ User Config ========================= */
// --- Wi-Fi & MQTT ---
#define WIFI_SSID            "iPhone"
#define WIFI_PASS            "yo1234567"
#define WIFI_CONNECT_TIMEOUT_MS 20000

#define BROKER_IP_STR        "172.20.10.3"
#define BROKER_PORT          1883
#define MQTT_TOPIC_TELEM     "pico/demo1/telemetry"

// --- Robot geometry ---
#define TRACK_WIDTH_M        0.115f   // wheel center-to-center (m)

// --- Per-wheel trims (like your testDemo1) ---
#ifndef WHEEL_TRIM_LEFT
#define WHEEL_TRIM_LEFT      (-2)
#endif
#ifndef WHEEL_TRIM_RIGHT
#define WHEEL_TRIM_RIGHT     (+8)
#endif

// --- Target speed (same as testDemo1 = 0.20 m/s) ---
#define V_TARGET_MPS         0.20f
#define V_TARGET_CMPS        (V_TARGET_MPS * 100.0f)

// --- Heading PID - Safe mode (minimal correction, just go straight) ---
#define KP_HEADING_SAFE      0.15f    // Very gentle - just keep straight
#define KI_HEADING_SAFE      0.00f    // No integral in safe mode
#define KD_HEADING_SAFE      0.10f    // Light damping

// --- Heading PID - Recovery mode (return to initial direction) ---
#define KP_HEADING_RECOVERY  2.50f    // VERY strong correction to get back on track
#define KI_HEADING_RECOVERY  0.15f    // Higher integral to eliminate steady-state error quickly
#define KD_HEADING_RECOVERY  0.80f    // High derivative to prevent overshoot

// --- Mode switching thresholds ---
#define HEADING_DEADBAND_DEG     2.0f   // Deadband in safe mode
#define RECOVERY_TRIGGER_DEG     10.0f  // Enter recovery if deviation > 10° from initial
#define SAFE_RETURN_DEG          3.0f   // Return to safe mode if within 3° of initial

#define HDG_EMA_ALPHA            0.20f
#define DEG2RAD                  (float)(M_PI / 180.0f)
#define HEADING_OFFSET_DEG       20.0f
#define HEADING_RATE_SCALE_SAFE     0.015f  // Very gentle in safe mode
#define HEADING_RATE_SCALE_RECOVERY 0.15f   // VERY aggressive in recovery mode

// --- Wheel-speed inner PID (same as testDemo1) ---
#define SPID_OUT_MIN         0.0f
#ifndef PWM_MAX_RIGHT
#define PWM_MAX_RIGHT        255
#endif
#define SPID_OUT_MAX         (float)(PWM_MAX_RIGHT)
#define SPID_KP              6.0f
#define SPID_KI              0.8f
#define SPID_KD              0.0f
#define SPID_IWIND_CLAMP     300.0f

// --- Encoder straightness PI (same as testDemo1) ---
#define STRAIGHT_KP          1.2f
#define STRAIGHT_KI          0.30f
#define STRAIGHT_I_CLAMP     50.0f

// --- Base & Slew (as in your code) ---
#ifndef PWM_MIN_LEFT
#define PWM_MIN_LEFT         80
#endif
#ifndef PWM_MIN_RIGHT
#define PWM_MIN_RIGHT        80
#endif
#ifndef PWM_MAX_LEFT
#define PWM_MAX_LEFT         255
#endif
#define BASE_PWM_L           (PWM_MIN_LEFT)
#define BASE_PWM_R           (PWM_MIN_RIGHT)
#define MAX_PWM_STEP         6

// --- Timing ---
#define LOOP_DT_MS           10
#define DT_S                 ((float)LOOP_DT_MS / 1000.0f)

/* ============================ Helpers ============================ */
static inline float clampf(float v, float lo, float hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline int clampi(int v, int lo, int hi){
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static inline float ema(float prev, float x, float a){
    return prev*(1.f - a) + x*a;
}
static inline float wrap_deg_pm180(float e){
    while (e > 180.f) e -= 360.f;
    while (e < -180.f) e += 360.f;
    return e;
}
static inline float wrap_deg_0_360(float e){
    while (e < 0.f) e += 360.f;
    while (e >= 360.f) e -= 360.f;
    return e;
}

/* ============================ Simple PID ============================ */
typedef struct {
    float kp, ki, kd;
    float integ, prev_err;
    float out_min, out_max;
    float integ_min, integ_max;
} pid_ctrl_t;

static inline void pid_init(pid_ctrl_t* p, float kp, float ki, float kd,
                            float out_min, float out_max, float integ_min, float integ_max){
    p->kp = kp; p->ki = ki; p->kd = kd;
    p->integ = 0.f; p->prev_err = 0.f;
    p->out_min = out_min; p->out_max = out_max;
    p->integ_min = integ_min; p->integ_max = integ_max;
}
static inline float pid_update(pid_ctrl_t* p, float err, float dt){
    p->integ += err * dt;
    p->integ = clampf(p->integ, p->integ_min, p->integ_max);
    float deriv = (err - p->prev_err) / dt;
    p->prev_err = err;
    float u = p->kp * err + p->ki * p->integ + p->kd * deriv;
    return clampf(u, p->out_min, p->out_max);
}

/* ============================ MQTT / Wi-Fi ============================ */
static mqtt_client_t *g_mqtt = NULL;
static ip_addr_t      g_broker_ip;
static SemaphoreHandle_t g_mqtt_mutex;   // protect publish

static void mqtt_pub_cb(void *arg, err_t result){
    (void)arg;
    // Optional: printf("[MQTT] publish result=%d\n", result);
}
static void mqtt_conn_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status){
    (void)arg;
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("[MQTT] Connected\n");
    } else {
        printf("[MQTT] Disconnected, status=%d\n", (int)status);
    }
}
static bool wifi_connect_blocking(uint32_t timeout_ms) {
    printf("[NET] Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);
    int err = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, timeout_ms);
    if (err) {
        printf("[NET] Wi-Fi connect failed (err=%d)\n", err);
        return false;
    }
    printf("[NET] Wi-Fi connected, IP acquired\n");
    return true;
}
static bool mqtt_connect_blocking(void) {
    if (!g_mqtt) g_mqtt = mqtt_client_new();
    if (!g_mqtt) { printf("[MQTT] client_new failed\n"); return false; }

    struct mqtt_connect_client_info_t ci = {0};
    ci.client_id   = "pico-demo1";
    ci.keep_alive  = 30;
    ci.will_topic  = NULL;
    ci.will_msg    = NULL;

    err_t er = mqtt_client_connect(g_mqtt, &g_broker_ip, BROKER_PORT, mqtt_conn_cb, NULL, &ci);
    if (er != ERR_OK) {
        printf("[MQTT] connect err=%d\n", er);
        return false;
    }
    // Wait briefly for state (non-blocking API). In practice you might poll is_connected in a loop.
    vTaskDelay(pdMS_TO_TICKS(500));
    return mqtt_client_is_connected(g_mqtt);
}
static bool mqtt_is_connected(void){
    return g_mqtt && mqtt_client_is_connected(g_mqtt);
}
static err_t mqtt_publish_str(const char *topic, const char *payload){
    if (!mqtt_is_connected()) return ERR_CONN;
    if (g_mqtt_mutex) xSemaphoreTake(g_mqtt_mutex, portMAX_DELAY);
    cyw43_arch_lwip_begin();
    err_t r = mqtt_publish(g_mqtt, topic,
                           (const u8_t*)payload, (u16_t)strlen(payload),
                           0 /*qos0*/, 0 /*retain*/, mqtt_pub_cb, NULL);
    cyw43_arch_lwip_end();
    if (g_mqtt_mutex) xSemaphoreGive(g_mqtt_mutex);
    return r;
}
static void vNetworkTask(void *param){
    (void)param;

    if (cyw43_arch_init()) {
        printf("[NET] cyw43 init failed\n");
        vTaskDelete(NULL);
        return;
    }
    cyw43_wifi_pm(&cyw43_state, CYW43_NO_POWERSAVE_MODE);
    cyw43_arch_enable_sta_mode();

    while (!wifi_connect_blocking(WIFI_CONNECT_TIMEOUT_MS)) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    // Resolve broker IP (literal dotted-quad here)
    ip4_addr_set_u32(ip_2_ip4(&g_broker_ip), ipaddr_addr(BROKER_IP_STR));
    printf("[NET] Broker %s:%d\n", BROKER_IP_STR, BROKER_PORT);

    for (;;) {
        if (!mqtt_is_connected()) {
            printf("[MQTT] Connecting...\n");
            mqtt_connect_blocking();
        }
        // blink LED as heartbeat
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(750));
    }
}

/* ============================ Control Task ============================ */
static void vDriveTask(void *pvParameters) {
    (void)pvParameters;

    // ---- IMU ----
    imu_t imu;
    imu.i2c      = i2c1;
    imu.i2c_baud = IMU_I2C_BAUD;
    imu.pin_sda  = IMU_SDA_PIN;
    imu.pin_scl  = IMU_SCL_PIN;
    imu.mx_off = imu.my_off = imu.mz_off = 0.f;
    if (!imu_init(&imu)) {
        printf("[CTRL] IMU init failed\n");
        vTaskDelete(NULL);
    }
    printf("[CTRL] IMU OK\n");

    // ---- Motors & encoders ----
    motor_init();
    encoder_init();
    printf("[CTRL] Motors & Encoders OK  (PWM L[%d..%d] R[%d..%d])\n",
           PWM_MIN_LEFT, PWM_MAX_LEFT, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

    // ---- Wheel speed PIDs ----
    pid_ctrl_t pidL, pidR;
    pid_init(&pidL, SPID_KP, SPID_KI, SPID_KD,
             SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);
    pid_init(&pidR, SPID_KP, SPID_KI, SPID_KD,
             SPID_OUT_MIN, SPID_OUT_MAX, -SPID_IWIND_CLAMP, +SPID_IWIND_CLAMP);

    // ---- Capture target heading (average of multiple readings) ----
    printf("[CTRL] Capturing initial heading...\n");
    float heading_sum = 0.0f;
    int num_samples = 50;  // Take 50 samples over 500ms
    
    for (int i = 0; i < num_samples; ++i){
        float h = imu_update_and_get_heading(&imu);
        h += HEADING_OFFSET_DEG;
        h = wrap_deg_0_360(h);
        heading_sum += h;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    const float target_heading_deg = heading_sum / (float)num_samples;
    printf("[CTRL] Target heading = %.1f deg (averaged from %d samples)\n", 
           (double)target_heading_deg, num_samples);
    
    // Initialize filtered heading to target
    float filt_hdg = target_heading_deg;

    // ---- Controllers' state ----
    float h_integ = 0.f, h_prev_err = 0.f; // outer IMU PID
    float s_int   = 0.0f;                  // straightness PI
    int   lastL   = PWM_MIN_LEFT;
    int   lastR   = PWM_MIN_RIGHT;
    
    // ---- Correction mode tracking ----
    bool recovery_mode = false;  // false = safe mode, true = recovery mode
    float safe_mode_reference = filt_hdg;  // Track current heading in safe mode

    // ---- Telemetry running state ----
    float dist_cm = 0.0f;

    // ---- Task timing ----
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        // === 1) IMU heading (raw + filtered) ===
        float raw_hdg = imu_update_and_get_heading(&imu);
        raw_hdg += HEADING_OFFSET_DEG;
        raw_hdg = wrap_deg_0_360(raw_hdg);
        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);

        // === Determine deviation from INITIAL heading ===
        float deviation_from_initial = wrap_deg_pm180(target_heading_deg - filt_hdg);
        float abs_deviation = fabsf(deviation_from_initial);
        
        // === Mode switching logic ===
        if (!recovery_mode && abs_deviation > RECOVERY_TRIGGER_DEG) {
            // Large deviation detected - enter RECOVERY mode
            recovery_mode = true;
            printf("[CTRL] *** RECOVERY MODE ENGAGED (deviation=%.1f deg from initial) ***\n", (double)abs_deviation);
            // Reset integral when switching modes to prevent windup
            h_integ = 0.0f;
        } else if (recovery_mode && abs_deviation < SAFE_RETURN_DEG) {
            // Back on track - return to SAFE mode
            recovery_mode = false;
            safe_mode_reference = filt_hdg;  // Update safe mode reference to current heading
            printf("[CTRL] --- Safe mode restored (deviation=%.1f deg) ---\n", (double)abs_deviation);
            // Reset integral when switching modes
            h_integ = 0.0f;
        }
        
        // === Calculate heading error based on mode ===
        float h_err;
        if (recovery_mode) {
            // RECOVERY: steer back to initial heading
            h_err = deviation_from_initial;
        } else {
            // SAFE: just go straight (track current heading, update reference gradually)
            safe_mode_reference = ema(safe_mode_reference, filt_hdg, 0.05f);  // Slowly adapt
            h_err = wrap_deg_pm180(safe_mode_reference - filt_hdg);
            // Apply deadband in safe mode
            if (fabsf(h_err) < HEADING_DEADBAND_DEG) {
                h_err = 0.f;
            }
        }
        
        // Select PID gains based on mode
        float kp = recovery_mode ? KP_HEADING_RECOVERY : KP_HEADING_SAFE;
        float ki = recovery_mode ? KI_HEADING_RECOVERY : KI_HEADING_SAFE;
        float kd = recovery_mode ? KD_HEADING_RECOVERY : KD_HEADING_SAFE;
        float rate_scale = recovery_mode ? HEADING_RATE_SCALE_RECOVERY : HEADING_RATE_SCALE_SAFE;

        float h_deriv = (h_err - h_prev_err) / DT_S;
        h_prev_err = h_err;

        // Integral with anti-windup
        if (ki > 0.f){
            float h_iw = 150.0f / fmaxf(ki, 1e-6f);
            h_integ += h_err * DT_S;
            h_integ = clampf(h_integ, -h_iw, +h_iw);
        }

        float delta_heading_rate_deg_s = kp * h_err + ki * h_integ + kd * h_deriv;
        float delta_w = delta_heading_rate_deg_s * DEG2RAD * rate_scale;

        // === 2) Command v (cm/s) ===
        const float v_cmd_cmps = V_TARGET_CMPS;

        // === 3) (v, delta_w) -> left/right targets (cm/s) ===
        float diff_cmps = (0.5f * TRACK_WIDTH_M * delta_w) * 100.0f;
        float vL_target = v_cmd_cmps - diff_cmps;
        float vR_target = v_cmd_cmps + diff_cmps;

        // === 4) Measure speeds (cm/s) ===
        float vL_meas = get_left_speed();
        float vR_meas = get_right_speed();
        static float vL_prev = 0.0f, vR_prev = 0.0f;
        if (vL_meas < 0.0f) vL_meas = vL_prev; else vL_prev = vL_meas;
        if (vR_meas < 0.0f) vR_meas = vR_prev; else vR_prev = vR_meas;

        // integrate distance (cm)
        float v_avg = 0.5f * (vL_meas + vR_meas);
        dist_cm += v_avg * DT_S;

        // === 5) Inner wheel speed PIDs -> delta-PWM ===
        float uL = pid_update(&pidL, (vL_target - vL_meas), DT_S);
        float uR = pid_update(&pidR, (vR_target - vR_meas), DT_S);

        // === 6) Straightness PI ===
        float s_err = (vR_meas - vL_meas);    // cm/s mismatch
        s_int += s_err * DT_S;
        s_int = clampf(s_int, -STRAIGHT_I_CLAMP, STRAIGHT_I_CLAMP);
        float s_trim = STRAIGHT_KP * s_err + STRAIGHT_KI * s_int;

        // === 7) Compose PWM = BASE + PID delta + trims, then clamp ===
        int pwmL = (int)lroundf(BASE_PWM_L + uL + WHEEL_TRIM_LEFT  + s_trim);
        int pwmR = (int)lroundf(BASE_PWM_R + uR + WHEEL_TRIM_RIGHT - s_trim);
        pwmL = clampi(pwmL, PWM_MIN_LEFT,  PWM_MAX_LEFT);
        pwmR = clampi(pwmR, PWM_MIN_RIGHT, PWM_MAX_RIGHT);

        // === 8) Slew limit ===
        int dL = pwmL - lastL, dR = pwmR - lastR;
        if (dL >  MAX_PWM_STEP) pwmL = lastL + MAX_PWM_STEP;
        if (dL < -MAX_PWM_STEP) pwmL = lastL - MAX_PWM_STEP;
        if (dR >  MAX_PWM_STEP) pwmR = lastR + MAX_PWM_STEP;
        if (dR < -MAX_PWM_STEP) pwmR = lastR - MAX_PWM_STEP;

        forward_motor_manual(pwmL, pwmR);
        lastL = pwmL; lastR = pwmR;

        // === 9) Telemetry (~5 Hz) ===
        static int div = 0;
        if ((div++ % (1000/LOOP_DT_MS/5)) == 0) {
            // USB serial (human-readable)
            printf("[CTRL] %s hdg=%.1f(raw=%.1f) dev=%.1f herr=%.2f dW=%.3f  v=%.1f  "
                   "vL[t/m]=%.2f/%.2f  vR[t/m]=%.2f/%.2f  dist=%.1f  "
                   "s_err=%.2f s_int=%.2f str=%.2f  PWM[L=%d R=%d]\n",
                   recovery_mode ? "[RCV]" : "[SAFE]",
                   (double)filt_hdg, (double)raw_hdg, (double)abs_deviation, (double)h_err, (double)delta_w,
                   (double)v_cmd_cmps,
                   (double)vL_target, (double)vL_meas,
                   (double)vR_target, (double)vR_meas,
                   (double)dist_cm,
                   (double)s_err, (double)s_int, (double)s_trim,
                   pwmL, pwmR);

            // MQTT JSON (machine-readable)
            if (mqtt_is_connected()) {
                char json[192];
                uint32_t ts_ms = to_ms_since_boot(get_absolute_time());
                int n = snprintf(json, sizeof(json),
                    "{\"ts\":%u,"
                     "\"vL\":%.2f,\"vR\":%.2f,\"vAvg\":%.2f,"
                     "\"dist\":%.1f,"
                     "\"hdgRaw\":%.1f,\"hdg\":%.1f}",
                    (unsigned)ts_ms,
                    (double)vL_meas, (double)vR_meas, (double)v_avg,
                    (double)dist_cm,
                    (double)raw_hdg, (double)filt_hdg);
                if (n > 0 && n < (int)sizeof(json)) {
                    err_t r = mqtt_publish_str(MQTT_TOPIC_TELEM, json);
                    if (r != ERR_OK) {
                        printf("[MQTT] publish err=%d\n", r);
                    }
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_DT_MS));
    }
}

/* ============================ Main ============================ */
int main(void){
    stdio_init_all();
    setvbuf(stdout, NULL, _IONBF, 0);
    sleep_ms(800);

    printf("\n[BOOT] Demo1 single-file: FreeRTOS + MQTT + Dual-Mode Control\n");
    printf("       Safe Mode PID(%.2f/%.2f/%.2f) | Recovery Mode PID(%.2f/%.2f/%.2f)\n",
           (double)KP_HEADING_SAFE, (double)KI_HEADING_SAFE, (double)KD_HEADING_SAFE,
           (double)KP_HEADING_RECOVERY, (double)KI_HEADING_RECOVERY, (double)KD_HEADING_RECOVERY);
    printf("       SpeedPID(%.2f/%.2f/%.2f)\n",
           (double)SPID_KP, (double)SPID_KI, (double)SPID_KD);

    g_mqtt_mutex = xSemaphoreCreateMutex();

    // Start network task (Wi-Fi + MQTT reconnect loop)
    xTaskCreate(vNetworkTask, "net", 4096, NULL, tskIDLE_PRIORITY + 3, NULL);

    // Start control loop task
    xTaskCreate(vDriveTask, "DriveTask", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);

    vTaskStartScheduler();
    while (true) {
        printf("Scheduler failed!\n");
        sleep_ms(1000);
    }
}
