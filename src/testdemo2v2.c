// Demo 2: Line Following + Barcode Navigation + Telemetry (single file)
// - One ADC line sensor on GP28 (ADC2) to follow the tape
// - One digital IR on GP7 for Barcode39 (start '*' + LETTER + end '*')
// - Barcode letter -> LEFT/RIGHT turn command via IMU heading target
// - Drive base uses Demo1-style inner speed PIDs + straightness PI
// - 10 ms loop, ~5 Hz MQTT telemetry to "pico/demo2/telemetry"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>  // <-- added

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "hardware/adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "lwip/apps/mqtt.h"

#include "motor.h"
#include "encoder.h"
#include "imu.h"        // IMU (LSM303)
#include "barcode.h"    // Barcode39 IRQ + task (DO on GP7)

// ============================ Wi-Fi + MQTT ============================
#define WIFI_SSID             "YOUR_WIFI"
#define WIFI_PASS             "YOUR_PASS"
#define WIFI_CONNECT_TIMEOUT_MS 20000

#define BROKER_IP_STR         "192.168.1.100"
#define BROKER_PORT           1883
#define MQTT_TOPIC_TELEM      "pico/demo2/telemetry"

// ============================ Pins / Sensors ==========================
// Line sensor: GP28 (ADC2)
#ifndef LINE_ADC_GPIO
#define LINE_ADC_GPIO  28
#endif
// Barcode DO pin is in barcode.h as IR_SENSOR_PIN = 7

// ============================ Line thresholds =========================
// Adjust to your floor/tape; use hysteresis band
#ifndef TH_LO
#define TH_LO   600     // clearly white / off line
#endif
#ifndef TH_HI
#define TH_HI   3000    // clearly black / on line
#endif

// ============================ Robot geometry & control =================
#define TRACK_WIDTH_M        0.115f

// Inner wheel-speed PID (same as your Demo1/testDemo1)
#define SPID_KP              6.0f
#define SPID_KI              0.8f
#define SPID_KD              0.0f
#define SPID_OUT_MIN         0.0f
#ifndef PWM_MAX_RIGHT
#define PWM_MAX_RIGHT        255
#endif
#define SPID_OUT_MAX         ((float)PWM_MAX_RIGHT)
#define SPID_IWIND_CLAMP     300.0f

// Straightness PI (encoder R-L)
#define STRAIGHT_KP          1.2f
#define STRAIGHT_KI          0.30f
#define STRAIGHT_I_CLAMP     50.0f

// Base & slew
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

// Loop timing
#define LOOP_DT_MS           10
#define DT_S                 ((float)LOOP_DT_MS / 1000.0f)

// IMU heading PID for hold/turn
#define KP_HEADING           0.30f
#define KI_HEADING           0.00f
#define KD_HEADING           0.18f
#define HEADING_DEADBAND_DEG 2.0f
#define HDG_EMA_ALPHA        0.2f
#define DEG2RAD              (float)(M_PI/180.f)
#define HEADING_RATE_SCALE   0.02f
#define HEADING_OFFSET_DEG   0.0f

// Cruise speed when following line
#define V_CRUISE_CMPS        10.0f   // ~0.10 m/s (adjust)
// Slower speed during turning
#define V_TURN_CMPS          6.0f

// ============================ Helpers ============================
static inline float clampf(float v, float a, float b){ return v<a?a:(v>b?b:v); }
static inline int   clampi(int v, int a, int b){ return v<a?a:(v>b?b:v); }
static inline float ema(float prev, float x, float a){ return prev*(1-a)+x*a; }
static inline float wrap_pm180(float e){ while(e>180) e-=360; while(e<-180) e+=360; return e; }
static inline float wrap_360(float e){ while(e<0) e+=360; while(e>=360) e-=360; return e; }
static inline bool  on_line_hys(uint16_t v){ static bool s=false; if(v>=TH_HI)s=true; else if(v<=TH_LO)s=false; return s; }

// ============================ Tiny PID type ==========================
typedef struct {
    float kp,ki,kd, integ, prev, out_lo, out_hi, iw_lo, iw_hi;
} pid_controller_t;

static inline void pid_init(pid_controller_t* p,float kp,float ki,float kd,float lo,float hi,float iw){
    p->kp=kp; p->ki=ki; p->kd=kd; p->integ=0; p->prev=0; p->out_lo=lo; p->out_hi=hi; p->iw_lo=-iw; p->iw_hi=+iw;
}
static inline float pid_step(pid_controller_t* p,float err,float dt){
    p->integ = clampf(p->integ + err*dt, p->iw_lo, p->iw_hi);
    float d = (err - p->prev)/dt; p->prev=err;
    return clampf(p->kp*err + p->ki*p->integ + p->kd*d, p->out_lo, p->out_hi);
}

// ============================ MQTT module (lite) =====================
static mqtt_client_t *g_mqtt=NULL;
static ip_addr_t      g_broker_ip;
static SemaphoreHandle_t g_mqtt_mtx;

static void mqtt_conn_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t s){
    (void)c;(void)arg; printf("[MQTT] %s (status=%d)\n", (s==MQTT_CONNECT_ACCEPTED)?"Connected":"Disconnected", s);
}
static bool mqtt_is_up(void){ return g_mqtt && mqtt_client_is_connected(g_mqtt); }
static err_t mqtt_pub_json(const char* topic,const char* json){
    if(!mqtt_is_up()) return ERR_CONN;
    if(g_mqtt_mtx) xSemaphoreTake(g_mqtt_mtx, portMAX_DELAY);
    cyw43_arch_lwip_begin();
    err_t r=mqtt_publish(g_mqtt, topic,(const u8_t*)json,(u16_t)strlen(json),0,0,NULL,NULL);
    cyw43_arch_lwip_end();
    if(g_mqtt_mtx) xSemaphoreGive(g_mqtt_mtx);
    return r;
}
static bool wifi_connect_blocking(uint32_t to_ms){
    printf("[NET] Wi-Fi %s\n", WIFI_SSID);
    int e=cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID,WIFI_PASS,CYW43_AUTH_WPA2_AES_PSK,to_ms);
    if(e){ printf("[NET] Wi-Fi err=%d\n",e); return false; }
    printf("[NET] Wi-Fi OK\n"); return true;
}

// Forward declarations with named params (match definitions)
static void vNetworkTask(void *pvParameters);
static void vDemo2Task(void *pvParameters);

static void vNetworkTask(void *pvParameters){
    (void)pvParameters;  // unused

    if(cyw43_arch_init()){ printf("[NET] cyw43 init fail\n"); vTaskDelete(NULL); }
    cyw43_arch_enable_sta_mode();

    while(!wifi_connect_blocking(WIFI_CONNECT_TIMEOUT_MS))
        vTaskDelay(pdMS_TO_TICKS(2000));

    ip4_addr_set_u32(ip_2_ip4(&g_broker_ip), ipaddr_addr(BROKER_IP_STR));
    g_mqtt=mqtt_client_new();
    struct mqtt_connect_client_info_t ci={0}; ci.client_id="pico-demo2"; ci.keep_alive=30;

    for(;;){
        if(!mqtt_is_up()){
            printf("[MQTT] Connecting %s:%d...\n", BROKER_IP_STR, BROKER_PORT);
            mqtt_client_connect(g_mqtt,&g_broker_ip,BROKER_PORT,mqtt_conn_cb,NULL,&ci);
        }
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,1); vTaskDelay(pdMS_TO_TICKS(200));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN,0); vTaskDelay(pdMS_TO_TICKS(800));
    }
}

// ============================ Barcode → turn mapping =================
static inline int barcode_is_right(char c){
    // Right if: A,C,E,G,I,K,M,O,Q,S,U,W,Y
    switch(c){ case 'A':case 'C':case 'E':case 'G':case 'I':case 'K':case 'M':
               case 'O':case 'Q':case 'S':case 'U':case 'W':case 'Y': return 1; }
    return 0;
}
static inline int barcode_is_left(char c){
    // Left if: B,D,F,H,J,L,N,P,R,T,V,X,Z
    switch(c){ case 'B':case 'D':case 'F':case 'H':case 'J':case 'L':case 'N':
               case 'P':case 'R':case 'T':case 'V':case 'X':case 'Z': return 1; }
    return 0;
}

// ============================ Line sensor init =======================
static void line_adc_init(void){
    adc_init();
    adc_gpio_init(LINE_ADC_GPIO);
    adc_select_input(2); // GP28 -> ADC2
}

// ============================ Control Task ==========================
// States
typedef enum { ST_FOLLOW=0, ST_TURNING } state_t;

static void vDemo2Task(void *pvParameters){
    (void)pvParameters;  // unused

    // --- IMU ---
    imu_t imu;
    imu.i2c=i2c1; imu.i2c_baud=IMU_I2C_BAUD; imu.pin_sda=IMU_SDA_PIN; imu.pin_scl=IMU_SCL_PIN;
    if(!imu_init(&imu)){ printf("[CTRL] IMU init fail\n"); vTaskDelete(NULL); }
    float filt_hdg=0.f; for(int i=0;i<20;i++){ float h=imu_update_and_get_heading(&imu); h+=HEADING_OFFSET_DEG; h=wrap_360(h); filt_hdg=ema(filt_hdg,h,0.2f); vTaskDelay(pdMS_TO_TICKS(10)); }
    float target_hdg=filt_hdg;

    // --- Drive stack ---
    motor_init();
    encoder_init();

    pid_controller_t pidL,pidR; pid_init(&pidL,SPID_KP,SPID_KI,SPID_KD,SPID_OUT_MIN,SPID_OUT_MAX,SPID_IWIND_CLAMP);
                      pid_init(&pidR,SPID_KP,SPID_KI,SPID_KD,SPID_OUT_MIN,SPID_OUT_MAX,SPID_IWIND_CLAMP);
    float s_int=0.f; int lastL=BASE_PWM_L, lastR=BASE_PWM_R;

    // --- Sensors ---
    line_adc_init();
    barcode_init();       // creates barcode task, sets up globals/flags
    init_barcode_irq();   // enables IRQ on GP7

    // --- Run-time vars ---
    state_t state = ST_FOLLOW;
    char last_barcode = '#';
    float dist_cm=0.f;
    float h_integ=0.f, h_prev=0.f;

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t tick=0;

    for(;;){
        // 1) Read IMU
        float raw_hdg = imu_update_and_get_heading(&imu);
        raw_hdg += HEADING_OFFSET_DEG; raw_hdg=wrap_360(raw_hdg);
        filt_hdg = ema(filt_hdg, raw_hdg, HDG_EMA_ALPHA);

        // 2) Line sensor
        uint16_t line_raw = adc_read();
        bool on_line = on_line_hys(line_raw);

        // 3) Barcode decision (decoded in IRQ/task)
        extern char decoded_barcode_char;   // from barcode.c
        char bc = decoded_barcode_char;     // snapshot
        if (bc != '#' && bc != '*'){        // valid letter scanned
            last_barcode = bc;
            // set a new turn goal ONLY when we are in follow mode
            if (state == ST_FOLLOW){
                float delta = 0.f;
                if (barcode_is_right(bc)) delta = +90.f;
                else if (barcode_is_left(bc)) delta = -90.f;
                if (delta != 0.f){
                    target_hdg = wrap_360(filt_hdg + delta);
                    state = ST_TURNING;
                }
            }
            // clear it so we don’t retrigger; barcode.c resets itself too
            decoded_barcode_char = '#';
        }

        // 4) Choose speed target
        float v_cmd = (state==ST_TURNING) ? V_TURN_CMPS : V_CRUISE_CMPS;

        // 5) Outer “heading” controller
        float h_err = 0.f;
        if (state == ST_FOLLOW){
            // while following, bias target toward the tape: simple proportional “nudge”
            // map ADC deviation around mid-band to a few degrees
            float mid = 0.5f*(TH_LO+TH_HI);
            float span = (float)(TH_HI-TH_LO);
            float dev = ((float)line_raw - mid) / (span*0.5f); // ~[-1..+1]
            dev = clampf(dev, -1.f, +1.f);
            float nudge_deg = 12.0f * (-dev);   // dev>0 (very black) => nudge left
            float follow_target = wrap_360(target_hdg + nudge_deg);
            h_err = wrap_pm180(follow_target - filt_hdg);
        } else {
            // turning to absolute heading
            h_err = wrap_pm180(target_hdg - filt_hdg);
            if (fabsf(h_err) < 5.0f && on_line){
                // close enough + back on the tape → resume follow
                state = ST_FOLLOW;
                // lock in new base heading
                target_hdg = filt_hdg;
            }
        }

        if (fabsf(h_err) < HEADING_DEADBAND_DEG) h_err = 0.f;
        float h_der = (h_err - h_prev)/DT_S; h_prev=h_err;
        if (KI_HEADING>0){ float iw=150.0f/fmaxf(KI_HEADING,1e-6f); h_integ=clampf(h_integ + h_err*DT_S, -iw, iw); }
        float delta_heading_rate_deg_s = KP_HEADING*h_err + KI_HEADING*h_integ + KD_HEADING*h_der;
        float delta_w = delta_heading_rate_deg_s * DEG2RAD * HEADING_RATE_SCALE;

        // 6) v,w -> left/right speed targets (cm/s)
        float diff_cmps = (0.5f * TRACK_WIDTH_M * delta_w) * 100.0f;
        float vL_t = v_cmd - diff_cmps;
        float vR_t = v_cmd + diff_cmps;

        // 7) Measure speeds, integrate distance
        float vL_m = get_left_speed();
        float vR_m = get_right_speed();
        static float vL_prev=0,vR_prev=0;
        if (vL_m < 0) vL_m=vL_prev; else vL_prev=vL_m;
        if (vR_m < 0) vR_m=vR_prev; else vR_prev=vR_m;
        float v_avg = 0.5f*(vL_m+vR_m); dist_cm += v_avg*DT_S;

        // 8) Inner speed PIDs
        float uL = pid_step(&pidL, vL_t - vL_m, DT_S);
        float uR = pid_step(&pidR, vR_t - vR_m, DT_S);

        // 9) Straightness PI
        float s_err = (vR_m - vL_m);
        s_int = clampf(s_int + s_err*DT_S, -STRAIGHT_I_CLAMP, STRAIGHT_I_CLAMP);
        float s_trim = STRAIGHT_KP*s_err + STRAIGHT_KI*s_int;

        // 10) Compose PWM and slew
        int pwmL=(int)lroundf(BASE_PWM_L + uL + s_trim);
        int pwmR=(int)lroundf(BASE_PWM_R + uR - s_trim);
        pwmL=clampi(pwmL,PWM_MIN_LEFT,PWM_MAX_LEFT);
        pwmR=clampi(pwmR,PWM_MIN_RIGHT,PWM_MAX_RIGHT);
        int dL=pwmL-lastL, dR=pwmR-lastR;
        if(dL>MAX_PWM_STEP)pwmL=lastL+MAX_PWM_STEP;
        if(dL<-MAX_PWM_STEP)pwmL=lastL-MAX_PWM_STEP;
        if(dR>MAX_PWM_STEP)pwmR=lastR+MAX_PWM_STEP;
        if(dR<-MAX_PWM_STEP)pwmR=lastR-MAX_PWM_STEP;
        forward_motor_manual(pwmL,pwmR); lastL=pwmL; lastR=pwmR;

        // 11) Telemetry @ ~5 Hz
        if ((tick++ % (1000/LOOP_DT_MS/5))==0){
            // console
            printf("[D2] st=%s bc=%c line=%u on=%d hdg=%.1f(raw=%.1f) herr=%.1f "
                   "vAvg=%.2f dist=%.1f PWM[%d,%d]\n",
                   (state==ST_FOLLOW)?"FOLLOW":"TURNING", last_barcode,
                   line_raw, on_line, (double)filt_hdg, (double)raw_hdg, (double)h_err,
                   (double)v_avg, (double)dist_cm, pwmL, pwmR);
            // mqtt
            if (g_mqtt && mqtt_client_is_connected(g_mqtt)) {
                char json[192];
                uint32_t ts = to_ms_since_boot(get_absolute_time());
                snprintf(json,sizeof(json),
                    "{\"ts\":%u,\"state\":\"%s\",\"barcode\":\"%c\","
                    "\"lineRaw\":%u,\"onLine\":%d,"
                    "\"vAvg\":%.2f,\"dist\":%.1f,"
                    "\"hdgRaw\":%.1f,\"hdg\":%.1f}",
                    (unsigned)ts, (state==ST_FOLLOW)?"FOLLOW":"TURNING", last_barcode,
                    line_raw, on_line, v_avg, dist_cm, raw_hdg, filt_hdg);
                mqtt_pub_json(MQTT_TOPIC_TELEM, json);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_DT_MS));
    }
}

// ============================ Main ==================================
int main(void){
    stdio_init_all();
    sleep_ms(600);

    // Start network (Wi-Fi + MQTT reconnect loop)
    g_mqtt_mtx = xSemaphoreCreateMutex();
    xTaskCreate(vNetworkTask,"net",4096,NULL,tskIDLE_PRIORITY+3,NULL);

    // Start Demo 2 control
    xTaskCreate(vDemo2Task,"demo2",4096,NULL,tskIDLE_PRIORITY+2,NULL);

    vTaskStartScheduler();
    while(true){ tight_loop_contents(); }
}
