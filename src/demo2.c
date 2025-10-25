#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "imu.h"
#include "motor.h"

#define TURNING_SPEED 80
#define TURN_TOLERANCE 2.0f
#define TARGET_TURN_ANGLE 90.0f

static inline float wrap180(float x){
    while (x > 180.f) x -= 360.f;
    while (x < -180.f) x += 360.f;
    return x;
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    imu_t imu;
    imu.i2c = i2c1;
    imu.i2c_baud = IMU_I2C_BAUD;
    imu.pin_sda = IMU_SDA_PIN;
    imu.pin_scl = IMU_SCL_PIN;
    imu.mx_off = imu.my_off = imu.mz_off = 0.f;
    if (!imu_init(&imu)) { 
        printf("[IMU] init failed\n"); 
        while (1); 
    }

    motor_init();
    disable_pid_control();

    float start_heading = imu_update_and_get_heading(&imu);
    float target_heading = start_heading + TARGET_TURN_ANGLE;
    if (target_heading >= 360.0f) target_heading -= 360.0f;

    printf("[IMU] Start heading: %.2f°\n", start_heading);
    printf("[IMU] Target heading: %.2f°\n", target_heading);
    printf("[IMU] Turning 90 deg right...\n");

    while (true) {
        float hdg = imu_update_and_get_heading(&imu);
        float err = wrap180(target_heading - hdg);

        // 🖨️ Print live data
        printf("hdg=%.2f  err=%.2f  tgt=%.2f\n", hdg, err, target_heading);

        if (fabsf(err) < TURN_TOLERANCE) break;

        float turn_KP = 2.0f;
        float corr = turn_KP * err;
        if (corr > 60) corr = 60;
        if (corr < -60) corr = -60;

        forward_motor_manual(TURNING_SPEED - corr, TURNING_SPEED + corr);
        sleep_ms(10);
    }

    stop_motor_manual();
    printf("[IMU] Turn complete.\n");
    while (true) sleep_ms(1000);
}
