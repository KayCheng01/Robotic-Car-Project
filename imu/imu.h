#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ===== Board wiring defaults (edit if needed) =====
#define IMU_I2C_BAUD   115200    // 100 kHz or 400 kHz
#define IMU_SDA_PIN    2          // GP2
#define IMU_SCL_PIN    3          // GP3

// ===== LSM303DLHC I2C addresses =====
#define IMU_ACC_ADDR   0x19       // Accelerometer
#define IMU_MAG_ADDR   0x1E       // Magnetometer

typedef struct {
    // configuration
    i2c_inst_t *i2c;              // i2c0 or i2c1
    uint32_t    i2c_baud;         // typically 400k
    uint8_t     pin_sda;          // GPIO number
    uint8_t     pin_scl;          // GPIO number

    // state
    bool        inited_acc;
    bool        inited_mag;

    // simple hard-iron offsets (counts) for magnetometer
    float       mx_off;
    float       my_off;
    float       mz_off;

    // last computed heading (deg 0..360)
    float       heading_deg;
} imu_t;

// Init I2C pins and configure both ACC (0x19) and MAG (0x1E)
bool imu_init(imu_t *imu);

// Read accelerometer in g units (±2g, HR mode -> ~1 mg/LSB)
bool imu_read_accel_g(imu_t *imu, float *ax, float *ay, float *az);

// Read magnetometer raw counts (X,Y,Z). Note: raw, not scaled to uT.
bool imu_read_mag_raw(imu_t *imu, float *mx, float *my, float *mz);

// Compute tilt-compensated heading from given accel & mag samples
float imu_compute_heading_deg(imu_t *imu, float ax, float ay, float az,
                              float mx, float my, float mz);

// Convenience: read sensors, update internal heading, and return it
float imu_update_and_get_heading(imu_t *imu);

// Optional: set hard-iron offsets after you measure min/max
static inline void imu_set_mag_offsets(imu_t *imu, float mx_off, float my_off, float mz_off) {
    if (!imu) return;
    imu->mx_off = mx_off;
    imu->my_off = my_off;
    imu->mz_off = mz_off;
}

// Get last heading without new reads
static inline float imu_get_heading(imu_t *imu) {
    return imu ? imu->heading_deg : 0.f;
}

#endif // IMU_H
