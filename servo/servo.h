#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include "pico/stdlib.h"

// ===== User Configuration =====
// Default values (can override in main.c before include)
#ifndef SERVO_CENTER_US
#define SERVO_CENTER_US 1500  // Center pulse width in µs
#endif

#ifndef SERVO_SPAN_US
#define SERVO_SPAN_US   500   // ±span around center (=> 1000–2000 µs)
#endif

#ifndef SERVO_MIN_DEG
#define SERVO_MIN_DEG   -90.0f
#endif

#ifndef SERVO_MAX_DEG
#define SERVO_MAX_DEG   +90.0f
#endif

#ifndef SERVO_BIAS_DEG
#define SERVO_BIAS_DEG   0.0f
#endif

#ifndef SERVO_DIR_SIGN
#define SERVO_DIR_SIGN  +1     // set to -1 to flip direction
#endif

#ifndef SERVO_SETTLE_MS
#define SERVO_SETTLE_MS 70     // wait time after each move
#endif
// ===============================

// Initialize servo
void servo_init(uint servo_gpio);

// Write angle (logical degrees)
void servo_write_deg(float logical_deg);

#endif // SERVO_H
