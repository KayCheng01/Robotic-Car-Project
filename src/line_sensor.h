#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include "pico/stdlib.h"

// Line sensor digital output pin
#define LINE_SENSOR_DO_PIN  28

// Initialize line sensor GPIO and IRQ
void line_sensor_init(void);

// Read current line state
// Returns true if on track (black), false if off track (white)
bool line_sensor_is_on_track(void);

#endif // LINE_SENSOR_H
