#ifndef DEMO3_LINE_RECOVERY_H
#define DEMO3_LINE_RECOVERY_H

#include <stdint.h>
#include <stdbool.h>

// ============== Line Recovery Configuration ==============
#define LINE_RECOVERY_TIMEOUT_MS    5000  // Give up searching after 5 seconds
#define LINE_SEARCH_TURN_ANGLE_DEG  45.0f // Turn angle when searching
#define LINE_SEARCH_SPEED_PWM       100   // Speed during search

// ============== Function Prototypes ==============

/**
 * Search for line using spiral/sweep pattern with IR sensors.
 * Returns true if line found, false if timeout.
 * Blocking call.
 */
bool demo3_search_for_line(void);

/**
 * Resume line following after recovery.
 * Re-engage the heading PID and IR-based line tracking.
 */
void demo3_resume_line_following(void);

/**
 * Quick check: is the line currently detected by IR sensors?
 */
bool demo3_line_detected(void);

#endif // DEMO3_LINE_RECOVERY_H
