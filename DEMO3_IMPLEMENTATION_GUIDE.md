# Demo 3: Obstacle Detection & Avoidance - Implementation Guide

## Overview

Demo 3 builds on the existing line-following system to add obstacle detection, intelligent avoidance, and recovery. The system uses:

- **Ultrasonic sensor** (HC-SR04): Distance measurement
- **Servo motor**: Rotates ultrasonic to scan left/center/right
- **IR sensors**: Line detection (existing)
- **IMU**: Heading stabilization (existing)
- **Encoders**: Speed/distance tracking (existing)

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ Main State Machine (testdemo3.c)                            │
├─────────────────────────────────────────────────────────────┤
│ 1. LINE_FOLLOW        → IR sensors guide the robot         │
│ 2. OBSTACLE_DETECTED  → Ultrasonic reading < threshold     │
│ 3. SCANNING           → Servo sweeps, measure 3 distances  │
│ 4. PLANNING           → Choose left or right avoidance    │
│ 5. AVOIDING           → Execute turn + forward movement    │
│ 6. SEARCHING          → Look for line using IR sensors    │
│ 7. RESUME             → Resume line following              │
└─────────────────────────────────────────────────────────────┘
```

## Modules Created

### 1. **Ultrasonic Sensor Driver** (`ultrasonic.h/c`)

**Purpose**: Measure distance to obstacles using GPIO-based echo pulse timing.

**Key Functions**:
- `ultrasonic_init()` - Initialize GPIO and interrupts
- `ultrasonic_get_distance_cm()` - Blocking distance measurement (returns cm or -1 on error)
- `ultrasonic_get_last_distance_cm()` - Get last valid reading
- `ultrasonic_obstacle_detected(threshold_cm)` - Quick check

**How it works**:
1. Send 10µs trigger pulse on GPIO (ULTRASONIC_TRIG_PIN)
2. Measure echo pulse width on GPIO (ULTRASONIC_ECHO_PIN)
3. Calculate: `distance = (pulse_width_us / 2) × 0.0343 cm/µs`
   - Speed of sound ≈ 343 m/s = 0.0343 cm/µs
   - Divide by 2 because sound travels there and back

**Pin Configuration** (edit as needed):
```c
#define ULTRASONIC_TRIG_PIN  7    // Output: trigger pulse
#define ULTRASONIC_ECHO_PIN  4    // Input: echo pulse
```

**Usage**:
```c
ultrasonic_init();
float dist = ultrasonic_get_distance_cm();
if (dist > 0 && dist < 25.0f) {
    // Obstacle detected!
}
```

---

### 2. **Servo Control Driver** (`servo.h/c`)

**Purpose**: Rotate servo 0°-180° to point ultrasonic in different directions.

**Key Functions**:
- `servo_init()` - Initialize PWM on SERVO_PIN
- `servo_set_angle(float angle_deg)` - Set to 0-180°
- `servo_get_angle()` - Get current angle
- `servo_move_left/center/right(delay_ms)` - Convenience functions

**How it works**:
- Standard RC servo on 50 Hz PWM (20ms period)
- Pulse width maps to angle:
  - 1.0ms → 0° (left)
  - 1.5ms → 90° (center)
  - 2.0ms → 180° (right)

**Pin Configuration**:
```c
#define SERVO_PIN  5  // PWM-capable GPIO
```

**Usage**:
```c
servo_init();
servo_move_left(300);      // Move left, wait 300ms
float dist_left = ultrasonic_get_distance_cm();

servo_move_center(300);    // Return to center
```

---

### 3. **Obstacle Scanning & Planning** (`demo3_obstacle.h/c`)

**Purpose**: Coordinate ultrasonic + servo scanning, analyze results, and plan avoidance.

**Data Structure**:
```c
typedef struct {
    float distance_cm;           // Center (straight ahead)
    float distance_left_cm;      // Left position (0°)
    float distance_right_cm;     // Right position (180°)
    float width_cm;              // Estimated obstacle width
    int clear_side;              // -1=left, +1=right, 0=blocked
} obstacle_data_t;
```

**Key Functions**:

#### `demo3_scan_obstacle(obstacle_data_t *obs)`
- Moves servo to left/center/right (300ms each for stabilization)
- Measures distance at each position
- Estimates obstacle width
- **Total time**: ~2 seconds

**Output Example**:
```
[DEMO3] Scanning: CENTER (90°)...
[DEMO3]   Center distance: 18.5 cm
[DEMO3] Scanning: LEFT (0°)...
[DEMO3]   Left distance: 35.2 cm
[DEMO3] Scanning: RIGHT (180°)...
[DEMO3]   Right distance: 22.3 cm
[DEMO3] Estimated width: 16.7 cm
```

#### `int demo3_choose_avoidance_side(obstacle_data_t *obs)`
- Analyzes left/right distances
- Returns: `-1` (left clear), `+1` (right clear), `0` (blocked)
- **Logic**: Side with > (THRESHOLD + CLEARANCE_MARGIN) is "clear"
  - Default threshold: 20cm + 5cm margin = 25cm

#### `demo3_plan_avoidance(...)`
- Calculates turn angle (~45° toward clear side)
- Calculates forward distance (obstacle width + margin)
- Sets parallel offset for line recovery

---

### 4. **Line Recovery Logic** (`demo3_line_recovery.h/c`)

**Purpose**: After obstacle avoidance, search for and re-engage the line.

**Key Functions**:

#### `bool demo3_search_for_line()`
- Executes spiral search pattern: right sweep → left sweep → escalate
- Checks IR sensors during search
- Timeout: 5 seconds (configurable)
- **Returns**: `true` if line found, `false` if timeout

**Search Pattern**:
```
Phase 1: Turn right, move forward, check IR
         │
Phase 2: Turn left (wider), move forward, check IR
         │
Phase 3: Back to center, reverse, escalate
         │
[repeat with larger radius]
```

#### `bool demo3_line_detected()`
- Returns `true` if either IR sensor detects black line
- Simple check: `ir_left_is_black() || ir_right_is_black()`

#### `demo3_resume_line_following()`
- Stabilization delay before re-engaging line tracking
- Placeholder for special recovery logic

---

### 5. **Main Demo 3 Loop** (`testdemo3.c`)

**State Machine**:

```
LINE_FOLLOW
    ↓ [obstacle < 25cm]
OBSTACLE_DETECT → SCANNING → PLANNING → AVOIDING → SEARCHING → RESUME → LINE_FOLLOW
                                          ↓
                                    [execute turn]
                                    [execute forward]
                                    [search for line]
```

**State Handlers**:

#### `demo3_line_follow_step()`
- Monitors ultrasonic distance ahead
- Uses IR sensors for lateral steering
- On obstacle detected: stop, transition to OBSTACLE_DETECT

#### `demo3_obstacle_detect_step()`
- Stop motors
- Brief pause for stability
- Transition to SCANNING

#### `demo3_scanning_step()`
- Call `demo3_scan_obstacle()` (2 seconds)
- Analyze results
- Transition to PLANNING

#### `demo3_planning_step()`
- Call `demo3_plan_avoidance()`
- Calculate maneuver parameters
- Transition to AVOIDING

#### `demo3_avoiding_step()`
- Execute turn (timing-based)
- Execute forward movement (timing-based)
- Transition to SEARCHING

#### `demo3_searching_step()`
- Call `demo3_search_for_line()` (up to 5 seconds)
- Transition to RESUME

#### `demo3_resume_step()`
- Call `demo3_resume_line_following()`
- Transition back to LINE_FOLLOW

---

## Configuration & Tuning

### Pin Assignments

**File**: Each `ultrasonic.h`, `servo.h`, etc.

```c
// Ultrasonic
#define ULTRASONIC_TRIG_PIN  7
#define ULTRASONIC_ECHO_PIN  4

// Servo
#define SERVO_PIN  5

// IR (existing)
#define IR_LEFT_PIN   1
#define IR_RIGHT_PIN  28
```

### Distance Thresholds

**File**: `testdemo3.c`

```c
#define OBSTACLE_THRESHOLD_CM     25.0f   // Trigger avoidance if < 25cm
```

**File**: `demo3_obstacle.c`

```c
#define OBSTACLE_DETECTION_THRESHOLD_CM   20.0f
#define OBSTACLE_CLEARANCE_MARGIN_CM      5.0f   // Need 5cm+ clearance
```

### Speed & Timing

```c
#define V_TARGET                    0.20f   // 0.2 m/s base speed
#define LOOP_DT_MS                  10      // 100 Hz control loop
```

### Avoidance Tuning

In `demo3_plan_avoidance()`:
- **Turn angle**: Currently 45° (tune if robot overshoots/undershoots)
- **Forward distance**: `width + 15cm` (tune based on your obstacle size)
- **Parallel offset**: 15cm (tune based on track width)

---

## Testing Checklist

### Phase 1: Individual Component Testing

- [ ] **Ultrasonic**
  - [ ] Compile and flash code
  - [ ] Verify distance readings at known distances (10cm, 20cm, 30cm)
  - [ ] Check range: 2-400cm

- [ ] **Servo**
  - [ ] Verify servo rotates left (0°) → center (90°) → right (180°)
  - [ ] Smooth movement, no grinding
  - [ ] Stabilization time: ~300ms

- [ ] **IR Sensors**
  - [ ] Verify line detection works (from existing demos)

### Phase 2: Subsystem Testing

- [ ] **Obstacle Scanning**
  - [ ] Place obstacle at 20cm straight ahead
  - [ ] Verify scan outputs 3 measurements (left/center/right)
  - [ ] Check width estimation

- [ ] **Avoidance Planning**
  - [ ] Verify correct side chosen (left or right with more clearance)
  - [ ] Verify turn angle and distance calculations

### Phase 3: Full Integration

- [ ] **Line Following + Obstacle Detection**
  - [ ] Robot follows line normally
  - [ ] Stops when obstacle detected

- [ ] **Full Avoidance Flow**
  - [ ] Robot scans obstacle
  - [ ] Turns correctly (left or right)
  - [ ] Moves forward around obstacle
  - [ ] Searches for line
  - [ ] Resumes line following

### Phase 4: Edge Cases

- [ ] Obstacle very close (<10cm)
- [ ] Both sides blocked (narrow corridor)
- [ ] Line lost after avoidance (search timeout)
- [ ] Multiple obstacles in sequence

---

## Telemetry Output

### Expected Console Output

```
[DEMO3] ========== DEMO 3: OBSTACLE AVOIDANCE ==========
[DEMO3] Version: 1.0 (IMU + IR + Ultrasonic + Servo)
[DEMO3] Compiled: Oct 30 2025 14:30:00
[DEMO3] IMU OK
[DEMO3] Motors OK
[DEMO3] Encoders OK
[DEMO3] IR sensors OK
[DEMO3] Obstacle detection OK
[DEMO3] Target heading = 45.2°
[DEMO3] Ready! Starting in 3 seconds...

[DEMO3:TELEM] State=LINE_FOLLOW Hdg=45.1° Speed=18.2 cm/s Obstacles=0
...
[DEMO3] Obstacle detected at 22.5 cm!
[DEMO3:STATE] LINE_FOLLOW → OBSTACLE_DETECT
[DEMO3:STATE] OBSTACLE_DETECT → SCANNING
[DEMO3] ========== OBSTACLE SCAN START ==========
[DEMO3] Scanning: CENTER (90°)...
[DEMO3]   Center distance: 18.5 cm
[DEMO3] Scanning: LEFT (0°)...
[DEMO3]   Left distance: 35.2 cm
[DEMO3] Scanning: RIGHT (180°)...
[DEMO3]   Right distance: 22.3 cm
[DEMO3] Estimated width: 16.7 cm
[DEMO3:STATE] SCANNING → PLANNING
[DEMO3] Choosing LEFT side (left=35.2, right=22.3)
[DEMO3] Avoidance plan: turn=45.0°, forward=31.7 cm, parallel=15.0 cm
[DEMO3:STATE] PLANNING → AVOIDING
[DEMO3:AVOID] Executing avoidance: turn=45.0°, forward=31.7 cm
[DEMO3:AVOID] Avoidance complete, searching for line...
[DEMO3:STATE] AVOIDING → SEARCHING
[DEMO3:RECOVERY] Starting line search...
[DEMO3:RECOVERY] Phase: RIGHT SWEEP
[DEMO3:RECOVERY] Line FOUND! Detected after 1250 ms
[DEMO3:STATE] SEARCHING → RESUME
[DEMO3:RECOVERY] Resuming line following...
[DEMO3:STATE] RESUME → LINE_FOLLOW
[DEMO3:TELEM] State=LINE_FOLLOW Hdg=46.3° Speed=19.1 cm/s Obstacles=1
```

---

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| Ultrasonic always reads -1 | GPIO interrupt not firing | Check ECHO_PIN, verify IRQ enabled |
| Servo doesn't move | PWM not initialized | Verify SERVO_PIN has PWM capability |
| Wrong avoidance side chosen | Clearance margin too high | Reduce OBSTACLE_CLEARANCE_MARGIN_CM |
| Robot overshoots obstacle | Turn angle too small | Increase turn angle in plan_avoidance |
| Line not found after avoidance | Search pattern too narrow | Increase spiral radius or timeout |
| Collision during avoidance | Timing estimates inaccurate | Tune forward_dist_cm and turn_angle_deg |

---

## Next Steps (Post-Demo 3)

1. **MQTT Telemetry**: Log all events to MQTT dashboard
2. **Encoder-Based Distance**: Replace timing-based movement with encoder odometry
3. **IMU-Based Turns**: Use IMU heading feedback for precise turns
4. **Adaptive Search**: Learn track geometry to improve line recovery
5. **Multiple Obstacles**: Handle sequences of obstacles
6. **Performance Logging**: CSV export of complete run data

---

## File Structure

```
src/
├── testdemo3.c           # Main state machine
├── demo3_obstacle.h/c    # Obstacle scanning & planning
├── demo3_line_recovery.h/c # Line search & recovery

ultrasonic/
├── ultrasonic.h/c        # Distance sensor driver

servo/
├── servo.h/c             # Servo control

existing/
├── motor.h/c             # Motor control (unchanged)
├── encoder.h/c           # Speed/distance (unchanged)
├── ir.h/c                # Line sensors (unchanged)
├── imu.h/c               # Heading (unchanged)
```

