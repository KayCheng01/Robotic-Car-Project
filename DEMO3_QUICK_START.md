# Demo 3: Quick Start Execution Steps

## Before You Start

### 1. Wire the Hardware

#### Ultrasonic Sensor (HC-SR04)
```
HC-SR04 Pin    →    Pico GPIO
VCC            →    3.3V
GND            →    GND
TRIG           →    GPIO 7
ECHO           →    GPIO 4 (with 3.3V level shifter if needed)
```

#### Servo Motor (SG90)
```
SG90 Pin       →    Pico GPIO
Red (VCC)      →    5V (or 3.3V with lower torque)
Brown (GND)    →    GND
Orange (PWM)   →    GPIO 5 (PWM-capable)
```

**Optional**: Verify which Pico GPIO pins support PWM:
- GPIO 0, 1 → Slice 0
- GPIO 2, 3 → Slice 1
- GPIO 4, 5 → Slice 2 ← Use GPIO 5
- GPIO 6, 7 → Slice 3
- etc.

### 2. Verify Pin Configuration

**Check in each header file**:
- `ultrasonic/ultrasonic.h`: `ULTRASONIC_TRIG_PIN` and `ULTRASONIC_ECHO_PIN`
- `servo/servo.h`: `SERVO_PIN`
- `ir/ir.h`: `IR_LEFT_PIN` and `IR_RIGHT_PIN` (should be from existing code)

Adjust if your wiring differs.

### 3. Build the Project

```bash
cd c:\Users\caiwe\Robotic-Car-Project
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

**Expected output**:
```
[100%] Built target main
Generated build/src/main.uf2
```

---

## Step 1: Test Ultrasonic Sensor

### Create a Quick Test File

**File**: `src/test_ultrasonic_only.c`

```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"

int main(void) {
    stdio_init_all();
    sleep_ms(800);
    
    printf("Ultrasonic Sensor Test\n");
    ultrasonic_init();
    
    while (true) {
        float dist = ultrasonic_get_distance_cm();
        printf("Distance: %.1f cm\n", (double)dist);
        sleep_ms(500);
    }
    return 0;
}
```

### Flash & Test
```bash
# Update CMakeLists.txt to use test_ultrasonic_only.c temporarily
# Then rebuild:
cd build
make
# Flash: hold BOOTSEL, drag build/src/main.uf2 to Pico
```

**Expected output** (open serial monitor at 115200 baud):
```
Ultrasonic Sensor Test
Distance: 150.5 cm
Distance: 125.3 cm
Distance: 25.2 cm  (when you move hand close)
Distance: 18.5 cm
Distance: -1.0 cm  (if timeout or out of range)
```

**Validation**:
- [ ] Readings near 0 when object very close (< 5cm)
- [ ] Readings near 400 when no obstacle
- [ ] Readings update every ~500ms
- [ ] No -1 errors during normal operation

---

## Step 2: Test Servo Motor

### Create a Quick Test File

**File**: `src/test_servo_only.c`

```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "servo.h"

int main(void) {
    stdio_init_all();
    sleep_ms(800);
    
    printf("Servo Motor Test\n");
    servo_init();
    
    while (true) {
        printf("Moving to LEFT (0°)\n");
        servo_move_left(500);
        
        sleep_ms(1000);
        printf("Moving to CENTER (90°)\n");
        servo_move_center(500);
        
        sleep_ms(1000);
        printf("Moving to RIGHT (180°)\n");
        servo_move_right(500);
        
        sleep_ms(1000);
    }
    return 0;
}
```

### Flash & Test
```bash
# Update CMakeLists.txt to use test_servo_only.c
cd build
make
# Flash Pico
```

**Expected output** (open serial monitor):
```
Servo Motor Test
Moving to LEFT (0°)
Moving to CENTER (90°)
Moving to RIGHT (180°)
[repeats]
```

**Validation**:
- [ ] Servo arm rotates smoothly from left (0°) to right (180°)
- [ ] No grinding or buzzing sounds
- [ ] Settles within ~300ms after movement command

---

## Step 3: Test Obstacle Scanning

### Create Combined Test File

**File**: `src/test_obstacle_scan_only.c`

```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "servo.h"
#include "demo3_obstacle.h"

int main(void) {
    stdio_init_all();
    sleep_ms(800);
    
    printf("Obstacle Scanning Test\n");
    printf("Place obstacle ~20cm ahead of the robot\n");
    printf("Press enter to start scan...\n");
    
    // Wait for serial input (or just delay)
    sleep_ms(3000);
    
    demo3_obstacle_init();
    
    obstacle_data_t obs = {0};
    while (true) {
        printf("\n=== Starting Scan ===\n");
        demo3_scan_obstacle(&obs);
        demo3_log_obstacle(&obs);
        
        printf("\nAnalyzing...\n");
        int side = demo3_choose_avoidance_side(&obs);
        printf("Clear side: %d (-1=LEFT, +1=RIGHT, 0=BLOCKED)\n", side);
        
        printf("\nWait 5 seconds before next scan...\n");
        sleep_ms(5000);
    }
    return 0;
}
```

### Flash & Test

**Expected output**:
```
Obstacle Scanning Test
Place obstacle ~20cm ahead of the robot
Press enter to start scan...

=== Starting Scan ===
[DEMO3] ========== OBSTACLE SCAN START ==========
[DEMO3] Scanning: CENTER (90°)...
[DEMO3]   Center distance: 19.5 cm
[DEMO3] Scanning: LEFT (0°)...
[DEMO3]   Left distance: 38.2 cm
[DEMO3] Scanning: RIGHT (180°)...
[DEMO3]   Right distance: 25.1 cm
[DEMO3] ========== OBSTACLE SCAN COMPLETE ==========
[DEMO3] Estimated width: 18.7 cm

[DEMO3:LOG] Obstacle Data:
[DEMO3:LOG]   Center: 19.5 cm
[DEMO3:LOG]   Left:   38.2 cm
[DEMO3:LOG]   Right:  25.1 cm
[DEMO3:LOG]   Width:  18.7 cm
[DEMO3:LOG]   Side:   LEFT

Analyzing...
[DEMO3] Both sides clear, choosing LEFT (38.2 > 25.1)
Clear side: -1 (LEFT)

Wait 5 seconds before next scan...
```

**Validation**:
- [ ] Center distance matches actual obstacle distance
- [ ] Left/right distances larger than center
- [ ] Width estimation reasonable
- [ ] Correct side chosen (left or right)

---

## Step 4: Test Line Recovery Search

### Verify IR Sensors First

```c
#include "ir.h"

int main(void) {
    ir_init();
    while (true) {
        printf("L=%d R=%d\n", ir_left_is_black(), ir_right_is_black());
        sleep_ms(100);
    }
}
```

Should show `1 1` when both sensors on black line, `0 0` when off line.

### Create Recovery Test

**File**: `src/test_line_recovery_only.c`

```c
#include <stdio.h>
#include "pico/stdlib.h"
#include "motor.h"
#include "ir.h"
#include "demo3_line_recovery.h"

int main(void) {
    stdio_init_all();
    sleep_ms(800);
    
    printf("Line Recovery Test\n");
    motor_init();
    ir_init();
    
    printf("Starting search (robot will move). Remove from line first.\n");
    sleep_ms(2000);
    
    bool found = demo3_search_for_line();
    if (found) {
        printf("SUCCESS: Line found!\n");
    } else {
        printf("FAILED: Line not found (timeout)\n");
    }
    
    stop_motor_manual();
    
    while (true) sleep_ms(1000);
    return 0;
}
```

**Validation**:
- [ ] Robot starts searching (moves in spiral pattern)
- [ ] Returns `true` when line is re-detected
- [ ] Stops after 5 seconds if no line found
- [ ] Correctly stops after finding line

---

## Step 5: Full Demo 3 Integration Test

### Setup

1. **Assemble track** with one obstacle blocking the path
   - Obstacle width: 10-20cm
   - Clearance on at least one side: > 25cm

2. **Position robot** on line upstream of obstacle

3. **Flash full testdemo3.c** (already compiled)

### Run Test

```bash
cd build
make
# Flash to Pico
# Open serial monitor (115200 baud)
```

### Watch for State Transitions

```
[DEMO3] Ready! Starting in 3 seconds...
[DEMO3:TELEM] State=LINE_FOLLOW Hdg=45.1° Speed=18.2 cm/s Obstacles=0
[DEMO3:TELEM] State=LINE_FOLLOW Hdg=45.3° Speed=19.1 cm/s Obstacles=0
...
[DEMO3] Obstacle detected at 22.5 cm!
[DEMO3:STATE] LINE_FOLLOW → OBSTACLE_DETECT
[DEMO3:STATE] OBSTACLE_DETECT → SCANNING
[DEMO3] ========== OBSTACLE SCAN START ==========
...
[DEMO3:STATE] AVOIDING → SEARCHING
[DEMO3:RECOVERY] Starting line search...
[DEMO3:RECOVERY] Line FOUND! Detected after 1200 ms
[DEMO3:STATE] SEARCHING → RESUME
[DEMO3:STATE] RESUME → LINE_FOLLOW
[DEMO3:TELEM] State=LINE_FOLLOW Hdg=46.5° Speed=20.1 cm/s Obstacles=1
```

### Success Criteria

- [ ] Robot follows line normally
- [ ] Stops at obstacle
- [ ] Scans all 3 positions (left/center/right)
- [ ] Chooses correct clear side
- [ ] Turns and moves around obstacle
- [ ] Searches for line
- [ ] Resumes line following after obstacle
- [ ] Telemetry shows correct state transitions
- [ ] Obstacle count incremented (shown as `Obstacles=1`)

---

## Debug Tips

### Ultrasonic Not Working
- Check GPIO for noise (try adding 100nF capacitor on ECHO pin)
- Verify TRIG pulse timing (use oscilloscope if available)
- Test with obstacle at different distances (5, 10, 20, 50cm)

### Servo Jittering
- Confirm PWM frequency is exactly 50 Hz
- Check power supply (servo draws ~1A peak, may need external supply)
- Try reducing PWM to GPIO 2 or 3 if GPIO 5 has noise

### Line Recovery Failing
- Verify IR sensors work in isolation first
- Increase search timeout in `demo3_line_recovery.c`
- Verify track is readable by IR (good contrast)
- Check servo is returning to center before line search

### Robot Collision During Avoidance
- Reduce turn angle (currently 45°, try 30°)
- Increase forward distance estimate
- Use encoder-based distance instead of timing (future improvement)

---

## Next: Collect Telemetry Data

Once full demo works, capture a run for analysis:

```bash
# Redirect serial output to file:
timeout 60 picocom /dev/ttyUSB0 -b 115200 > demo3_run.log
# Windows: use Tera Term or similar

# Post-run analysis:
cat demo3_run.log | grep -E "STATE|TELEM|SCAN|AVOID|RECOVERY"
```

---

## When Ready: MQTT Integration

After confirming all features work, add MQTT logging:

1. Modify `testdemo3.c` to call `mqtt_publish()` for key events
2. Use structure:
   ```c
   mqtt_publish("demo3/state", "AVOIDING");
   mqtt_publish("demo3/obstacle_width", "16.7");
   mqtt_publish("demo3/clear_side", "LEFT");
   ```

3. View on MQTT dashboard in real-time

---

**Estimated Time to Full Demo 3**: 2-3 hours
- Phase 1 (component testing): 30 min
- Phase 2 (subsystem testing): 45 min
- Phase 3 (full integration): 60 min
- Phase 4 (edge cases & tuning): 30 min

