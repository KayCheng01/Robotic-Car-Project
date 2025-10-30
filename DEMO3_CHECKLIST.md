# Demo 3 - Implementation Checklist & Reference Card

## 📋 Pre-Implementation Checklist

### Hardware Setup
- [ ] Ultrasonic sensor (HC-SR04) wired to GPIO 7 (TRIG) and GPIO 4 (ECHO)
- [ ] Servo motor (SG90) wired to GPIO 5 (PWM), 5V power, GND
- [ ] IR sensors already configured (GPIO 1, GPIO 28)
- [ ] Motor driver and encoders already working (from previous demos)
- [ ] IMU (LSM303) already configured on I2C
- [ ] Serial monitor available (115200 baud)

### Software Setup
- [ ] Pico SDK v1.3.0+ installed
- [ ] CMake v3.13+ installed
- [ ] ARM GCC toolchain configured
- [ ] Git repository cloned and accessible

---

## 🔨 Build Checklist

### Step 1: Create New Drivers
- [x] `ultrasonic/ultrasonic.h` - Header with API
- [x] `ultrasonic/ultrasonic.c` - Implementation with GPIO interrupt
- [x] `servo/servo.h` - Header with API
- [x] `servo/servo.c` - Implementation with PWM

### Step 2: Create Logic Modules
- [x] `src/demo3_obstacle.h` - Header with scanning/planning API
- [x] `src/demo3_obstacle.c` - Obstacle detection implementation
- [x] `src/demo3_line_recovery.h` - Header with search API
- [x] `src/demo3_line_recovery.c` - Line search implementation

### Step 3: Create Main Application
- [x] `src/testdemo3.c` - State machine and main loop

### Step 4: Update Build Configuration
- [x] `CMakeLists.txt` - Add `add_subdirectory(ultrasonic)` and `add_subdirectory(servo)`
- [x] `src/CMakeLists.txt` - Add demo3 source files and link servo_lib
- [x] `ultrasonic/CMakeLists.txt` - Create library (already exists)
- [x] `servo/CMakeLists.txt` - Create library (already exists)

### Step 5: Compile
```bash
cd c:\Users\caiwe\Robotic-Car-Project\build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
# Expected: [100%] Built target main
# Output: build/src/main.uf2
```
- [ ] Compilation successful (no errors)
- [ ] Warnings acceptable (no -Werror)
- [ ] main.uf2 file generated

---

## 🧪 Component Testing Checklist

### Test 1: Ultrasonic Sensor
**Goal**: Verify distance measurements

**Test Code**: `src/test_ultrasonic_only.c` (provided in DEMO3_QUICK_START.md)

**Hardware**: Ultrasonic sensor wired to GPIO 7 (TRIG) and GPIO 4 (ECHO)

**Procedure**:
1. Create test file with ultrasonic initialization loop
2. Compile and flash
3. Open serial monitor (115200 baud)
4. Place object at various distances (5cm, 10cm, 20cm, 50cm, 100cm)

**Expected Results**:
- [ ] Readings change with distance
- [ ] 5cm object reads ~5cm ±2cm
- [ ] 100cm reads ~100cm ±10cm
- [ ] No timeout errors (-1.0) during normal operation
- [ ] Update frequency ~2-5 Hz (every 200-500ms)

**Troubleshooting**:
- [ ] Always reads -1? Check GPIO 4 ECHO pin wiring
- [ ] Always reads >300cm? Check GPIO 7 TRIG pin wiring
- [ ] Jittery readings? Add 100nF capacitor on ECHO line

---

### Test 2: Servo Motor
**Goal**: Verify 3-position sweep

**Test Code**: `src/test_servo_only.c` (provided in DEMO3_QUICK_START.md)

**Hardware**: Servo wired to GPIO 5 (PWM), 5V power, GND

**Procedure**:
1. Create test file with servo sweep loop (LEFT → CENTER → RIGHT)
2. Compile and flash
3. Observe servo movement
4. Check console for position updates

**Expected Results**:
- [ ] Servo rotates smoothly LEFT → CENTER → RIGHT
- [ ] No grinding/buzzing sounds
- [ ] Settles position within ~300ms
- [ ] Each position clearly distinct
- [ ] Repeats smoothly

**Troubleshooting**:
- [ ] Servo doesn't move? Check GPIO 5 is PWM-capable, verify power
- [ ] Servo jitters? Reduce PWM noise or add external power supply
- [ ] Weak movement? Increase servo power to 5V or external battery

---

### Test 3: IR Sensors
**Goal**: Verify line detection (should already work from previous demos)

**Procedure**:
1. Use existing IR test or inline code:
   ```c
   ir_init();
   while(true) {
       printf("L=%d R=%d\n", ir_left_is_black(), ir_right_is_black());
       sleep_ms(100);
   }
   ```
2. Place robot on black line
3. Place robot on white surface

**Expected Results**:
- [ ] On black line: `L=1 R=1` or similar (depending on IR_BLACK_IS_LOW config)
- [ ] On white: `L=0 R=0`
- [ ] Responds within ~100ms

---

## 🔗 Subsystem Testing Checklist

### Test 4: Obstacle Scanning
**Goal**: Verify automatic width measurement

**Test Code**: `src/test_obstacle_scan_only.c` (provided in DEMO3_QUICK_START.md)

**Setup**:
1. Place obstacle ~15-25cm ahead of robot
2. Clear space around obstacle (especially left and right)

**Procedure**:
1. Compile and flash test code
2. Let robot run 5+ scans
3. Verify measurements

**Expected Results** (example):
```
[DEMO3] ========== OBSTACLE SCAN START ==========
[DEMO3]   Center: 19.5 cm ← Close to actual
[DEMO3]   Left:   35.2 cm ← Open space
[DEMO3]   Right:  22.1 cm ← Slightly open
[DEMO3] Estimated width: 15.7 cm ← width = max(35.2, 22.1) - 19.5
```

- [ ] Center distance ~19-22cm (close to obstacle)
- [ ] Left distance > 25cm (clear)
- [ ] Width estimation reasonable (±5cm)
- [ ] Servo returns to center after scan

**Troubleshooting**:
- [ ] All distances same? Servo not moving (check servo wiring)
- [ ] Width negative? Left/right readings too close to center (servo travel)
- [ ] Measurements vary wildly? Servo settling time too short (increase 300ms)

---

### Test 5: Side Selection
**Goal**: Verify correct left/right choice

**Procedure**:
1. Run obstacle scan test with various obstacles
2. Check console output: `Clear side: -1` (LEFT) or `+1` (RIGHT)

**Expected Results**:
- [ ] LEFT chosen when distance_left > distance_right
- [ ] RIGHT chosen when distance_right > distance_left
- [ ] Prefers side with MORE clearance
- [ ] Returns 0 if both blocked (rare)

**Test Cases**:
- [ ] Wide left, narrow right → Choose LEFT ✓
- [ ] Narrow left, wide right → Choose RIGHT ✓
- [ ] Both open, left more → Choose LEFT ✓
- [ ] Both open, right more → Choose RIGHT ✓

---

### Test 6: Avoidance Maneuver
**Goal**: Verify turn and forward execution

**Procedure**:
1. Run avoidance in open area (no obstacles, clear space)
2. Watch robot execute: turn + forward movement
3. Measure/estimate distance traveled

**Expected Results**:
- [ ] Robot turns ~45° in intended direction
- [ ] Robot moves forward ~30-40cm
- [ ] No collision with surroundings
- [ ] Stops and waits for next command
- [ ] Total time: ~4-6 seconds

**Measurement**:
- [ ] Turn accuracy: ±15° (±33% of 45°) acceptable for timing-based
- [ ] Distance accuracy: ±10cm (±30% of 30cm) acceptable for timing-based
- [ ] Smooth motion, no jerking

---

### Test 7: Line Recovery Search
**Goal**: Verify spiral search pattern

**Procedure**:
1. Place robot OFF the black line (white area)
2. Run line search test
3. Wait for robot to find line or timeout

**Expected Results**:
- [ ] Robot starts moving in spiral pattern (right sweep, left sweep, escalate)
- [ ] Searches for 5 seconds maximum
- [ ] Returns `true` if line found within timeout
- [ ] Returns `false` if timeout reached
- [ ] Stops immediately after finding line or timeout

**Success Criteria**:
- [ ] Find line within 5 seconds: >90% success rate
- [ ] False positives <5% (no detecting obstacles as line)

---

## 🎬 Full Integration Testing Checklist

### Test 8: Complete Obstacle Avoidance Cycle
**Goal**: Full system test with state machine

**Setup**:
1. Place obstacle on track, ~40-50cm downstream from start
2. Configure track so robot can go around (clear space on one side)
3. Ensure line can be re-detected after avoidance

**Procedure**:
1. Position robot on line upstream of obstacle
2. Compile and flash full `testdemo3.c`
3. Power on and let it run for 60+ seconds
4. Observe console for state transitions

**Expected State Sequence**:
```
LINE_FOLLOW
  → [obstacle detected]
  → OBSTACLE_DETECT
  → SCANNING [~2 seconds]
  → PLANNING
  → AVOIDING [~4 seconds]
  → SEARCHING [up to 5 seconds]
  → RESUME
  → LINE_FOLLOW [obstacle count = 1]
```

**Success Criteria**:
- [ ] All states execute in order
- [ ] No crashes or hangs
- [ ] Telemetry prints every 200ms
- [ ] Obstacle count incremented
- [ ] Robot resumes line following
- [ ] Can repeat if multiple obstacles placed

**Console Output**: Should see
- [ ] `[DEMO3:TELEM]` lines at ~5 Hz
- [ ] State transitions logged
- [ ] Obstacle measurements printed
- [ ] Side selection decision shown
- [ ] Avoidance plan details
- [ ] Line search result

---

## 🐛 Edge Case Testing Checklist

### Test 9: Very Close Obstacle (<10cm)
- [ ] Detects and stops before collision
- [ ] Scan measurements still reasonable
- [ ] Avoidance turns and moves correctly

### Test 10: Very Wide Obstacle (>30cm)
- [ ] Both sides may show as blocked
- [ ] System handles gracefully (choose right as default)
- [ ] May not complete avoidance (depends on space)

### Test 11: Narrow Clearance (5-10cm)
- [ ] Requires exact side selection
- [ ] Turn angle may need adjustment
- [ ] Test both left and right obstacles

### Test 12: Blocked Path After Avoidance
- [ ] If line not found within 5 seconds, search times out
- [ ] System gracefully resumes line following anyway
- [ ] No infinite loops or hangs

### Test 13: Multiple Obstacles
- [ ] Place 2-3 obstacles on track
- [ ] Robot should handle sequence
- [ ] Each obstacle increments counter
- [ ] Final obstacle count = number of obstacles encountered

### Test 14: Rapid Obstacle Detection
- [ ] Place obstacle very close (10cm)
- [ ] Obstacle scan may fail if too close
- [ ] System handles error gracefully

---

## 📊 Telemetry Verification Checklist

### Console Output Format Check
```
Expected format:
[DEMO3:TELEM] State=<state> Hdg=<heading>° Speed=<speed> cm/s Obstacles=<count>
```

- [ ] State name correct (LINE_FOLLOW, OBSTACLE_DETECT, etc.)
- [ ] Heading in range 0-360 degrees
- [ ] Speed in range 0-30 cm/s
- [ ] Obstacle count increments from 0

### Event Logging Check
- [ ] Obstacle detection message when distance drops
- [ ] State transition messages on each state change
- [ ] Scan measurement lines show 3 distances
- [ ] Avoidance plan shows turn angle and distance
- [ ] Line search result (FOUND or TIMEOUT)

---

## ✅ Final Validation Checklist

### Functionality
- [ ] Line following works for 60+ seconds without interference
- [ ] Obstacle detected within 1 second of placement
- [ ] Scan completes in <3 seconds
- [ ] Avoidance maneuver completes without collision
- [ ] Line recovery succeeds in <5 seconds (>90% of time)

### Performance
- [ ] Control loop runs at ~100 Hz (10ms cycle)
- [ ] No watchdog resets or crashes
- [ ] Telemetry updates at ~5 Hz
- [ ] State transitions logged correctly

### Robustness
- [ ] Works with different obstacle sizes (10-30cm)
- [ ] Works with different clearance widths (5-50cm)
- [ ] Works with multiple obstacles (tested 2-3 sequential)
- [ ] Handles edge cases gracefully (very close, blocked)

### Integration
- [ ] No conflicts with existing code (motor, encoder, IMU, IR)
- [ ] Clean compilation (no warnings as errors)
- [ ] No memory leaks (stack usage reasonable)
- [ ] CMake build is clean and reproducible

---

## 🎯 Success Criteria Summary

**Minimum Success**: 
- Robot follows line normally ✓
- Stops when obstacle detected ✓
- Scans obstacle ✓
- Attempts avoidance ✓
- Searches for line ✓

**Good Success**:
- All above + completes avoidance without collision ✓
- Finds line successfully (>80% of time) ✓
- Resumes line following ✓

**Excellent Success**:
- All above + multiple obstacles handled ✓
- Various obstacle sizes work ✓
- Consistent performance across trials ✓
- Telemetry complete and accurate ✓

---

## 📝 Testing Log Template

```
Date: ___________
Test: ___________
Duration: ___________

Hardware:
- Ultrasonic: [ ] Working
- Servo: [ ] Working
- IR Sensors: [ ] Working

Results:
- Obstacle Detection: [ ] Pass [ ] Fail
- Avoidance: [ ] Pass [ ] Fail
- Line Recovery: [ ] Pass [ ] Fail

Issues Found:
_________________________________

Fixes Applied:
_________________________________

Next Steps:
_________________________________
```

---

## 🔗 Quick Links

| Document | Use When |
|----------|----------|
| DEMO3_QUICK_START.md | Testing step-by-step |
| DEMO3_IMPLEMENTATION_GUIDE.md | Understanding implementation details |
| DEMO3_ARCHITECTURE_SUMMARY.md | System design and performance |
| README_DEMO3.md | Getting started overview |
| This Checklist | Tracking progress and validation |

---

## ⏱️ Estimated Timeline

| Phase | Duration | Status |
|-------|----------|--------|
| Component Testing | 30 min | TODO |
| Subsystem Testing | 45 min | TODO |
| Full Integration | 60 min | TODO |
| Edge Cases & Tuning | 30 min | TODO |
| **Total** | **~2.5 hours** | TODO |

---

**Start with Component Testing → Subsystem Testing → Full Integration**

**Reference this checklist frequently for validation!**

