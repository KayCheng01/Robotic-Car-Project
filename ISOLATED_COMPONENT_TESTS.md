# Isolated Component Testing Guide

## Overview
Two minimal test runners allow you to test ultrasonic and servo independently before running the full Demo 3.

Files:
- `src/test_ultrasonic_only.c` — Distance sensor loop
- `src/test_servo_only.c` — Servo position sweep
- `src/testdemo3.c` — Full integration (default)

---

## Test 1: Ultrasonic Sensor Only (10 min)

### Step 1: Swap to test file
Edit `src/CMakeLists.txt` and uncomment the test file:

```cmake
add_executable(main
    ...
    #testdemo3.c              ← Comment out
    #demo3_obstacle.c         ← Comment out
    #demo3_line_recovery.c    ← Comment out
    test_ultrasonic_only.c    ← Uncomment
    #test_servo_only.c
)
```

### Step 2: Rebuild
```powershell
cd C:\Users\caiwe\Robotic-Car-Project\build
cmake .. 
make
```

### Step 3: Flash
```powershell
Copy-Item -Path .\src\main.uf2 -Destination "E:\" # Replace E: with your Pico drive
```
(Or hold BOOTSEL and drag main.uf2 to Pico drive in Explorer)

### Step 4: Test
- Open serial monitor (115200 baud, COM port from Device Manager)
- You should see:
  ```
  === Ultrasonic Sensor Test ===
  Initializing ultrasonic sensor...
  Initialized OK
  Place objects at known distances and observe readings.
  
  [   0] Distance: 245.3 cm (NO OBJECT)
  [   1] Distance: 242.1 cm (NO OBJECT)
  ```

- **Test procedure:**
  1. Place a flat object (ruler, book, hand) at exactly 5 cm
  2. Watch the readings stabilize
  3. Slowly move from 5 → 10 → 20 → 50 cm
  4. Verify readings decrease as object approaches

- **Pass criteria:**
  - Readings roughly match object distance (within ±2 cm is good)
  - No persistent `ERROR` (timeout)
  - Readings update every 500ms

- **Troubleshooting:**
  - All readings `ERROR` or `-1`: Check GPIO 4 (ECHO) wiring; verify it's 3.3V level (not 5V); add 100nF cap on ECHO if noisy
  - Readings stuck > 400cm: No echo detected; check GPIO 7 (TRIG) wiring
  - Jittery values: Normal; average is what matters; try placing object further away

### Step 5: Log results
Save output to file for reference:
```powershell
# In PuTTY: Session → Logging → browse to file.log
# Or in miniterm: Ctrl+R to redirect (if supported)
```

**Sample pass output:**
```
[   5] Distance: 5.2 cm ✓
[   6] Distance: 5.1 cm ✓
[  15] Distance: 9.8 cm ✓
[  25] Distance: 20.3 cm ✓
[  50] Distance: 49.9 cm ✓
```

---

## Test 2: Servo Motor Only (10 min)

### Step 1: Swap to test file
Edit `src/CMakeLists.txt`:

```cmake
add_executable(main
    ...
    #testdemo3.c              ← Comment out
    #demo3_obstacle.c         ← Comment out
    #demo3_line_recovery.c    ← Comment out
    #test_ultrasonic_only.c
    test_servo_only.c         ← Uncomment
)
```

### Step 2: Rebuild
```powershell
cd C:\Users\caiwe\Robotic-Car-Project\build
cmake .. 
make
```

### Step 3: Flash
```powershell
Copy-Item -Path .\src\main.uf2 -Destination "E:\"
```

### Step 4: Test
- Open serial monitor (115200 baud)
- You should see:
  ```
  === Servo Motor Test ===
  Initializing servo on GPIO5 (PWM)...
  [SERVO] Init OK (pin=5, slice=2, ch=0)
  Servo initialized. Starting sweep test.
  Observe physical servo movement...
  
  --- Cycle 1 ---
  Moving to LEFT (0°)...
  LEFT position reached. Holding 1 second...
  Moving to CENTER (90°)...
  CENTER position reached. Holding 1 second...
  Moving to RIGHT (180°)...
  RIGHT position reached. Holding 1 second...
  Cycle 1 complete.
  ```

- **Physical observation:**
  - Servo arm should rotate smoothly from left (0°) → center (90°) → right (180°)
  - Each transition should complete in ~300 ms
  - No grinding, buzzing, or humming
  - Arm holds position firmly during 1-second pause

- **Pass criteria:**
  - Servo responds to all three commands
  - Movement is smooth (not jerky)
  - No mechanical noise
  - Holds position without drift

- **Troubleshooting:**
  - No movement: 
    - Check servo power (should be 5V, separate from Pico USB)
    - Verify GPIO 5 is PWM-capable (it is—it's on PWM slice 2)
    - Ensure servo signal is connected to GPIO 5
  - Jittering at rest:
    - Check servo power supply is stable and regulated
    - Add a capacitor (220µF) on servo power lines
    - Separate servo power from motor power if using same supply
  - Weak movement:
    - Increase servo power to 5V (not 3.3V)
    - Or use external battery for servo

### Step 5: Verify positions
- Mark servo position at each angle using tape or marker
- Measure with protractor or estimate by eye
- Confirm 0° is fully left, 90° is center, 180° is fully right

---

## Test 3: Back to Full Integration

### Step 1: Restore main test file
Edit `src/CMakeLists.txt`:

```cmake
add_executable(main
    ...
    testdemo3.c              ← Uncomment
    demo3_obstacle.c         ← Uncomment
    demo3_line_recovery.c    ← Uncomment
    #test_ultrasonic_only.c
    #test_servo_only.c
)
```

### Step 2: Rebuild
```powershell
cd C:\Users\caiwe\Robotic-Car-Project\build
cmake .. 
make
```

### Step 3: Flash full demo
```powershell
Copy-Item -Path .\src\main.uf2 -Destination "E:\"
```

### Step 4: Full integration test (next phase)
See **Physical Testing Guide** for step-by-step full Demo 3 execution with obstacle on track.

---

## Quick Reference: Swapping Between Tests

| Test | Uncomment | Comment Out |
|------|-----------|-------------|
| **Ultrasonic** | `test_ultrasonic_only.c` | `testdemo3.c`, `demo3_obstacle.c`, `demo3_line_recovery.c` |
| **Servo** | `test_servo_only.c` | `testdemo3.c`, `demo3_obstacle.c`, `demo3_line_recovery.c` |
| **Full Demo 3** | `testdemo3.c`, `demo3_obstacle.c`, `demo3_line_recovery.c` | `test_ultrasonic_only.c`, `test_servo_only.c` |

Then:
```powershell
cd build && cmake .. && make && Copy-Item -Path .\src\main.uf2 -Destination "E:\"
```

---

## Expected Timeline

| Test | Duration | Status |
|------|----------|--------|
| Setup & wiring | 10 min | |
| Ultrasonic component test | 10 min | |
| Servo component test | 10 min | |
| Full integration test | 30–60 min | |
| **Total** | **60–90 min** | |

---

## Logs to Capture
For debugging, save serial output from each test:

```powershell
# Using PuTTY: Session → Logging (browse to file)
# Using Tera Term: File → Log... (select file)
# Using miniterm + redirect:
python -m serial.tools.miniterm COM3 115200 --logfile=ultrasonic_test.log
```

Keep logs for each test run with naming convention: `demo3_ultrasonic_run1.log`, `demo3_servo_run1.log`, etc.

---

## What's Next
After both component tests pass:
1. Restore `testdemo3.c` as the main target
2. Build and flash full integration
3. Set up track with obstacle
4. Follow **Physical Testing Guide** for full Demo 3 validation

---

**Questions?** Check pin definitions:
- **Ultrasonic**: `ultrasonic/ultrasonic.h` → `ULTRASONIC_TRIG_PIN` (GPIO 7), `ULTRASONIC_ECHO_PIN` (GPIO 4)
- **Servo**: `servo/servo.h` → `SERVO_PIN` (GPIO 5)
- Adjust if your wiring differs!
