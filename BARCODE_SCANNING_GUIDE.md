# Barcode Scanning Setup Guide

## Hardware Configuration

| Component | GPIO | ADC Channel | Purpose |
|-----------|------|-------------|---------|
| **Barcode IR Sensor** | GPIO 27 | — | Digital edge detection (bar timing) |
| **Intensity ADC** | GPIO 26 | ADC0 | Black/white intensity reading |
| **Line Following** | GPIO 28 | ADC2 | Reserved for demo2.c (do NOT use) |

## Files Available

### 1. `barcode_test_ir.c` (Testing & Calibration)
- **Purpose**: Characterize your barcode and determine thresholds
- **Features**:
  - Captures bar timing data
  - Reads ADC intensity values
  - Analyzes black/white levels
  - Generates recommended thresholds
- **To Use**:
  ```cmake
  # In src/CMakeLists.txt, uncomment:
  barcode_test_ir.c
  ```

### 2. `barcodedemo2.c` (Production Barcode Movement)
- **Purpose**: Scan barcode and control robot movement
- **Features**:
  - Code39 barcode decoding
  - Letter-based movement control:
    - **A, C, E, G, I, ...** → Turn LEFT
    - **B, D, F, H, J, ...** → Turn RIGHT
  - FreeRTOS task integration
- **To Use**:
  ```cmake
  # In src/CMakeLists.txt, uncomment:
  barcodedemo2.c
  ```

## Step-by-Step Barcode Scanning Workflow

### Phase 1: Calibration (Using barcode_test_ir.c)

1. **Build and Flash**
   ```powershell
   cd c:\Users\caiwe\Robotic-Car-Project\build
   cmake --build .
   # Flash main.uf2 to Pico W
   ```

2. **Connect Serial Monitor**
   - Baud rate: **115200**
   - Terminal program (PuTTY, Arduino IDE, or VS Code Serial Monitor)

3. **Run the Test**
   - Program starts and waits for barcode
   - Output shows: `[READY] Ready for next scan...`

4. **Scan Your Barcode**
   - Slowly pass your printed barcode over GPIO 27 IR sensor
   - Program captures edge transitions
   - Auto-completes after 9 bars (Code39 standard)

5. **Review Output**
   ```
   ========== BAR TIMING DATA ==========
   Bar #  | Timing (µs) | ADC Value | Comment
   -------|-------------|-----------|----------
   0      |       3421  |      1200 | NARROW
   1      |       2156  |      3500 | NARROW
   ...
   
   ========== BLACK/WHITE INTENSITY ANALYSIS ========
   BLACK (dark bar)  ADC: 1200
   WHITE (light bar) ADC: 3500
   Average ADC:      2350
   ADC Range:        2300
   
   ========== RECOMMENDED THRESHOLD ==========
   Threshold Value: 2350
     - ADC <= 2350: BLACK bar
     - ADC > 2350:  WHITE space
   ```

6. **Note Your Values**
   - BLACK intensity: `____`
   - WHITE intensity: `____`
   - Recommended threshold: `____`

### Phase 2: Production (Using barcodedemo2.c)

1. **Update barcodedemo2.c** (if needed with your thresholds)
   ```c
   // In barcodedemo2.c, line ~50:
   #define BLACK_THRESHOLD 2350  // Update with your value
   ```

2. **Switch to Production Build**
   ```cmake
   # In src/CMakeLists.txt:
   # barcode_test_ir.c
   barcodedemo2.c
   ```

3. **Rebuild**
   ```powershell
   cd c:\Users\caiwe\Robotic-Car-Project\build
   cmake --build .
   ```

4. **Flash and Run**
   - Flash `main.uf2` to Pico W
   - Program is ready for barcode scanning
   - Scan a barcode → robot executes movement based on letter

## Expected Behavior

### During Test (barcode_test_ir.c)
```
[INIT] All systems initialized
[INFO] Move your printed barcode slowly over the IR sensor
[MONITOR] GPIO27=1 | ADC=0000 | Waiting...
[SCAN START] First edge detected
[BAR  0] Timing:    3421 µs | ADC: 1200 | GPIO: 0
[BAR  1] Timing:    2156 µs | ADC: 3500 | GPIO: 1
...
[SCAN COMPLETE] Captured 9 bars
[Analysis output...]
[READY] Ready for next scan...
```

### During Production (barcodedemo2.c)
```
[BARCODE] Decoded character: A
[MOVE] Letter A (position 0) - Turning LEFT
[MOVE] STOP
[READY] Ready for next scan...
```

## Troubleshooting

### Problem: No bars captured
- **Check**: Is your barcode printed with clear black and white contrast?
- **Fix**: Try slower movement over the sensor
- **Fix**: Check GPIO 27 connection to IR sensor

### Problem: ADC values all the same
- **Check**: Is GPIO 26 actually connected to the sensor?
- **Fix**: Verify ADC is reading the intensity signal

### Problem: Timing values erratic
- **Check**: Debounce delay too short?
- **Fix**: Increase `BARCODE_DEBOUNCE_US` in config

### Problem: Cannot decode barcode
- **Check**: Is the timing showing clear narrow/wide pattern?
- **Fix**: Ensure barcode pattern matches Code39 standard
- **Fix**: Try different scanning speed

## Code39 Barcode Format

- **Standard**: 3 wide bars + 2 wide spaces per character
- **Bars**: 5 black + 4 white alternating
- **Delimiter**: Start with `*` character
- **Format**: `*[DATA]*` (e.g., `*HELLO*`)
- **Timing**: Wide bars/spaces = 2-3x narrow bars/spaces

## Next Steps

1. ✅ Build `barcode_test_ir.c`
2. ✅ Flash and calibrate with your printed barcode
3. ✅ Record the threshold values
4. ✅ Update `barcodedemo2.c` with your thresholds
5. ✅ Build and test movement control
6. ✅ Integrate with line-following (use GPIO 28 in demo2.c)

---
**Note**: GPIO 28 is exclusively for line-following sensor. Do NOT use it for barcode testing.
