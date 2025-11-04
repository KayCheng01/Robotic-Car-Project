# Barcode Timing Implementation - GPIO 27

## Overview
This implementation uses **continuous ADC polling** on GPIO 26 to detect black/white transitions and measures the duration of each element in the barcode.

## Hardware Configuration
- **GPIO 27**: Barcode IR sensor (not used directly - we read ADC instead)
- **GPIO 26 ADC0**: Intensity reading for black/white detection
- **GPIO 28 ADC2**: RESERVED for line-following (demo2.c) - NOT USED HERE

## How It Works

### 1. Continuous Monitoring (1ms sampling rate)
- Task continuously reads ADC value from GPIO 26
- Compares against `BLACK_THRESHOLD` (default 2000)
- Determines if sensor sees BLACK bar (high ADC) or WHITE space (low ADC)

### 2. Scan Trigger
- Scan starts automatically when BLACK bar detected (ADC >= 2000)
- Records start time of first black bar
- System is now "scanning"

### 3. Element Timing
For each black/white element:
- **Current state**: BLACK or WHITE
- **Duration**: Time from start to transition
- **Transition**: When ADC crosses threshold (BLACK ↔ WHITE)
- **On transition**: 
  - Calculate duration of previous element
  - Store: `duration_us`, `is_black`, `adc_value`
  - Print element data
  - Update state and start new element timer

### 4. Scan Completion
- After capturing 9 elements (5 black + 4 white)
- Automatically stops scanning
- Triggers analysis task

### 5. Analysis Output
```
========== BARCODE TIMING DATA (9 ELEMENTS) ==========
Element # | Type  | Duration (µs) | ADC Value | Width
----------|-------|---------------|-----------|--------
0         | BLACK |          5230 |      3421 | WIDE
1         | WHITE |          2100 |       450 | NARROW
2         | BLACK |          2150 |      3500 | NARROW
...

========== STATISTICS ==========
Total elements: 9 (should be 9 for Code39)
Min timing: 2100 µs
Max timing: 5230 µs
Estimated threshold: 3665 µs
  NARROW <= 3665 µs
  WIDE   >  3665 µs

Binary pattern (0=narrow, 1=wide):
101001000

Wide elements: 3, Narrow elements: 6
Expected for Code39: 3 wide + 6 narrow
```

## Code39 Barcode Structure
- **9 elements per character**
- **5 black bars + 4 white spaces**
- **3 wide + 6 narrow** elements
- Example: `101001000` = 3 wide (positions 0, 2, 5) + 6 narrow

## Advantages of This Approach
1. ✅ **No edge detection interrupts** - simpler, more reliable
2. ✅ **Measures actual duration** - accounts for fixed scanning speed
3. ✅ **ADC-based black/white** - works with analog sensors
4. ✅ **Real-time element printing** - see data as it's captured
5. ✅ **Automatic analysis** - generates statistics and binary pattern

## Configuration

### Adjust Black Threshold
```c
#define BLACK_THRESHOLD 2000  // Increase if false positives, decrease if missing blacks
```

### Adjust Sampling Rate
```c
const TickType_t sample_delay = pdMS_TO_TICKS(1); // Currently 1ms
```

### Expected Timing
For a barcode moving at fixed speed:
- If scanning takes ~20ms total
- Each narrow element: ~2ms
- Each wide element: ~4-6ms (2-3x narrow)

## Usage Instructions

1. **Flash main.uf2** to Pico W
2. **Open serial monitor** (115200 baud)
3. **Wait for**: `[MONITOR] ADC=... | Waiting for black bar...`
4. **Move barcode** at **constant speed** over sensor
5. **Watch real-time output** as elements are captured
6. **Review analysis** when complete
7. **Ready for next scan** automatically

## Troubleshooting

### No scan starting
- ADC value not reaching BLACK_THRESHOLD (2000)
- Adjust threshold lower

### Elements captured too fast/slow
- Scanning speed inconsistent
- Need to maintain fixed speed

### Wrong number of elements
- Speed varies during scan
- Threshold might need adjustment
- Check ADC values are clearly separated (black vs white)

### Timing values erratic
- Scanning speed not constant
- Use motor-driven scan for best results

## Next Steps

Once calibrated:
1. Note the narrow/wide threshold
2. Note the binary pattern for test character
3. Update `barcodedemo2.c` with your threshold
4. Test with multiple barcode characters
5. Verify pattern matches Code39 standard
