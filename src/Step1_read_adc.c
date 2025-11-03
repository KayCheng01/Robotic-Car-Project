#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define LINE_SENSOR_AO 28     // GP28 connected to IR AO pin
#define ADC_CHANNEL    2      // ADC2 for GP28

int main() {
    stdio_init_all();
    sleep_ms(500);
    printf("=== Step 1: ADC Raw Reading Test ===\n");

    // Initialize ADC and GPIO
    adc_init();
    adc_gpio_init(LINE_SENSOR_AO);
    adc_select_input(ADC_CHANNEL);

    while (true) {
        uint16_t raw = adc_read();             // 0–4095
        float voltage = raw * 3.3f / 4095.0f;  // optional, for curiosity
        printf("ADC raw=%u  voltage=%.3f V\n", raw, voltage);
        sleep_ms(100);                         // 10 readings/sec
    }
}
