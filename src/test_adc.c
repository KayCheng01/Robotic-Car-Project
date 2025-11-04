#include "pico/stdlib.h"
#include "hardware/adc.h"
#include <stdio.h>

int main() {
    stdio_init_all();   // Enable serial output
    adc_init();         // Initialize ADC hardware

    // Choose ADC input pin (e.g. GP26 = ADC0)
    adc_gpio_init(28);
    adc_select_input(2); // 0 = ADC0, 1 = ADC1, etc.

    const float conversion_factor = 3.3f / (1 << 12); // 12-bit ADC → 4096 levels

    while (true) {
        uint16_t raw = adc_read();                    // Read raw ADC value
        float voltage = raw * conversion_factor;      // Convert to volts
        printf("Raw: %u  |  Voltage: %.3f V\n", raw, voltage);
        sleep_ms(200);
    }
}
