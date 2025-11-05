#include "line_sensor.h"
#include "hardware/gpio.h"

// Line state (updated by IRQ)
static volatile bool line_state = false;  // false = white, true = black

// GPIO IRQ handler
static void line_sensor_irq_handler(uint gpio, uint32_t events) {
    if (gpio == LINE_SENSOR_DO_PIN) {
        // Read current level
        // Assuming DO is HIGH on black, LOW on white
        line_state = gpio_get(LINE_SENSOR_DO_PIN);
    }
}

void line_sensor_init(void) {
    // Configure GPIO
    gpio_init(LINE_SENSOR_DO_PIN);
    gpio_set_dir(LINE_SENSOR_DO_PIN, GPIO_IN);
    gpio_pull_down(LINE_SENSOR_DO_PIN);  // Default pull-down
    
    // Read initial state
    line_state = gpio_get(LINE_SENSOR_DO_PIN);
    
    // Set up IRQ on both edges
    gpio_set_irq_enabled_with_callback(LINE_SENSOR_DO_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &line_sensor_irq_handler);
}

bool line_sensor_is_on_track(void) {
    return line_state;
}
