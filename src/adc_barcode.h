#ifndef ADC_BARCODE_H
#define ADC_BARCODE_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

// ADC configuration for barcode sensor
#define BARCODE_ADC_PIN         27
#define BARCODE_ADC_INPUT       1       // GPIO27 = ADC1
#define BARCODE_ADC_THRESHOLD   2000
#define BARCODE_ADC_HYSTERESIS  150
#define BARCODE_SAMPLE_RATE_HZ  15000   // 15 kHz sampling

// Edge event structure
typedef struct {
    uint32_t timestamp_us;
    bool is_black;  // true = black bar, false = white space
} barcode_edge_t;

// Initialize ADC+DMA for barcode scanning on GPIO27/ADC1
void adc_barcode_init(void);

// Get the edge event queue handle (for task consumption)
QueueHandle_t adc_barcode_get_queue(void);

// Start/stop DMA sampling
void adc_barcode_start(void);
void adc_barcode_stop(void);

#endif // ADC_BARCODE_H
