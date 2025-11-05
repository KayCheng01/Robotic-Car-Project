#include "adc_barcode.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include <string.h>
#include <stdio.h>

// DMA configuration
#define DMA_BUFFER_SIZE  256
#define EDGE_QUEUE_SIZE  64
#define MIN_PULSE_WIDTH_US 30  // Ignore pulses shorter than this (glitch filter)

static uint16_t dma_buf_a[DMA_BUFFER_SIZE];
static int dma_chan;
static QueueHandle_t edge_queue = NULL;

// Edge detection state (for hysteresis and glitch filtering)
static bool current_state = false;  // false = white, true = black
static uint32_t last_edge_time_us = 0;

// Process a DMA buffer and detect edges
static void process_buffer(const uint16_t *buffer, size_t count) {
    for (size_t i = 0; i < count; i++) {
        uint16_t sample = buffer[i];
        bool is_black;
        
        // Hysteresis threshold
        if (current_state) {
            // Currently black: need to drop below (threshold - hysteresis) to go white
            is_black = (sample > (BARCODE_ADC_THRESHOLD - BARCODE_ADC_HYSTERESIS));
        } else {
            // Currently white: need to rise above (threshold + hysteresis) to go black
            is_black = (sample > (BARCODE_ADC_THRESHOLD + BARCODE_ADC_HYSTERESIS));
        }
        
        // Edge detected?
        if (is_black != current_state) {
            uint32_t now = (uint32_t)time_us_64();
            uint32_t pulse_width = now - last_edge_time_us;
            
            // Glitch filter: ignore very short pulses
            if (pulse_width >= MIN_PULSE_WIDTH_US || last_edge_time_us == 0) {
                barcode_edge_t edge = {
                    .timestamp_us = now,
                    .is_black = is_black
                };
                
                BaseType_t higher_prio = pdFALSE;
                xQueueSendFromISR(edge_queue, &edge, &higher_prio);
                portYIELD_FROM_ISR(higher_prio);
                
                current_state = is_black;
                last_edge_time_us = now;
            }
        }
    }
}

// DMA IRQ handler
static void dma_irq_handler(void) {
    if (dma_channel_get_irq0_status(dma_chan)) {
        dma_channel_acknowledge_irq0(dma_chan);
        
        // Process buffer (includes edge detection)
        process_buffer(dma_buf_a, DMA_BUFFER_SIZE);
        
        // Restart the DMA transfer
        dma_channel_set_write_addr(dma_chan, dma_buf_a, true); // true = trigger immediately
    }
}

void adc_barcode_init(void) {
    // Create edge event queue
    edge_queue = xQueueCreate(EDGE_QUEUE_SIZE, sizeof(barcode_edge_t));
    configASSERT(edge_queue != NULL);
    
    printf("[ADC_BC] Initializing barcode ADC on GPIO27/ADC1...\n");
    
    // Initialize ADC
    adc_init();
    adc_gpio_init(BARCODE_ADC_PIN);
    adc_select_input(BARCODE_ADC_INPUT);
    
    printf("[ADC_BC] ADC configured\n");
    
    // Set ADC clock divider for target sample rate
    // ADC runs at 48 MHz, we want ~15 kHz sampling
    // 48 MHz / 15 kHz = 3200 cycles per sample
    // Each conversion takes 96 cycles minimum, so divider = 3200/96 ≈ 33
    adc_set_clkdiv(48000000.0f / (BARCODE_SAMPLE_RATE_HZ * 96.0f));
    
    // Configure ADC FIFO
    adc_fifo_setup(
        true,    // enable FIFO
        true,    // enable DMA requests
        1,       // DREQ threshold
        false,   // no error bit
        false    // no byte shift (keep 12-bit samples)
    );
    
    // Claim DMA channel
    dma_chan = dma_claim_unused_channel(true);
    
    printf("[ADC_BC] Claimed DMA channel: %d\n", dma_chan);
    
    // Configure DMA channel
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);  // 16-bit ADC samples
    channel_config_set_read_increment(&cfg, false);  // Read from same ADC FIFO address
    channel_config_set_write_increment(&cfg, true);  // Write to incrementing buffer
    channel_config_set_dreq(&cfg, DREQ_ADC);         // Pace by ADC FIFO
    
    // Configure the channel to write to buf_a initially
    dma_channel_configure(
        dma_chan,
        &cfg,
        dma_buf_a,              // Initial write address
        &adc_hw->fifo,          // Read from ADC FIFO
        DMA_BUFFER_SIZE,        // Transfer count
        false                   // Don't start yet
    );
    
    printf("[ADC_BC] DMA channel configured\n");
    
    // Enable DMA interrupt
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    
    printf("[ADC_BC] DMA channel %d configured with IRQ\n", dma_chan);
    
    // Reset edge detection state
    current_state = false;
    last_edge_time_us = 0;
    
    printf("[ADC_BC] Initialization complete\n");
}

void adc_barcode_start(void) {
    printf("[ADC_BC] Starting ADC and DMA...\n");
    
    // Clear FIFO
    adc_fifo_drain();
    
    // Start free-running ADC
    adc_run(true);
    
    // Start DMA
    dma_channel_start(dma_chan);
    
    printf("[ADC_BC] ADC running, DMA active\n");
}

void adc_barcode_stop(void) {
    // Stop DMA
    dma_channel_abort(dma_chan);
    
    // Stop ADC
    adc_run(false);
    adc_fifo_drain();
}

QueueHandle_t adc_barcode_get_queue(void) {
    return edge_queue;
}
