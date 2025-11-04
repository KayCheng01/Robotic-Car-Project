// ==============================
// main.c — Generic Band Scanner + (optional) Code-39 decode via string tables
// ==============================
// - IR module DO connected to DO_PIN (this module is HIGH on black, LOW on white).
// - Captures rise/fall edges, converts to widths (µs), classifies to N/W.
// - Prints widths + N/W sequence for the whole pass.
// - Then attempts Code-39 decode using your string tables (forward + reverse).
//   Requires '*' sentinels if you want a "confirmed" decode.
// ==============================

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"

// ---------- CONFIG ----------
#define DO_PIN               26      // your DO pin
#define ACTIVE_LOW_BLACK     0       // 0: HIGH on black (your module), 1: LOW on black
#define EDGE_BUF             512     // max edges per pass
#define IDLE_GAP_US          300000  // pass ends after 300 ms without edges (tune 200k–600k)
#define RATIO_NARROW_MAX     1.9f    // width <= median*R => Narrow, else Wide (try 1.7–2.3)
#define MIN_EDGES_TO_PROCESS 6       // ignore tiny/noisy captures
#define MAX_CHARS_OUT        48
#define DEBUG_BITS           0       // 1 = print each 9-bit window checked

// ---------- Edge buffer ----------
static volatile struct {
    uint64_t t[EDGE_BUF];
    uint16_t n;
    uint64_t last_t;
    bool recording;      // true when we're actively recording a barcode
    bool first_is_black; // true if first edge was rising (entering black)
} gbuf;

// ---------- IRQ ----------
static void irq_cb(uint gpio, uint32_t events){
    (void)gpio;
    
    // Detect if we're entering a black bar (rising edge since HIGH=black)
    if (!gbuf.recording && (events & GPIO_IRQ_EDGE_RISE)) {
        // Start recording when we detect entry into a black bar
        gbuf.recording = true;
        gbuf.first_is_black = true;
        gbuf.n = 0; // Reset buffer for fresh start
    }
    
    // Only record edges if we're in recording mode
    if (gbuf.recording && gbuf.n < EDGE_BUF) {
        gbuf.t[gbuf.n++] = time_us_64();
    }
    
    gbuf.last_t = time_us_64();
}

static void gpio_init_do(void){
    gpio_init(DO_PIN);
    gpio_set_dir(DO_PIN, GPIO_IN);
    // Module drives the line; leave pulls off.
    gpio_disable_pulls(DO_PIN);
    gpio_set_irq_enabled_with_callback(DO_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &irq_cb);
}

// ---------- Utils ----------
static float median_u32(uint32_t *a, int n){
    for (int i=1;i<n;i++){
        uint32_t x=a[i]; int j=i-1;
        while (j>=0 && a[j]>x){ a[j+1]=a[j]; j--; }
        a[j+1]=x;
    }
    return (n&1) ? (float)a[n/2] : 0.5f*(a[n/2-1] + a[n/2]);
}

// Convert widths -> N/W flags with threshold = median * RATIO_NARROW_MAX
static float widths_to_nw(const uint32_t *w, int wn, uint8_t *nw_out){
    uint32_t tmp[EDGE_BUF];
    for (int i=0;i<wn;i++) tmp[i]=w[i];
    float med = median_u32(tmp, wn);
    float thr = med * RATIO_NARROW_MAX;
    for (int i=0;i<wn;i++) nw_out[i] = (w[i] <= (uint32_t)thr) ? 0 : 1; // 0=Narrow,1=Wide
    return thr;
}

// ---------- Your Code-39 tables (strings) ----------
// Original set + '*' sentinel added at end.
// If your sheet uses '-' instead of '_', just change '_' to '-'.
#define TOTAL_CHAR 44
static const char char_array[TOTAL_CHAR] = {
  '0','1','2','3','4','5','6','7','8','9',
  'A','B','C','D','E','F','G','H','I','J',
  'K','L','M','N','O','P','Q','R','S','T',
  'U','V','W','X','Y','Z','_', '.', '$','/','+','%',' ', '*'
};

static const char *code_array[TOTAL_CHAR] = {
  "000110100","100100001","001100001","101100000","000110001","100110000","001110000",
  "000100101","100100100","001100100","100001001","001001001","101001000","000011001",
  "100011000","001011000","000001101","100001100","001001100","000011100","100000011",
  "001000011","101000010","000010011","100010010","001010010","000000111","100000110",
  "001000110","000010110","110000001","011000001","111000000","010010001","110010000",
  "011010000","010000101","110000100","010101000","010100010","010001010","000101010",
  "011000100","100010110" // '*'
};

static const char *reverse_code_array[TOTAL_CHAR] = {
  "001011000","100001001","100001100","000001101","100011000","000011001",
  "000011100","101001000","001001001","001001100","100100001","100100100",
  "000100101","100110000","000110001","000110100","101100000","001100001",
  "001100100","001110000","110000001","110000100","010000101","110010000",
  "010010001","010010100","111000000","011000001","011000100","011010000",
  "100000011","100000110","000000111","100010010","000010011","000010110",
  "101000010","001000011","000101010","010001010","010100010","010101000",
  "001000110","011010001" // '*'
};

// Turn 9 NW bits (0/1) at offset -> "001010110" and lookup in tables.
static int decode_c39_bits(const uint8_t *nw, int off){
    char bits[10];
    for (int i=0;i<9;i++) bits[i] = nw[off+i] ? '1':'0';
    bits[9] = '\0';
#if DEBUG_BITS
    printf("[C39] try bits[%02d]=%s\n", off, bits);
#endif
    for (int k=0;k<TOTAL_CHAR;k++){
        if (strcmp(bits, code_array[k])==0 || strcmp(bits, reverse_code_array[k])==0)
            return (int)char_array[k];
    }
    return -1;
}

// Try to decode a stream of NW bits as Code-39 characters.
// Slides across; on success, jumps by 10 (9 modules + 1 inter-char gap).
static int try_decode_code39(const uint8_t *nw, int wn){
    char out[MAX_CHARS_OUT]; int outn=0;
    for (int i=0; i+9<=wn && outn < MAX_CHARS_OUT-1; ){
        int ch = decode_c39_bits(nw, i);
        if (ch < 0) { i++; continue; }    // resync
        out[outn++] = (char)ch;
        i += 10;
    }
    out[outn] = '\0';

    if (outn >= 2 && out[0]=='*' && out[outn-1]=='*'){
        printf("[C39] \"%s\"\n", out); // keep '*' visible
        return 0;
    }
    if (outn>0) printf("[C39] partial=\"%s\" (no/invalid sentinels)\n", out);
    else        printf("[C39] no characters matched\n");
    return -1;
}

// ---------- Band scanner task ----------
static void bandTask(void *p){
    (void)p;
    gbuf.n = 0;
    gbuf.last_t = time_us_64();
    gbuf.recording = false;
    gbuf.first_is_black = false;

    for(;;){
        uint64_t now = time_us_64();

        // A pass ends when we have edges and then a long enough quiet gap
        if (gbuf.recording && gbuf.n > MIN_EDGES_TO_PROCESS && (now - gbuf.last_t) > IDLE_GAP_US){
            // Snapshot
            uint16_t n = gbuf.n; if (n > EDGE_BUF) n = EDGE_BUF;
            uint64_t t[EDGE_BUF];
            for (uint16_t i=0;i<n;i++) t[i] = gbuf.t[i];
            
            // Reset for next scan
            gbuf.n = 0;
            gbuf.recording = false; // Stop recording until next black bar detected

            // Compute widths
            if (n < 2) continue;
            uint32_t w[EDGE_BUF]; int wn=0;
            for (uint16_t i=1;i<n;i++) w[wn++] = (uint32_t)(t[i]-t[i-1]);

            // First bar is black (we started recording on rising edge = entering black)
            bool is_black = true;
            
            printf("\n========== BARCODE SCAN (Started on BLACK bar) ==========\n");
            printf("Individual Bar Timings (µs):\n");
            
            // Print each bar with its color and timing
            for (int i=0; i<wn; i++){
                printf("  Bar %2d: %s %5u µs", i+1, is_black ? "BLACK" : "WHITE", w[i]);
                
                // Print character boundary markers (every 9 bars = 1 character in Code-39)
                if ((i+1) % 9 == 0){
                    printf("  <-- End of Character %d", (i+1)/9);
                }
                printf("\n");
                
                // Toggle color for next bar
                is_black = !is_black;
            }
            printf("=========================================================\n\n");

            // Threshold and classify
            uint8_t nw[EDGE_BUF];
            float thr = widths_to_nw(w, wn, nw);

            // Print widths summary
            printf("[BANDS] widths(us):");
            int show = (wn<24)? wn:24;
            for (int i=0;i<show;i++) printf(" %u", w[i]);
            if (wn>show) printf(" ...");
            printf("\n[BANDS] thr≈%.0f us (N<=thr, W>thr)\n", thr);

            // Print N/W sequence
            printf("[BANDS] seq: ");
            for (int i=0;i<wn;i++){ putchar(nw[i]?'W':'N'); putchar(' '); }
            putchar('\n');

            // Attempt Code-39 decode
            (void)try_decode_code39(nw, wn);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------- main ----------
int main(void){
    stdio_init_all();
    sleep_ms(5000);

    gpio_init_do();

    int lvl = gpio_get(DO_PIN);
    printf("[MAIN] Band scanner. DO=GP%d (%s on black). Level=%d\n",
           DO_PIN, ACTIVE_LOW_BLACK ? "LOW" : "HIGH", lvl);
    printf("Slide the barcode steadily under the sensor.\n");

    xTaskCreate(bandTask, "Bands", configMINIMAL_STACK_SIZE*3,
                NULL, tskIDLE_PRIORITY+1, NULL);

    vTaskStartScheduler();
    while (true) { tight_loop_contents(); }
    return 0;
}
