#ifndef BARCODE_TASK_H
#define BARCODE_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>

// Enable verbose debug output (comment out for production)
// #define BARCODE_DEBUG

// Create and start the barcode decoder task
void barcode_task_create(void);

// Get the last decoded barcode string (thread-safe)
// Returns true if a new barcode was decoded since last call
bool barcode_get_result(char *buf, size_t buf_size);

// Reset the barcode detection state to allow new scans
void barcode_reset(void);

// Check if a barcode scan is complete
bool barcode_scan_complete(void);

#endif // BARCODE_TASK_H
