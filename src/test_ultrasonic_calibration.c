#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "../ultrasonic/ultrasonic.h"

// ============== Calibration Configuration ==============
#define NUM_SAMPLES_PER_DISTANCE    10  // Number of measurements per distance
#define MEASUREMENT_DELAY_MS        500 // Delay between measurements (ms)
#define NUM_CALIBRATION_POINTS      6   // Number of test distances

// ============== Calibration Data Storage ==============
typedef struct {
    float expected_distance_cm;
    float measurements[NUM_SAMPLES_PER_DISTANCE];
    float min_reading;
    float max_reading;
    float average_reading;
    float std_deviation;
    float error_percent;
} calibration_point_t;

// ============== Helper Functions ==============

/**
 * Calculate statistics for a set of measurements
 */
void calculate_statistics(calibration_point_t *point) {
    float sum = 0.0f;
    point->min_reading = point->measurements[0];
    point->max_reading = point->measurements[0];
    
    // Find min, max, and sum
    for (int i = 0; i < NUM_SAMPLES_PER_DISTANCE; i++) {
        if (point->measurements[i] < point->min_reading) {
            point->min_reading = point->measurements[i];
        }
        if (point->measurements[i] > point->max_reading) {
            point->max_reading = point->measurements[i];
        }
        sum += point->measurements[i];
    }
    
    // Calculate average
    point->average_reading = sum / NUM_SAMPLES_PER_DISTANCE;
    
    // Calculate standard deviation
    float variance_sum = 0.0f;
    for (int i = 0; i < NUM_SAMPLES_PER_DISTANCE; i++) {
        float diff = point->measurements[i] - point->average_reading;
        variance_sum += diff * diff;
    }
    point->std_deviation = sqrtf(variance_sum / NUM_SAMPLES_PER_DISTANCE);
    
    // Calculate error percentage
    point->error_percent = fabsf((point->average_reading - point->expected_distance_cm) / point->expected_distance_cm) * 100.0f;
}

/**
 * Print calibration results in a formatted table
 */
void print_calibration_results(calibration_point_t *points, int num_points) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                    ULTRASONIC SENSOR CALIBRATION RESULTS                       ║\n");
    printf("╚════════════════════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    printf("┌─────────┬──────────┬──────────┬──────────┬──────────┬────────────┬─────────────┐\n");
    printf("│ Expected│  Average │    Min   │    Max   │   StdDev │   Range    │   Error     │\n");
    printf("│ Distance│ Reading  │ Reading  │ Reading  │          │            │   (%)       │\n");
    printf("│  (cm)   │   (cm)   │   (cm)   │   (cm)   │   (cm)   │   (cm)     │             │\n");
    printf("├─────────┼──────────┼──────────┼──────────┼──────────┼────────────┼─────────────┤\n");
    
    for (int i = 0; i < num_points; i++) {
        float range = points[i].max_reading - points[i].min_reading;
        printf("│ %7.1f │ %8.2f │ %8.2f │ %8.2f │ %8.2f │ %10.2f │ %10.2f%% │\n",
               points[i].expected_distance_cm,
               points[i].average_reading,
               points[i].min_reading,
               points[i].max_reading,
               points[i].std_deviation,
               range,
               points[i].error_percent);
    }
    
    printf("└─────────┴──────────┴──────────┴──────────┴──────────┴────────────┴─────────────┘\n");
    
    // Calculate overall calibration quality
    float total_error = 0.0f;
    float max_error = 0.0f;
    for (int i = 0; i < num_points; i++) {
        total_error += points[i].error_percent;
        if (points[i].error_percent > max_error) {
            max_error = points[i].error_percent;
        }
    }
    float avg_error = total_error / num_points;
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                        CALIBRATION QUALITY METRICS                            ║\n");
    printf("├────────────────────────────────────────────────────────────────────────────────┤\n");
    printf("║  Average Error Across All Points:        %6.2f%%                              ║\n", avg_error);
    printf("║  Maximum Error (Worst Point):            %6.2f%%                              ║\n", max_error);
    
    if (avg_error < 5.0f) {
        printf("║  Calibration Status:                     ✓ EXCELLENT                         ║\n");
    } else if (avg_error < 10.0f) {
        printf("║  Calibration Status:                     ✓ GOOD                              ║\n");
    } else if (avg_error < 15.0f) {
        printf("║  Calibration Status:                     ⚠ ACCEPTABLE                       ║\n");
    } else {
        printf("║  Calibration Status:                     ✗ NEEDS ADJUSTMENT                 ║\n");
    }
    printf("╚════════════════════════════════════════════════════════════════════════════════╝\n\n");
}

/**
 * Print detailed measurements for a single distance
 */
void print_distance_details(calibration_point_t *point) {
    printf("\n  Individual measurements for %.1f cm:\n", point->expected_distance_cm);
    printf("  ┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┐\n");
    printf("  │ Sample │   1    │   2    │   3    │   4    │   5    │   6    │   7    │   8    │   9    │   10   │\n");
    printf("  ├────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┼────────┤\n");
    printf("  │ (cm)   │");
    for (int i = 0; i < NUM_SAMPLES_PER_DISTANCE; i++) {
        printf(" %6.2f │", point->measurements[i]);
    }
    printf("\n");
    printf("  └────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┘\n");
}

// ============== Main Calibration Routine ==============
int main(void) {
    stdio_init_all();
    
    // Give USB time to initialize and for terminal to connect
    for (int i = 0; i < 5; i++) {
        printf(".");
        sleep_ms(500);
    }
    printf("\n\n");
    
    printf("\n╔════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                  ULTRASONIC SENSOR DISTANCE CALIBRATION TEST                  ║\n");
    printf("║                                                                                ║\n");
    printf("║  Instructions:                                                                 ║\n");
    printf("║  1. Place the car at a known distance from an obstacle                        ║\n");
    printf("║  2. The program will take multiple measurements at each distance              ║\n");
    printf("║  3. Move to the next distance when prompted                                   ║\n");
    printf("║  4. Recommended test distances: 10cm, 20cm, 30cm, 50cm, 100cm, 200cm          ║\n");
    printf("║                                                                                ║\n");
    printf("╚════════════════════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    // Initialize ultrasonic sensor
    printf("[INFO] Initializing ultrasonic sensor...\n");
    fflush(stdout);
    ultrasonic_init();
    printf("[INFO] Ultrasonic sensor initialized successfully\n");
    fflush(stdout);
    sleep_ms(1000);
    
    printf("Press ENTER to start calibration...\n");
    fflush(stdout);
    getchar();
    
    // Calibration points to test
    float test_distances[NUM_CALIBRATION_POINTS] = {10.0f, 20.0f, 30.0f, 50.0f, 100.0f, 200.0f};
    calibration_point_t calibration_data[NUM_CALIBRATION_POINTS];
    
    // Initialize calibration data
    for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
        calibration_data[i].expected_distance_cm = test_distances[i];
        calibration_data[i].min_reading = 0.0f;
        calibration_data[i].max_reading = 0.0f;
        calibration_data[i].average_reading = 0.0f;
        calibration_data[i].std_deviation = 0.0f;
        calibration_data[i].error_percent = 0.0f;
    }
    
    // Run calibration for each distance
    for (int dist_idx = 0; dist_idx < NUM_CALIBRATION_POINTS; dist_idx++) {
        float expected_dist = calibration_data[dist_idx].expected_distance_cm;
        
        printf("\n╔════════════════════════════════════════════════════════════════════════════════╗\n");
        printf("║  CALIBRATION POINT %d / %d: %.1f cm                                            ║\n", 
               dist_idx + 1, NUM_CALIBRATION_POINTS, expected_dist);
        printf("╚════════════════════════════════════════════════════════════════════════════════╝\n");
        printf("  Position the obstacle at exactly %.1f cm from the sensor\n", expected_dist);
        printf("  Press ENTER to begin measurements...\n");
        getchar();
        
        printf("  Taking %d measurements", NUM_SAMPLES_PER_DISTANCE);
        fflush(stdout);
        
        // Take measurements
        for (int sample = 0; sample < NUM_SAMPLES_PER_DISTANCE; sample++) {
            float distance = ultrasonic_get_distance_cm();
            calibration_data[dist_idx].measurements[sample] = distance;
            
            printf(".");
            fflush(stdout);
            
            sleep_ms(MEASUREMENT_DELAY_MS);
        }
        printf("\n  ✓ Measurements complete\n");
        
        // Calculate statistics
        calculate_statistics(&calibration_data[dist_idx]);
        
        // Print detailed results for this distance
        printf("\n  Results for %.1f cm:\n", expected_dist);
        printf("  ├─ Average:       %.2f cm\n", calibration_data[dist_idx].average_reading);
        printf("  ├─ Min:           %.2f cm\n", calibration_data[dist_idx].min_reading);
        printf("  ├─ Max:           %.2f cm\n", calibration_data[dist_idx].max_reading);
        printf("  ├─ Range:         %.2f cm (Max - Min)\n", 
               calibration_data[dist_idx].max_reading - calibration_data[dist_idx].min_reading);
        printf("  ├─ Std Deviation: %.2f cm\n", calibration_data[dist_idx].std_deviation);
        printf("  └─ Error:         %.2f%% (|avg - expected| / expected × 100)\n", 
               calibration_data[dist_idx].error_percent);
        
        // Show individual measurements
        print_distance_details(&calibration_data[dist_idx]);
    }
    
    // Print final calibration summary
    print_calibration_results(calibration_data, NUM_CALIBRATION_POINTS);
    
    // Analysis and recommendations
    printf("╔════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                       CALIBRATION ANALYSIS & RECOMMENDATIONS                  ║\n");
    printf("╚════════════════════════════════════════════════════════════════════════════════╝\n");
    printf("\n");
    
    float total_error = 0.0f;
    for (int i = 0; i < NUM_CALIBRATION_POINTS; i++) {
        total_error += calibration_data[i].error_percent;
    }
    float avg_error = total_error / NUM_CALIBRATION_POINTS;
    
    printf("Overall Calibration Accuracy: %.2f%%\n\n", 100.0f - avg_error);
    
    if (avg_error < 5.0f) {
        printf("✓ Sensor is well calibrated! No adjustment needed.\n");
    } else if (avg_error < 10.0f) {
        printf("✓ Sensor calibration is acceptable for most applications.\n");
    } else {
        printf("⚠ Sensor may need calibration adjustment.\n");
        printf("\nPossible causes:\n");
        printf("  1. Speed of sound constant may need adjustment (currently 0.0343 cm/µs)\n");
        printf("  2. GPIO timing sensitivity on Pico board\n");
        printf("  3. Environmental factors (temperature, humidity, air pressure)\n");
        printf("  4. Ultrasonic sensor hardware issue\n");
        printf("\nTo improve accuracy:\n");
        printf("  - Adjust the speed of sound constant in ultrasonic.c\n");
        printf("  - Verify the trigger and echo GPIO pins are correct\n");
        printf("  - Test at different temperatures and humidity levels\n");
    }
    
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║  Calibration test complete. Press ENTER to continue monitoring distance...    ║\n");
    printf("╚════════════════════════════════════════════════════════════════════════════════╝\n");
    
    getchar();
    
    // Continuous monitoring mode
    printf("\nEntering continuous monitoring mode (press Ctrl+C to exit)...\n");
    printf("═══════════════════════════════════════════════════════════════════════════════════\n");
    printf("┌────────────────┬────────────────┐\n");
    printf("│ Distance (cm)  │ Status         │\n");
    printf("├────────────────┼────────────────┤\n");
    
    while (1) {
        float distance = ultrasonic_get_distance_cm();
        
        const char *status;
        if (distance < 0) {
            status = "OUT_OF_RANGE";
        } else if (distance < 2.0f) {
            status = "TOO_CLOSE";
        } else if (distance <= 30.0f) {
            status = "OBSTACLE_FOUND";
        } else if (distance <= 50.0f) {
            status = "CLEAR_TO_PROCEED";
        } else {
            status = "FAR_AWAY";
        }
        
        printf("│ %14.2f │ %-14s │\n", distance, status);
        fflush(stdout);
        
        sleep_ms(500);
    }
    
    return 0;
}
