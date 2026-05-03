// ad4170_demo.c — Example acquisition loop for AD4170-4 on Raspberry Pi 5
//
// Wiring (RPi5 SPI0):
//   RPi5 GPIO 11 (SCLK) -> AD4170 SCLK
//   RPi5 GPIO 10 (MOSI) -> AD4170 DIN
//   RPi5 GPIO  9 (MISO) -> AD4170 DOUT/RDY
//   RPi5 GPIO  8 (CE0)  -> AD4170 /CS
//   3.3 V               -> IOVDD (logic supply)
//   AVDD/AVSS           -> analog supply per your schematic
//
// Build:
//   gcc -O2 -Wall -o ad4170_demo ad4170.c ad4170_demo.c -lm
//
// Run:
//   sudo ./ad4170_demo

#include "ad4170.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SPIDEV_PATH   "/dev/spidev0.0"  // SPI0, CS0 on RPi5
#define SPI_SPEED_HZ  1000000           // 1 MHz — safe for bring-up
#define VREF          2.5               // Internal reference voltage
#define NUM_SAMPLES   100               // Samples to collect

int main(void)
{
    ad4170_dev_t dev;

    // ----------------------------------------------------------
    // 1. Open SPI
    // ----------------------------------------------------------
    if (ad4170_init(&dev, SPIDEV_PATH, SPI_SPEED_HZ) < 0) {
        fprintf(stderr, "Failed to open SPI device.\n");
        return EXIT_FAILURE;
    }
    printf("SPI opened: %s @ %d Hz\n", SPIDEV_PATH, SPI_SPEED_HZ);

    // ----------------------------------------------------------
    // 2. Hardware reset (64 SCLK pulses with DIN high)
    // ----------------------------------------------------------
    if (ad4170_reset(&dev) < 0) {
        fprintf(stderr, "Reset failed.\n");
        goto fail;
    }
    printf("Device reset OK\n");

    // ----------------------------------------------------------
    // 3. Verify Product ID
    // ----------------------------------------------------------
    if (ad4170_check_id(&dev) < 0) {
        fprintf(stderr, "Product ID mismatch — check wiring and SPI config.\n");
        goto fail;
    }

    // ----------------------------------------------------------
    // 4. Configure: single-ended AIN0 vs AVSS, internal 2.5V ref
    // ----------------------------------------------------------
    if (ad4170_configure_single_channel(&dev,
                                        0,                    // AIN0 positive
                                        AD4170_AINM_AVSS)     // AVSS negative
        < 0)
    {
        fprintf(stderr, "Configuration failed.\n");
        goto fail;
    }
    printf("ADC configured: AIN0 single-ended, Sinc5+1 filter\n");

    // ----------------------------------------------------------
    // 5. Acquire NUM_SAMPLES and print
    // ----------------------------------------------------------
    printf("\nSample , Raw Code  , Voltage (V)\n");
    printf("--------+-----------+------------\n");

    int32_t buf[NUM_SAMPLES];
    if (ad4170_read_continuous(&dev, buf, NUM_SAMPLES) < 0) {
        fprintf(stderr, "Data acquisition failed.\n");
        goto fail;
    }

    double sum = 0.0, sum_sq = 0.0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        double v = ad4170_code_to_voltage(buf[i], VREF, /*bipolar=*/1);
        sum    += v;
        sum_sq += v * v;
        printf("%6d  , %9d , %+.6f\n", i, buf[i], v);
    }

    double mean   = sum / NUM_SAMPLES;
    double var    = (sum_sq / NUM_SAMPLES) - (mean * mean);
    double stddev = sqrt(var < 0.0 ? 0.0 : var);

    printf("\n--- Statistics over %d samples ---\n", NUM_SAMPLES);
    printf("  Mean    : %+.6f V\n", mean);
    printf("  Std dev : %.2f µV (%.2f LSB)\n",
           stddev * 1e6,
           stddev / (VREF / pow(2.0, 23)));    // noise in LSBs (23-bit bipolar)

    ad4170_close(&dev);
    return EXIT_SUCCESS;

fail:
    ad4170_close(&dev);
    return EXIT_FAILURE;
}
