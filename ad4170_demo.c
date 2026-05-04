// ad4170_demo.c — matches the ESP32 loop() behavior
//
// Wiring for SPI1 on RPi5 header:
//   GPIO21 / Pin 40  → SCLK
//   GPIO20 / Pin 38  → MOSI (DIN)
//   GPIO19 / Pin 35  → MISO (DOUT/RDY)
//   GPIO18 / Pin 12  → CS0
//
// (If you wired to SPI0 pins instead, change SPIDEV_PATH to /dev/spidev0.0)
//
// Build: gcc -O2 -Wall -o ad4170_demo ad4170.c ad4170_demo.c -lm
// Run:   sudo ./ad4170_demo

#include "ad4170.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define SPIDEV_PATH  "/dev/spidev1.0"
#define SPI_SPEED_HZ  200000            // 200 kHz — matches ESP32 SPISettings(200000,...)

int main(void)
{
    ad4170_dev_t dev;

    // ---- Open SPI ----
    if (ad4170_init(&dev, SPIDEV_PATH, SPI_SPEED_HZ) < 0)
        return EXIT_FAILURE;
    printf("SPI opened: %s @ %d Hz\n", SPIDEV_PATH, SPI_SPEED_HZ);

    // ---- Reset (hardware pattern, CS held low) ----
    if (ad4170_reset(&dev) < 0) {
        fprintf(stderr, "Reset failed\n");
        goto fail;
    }
    printf("Reset OK\n");

    // ---- Configure (mirrors setup_ADC) ----
    if (ad4170_setup(&dev) < 0) {
        fprintf(stderr, "Setup failed\n");
        goto fail;
    }
    printf("Setup OK\n");

    // ---- Read-back registers to verify (mirrors ESP32 setup() readbacks) ----
    {
        uint16_t ch_en, ch0, ch1, ch2, afe0, afe1, afe2;
        uint32_t off0;

        ad4170_read16(&dev, AD4170_ADDR_CH_EN,   &ch_en);
        ad4170_read16(&dev, AD4170_ADDR_CH0_MAP, &ch0);
        ad4170_read16(&dev, AD4170_ADDR_CH1_MAP, &ch1);
        ad4170_read16(&dev, AD4170_ADDR_CH2_MAP, &ch2);
        ad4170_read16(&dev, AD4170_ADDR_AFE0,    &afe0);
        ad4170_read16(&dev, AD4170_ADDR_AFE1,    &afe1);
        ad4170_read16(&dev, AD4170_ADDR_AFE2,    &afe2);
        ad4170_read24(&dev, AD4170_ADDR_OFFSET0, &off0);

        printf("\n--- Register Readback ---\n");
        printf("CH_ENABLE : 0x%04X  (expect 0x0007)\n", ch_en);
        printf("CH0 map   : 0x%04X  (expect 0x0001)\n", ch0);
        printf("CH1 map   : 0x%04X  (expect 0x0203)\n", ch1);
        printf("CH2 map   : 0x%04X  (expect 0x0405)\n", ch2);
        printf("AFE0      : 0x%04X  (expect 0x0069)\n", afe0);
        printf("AFE1      : 0x%04X  (expect 0x0069)\n", afe1);
        printf("AFE2      : 0x%04X  (expect 0x0069)\n", afe2);
        printf("OFFSET0   : 0x%06X\n", off0);
        printf("-------------------------\n\n");
    }

    // ---- Check chip type ----
    if (ad4170_check_chip_type(&dev) < 0)
        fprintf(stderr, "Warning: chip type mismatch\n");

    // ---- Continuous read loop (mirrors ESP32 loop()) ----
    printf("Starting acquisition... Ctrl-C to stop\n\n");
    for (;;) {
        uint8_t  chip_id;
        uint32_t raw0, raw1, raw2;
        float    v0, v1, v2;

        ad4170_read8(&dev, AD4170_ADDR_CHIP_TYPE, &chip_id);

        ad4170_read_channel(&dev, 0, &raw0, &v0);
        ad4170_read_channel(&dev, 1, &raw1, &v1);
        ad4170_read_channel(&dev, 2, &raw2, &v2);

        printf("ID: 0x%02X  |  "
               "CH0: %8u  %.6f V  |  "
               "CH1: %8u  %.6f V  |  "
               "CH2: %8u  %.6f V\n",
               chip_id,
               raw0, v0,
               raw1, v1,
               raw2, v2);

        usleep(50000);  // ~20 Hz print rate
    }

    ad4170_close(&dev);
    return EXIT_SUCCESS;

fail:
    ad4170_close(&dev);
    return EXIT_FAILURE;
}
