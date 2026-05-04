// ad4170.c — AD4170-4 SPI driver for Raspberry Pi 5
// Translated from known-working ESP32/Arduino code.
//
// Key fixes vs. original attempt:
//   1. SPI Mode 3 (CPOL=1, CPHA=1), not Mode 0
//   2. 14-bit addressing: 16-bit command word before each transfer
//   3. Correct reset: 3x (7x 0xFF + 0xFE) with CS held low throughout
//   4. setup() sequence matches ESP32 setup_ADC() exactly
//   5. Voltage conversion: raw * vref / 2^24 (unipolar, 5V ref)
//
// Build: gcc -O2 -Wall -o ad4170_demo ad4170.c ad4170_demo.c -lm
// Run:   sudo ./ad4170_demo

#include "ad4170.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

// ============================================================
//  Internal helper
// ============================================================

static int spi_xfer(ad4170_dev_t *dev,
                    const uint8_t *tx, uint8_t *rx, size_t len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)tx,
        .rx_buf        = (unsigned long)rx,
        .len           = (uint32_t)len,
        .speed_hz      = (uint32_t)dev->spi_speed_hz,
        .bits_per_word = dev->bits_per_word,
        .delay_usecs   = 0,
        .cs_change     = 0,
    };
    int ret = ioctl(dev->spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        perror("spi_xfer");
        return -1;
    }
    return 0;
}

// ============================================================
//  Init / close
// ============================================================

int ad4170_init(ad4170_dev_t *dev, const char *spidev_path, int speed_hz)
{
    dev->spi_speed_hz  = speed_hz;
    dev->spi_mode      = SPI_MODE_3;    // CPOL=1, CPHA=1
    dev->bits_per_word = 8;

    dev->spi_fd = open(spidev_path, O_RDWR);
    if (dev->spi_fd < 0) {
        fprintf(stderr, "ad4170_init: cannot open %s: %s\n",
                spidev_path, strerror(errno));
        return -1;
    }

    if (ioctl(dev->spi_fd, SPI_IOC_WR_MODE,          &dev->spi_mode)      < 0 ||
        ioctl(dev->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &dev->bits_per_word) < 0 ||
        ioctl(dev->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,  &dev->spi_speed_hz)  < 0)
    {
        perror("ad4170_init: ioctl config");
        close(dev->spi_fd);
        return -1;
    }
    return 0;
}

void ad4170_close(ad4170_dev_t *dev)
{
    if (dev->spi_fd >= 0) {
        close(dev->spi_fd);
        dev->spi_fd = -1;
    }
}

// ============================================================
//  Low-level register access
// ============================================================

// Write: [cmd_hi][cmd_lo][data bytes...]  — CS held for entire transaction
int ad4170_write(ad4170_dev_t *dev, uint16_t cmd,
                 const uint8_t *data, size_t data_len)
{
    size_t total = 2 + data_len;
    uint8_t tx[total], rx[total];
    memset(rx, 0, total);
    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] =  cmd       & 0xFF;
    for (size_t i = 0; i < data_len; i++)
        tx[2 + i] = data[i];
    return spi_xfer(dev, tx, rx, total);
}

// Read: [cmd_hi][cmd_lo][0x00 × data_len] → data bytes in rx[2..]
int ad4170_read(ad4170_dev_t *dev, uint16_t cmd,
                uint8_t *data, size_t data_len)
{
    size_t total = 2 + data_len;
    uint8_t tx[total], rx[total];
    memset(tx, 0x00, total);
    memset(rx, 0x00, total);
    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] =  cmd       & 0xFF;
    if (spi_xfer(dev, tx, rx, total) < 0) return -1;
    for (size_t i = 0; i < data_len; i++)
        data[i] = rx[2 + i];
    return 0;
}

int ad4170_write16(ad4170_dev_t *dev, uint16_t addr, uint16_t val)
{
    uint8_t d[2] = { (val >> 8) & 0xFF, val & 0xFF };
    return ad4170_write(dev, AD4170_CMD_WRITE(addr), d, 2);
}

int ad4170_write8(ad4170_dev_t *dev, uint16_t addr, uint8_t val)
{
    return ad4170_write(dev, AD4170_CMD_WRITE(addr), &val, 1);
}

int ad4170_read16(ad4170_dev_t *dev, uint16_t addr, uint16_t *out)
{
    uint8_t d[2] = {0};
    if (ad4170_read(dev, AD4170_CMD_READ(addr), d, 2) < 0) return -1;
    *out = ((uint16_t)d[0] << 8) | d[1];
    return 0;
}

int ad4170_read24(ad4170_dev_t *dev, uint16_t addr, uint32_t *out)
{
    uint8_t d[3] = {0};
    if (ad4170_read(dev, AD4170_CMD_READ(addr), d, 3) < 0) return -1;
    *out = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
    return 0;
}

int ad4170_read8(ad4170_dev_t *dev, uint16_t addr, uint8_t *out)
{
    uint8_t d[1] = {0};
    if (ad4170_read(dev, AD4170_CMD_READ(addr), d, 1) < 0) return -1;
    *out = d[0];
    return 0;
}

// ============================================================
//  High-level
// ============================================================

// Reset: CS low → 3x(7x 0xFF + 0xFE) → CS high
// This is exactly what setup_ADC() does on the ESP32
int ad4170_reset(ad4170_dev_t *dev)
{
    uint8_t tx[24], rx[24];
    memset(rx, 0, sizeof(rx));
    int idx = 0;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) tx[idx++] = 0xFF;
        tx[idx++] = 0xFE;
    }
    return spi_xfer(dev, tx, rx, sizeof(tx));
}

// Chip type register should read 0x07
int ad4170_check_chip_type(ad4170_dev_t *dev)
{
    uint8_t id = 0;
    if (ad4170_read8(dev, AD4170_ADDR_CHIP_TYPE, &id) < 0) return -1;
    printf("Chip type: 0x%02X (expected 0x07)\n", id);
    return (id == 0x07) ? 0 : -1;
}

// Mirrors setup_ADC() from the ESP32 code, step for step
int ad4170_setup(ad4170_dev_t *dev)
{
    int r;

    // Switch to 14-bit addressing
    r = ad4170_write8(dev, AD4170_ADDR_INTERFACE_CFG_B, AD4170_IFCFG_B_14BIT_ADDR);
    if (r < 0) return r;

    // Soft reset
    r = ad4170_write8(dev, AD4170_ADDR_INTERFACE_CFG_A, AD4170_IFCFG_A_RESET);
    if (r < 0) return r;

    // SPI: no CRC, no status append
    r = ad4170_write8(dev, AD4170_ADDR_SPI_CFG, AD4170_IFCFG_SPI_NO_CRC);
    if (r < 0) return r;

    // ADC control: continuous read mode
    r = ad4170_write16(dev, AD4170_ADDR_ADC_CTRL, AD4170_ADC_CTRL_CONT_READ);
    if (r < 0) return r;

    // Enable channels 0, 1, 2
    usleep(10000);
    r = ad4170_write16(dev, AD4170_ADDR_CH_EN, AD4170_CH_EN_0_1_2);
    if (r < 0) return r;

    // CH_MAP0: AIN0(+), AIN1(-)
    r = ad4170_write16(dev, AD4170_ADDR_CH0_MAP, AD4170_CH0_MAP_VAL);
    if (r < 0) return r;

    // CH_MAP1: AIN2(+), AIN3(-)
    usleep(10000);
    r = ad4170_write16(dev, AD4170_ADDR_CH1_MAP, AD4170_CH1_MAP_VAL);
    if (r < 0) return r;

    // CH_MAP2: AIN4(+), AIN5(-)
    r = ad4170_write16(dev, AD4170_ADDR_CH2_MAP, AD4170_CH2_MAP_VAL);
    if (r < 0) return r;

    // AFE 0, 1, 2
    usleep(10000);
    r = ad4170_write16(dev, AD4170_ADDR_AFE0, AD4170_AFE_VALUE); if (r < 0) return r;
    r = ad4170_write16(dev, AD4170_ADDR_AFE1, AD4170_AFE_VALUE); if (r < 0) return r;
    r = ad4170_write16(dev, AD4170_ADDR_AFE2, AD4170_AFE_VALUE); if (r < 0) return r;

    // VBIAS
    usleep(10000);
    r = ad4170_write16(dev, AD4170_ADDR_VBIAS, 0x0000);
    if (r < 0) return r;

    return 0;
}

int ad4170_read_channel(ad4170_dev_t *dev, int ch,
                        uint32_t *raw, float *voltage)
{
    static const uint16_t addrs[3] = {
        AD4170_ADDR_DATA_CH0,
        AD4170_ADDR_DATA_CH1,
        AD4170_ADDR_DATA_CH2,
    };
    if (ch < 0 || ch > 2) return -1;
    if (ad4170_read24(dev, addrs[ch], raw) < 0) return -1;
    if (voltage) *voltage = ad4170_to_voltage(*raw, 5.0f);
    return 0;
}

// Matches ESP32: voltage = raw * vref / 2^24
float ad4170_to_voltage(uint32_t raw, float vref)
{
    return ((float)raw * vref) / 16777216.0f;
}
