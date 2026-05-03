// ad4170.c — AD4170-4 SPI driver for Raspberry Pi 5
// Tested with Linux spidev on RPi5 (kernel 6.x)
//
// Build example:
//   gcc -O2 -Wall -o ad4170_demo ad4170.c ad4170_demo.c
//
// Enable SPI on RPi5:
//   sudo raspi-config  -> Interface Options -> SPI -> Enable
//   (or add  dtparam=spi=on  to /boot/firmware/config.txt)

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
//  Internal helpers
// ============================================================

static int spi_transfer(ad4170_dev_t *dev,
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
        perror("spi_transfer: ioctl SPI_IOC_MESSAGE");
        return -1;
    }
    return 0;
}

// ============================================================
//  Public API
// ============================================================

int ad4170_init(ad4170_dev_t *dev, const char *spidev_path, int speed_hz)
{
    dev->spi_speed_hz  = speed_hz;
    dev->spi_mode      = SPI_MODE_0;    // CPOL=0, CPHA=0
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
        perror("ad4170_init: ioctl configuration");
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

// Write  num_bytes  of  value  to register at  addr.
// The AD4170 SPI frame: [ADDR byte (R/W=0)] [DATA MSB ... LSB]
int ad4170_write_reg(ad4170_dev_t *dev, uint8_t addr,
                     uint32_t value, size_t num_bytes)
{
    if (num_bytes < 1 || num_bytes > 4) return -1;

    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};

    tx[0] = addr & 0x3F;                    // write: bit6 = 0
    for (size_t i = 0; i < num_bytes; i++) {
        tx[1 + i] = (value >> (8 * (num_bytes - 1 - i))) & 0xFF;
    }

    return spi_transfer(dev, tx, rx, 1 + num_bytes);
}

// Read  num_bytes  from register at  addr  into  *out.
int ad4170_read_reg(ad4170_dev_t *dev, uint8_t addr,
                    uint32_t *out, size_t num_bytes)
{
    if (num_bytes < 1 || num_bytes > 4) return -1;

    uint8_t tx[5] = {0};
    uint8_t rx[5] = {0};

    tx[0] = (addr & 0x3F) | AD4170_REG_READ;   // read: bit6 = 1

    if (spi_transfer(dev, tx, rx, 1 + num_bytes) < 0)
        return -1;

    *out = 0;
    for (size_t i = 0; i < num_bytes; i++) {
        *out = (*out << 8) | rx[1 + i];
    }
    return 0;
}

// Send 64 SCLKs with DIN high → hardware reset per datasheet §9.4.2
int ad4170_reset(ad4170_dev_t *dev)
{
    uint8_t tx[8];
    uint8_t rx[8];
    memset(tx, 0xFF, sizeof(tx));   // DIN = 1 for 64 clock cycles
    int ret = spi_transfer(dev, tx, rx, sizeof(tx));
    usleep(500);                    // Wait for reset to complete (~100 µs)
    return ret;
}

// Read Product ID and verify it matches expected value
int ad4170_check_id(ad4170_dev_t *dev)
{
    uint32_t id = 0;
    if (ad4170_read_reg(dev, AD4170_REG_ID, &id, 2) < 0)
        return -1;

    printf("AD4170 Product ID: 0x%04X (expected 0x%04X)\n",
           (unsigned)(id & 0xFFFF), AD4170_PRODUCT_ID);

    return ((id & 0xFFFF) == AD4170_PRODUCT_ID) ? 0 : -1;
}

// Configure device for single-ended or differential acquisition on channel 0.
//   ain_pos : positive input pin (0–3 for AIN0–AIN3)
//   ain_neg : negative input pin, or AD4170_AINM_AVSS for single-ended
int ad4170_configure_single_channel(ad4170_dev_t *dev,
                                    uint8_t ain_pos, uint8_t ain_neg)
{
    int ret;

    // 1. ADC control: continuous conversion, internal reference enabled
    uint16_t adc_ctrl = AD4170_ADC_CTRL_MODE_CONTINUOUS
                      | AD4170_ADC_CTRL_REF_EN
                      | AD4170_ADC_CTRL_MCLK_SEL_INT;
    ret = ad4170_write_reg(dev, AD4170_REG_ADC_CTRL, adc_ctrl, 2);
    if (ret < 0) return ret;

    // 2. Setup 0: PGA gain = 1, internal 2.5 V reference, no chopping
    uint16_t setup0 = AD4170_SETUP_CHOP_NONE
                    | AD4170_SETUP_REF_INT
                    | AD4170_SETUP_GAIN(0);  // gain = 1
    ret = ad4170_write_reg(dev, AD4170_REG_SETUP0, setup0, 2);
    if (ret < 0) return ret;

    // 3. Filter 0: Sinc5+Sinc1, ODR code 7 (~2.4 kSPS at 16 MHz MCLK)
    uint32_t filt0 = AD4170_FILT_SINC5_SINC1 | AD4170_FILT_ODR(7);
    ret = ad4170_write_reg(dev, AD4170_REG_FILT0, filt0, 3);
    if (ret < 0) return ret;

    // 4. Channel 0 map: assign setup 0, set AINp/AINm
    uint16_t ch0_map = AD4170_CH_MAP_SETUP(0)
                     | AD4170_CH_MAP_AINP(ain_pos)
                     | AD4170_CH_MAP_AINM(ain_neg);
    ret = ad4170_write_reg(dev, AD4170_REG_CH0_MAP, ch0_map, 2);
    if (ret < 0) return ret;

    // 5. Enable channel 0 only
    ret = ad4170_write_reg(dev, AD4170_REG_CH_EN, 0x0001, 2);
    if (ret < 0) return ret;

    return 0;
}

// Poll the STATUS register until a new result is ready, then read DATA.
// result is sign-extended to int32_t (24-bit two's complement).
int ad4170_read_data(ad4170_dev_t *dev, int32_t *result)
{
    const int max_polls = 10000;
    uint32_t status;

    // Wait for DATA_READY bit (bit 0 of STATUS = 0 means ready)
    for (int i = 0; i < max_polls; i++) {
        if (ad4170_read_reg(dev, AD4170_REG_STATUS, &status, 1) < 0)
            return -1;
        if ((status & 0x80) == 0)   // RDY bit (bit 7) goes low when data ready
            break;
        if (i == max_polls - 1) {
            fprintf(stderr, "ad4170_read_data: timeout waiting for RDY\n");
            return -1;
        }
        usleep(10);
    }

    uint32_t raw = 0;
    if (ad4170_read_reg(dev, AD4170_REG_DATA, &raw, 3) < 0)
        return -1;

    // Sign-extend 24-bit value to int32_t
    if (raw & 0x800000)
        *result = (int32_t)(raw | 0xFF000000);
    else
        *result = (int32_t)raw;

    return 0;
}

// Read  count  samples in continuous mode into  buf[].
int ad4170_read_continuous(ad4170_dev_t *dev, int32_t *buf, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (ad4170_read_data(dev, &buf[i]) < 0)
            return -1;
    }
    return 0;
}

// Convert a 24-bit ADC code to volts.
//   vref    : reference voltage in volts (2.5 for internal)
//   bipolar : 1 = bipolar (two's complement), 0 = unipolar
double ad4170_code_to_voltage(int32_t code, double vref, int bipolar)
{
    if (bipolar)
        return ((double)code / 8388608.0) * vref;   // ÷ 2^23
    else
        return ((double)(uint32_t)code / 16777216.0) * vref; // ÷ 2^24
}
