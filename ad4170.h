#ifndef AD4170_H
#define AD4170_H

#include <stdint.h>
#include <stddef.h>

// ============================================================
//  AD4170-4  SPI framing — 14-bit addressing mode
//
//  SPI Mode 3 (CPOL=1, CPHA=1) — data clocked on falling edge
//
//  16-bit command word sent before data:
//    Bit 15-14 : 01 = read, 00 = write  (in 14-bit addr mode)
//    Bits 13-0 : 14-bit register address
// ============================================================

#define AD4170_SPI_MODE             3   // CPOL=1, CPHA=1

// Build read/write command words matching ESP32 bit patterns
#define AD4170_CMD_WRITE(addr)      ((uint16_t)((addr) & 0x3FFF))
#define AD4170_CMD_READ(addr)       ((uint16_t)(0x4000 | ((addr) & 0x3FFF)))

// Register addresses (14-bit)
#define AD4170_ADDR_INTERFACE_CFG_A 0x0000
#define AD4170_ADDR_INTERFACE_CFG_B 0x0001
#define AD4170_ADDR_SPI_CFG         0x0010
#define AD4170_ADDR_CHIP_TYPE       0x0003
#define AD4170_ADDR_ADC_CTRL        0x0070
#define AD4170_ADDR_CH_EN           0x0079
#define AD4170_ADDR_CH0_MAP         0x0083
#define AD4170_ADDR_CH1_MAP         0x0087
#define AD4170_ADDR_CH2_MAP         0x008B
#define AD4170_ADDR_AFE0            0x00C3
#define AD4170_ADDR_AFE1            0x00D1
#define AD4170_ADDR_AFE2            0x00DF
#define AD4170_ADDR_OFFSET0         0x00CA
#define AD4170_ADDR_VBIAS           0x0135
#define AD4170_ADDR_DATA_CH0        0x002A
#define AD4170_ADDR_DATA_CH1        0x002E
#define AD4170_ADDR_DATA_CH2        0x0032

// Register values (from working ESP32 code)
#define AD4170_IFCFG_B_14BIT_ADDR   0x80
#define AD4170_IFCFG_A_RESET        0x91
#define AD4170_IFCFG_SPI_NO_CRC     0x37
#define AD4170_ADC_CTRL_CONT_READ   0x0010
#define AD4170_CH_EN_0_1_2          0x0007
#define AD4170_CH0_MAP_VAL          0x0001  // AIN0(+), AIN1(-)
#define AD4170_CH1_MAP_VAL          0x0203  // AIN2(+), AIN3(-)
#define AD4170_CH2_MAP_VAL          0x0405  // AIN4(+), AIN5(-)
#define AD4170_AFE_VALUE            0x0069

// ============================================================
//  Driver handle
// ============================================================

typedef struct {
    int     spi_fd;
    int     spi_speed_hz;
    uint8_t spi_mode;
    uint8_t bits_per_word;
} ad4170_dev_t;

// ============================================================
//  API
// ============================================================

int   ad4170_init(ad4170_dev_t *dev, const char *spidev_path, int speed_hz);
void  ad4170_close(ad4170_dev_t *dev);

int   ad4170_write(ad4170_dev_t *dev, uint16_t cmd, const uint8_t *data, size_t len);
int   ad4170_read (ad4170_dev_t *dev, uint16_t cmd, uint8_t *data, size_t len);
int   ad4170_write16(ad4170_dev_t *dev, uint16_t addr, uint16_t val);
int   ad4170_write8 (ad4170_dev_t *dev, uint16_t addr, uint8_t  val);
int   ad4170_read16 (ad4170_dev_t *dev, uint16_t addr, uint16_t *out);
int   ad4170_read24 (ad4170_dev_t *dev, uint16_t addr, uint32_t *out);
int   ad4170_read8  (ad4170_dev_t *dev, uint16_t addr, uint8_t  *out);

int   ad4170_reset(ad4170_dev_t *dev);
int   ad4170_setup(ad4170_dev_t *dev);
int   ad4170_check_chip_type(ad4170_dev_t *dev);
int   ad4170_read_channel(ad4170_dev_t *dev, int ch, uint32_t *raw, float *voltage);
float ad4170_to_voltage(uint32_t raw, float vref);

#endif /* AD4170_H */
