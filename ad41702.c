// ad4170.c — AD4170-4 SPI driver + acquisition demo for Raspberry Pi 5
// Translated from working ESP32 Arduino code.
//
// Build:   gcc -O2 -Wall -o ad4170 ad4170.c -lm
// Run:     sudo ./ad4170
//
// SPI1 pin mapping on RPi5 40-pin header:
//   Function  GPIO    Pin
//   --------  ------  ---
//   SCLK      GPIO21  40
//   MOSI      GPIO20  38
//   MISO      GPIO19  35
//   CS0       GPIO18  12
//
// /boot/firmware/config.txt must contain:
//   dtparam=spi=on
//   dtoverlay=spi1-1cs
 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
 
// ============================================================
//  Config
// ============================================================
#define SPIDEV_PATH   "/dev/spidev1.0"
#define SPI_SPEED_HZ  200000        // 200 kHz, matches ESP32 code
 
// ============================================================
//  AD4170 command words (16-bit, MSB first)
//  Write: bit15:14 = 00
//  Read:  bit15:14 = 01  (bit14 set)
// ============================================================
 
// Write commands
#define CMD_W_COMMS         0x0001
#define CMD_W_ADC_CTRL      0x0000
#define CMD_W_SPI_CTRL      0x0010
#define CMD_W_CONT_READ     0x0071
#define CMD_W_CH_ENABLE     0x0079
#define CMD_W_CH0_MAP       0x0083
#define CMD_W_CH1_MAP       0x0087
#define CMD_W_CH2_MAP       0x008B
#define CMD_W_AFE0          0x00C3
#define CMD_W_AFE1          0x00D1
#define CMD_W_AFE2          0x00DF
#define CMD_W_VBIAS         0x0135
 
// Read commands
#define CMD_R_ID            0x4003
#define CMD_R_CH_ENABLE     0x4079
#define CMD_R_CH0_MAP       0x4083
#define CMD_R_CH1_MAP       0x4087
#define CMD_R_CH2_MAP       0x408B
#define CMD_R_AFE0          0x40C3
#define CMD_R_AFE1          0x40D1
#define CMD_R_AFE2          0x40DF
#define CMD_R_OFFSET0       0x40CA
#define CMD_R_DATA_CH0      0x402A
#define CMD_R_DATA_CH1      0x402E
#define CMD_R_DATA_CH2      0x4032
 
// Register values
#define VAL_COMMS_14BIT     0x80
#define VAL_ADC_SOFT_RESET  0x91
#define VAL_SPI_NO_CRC      0x37
#define VAL_CONT_READ       0x0010
#define VAL_CH_EN_0_1_2     0x0007
#define VAL_CH0_MAP         0x0001  // AIN0+ / AIN1-
#define VAL_CH1_MAP         0x0203  // AIN2+ / AIN3-
#define VAL_CH2_MAP         0x0405  // AIN4+ / AIN5-
#define VAL_AFE_CFG         0x0069
#define VAL_VBIAS_OFF       0x0000
 
#define AD4170_VREF         5.0
#define AD4170_FULL_SCALE   16777216.0  // 2^24
 
// ============================================================
//  SPI state
// ============================================================
static int spi_fd = -1;
 
static int spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    struct spi_ioc_transfer tr = {
        .tx_buf        = (unsigned long)tx,
        .rx_buf        = (unsigned long)rx,
        .len           = (uint32_t)len,
        .speed_hz      = SPI_SPEED_HZ,
        .bits_per_word = 8,
        .delay_usecs   = 0,
        .cs_change     = 0,
    };
    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }
    return 0;
}
 
// ============================================================
//  Transfer helpers
// ============================================================
static int write8(uint16_t cmd, uint8_t data)
{
    uint8_t tx[3] = { cmd >> 8, cmd & 0xFF, data };
    uint8_t rx[3];
    return spi_xfer(tx, rx, 3);
}
 
static int write16(uint16_t cmd, uint16_t data)
{
    uint8_t tx[4] = { cmd >> 8, cmd & 0xFF, data >> 8, data & 0xFF };
    uint8_t rx[4];
    return spi_xfer(tx, rx, 4);
}
 
static int read8(uint16_t cmd, uint8_t *out)
{
    uint8_t tx[3] = { cmd >> 8, cmd & 0xFF, 0x00 };
    uint8_t rx[3] = {0};
    if (spi_xfer(tx, rx, 3) < 0) return -1;
    *out = rx[2];
    return 0;
}
 
static int read16(uint16_t cmd, uint16_t *out)
{
    uint8_t tx[4] = { cmd >> 8, cmd & 0xFF, 0x00, 0x00 };
    uint8_t rx[4] = {0};
    if (spi_xfer(tx, rx, 4) < 0) return -1;
    *out = ((uint16_t)rx[2] << 8) | rx[3];
    return 0;
}
 
static int read24(uint16_t cmd, uint32_t *out)
{
    uint8_t tx[5] = { cmd >> 8, cmd & 0xFF, 0x00, 0x00, 0x00 };
    uint8_t rx[5] = {0};
    if (spi_xfer(tx, rx, 5) < 0) return -1;
    *out = ((uint32_t)rx[2] << 16) | ((uint32_t)rx[3] << 8) | rx[4];
    return 0;
}
 
// ============================================================
//  Reset: 3 × (7 × 0xFF + 0xFE), single CS assertion
// ============================================================
static int adc_reset(void)
{
    uint8_t tx[24], rx[24];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) tx[i*8 + j] = 0xFF;
        tx[i*8 + 7] = 0xFE;
    }
    if (spi_xfer(tx, rx, 24) < 0) return -1;
    usleep(500);
    return 0;
}
 
// ============================================================
//  Setup — mirrors ESP32 setup_ADC() exactly
// ============================================================
static int adc_setup(void)
{
    if (write8 (CMD_W_COMMS,     VAL_COMMS_14BIT)    < 0) return -1;
    if (write8 (CMD_W_ADC_CTRL,  VAL_ADC_SOFT_RESET)  < 0) return -1;
    if (write8 (CMD_W_SPI_CTRL,  VAL_SPI_NO_CRC)      < 0) return -1;
    if (write16(CMD_W_CONT_READ, VAL_CONT_READ)        < 0) return -1;
    if (write16(CMD_W_CH_ENABLE, VAL_CH_EN_0_1_2)     < 0) return -1;
    if (write16(CMD_W_CH0_MAP,   VAL_CH0_MAP)          < 0) return -1;
    usleep(10000);
    if (write16(CMD_W_CH1_MAP,   VAL_CH1_MAP)          < 0) return -1;
    usleep(10000);
    if (write16(CMD_W_CH2_MAP,   VAL_CH2_MAP)          < 0) return -1;
    usleep(10000);
    if (write16(CMD_W_AFE0,      VAL_AFE_CFG)          < 0) return -1;
    usleep(10000);
    if (write16(CMD_W_AFE1,      VAL_AFE_CFG)          < 0) return -1;
    if (write16(CMD_W_AFE2,      VAL_AFE_CFG)          < 0) return -1;
    if (write16(CMD_W_VBIAS,     VAL_VBIAS_OFF)        < 0) return -1;
    return 0;
}
 
// ============================================================
//  Register readback — mirrors ESP32 setup() print block
// ============================================================
static void print_config(void)
{
    uint8_t  id   = 0;
    uint16_t v16  = 0;
    uint32_t v24  = 0;
 
    read8 (CMD_R_ID,        &id);  printf("ID        : 0x%02X (expect 0x07)\n", id);
    read16(CMD_R_CH_ENABLE, &v16); printf("CH_ENABLE : 0x%04X\n", v16);
    read16(CMD_R_CH0_MAP,   &v16); printf("CH0_MAP   : 0x%04X\n", v16);
    read16(CMD_R_CH1_MAP,   &v16); printf("CH1_MAP   : 0x%04X\n", v16);
    read16(CMD_R_CH2_MAP,   &v16); printf("CH2_MAP   : 0x%04X\n", v16);
    read16(CMD_R_AFE0,      &v16); printf("AFE0      : 0x%04X\n", v16);
    read16(CMD_R_AFE1,      &v16); printf("AFE1      : 0x%04X\n", v16);
    read16(CMD_R_AFE2,      &v16); printf("AFE2      : 0x%04X\n", v16);
    read24(CMD_R_OFFSET0,   &v24); printf("OFFSET0   : 0x%06X\n", v24);
}
 
// ============================================================
//  Main
// ============================================================
static volatile int running = 1;
static void on_sigint(int s) { (void)s; running = 0; }
 
int main(void)
{
    signal(SIGINT, on_sigint);
 
    // Open SPI
    spi_fd = open(SPIDEV_PATH, O_RDWR);
    if (spi_fd < 0) {
        fprintf(stderr, "Cannot open %s: %s\n", SPIDEV_PATH, strerror(errno));
        return EXIT_FAILURE;
    }
 
    uint8_t mode = SPI_MODE_3;  // CPOL=1, CPHA=1
    uint8_t bits = 8;
    uint32_t speed = SPI_SPEED_HZ;
    ioctl(spi_fd, SPI_IOC_WR_MODE,          &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,  &speed);
 
    printf("SPI opened: %s @ %d Hz, Mode 3\n\n", SPIDEV_PATH, SPI_SPEED_HZ);
 
    // Reset + setup
    if (adc_reset() < 0)  { fprintf(stderr, "Reset failed\n");  goto done; }
    printf("Reset OK\n");
    if (adc_setup() < 0)  { fprintf(stderr, "Setup failed\n");  goto done; }
    printf("Setup OK\n\n");
 
    // Readback
    print_config();
    printf("\nStarting acquisition — Ctrl-C to stop\n\n");
    printf("%-8s  %-14s  %-14s  %-14s\n", "ID", "Voltage X", "Voltage Y", "Voltage Z");
    printf("--------  --------------  --------------  --------------\n");
 
    // Acquisition loop — mirrors ESP32 loop()
    while (running) {
        uint8_t  id = 0;
        uint32_t raw0 = 0, raw1 = 0, raw2 = 0;
 
        read8 (CMD_R_ID,       &id);
        read24(CMD_R_DATA_CH0, &raw0);
        read24(CMD_R_DATA_CH1, &raw1);
        read24(CMD_R_DATA_CH2, &raw2);
 
        float v0 = (raw0 * AD4170_VREF) / AD4170_FULL_SCALE;
        float v1 = (raw1 * AD4170_VREF) / AD4170_FULL_SCALE;
        float v2 = (raw2 * AD4170_VREF) / AD4170_FULL_SCALE;
 
        printf("0x%02X      %+.6f V    %+.6f V    %+.6f V\n", id, v0, v1, v2);
 
        usleep(10000);  // ~100 Hz
    }
 
done:
    printf("\nDone.\n");
    close(spi_fd);
    return EXIT_SUCCESS;
}
 
