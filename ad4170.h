#ifndef AD4170_H
#define AD4170_H
 
#include <stdint.h>
#include <stddef.h>
 
// ============================================================
//  AD4170-4  --  Register Map (partial, most-used registers)
// ============================================================
 
// SPI framing
#define AD4170_REG_READ         0x40    // OR into address byte for reads
#define AD4170_REG_WRITE        0x00    // (default, no OR needed)
 
// Register addresses
#define AD4170_REG_COMMS        0x00    // Communications / address register
#define AD4170_REG_STATUS       0x00    // Status (read)
#define AD4170_REG_ADC_CTRL     0x01    // ADC control
#define AD4170_REG_DATA         0x04    // ADC data output (24-bit)
#define AD4170_REG_IOC          0x07    // I/O control
#define AD4170_REG_VBIAS        0x08    // VBIAS control
#define AD4170_REG_ID           0x09    // Product ID
#define AD4170_REG_ERROR        0x0A    // Error register
#define AD4170_REG_ERROR_EN     0x0B    // Error enable register
#define AD4170_REG_CH_EN        0x10    // Channel enable (16-bit)
#define AD4170_REG_CH0_MAP      0x11    // Channel 0 input mapping
#define AD4170_REG_CH1_MAP      0x15    // Channel 1 input mapping
#define AD4170_REG_CH2_MAP      0x19    // Channel 2 input mapping
#define AD4170_REG_CH3_MAP      0x1D    // Channel 3 input mapping
#define AD4170_REG_SETUP0       0x20    // Setup 0 (filter, gain, ref select)
#define AD4170_REG_SETUP1       0x28    // Setup 1
#define AD4170_REG_FILT0        0x21    // Filter config setup 0
#define AD4170_REG_FILT1        0x29    // Filter config setup 1
#define AD4170_REG_OFFSET0      0x26    // Offset calibration setup 0
#define AD4170_REG_GAIN0        0x27    // Gain calibration setup 0
 
// ADC_CTRL register fields (16-bit)
#define AD4170_ADC_CTRL_MODE_CONTINUOUS     (0x0 << 0)
#define AD4170_ADC_CTRL_MODE_SINGLE         (0x4 << 0)
#define AD4170_ADC_CTRL_MODE_STANDBY        (0x2 << 0)
#define AD4170_ADC_CTRL_MODE_POWERDOWN      (0x3 << 0)
#define AD4170_ADC_CTRL_SING_CYC            (1 << 4)    // Single cycle mode
#define AD4170_ADC_CTRL_DOUT_RDY_DEL        (1 << 5)    // DOUT/RDY delay
#define AD4170_ADC_CTRL_CSB_EN              (1 << 9)    // CS decode enable
#define AD4170_ADC_CTRL_REF_EN              (1 << 8)    // Internal ref enable
#define AD4170_ADC_CTRL_MCLK_SEL_INT        (0x0 << 10) // Internal 16 MHz clock
 
// Channel map fields
#define AD4170_CH_MAP_SETUP(s)      ((s) << 12)
#define AD4170_CH_MAP_AINP(p)       ((p) << 5)
#define AD4170_CH_MAP_AINM(m)       ((m) << 0)
#define AD4170_AINM_AVSS            0x10    // AVSS as negative input
 
// Setup register fields (16-bit)
#define AD4170_SETUP_CHOP_NONE      (0x0 << 8)
#define AD4170_SETUP_CHOP_IEXCEX    (0x2 << 8)
#define AD4170_SETUP_REF_INT        (0x0 << 4)  // Internal 2.5 V reference
#define AD4170_SETUP_REF_EXT        (0x1 << 4)  // External REFIN1
#define AD4170_SETUP_REF_AVDD       (0x2 << 4)  // AVDD reference
#define AD4170_SETUP_GAIN(g)        ((g) << 0)  // PGA gain 0=1, 1=2 ... 6=128
 
// Filter config fields (32-bit)
#define AD4170_FILT_SINC5_SINC1     (0x0 << 21)
#define AD4170_FILT_SINC3           (0x6 << 21)
#define AD4170_FILT_ODR(r)          ((r) << 0)  // Output data rate code
 
// Product ID expected value
#define AD4170_PRODUCT_ID           0x3430
 
// ============================================================
//  Driver handle
// ============================================================
 
typedef struct {
    int     spi_fd;         // file descriptor from open("/dev/spidevX.Y")
    int     spi_speed_hz;   // SPI clock (recommend 1-5 MHz for bring-up)
    uint8_t spi_mode;       // SPI_MODE_0
    uint8_t bits_per_word;  // 8
} ad4170_dev_t;
 
// ============================================================
//  Function declarations
// ============================================================
 
int  ad4170_init(ad4170_dev_t *dev, const char *spidev_path, int speed_hz);
void ad4170_close(ad4170_dev_t *dev);
 
int  ad4170_write_reg(ad4170_dev_t *dev, uint8_t addr, uint32_t value, size_t num_bytes);
int  ad4170_read_reg (ad4170_dev_t *dev, uint8_t addr, uint32_t *out, size_t num_bytes);
 
int  ad4170_check_id(ad4170_dev_t *dev);
int  ad4170_reset(ad4170_dev_t *dev);
int  ad4170_configure_single_channel(ad4170_dev_t *dev, uint8_t ain_pos, uint8_t ain_neg);
int  ad4170_read_data(ad4170_dev_t *dev, int32_t *result);
int  ad4170_read_continuous(ad4170_dev_t *dev, int32_t *buf, size_t count);
 
double ad4170_code_to_voltage(int32_t code, double vref, int bipolar);
 
#endif /* AD4170_H */
