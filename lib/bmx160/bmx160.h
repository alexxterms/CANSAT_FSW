#ifndef BMX160_H
#define BMX160_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

// BMX160 I2C Address
#define BMX160_I2C_ADDR                 0x68    // Primary address
#define BMX160_I2C_ADDR_ALT             0x69    // Alternative address

// BMX160 Chip ID
#define BMX160_CHIP_ID                  0xD8

// BMX160 Register Addresses
#define BMX160_REG_CHIP_ID              0x00
#define BMX160_REG_ERR_REG              0x02
#define BMX160_REG_PMU_STATUS           0x03
#define BMX160_REG_DATA_0               0x04    // Start of data registers
#define BMX160_REG_DATA_1               0x05
#define BMX160_REG_DATA_2               0x06
#define BMX160_REG_DATA_3               0x07
#define BMX160_REG_DATA_4               0x08
#define BMX160_REG_DATA_5               0x09
#define BMX160_REG_DATA_6               0x0A
#define BMX160_REG_DATA_7               0x0B
#define BMX160_REG_DATA_8               0x0C
#define BMX160_REG_DATA_9               0x0D
#define BMX160_REG_DATA_10              0x0E
#define BMX160_REG_DATA_11              0x0F
#define BMX160_REG_DATA_12              0x10
#define BMX160_REG_DATA_13              0x11
#define BMX160_REG_DATA_14              0x12
#define BMX160_REG_DATA_15              0x13
#define BMX160_REG_DATA_16              0x14
#define BMX160_REG_DATA_17              0x15
#define BMX160_REG_DATA_18              0x16
#define BMX160_REG_DATA_19              0x17
#define BMX160_REG_SENSORTIME_0         0x18
#define BMX160_REG_SENSORTIME_1         0x19
#define BMX160_REG_SENSORTIME_2         0x1A
#define BMX160_REG_STATUS               0x1B
#define BMX160_REG_INT_STATUS_0         0x1C
#define BMX160_REG_INT_STATUS_1         0x1D
#define BMX160_REG_INT_STATUS_2         0x1E
#define BMX160_REG_INT_STATUS_3         0x1F
#define BMX160_REG_TEMPERATURE_0        0x20
#define BMX160_REG_TEMPERATURE_1        0x21
#define BMX160_REG_FIFO_LENGTH_0        0x22
#define BMX160_REG_FIFO_LENGTH_1        0x23
#define BMX160_REG_FIFO_DATA            0x24
#define BMX160_REG_ACC_CONF             0x40
#define BMX160_REG_ACC_RANGE            0x41
#define BMX160_REG_GYR_CONF             0x42
#define BMX160_REG_GYR_RANGE            0x43
#define BMX160_REG_MAG_CONF             0x44
#define BMX160_REG_FIFO_DOWNS           0x45
#define BMX160_REG_FIFO_CONFIG_0        0x46
#define BMX160_REG_FIFO_CONFIG_1        0x47
#define BMX160_REG_MAG_IF_0             0x4B
#define BMX160_REG_MAG_IF_1             0x4C
#define BMX160_REG_MAG_IF_2             0x4D
#define BMX160_REG_MAG_IF_3             0x4E
#define BMX160_REG_MAG_IF_4             0x4F
#define BMX160_REG_INT_EN_0             0x50
#define BMX160_REG_INT_EN_1             0x51
#define BMX160_REG_INT_EN_2             0x52
#define BMX160_REG_INT_OUT_CTRL         0x53
#define BMX160_REG_INT_LATCH            0x54
#define BMX160_REG_INT_MAP_0            0x55
#define BMX160_REG_INT_MAP_1            0x56
#define BMX160_REG_INT_MAP_2            0x57
#define BMX160_REG_INT_DATA_0           0x58
#define BMX160_REG_INT_DATA_1           0x59
#define BMX160_REG_INT_LOWHIGH_0        0x5A
#define BMX160_REG_INT_LOWHIGH_1        0x5B
#define BMX160_REG_INT_LOWHIGH_2        0x5C
#define BMX160_REG_INT_LOWHIGH_3        0x5D
#define BMX160_REG_INT_LOWHIGH_4        0x5E
#define BMX160_REG_INT_MOTION_0         0x5F
#define BMX160_REG_INT_MOTION_1         0x60
#define BMX160_REG_INT_MOTION_2         0x61
#define BMX160_REG_INT_MOTION_3         0x62
#define BMX160_REG_INT_TAP_0            0x63
#define BMX160_REG_INT_TAP_1            0x64
#define BMX160_REG_INT_ORIENT_0         0x65
#define BMX160_REG_INT_ORIENT_1         0x66
#define BMX160_REG_INT_FLAT_0           0x67
#define BMX160_REG_INT_FLAT_1           0x68
#define BMX160_REG_FOC_CONF             0x69
#define BMX160_REG_CONF                 0x6A
#define BMX160_REG_IF_CONF              0x6B
#define BMX160_REG_PMU_TRIGGER          0x6C
#define BMX160_REG_SELF_TEST            0x6D
#define BMX160_REG_NV_CONF              0x70
#define BMX160_REG_OFFSET_0             0x71
#define BMX160_REG_OFFSET_1             0x72
#define BMX160_REG_OFFSET_2             0x73
#define BMX160_REG_OFFSET_3             0x74
#define BMX160_REG_OFFSET_4             0x75
#define BMX160_REG_OFFSET_5             0x76
#define BMX160_REG_OFFSET_6             0x77
#define BMX160_REG_STEP_CNT_0           0x78
#define BMX160_REG_STEP_CNT_1           0x79
#define BMX160_REG_STEP_CONF_0          0x7A
#define BMX160_REG_STEP_CONF_1          0x7B
#define BMX160_REG_CMD                  0x7E

// BMX160 Commands
#define BMX160_CMD_START_FOC            0x03
#define BMX160_CMD_ACC_SET_PMU_MODE     0x10    // + mode (0=suspend, 1=normal, 2=lowpower)
#define BMX160_CMD_GYR_SET_PMU_MODE     0x14    // + mode (0=suspend, 1=normal, 2=fast_startup, 3=lowpower)
#define BMX160_CMD_MAG_SET_PMU_MODE     0x18    // + mode (0=suspend, 1=normal, 2=lowpower)
#define BMX160_CMD_PROG_NVM             0xA0
#define BMX160_CMD_FIFO_FLUSH           0xB0
#define BMX160_CMD_INT_RESET            0xB1
#define BMX160_CMD_SOFTRESET            0xB6
#define BMX160_CMD_STEP_CNT_CLR         0xB2

// Power modes
#define BMX160_PMU_SUSPEND              0x00
#define BMX160_PMU_NORMAL               0x01
#define BMX160_PMU_LOWPOWER             0x02
#define BMX160_PMU_FASTSTARTUP          0x03

// Accelerometer ranges (±g)
#define BMX160_ACC_RANGE_2G             0x03
#define BMX160_ACC_RANGE_4G             0x05
#define BMX160_ACC_RANGE_8G             0x08
#define BMX160_ACC_RANGE_16G            0x0C

// Gyroscope ranges (±dps)
#define BMX160_GYR_RANGE_2000DPS        0x00
#define BMX160_GYR_RANGE_1000DPS        0x01
#define BMX160_GYR_RANGE_500DPS         0x02
#define BMX160_GYR_RANGE_250DPS         0x03
#define BMX160_GYR_RANGE_125DPS         0x04

// Output Data Rates (ODR)
#define BMX160_ACC_ODR_0_78HZ           0x01
#define BMX160_ACC_ODR_1_56HZ           0x02
#define BMX160_ACC_ODR_3_12HZ           0x03
#define BMX160_ACC_ODR_6_25HZ           0x04
#define BMX160_ACC_ODR_12_5HZ           0x05
#define BMX160_ACC_ODR_25HZ             0x06
#define BMX160_ACC_ODR_50HZ             0x07
#define BMX160_ACC_ODR_100HZ            0x08
#define BMX160_ACC_ODR_200HZ            0x09
#define BMX160_ACC_ODR_400HZ            0x0A
#define BMX160_ACC_ODR_800HZ            0x0B
#define BMX160_ACC_ODR_1600HZ           0x0C

#define BMX160_GYR_ODR_25HZ             0x06
#define BMX160_GYR_ODR_50HZ             0x07
#define BMX160_GYR_ODR_100HZ            0x08
#define BMX160_GYR_ODR_200HZ            0x09
#define BMX160_GYR_ODR_400HZ            0x0A
#define BMX160_GYR_ODR_800HZ            0x0B
#define BMX160_GYR_ODR_1600HZ           0x0C
#define BMX160_GYR_ODR_3200HZ           0x0D

// Data structure for 3-axis data
typedef struct {
    float x;
    float y;
    float z;
} bmx160_vector_t;

// Calibration data structure (compatible with existing AHRS)
typedef struct {
    bmx160_vector_t mag_offset;
    bmx160_vector_t mag_scale;
    bmx160_vector_t accel_offset;
    bmx160_vector_t accel_scale_lo;
    bmx160_vector_t accel_scale_hi;
    bmx160_vector_t gyro_bias_offset;
} bmx160_calibration_t;

// Configuration structure
typedef struct {
    uint8_t acc_range;      // Accelerometer range
    uint8_t acc_odr;        // Accelerometer ODR
    uint8_t gyro_range;     // Gyroscope range
    uint8_t gyro_odr;       // Gyroscope ODR
    uint8_t mag_odr;        // Magnetometer ODR
} bmx160_config_t;

// Function declarations
esp_err_t bmx160_init(i2c_port_t i2c_port, const bmx160_calibration_t *cal);
esp_err_t bmx160_configure(const bmx160_config_t *config);
esp_err_t bmx160_set_power_mode(uint8_t acc_mode, uint8_t gyro_mode, uint8_t mag_mode);

// Data reading functions
esp_err_t bmx160_read_accel_raw(int16_t *x, int16_t *y, int16_t *z);
esp_err_t bmx160_read_gyro_raw(int16_t *x, int16_t *y, int16_t *z);
esp_err_t bmx160_read_mag_raw(int16_t *x, int16_t *y, int16_t *z);

esp_err_t bmx160_get_accel(bmx160_vector_t *accel);
esp_err_t bmx160_get_gyro(bmx160_vector_t *gyro);
esp_err_t bmx160_get_mag(bmx160_vector_t *mag);
esp_err_t bmx160_get_accel_gyro_mag(bmx160_vector_t *accel, bmx160_vector_t *gyro, bmx160_vector_t *mag);

esp_err_t bmx160_get_temperature_celsius(float *temperature);

// Compatibility functions with existing MPU9250 interface
#define vector_t bmx160_vector_t
#define calibration_t bmx160_calibration_t

esp_err_t i2c_bmx160_init(const bmx160_calibration_t *cal);
esp_err_t get_accel_gyro_mag(bmx160_vector_t *va, bmx160_vector_t *vg, bmx160_vector_t *vm);
esp_err_t get_accel(bmx160_vector_t *v);
esp_err_t get_gyro(bmx160_vector_t *v);
esp_err_t get_mag(bmx160_vector_t *v);
esp_err_t get_temperature_celsius(float *val);

// Calibration functions
void bmx160_calibrate_gyro(void);
void bmx160_calibrate_accel(void);
void bmx160_calibrate_mag(void);

// Utility functions
esp_err_t bmx160_soft_reset(void);
esp_err_t bmx160_read_chip_id(uint8_t *chip_id);

#endif // BMX160_H