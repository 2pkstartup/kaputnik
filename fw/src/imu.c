/*
 * imu.c – SPI driver pro InvenSense MPU-6500 / MPU-9250 IMU
 *
 * Podporované funkce:
 *  - Accel/gyro (MPU-6500/9250) přes SPI
 *  - AK8963 magnetometr (na MPU-9250) přes interní I2C master
 */

#include "imu.h"
#include "config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_CONFIG2   0x1D
#define MPU_REG_INT_PIN_CFG     0x37
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_EXT_SENS_DATA_00 0x49
#define MPU_REG_USER_CTRL       0x6A
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_PWR_MGMT_2      0x6C
#define MPU_REG_I2C_MST_CTRL    0x24
#define MPU_REG_I2C_SLV0_ADDR   0x25
#define MPU_REG_I2C_SLV0_REG    0x26
#define MPU_REG_I2C_SLV0_CTRL   0x27
#define MPU_REG_I2C_SLV0_DO     0x63
#define MPU_REG_WHO_AM_I        0x75

#define MPU_WHO_AM_I_MPU6500    0x70
#define MPU_WHO_AM_I_MPU9250    0x71

#define MPU_READ_FLAG           0x80

#define AK8963_I2C_ADDR         0x0C
#define AK8963_WHO_AM_I         0x00
#define AK8963_ST1              0x02
#define AK8963_HXL              0x03
#define AK8963_ST2              0x09
#define AK8963_CNTL1            0x0A
#define AK8963_WHO_AM_I_VALUE   0x48

static bool mag_available = false;

static inline void cs_select(void) {
    gpio_put(MPU_PIN_CS, 0);
    sleep_us(1);
}

static inline void cs_deselect(void) {
    sleep_us(1);
    gpio_put(MPU_PIN_CS, 1);
}

static void imu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, buf, 2);
    cs_deselect();
}

static uint8_t imu_read_reg(uint8_t reg) {
    uint8_t tx = reg | MPU_READ_FLAG;
    uint8_t rx;
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, &tx, 1);
    spi_read_blocking(MPU_SPI_PORT, 0, &rx, 1);
    cs_deselect();
    return rx;
}

static void imu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t tx = reg | MPU_READ_FLAG;
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, &tx, 1);
    spi_read_blocking(MPU_SPI_PORT, 0, buf, len);
    cs_deselect();
}

static void imu_i2c_slv0_disable(void) {
    imu_write_reg(MPU_REG_I2C_SLV0_CTRL, 0x00);
}

static void imu_ak8963_write(uint8_t reg, uint8_t val) {
    imu_write_reg(MPU_REG_I2C_SLV0_ADDR, AK8963_I2C_ADDR & 0x7F);
    imu_write_reg(MPU_REG_I2C_SLV0_REG, reg);
    imu_write_reg(MPU_REG_I2C_SLV0_DO, val);
    imu_write_reg(MPU_REG_I2C_SLV0_CTRL, 0x81);
    sleep_ms(2);
    imu_i2c_slv0_disable();
}

static void imu_ak8963_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    imu_write_reg(MPU_REG_I2C_SLV0_ADDR, (AK8963_I2C_ADDR | 0x80));
    imu_write_reg(MPU_REG_I2C_SLV0_REG, reg);
    imu_write_reg(MPU_REG_I2C_SLV0_CTRL, (uint8_t)(0x80 | (len & 0x0F)));
    sleep_ms(2);
    imu_read_regs(MPU_REG_EXT_SENS_DATA_00, buf, len);
    imu_i2c_slv0_disable();
}

uint8_t imu_who_am_i(void) {
    return imu_read_reg(MPU_REG_WHO_AM_I);
}

void imu_read_all(imu_data_t *data) {
    uint8_t buf[14];
    imu_read_regs(MPU_REG_ACCEL_XOUT_H, buf, 14);

    data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    data->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
}

void imu_set_accel_range(uint8_t range) {
    imu_write_reg(MPU_REG_ACCEL_CONFIG, (range & 0x03) << 3);
}

void imu_set_gyro_range(uint8_t range) {
    imu_write_reg(MPU_REG_GYRO_CONFIG, (range & 0x03) << 3);
}

void imu_set_sample_rate_div(uint8_t divider) {
    imu_write_reg(MPU_REG_SMPLRT_DIV, divider);
}

uint8_t imu_mag_who_am_i(void) {
    uint8_t id = 0;
    imu_ak8963_read(AK8963_WHO_AM_I, &id, 1);
    return id;
}

bool imu_mag_is_available(void) {
    return mag_available;
}

bool imu_mag_read(imu_mag_data_t *mag) {
    if (!mag_available) return false;

    uint8_t raw[8];
    imu_ak8963_read(AK8963_ST1, raw, sizeof(raw));

    if ((raw[0] & 0x01) == 0) {
        return false;
    }
    if ((raw[7] & 0x08) != 0) {
        return false;
    }

    mag->mag_x = (int16_t)((raw[2] << 8) | raw[1]);
    mag->mag_y = (int16_t)((raw[4] << 8) | raw[3]);
    mag->mag_z = (int16_t)((raw[6] << 8) | raw[5]);
    return true;
}

bool imu_init(void) {
    spi_init(MPU_SPI_PORT, MPU_SPI_FREQ);
    gpio_set_function(MPU_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(MPU_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MPU_PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(MPU_PIN_CS);
    gpio_set_dir(MPU_PIN_CS, GPIO_OUT);
    gpio_put(MPU_PIN_CS, 1);

    sleep_ms(100);

    imu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);

    imu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    sleep_ms(10);

    uint8_t id = imu_who_am_i();
    if (id != MPU_WHO_AM_I_MPU6500 && id != MPU_WHO_AM_I_MPU9250) {
        return false;
    }

    imu_write_reg(MPU_REG_CONFIG, 0x02);
    imu_write_reg(MPU_REG_ACCEL_CONFIG2, 0x02);
    imu_set_sample_rate_div(1);
    imu_set_accel_range(3);
    imu_set_gyro_range(3);
    imu_write_reg(MPU_REG_PWR_MGMT_2, 0x00);

    mag_available = false;
    if (id == MPU_WHO_AM_I_MPU9250) {
        imu_write_reg(MPU_REG_USER_CTRL, 0x20);
        imu_write_reg(MPU_REG_INT_PIN_CFG, 0x00);
        imu_write_reg(MPU_REG_I2C_MST_CTRL, 0x0D);
        sleep_ms(10);

        if (imu_mag_who_am_i() == AK8963_WHO_AM_I_VALUE) {
            imu_ak8963_write(AK8963_CNTL1, 0x00);
            sleep_ms(10);
            imu_ak8963_write(AK8963_CNTL1, 0x16);
            sleep_ms(10);
            mag_available = true;
        }
    }

    return true;
}
