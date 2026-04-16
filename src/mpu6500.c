#include "mpu6500.h"
#include "config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// MPU-6500 Register addresses
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_CONFIG2   0x1D
#define MPU_REG_ACCEL_XOUT_H   0x3B
#define MPU_REG_TEMP_OUT_H      0x41
#define MPU_REG_GYRO_XOUT_H    0x43
#define MPU_REG_PWR_MGMT_1     0x6B
#define MPU_REG_PWR_MGMT_2     0x6C
#define MPU_REG_WHO_AM_I       0x75

#define MPU_READ_FLAG  0x80

static inline void cs_select(void) {
    gpio_put(MPU_PIN_CS, 0);
    sleep_us(1);
}

static inline void cs_deselect(void) {
    sleep_us(1);
    gpio_put(MPU_PIN_CS, 1);
}

static void mpu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, buf, 2);
    cs_deselect();
}

static uint8_t mpu_read_reg(uint8_t reg) {
    uint8_t tx = reg | MPU_READ_FLAG;
    uint8_t rx;
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, &tx, 1);
    spi_read_blocking(MPU_SPI_PORT, 0, &rx, 1);
    cs_deselect();
    return rx;
}

static void mpu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t tx = reg | MPU_READ_FLAG;
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, &tx, 1);
    spi_read_blocking(MPU_SPI_PORT, 0, buf, len);
    cs_deselect();
}

bool mpu6500_init(void) {
    // Init SPI pins
    spi_init(MPU_SPI_PORT, MPU_SPI_FREQ);
    gpio_set_function(MPU_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(MPU_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MPU_PIN_MISO, GPIO_FUNC_SPI);

    // CS pin as GPIO output, deselect
    gpio_init(MPU_PIN_CS);
    gpio_set_dir(MPU_PIN_CS, GPIO_OUT);
    gpio_put(MPU_PIN_CS, 1);

    sleep_ms(100);

    // Reset device
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);

    // Wake up, use auto-select best clock
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    sleep_ms(10);

    // Check WHO_AM_I
    uint8_t id = mpu6500_who_am_i();
    if (id != 0x70) {
        return false;
    }

    // DLPF config: bandwidth ~92 Hz for accel and gyro
    mpu_write_reg(MPU_REG_CONFIG, 0x02);
    mpu_write_reg(MPU_REG_ACCEL_CONFIG2, 0x02);

    // Set sample rate divider: 1kHz / (1+1) = 500 Hz
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 1);

    // Set accel range ±16g
    mpu6500_set_accel_range(3);

    // Set gyro range ±2000 °/s
    mpu6500_set_gyro_range(3);

    // Enable all axes
    mpu_write_reg(MPU_REG_PWR_MGMT_2, 0x00);

    return true;
}

uint8_t mpu6500_who_am_i(void) {
    return mpu_read_reg(MPU_REG_WHO_AM_I);
}

void mpu6500_read_all(mpu6500_data_t *data) {
    uint8_t buf[14];
    // Read 14 bytes starting from ACCEL_XOUT_H
    // Layout: AXH AXL AYH AYL AZH AZL TH TL GXH GXL GYH GYL GZH GZL
    mpu_read_regs(MPU_REG_ACCEL_XOUT_H, buf, 14);

    data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    data->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
}

void mpu6500_set_accel_range(uint8_t range) {
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, (range & 0x03) << 3);
}

void mpu6500_set_gyro_range(uint8_t range) {
    mpu_write_reg(MPU_REG_GYRO_CONFIG, (range & 0x03) << 3);
}

void mpu6500_set_sample_rate_div(uint8_t divider) {
    mpu_write_reg(MPU_REG_SMPLRT_DIV, divider);
}
