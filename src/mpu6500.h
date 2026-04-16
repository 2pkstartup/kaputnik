#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp_raw;
} mpu6500_data_t;

// Initialize MPU-6500 over SPI. Returns true on success.
bool mpu6500_init(void);

// Read WHO_AM_I register (should return 0x70 for MPU-6500).
uint8_t mpu6500_who_am_i(void);

// Read all sensor data (accel + gyro + temp) in one burst.
void mpu6500_read_all(mpu6500_data_t *data);

// Set accelerometer full-scale range: 0=±2g, 1=±4g, 2=±8g, 3=±16g
void mpu6500_set_accel_range(uint8_t range);

// Set gyroscope full-scale range: 0=±250, 1=±500, 2=±1000, 3=±2000 °/s
void mpu6500_set_gyro_range(uint8_t range);

// Set sample rate divider (sample rate = 1kHz / (1 + divider))
void mpu6500_set_sample_rate_div(uint8_t divider);

#endif // MPU6500_H
