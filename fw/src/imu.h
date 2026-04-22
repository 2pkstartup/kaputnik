/*
 * imu.h – SPI driver pro InvenSense MPU-6500 / MPU-9250 IMU
 *
 * Driver obsluhuje akcelerometr + gyroskop přes SPI a na MPU-9250 také
 * integrovaný magnetometr AK8963 přes interní I2C master.
 */

#ifndef IMU_H
#define IMU_H

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
} imu_data_t;

typedef struct {
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} imu_mag_data_t;

bool imu_init(void);
uint8_t imu_who_am_i(void);
void imu_read_all(imu_data_t *data);
void imu_set_accel_range(uint8_t range);
void imu_set_gyro_range(uint8_t range);
void imu_set_sample_rate_div(uint8_t divider);

bool imu_mag_is_available(void);
uint8_t imu_mag_who_am_i(void);
bool imu_mag_read(imu_mag_data_t *mag);

#endif /* IMU_H */
