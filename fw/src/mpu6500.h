/*
 * mpu6500.h – SPI driver pro InvenSense MPU-6500 IMU
 *
 * MPU-6500 je 6-osý senzor pohybu (3-osý akcelerometr + 3-osý gyroskop)
 * připojený přes SPI0. Driver provádí inicializaci, konfiguraci rozsahů
 * a burst čtení všech os najednou (14 bytů = accel + temp + gyro).
 *
 * Výchozí nastavení pro Kaputnik:
 *   - Akcelerometr: ±16g  (rozsah 3)
 *   - Gyroskop:     ±2000°/s (rozsah 3)
 *   - DLPF:         ~92 Hz bandwidth
 *   - Sample rate:  500 Hz (divider = 1)
 *   - WHO_AM_I:     0x70
 */

#ifndef MPU6500_H
#define MPU6500_H

#include <stdint.h>
#include <stdbool.h>

/* Struktura pro surová data ze všech os MPU-6500 */
typedef struct {
    int16_t accel_x;    /* Raw akcelerometr osa X (LSB, ±16g → ±32768) */
    int16_t accel_y;    /* Raw akcelerometr osa Y                       */
    int16_t accel_z;    /* Raw akcelerometr osa Z                       */
    int16_t gyro_x;     /* Raw gyroskop osa X (LSB, ±2000°/s → ±32768) */
    int16_t gyro_y;     /* Raw gyroskop osa Y                           */
    int16_t gyro_z;     /* Raw gyroskop osa Z                           */
    int16_t temp_raw;   /* Raw teplota čipu (nepoužíváno při záznamu)   */
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
