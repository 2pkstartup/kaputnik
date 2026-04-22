#include <stdio.h>

#include "pico/stdlib.h"

#include "config.h"
#include "imu.h"
#include "ws2812.h"

#define STREAM_RATE_HZ 50
#define STREAM_INTERVAL_MS (1000 / STREAM_RATE_HZ)

int main(void) {
    stdio_init_all();

    ws2812_init(WS2812_PIN);
    ws2812_put_pixel(ws2812_rgb(40, 14, 0));

    sleep_ms(2000);

    printf("\n=== KAPUTNIK IMU USB TEST ===\n");
    printf("# Wiring: GP2=SCK GP3=MOSI GP4=MISO GP5=CS\n");

    if (!imu_init()) {
        printf("ERROR: IMU init failed (WHO_AM_I: 0x%02X)\n", imu_who_am_i());
        ws2812_put_pixel(ws2812_rgb(40, 0, 0));
        while (1) {
            tight_loop_contents();
        }
    }

    printf("IMU OK (WHO_AM_I: 0x%02X)\n", imu_who_am_i());
    if (imu_mag_is_available()) {
        printf("MAG OK (AK8963 WHO_AM_I: 0x%02X)\n", imu_mag_who_am_i());
    } else {
        printf("MAG unavailable (expected for MPU-6500 based modules)\n");
    }

    ws2812_put_pixel(ws2812_rgb(0, 40, 0));

    printf("# Streaming raw accel/gyro as CSV at %d Hz\n", STREAM_RATE_HZ);
    printf("timestamp_us,ax,ay,az,gx,gy,gz,temp_raw\n");

    while (1) {
        imu_data_t sample;
        uint64_t timestamp_us = time_us_64();

        imu_read_all(&sample);
        printf("%llu,%d,%d,%d,%d,%d,%d,%d\n",
               timestamp_us,
               sample.accel_x,
               sample.accel_y,
               sample.accel_z,
               sample.gyro_x,
               sample.gyro_y,
               sample.gyro_z,
               sample.temp_raw);

        sleep_ms(STREAM_INTERVAL_MS);
    }

    return 0;
}