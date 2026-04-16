#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "config.h"
#include "mpu6500.h"
#include "w25q64.h"

// --- Data structures stored on flash ---

typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;  // microseconds since recording start
    int16_t  ax, ay, az;    // raw accelerometer
    int16_t  gx, gy, gz;    // raw gyroscope
} sample_record_t;          // 16 bytes

_Static_assert(sizeof(sample_record_t) == 16, "sample_record_t must be 16 bytes");

typedef struct __attribute__((packed)) {
    uint32_t magic;          // DATA_MAGIC
    uint32_t version;        // DATA_VERSION
    uint32_t sample_rate_hz;
    uint32_t num_samples;
    uint32_t accel_range;    // full scale in g
    uint32_t gyro_range;     // full scale in dps
    uint8_t  reserved[256 - 24];
} flash_header_t;            // 256 bytes = 1 page

_Static_assert(sizeof(flash_header_t) == 256, "flash_header_t must be 256 bytes");

// --- State ---

static volatile bool recording = false;
static uint32_t num_samples = 0;
static uint32_t flash_write_addr = 0;

// Page buffer for batching writes (256 bytes = 16 records)
static uint8_t page_buf[FLASH_PAGE_SIZE];
static uint16_t page_buf_pos = 0;

// --- Flash page write helper ---

static void flush_page_buf(void) {
    if (page_buf_pos == 0) return;

    // Pad remaining with 0xFF
    if (page_buf_pos < FLASH_PAGE_SIZE) {
        memset(page_buf + page_buf_pos, 0xFF, FLASH_PAGE_SIZE - page_buf_pos);
    }

    // Erase sector if at sector boundary
    if ((flash_write_addr % FLASH_SECTOR_SIZE) == 0) {
        w25q64_sector_erase(flash_write_addr);
    }

    w25q64_page_program(flash_write_addr, page_buf, FLASH_PAGE_SIZE);
    flash_write_addr += FLASH_PAGE_SIZE;
    page_buf_pos = 0;
}

static void write_record(const sample_record_t *rec) {
    memcpy(page_buf + page_buf_pos, rec, sizeof(sample_record_t));
    page_buf_pos += sizeof(sample_record_t);

    if (page_buf_pos >= FLASH_PAGE_SIZE) {
        flush_page_buf();
    }
}

// --- Write header to flash page 0 ---

static void write_header(void) {
    flash_header_t hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    hdr.magic = DATA_MAGIC;
    hdr.version = DATA_VERSION;
    hdr.sample_rate_hz = SAMPLE_RATE_HZ;
    hdr.num_samples = num_samples;
    hdr.accel_range = MPU_ACCEL_RANGE_G;
    hdr.gyro_range = MPU_GYRO_RANGE_DPS;

    // Erase first sector (contains header)
    w25q64_sector_erase(0);
    w25q64_page_program(0, (const uint8_t *)&hdr, sizeof(hdr));
}

// --- USB serial command processing ---

static void process_command(const char *cmd) {
    if (strcmp(cmd, "dump") == 0) {
        // Read header
        flash_header_t hdr;
        w25q64_read(0, (uint8_t *)&hdr, sizeof(hdr));

        if (hdr.magic != DATA_MAGIC) {
            printf("ERROR: No valid data on flash\n");
            return;
        }

        printf("# KAPUTNIK Flight Data v%u\n", hdr.version);
        printf("# Sample rate: %u Hz\n", hdr.sample_rate_hz);
        printf("# Samples: %u\n", hdr.num_samples);
        printf("# Accel range: +/-%u g\n", hdr.accel_range);
        printf("# Gyro range: +/-%u dps\n", hdr.gyro_range);
        printf("timestamp_us,ax,ay,az,gx,gy,gz\n");

        uint32_t addr = FLASH_DATA_START;
        sample_record_t rec;
        for (uint32_t i = 0; i < hdr.num_samples; i++) {
            w25q64_read(addr, (uint8_t *)&rec, sizeof(rec));
            printf("%u,%d,%d,%d,%d,%d,%d\n",
                   rec.timestamp_us,
                   rec.ax, rec.ay, rec.az,
                   rec.gx, rec.gy, rec.gz);
            addr += sizeof(rec);
        }
        printf("# END\n");

    } else if (strcmp(cmd, "erase") == 0) {
        if (recording) {
            printf("ERROR: Stop recording first\n");
            return;
        }
        printf("Erasing flash...\n");
        w25q64_chip_erase();
        printf("Done\n");

    } else if (strcmp(cmd, "status") == 0) {
        flash_header_t hdr;
        w25q64_read(0, (uint8_t *)&hdr, sizeof(hdr));

        printf("Recording: %s\n", recording ? "YES" : "NO");
        if (hdr.magic == DATA_MAGIC) {
            printf("Flash data: %u samples @ %u Hz\n", hdr.num_samples, hdr.sample_rate_hz);
        } else {
            printf("Flash data: empty/invalid\n");
        }
        printf("MPU-6500 WHO_AM_I: 0x%02X\n", mpu6500_who_am_i());
        printf("W25Q64 JEDEC ID: 0x%06X\n", w25q64_read_jedec_id());

    } else if (strcmp(cmd, "start") == 0) {
        if (!recording) {
            recording = true;
            printf("Recording started via USB\n");
        }

    } else if (strcmp(cmd, "stop") == 0) {
        if (recording) {
            recording = false;
            printf("Recording stopped via USB\n");
        }

    } else if (strlen(cmd) > 0) {
        printf("Commands: dump, erase, status, start, stop\n");
    }
}

// --- Button handling with debounce ---

static uint32_t last_button_time = 0;

static void button_callback(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_button_time < 300) return; // debounce
    last_button_time = now;

    recording = !recording;
}

// --- LED blink patterns ---

static void led_set(bool on) {
    gpio_put(LED_PIN, on);
}

// --- Main ---

int main(void) {
    stdio_init_all();

    // LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    led_set(false);

    // Button with pull-up
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, button_callback);

    // Parachute pin (future)
    gpio_init(PARACHUTE_PIN);
    gpio_set_dir(PARACHUTE_PIN, GPIO_OUT);
    gpio_put(PARACHUTE_PIN, 0);

    // Wait a bit for USB to connect
    sleep_ms(2000);

    printf("\n=== KAPUTNIK Flight Logger ===\n");

    // Init MPU-6500
    if (!mpu6500_init()) {
        printf("ERROR: MPU-6500 init failed (WHO_AM_I: 0x%02X)\n", mpu6500_who_am_i());
        while (1) {
            led_set(true); sleep_ms(100);
            led_set(false); sleep_ms(100);
        }
    }
    printf("MPU-6500 OK (WHO_AM_I: 0x%02X)\n", mpu6500_who_am_i());

    // Init W25Q64
    if (!w25q64_init()) {
        printf("ERROR: W25Q64 init failed (JEDEC: 0x%06X)\n", w25q64_read_jedec_id());
        while (1) {
            led_set(true); sleep_ms(200);
            led_set(false); sleep_ms(200);
        }
    }
    printf("W25Q64 OK (JEDEC: 0x%06X)\n", w25q64_read_jedec_id());

    printf("Ready. Press button or type 'start' to begin recording.\n");
    printf("Commands: dump, erase, status, start, stop\n");

    // --- Main loop ---
    char cmd_buf[64];
    uint8_t cmd_pos = 0;
    bool was_recording = false;
    uint64_t record_start_us = 0;
    uint64_t next_sample_us = 0;
    uint32_t led_toggle_ms = 0;

    while (1) {
        // --- Check USB serial for commands ---
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            if (ch == '\r' || ch == '\n') {
                cmd_buf[cmd_pos] = '\0';
                if (cmd_pos > 0) {
                    process_command(cmd_buf);
                }
                cmd_pos = 0;
            } else if (cmd_pos < sizeof(cmd_buf) - 1) {
                cmd_buf[cmd_pos++] = (char)ch;
            }
        }

        // --- Recording state transitions ---
        if (recording && !was_recording) {
            // Start recording
            printf("Recording started! Erasing flash...\n");

            // Erase first 4KB sector for header
            w25q64_sector_erase(0);

            num_samples = 0;
            flash_write_addr = FLASH_DATA_START;
            page_buf_pos = 0;
            record_start_us = time_us_64();
            next_sample_us = record_start_us;
            was_recording = true;

            printf("Recording at %d Hz. Press button or 'stop' to end.\n", SAMPLE_RATE_HZ);
        }

        if (!recording && was_recording) {
            // Stop recording
            flush_page_buf();
            write_header();
            was_recording = false;
            printf("Recording stopped. %u samples saved.\n", num_samples);
            printf("Use 'dump' to download data as CSV.\n");
            led_set(false);
        }

        // --- Sample data ---
        if (recording) {
            uint64_t now_us = time_us_64();

            if (now_us >= next_sample_us) {
                // Check if flash is full
                if (flash_write_addr + page_buf_pos + sizeof(sample_record_t) >= FLASH_TOTAL_SIZE) {
                    printf("Flash full! Stopping.\n");
                    recording = false;
                    continue;
                }

                mpu6500_data_t mpu_data;
                mpu6500_read_all(&mpu_data);

                sample_record_t rec;
                rec.timestamp_us = (uint32_t)(now_us - record_start_us);
                rec.ax = mpu_data.accel_x;
                rec.ay = mpu_data.accel_y;
                rec.az = mpu_data.accel_z;
                rec.gx = mpu_data.gyro_x;
                rec.gy = mpu_data.gyro_y;
                rec.gz = mpu_data.gyro_z;

                write_record(&rec);
                num_samples++;

                next_sample_us += SAMPLE_INTERVAL_US;

                // If we fell behind, catch up
                if (next_sample_us < now_us) {
                    next_sample_us = now_us + SAMPLE_INTERVAL_US;
                }
            }

            // Blink LED while recording
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            if (now_ms - led_toggle_ms >= 500) {
                led_toggle_ms = now_ms;
                gpio_xor_mask(1u << LED_PIN);
            }
        }
    }

    return 0;
}
