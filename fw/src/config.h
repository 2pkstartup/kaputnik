#ifndef CONFIG_H
#define CONFIG_H

// --- MPU-6500 on SPI0 ---
#define MPU_SPI_PORT    spi0
#define MPU_SPI_FREQ    1000000  // 1 MHz
#define MPU_PIN_SCK     2
#define MPU_PIN_MOSI    3
#define MPU_PIN_MISO    4
#define MPU_PIN_CS      5

// --- W25Q64 Flash on SPI1 ---
#define FLASH_SPI_PORT  spi1
#define FLASH_SPI_FREQ  10000000 // 10 MHz
#define FLASH_PIN_SCK   10
#define FLASH_PIN_MOSI  11
#define FLASH_PIN_MISO  12
#define FLASH_PIN_CS    13

// --- Button (active low, internal pull-up) ---
#define BUTTON_PIN      15

// --- Onboard LED ---
#define LED_PIN         25

// --- Parachute deploy output (future use) ---
#define PARACHUTE_PIN   16

// --- Sampling ---
#define SAMPLE_RATE_HZ  500
#define SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE_HZ)

// --- MPU-6500 settings ---
#define MPU_ACCEL_RANGE_G   16   // ±16g (suitable for rocket)
#define MPU_GYRO_RANGE_DPS  2000 // ±2000 °/s

// --- Flash data layout ---
// Header: page 0 (256 bytes)
// Data:   page 1+ (16 bytes per record, 16 records per page)
#define FLASH_PAGE_SIZE     256
#define FLASH_SECTOR_SIZE   4096
#define FLASH_TOTAL_SIZE    (8 * 1024 * 1024) // 8 MB = 64 Mbit
#define FLASH_DATA_START    FLASH_PAGE_SIZE    // data starts after header page
#define FLASH_MAX_RECORDS   ((FLASH_TOTAL_SIZE - FLASH_DATA_START) / sizeof(sample_record_t))

// --- Data record magic ---
#define DATA_MAGIC  0x5550414B  // "KAPU"
#define DATA_VERSION 2

#endif // CONFIG_H
