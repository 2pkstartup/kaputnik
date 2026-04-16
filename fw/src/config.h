/*
 * config.h – Konfigurace pinů a parametrů pro Kaputnik flight logger
 *
 * Cílová deska: Waveshare RP2040-Zero
 *
 * Všechny GPIO piny a konstanty pro periferie (MPU-6500, W25Q64, tlačítko,
 * LED, padák) a parametry záznamu (vzorkovací frekvence, rozsahy, layout
 * dat na flash) jsou definovány zde na jednom místě.
 */

#ifndef CONFIG_H
#define CONFIG_H

/* -----------------------------------------------------------------------
 *  MPU-6500 IMU na SPI0
 *  3-osý akcelerometr + gyroskop (InvenSense/TDK)
 * ----------------------------------------------------------------------- */
#define MPU_SPI_PORT    spi0
#define MPU_SPI_FREQ    1000000  /* 1 MHz – bezpečná frekvence pro SPI     */
#define MPU_PIN_SCK     2        /* GP2  – SPI0 SCK                        */
#define MPU_PIN_MOSI    3        /* GP3  – SPI0 TX  (Master Out Slave In)  */
#define MPU_PIN_MISO    4        /* GP4  – SPI0 RX  (Master In Slave Out)  */
#define MPU_PIN_CS      5        /* GP5  – Chip Select (active low, GPIO)  */

/* -----------------------------------------------------------------------
 *  W25Q64 Flash paměť na SPI1
 *  Winbond 64 Mbit (8 MB) NOR flash
 * ----------------------------------------------------------------------- */
#define FLASH_SPI_PORT  spi1
#define FLASH_SPI_FREQ  10000000 /* 10 MHz – max pro spolehlivou komunikaci */
#define FLASH_PIN_SCK   10       /* GP10 – SPI1 SCK                        */
#define FLASH_PIN_MOSI  11       /* GP11 – SPI1 TX                         */
#define FLASH_PIN_MISO  12       /* GP12 – SPI1 RX                         */
#define FLASH_PIN_CS    13       /* GP13 – Chip Select (active low, GPIO)  */

/* -----------------------------------------------------------------------
 *  Ovládací piny
 * ----------------------------------------------------------------------- */
#define BUTTON_PIN      15       /* GP15 – Tlačítko start/stop záznamu      */
                                 /*        Active low, interní pull-up      */

#define WS2812_PIN      16       /* GP16 – WS2812B RGB LED (onboard)        */
                                 /*        Řízeno přes PIO, 800 kHz         */

#define PARACHUTE_PIN   14       /* GP14 – Výstup pro padák (budoucí)       */
                                 /*        GPIO output, active high         */

/* -----------------------------------------------------------------------
 *  Vzorkování IMU
 * ----------------------------------------------------------------------- */
#define SAMPLE_RATE_HZ  500                           /* Vzorkovací frekvence [Hz] */
#define SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE_HZ) /* Perioda vzorkování [µs]   */

/* -----------------------------------------------------------------------
 *  Nastavení MPU-6500
 * ----------------------------------------------------------------------- */
#define MPU_ACCEL_RANGE_G   16   /* ±16g – plný rozsah pro raketové lety    */
#define MPU_GYRO_RANGE_DPS  2000 /* ±2000 °/s – plný rozsah                 */

/* -----------------------------------------------------------------------
 *  Layout dat na flash paměti
 *
 *  Stránka 0 (256 B):  Hlavička záznamu (flash_header_t)
 *  Stránky 1+:         Data (sample_record_t × N)
 *
 *  Sektory:  4 KB (nejmenší mazetelná jednotka)
 *  Stránky:  256 B (jednotka programování)
 * ----------------------------------------------------------------------- */
#define FLASH_PAGE_SIZE     256
#define FLASH_SECTOR_SIZE   4096
#define FLASH_TOTAL_SIZE    (8 * 1024 * 1024) /* 8 MB = 64 Mbit             */
#define FLASH_DATA_START    FLASH_PAGE_SIZE    /* Data začínají na stránce 1 */
#define FLASH_MAX_RECORDS   ((FLASH_TOTAL_SIZE - FLASH_DATA_START) / sizeof(sample_record_t))

/* -----------------------------------------------------------------------
 *  Identifikace datového formátu
 * ----------------------------------------------------------------------- */
#define DATA_MAGIC  0x5550414B  /* ASCII "KAPU" (little-endian)              */
#define DATA_VERSION 2          /* Verze formátu (v2 = s epoch_ms_start)     */

#endif /* CONFIG_H */
