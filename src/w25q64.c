#include "w25q64.h"
#include "config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// W25Q64 SPI Commands
#define W25Q_CMD_WRITE_ENABLE   0x06
#define W25Q_CMD_WRITE_DISABLE  0x04
#define W25Q_CMD_READ_STATUS1   0x05
#define W25Q_CMD_READ_DATA      0x03
#define W25Q_CMD_PAGE_PROGRAM   0x02
#define W25Q_CMD_SECTOR_ERASE   0x20
#define W25Q_CMD_BLOCK_ERASE64  0xD8
#define W25Q_CMD_CHIP_ERASE     0xC7
#define W25Q_CMD_JEDEC_ID       0x9F
#define W25Q_CMD_POWER_UP       0xAB
#define W25Q_CMD_POWER_DOWN     0xB9

#define W25Q_STATUS_BUSY        0x01

static inline void cs_select(void) {
    gpio_put(FLASH_PIN_CS, 0);
    sleep_us(1);
}

static inline void cs_deselect(void) {
    sleep_us(1);
    gpio_put(FLASH_PIN_CS, 1);
}

static void write_enable(void) {
    uint8_t cmd = W25Q_CMD_WRITE_ENABLE;
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, &cmd, 1);
    cs_deselect();
}

static uint8_t read_status(void) {
    uint8_t cmd = W25Q_CMD_READ_STATUS1;
    uint8_t status;
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, &cmd, 1);
    spi_read_blocking(FLASH_SPI_PORT, 0, &status, 1);
    cs_deselect();
    return status;
}

bool w25q64_init(void) {
    spi_init(FLASH_SPI_PORT, FLASH_SPI_FREQ);
    gpio_set_function(FLASH_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(FLASH_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(FLASH_PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(FLASH_PIN_CS);
    gpio_set_dir(FLASH_PIN_CS, GPIO_OUT);
    gpio_put(FLASH_PIN_CS, 1);

    sleep_ms(10);

    // Wake up from power down
    uint8_t cmd = W25Q_CMD_POWER_UP;
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, &cmd, 1);
    cs_deselect();
    sleep_us(5);

    // Verify JEDEC ID
    uint32_t id = w25q64_read_jedec_id();
    // W25Q64: Manufacturer 0xEF, Device 0x4017
    if ((id >> 16) != 0xEF) {
        return false;
    }

    return true;
}

uint32_t w25q64_read_jedec_id(void) {
    uint8_t cmd = W25Q_CMD_JEDEC_ID;
    uint8_t id[3];
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, &cmd, 1);
    spi_read_blocking(FLASH_SPI_PORT, 0, id, 3);
    cs_deselect();
    return ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
}

void w25q64_read(uint32_t addr, uint8_t *buf, uint32_t len) {
    uint8_t cmd[4] = {
        W25Q_CMD_READ_DATA,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)(addr)
    };
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, cmd, 4);
    spi_read_blocking(FLASH_SPI_PORT, 0, buf, len);
    cs_deselect();
}

void w25q64_page_program(uint32_t addr, const uint8_t *data, uint16_t len) {
    if (len == 0 || len > 256) return;

    write_enable();

    uint8_t cmd[4] = {
        W25Q_CMD_PAGE_PROGRAM,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)(addr)
    };
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, cmd, 4);
    spi_write_blocking(FLASH_SPI_PORT, data, len);
    cs_deselect();

    w25q64_wait_busy();
}

void w25q64_sector_erase(uint32_t addr) {
    write_enable();

    uint8_t cmd[4] = {
        W25Q_CMD_SECTOR_ERASE,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)(addr)
    };
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, cmd, 4);
    cs_deselect();

    w25q64_wait_busy();
}

void w25q64_block_erase_64k(uint32_t addr) {
    write_enable();

    uint8_t cmd[4] = {
        W25Q_CMD_BLOCK_ERASE64,
        (uint8_t)(addr >> 16),
        (uint8_t)(addr >> 8),
        (uint8_t)(addr)
    };
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, cmd, 4);
    cs_deselect();

    w25q64_wait_busy();
}

void w25q64_chip_erase(void) {
    write_enable();

    uint8_t cmd = W25Q_CMD_CHIP_ERASE;
    cs_select();
    spi_write_blocking(FLASH_SPI_PORT, &cmd, 1);
    cs_deselect();

    // Chip erase can take up to 100 seconds for W25Q64
    w25q64_wait_busy();
}

bool w25q64_is_busy(void) {
    return (read_status() & W25Q_STATUS_BUSY) != 0;
}

void w25q64_wait_busy(void) {
    while (w25q64_is_busy()) {
        sleep_us(100);
    }
}
