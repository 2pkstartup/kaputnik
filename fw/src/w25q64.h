#ifndef W25Q64_H
#define W25Q64_H

#include <stdint.h>
#include <stdbool.h>

// Initialize W25Q64 SPI flash. Returns true on success.
bool w25q64_init(void);

// Read JEDEC ID (should be 0xEF4017 for W25Q64).
uint32_t w25q64_read_jedec_id(void);

// Read data from flash.
void w25q64_read(uint32_t addr, uint8_t *buf, uint32_t len);

// Program a page (up to 256 bytes). addr must be page-aligned for full page writes.
// Data must not cross a 256-byte page boundary.
void w25q64_page_program(uint32_t addr, const uint8_t *data, uint16_t len);

// Erase a 4KB sector. addr must be sector-aligned (multiple of 4096).
void w25q64_sector_erase(uint32_t addr);

// Erase a 64KB block. addr must be block-aligned (multiple of 65536).
void w25q64_block_erase_64k(uint32_t addr);

// Erase entire chip (takes several seconds).
void w25q64_chip_erase(void);

// Check if flash is busy with write/erase operation.
bool w25q64_is_busy(void);

// Wait until flash is no longer busy.
void w25q64_wait_busy(void);

#endif // W25Q64_H
