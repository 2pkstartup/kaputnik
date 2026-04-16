/*
 * w25q64.h – SPI driver pro Winbond W25Q64 NOR flash paměť
 *
 * W25Q64FVSSIQ je 64 Mbit (8 MB) sériová NOR flash s SPI rozhraním.
 * Připojeno na SPI1 (piny definované v config.h).
 *
 * Organizace paměti:
 *   - 128 bloků × 64 KB = 8 MB
 *   - 2048 sektorů × 4 KB
 *   - 32768 stránek × 256 B  (minimální jednotka programování)
 *
 * Klíčové vlastnosti:
 *   - Stránkové programování: max 256 B najednou
 *   - Mazání po sektorech (4 KB), blocích (64 KB) nebo celý čip
 *   - JEDEC ID: 0xEF4017 (Winbond, W25Q64)
 *   - Maximální SPI frekvence: 104 MHz (používáme 10 MHz)
 */

#ifndef W25Q64_H
#define W25Q64_H

#include <stdint.h>
#include <stdbool.h>

/* Inicializace W25Q64 – SPI piny, wake-up, ověření JEDEC ID.
 * Vrací true pokud je čip rozpoznán (Winbond manufacturer ID 0xEF). */
bool w25q64_init(void);

/* Čtení JEDEC ID – 3 byty: [manufacturer][memory_type][capacity]
 * Pro W25Q64: 0xEF4017 */
uint32_t w25q64_read_jedec_id(void);

/* Čtení dat z libovolné adresy. Nemá omezení na hranice stránek. */
void w25q64_read(uint32_t addr, uint8_t *buf, uint32_t len);

/* Programování jedné stránky (max 256 B). Data nesmí přesahovat
 * hranici 256B stránky. Blokuje dokud zápis neskončí. */
void w25q64_page_program(uint32_t addr, const uint8_t *data, uint16_t len);

/* Mazání 4KB sektoru. Adresa musí být zarovnána na 4096.
 * Typická doba trvání: 45-400 ms. Blokuje. */
void w25q64_sector_erase(uint32_t addr);

/* Mazání 64KB bloku. Adresa musí být zarovnána na 65536.
 * Typická doba trvání: 150-2000 ms. Blokuje. */
void w25q64_block_erase_64k(uint32_t addr);

/* Mazání celého čipu (8 MB).
 * Typická doba trvání: 20-100 s! Blokuje. */
void w25q64_chip_erase(void);

/* Kontrola zda probíhá operace zápisu/mazání (BUSY bit ve Status Reg 1) */
bool w25q64_is_busy(void);

/* Čeká v cyklu dokud flash není volná (BUSY == 0) */
void w25q64_wait_busy(void);

#endif /* W25Q64_H */
