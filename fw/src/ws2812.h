/*
 * ws2812.h – Minimální driver pro WS2812B RGB NeoPixel LED přes PIO
 *
 * RP2040-Zero má onboard WS2812B na GP16 (řízeno přes PIO0).
 * Protokol WS2812B vyžaduje přesné časování bitového proudu (800 kHz),
 * proto se používá PIO (Programmable I/O) místo bitbangingu.
 *
 * Formát dat:  GRB, 24 bitů na pixel, MSB first.
 * Řazení bytů:  [G7..G0] [R7..R0] [B7..B0]
 *
 * Použití:
 *   ws2812_init(16);                          // inicializace na GP16
 *   ws2812_put_pixel(ws2812_rgb(0, 40, 0));   // zelená (nízký jas)
 *   ws2812_off();                              // zhasnout
 */

#ifndef WS2812_H
#define WS2812_H

#include <stdint.h>
#include <stdbool.h>

/* Inicializace WS2812 PIO programu na zvoleném GPIO pinu */
void ws2812_init(unsigned int pin);

/* Odeslání barvy na LED (formát GRB, 24 bit, zarovnáno do horních 24 bitů uint32) */
void ws2812_put_pixel(uint32_t pixel_grb);

/* Helper: převede RGB hodnoty (0-255) na GRB formát pro WS2812 */
static inline uint32_t ws2812_rgb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
}

/* Zhasnutí LED (barva = 0x000000) */
void ws2812_off(void);

#endif /* WS2812_H */
