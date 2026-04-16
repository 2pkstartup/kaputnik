/*
 * ws2812.c – Implementace WS2812B driveru přes PIO
 *
 * Využívá PIO0 state machine pro generování WS2812B protokolu.
 * PIO program (ws2812.pio) z Pico SDK zajišťuje přesné časování
 * bitového proudu na 800 kHz bez zátěže CPU.
 */

#include "ws2812.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"              /* Generovaný header z .pio souboru */

static PIO ws2812_pio = pio0;         /* PIO instance (0 nebo 1)           */
static uint ws2812_sm = 0;            /* State machine index               */

/* Inicializace: načte PIO program, nakonfiguruje SM a povolí výstup */
void ws2812_init(unsigned int pin) {
    uint offset = pio_add_program(ws2812_pio, &ws2812_program);
    ws2812_sm = pio_claim_unused_sm(ws2812_pio, true);
    ws2812_program_init(ws2812_pio, ws2812_sm, offset, pin, 800000, false);
}

/* Odeslání 24-bitové barvy (GRB) do PIO FIFO – blokující zápis */
void ws2812_put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(ws2812_pio, ws2812_sm, pixel_grb);
}

/* Zhasnutí LED – pošle nulovou barvu */
void ws2812_off(void) {
    ws2812_put_pixel(0);
}
