/*
 * ==========================================================================
 *  KAPUTNIK – Flight Data Logger pro modelářské rakety
 * ==========================================================================
 *
 *  Projekt:    Kaputnik
 *  Soubor:     main.c
 *  Popis:      Hlavní řídicí smyčka firmware – inicializace hardware,
 *              záznam letových dat z IMU na externí flash paměť,
 *              USB sériová komunikace pro stahování dat a ovládání.
 *
 *  MCU:        RP2040 (Waveshare RP2040-Zero)
 *  IMU:        InvenSense MPU-6500  (3-osý akcelerometr + gyroskop, SPI)
 *  Flash:      Winbond W25Q64FVSSIQ (64 Mbit / 8 MB, SPI)
 *  LED:        WS2812B RGB NeoPixel (onboard na RP2040-Zero, PIO)
 *
 * --------------------------------------------------------------------------
 *  ZAPOJENÍ PINŮ  (Waveshare RP2040-Zero)
 * --------------------------------------------------------------------------
 *
 *  MPU-6500 (SPI0):
 *      GP2  ── SCK   (SPI0 SCK)
 *      GP3  ── MOSI  (SPI0 TX)
 *      GP4  ── MISO  (SPI0 RX)
 *      GP5  ── CS    (GPIO, active low)
 *
 *  W25Q64 Flash (SPI1):
 *      GP10 ── SCK   (SPI1 SCK)
 *      GP11 ── MOSI  (SPI1 TX)
 *      GP12 ── MISO  (SPI1 RX)
 *      GP13 ── CS    (GPIO, active low)
 *
 *  Ovládání:
 *      GP15 ── Tlačítko start/stop záznamu (active low, interní pull-up)
 *      GP14 ── Padákový výstup (GPIO output, active high, 1 s puls)
 *
 *  Onboard:
 *      GP16 ── WS2812B RGB LED (řízeno přes PIO)
 *      USB-C ── CDC sériová linka (115200 baud)
 *
 * --------------------------------------------------------------------------
 *  FORMÁT DAT NA FLASH
 * --------------------------------------------------------------------------
 *
 *  Stránka 0 (256 B):  flash_header_t – magic, verze, počet vzorků, epoch
 *  Stránky 1+:         sample_record_t – 16 B na záznam, 16 záznamů/stránka
 *
 *  Jeden záznam (16 B, packed):
 *      uint32_t timestamp_us   – čas od začátku záznamu [µs]
 *      int16_t  ax, ay, az     – raw akcelerometr (±16g → ±32768)
 *      int16_t  gx, gy, gz     – raw gyroskop (±2000°/s → ±32768)
 *
 *  Kapacita: ~500 000 vzorků ≈ 16 minut při 500 Hz
 *
 * --------------------------------------------------------------------------
 *  USB PŘÍKAZY (sériový terminál 115200 baud)
 * --------------------------------------------------------------------------
 *
 *      start              – spustit záznam
 *      stop               – zastavit záznam
 *      dump               – vypsat data jako CSV (epoch_ms,ax,ay,az,gx,gy,gz)
 *      erase              – smazat celou flash
 *      status             – stav zařízení (MPU, flash, čas, záznam)
 *      settime <epoch_s>  – nastavit vnitřní hodiny (epoch sekundy od 1970)
 *
 * --------------------------------------------------------------------------
 *  LED INDIKACE (WS2812B RGB)
 * --------------------------------------------------------------------------
 *
 *      Zelená (svítí)           – připraveno (ready)
 *      Modrá (bliká 500 ms)    – probíhá záznam
 *      Fialová (bliká 300 ms)  – apogee detekováno, padák aktivován
 *      Červená (bliká 100 ms)  – chyba MPU-6500
 *      Žlutá (bliká 200 ms)   – chyba W25Q64 flash
 *
 * --------------------------------------------------------------------------
 *  EMA FILTR A DETEKCE APOGEA
 * --------------------------------------------------------------------------
 *
 *  Surová data z IMU jsou vyhlazena exponenciálním klouzavým průměrem (EMA),
 *  který odstraní vibrace motoru a strukturální rezonance trupu rakety.
 *  Filtrovaná Z-osa akcelerometru se používá pro detekci fází letu:
 *
 *      IDLE      → čeká se na startu (baseline az ≈ +1g)
 *      LAUNCHED  → tah motoru detekován (az > baseline + 3g)
 *      APOGEE    → rychlost klesla pod nulu → padák aktivován
 *
 *  Rychlost se odhaduje numerickou integrací filtrovaného zrychlení
 *  (po odečtení gravitační složky).
 *
 * --------------------------------------------------------------------------
 *  Licence:    MIT
 * ==========================================================================
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "config.h"
#include "mpu6500.h"
#include "w25q64.h"
#include "ws2812.h"

/* =======================================================================
 *  Datové struktury ukládané na flash
 * =======================================================================
 *
 *  Flash layout:
 *    Adresa 0x000000 .. 0x0000FF  →  flash_header_t (1 stránka = 256 B)
 *    Adresa 0x000100 .. 0x7FFFFF  →  sample_record_t (16 B × N)
 *
 *  Každá stránka (256 B) pojme přesně 16 záznamů.
 *  Zápis probíhá stránkově – záznamy se bufferují v RAM a zapisují
 *  najednou po naplnění 256B bufferu.
 * ======================================================================= */

/* Jeden vzorek letových dat – 16 bytů, packed pro přesné zarovnání na flash */
typedef struct __attribute__((packed)) {
    uint32_t timestamp_us;  /* Čas od začátku záznamu [µs]                   */
    int16_t  ax, ay, az;    /* Raw akcelerometr (LSB, ±16g → ±32768)         */
    int16_t  gx, gy, gz;    /* Raw gyroskop     (LSB, ±2000°/s → ±32768)     */
} sample_record_t;          /* Celkem: 4 + 6×2 = 16 bytů                     */

_Static_assert(sizeof(sample_record_t) == 16, "sample_record_t must be 16 bytes");

/* Hlavička záznamu – uložena na stránce 0 flash po ukončení záznamu */
typedef struct __attribute__((packed)) {
    uint32_t magic;          /* DATA_MAGIC ("KAPU" = 0x5550414B)             */
    uint32_t version;        /* DATA_VERSION – formát dat                    */
    uint32_t sample_rate_hz; /* Vzorkovací frekvence [Hz]                    */
    uint32_t num_samples;    /* Celkový počet uložených vzorků               */
    uint32_t accel_range;    /* Rozsah akcelerometru [g]                     */
    uint32_t gyro_range;     /* Rozsah gyroskopu [°/s]                       */
    uint64_t epoch_ms_start; /* Epoch čas začátku záznamu [ms] (0 = nenastaveno) */
    uint8_t  reserved[256 - 32]; /* Rezerva do konce stránky                 */
} flash_header_t;            /* Celkem: 256 bytů = 1 flash stránka           */

_Static_assert(sizeof(flash_header_t) == 256, "flash_header_t must be 256 bytes");

/* =======================================================================
 *  Globální stav aplikace
 * ======================================================================= */

static volatile bool recording = false;    /* true = probíhá záznam (nastavuje ISR i USB) */
static uint32_t num_samples = 0;           /* Počet dosud zaznamenaných vzorků              */
static uint32_t flash_write_addr = 0;      /* Aktuální adresa pro zápis na flash            */

/* =======================================================================
 *  EMA (Exponenciální klouzavý průměr)
 *
 *  Vyhlazuje surová IMU data a odstraňuje vysokofrekvenční vibrace.
 *  Filtr běží na všech 6 osách, ale pro detekci apogea se primárně
 *  používá filtrovaná Z-osa akcelerometru.
 *
 *  Vzorec:  ema[n] = α · raw[n] + (1 − α) · ema[n−1]
 * ======================================================================= */
static float ema_ax = 0, ema_ay = 0, ema_az = 0;  /* Filtrovaný accel [LSB] */
static float ema_gx = 0, ema_gy = 0, ema_gz = 0;  /* Filtrovaný gyro  [LSB] */
static bool ema_initialized = false;               /* false = první vzorek   */

/* Aktualizuje EMA filtr na všech 6 osách. Při prvním volání inicializuje
 * stav filtru přímo na vstupní hodnotu (bez přechodového jevu). */
static void ema_update(const mpu6500_data_t *d) {
    if (!ema_initialized) {
        ema_ax = d->accel_x; ema_ay = d->accel_y; ema_az = d->accel_z;
        ema_gx = d->gyro_x;  ema_gy = d->gyro_y;  ema_gz = d->gyro_z;
        ema_initialized = true;
        return;
    }
    const float a = EMA_ALPHA;
    ema_ax = a * d->accel_x + (1 - a) * ema_ax;
    ema_ay = a * d->accel_y + (1 - a) * ema_ay;
    ema_az = a * d->accel_z + (1 - a) * ema_az;
    ema_gx = a * d->gyro_x  + (1 - a) * ema_gx;
    ema_gy = a * d->gyro_y  + (1 - a) * ema_gy;
    ema_gz = a * d->gyro_z  + (1 - a) * ema_gz;
}

/* =======================================================================
 *  Detekce fází letu a apogea
 *
 *  Stavový automat:
 *    FLIGHT_IDLE     – raketa stojí na rampě, měří se baseline az
 *    FLIGHT_LAUNCHED – tah motoru detekován, integruje se rychlost
 *    FLIGHT_APOGEE   – rychlost ≤ 0, padák aktivován
 *
 *  Rychlost se odhaduje integrací filtrovaného az (po odečtení
 *  klidové hodnoty / gravitace) metodou obdélníkové integrace.
 * ======================================================================= */
typedef enum {
    FLIGHT_IDLE,       /* Na rampě – čeká na start                       */
    FLIGHT_LAUNCHED,   /* Let – motor / coast fáze                        */
    FLIGHT_APOGEE,     /* Apogee detekováno – padák aktivován             */
} flight_state_t;

static flight_state_t flight_state = FLIGHT_IDLE;
static float baseline_az = 0;       /* Klidová hodnota az [LSB] (≈ +1g)     */
static float est_velocity = 0;      /* Odhadovaná svislá rychlost [m/s]     */
static uint32_t launch_detect_ms = 0;   /* Čas první detekce tahu [ms]      */
static uint32_t apogee_detect_ms = 0;   /* Čas první detekce apogee [ms]    */
static uint32_t flight_start_ms = 0;    /* Čas potvrzeného startu [ms]      */
static bool parachute_fired = false;    /* Padák byl aktivován (jednorázově) */
static uint32_t parachute_off_ms = 0;   /* Čas deaktivace padákového pinu   */

/*
 * Epoch čas – RP2040 nemá RTC s baterií, proto se čas nastavuje
 * přes USB příkaz "settime <epoch_sec>". Interně se počítá offset
 * vůči monotónnímu čítači (ms_since_boot).
 *
 *   epoch_ms = to_ms_since_boot() + epoch_offset_ms
 */
static int64_t epoch_offset_ms = 0;        /* Offset pro výpočet epoch času                */
static bool time_is_set = false;           /* true pokud byl čas nastaven přes settime      */
static uint64_t record_epoch_ms_start = 0; /* Epoch [ms] při startu záznamu                */

/* Vrátí aktuální epoch čas v milisekundách */
static uint64_t get_epoch_ms(void) {
    return (uint64_t)((int64_t)to_ms_since_boot(get_absolute_time()) + epoch_offset_ms);
}

/* Stránkový buffer – akumuluje záznamy v RAM a zapisuje po 256 B najednou.
 * Tím se minimalizuje počet SPI transakcí a sladí se s flash page write. */
static uint8_t page_buf[FLASH_PAGE_SIZE];
static uint16_t page_buf_pos = 0;

/* =======================================================================
 *  Pomocné funkce pro zápis na flash
 * ======================================================================= */

/* Zapíše aktuální stránkový buffer na flash. Zbylé místo doplní 0xFF.
 * Pokud je adresa na hranici sektoru (4 KB), provede nejprve sector erase. */
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

/* Přidá jeden záznam do stránkového bufferu.
 * Po naplnění 16 záznamů (256 B) automaticky zapíše na flash. */
static void write_record(const sample_record_t *rec) {
    memcpy(page_buf + page_buf_pos, rec, sizeof(sample_record_t));
    page_buf_pos += sizeof(sample_record_t);

    if (page_buf_pos >= FLASH_PAGE_SIZE) {
        flush_page_buf();
    }
}

/* Zapíše hlavičku záznamu na stránku 0 flash.
 * Volá se po ukončení záznamu – obsahuje finální počet vzorků
 * a parametry nastavení. */
static void write_header(void) {
    flash_header_t hdr;
    memset(&hdr, 0xFF, sizeof(hdr));
    hdr.magic = DATA_MAGIC;
    hdr.version = DATA_VERSION;
    hdr.sample_rate_hz = SAMPLE_RATE_HZ;
    hdr.num_samples = num_samples;
    hdr.accel_range = MPU_ACCEL_RANGE_G;
    hdr.gyro_range = MPU_GYRO_RANGE_DPS;
    hdr.epoch_ms_start = record_epoch_ms_start;

    // Erase first sector (contains header)
    w25q64_sector_erase(0);
    w25q64_page_program(0, (const uint8_t *)&hdr, sizeof(hdr));
}

/* =======================================================================
 *  Zpracování USB sériových příkazů
 *
 *  Příkazy se přijímají po řádcích (ukončené \r nebo \n).
 *  Podporované příkazy: dump, erase, status, start, stop, settime
 * ======================================================================= */

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
        printf("# Epoch start: %llu\n", hdr.epoch_ms_start);
        printf("epoch_ms,ax,ay,az,gx,gy,gz\n");

        uint32_t addr = FLASH_DATA_START;
        sample_record_t rec;
        for (uint32_t i = 0; i < hdr.num_samples; i++) {
            w25q64_read(addr, (uint8_t *)&rec, sizeof(rec));
            uint64_t epoch_ms = hdr.epoch_ms_start + (uint64_t)(rec.timestamp_us / 1000);
            printf("%llu,%d,%d,%d,%d,%d,%d\n",
                   epoch_ms,
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
        printf("Time set: %s\n", time_is_set ? "YES" : "NO");
        if (time_is_set) {
            printf("Current epoch ms: %llu\n", get_epoch_ms());
        }
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
            printf("START OK: recording requested\n");
        } else {
            printf("START OK: already recording\n");
        }

    } else if (strcmp(cmd, "stop") == 0) {
        if (recording) {
            recording = false;
            printf("Recording stopped via USB\n");
        }

    } else if (strlen(cmd) > 7 && strncmp(cmd, "settime ", 7) == 0) {
        /* Parsování epoch sekund – jednoduchý ASCII → uint64 převod */
        uint64_t epoch_sec = 0;
        const char *p = cmd + 7;
        while (*p >= '0' && *p <= '9') {
            epoch_sec = epoch_sec * 10 + (*p - '0');
            p++;
        }
        if (epoch_sec > 1000000000ULL) { // sanity: after 2001
            epoch_offset_ms = (int64_t)(epoch_sec * 1000ULL) - (int64_t)to_ms_since_boot(get_absolute_time());
            time_is_set = true;
            printf("SYNC OK: epoch_ms=%llu\n", get_epoch_ms());
        } else {
            printf("ERROR: Invalid epoch (use seconds since 1970)\n");
        }

    } else if (strlen(cmd) > 0) {
        printf("Commands: dump, erase, status, start, stop, settime <epoch_sec>\n");
    }
}

/* =======================================================================
 *  Obsluha tlačítka s debouncingem (300 ms)
 *
 *  Tlačítko je na GP15, active low s interním pull-up.
 *  Stisk přepíná stav záznamu (recording = !recording).
 *  Vlastní start/stop logika je v hlavní smyčce (state machine).
 * ======================================================================= */

static uint32_t last_button_time = 0;

static void button_callback(uint gpio, uint32_t events) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_button_time < 300) return;  /* Debounce – ignoruj opakované hrany */
    last_button_time = now;

    recording = !recording;
}

/* =======================================================================
 *  LED indikace stavů – WS2812B RGB NeoPixel (onboard GP16)
 *
 *  Stav se nastavuje přes proměnnou led_mode, aktualizace barvy/blikání
 *  probíhá voláním led_update() v hlavní smyčce.
 *
 *     LED_OFF          →  zhasnuto
 *     LED_ON           →  zelená (ready)
 *     LED_BLINK_SLOW   →  modrá, bliká 500 ms (záznam)
 *     LED_BLINK_FAST   →  červená, bliká 100 ms (chyba MPU-6500)
 *     LED_BLINK_FASTER →  žlutá, bliká 200 ms (chyba W25Q64)
 * ======================================================================= */

typedef enum {
    LED_OFF,          /* Vypnuto                                          */
    LED_ON,           /* Trvale zelená – připraveno ke startu             */
    LED_BLINK_SLOW,   /* Modrá, bliká 500 ms – probíhá záznam            */
    LED_BLINK_FAST,   /* Červená, bliká 100 ms – chyba inicializace MPU  */
    LED_BLINK_FASTER, /* Žlutá, bliká 200 ms – chyba inicializace flash  */
    LED_BLINK_APOGEE, /* Fialová, bliká 300 ms – apogee detekováno       */
} led_mode_t;

static led_mode_t led_mode = LED_OFF;       /* Aktuální režim LED       */
static uint32_t led_last_toggle_ms = 0;      /* Čas posledního přepnutí  */
static bool led_blink_on = false;            /* Stav blikání (on/off)    */

/* Vrátí barvu (GRB uint32) odpovídající aktuálnímu LED režimu */
static uint32_t led_mode_color(void) {
    switch (led_mode) {
        case LED_ON:           return ws2812_rgb(0, 40, 0);    // green
        case LED_BLINK_SLOW:   return ws2812_rgb(0, 0, 40);    // blue
        case LED_BLINK_FAST:   return ws2812_rgb(40, 0, 0);    // red
        case LED_BLINK_FASTER: return ws2812_rgb(40, 30, 0);   // yellow
        case LED_BLINK_APOGEE: return ws2812_rgb(40, 0, 40);   // purple
        default:               return 0;
    }
}

/* Aktualizuje stav LED – volat periodicky v hlavní smyčce.
 * Pro režimy ON/OFF nastaví barvu přímo, pro blikání přepíná
 * mezi barvou a zhasnutím podle zvoleného intervalu. */
static void led_update(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    uint32_t interval = 0;

    switch (led_mode) {
        case LED_OFF:
            ws2812_put_pixel(0);
            return;
        case LED_ON:
            ws2812_put_pixel(led_mode_color());
            return;
        case LED_BLINK_SLOW:
            interval = 500;
            break;
        case LED_BLINK_FAST:
            interval = 100;
            break;
        case LED_BLINK_FASTER:
            interval = 200;
            break;
        case LED_BLINK_APOGEE:
            interval = 300;
            break;
    }

    if (now_ms - led_last_toggle_ms >= interval) {
        led_last_toggle_ms = now_ms;
        led_blink_on = !led_blink_on;
        ws2812_put_pixel(led_blink_on ? led_mode_color() : 0);
    }
}

/* =======================================================================
 *  Hlavní funkce – main()
 *
 *  1. Inicializace periferií (LED, tlačítko, padák, MPU-6500, W25Q64)
 *  2. Čekání na USB připojení (2 s)
 *  3. Nekonečná smyčka:
 *     a) Příjem USB příkazů (neblokující čtení znaků)
 *     b) State machine záznamu – start/stop přechody
 *     c) Vzorkování IMU dat na 500 Hz s zápisem do flash
 *     d) Aktualizace LED indikace
 * ======================================================================= */

int main(void) {
    stdio_init_all();

    // WS2812B RGB LED
    ws2812_init(WS2812_PIN);
    ws2812_off();

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
        led_mode = LED_BLINK_FAST;
        while (1) { led_update(); }
    }
    printf("MPU-6500 OK (WHO_AM_I: 0x%02X)\n", mpu6500_who_am_i());

    // Init W25Q64
    if (!w25q64_init()) {
        printf("ERROR: W25Q64 init failed (JEDEC: 0x%06X)\n", w25q64_read_jedec_id());
        led_mode = LED_BLINK_FASTER;
        while (1) { led_update(); }
    }
    printf("W25Q64 OK (JEDEC: 0x%06X)\n", w25q64_read_jedec_id());

    // Ready – LED trvale svítí
    led_mode = LED_ON;

    printf("Ready. Press button or type 'start' to begin recording.\n");
    printf("Commands: dump, erase, status, start, stop, settime <epoch_sec>\n");
    if (!time_is_set) {
        printf("WARNING: Time not set. Use 'settime <epoch_sec>' to set clock.\n");
    }

    /* ===================================================================
     *  Hlavní smyčka
     *
     *  Běží bez RTOS – kooperativní multitasking:
     *  - USB čtení (neblokující getchar)
     *  - State machine záznamu (recording → was_recording přechody)
     *  - Vzorkování IMU s přesným 500 Hz timingem (time_us_64)
     *  - LED blikání
     * =================================================================== */
    char cmd_buf[64];              /* Buffer pro USB příkaz (max 63 znaků)  */
    uint8_t cmd_pos = 0;           /* Pozice v příkazovém bufferu           */
    bool was_recording = false;    /* Předchozí stav – detekce přechodů    */
    uint64_t record_start_us = 0;  /* Čas startu záznamu [µs]              */
    uint64_t next_sample_us = 0;   /* Čas příštího vzorku [µs]             */

    /* Kalibrace baseline – změříme klidovou hodnotu az ještě před
     * zahájením záznamu (raketa stojí na rampě, az ≈ +1g). */
    {
        printf("Calibrating baseline (hold still)...\n");
        float sum_az = 0;
        const int CAL_SAMPLES = 200; /* 200 vzorků = 0.4 s */
        for (int i = 0; i < CAL_SAMPLES; i++) {
            mpu6500_data_t cal;
            mpu6500_read_all(&cal);
            sum_az += cal.accel_z;
            sleep_ms(2);
        }
        baseline_az = sum_az / CAL_SAMPLES;
        printf("Baseline az = %.1f LSB (%.2f g)\n",
               baseline_az, baseline_az / ACCEL_LSB_PER_G);
    }

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
            record_epoch_ms_start = time_is_set ? get_epoch_ms() : 0;
            record_start_us = time_us_64();
            next_sample_us = record_start_us;
            was_recording = true;
            led_mode = LED_BLINK_SLOW;
            led_last_toggle_ms = to_ms_since_boot(get_absolute_time());
            led_blink_on = true;
            ws2812_put_pixel(led_mode_color());
            printf("LED: BLUE BLINK\n");

            /* Reset stavu letu a EMA filtru pro nový záznam */
            flight_state = FLIGHT_IDLE;
            est_velocity = 0;
            launch_detect_ms = 0;
            apogee_detect_ms = 0;
            flight_start_ms = 0;
            parachute_fired = false;
            ema_initialized = false;

            printf("Recording at %d Hz. Press button or 'stop' to end.\n", SAMPLE_RATE_HZ);
        }

        if (!recording && was_recording) {
            // Stop recording
            flush_page_buf();
            write_header();
            was_recording = false;
            printf("Recording stopped. %u samples saved.\n", num_samples);
            printf("Use 'dump' to download data as CSV.\n");
            led_mode = LED_ON;
            ws2812_put_pixel(led_mode_color());
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

                /* -- EMA filtr – vyhlazení surových dat --------------------- */
                ema_update(&mpu_data);

                /* -- Detekce fází letu (state machine) ---------------------- */
                uint32_t now_ms = to_ms_since_boot(get_absolute_time());
                float delta_az_g = (ema_az - baseline_az) / ACCEL_LSB_PER_G;
                const float dt = 1.0f / SAMPLE_RATE_HZ;

                switch (flight_state) {
                case FLIGHT_IDLE:
                    /* Čeká se na detekci tahu motoru.
                     * Podmínka: filtrované zrychlení > baseline + LAUNCH_ACCEL_G
                     * po dobu LAUNCH_CONFIRM_MS. */
                    if (delta_az_g > LAUNCH_ACCEL_G) {
                        if (launch_detect_ms == 0) launch_detect_ms = now_ms;
                        if (now_ms - launch_detect_ms >= LAUNCH_CONFIRM_MS) {
                            flight_state = FLIGHT_LAUNCHED;
                            flight_start_ms = now_ms;
                            est_velocity = 0;
                            printf("LAUNCH detected! (az=%.1fg)\n", delta_az_g);
                        }
                    } else {
                        launch_detect_ms = 0; /* Reset – nebyl trvalý tah */
                    }
                    break;

                case FLIGHT_LAUNCHED:
                    /* Integrace zrychlení → odhad rychlosti [m/s].
                     * delta_az_g je zrychlení v g nad klidovým stavem,
                     * převedeme na m/s² (* 9.81) a integrujeme (× dt). */
                    est_velocity += delta_az_g * 9.81f * dt;

                    /* Apogee detekce – rychlost klesla pod nulu.
                     * Bezpečnostní podmínky:
                     *   1. Min. doba letu (APOGEE_MIN_FLIGHT_MS)
                     *   2. Rychlost ≤ 0 po dobu APOGEE_VEL_CONFIRM_MS */
                    if (now_ms - flight_start_ms >= APOGEE_MIN_FLIGHT_MS) {
                        if (est_velocity <= 0) {
                            if (apogee_detect_ms == 0) apogee_detect_ms = now_ms;
                            if (now_ms - apogee_detect_ms >= APOGEE_VEL_CONFIRM_MS) {
                                flight_state = FLIGHT_APOGEE;
                                printf("APOGEE detected at T+%lu ms!\n",
                                       now_ms - flight_start_ms);

                                /* Aktivace padáku */
                                if (!parachute_fired) {
                                    gpio_put(PARACHUTE_PIN, 1);
                                    parachute_off_ms = now_ms + PARACHUTE_ACTIVE_MS;
                                    parachute_fired = true;
                                    led_mode = LED_BLINK_APOGEE;
                                    printf("PARACHUTE fired!\n");
                                }
                            }
                        } else {
                            apogee_detect_ms = 0; /* Reset – stále stoupá */
                        }
                    }
                    break;

                case FLIGHT_APOGEE:
                    /* Po detekci apogea – deaktivace padákového pinu po
                     * uplynutí PARACHUTE_ACTIVE_MS */
                    if (parachute_fired && now_ms >= parachute_off_ms) {
                        gpio_put(PARACHUTE_PIN, 0);
                    }
                    break;
                }

                /* -- Záznam surových dat na flash (nefiltrovaná!) ---------- */
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

        }

        /* Aktualizace LED ve všech stavech (blikání / statická barva) */
        led_update();
    }

    return 0;
}
