# Kaputnik – Flight Data Logger pro modelářské rakety

Flight data logger postavený na **RP2040** se snímačem **MPU-6500** (3-osý akcelerometr + gyroskop) a externí **W25Q64** SPI flash pamětí (64 Mbit / 8 MB).

Cílem projektu je zaznamenávat letová data modelu rakety, detekovat horní úvrať (apogee) a automaticky aktivovat padák.

## Hardware

| Komponenta | Popis |
|---|---|
| MCU | RP2040 (Waveshare RP2040-Zero) |
| IMU | MPU-6500 – 3-osý akcelerometr + gyroskop, SPI |
| Flash | Winbond W25Q64FVSSIQ – 64 Mbit SPI flash |
| Tlačítko | Start/stop záznamu (active low, interní pull-up) |
| LED | WS2812B RGB NeoPixel (onboard) – barevná indikace stavu |

### Zapojení pinů

| Periferie | SCK | MOSI | MISO | CS |
|---|---|---|---|---|
| MPU-6500 (SPI0) | GP2 | GP3 | GP4 | GP5 |
| W25Q64 (SPI1) | GP10 | GP11 | GP12 | GP13 |

| Funkce | Pin |
|---|---|
| Tlačítko | GP15 |
| WS2812B RGB LED | GP16 (onboard) |
| Padák (výstup) | GP14 |

## Funkce

- Po stisku tlačítka (nebo USB příkazu `start`) se spustí záznam dat z MPU-6500 na **500 Hz**
- Každý vzorek (16 bytů): timestamp (µs), 3× akcelerometr, 3× gyroskop (raw int16)
- Data se zapisují na externí W25Q64 flash (stránkovaný zápis po 256 B)
- Kapacita: cca **500 000 vzorků** (~16 minut záznamu)
- Opětovný stisk tlačítka nebo příkaz `stop` zastaví záznam
- Přes USB sériovou linku lze data stáhnout jako **CSV**
- **EMA filtr** vyhlazuje data v reálném čase pro detekci fází letu
- **Automatická detekce apogea** a aktivace padáku na GP14

### LED indikace (WS2812B RGB)

| Barva | Režim | Stav |
|---|---|---|
| 🟢 Zelená | Trvale svítí | Připraveno (ready) |
| 🔵 Modrá | Bliká pomalu (500 ms) | Probíhá záznam |
| � Fialová | Bliká (300 ms) | Apogee detekováno, padák aktivován |
| �🔴 Červená | Bliká rychle (100 ms) | Chyba MPU-6500 |
| 🟡 Žlutá | Bliká rychleji (200 ms) | Chyba W25Q64 flash |

### USB příkazy

Připojte se sériovým terminálem (115200 baud):

| Příkaz | Akce |
|---|---|
| `start` | Spustí záznam |
| `stop` | Zastaví záznam |
| `dump` | Vypíše data jako CSV |
| `erase` | Smaže celou flash |
| `status` | Zobrazí stav zařízení |
| `settime <epoch_sec>` | Nastaví vnitřní hodiny (epoch sekundy) |

### CSV formát výstupu

```
# KAPUTNIK Flight Data v2
# Sample rate: 500 Hz
# Samples: 12345
# Accel range: +/-16 g
# Gyro range: +/-2000 dps
# Epoch start: 1776556800000
epoch_ms,ax,ay,az,gx,gy,gz
1776556800000,123,-456,16384,10,-5,2
1776556800002,125,-460,16380,12,-3,1
...
# END
```

Každý záznam obsahuje absolutní časový otisk `epoch_ms` (milisekundy od 1.1.1970). Vyžaduje synchronizaci hodin před záznamem.

## EMA filtr – vyhlazování dat pro detekci apogea

### Problém: vibrace rakety

Surová data z IMU za letu obsahují silné vysokofrekvenční vibrace:
- **Vibrace motoru** (stovky Hz) – mechanický přenos přes trup rakety
- **Aerodynamické turbulence** – proudění kolem těla rakety
- **Strukturální rezonance** – vlastní kmity trupu a žeber

Tyto vibrace znemožňují přímé použití raw dat pro spolehlivou detekci fází letu (start, apogee, pád). Potřebujeme z dat extrahovat **pomalý trend** – skutečný směr a velikost zrychlení – a potlačit šum.

### Řešení: Exponenciální klouzavý průměr (EMA)

**EMA (Exponential Moving Average)** je jednoduchý digitální filtr typu dolní propust (low-pass). Na rozdíl od klasického klouzavého průměru (SMA) nepotřebuje buffer starých vzorků – stačí jediná proměnná na osu.

#### Vzorec

```
ema[n] = α · raw[n] + (1 − α) · ema[n−1]
```

Kde:
- `raw[n]` – aktuální surový vzorek z IMU
- `ema[n−1]` – předchozí filtrovaná hodnota
- `α` (alpha) – vyhlazovací koeficient, 0 < α < 1

#### Jak α ovlivňuje chování

| α | Chování | Mezní frekvence (při 500 Hz) |
|---|---|---|
| 0.5 | Téměř žádné vyhlazení | ~53 Hz |
| 0.1 | Mírné vyhlazení | ~8 Hz |
| **0.02** | **Silné vyhlazení (použito)** | **~1.6 Hz** |
| 0.01 | Velmi silné vyhlazení | ~0.8 Hz |
| 0.001 | Extrémní vyhlazení | ~0.08 Hz |

Mezní frekvenci lze přibližně spočítat jako:

```
f_c = (α × f_vzork) / (2π)
f_c = (0.02 × 500) / (2π) ≈ 1.59 Hz
```

#### Příklad – průběh letu

```
Raw az:       ████ ▓▓▓░░▒▒▒████▓▓░░  ← chaotické vibrace
Filtr (EMA):  ─────╱‾‾‾‾‾╲──────     ← hladký průběh
                   ↑       ↑
                 start   apogee
```

#### Proč EMA a ne jiný filtr?

| Filtr | RAM | CPU | Vhodnost |
|---|---|---|---|
| **EMA** | **1 float/osa** | **1 násobení** | ✅ Ideální pro MCU |
| SMA (klouzavý průměr) | N floatů/osa | N sčítání | ❌ Spotřebuje RAM |
| Butterworth | Řád × 2 floaty | Řád × operace | ⚠️ Složitější |
| Kalman | Matice stavů | Maticové operace | ⚠️ Pro odhad orientace |

EMA je optimální volba pro RP2040 – minimální paměťová i výpočetní náročnost.

### Implementace v Kaputniku

Filtr běží na **všech 6 osách** (3× accel + 3× gyro) při každém vzorku (500×/s):

```c
static float ema_az = 0;  // filtrovaná hodnota osy Z

void ema_update(int16_t raw_az) {
    ema_az = 0.02f * raw_az + 0.98f * ema_az;
}
```

Při prvním vzorku se EMA inicializuje přímo na vstupní hodnotu (bez přechodového jevu).

**Důležité:** Na flash se ukládají **surová (nefiltrovaná) data**. EMA se používá pouze pro real-time detekci letu. Díky tomu máte v CSV kompletní informace pro pozdější analýzu.

## Detekce apogea a padák

### Stavový automat letu

Firmware obsahuje stavový automat se třemi fázemi:

```
  IDLE ──────► LAUNCHED ──────► APOGEE
  (rampa)       (let)        (padák)
```

#### 1. IDLE – čekání na startu

- Raketa stojí na rampě
- Při spuštění záznamu se změří **baseline** hodnota os Z akcelerometru (200 vzorků ≈ 0.4 s)
- Baseline ≈ +1g (klid na zemi, gravitace)
- Čeká se na překročení prahu startu

#### 2. LAUNCHED – detekce startu a integrace rychlosti

**Podmínka startu:**
- Filtrované zrychlení az > baseline + **3g** (konfigurovatelné)
- Musí trvat minimálně **50 ms** (ochrana proti falešné detekci)

**Odhad rychlosti:**
- Numerická integrace filtrovaného zrychlení (po odečtení baseline/gravitace)
- `rychlost += (az_filtr − baseline) / 2048 × 9.81 × dt`
- Kde dt = 1/500 = 0.002 s

#### 3. APOGEE – detekce horní úvrati

**Podmínky detekce:**
1. Minimální doba letu **500 ms** (ochrana proti falešné detekci při startu)
2. Odhadovaná rychlost klesne **pod nulu**
3. Rychlost zůstane ≤ 0 po dobu **50 ms** (potvrzení)

**Akce při detekci apogea:**
- GPIO pin GP14 se nastaví na HIGH na **1 sekundu** (puls pro aktivaci padáku)
- LED změní barvu na **fialovou** (blikání 300 ms)
- Informace se vypíše na USB (`APOGEE detected`, `PARACHUTE fired`)
- Padák se aktivuje pouze **jednou** za záznam (bezpečnostní pojistka)

### Konfigurovatelné parametry

Všechny parametry detekce jsou v `fw/src/config.h`:

| Konstanta | Výchozí | Popis |
|---|---|---|
| `EMA_ALPHA` | 0.02 | Vyhlazovací koeficient EMA filtru |
| `LAUNCH_ACCEL_G` | 3.0 | Práh detekce startu [g nad klidem] |
| `LAUNCH_CONFIRM_MS` | 50 | Potvrzení startu – minimální doba [ms] |
| `APOGEE_MIN_FLIGHT_MS` | 500 | Minimální doba letu před detekcí apogea [ms] |
| `APOGEE_VEL_CONFIRM_MS` | 50 | Rychlost ≤ 0 po tuto dobu → apogee [ms] |
| `PARACHUTE_ACTIVE_MS` | 1000 | Doba aktivace padákového pinu [ms] |
| `ACCEL_LSB_PER_G` | 2048 | Převodní konstanta LSB/g při ±16g |

## Nastavení MPU-6500

| Parametr | Hodnota |
|---|---|
| Rozsah akcelerometru | ±16 g |
| Rozsah gyroskopu | ±2000 °/s |
| DLPF bandwidth | ~92 Hz (hardwarový low-pass v čipu) |
| Vzorkovací frekvence | 500 Hz |

> **Poznámka:** DLPF (Digital Low-Pass Filter) v MPU-6500 na ~92 Hz je **první stupeň filtrace** přímo v hardware čipu. EMA filtr ve firmware je **druhý stupeň**, který dále vyhladí data pro detekci apogea.

## Build

### Firmware (fw/)

Vyžaduje [Pico SDK](https://github.com/raspberrypi/pico-sdk) nainstalované v `~/pico/pico-sdk` a ARM toolchain (`arm-none-eabi-gcc`).

```bash
cd fw
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

Výstupem je `fw/build/kaputnik.uf2`.

### PC aplikace (sw/)

Vyžaduje [Rust](https://rustup.rs/) toolchain.

```bash
cd sw
cargo build --release
```

Výstupem je `sw/target/release/kaputnik-downloader`.

## Nahrání firmware

1. Podržte tlačítko **BOOTSEL** na RP2040
2. Připojte USB kabel
3. Nakopírujte `fw/build/kaputnik.uf2` na USB disk, který se objeví

## PC aplikace – kaputnik-downloader

Rust CLI nástroj pro komunikaci s Kaputnikem přes USB sériovou linku.

```bash
# Výpis dostupných sériových portů
kaputnik-downloader list

# Stav zařízení
kaputnik-downloader -p /dev/ttyACM0 status

# Stáhnutí dat do CSV souboru
kaputnik-downloader -p /dev/ttyACM0 dump -o flight.csv

# Stáhnutí dat na stdout
kaputnik-downloader -p /dev/ttyACM0 dump

# Spuštění / zastavení záznamu
kaputnik-downloader -p /dev/ttyACM0 start
kaputnik-downloader -p /dev/ttyACM0 stop

# Smazání flash
kaputnik-downloader -p /dev/ttyACM0 erase

# Synchronizace hodin (nastaví čas zařízení na aktuální čas PC)
kaputnik-downloader -p /dev/ttyACM0 sync
```

Výchozí port je `/dev/ttyACM0`, lze změnit přepínačem `-p`.

### Typický workflow před letem

```bash
# 1. Synchronizace hodin
kaputnik-downloader sync

# 2. Spuštění záznamu (nebo tlačítkem na desce)
kaputnik-downloader start

# 3. ...let...

# 4. Zastavení záznamu (nebo tlačítkem)
kaputnik-downloader stop

# 5. Stažení dat
kaputnik-downloader dump -o flight.csv
```

## Struktura projektu

```
kaputnik/
├── fw/                          ← firmware pro RP2040
│   ├── CMakeLists.txt
│   ├── pico_sdk_import.cmake
│   └── src/
│       ├── config.h             – definice pinů, parametry snímání
│       ├── main.c               – hlavní logika (záznam, USB příkazy, LED)
│       ├── mpu6500.h/c          – SPI driver pro MPU-6500
│       ├── w25q64.h/c           – SPI driver pro W25Q64 flash
│       ├── ws2812.h/c           – PIO driver pro WS2812B RGB LED
│       └── ws2812.pio           – PIO program pro WS2812B protokol
├── sw/                          ← PC aplikace (Rust)
│   ├── Cargo.toml
│   └── src/main.rs              – CLI downloader
├── .gitignore
└── README.md
```

## Plánovaný vývoj

- [x] Záznam 6-osých IMU dat na flash (500 Hz)
- [x] USB příkazy pro ovládání a stahování dat
- [x] WS2812B RGB LED indikace stavů
- [x] Epoch timestamps (synchronizace hodin z PC)
- [x] EMA filtr pro vyhlazení dat
- [x] Detekce startu a horní úvrati (apogee)
- [x] Automatická aktivace padáku (GP14)
- [x] Kalibrace baseline akcelerometru při startu
- [ ] Vizualizace dat (grafy v PC aplikaci)
- [ ] Detekce orientace (gyro integrace)
- [ ] Logovací mode – filtrovaná data do CSV

## Odkazy
https://www.waveshare.com/wiki/RP2040-Zero

https://www.laskakit.cz/user/related_files/mpu-6500-datasheet2.pdf

https://www.laskakit.cz/user/related_files/winbond-elec-w25q64fv.pdf

## Licence

MIT
