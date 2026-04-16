# Kaputnik – Flight Data Logger pro modelářské rakety

Flight data logger postavený na **RP2040** se snímačem **MPU-6500** (3-osý akcelerometr + gyroskop) a externí **W25Q64** SPI flash pamětí (64 Mbit / 8 MB).

Cílem projektu je zaznamenávat letová data modelu rakety a v budoucnu detekovat ztrátu tahu motoru / horní úvrať pro automatické vypuštění padáku.

## Hardware

| Komponenta | Popis |
|---|---|
| MCU | RP2040 (Raspberry Pi Pico) |
| IMU | MPU-6500 – 3-osý akcelerometr + gyroskop, SPI |
| Flash | Winbond W25Q64FVSSIQ – 64 Mbit SPI flash |
| Tlačítko | Start/stop záznamu (active low, interní pull-up) |
| LED | Onboard LED – indikuje stav přístroje |

### Zapojení pinů

| Periferie | SCK | MOSI | MISO | CS |
|---|---|---|---|---|
| MPU-6500 (SPI0) | GP2 | GP3 | GP4 | GP5 |
| W25Q64 (SPI1) | GP10 | GP11 | GP12 | GP13 |

| Funkce | Pin |
|---|---|
| Tlačítko | GP15 |
| LED | GP25 |
| Padák (budoucí) | GP16 |

## Funkce

- Po stisku tlačítka (nebo USB příkazu `start`) se spustí záznam dat z MPU-6500 na **500 Hz**
- Každý vzorek (16 bytů): timestamp (µs), 3× akcelerometr, 3× gyroskop (raw int16)
- Data se zapisují na externí W25Q64 flash (stránkovaný zápis po 256 B)
- Kapacita: cca **500 000 vzorků** (~16 minut záznamu)
- Opětovný stisk tlačítka nebo příkaz `stop` zastaví záznam
- Přes USB sériovou linku lze data stáhnout jako **CSV**

### USB příkazy

Připojte se sériovým terminálem (115200 baud):

| Příkaz | Akce |
|---|---|
| `start` | Spustí záznam |
| `stop` | Zastaví záznam |
| `dump` | Vypíše data jako CSV |
| `erase` | Smaže celou flash |
| `status` | Zobrazí stav zařízení |

### CSV formát výstupu

```
# KAPUTNIK Flight Data v1
# Sample rate: 500 Hz
# Samples: 12345
# Accel range: +/-16 g
# Gyro range: +/-2000 dps
timestamp_us,ax,ay,az,gx,gy,gz
0,123,-456,16384,10,-5,2
2000,125,-460,16380,12,-3,1
...
# END
```

## Nastavení MPU-6500

| Parametr | Hodnota |
|---|---|
| Rozsah akcelerometru | ±16 g |
| Rozsah gyroskopu | ±2000 °/s |
| DLPF bandwidth | ~92 Hz |
| Vzorkovací frekvence | 500 Hz |

## Build

Vyžaduje [Pico SDK](https://github.com/raspberrypi/pico-sdk) nainstalované v `~/pico/pico-sdk`.

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

Výstupem je `build/kaputnik.uf2`.

## Nahrání firmware

1. Podržte tlačítko **BOOTSEL** na RP2040
2. Připojte USB kabel
3. Nakopírujte `kaputnik.uf2` na USB disk, který se objeví

## Struktura projektu

```
src/
├── config.h      – definice pinů, parametry snímání
├── main.c        – hlavní logika (záznam, USB příkazy)
├── mpu6500.h/c   – SPI driver pro MPU-6500
└── w25q64.h/c    – SPI driver pro W25Q64 flash
```

## Plánovaný vývoj

- [ ] Detekce ztráty tahu raketového motoru
- [ ] Detekce horní úvrati (apogee)
- [ ] Automatické vypuštění padáku (GP16)
- [ ] Kalibrace akcelerometru před startem

## Licence

MIT
