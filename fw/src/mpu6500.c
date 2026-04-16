/*
 * mpu6500.c – SPI driver pro InvenSense MPU-6500 IMU
 *
 * Komunikace probíhá přes SPI0 (piny definované v config.h).
 * SPI protokol MPU-6500:
 *   - Zápis: bit 7 = 0, bity 6:0 = adresa registru
 *   - Čtení: bit 7 = 1, bity 6:0 = adresa registru
 *   - CS (chip select) active low, manuálně řízený přes GPIO
 *
 * Inicializační sekvence:
 *   1. Reset (PWR_MGMT_1 = 0x80), čekání 100 ms
 *   2. Clock source = auto-select (PWR_MGMT_1 = 0x01)
 *   3. DLPF nastavení ~92 Hz pro accel i gyro
 *   4. Sample rate divider = 1 → 1 kHz / 2 = 500 Hz
 *   5. Accel ±16g, Gyro ±2000°/s
 *   6. Všechny osy povoleny (PWR_MGMT_2 = 0x00)
 */

#include "mpu6500.h"
#include "config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

/* -----------------------------------------------------------------------
 *  Registry MPU-6500 (pouze používané)
 * ----------------------------------------------------------------------- */
#define MPU_REG_SMPLRT_DIV      0x19  /* Sample rate divider                */
#define MPU_REG_CONFIG          0x1A  /* DLPF konfigurace (gyro)            */
#define MPU_REG_GYRO_CONFIG     0x1B  /* Rozsah gyroskopu                   */
#define MPU_REG_ACCEL_CONFIG    0x1C  /* Rozsah akcelerometru               */
#define MPU_REG_ACCEL_CONFIG2   0x1D  /* DLPF konfigurace (accel)           */
#define MPU_REG_ACCEL_XOUT_H   0x3B  /* Začátek bloku dat (14 bytů)        */
#define MPU_REG_TEMP_OUT_H      0x41  /* Teplota (high byte)                */
#define MPU_REG_GYRO_XOUT_H    0x43  /* Gyroskop X (high byte)             */
#define MPU_REG_PWR_MGMT_1     0x6B  /* Power management 1                 */
#define MPU_REG_PWR_MGMT_2     0x6C  /* Power management 2                 */
#define MPU_REG_WHO_AM_I       0x75  /* Identifikace čipu (0x70)           */

#define MPU_READ_FLAG  0x80           /* Bit 7 = 1 pro čtení registru      */

/* -----------------------------------------------------------------------
 *  Nízkoúrovňové SPI helpery
 * ----------------------------------------------------------------------- */

/* Aktivace CS – stahuje CS pin na low */
static inline void cs_select(void) {
    gpio_put(MPU_PIN_CS, 0);
    sleep_us(1);
}

/* Deaktivace CS – vrací CS pin na high */
static inline void cs_deselect(void) {
    sleep_us(1);
    gpio_put(MPU_PIN_CS, 1);
}

/* Zápis jednoho bytu do registru */
static void mpu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & 0x7F, val };
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, buf, 2);
    cs_deselect();
}

/* Čtení jednoho bytu z registru */
static uint8_t mpu_read_reg(uint8_t reg) {
    uint8_t tx = reg | MPU_READ_FLAG;
    uint8_t rx;
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, &tx, 1);
    spi_read_blocking(MPU_SPI_PORT, 0, &rx, 1);
    cs_deselect();
    return rx;
}

/* Burst čtení více bytů ze sekvenčních registrů */
static void mpu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len) {
    uint8_t tx = reg | MPU_READ_FLAG;
    cs_select();
    spi_write_blocking(MPU_SPI_PORT, &tx, 1);
    spi_read_blocking(MPU_SPI_PORT, 0, buf, len);
    cs_deselect();
}

/* -----------------------------------------------------------------------
 *  Veřejné API
 * ----------------------------------------------------------------------- */

bool mpu6500_init(void) {
    /* Inicializace SPI0 periferie a pinů */
    spi_init(MPU_SPI_PORT, MPU_SPI_FREQ);
    gpio_set_function(MPU_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(MPU_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MPU_PIN_MISO, GPIO_FUNC_SPI);

    // CS pin as GPIO output, deselect
    gpio_init(MPU_PIN_CS);
    gpio_set_dir(MPU_PIN_CS, GPIO_OUT);
    gpio_put(MPU_PIN_CS, 1);

    sleep_ms(100);

    /* Reset čipu – všechny registry na výchozí hodnoty */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);

    /* Probuzení, automatický výběr nejlepšího zdroje hodin */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    sleep_ms(10);

    /* Ověření identity čipu (WHO_AM_I = 0x70 pro MPU-6500) */
    uint8_t id = mpu6500_who_am_i();
    if (id != 0x70) {
        return false;
    }

    /* DLPF: bandwidth ~92 Hz – filtrace šumu, vhodné pro 500 Hz sampling */
    mpu_write_reg(MPU_REG_CONFIG, 0x02);
    mpu_write_reg(MPU_REG_ACCEL_CONFIG2, 0x02);

    /* Sample rate = 1 kHz / (1 + divider) = 500 Hz */
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 1);

    /* Rozsah akcelerometru: 3 = ±16g */
    mpu6500_set_accel_range(3);

    /* Rozsah gyroskopu: 3 = ±2000 °/s */
    mpu6500_set_gyro_range(3);

    /* Povoleni všech os akcelerometru i gyroskopu */
    mpu_write_reg(MPU_REG_PWR_MGMT_2, 0x00);

    return true;
}

uint8_t mpu6500_who_am_i(void) {
    return mpu_read_reg(MPU_REG_WHO_AM_I);
}

void mpu6500_read_all(mpu6500_data_t *data) {
    uint8_t buf[14];
    /* Burst čtení 14 bytů od registru ACCEL_XOUT_H:
     * [0..5]  = AXH AXL AYH AYL AZH AZL   (akcelerometr)
     * [6..7]  = TH TL                        (teplota)
     * [8..13] = GXH GXL GYH GYL GZH GZL    (gyroskop)
     *
     * Data jsou big-endian (MSB first). */
    mpu_read_regs(MPU_REG_ACCEL_XOUT_H, buf, 14);

    data->accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    data->temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
}

void mpu6500_set_accel_range(uint8_t range) {
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, (range & 0x03) << 3);
}

void mpu6500_set_gyro_range(uint8_t range) {
    mpu_write_reg(MPU_REG_GYRO_CONFIG, (range & 0x03) << 3);
}

void mpu6500_set_sample_rate_div(uint8_t divider) {
    mpu_write_reg(MPU_REG_SMPLRT_DIV, divider);
}
