"""
generate_sample_data.py – Generátor vzorových letových dat pro Kaputnik

Simuluje realistický let modelářské rakety s těmito fázemi:
  1. Pad (rampa)    – ~0.5 s – raketa stojí, az ≈ +1g
  2. Boost (tah)    – ~1.5 s – motor hoří, az ≈ 5–10g
  3. Coast (výběh)  – ~2.0 s – motor vyhasl, gravitace brzdí
  4. Apogee (úvrať) – okamžik – rychlost = 0
  5. Descent (pád)  – ~2.0 s – padák, az ≈ +1g s oscilacemi

Výstup: CSV ve formátu shodném s firmware příkazem "dump".
"""

import math
import random
import sys

# === Konfigurace ===
SAMPLE_RATE_HZ = 500
DT = 1.0 / SAMPLE_RATE_HZ  # 2 ms
ACCEL_LSB_PER_G = 2048.0    # ±16g → 32768/16 = 2048 LSB/g
GYRO_LSB_PER_DPS = 16.384   # ±2000°/s → 32768/2000

# Epoch start – fiktivní datum: 2026-07-04 14:00:00 UTC (Independence Day launch!)
EPOCH_START_MS = 1783350000000

# === Fáze letu (čas v sekundách) ===
T_PAD = 0.5       # Čekání na rampě
T_BOOST = 1.5     # Fáze tahu motoru
T_COAST = 2.5     # Výběh (po vyhoření motoru do apogea)
T_DESCENT = 2.0   # Sestup pod padákem
T_TOTAL = T_PAD + T_BOOST + T_COAST + T_DESCENT

NUM_SAMPLES = int(T_TOTAL * SAMPLE_RATE_HZ)


def clamp_i16(val):
    """Omezí hodnotu na rozsah int16 (-32768..32767)."""
    return max(-32768, min(32767, int(round(val))))


def add_noise(base, sigma):
    """Přidá gaussovský šum k hodnotě."""
    return base + random.gauss(0, sigma)


def add_vibration(t, amplitude, freq_range=(80, 400)):
    """Simuluje mechanické vibrace motoru/trupu (náhodné frekvence)."""
    vib = 0
    for f in [127, 213, 340, 89, 178, 267, 310]:
        vib += amplitude * math.sin(2 * math.pi * f * t + random.uniform(0, 2 * math.pi))
    return vib * 0.3


def generate_flight_data():
    samples = []
    random.seed(42)

    # Stav rakety
    velocity = 0.0  # m/s (svislá rychlost)
    altitude = 0.0  # m

    for i in range(NUM_SAMPLES):
        t = i * DT  # Čas od začátku záznamu [s]
        timestamp_us = int(t * 1_000_000)

        # === Fyzikální model (zrychlení v g) ===
        if t < T_PAD:
            # --- Rampa: raketa stojí ---
            accel_z_g = 1.0  # Gravitace (+1g ve svislé ose)
            accel_x_g = 0.0
            accel_y_g = 0.0
            gyro_x_dps = 0.0
            gyro_y_dps = 0.0
            gyro_z_dps = 0.0
            noise_a = 15    # Malý šum senzoru
            noise_g = 5
            vib_amp = 0

        elif t < T_PAD + T_BOOST:
            # --- Boost: motor hoří ---
            t_boost = t - T_PAD
            # Profil tahu: rychlý nástup, pak postupný pokles
            thrust_profile = 8.0 * math.exp(-0.3 * t_boost) * (1 - math.exp(-10 * t_boost))
            accel_z_g = 1.0 + thrust_profile  # Gravitace + tah
            accel_x_g = 0.1 * math.sin(3 * t_boost)  # Mírné příčné zrychlení
            accel_y_g = 0.05 * math.cos(5 * t_boost)
            # Gyroskop – lehká rotace kolem podélné osy (spin stabilizace)
            gyro_z_dps = 120 + 30 * math.sin(2 * t_boost)
            gyro_x_dps = 5 * math.sin(10 * t_boost)
            gyro_y_dps = 3 * math.cos(8 * t_boost)
            noise_a = 80    # Silné vibrace motoru
            noise_g = 30
            vib_amp = 200   # Vibrace struktury

            velocity += (accel_z_g - 1.0) * 9.81 * DT

        elif t < T_PAD + T_BOOST + T_COAST:
            # --- Coast: motor vyhasl, raketa zpomaluje ---
            t_coast = t - T_PAD - T_BOOST
            # Zrychlení klesá (gravitace + aerodynamický odpor)
            drag_decel = 0.3 * (velocity / 50) ** 2 if velocity > 0 else 0
            accel_z_g = 1.0 - drag_decel * 0.1  # Pod 1g (brzdění)
            # Blízko apogea klesá k ~0g (volný pád)
            if velocity < 5:
                accel_z_g = max(0.05, accel_z_g * (velocity / 5))

            accel_x_g = 0.02 * math.sin(2 * t_coast)
            accel_y_g = 0.01 * math.cos(3 * t_coast)
            gyro_z_dps = 120 * math.exp(-0.5 * t_coast)  # Spin se zpomaluje
            gyro_x_dps = 2 * math.sin(5 * t_coast)
            gyro_y_dps = 1.5 * math.cos(4 * t_coast)
            noise_a = 30    # Méně vibrací (žádný motor)
            noise_g = 15
            vib_amp = 30    # Aerodynamický šum

            velocity -= (1.0 - accel_z_g + drag_decel * 0.05) * 9.81 * DT
            velocity = max(-5, velocity)  # Omezení pro simulaci

        else:
            # --- Descent: padák otevřen ---
            t_desc = t - T_PAD - T_BOOST - T_COAST
            # Kývání pod padákem
            swing = math.sin(2 * math.pi * 0.8 * t_desc) * math.exp(-0.3 * t_desc)
            accel_z_g = 1.0 + 0.3 * swing  # ~1g s oscilací
            accel_x_g = 0.4 * swing
            accel_y_g = 0.3 * math.cos(2 * math.pi * 0.6 * t_desc) * math.exp(-0.2 * t_desc)
            gyro_x_dps = 50 * swing
            gyro_y_dps = 40 * math.cos(2 * math.pi * 0.6 * t_desc)
            gyro_z_dps = 10 * math.sin(2 * math.pi * 1.2 * t_desc)
            noise_a = 25
            noise_g = 10
            vib_amp = 10
            velocity = -3.0  # Ustálená rychlost sestupu

        altitude += velocity * DT

        # === Převod na raw LSB hodnoty ===
        ax = clamp_i16(add_noise(accel_x_g * ACCEL_LSB_PER_G, noise_a)
                       + add_vibration(t, vib_amp))
        ay = clamp_i16(add_noise(accel_y_g * ACCEL_LSB_PER_G, noise_a)
                       + add_vibration(t + 0.3, vib_amp * 0.8))
        az = clamp_i16(add_noise(accel_z_g * ACCEL_LSB_PER_G, noise_a)
                       + add_vibration(t + 0.7, vib_amp * 0.6))
        gx = clamp_i16(add_noise(gyro_x_dps * GYRO_LSB_PER_DPS, noise_g))
        gy = clamp_i16(add_noise(gyro_y_dps * GYRO_LSB_PER_DPS, noise_g))
        gz = clamp_i16(add_noise(gyro_z_dps * GYRO_LSB_PER_DPS, noise_g))

        epoch_ms = EPOCH_START_MS + timestamp_us // 1000

        samples.append((epoch_ms, ax, ay, az, gx, gy, gz))

    return samples


def main():
    output_file = sys.argv[1] if len(sys.argv) > 1 else None

    samples = generate_flight_data()

    lines = []
    lines.append(f"# KAPUTNIK Flight Data v2")
    lines.append(f"# Sample rate: {SAMPLE_RATE_HZ} Hz")
    lines.append(f"# Samples: {len(samples)}")
    lines.append(f"# Accel range: +/-16 g")
    lines.append(f"# Gyro range: +/-2000 dps")
    lines.append(f"# Epoch start: {EPOCH_START_MS}")
    lines.append(f"epoch_ms,ax,ay,az,gx,gy,gz")

    for s in samples:
        lines.append(f"{s[0]},{s[1]},{s[2]},{s[3]},{s[4]},{s[5]},{s[6]}")

    lines.append("# END")

    text = "\n".join(lines) + "\n"

    if output_file:
        with open(output_file, "w") as f:
            f.write(text)
        print(f"Generated {len(samples)} samples → {output_file}", file=sys.stderr)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
