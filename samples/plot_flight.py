"""
plot_flight.py – Vizualizace letových dat Kaputnik

Vykreslí 4 panely:
  1. Akcelerometr (ax, ay, az) – raw LSB + druhá osa v g
  2. Gyroskop (gx, gy, gz) – raw LSB + druhá osa v °/s
  3. Celkové zrychlení |a| s EMA filtrem a detekcí fází letu
  4. Odhadovaná rychlost a výška (numerická integrace)

Použití:
  py plot_flight.py sample_flight.csv
  py plot_flight.py sample_flight.csv -o flight_plot.png
"""

import csv
import math
import sys
import argparse

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import FuncFormatter

# === Konstanty (shodné s firmware) ===
ACCEL_LSB_PER_G = 2048.0
GYRO_LSB_PER_DPS = 16.384
EMA_ALPHA = 0.02
LAUNCH_ACCEL_G = 3.0


def parse_csv(path):
    """Načte Kaputnik CSV a vrátí seznamy hodnot (časy v milisekundách)."""
    epoch_ms, ax, ay, az, gx, gy, gz = [], [], [], [], [], [], []
    is_epoch_us = False

    with open(path, "r") as f:
        reader = csv.reader(f)
        for row_idx, row in enumerate(reader):
            if not row:
                continue
            
            # Skip firmware header comments
            if row[0].startswith("#"):
                continue
            
            # Detect header and determine time format
            if row[0] in ("epoch_ms", "epoch_us"):
                is_epoch_us = (row[0] == "epoch_us")
                continue
            
            if row[0] == "# END":
                break
            
            try:
                time_val = int(row[0])
                # Convert microseconds to milliseconds if needed
                if is_epoch_us:
                    time_val = time_val // 1000
                epoch_ms.append(time_val)
                ax.append(int(row[1]))
                ay.append(int(row[2]))
                az.append(int(row[3]))
                gx.append(int(row[4]))
                gy.append(int(row[5]))
                gz.append(int(row[6]))
            except (ValueError, IndexError) as e:
                print(f"Warning: skipped malformed row {row_idx}: {row}", file=sys.stderr)
                continue

    return epoch_ms, ax, ay, az, gx, gy, gz


def ema_filter(data, alpha=EMA_ALPHA):
    """Exponenciální klouzavý průměr."""
    out = [data[0]]
    for i in range(1, len(data)):
        out.append(alpha * data[i] + (1 - alpha) * out[-1])
    return out


def detect_phases(t_s, az_g_ema):
    """Detekuje fáze letu ze EMA filtrovaného az. Vrátí indexy přechodů."""
    baseline = az_g_ema[0]
    launched_idx = None
    apogee_idx = None
    velocity = 0.0

    for i in range(1, len(az_g_ema)):
        dt = t_s[i] - t_s[i - 1]

        if launched_idx is None:
            if az_g_ema[i] > baseline + LAUNCH_ACCEL_G:
                launched_idx = i
        else:
            net_accel_g = az_g_ema[i] - baseline
            velocity += net_accel_g * 9.81 * dt
            if apogee_idx is None and velocity <= 0 and (t_s[i] - t_s[launched_idx]) > 0.5:
                apogee_idx = i

    return launched_idx, apogee_idx


def integrate_velocity_altitude(t_s, az_g_ema, launch_idx):
    """Numerická integrace filtrovaného zrychlení → rychlost a výška."""
    baseline = az_g_ema[0]
    vel = [0.0] * len(t_s)
    alt = [0.0] * len(t_s)

    for i in range(1, len(t_s)):
        dt = t_s[i] - t_s[i - 1]
        if launch_idx is not None and i >= launch_idx:
            net_g = az_g_ema[i] - baseline
            vel[i] = vel[i - 1] + net_g * 9.81 * dt
            alt[i] = alt[i - 1] + vel[i] * dt
        else:
            vel[i] = 0
            alt[i] = 0

    return vel, alt


def main():
    parser = argparse.ArgumentParser(description="Kaputnik flight data plotter")
    parser.add_argument("csv", nargs="?", default="sample_flight.csv", help="CSV soubor s letovými daty (výchozí: sample_flight.csv)")
    parser.add_argument("-o", "--output", help="Uložit graf do souboru (PNG/SVG/PDF)")
    args = parser.parse_args()

    try:
        epoch_ms, ax, ay, az, gx, gy, gz = parse_csv(args.csv)
    except FileNotFoundError:
        print(f"Error: CSV soubor '{args.csv}' nenalezen.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error při načítání CSV: {e}", file=sys.stderr)
        sys.exit(1)
    
    n = len(epoch_ms)
    if n == 0:
        print("Error: Žádná data v CSV souboru.", file=sys.stderr)
        sys.exit(1)

    # Čas v sekundách od začátku záznamu (epoch_ms je v ms, takže /1000)
    t0 = epoch_ms[0]
    t_s = [(e - t0) / 1000.0 for e in epoch_ms]
    
    print(f"Loaded {n} samples, duration {t_s[-1]:.2f} s", file=sys.stderr)

    # Převod na fyzikální jednotky
    ax_g = [v / ACCEL_LSB_PER_G for v in ax]
    ay_g = [v / ACCEL_LSB_PER_G for v in ay]
    az_g = [v / ACCEL_LSB_PER_G for v in az]
    gx_dps = [v / GYRO_LSB_PER_DPS for v in gx]
    gy_dps = [v / GYRO_LSB_PER_DPS for v in gy]
    gz_dps = [v / GYRO_LSB_PER_DPS for v in gz]

    # Celkové zrychlení
    a_total_g = [math.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(ax_g, ay_g, az_g)]

    # EMA filtr
    az_ema = ema_filter(az_g)
    a_total_ema = ema_filter(a_total_g)

    # Detekce fází
    launch_idx, apogee_idx = detect_phases(t_s, az_ema)

    # Integrace rychlosti a výšky
    vel, alt = integrate_velocity_altitude(t_s, az_ema, launch_idx)

    # === Vykreslení ===
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle("KAPUTNIK – Letová data", fontsize=15, fontweight="bold")

    colors = {"x": "#e74c3c", "y": "#2ecc71", "z": "#3498db"}

    def shade_phases(ax_obj):
        """Vybarví pozadí podle fází letu."""
        if launch_idx is not None:
            ax_obj.axvspan(0, t_s[launch_idx], alpha=0.08, color="green", label="_pad")
            if apogee_idx is not None:
                ax_obj.axvspan(t_s[launch_idx], t_s[apogee_idx], alpha=0.08, color="orange")
                ax_obj.axvspan(t_s[apogee_idx], t_s[-1], alpha=0.08, color="blue")
                ax_obj.axvline(t_s[apogee_idx], color="purple", ls="--", lw=1.5, alpha=0.7)

    # --- Panel 1: Akcelerometr ---
    ax1 = axes[0]
    shade_phases(ax1)
    ax1.plot(t_s, ax_g, color=colors["x"], alpha=0.5, lw=0.4, label="ax")
    ax1.plot(t_s, ay_g, color=colors["y"], alpha=0.5, lw=0.4, label="ay")
    ax1.plot(t_s, az_g, color=colors["z"], alpha=0.5, lw=0.4, label="az (raw)")
    ax1.plot(t_s, az_ema, color="#1a1a2e", lw=1.8, label="az (EMA)")
    ax1.set_ylabel("Zrychlení [g]")
    ax1.legend(loc="upper right", fontsize=8, ncol=4)
    ax1.set_title("Akcelerometr", fontsize=11)
    ax1.grid(True, alpha=0.3)

    # --- Panel 2: Gyroskop ---
    ax2 = axes[1]
    shade_phases(ax2)
    ax2.plot(t_s, gx_dps, color=colors["x"], alpha=0.6, lw=0.5, label="gx")
    ax2.plot(t_s, gy_dps, color=colors["y"], alpha=0.6, lw=0.5, label="gy")
    ax2.plot(t_s, gz_dps, color=colors["z"], alpha=0.6, lw=0.5, label="gz")
    ax2.set_ylabel("Úhlová rychlost [°/s]")
    ax2.legend(loc="upper right", fontsize=8, ncol=3)
    ax2.set_title("Gyroskop", fontsize=11)
    ax2.grid(True, alpha=0.3)

    # --- Panel 3: Celkové zrychlení + fáze ---
    ax3 = axes[2]
    shade_phases(ax3)
    ax3.plot(t_s, a_total_g, color="#bdc3c7", alpha=0.4, lw=0.4, label="|a| raw")
    ax3.plot(t_s, a_total_ema, color="#e74c3c", lw=2, label="|a| EMA")
    ax3.axhline(1.0, color="gray", ls=":", lw=1, alpha=0.5, label="1g")
    if launch_idx:
        ax3.axhline(1.0 + LAUNCH_ACCEL_G, color="orange", ls=":", lw=1, alpha=0.5,
                     label=f"Práh startu ({1+LAUNCH_ACCEL_G:.0f}g)")
    ax3.set_ylabel("|zrychlení| [g]")
    ax3.legend(loc="upper right", fontsize=8, ncol=4)
    ax3.set_title("Celkové zrychlení a detekce fází", fontsize=11)
    ax3.grid(True, alpha=0.3)

    # --- Panel 4: Rychlost a výška ---
    ax4 = axes[3]
    shade_phases(ax4)
    ln1 = ax4.plot(t_s, vel, color="#e67e22", lw=1.5, label="Rychlost [m/s]")
    ax4.set_ylabel("Rychlost [m/s]", color="#e67e22")
    ax4.tick_params(axis="y", labelcolor="#e67e22")

    ax4b = ax4.twinx()
    ln2 = ax4b.plot(t_s, alt, color="#8e44ad", lw=1.5, label="Výška [m]")
    ax4b.set_ylabel("Výška [m]", color="#8e44ad")
    ax4b.tick_params(axis="y", labelcolor="#8e44ad")

    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    ax4.legend(lns, labs, loc="upper right", fontsize=8, ncol=2)
    ax4.set_title("Odhadovaná rychlost a výška (integrace EMA)", fontsize=11)
    ax4.grid(True, alpha=0.3)

    ax4.set_xlabel("Čas [s]")

    # Legendy fází
    phase_patches = [
        mpatches.Patch(color="green", alpha=0.15, label="Rampa"),
        mpatches.Patch(color="orange", alpha=0.15, label="Let (boost + coast)"),
        mpatches.Patch(color="blue", alpha=0.15, label="Sestup (padák)"),
    ]
    if apogee_idx:
        phase_patches.append(plt.Line2D([0], [0], color="purple", ls="--", lw=1.5, label="Apogee"))
    fig.legend(handles=phase_patches, loc="lower center", ncol=4, fontsize=9,
               framealpha=0.9, edgecolor="gray")

    plt.tight_layout(rect=[0, 0.04, 1, 0.97])

    if args.output:
        fig.savefig(args.output, dpi=150, bbox_inches="tight")
        print(f"Saved → {args.output}", file=sys.stderr)
    else:
        plt.show()


if __name__ == "__main__":
    main()
