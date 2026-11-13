#!/usr/bin/env python3
"""Capture and plot CubliV2 telemetry ($T lines) or a `log` dump ($L lines).

Capture from the USB console (needs pyserial):
    python3 tools/plot_telemetry.py --port /dev/ttyUSB0 --seconds 10 --out run.csv
    (in the console beforehand: `tlm 200`)
Plot a saved file (needs matplotlib):
    python3 tools/plot_telemetry.py run.csv

$T,t_ms,state,brake,th0_deg,th1_deg,rate0,rate1,w1,w2,w3,u1,u2,u3,V,A,exec_us
$L,i,dt,theta0,theta1,rate0,rate1,wheel_angle0,wheel_angle1,u0,u1
"""
import argparse
import sys
import time

T_COLS = ["t_ms", "state", "brake", "th0_deg", "th1_deg", "rate0", "rate1", "w1", "w2", "w3",
          "u1", "u2", "u3", "V", "A", "exec_us"]
L_COLS = ["i", "dt", "theta0", "theta1", "rate0", "rate1", "wheel_angle0", "wheel_angle1", "u0", "u1"]


def capture(port, baud, seconds, out):
    import serial  # pyserial

    with serial.Serial(port, baud, timeout=0.1, dsrdtr=False, rtscts=False) as ser, open(out, "w") as f:
        ser.dtr = False  # DTR/RTS drive EN/IO0 through the auto-reset circuit
        ser.rts = False
        end = time.time() + seconds
        n = 0
        while time.time() < end:
            line = ser.readline().decode(errors="replace").strip()
            if line.startswith("$T,") or line.startswith("$L,"):
                f.write(line + "\n")
                n += 1
        print(f"{n} lines -> {out}")


def load(path):
    rows_t, rows_l = [], []
    for line in open(path):
        parts = line.strip().split(",")
        try:
            if parts[0] == "$T" and len(parts) == len(T_COLS) + 1:
                rows_t.append([float(x) for x in parts[1:]])
            elif parts[0] == "$L" and len(parts) == len(L_COLS) + 1:
                rows_l.append([float(x) for x in parts[1:]])
        except ValueError:
            pass
    return rows_t, rows_l


def plot(path):
    import matplotlib.pyplot as plt

    rows_t, rows_l = load(path)
    if rows_t:
        col = {n: [r[i] for r in rows_t] for i, n in enumerate(T_COLS)}
        t = [(x - col["t_ms"][0]) / 1000 for x in col["t_ms"]]
        fig, ax = plt.subplots(4, 1, sharex=True, figsize=(10, 9))
        ax[0].plot(t, col["th0_deg"], label="tilt axis 0")
        ax[0].plot(t, col["th1_deg"], label="tilt axis 1")
        ax[0].set_ylabel("deg")
        ax[1].plot(t, col["rate0"], label="rate 0")
        ax[1].plot(t, col["rate1"], label="rate 1")
        ax[1].set_ylabel("rad/s")
        for k in (1, 2, 3):
            ax[2].plot(t, col[f"w{k}"], label=f"wheel {k}")
            ax[3].plot(t, col[f"u{k}"], label=f"cmd {k}")
        ax[2].set_ylabel("rad/s")
        ax[3].set_ylabel("command")
        ax[3].set_xlabel("s")
    elif rows_l:
        col = {n: [r[i] for r in rows_l] for i, n in enumerate(L_COLS)}
        t, acc = [], 0.0
        for d in col["dt"]:
            acc += d
            t.append(acc)
        fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
        ax[0].plot(t, col["theta0"], label="theta 0")
        ax[0].plot(t, col["theta1"], label="theta 1")
        ax[1].plot(t, col["wheel_angle0"], label="wheel angle 0")
        ax[1].plot(t, col["wheel_angle1"], label="wheel angle 1")
        ax[2].plot(t, col["u0"], label="u 0")
        ax[2].plot(t, col["u1"], label="u 1")
        ax[2].set_xlabel("s")
    else:
        sys.exit("no $T or $L lines found")
    for a in ax:
        a.grid(True)
        a.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument("--out", default="telemetry.csv")
    a = ap.parse_args()
    if a.port:
        capture(a.port, a.baud, a.seconds, a.out)
    elif a.file:
        plot(a.file)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
