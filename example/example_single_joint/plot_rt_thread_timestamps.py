#!/usr/bin/env python3
import argparse
import csv
import statistics
from pathlib import Path

import matplotlib.pyplot as plt


DEFAULT_CSV = "../../build/rt_thread_timestamps.csv"
PERIOD_NS = 5_000_000


def read_timestamps(csv_path):
    timestamps = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            timestamps.append(int(row["time_ns"]))
    return timestamps


def calc_jitter_ms(timestamps):
    jitter_ms = []
    for i in range(1, len(timestamps)):
        jitter_ns = timestamps[i] - timestamps[i - 1] - PERIOD_NS
        jitter_ms.append(round(abs(jitter_ns) / 1_000_000.0, 6))
    return jitter_ms


def main():
    parser = argparse.ArgumentParser(description="Plot realtime thread jitter.")
    parser.add_argument("--csv", default=DEFAULT_CSV, help="timestamp CSV path")
    parser.add_argument("--out", default="", help="optional output image path")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if not csv_path.is_absolute():
        csv_path = Path(__file__).resolve().parent / csv_path
    timestamps = read_timestamps(csv_path)
    if len(timestamps) < 2:
        raise RuntimeError("need at least 2 timestamps")

    jitter_ms = calc_jitter_ms(timestamps)
    x = list(range(1, len(timestamps)))

    print(f"samples={len(timestamps)}")
    min_jitter_ms = min(jitter_ms)
    max_jitter_ms = max(jitter_ms)
    avg_jitter_ms = statistics.mean(jitter_ms)
    print(f"abs_jitter_ms min={min_jitter_ms:.6f}")
    print(f"abs_jitter_ms max={max_jitter_ms:.6f}")
    print(f"abs_jitter_ms avg={avg_jitter_ms:.6f}")

    plt.figure(figsize=(12, 5))
    plt.plot(x, jitter_ms, linewidth=0.8)
    plt.axhline(avg_jitter_ms, color="red", linestyle="--", linewidth=0.8,
                label=f"avg = {avg_jitter_ms:.6f} ms")
    plt.xlabel("sample index")
    plt.ylabel("abs jitter (ms)")
    plt.title(
        "Realtime Thread Absolute Clock Jitter, period = 5 ms\n"
        f"min={min_jitter_ms:.6f} ms, max={max_jitter_ms:.6f} ms, "
        f"avg={avg_jitter_ms:.6f} ms"
    )
    plt.grid(True, linewidth=0.3)
    plt.legend()
    plt.tight_layout()

    if args.out:
        plt.savefig(args.out, dpi=150)
        print(f"saved={args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
