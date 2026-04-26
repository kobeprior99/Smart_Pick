"""
compare_sensor_data.py
======================
Compare multiple CSV files containing sensor data (first 3200 samples each).

Expected CSV format (with or without a header row):
    timestamp_us, acceleration_lsb, capacitance_adc

Usage:
    python compare_sensor_data.py file1.csv file2.csv file3.csv ...
    python compare_sensor_data.py *.csv                        # glob works too
    python compare_sensor_data.py --dir ./data_folder          # scan a directory

Outputs:
    • Console: per-file & cross-file statistics table
    • Plots:   time-series overlay + histogram + correlation scatter (saved as PNG)
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.ticker import AutoMinorLocator

# ── constants ────────────────────────────────────────────────────────────────
MAX_SAMPLES   = 3200
COLUMNS       = ["timestamp_us", "acceleration_lsb", "capacitance_adc"]
PALETTE       = [
    "#00bcd4", "#ff6b35", "#8bc34a", "#ab47bc",
    "#ffc107", "#26a69a", "#ef5350", "#5c6bc0",
]

# ── helpers ──────────────────────────────────────────────────────────────────

def load_file(path: Path) -> pd.DataFrame:
    """Load up to MAX_SAMPLES rows from a CSV, auto-detecting the header."""
    raw = pd.read_csv(path, nrows=MAX_SAMPLES + 21, header=None, comment="#")

    # detect header: first row is a header when its first cell is not numeric
    try:
        float(raw.iloc[0, 0])
        raw.columns = COLUMNS[: raw.shape[1]]
    except (ValueError, TypeError):
        raw = pd.read_csv(path, nrows=MAX_SAMPLES + 20, comment="#")
        # normalise column names to our standard set
        raw.columns = COLUMNS[: raw.shape[1]]

    df = raw.iloc[20:MAX_SAMPLES + 20].copy().reset_index(drop=True)

    # ensure numeric types
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df["sample_index"] = df.index

    # 200-sample centred moving average of capacitance (min_periods=1 avoids NaN at edges)
    if "capacitance_adc" in df.columns:
        df["capacitance_ma200"] = (
            df["capacitance_adc"]
            .rolling(window=200, center=True, min_periods=1)
            .mean()
        )

    return df


def compute_stats(df: pd.DataFrame, label: str) -> dict:
    """Return a flat dict of descriptive statistics for one file."""
    stats = {"file": label, "n_samples": len(df)}
    for col in ["acceleration_lsb", "capacitance_adc", "capacitance_ma200"]:
        if col not in df.columns:
            continue
        s = df[col].dropna()
        prefix = col.split("_")[0][:5]          # 'accel' or 'capac'
        stats.update({
            f"{prefix}_mean":   s.mean(),
            f"{prefix}_std":    s.std(),
            f"{prefix}_min":    s.min(),
            f"{prefix}_max":    s.max(),
            f"{prefix}_p25":    s.quantile(0.25),
            f"{prefix}_p50":    s.quantile(0.50),
            f"{prefix}_p75":    s.quantile(0.75),
            f"{prefix}_rms":    np.sqrt(np.mean(s**2)),
            f"{prefix}_range":  s.max() - s.min(),
        })
    # timestamp sanity
    if "timestamp_us" in df.columns:
        ts = df["timestamp_us"].dropna()
        duration_ms = (ts.iloc[-1] - ts.iloc[0]) / 1_000 if len(ts) > 1 else 0
        stats["duration_ms"] = round(duration_ms, 3)
        stats["sample_rate_kHz"] = round(
            (len(ts) - 1) / (duration_ms * 1e-3) / 1_000, 3
        ) if duration_ms > 0 else float("nan")
    return stats


def print_stats_table(all_stats: list[dict]) -> None:
    df = pd.DataFrame(all_stats).set_index("file")
    pd.set_option("display.float_format", "{:.3f}".format)
    pd.set_option("display.max_columns", 30)
    pd.set_option("display.width", 200)
    print("\n" + "=" * 80)
    print("  SENSOR DATA COMPARISON  —  first {:,} samples per file".format(MAX_SAMPLES))
    print("=" * 80)
    print(df.T.to_string())
    print("=" * 80 + "\n")

    # cross-file delta
    if len(all_stats) >= 2:
        print("── Cross-file deltas (max − min across files) ──")
        numeric = df.select_dtypes(include="number")
        delta = numeric.max() - numeric.min()
        print(delta.to_string())
        print()


# ── plotting ─────────────────────────────────────────────────────────────────

def plot_all(datasets: list[tuple[str, pd.DataFrame]], out_path: Path) -> None:
    n = len(datasets)
    colors = (PALETTE * ((n // len(PALETTE)) + 1))[:n]

    fig = plt.figure(figsize=(18, 22), facecolor="#0d1117")
    fig.suptitle(
        "Sensor Data Comparison  ·  first {:,} samples".format(MAX_SAMPLES),
        fontsize=16, color="white", y=0.99, fontweight="bold"
    )

    gs = gridspec.GridSpec(
        4, 2, figure=fig,
        hspace=0.48, wspace=0.32,
        top=0.96, bottom=0.04, left=0.07, right=0.97
    )

    axes = {
        "accel_ts":    fig.add_subplot(gs[0, :]),   # full-width time series
        "cap_ts":      fig.add_subplot(gs[1, :]),
        "cap_mean":    fig.add_subplot(gs[2, :]),   # mean ± 1σ capacitance MA200
        "accel_fft":   fig.add_subplot(gs[3, :]),   # frequency spectrum overlay
    }

    style = dict(facecolor="#161b22", edgecolor="#30363d")
    for ax in axes.values():
        ax.set_facecolor(style["facecolor"])
        for spine in ax.spines.values():
            spine.set_edgecolor(style["edgecolor"])
        ax.tick_params(colors="#8b949e", labelsize=8)
        ax.xaxis.label.set_color("#8b949e")
        ax.yaxis.label.set_color("#8b949e")
        ax.title.set_color("#c9d1d9")
        ax.xaxis.set_minor_locator(AutoMinorLocator())
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.grid(which="major", color="#21262d", linewidth=0.7)
        ax.grid(which="minor", color="#161b22", linewidth=0.3)

    # ── time-series ──
    for (label, df), color in zip(datasets, colors):
        x = df["timestamp_us"] if "timestamp_us" in df.columns else df["sample_index"]
        x_label = "Timestamp (µs)" if "timestamp_us" in df.columns else "Sample index"

        if "acceleration_lsb" in df.columns:
            axes["accel_ts"].plot(
                x, df["acceleration_lsb"],
                color=color, linewidth=0.7, alpha=0.85, label=label
            )
        if "capacitance_ma200" in df.columns:
            axes["cap_ts"].plot(
                x, df["capacitance_ma200"],
                color=color, linewidth=1.8, alpha=0.9, label=label
            )

    axes["accel_ts"].set_title("Acceleration  [LSB]")
    axes["accel_ts"].set_xlabel(x_label)
    axes["accel_ts"].set_ylabel("Acceleration (LSB)")
    axes["accel_ts"].legend(
        fontsize=7, framealpha=0.3,
        facecolor="#0d1117", edgecolor="#30363d", labelcolor="white"
    )

    axes["cap_ts"].set_title("Capacitance MA200  [ADC counts]")
    axes["cap_ts"].set_xlabel(x_label)
    axes["cap_ts"].set_ylabel("Capacitance MA200 (ADC)")
    axes["cap_ts"].legend(
        fontsize=7, framealpha=0.3,
        facecolor="#0d1117", edgecolor="#30363d", labelcolor="white"
    )

    # ── mean ± 1σ capacitance across all trials (MA200) ──
    if "capacitance_ma200" in datasets[0][1].columns:
        ma200_arrays = [df["capacitance_ma200"].values for _, df in datasets
                        if "capacitance_ma200" in df.columns]
        min_len = min(len(a) for a in ma200_arrays)
        ma200_matrix = np.array([a[:min_len] for a in ma200_arrays], dtype=float)

        x_mean = np.arange(min_len)
        x_label_mean = "Sample index"
        ref_ts = datasets[0][1]["timestamp_us"].values[:min_len] if "timestamp_us" in datasets[0][1].columns else None
        if ref_ts is not None:
            x_mean = ref_ts
            x_label_mean = "Timestamp (µs)"

        mean_ma200 = np.mean(ma200_matrix, axis=0)
        std_ma200  = np.std(ma200_matrix, axis=0, ddof=1)

        ax_m = axes["cap_mean"]
        # individual MA200 trial traces (faint)
        for (label, df), color in zip(datasets, colors):
            if "capacitance_ma200" not in df.columns:
                continue
            ax_m.plot(
                x_mean, df["capacitance_ma200"].values[:min_len],
                color=color, linewidth=1.0, alpha=0.4, label=label
            )
        # ±1σ band around MA200 mean
        ax_m.fill_between(
            x_mean, mean_ma200 - std_ma200, mean_ma200 + std_ma200,
            color="#00bcd4", alpha=0.22, label="±1σ band"
        )
        # ±1σ edge lines
        ax_m.plot(x_mean, mean_ma200 + std_ma200, color="#00bcd4", linewidth=0.9,
                  linestyle="--", alpha=0.7, label="+1σ")
        ax_m.plot(x_mean, mean_ma200 - std_ma200, color="#00bcd4", linewidth=0.9,
                  linestyle="--", alpha=0.7, label="−1σ")
        # mean MA200 line on top
        ax_m.plot(
            x_mean, mean_ma200,
            color="#ffd740", linewidth=2.2, alpha=0.95, label="Mean MA200"
        )

        ax_m.set_title(
            f"Capacitance MA200  —  mean ± 1σ across {len(ma200_arrays)} trial(s)"
        )
        ax_m.set_xlabel(x_label_mean)
        ax_m.set_ylabel("Capacitance MA200 (ADC)")
        ax_m.legend(
            fontsize=7, framealpha=0.3,
            facecolor="#0d1117", edgecolor="#30363d", labelcolor="white"
        )

    # ── frequency spectrum of acceleration (FFT) ──
    ax_f = axes["accel_fft"]
    mean_psd = None
    fft_n = None

    for (label, df), color in zip(datasets, colors):
        if "acceleration_lsb" not in df.columns:
            continue
        sig = df["acceleration_lsb"].dropna().values
        n   = len(sig)

        # estimate sample rate from timestamps if available, else assume 1 Hz spacing
        if "timestamp_us" in df.columns:
            ts  = df["timestamp_us"].dropna().values
            dt  = np.mean(np.diff(ts)) * 1e-6          # seconds
            fs  = 1.0 / dt if dt > 0 else 1.0
        else:
            fs = 1.0

        # windowed FFT → single-sided power spectrum
        window    = np.hanning(n)
        fft_vals  = np.fft.rfft(sig * window)
        freqs     = np.fft.rfftfreq(n, d=1.0 / fs)
        power_db  = 20 * np.log10(np.abs(fft_vals) / n + 1e-12)

        ax_f.plot(freqs, power_db, color=color, linewidth=0.8, alpha=0.75, label=label)

        # accumulate for mean spectrum
        if mean_psd is None:
            mean_psd = np.abs(fft_vals) / n
            fft_freqs = freqs
        else:
            vals = np.abs(fft_vals) / n
            min_k = min(len(mean_psd), len(vals))
            mean_psd = mean_psd[:min_k] + vals[:min_k]
            fft_freqs = fft_freqs[:min_k]

        fft_n = (fft_n or 0) + 1

    # overlay mean spectrum in white
    if mean_psd is not None and fft_n:
        mean_db = 20 * np.log10(mean_psd / fft_n + 1e-12)
        ax_f.plot(fft_freqs, mean_db, color="#ffffff", linewidth=2.0,
                  alpha=0.95, label="Mean", zorder=10)

    ax_f.set_title("Acceleration frequency spectrum  [Hanning window, single-sided]")
    ax_f.set_xlabel("Frequency (Hz)")
    ax_f.set_ylabel("Magnitude (dB)")
    ax_f.legend(fontsize=7, framealpha=0.3,
                facecolor="#0d1117", edgecolor="#30363d", labelcolor="white")

    fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"  Plot saved → {out_path}")
    plt.close(fig)


# ── cross-correlation helper ──────────────────────────────────────────────────

def cross_correlate(datasets: list[tuple[str, pd.DataFrame]]) -> None:
    """Print normalised cross-correlation between every pair of files."""
    if len(datasets) < 2:
        return
    print("── Normalised cross-correlation (peak, lag) ──")
    for col, unit in [("acceleration_lsb", "LSB"), ("capacitance_ma200", "ADC MA200")]:
        print(f"\n  {col}:")
        for i in range(len(datasets)):
            for j in range(i + 1, len(datasets)):
                la, da = datasets[i]
                lb, db = datasets[j]
                if col not in da.columns or col not in db.columns:
                    continue
                a = da[col].dropna().values
                b = db[col].dropna().values
                # trim to same length
                n = min(len(a), len(b))
                a, b = a[:n], b[:n]
                a = (a - a.mean()) / (a.std() or 1)
                b = (b - b.mean()) / (b.std() or 1)
                xcorr = np.correlate(a, b, mode="full") / n
                lag   = np.argmax(np.abs(xcorr)) - (n - 1)
                peak  = xcorr[np.argmax(np.abs(xcorr))]
                print(f"    {la} ↔ {lb}:  peak={peak:+.4f},  lag={lag:+d} samples")
    print()


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Compare CSV sensor data files (first {:,} samples each).".format(MAX_SAMPLES)
    )
    parser.add_argument(
        "files", nargs="*", metavar="CSV",
        help="CSV files to compare"
    )
    parser.add_argument(
        "--dir", "-d", metavar="DIR",
        help="Directory to scan for *.csv files"
    )
    parser.add_argument(
        "--out", "-o", default="sensor_comparison.png",
        help="Output plot filename (default: sensor_comparison.png)"
    )
    parser.add_argument(
        "--no-plot", action="store_true",
        help="Skip generating the plot"
    )
    args = parser.parse_args()

    # ── collect file paths ──
    paths: list[Path] = []
    if args.dir:
        paths.extend(sorted(Path(args.dir).glob("*.csv")))
    for f in args.files:
        p = Path(f)
        if p.is_dir():
            paths.extend(sorted(p.glob("*.csv")))
        else:
            paths.append(p)

    if not paths:
        print(
            "No CSV files provided.\n"
            "Usage:\n"
            "  python compare_sensor_data.py file1.csv file2.csv ...\n"
            "  python compare_sensor_data.py --dir ./data_folder\n",
            file=sys.stderr
        )
        sys.exit(1)

    # ── load ──
    datasets: list[tuple[str, pd.DataFrame]] = []
    print(f"\nLoading {len(paths)} file(s)…")
    for p in paths:
        try:
            df = load_file(p)
            datasets.append((p.name, df))
            print(f"  ✓  {p.name}  ({len(df):,} samples)")
        except Exception as e:
            print(f"  ✗  {p.name}  — skipped ({e})", file=sys.stderr)

    if not datasets:
        print("No files could be loaded. Exiting.", file=sys.stderr)
        sys.exit(1)

    # ── stats ──
    all_stats = [compute_stats(df, label) for label, df in datasets]
    print_stats_table(all_stats)

    # ── cross-correlation ──
    cross_correlate(datasets)

    # ── plot ──
    if not args.no_plot:
        out_path = Path(args.out)
        plot_all(datasets, out_path)

    print("Done.\n")


if __name__ == "__main__":
    main()
