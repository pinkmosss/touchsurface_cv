 #!/usr/bin/env python3
"""Analyze CSV logs captured by log_serial.py, per evaluation-protocol.md.

Each subcommand corresponds to one test and reads the CSV(s) written by
log_serial.py (no header row; first column is always the tag, e.g. "FPS").

Examples:
    # Test 1 - Frame Rate: one file per condition
    python3 analyze_logs.py fps test1_untouched.csv test1_touched.csv \\
        --labels untouched touched

    # Test 2 - Internal Processing Latency
    python3 analyze_logs.py latency test2.csv

    # Test 3 - Positional Accuracy: file:expected_tx,expected_rx per point
    python3 analyze_logs.py accuracy \\
        test3_tl.csv:0,0 test3_tr.csv:0,7 test3_bl.csv:7,0 test3_br.csv:7,7 test3_center.csv:3,3 \\
        --labels top-left top-right bottom-left bottom-right center

    # Test 4 - Hysteresis Effectiveness
    python3 analyze_logs.py hysteresis with_hysteresis.csv without_hysteresis.csv \\
        --labels "hysteresis on" "hysteresis off"

    # Test 5 - Size-Estimate Correlation
    python3 analyze_logs.py size fingertip.csv thumb.csv twofinger.csv \\
        --labels fingertip thumb two-finger

    # Test 6 - Recording / Playback Fidelity
    python3 analyze_logs.py playback --record record.csv --replay replay.csv

Requires: pandas (pip3 install pandas)
"""
import argparse
import csv
from pathlib import Path

import pandas as pd


def add_common_out_arg(p):
    p.add_argument("--out", help="Also write the summary table to this CSV path")


def maybe_save(df, args):
    if args.out:
        df.to_csv(args.out, index=False)
        print(f"\nSaved summary to {args.out}")


# --- Test 1: Frame Rate ---------------------------------------------------

def cmd_fps(args):
    labels = args.labels or [Path(f).stem for f in args.files]
    if len(labels) != len(args.files):
        raise SystemExit("--labels must have one entry per file")

    rows = []
    for f, label in zip(args.files, labels):
        df = pd.read_csv(f, header=None, names=["tag", "fps"])
        rows.append({
            "condition": label,
            "n_seconds": len(df),
            "mean_fps": df["fps"].mean(),
            "std_fps": df["fps"].std(),
            "min_fps": df["fps"].min(),
            "max_fps": df["fps"].max(),
        })

    out = pd.DataFrame(rows)
    print(out.to_string(index=False))
    maybe_save(out, args)


# --- Test 2: Internal Processing Latency -----------------------------------

def cmd_latency(args):
    df = pd.read_csv(args.file, header=None, names=["tag", "latency_us"])
    s = df["latency_us"].sort_values().reset_index(drop=True)

    print(f"n      = {len(s)}")
    print(f"mean   = {s.mean():.1f} us")
    print(f"median = {s.median():.1f} us")
    print(f"std    = {s.std():.1f} us")
    print(f"max    = {s.max()} us")

    # The protocol anticipated two clusters (DAC write vs. cache-skipped
    # frames), but real captures can show more tiers — e.g. an active-touch
    # cost (extra 3x3 neighbor scan) and a periodic control-pad ADC poll
    # (ENABLE_CONTROL_PADS, CONTROL_PAD_SAMPLE_INTERVAL_MS) independent of
    # DAC writes. Report the actual shape via a histogram rather than
    # forcing a single low/high split that can hide real intermediate
    # clusters.
    bin_width = args.bin_width
    binned = (s // bin_width * bin_width).value_counts().sort_index()
    print(f"\nHistogram ({bin_width} us bins):")
    max_count = binned.max()
    for edge, count in binned.items():
        bar_len = int(count / max_count * 50)
        pct = count / len(s) * 100
        print(f"  {edge:5d}-{edge + bin_width - 1:<5d} us: {count:4d} ({pct:4.1f}%) " + "#" * bar_len)

    if args.out:
        s.to_frame(name="latency_us_sorted").to_csv(args.out, index=False)
        print(f"\nSaved sorted samples to {args.out}")


# --- Test 3: Positional Accuracy and Repeatability -------------------------

def parse_point_arg(token):
    # "path/to/file.csv:tx,rx"
    file_part, coord_part = token.rsplit(":", 1)
    tx_str, rx_str = coord_part.split(",")
    return file_part, int(tx_str), int(rx_str)


def cmd_accuracy(args):
    labels = args.labels or [Path(parse_point_arg(t)[0]).stem for t in args.points]
    if len(labels) != len(args.points):
        raise SystemExit("--labels must have one entry per point")

    rows = []
    all_touch_errs = []
    for token, label in zip(args.points, labels):
        f, exp_tx, exp_rx = parse_point_arg(token)
        df = pd.read_csv(f, header=None, names=["tag", "tx", "rx"])
        mean_tx, mean_rx = df["tx"].mean(), df["rx"].mean()
        err_tx = mean_tx - exp_tx
        err_rx = mean_rx - exp_rx

        # Per-touch distance from the true point, computed per sample before
        # any averaging — unlike the centroid-offset ("bias") below, this
        # cannot hide error through cancellation: two touches that land on
        # opposite sides of the true point both count fully here, even
        # though their *mean* position might sit right on top of it.
        touch_errs = ((df["tx"] - exp_tx) ** 2 + (df["rx"] - exp_rx) ** 2) ** 0.5
        all_touch_errs.extend(touch_errs.tolist())

        rows.append({
            "point": label,
            "n_touches": len(df),
            "expected_tx": exp_tx,
            "expected_rx": exp_rx,
            "mean_tx": round(mean_tx, 2),
            "std_tx": round(df["tx"].std(), 2),
            "mean_rx": round(mean_rx, 2),
            "std_rx": round(df["rx"].std(), 2),
            "bias_tx_cells": round(err_tx, 2),
            "bias_rx_cells": round(err_rx, 2),
            "bias_euclidean_cells": round((err_tx ** 2 + err_rx ** 2) ** 0.5, 2),
            "per_touch_mean_abs_cells": round(touch_errs.mean(), 2),
            "per_touch_rmse_cells": round((touch_errs ** 2).mean() ** 0.5, 2),
        })

    out = pd.DataFrame(rows)
    print(out.to_string(index=False))
    print(f"\nMean centroid bias across points (systematic offset, per-touch "
          f"errors can cancel in the average): {out['bias_euclidean_cells'].mean():.2f} cells")
    pooled = pd.Series(all_touch_errs)
    print(f"Pooled per-touch RMSE across all {len(pooled)} touches (typical "
          f"single-touch error, bias + spread): {(pooled**2).mean()**0.5:.2f} cells")
    maybe_save(out, args)


# --- Test 4: Hysteresis Effectiveness --------------------------------------

def cmd_hysteresis(args):
    labels = args.labels or [Path(f).stem for f in args.files]
    if len(labels) != len(args.files):
        raise SystemExit("--labels must have one entry per file")

    rows = []
    for f, label in zip(args.files, labels):
        df = pd.read_csv(f, header=None, names=["tag", "ts_ms", "state"])
        duration_s = (df["ts_ms"].max() - df["ts_ms"].min()) / 1000 if len(df) else 0
        rows.append({
            "condition": label,
            "n_transitions": len(df),
            "duration_s": round(duration_s, 2),
            "transitions_per_s": round(len(df) / duration_s, 2) if duration_s else float("nan"),
        })

    out = pd.DataFrame(rows)
    print(out.to_string(index=False))
    if len(out) == 2:
        reduction = 1 - (out["n_transitions"].iloc[0] / out["n_transitions"].iloc[1])
        print(f"\n{out['condition'].iloc[0]!r} vs {out['condition'].iloc[1]!r}: "
              f"{reduction:.0%} fewer transitions")
    maybe_save(out, args)


# --- Test 5: Size-Estimate Correlation --------------------------------------

def cmd_size(args):
    labels = args.labels or [Path(f).stem for f in args.files]
    if len(labels) != len(args.files):
        raise SystemExit("--labels must have one entry per file")

    rows = []
    for f, label in zip(args.files, labels):
        df = pd.read_csv(f, header=None, names=["tag", "tx", "rx", "size"])
        rows.append({
            "condition": label,
            "n_trials": len(df),
            "mean_size": df["size"].mean(),
            "std_size": df["size"].std(),
            "min_size": df["size"].min(),
            "max_size": df["size"].max(),
        })

    out = pd.DataFrame(rows)
    print(out.to_string(index=False))

    means = out.set_index("condition")["mean_size"]
    is_monotonic = means.is_monotonic_increasing or means.is_monotonic_decreasing
    ordering = " < ".join(f"{c}={means[c]:.2f}" for c in means.index)
    print(f"\nMonotonic in the order given? {'yes' if is_monotonic else 'no'} ({ordering})")
    maybe_save(out, args)


# --- Test 6: Recording / Playback Fidelity ----------------------------------

def read_tagged_rows(path):
    with open(path, newline="") as f:
        return [row for row in csv.reader(f) if row]


def parse_record_rows(rows):
    parsed = []
    for r in rows:
        if r[1] == "NONE":
            parsed.append({"tx": None, "rx": None})
        else:
            parsed.append({"tx": int(r[1]), "rx": int(r[2])})
    return parsed


def parse_replay_rows(rows):
    parsed = []
    for r in rows:
        ts_ms = int(r[1])
        if r[2] == "NONE":
            parsed.append({"ts_ms": ts_ms, "tx": None, "rx": None})
        else:
            parsed.append({"ts_ms": ts_ms, "tx": int(r[2]), "rx": int(r[3])})
    return parsed


def cmd_playback(args):
    record_samples = parse_record_rows(read_tagged_rows(args.record))
    replay_samples = parse_replay_rows(read_tagged_rows(args.replay))

    print(f"Recorded samples: {len(record_samples)}")
    print(f"Replayed samples: {len(replay_samples)}")
    if len(record_samples) != len(replay_samples):
        print(f"WARNING: length mismatch ({len(record_samples)} recorded vs "
              f"{len(replay_samples)} replayed)")

    n = min(len(record_samples), len(replay_samples))
    mismatches = [
        (i, record_samples[i], replay_samples[i])
        for i in range(n)
        if (record_samples[i]["tx"], record_samples[i]["rx"])
        != (replay_samples[i]["tx"], replay_samples[i]["rx"])
    ]

    if mismatches:
        print(f"\n{len(mismatches)}/{n} mismatched sample(s):")
        for i, rec, rep in mismatches[:20]:
            print(f"  step {i}: recorded=({rec['tx']},{rec['rx']}) replayed=({rep['tx']},{rep['rx']})")
        if len(mismatches) > 20:
            print(f"  ... and {len(mismatches) - 20} more")
    else:
        print(f"\nAll {n} compared samples matched exactly.")

    ts = pd.Series([r["ts_ms"] for r in replay_samples])
    deltas = ts.diff().dropna()
    print("\nReplay step interval (ms) — short gaps follow a manual trigger edge;")
    print("~500 ms gaps are the fallback timer advancing on its own:")
    print(deltas.describe().to_string())

    if args.out:
        pd.DataFrame({
            "step": range(n),
            "record_tx": [record_samples[i]["tx"] for i in range(n)],
            "record_rx": [record_samples[i]["rx"] for i in range(n)],
            "replay_tx": [replay_samples[i]["tx"] for i in range(n)],
            "replay_rx": [replay_samples[i]["rx"] for i in range(n)],
            "match": [
                (record_samples[i]["tx"], record_samples[i]["rx"])
                == (replay_samples[i]["tx"], replay_samples[i]["rx"])
                for i in range(n)
            ],
        }).to_csv(args.out, index=False)
        print(f"\nSaved step-by-step comparison to {args.out}")


def build_parser():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("fps", help="Test 1 - Frame Rate")
    p.add_argument("files", nargs="+", help="FPS,<count> log(s), one per condition")
    p.add_argument("--labels", nargs="+", help="Condition label per file, same order as files")
    add_common_out_arg(p)
    p.set_defaults(func=cmd_fps)

    p = sub.add_parser("latency", help="Test 2 - Internal Processing Latency")
    p.add_argument("file", help="LATENCY_US,<us> log")
    p.add_argument("--bin-width", type=int, default=50, help="Histogram bin width in us (default: 50)")
    add_common_out_arg(p)
    p.set_defaults(func=cmd_latency)

    p = sub.add_parser("accuracy", help="Test 3 - Positional Accuracy and Repeatability")
    p.add_argument("points", nargs="+", help="file.csv:expected_tx,expected_rx, one per reference point")
    p.add_argument("--labels", nargs="+", help="Point label per entry, same order as points")
    add_common_out_arg(p)
    p.set_defaults(func=cmd_accuracy)

    p = sub.add_parser("hysteresis", help="Test 4 - Hysteresis Effectiveness")
    p.add_argument("files", nargs="+", help="TRANSITION,<ms>,<ACTIVE|INACTIVE> log(s), one per condition")
    p.add_argument("--labels", nargs="+", help="Condition label per file, same order as files")
    add_common_out_arg(p)
    p.set_defaults(func=cmd_hysteresis)

    p = sub.add_parser("size", help="Test 5 - Size-Estimate Correlation")
    p.add_argument("files", nargs="+", help="SIZE,<tx>,<rx>,<size> log(s), one per touch condition")
    p.add_argument("--labels", nargs="+", help="Condition label per file, same order as files")
    add_common_out_arg(p)
    p.set_defaults(func=cmd_size)

    p = sub.add_parser("playback", help="Test 6 - Recording / Playback Fidelity")
    p.add_argument("--record", required=True, help="RECORD_SAMPLE,... log")
    p.add_argument("--replay", required=True, help="REPLAY_SAMPLE,... log")
    add_common_out_arg(p)
    p.set_defaults(func=cmd_playback)

    return parser


def main():
    args = build_parser().parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
