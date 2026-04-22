#!/usr/bin/env python3
import argparse
import re
import pandas as pd
import numpy as np


def main():
    parser = argparse.ArgumentParser(description="Analyze cam_sync_bench CSV output.")
    parser.add_argument("csv", nargs="?", default="/tmp/sync_bench.csv")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    # Auto-detect pair columns (delta_I_J_ms) — handles N=2,3,4 transparently.
    pair_cols = sorted(c for c in df.columns if re.match(r"delta_\d+_\d+_ms", c))
    if not pair_cols:
        # Legacy 2-topic format (t_a_ns / t_b_ns, no delta column)
        df["delta_0_1_ms"] = (df["t_a_ns"] - df["t_b_ns"]) / 1e6
        pair_cols = ["delta_0_1_ms"]

    # Max absolute spread across all pairs — the primary sync quality metric.
    df["max_spread_ms"] = df[pair_cols].abs().max(axis=1)

    print(f"\n{'─'*56}")
    print(f"  File   : {args.csv}")
    print(f"  Rows   : {len(df)}   Pairs: {len(pair_cols)}")
    print(f"{'─'*56}")

    for col in pair_cols:
        m = re.match(r"delta_(\d+)_(\d+)_ms", col)
        i, j = m.group(1), m.group(2)
        s = df[col]
        one_sided = (s > 0).mean()
        print(f"\n  Pair [{i},{j}]  signed delta = t_{i} − t_{j}")
        print(s.describe().to_string())
        print(f"  bias      : {s.mean():+.2f} ms  ({'A leads' if s.mean() > 0 else 'B leads'})")
        print(f"  jitter 1σ : {s.std():.2f} ms")
        print(f"  positive  : {one_sided*100:.1f}%  (≈100% or ≈0% → fixed pipeline bias)")

    print(f"\n{'─'*56}")
    print(f"  Max-pair |spread| percentiles [ms]  (budget = 34 ms)")
    for p in [50, 90, 95, 99]:
        val = np.percentile(df["max_spread_ms"], p)
        mark = "✓" if val < 34 else "✗"
        print(f"    p{p:<3}: {val:7.2f}  {mark}")
    print(f"    max : {df['max_spread_ms'].max():7.2f}")
    print(f"{'─'*56}")

    p99 = np.percentile(df["max_spread_ms"], 99)
    if p99 < 34:
        verdict = "WITHIN BUDGET (p99 < 34 ms)"
    elif p99 < 80:
        verdict = "OVER BUDGET — check USB topology or raise usbfs_memory_mb"
    else:
        verdict = "BROKEN — likely QoS mismatch or clock source issue"

    # Bias note only meaningful for a single dominant pair (N=2).
    bias_note = ""
    if len(pair_cols) == 1:
        s = df[pair_cols[0]]
        if abs(s.mean()) > 5 and (s > 0).mean() > 0.9:
            bias_note = (
                f"\n  NOTE: {abs(s.mean()):.1f} ms is fixed pipeline bias "
                f"(V4L2 kernel-arrival lag vs RealSense HW timestamp)."
                f"\n        True jitter = {s.std():.2f} ms stddev."
            )

    print(f"\n  Verdict: {verdict}{bias_note}\n")


if __name__ == "__main__":
    main()
