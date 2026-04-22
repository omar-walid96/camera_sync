#!/usr/bin/env python3
import argparse
import pandas as pd
import numpy as np

def main():
    parser = argparse.ArgumentParser(description="Analyze cam_sync_bench CSV output.")
    parser.add_argument("csv", nargs="?", default="/tmp/sync_bench.csv")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    # delta_ms is already signed (t_a - t_b) if present; recompute from raw ns otherwise
    if "delta_ms" in df.columns:
        df["signed_ms"] = df["delta_ms"]
    else:
        df["signed_ms"] = (df["t_a_ns"] - df["t_b_ns"]) / 1e6

    df["spread_ms"] = df["signed_ms"].abs()

    n = len(df)
    bias   = df["signed_ms"].mean()
    jitter = df["signed_ms"].std()
    one_sided = (df["signed_ms"] > 0).mean()   # fraction positive

    print(f"\n{'─'*52}")
    print(f"  File   : {args.csv}")
    print(f"  Rows   : {n}")
    print(f"{'─'*52}")
    print(f"  signed delta (t_a − t_b)  [ms]")
    print(df["signed_ms"].describe().to_string())
    print(f"{'─'*52}")
    print(f"  bias        : {bias:+.2f} ms  ({'A leads B' if bias > 0 else 'B leads A'})")
    print(f"  jitter 1σ   : {jitter:.2f} ms")
    print(f"  positive    : {one_sided*100:.1f}%  (100% or 0% → pure bias)")
    print(f"{'─'*52}")
    print(f"  |spread| percentiles [ms]")
    for p in [50, 90, 95, 99]:
        val = np.percentile(df["spread_ms"], p)
        budget = "✓" if val < 34 else "✗"
        print(f"    p{p:<3}: {val:6.2f}  {budget}")
    print(f"    max : {df['spread_ms'].max():6.2f}")
    print(f"{'─'*52}")

    # Verdict
    p99 = np.percentile(df["spread_ms"], 99)
    if p99 < 34:
        verdict = "WITHIN BUDGET (p99 < 34 ms)"
    elif p99 < 80:
        verdict = "OVER BUDGET — check USB topology or raise usbfs_memory_mb"
    else:
        verdict = "BROKEN — likely QoS mismatch or clock source issue"

    bias_note = ""
    if abs(bias) > 5 and one_sided > 0.9:
        bias_note = f"\n  NOTE: {abs(bias):.1f} ms is fixed pipeline bias (not fixable by software sync)."
        bias_note += f"\n        Real jitter = {jitter:.2f} ms stddev."

    print(f"\n  Verdict: {verdict}{bias_note}\n")

if __name__ == "__main__":
    main()
