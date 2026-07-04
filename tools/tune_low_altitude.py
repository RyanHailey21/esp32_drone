#!/usr/bin/env python3
"""Low-altitude altitude-hold analysis from ESP32 flight logs.

This is not a full plant simulator. It extracts what the logs can support:
response delay, altitude turning points, and how candidate gains would have
commanded throttle against the recorded altitude/vario stream.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from pathlib import Path


def discover_latest(log_dir: Path) -> Path:
    logs = sorted(log_dir.glob("quad-flight-log-*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not logs:
        raise SystemExit(f"No logs found in {log_dir}")
    return logs[0]


def parse_run(line: str) -> dict[str, float]:
    out: dict[str, float] = {}
    for key, value in re.findall(r"(\w+)=(-?\d+(?:\.\d+)?)", line):
        out[key] = float(value)
    return out


def load_log(path: Path):
    run = ""
    header = None
    rows = []
    time_offset = 0.0
    last_ms = None
    last_plot_ms = 0.0
    last_dt = 80.0

    for raw in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw.strip()
        if line.startswith("[RUN]"):
            run = line
            continue
        if not line.startswith("[FLT]"):
            continue
        content = line[5:].strip()
        if content.startswith("ms,"):
            header = [part.strip() for part in content.split(",")]
            continue
        if not header:
            continue
        values = next(csv.reader([content]))
        if len(values) != len(header):
            continue
        row = dict(zip(header, values))
        for key, value in list(row.items()):
            try:
                row[key] = float(value)
            except ValueError:
                pass
        ms = row.get("ms")
        if isinstance(ms, float):
            if last_ms is not None:
                if ms < last_ms:
                    time_offset = last_plot_ms + last_dt - ms
                else:
                    last_dt = max(1.0, ms - last_ms)
            row["t"] = (time_offset + ms) / 1000.0
            last_plot_ms = time_offset + ms
            last_ms = ms
        rows.append(row)
    return run, rows


def finite(row, key: str):
    value = row.get(key)
    return value if isinstance(value, float) and math.isfinite(value) else None


def corr(xs, ys) -> float:
    if len(xs) < 4:
        return 0.0
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    vx = sum((x - mx) ** 2 for x in xs)
    vy = sum((y - my) ** 2 for y in ys)
    if vx <= 0.0 or vy <= 0.0:
        return 0.0
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / math.sqrt(vx * vy)


def turning_points(rows):
    turns = []
    for i in range(1, len(rows) - 1):
        a, b, c = finite(rows[i - 1], "alt"), finite(rows[i], "alt"), finite(rows[i + 1], "alt")
        if a is None or b is None or c is None:
            continue
        if a < b > c:
            turns.append((finite(rows[i], "t"), b, "peak"))
        elif a > b < c:
            turns.append((finite(rows[i], "t"), b, "valley"))
    return turns


def delay_scan(rows, hover: float):
    out = []
    for lag in range(0, 13):
        xs, ys = [], []
        for i in range(len(rows) - lag - 1):
            t0, t1 = finite(rows[i + lag], "t"), finite(rows[i + lag + 1], "t")
            alt0, alt1 = finite(rows[i + lag], "alt"), finite(rows[i + lag + 1], "alt")
            thr = finite(rows[i], "thr")
            if None in (t0, t1, alt0, alt1, thr):
                continue
            dt = t1 - t0
            if dt <= 0.0:
                continue
            xs.append(thr - hover)
            ys.append((alt1 - alt0) / dt)
        out.append((lag, corr(xs, ys)))
    return out


def replay_candidate(rows, *, kp: float, kd: float, max_climb: float, max_desc: float, ramp: float, hover: float):
    if not rows:
        return None
    internal = finite(rows[0], "alt") or 0.0
    target = max(finite(row, "setpt") or internal for row in rows)
    last_t = finite(rows[0], "t") or 0.0
    throttles = []
    desired = []
    for row in rows:
        t = finite(row, "t") or last_t
        dt = max(0.02, min(0.15, t - last_t))
        last_t = t
        alt = finite(row, "alt")
        f_v = finite(row, "fV")
        if alt is None or f_v is None:
            continue
        if internal < target:
            internal = min(internal + ramp * dt, target)
        elif internal > target:
            internal = max(internal - ramp * dt, target)
        err = internal - alt
        climb = max_climb
        desc = max_desc
        if abs(err) < 0.5:
            factor = 0.35 + 0.65 * abs(err) / 0.5
            climb *= factor
            desc *= factor
        des_v = max(-desc, min(climb, kp * err))
        desired.append(des_v)
        throttles.append(hover + kd * (des_v - f_v))
    if not throttles:
        return None
    return {
        "kp": kp,
        "kd": kd,
        "max_climb": max_climb,
        "max_desc": max_desc,
        "ramp": ramp,
        "thr_min": min(throttles),
        "thr_max": max(throttles),
        "thr_avg": sum(throttles) / len(throttles),
        "des_min": min(desired),
        "des_max": max(desired),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log", nargs="?", type=Path)
    parser.add_argument("--log-dir", type=Path, default=Path("flight_logs"))
    args = parser.parse_args()

    path = args.log or discover_latest(args.log_dir)
    run, rows = load_log(path)
    params = parse_run(run)
    hover = params.get("hover", 1430.0)
    cascade = [row for row in rows if row.get("phase") == "cascade"]
    print(f"File: {path}")
    print(run)
    print(f"Cascade rows: {len(cascade)}")
    if not cascade:
        return

    alts = [finite(r, "alt") for r in cascade if finite(r, "alt") is not None]
    thrs = [finite(r, "thr") for r in cascade if finite(r, "thr") is not None]
    fvs = [finite(r, "fV") for r in cascade if finite(r, "fV") is not None]
    print(f"Altitude range: {min(alts):.3f}..{max(alts):.3f} m")
    print(f"Throttle range: {min(thrs):.0f}..{max(thrs):.0f} us")
    print(f"Filtered vario range: {min(fvs):.3f}..{max(fvs):.3f} m/s")
    print("Turning points:")
    for t, alt, kind in turning_points(cascade):
        print(f"  {kind:6s} t={t:.2f}s alt={alt:.3f}m")

    scan = delay_scan(cascade, hover)
    best_lag, best_corr = max(scan, key=lambda item: item[1])
    print("Throttle-to-future-alt-rate correlation:")
    for lag, value in scan:
        print(f"  lag~{lag * 80:4d}ms corr={value:+.3f}")
    print(f"Best observed delay band: ~{best_lag * 80}ms (corr={best_corr:+.3f})")

    candidates = [
        (0.7, 70, 0.25, 0.20, 0.4),
        (0.8, 70, 0.30, 0.22, 0.5),
        (0.9, 75, 0.35, 0.25, 0.6),
        (1.0, 80, 0.45, 0.30, 0.8),
    ]
    print("Recorded-sensor command replay:")
    for c in candidates:
        result = replay_candidate(cascade, kp=c[0], kd=c[1], max_climb=c[2], max_desc=c[3], ramp=c[4], hover=hover)
        if result:
            print(
                f"  kp={result['kp']:.2f} kd={result['kd']:.0f} climb={result['max_climb']:.2f} "
                f"desc={result['max_desc']:.2f} ramp={result['ramp']:.2f} -> "
                f"thr={result['thr_min']:.0f}..{result['thr_max']:.0f}us "
                f"desV={result['des_min']:.2f}..{result['des_max']:.2f}m/s"
            )


if __name__ == "__main__":
    main()
