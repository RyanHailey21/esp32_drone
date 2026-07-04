#!/usr/bin/env python3
"""Summarize ESP32 quad flight logs emitted by the web UI."""

from __future__ import annotations

import argparse
import csv
import html
import json
import webbrowser
from collections import Counter
from pathlib import Path
from statistics import mean


NUMERIC_FIELDS = {
    "ms", "state", "alt", "lowRel", "tof", "tofW", "baro", "setpt", "fV",
    "cbaro", "src", "rawV", "desV", "aErr", "vErr", "P", "I", "rawThr",
    "thr", "minThr", "maxThr", "sat",
}


def parse_log(path: Path):
    run = ""
    header = None
    rows = []

    for raw_line in path.read_text(encoding="utf-8-sig").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("[RUN]"):
            run = line
            continue
        if line.startswith("[FLT]"):
            content = line[5:].strip()
            if content.startswith("ms,"):
                header = [part.strip() for part in content.split(",")]
                continue
            if header is None:
                continue
            values = next(csv.reader([content]))
            if len(values) != len(header):
                continue
            row = dict(zip(header, values))
            for key in NUMERIC_FIELDS & row.keys():
                try:
                    row[key] = float(row[key])
                except ValueError:
                    pass
            rows.append(row)

    return run, rows


def fmt(value: float | None, digits: int = 3) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


def span(rows, key: str):
    vals = [row[key] for row in rows if isinstance(row.get(key), float)]
    if not vals:
        return None, None
    return min(vals), max(vals)


def runs_of_invalid_tof(rows):
    runs = []
    start = None
    end = None
    for row in rows:
        invalid = row.get("tof", -1.0) < 0 or row.get("tofW", 0.0) <= 0
        ms = int(row.get("ms", 0))
        if invalid:
            if start is None:
                start = ms
            end = ms
        elif start is not None:
            runs.append((start, end))
            start = None
    if start is not None:
        runs.append((start, end))
    return runs


def source_jump_count(rows, threshold_m: float):
    jumps = []
    last = None
    for row in rows:
        alt = row.get("alt")
        if not isinstance(alt, float):
            continue
        if last is not None:
            delta = alt - last["alt"]
            if abs(delta) >= threshold_m:
                jumps.append((int(last["ms"]), int(row["ms"]), delta))
        last = row
    return jumps


def number_or_none(row, key: str):
    value = row.get(key)
    return value if isinstance(value, float) else None


def preview_path_for(log_path: Path, output: Path | None, multiple: bool) -> Path:
    if output is None:
        return log_path.with_suffix(".preview.html")
    if multiple or output.is_dir():
        output.mkdir(parents=True, exist_ok=True)
        return output / f"{log_path.stem}.preview.html"
    output.parent.mkdir(parents=True, exist_ok=True)
    return output


def write_preview(path: Path, run: str, rows, output: Path, jump_threshold_m: float):
    points = []
    for row in rows:
        ms = number_or_none(row, "ms")
        if ms is None:
            continue
        points.append({
            "ms": ms,
            "phase": str(row.get("phase", "")),
            "alt": number_or_none(row, "alt"),
            "lowRel": number_or_none(row, "lowRel"),
            "tof": number_or_none(row, "tof"),
            "tofW": number_or_none(row, "tofW"),
            "baro": number_or_none(row, "baro"),
            "cbaro": number_or_none(row, "cbaro"),
            "setpt": number_or_none(row, "setpt"),
            "fV": number_or_none(row, "fV"),
            "desV": number_or_none(row, "desV"),
            "thr": number_or_none(row, "thr"),
            "sat": number_or_none(row, "sat"),
            "src": number_or_none(row, "src"),
        })

    jumps = source_jump_count(rows, jump_threshold_m)
    source_names = {0: "BARO", 1: "TOF", 2: "BLEND", 3: "TOF HOLD"}
    source_counts = Counter(int(row.get("src", -1)) for row in rows if isinstance(row.get("src"), float))
    source_summary = ", ".join(
        f"{source_names.get(k, str(k))}={v}" for k, v in sorted(source_counts.items())
    ) or "not present"

    payload = {
        "title": path.name,
        "run": run,
        "points": points,
        "jumps": [{"from": a, "to": b, "delta": d} for a, b, d in jumps],
        "jumpThreshold": jump_threshold_m,
        "sourceSummary": source_summary,
    }
    payload_json = json.dumps(payload, separators=(",", ":"))

    document = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(path.name)} preview</title>
<style>
body {{ margin: 0; font-family: system-ui, Segoe UI, Arial, sans-serif; background: #101316; color: #e8eef2; }}
main {{ max-width: 1180px; margin: 0 auto; padding: 24px; }}
h1 {{ margin: 0 0 6px; font-size: 22px; }}
.run {{ color: #9fb0bd; margin-bottom: 18px; font-family: Consolas, monospace; font-size: 13px; }}
.grid {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; margin-bottom: 14px; }}
.card {{ background: #171c20; border: 1px solid #2a343c; border-radius: 8px; padding: 10px 12px; }}
.label {{ color: #91a1ad; font-size: 11px; text-transform: uppercase; letter-spacing: .05em; }}
.value {{ font-size: 18px; margin-top: 3px; }}
.plot {{ background: #151a1f; border: 1px solid #2a343c; border-radius: 8px; margin: 14px 0; padding: 12px; }}
.plot-title {{ display: flex; justify-content: space-between; color: #cbd5dc; font-size: 13px; margin-bottom: 8px; }}
svg {{ display: block; width: 100%; height: 320px; background: #0d1115; border-radius: 4px; }}
.legend {{ display: flex; gap: 14px; flex-wrap: wrap; color: #9fb0bd; font-size: 12px; margin-top: 8px; }}
.key {{ display: inline-flex; align-items: center; gap: 6px; }}
.sw {{ width: 18px; height: 3px; border-radius: 2px; }}
.hint {{ color: #91a1ad; font-size: 12px; margin-top: 10px; }}
@media (max-width: 760px) {{ .grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }} main {{ padding: 14px; }} }}
</style>
</head>
<body>
<main>
  <h1>{html.escape(path.name)}</h1>
  <div class="run">{html.escape(run or "No [RUN] header found")}</div>
  <div class="grid" id="stats"></div>
  <section class="plot">
    <div class="plot-title"><span>Altitude Sources</span><span>meters vs time</span></div>
    <svg id="altPlot" viewBox="0 0 1100 320" role="img" aria-label="Altitude plot"></svg>
    <div class="legend">
      <span class="key"><span class="sw" style="background:#39d98a"></span>alt</span>
      <span class="key"><span class="sw" style="background:#61dafb"></span>tof</span>
      <span class="key"><span class="sw" style="background:#f7b955"></span>baro</span>
      <span class="key"><span class="sw" style="background:#b084f5"></span>corrected baro</span>
      <span class="key"><span class="sw" style="background:#ffffff"></span>setpoint</span>
    </div>
  </section>
  <section class="plot">
    <div class="plot-title"><span>Control</span><span>throttle us, vertical speed m/s</span></div>
    <svg id="ctrlPlot" viewBox="0 0 1100 320" role="img" aria-label="Control plot"></svg>
    <div class="legend">
      <span class="key"><span class="sw" style="background:#ff6b6b"></span>throttle</span>
      <span class="key"><span class="sw" style="background:#61dafb"></span>filtered vario</span>
      <span class="key"><span class="sw" style="background:#f7b955"></span>desired speed</span>
    </div>
  </section>
  <section class="plot">
    <div class="plot-title"><span>Altitude Source</span><span>0 baro, 1 tof, 2 blend, 3 tof hold</span></div>
    <svg id="srcPlot" viewBox="0 0 1100 180" role="img" aria-label="Source plot"></svg>
    <div class="hint">Red vertical markers indicate parsed altitude jumps above the configured threshold.</div>
  </section>
</main>
<script>
const DATA = {payload_json};
const W = 1100, H = 320, M = {{l:54, r:16, t:16, b:30}};
const COLORS = {{ alt:'#39d98a', tof:'#61dafb', baro:'#f7b955', cbaro:'#b084f5', setpt:'#fff', thr:'#ff6b6b', fV:'#61dafb', desV:'#f7b955', src:'#39d98a', jump:'#ff4d4d' }};
const pts = DATA.points;
function finite(v) {{ return typeof v === 'number' && Number.isFinite(v); }}
function extent(keys) {{
  let vals = [];
  for (const p of pts) for (const k of keys) if (finite(p[k]) && !(k === 'tof' && p[k] < 0)) vals.push(p[k]);
  if (!vals.length) return [0, 1];
  let lo = Math.min(...vals), hi = Math.max(...vals);
  if (lo === hi) {{ lo -= 1; hi += 1; }}
  const pad = (hi - lo) * 0.08;
  return [lo - pad, hi + pad];
}}
function xScale(ms) {{
  const first = pts[0]?.ms ?? 0, last = pts[pts.length - 1]?.ms ?? 1;
  return M.l + (ms - first) / Math.max(1, last - first) * (W - M.l - M.r);
}}
function yScale(v, lo, hi, height=H) {{ return M.t + (hi - v) / (hi - lo) * (height - M.t - M.b); }}
function linePath(key, lo, hi, height=H) {{
  let d = '', open = false;
  for (const p of pts) {{
    let v = p[key];
    if (!finite(v) || (key === 'tof' && v < 0)) {{ open = false; continue; }}
    const cmd = open ? 'L' : 'M';
    d += `${{cmd}}${{xScale(p.ms).toFixed(1)}},${{yScale(v, lo, hi, height).toFixed(1)}}`;
    open = true;
  }}
  return d;
}}
function drawAxes(svg, lo, hi, height=H) {{
  const plotW = W - M.l - M.r, plotH = height - M.t - M.b;
  svg.innerHTML = `<line x1="${{M.l}}" y1="${{M.t}}" x2="${{M.l}}" y2="${{M.t+plotH}}" stroke="#33414a"/>
    <line x1="${{M.l}}" y1="${{M.t+plotH}}" x2="${{M.l+plotW}}" y2="${{M.t+plotH}}" stroke="#33414a"/>`;
  for (let i=0; i<=4; i++) {{
    const y = M.t + plotH * i / 4;
    const v = hi - (hi - lo) * i / 4;
    svg.innerHTML += `<line x1="${{M.l}}" y1="${{y}}" x2="${{M.l+plotW}}" y2="${{y}}" stroke="#1f2930"/>
      <text x="8" y="${{y+4}}" fill="#91a1ad" font-size="11">${{v.toFixed(2)}}</text>`;
  }}
}}
function drawLines(id, keys) {{
  const svg = document.getElementById(id);
  const [lo, hi] = extent(keys);
  drawAxes(svg, lo, hi);
  for (const j of DATA.jumps) {{
    const x = xScale(j.to);
    svg.innerHTML += `<line x1="${{x}}" y1="${{M.t}}" x2="${{x}}" y2="${{H-M.b}}" stroke="${{COLORS.jump}}" stroke-opacity=".45"/>`;
  }}
  for (const key of keys) {{
    svg.innerHTML += `<path d="${{linePath(key, lo, hi)}}" fill="none" stroke="${{COLORS[key]}}" stroke-width="2"/>`;
  }}
}}
function drawSource() {{
  const svg = document.getElementById('srcPlot');
  const height = 180, lo = -0.2, hi = 3.2;
  drawAxes(svg, lo, hi, height);
  for (const p of pts) {{
    if (!finite(p.src)) continue;
    const x = xScale(p.ms), y = yScale(p.src, lo, hi, height);
    const color = ['#f7b955','#61dafb','#b084f5','#39d98a'][p.src] || '#999';
    svg.innerHTML += `<circle cx="${{x.toFixed(1)}}" cy="${{y.toFixed(1)}}" r="3" fill="${{color}}"/>`;
  }}
}}
function stat(label, value) {{
  return `<div class="card"><div class="label">${{label}}</div><div class="value">${{value}}</div></div>`;
}}
document.getElementById('stats').innerHTML = [
  stat('Rows', pts.length),
  stat('Duration', pts.length ? ((pts[pts.length-1].ms - pts[0].ms)/1000).toFixed(2) + 's' : 'n/a'),
  stat('Source counts', DATA.sourceSummary),
  stat('Altitude jumps', DATA.jumps.length + ' >= ' + DATA.jumpThreshold.toFixed(2) + 'm')
].join('');
drawLines('altPlot', ['alt','tof','baro','cbaro','setpt']);
drawLines('ctrlPlot', ['thr','fV','desV']);
drawSource();
</script>
</body>
</html>
"""
    output.write_text(document, encoding="utf-8")


def summarize(path: Path, jump_threshold_m: float):
    run, rows = parse_log(path)
    print(f"File: {path}")
    if run:
        print(run)
    print(f"Rows: {len(rows)}")
    if not rows:
        return run, rows

    phase_counts = Counter(str(row.get("phase", "")) for row in rows)
    sat_counts = Counter(int(row.get("sat", 0)) for row in rows)
    source_counts = Counter(int(row.get("src", -1)) for row in rows if isinstance(row.get("src"), float))
    tof_valid_rows = [row for row in rows if row.get("tof", -1.0) >= 0 and row.get("tofW", 0.0) > 0]
    invalid_runs = runs_of_invalid_tof(rows)
    jumps = source_jump_count(rows, jump_threshold_m)

    print("Phases:", ", ".join(f"{k}={v}" for k, v in sorted(phase_counts.items())))
    print("Saturation:", ", ".join(f"{k}={v}" for k, v in sorted(sat_counts.items())))
    if source_counts:
        source_names = {0: "baro", 1: "tof", 2: "blend", 3: "tof_hold"}
        print("Sources:", ", ".join(f"{source_names.get(k, k)}={v}" for k, v in sorted(source_counts.items())))
    print(f"ToF valid: {len(tof_valid_rows)}/{len(rows)} ({100.0 * len(tof_valid_rows) / len(rows):.1f}%)")
    if invalid_runs:
        longest = max(end - start for start, end in invalid_runs)
        preview = ", ".join(f"{start}-{end}ms" for start, end in invalid_runs[:8])
        suffix = " ..." if len(invalid_runs) > 8 else ""
        print(f"ToF invalid runs: {len(invalid_runs)}, longest={longest}ms, {preview}{suffix}")

    for key in ("alt", "lowRel", "tof", "baro", "cbaro", "setpt", "fV", "desV", "thr"):
        low, high = span(rows, key)
        print(f"{key}: min={fmt(low)} max={fmt(high)}")

    if jumps:
        print(f"Altitude jumps >= {jump_threshold_m:.2f}m: {len(jumps)}")
        for start, end, delta in jumps[:8]:
            print(f"  {start}->{end}ms delta={delta:+.3f}m")
    else:
        print(f"Altitude jumps >= {jump_threshold_m:.2f}m: 0")

    cascade = [row for row in rows if row.get("phase") == "cascade"]
    if cascade:
        final = cascade[-1]
        avg_thr = mean(row["thr"] for row in cascade if isinstance(row.get("thr"), float))
        print(
            "Final cascade: "
            f"t={int(final['ms'])}ms alt={fmt(final.get('alt'))}m "
            f"setpt={fmt(final.get('setpt'))}m fV={fmt(final.get('fV'))}m/s "
            f"desV={fmt(final.get('desV'))}m/s thr={int(final.get('thr', 0))}"
        )
        print(f"Cascade average throttle: {avg_thr:.0f}us")

    return run, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="+", type=Path, help="CSV log file(s) to summarize")
    parser.add_argument("--jump-threshold-m", type=float, default=0.35)
    parser.add_argument("--preview", action="store_true", help="write standalone HTML plot preview(s)")
    parser.add_argument("--open", action="store_true", help="open generated preview(s) in the default browser")
    parser.add_argument("--preview-output", type=Path, help="preview file path, or directory when parsing multiple logs")
    args = parser.parse_args()

    for index, path in enumerate(args.logs):
        if index:
            print()
        run, rows = summarize(path, args.jump_threshold_m)
        if args.preview:
            out = preview_path_for(path, args.preview_output, len(args.logs) > 1)
            write_preview(path, run, rows, out, args.jump_threshold_m)
            print(f"Preview: {out}")
            if args.open:
                webbrowser.open(out.resolve().as_uri())


if __name__ == "__main__":
    main()
