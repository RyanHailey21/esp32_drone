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
    "cbaro", "src", "rawV", "usedV", "bfV", "derV", "vsrc", "desV", "aErr", "vErr", "P", "I", "rawThr",
    "thr", "minThr", "maxThr", "sat", "accX", "accY", "accZ", "gyroX",
    "gyroY", "gyroZ", "roll", "pitch", "yaw", "cycle", "sensors", "rcThr",
    "rcArm", "rcAngle", "vbat", "amps", "diag", "tofRaw", "tofReadOk",
    "tofReject", "tofDt", "tofStatus", "tofI2c",
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
    alias = {"usedV": "rawV"}
    vals = []
    for row in rows:
        value = row.get(key)
        if value is None and key in alias:
            value = row.get(alias[key])
        if isinstance(value, float):
            vals.append(value)
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


def discover_logs(log_dir: Path) -> list[Path]:
    return sorted(
        log_dir.glob("quad-flight-log-*.csv"),
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )


def logs_needing_preview(log_dir: Path) -> list[Path]:
    logs = []
    for log in discover_logs(log_dir):
        preview = log.with_suffix(".preview.html")
        if not preview.exists() or preview.stat().st_mtime < log.stat().st_mtime:
            logs.append(log)
    return logs


def write_preview(path: Path, run: str, rows, output: Path, jump_threshold_m: float):
    points = []
    last_ms = None
    last_plot_ms = 0.0
    last_dt_ms = 80.0
    time_offset_ms = 0.0
    for row in rows:
        ms = number_or_none(row, "ms")
        if ms is None:
            continue
        if last_ms is not None:
            if ms < last_ms:
                time_offset_ms = last_plot_ms + last_dt_ms - ms
            else:
                last_dt_ms = max(1.0, ms - last_ms)
        plot_ms = time_offset_ms + ms
        last_ms = ms
        last_plot_ms = plot_ms
        points.append({
            "ms": ms,
            "t": plot_ms / 1000.0,
            "phase": str(row.get("phase", "")),
            "alt": number_or_none(row, "alt"),
            "lowRel": number_or_none(row, "lowRel"),
            "tof": number_or_none(row, "tof"),
            "tofW": number_or_none(row, "tofW"),
            "baro": number_or_none(row, "baro"),
            "cbaro": number_or_none(row, "cbaro"),
            "setpt": number_or_none(row, "setpt"),
            "fV": number_or_none(row, "fV"),
            "usedV": number_or_none(row, "usedV") if number_or_none(row, "usedV") is not None else number_or_none(row, "rawV"),
            "bfV": number_or_none(row, "bfV"),
            "derV": number_or_none(row, "derV"),
            "vsrc": number_or_none(row, "vsrc"),
            "desV": number_or_none(row, "desV"),
            "thr": number_or_none(row, "thr"),
            "sat": number_or_none(row, "sat"),
            "src": number_or_none(row, "src"),
            "accX": number_or_none(row, "accX"),
            "accY": number_or_none(row, "accY"),
            "accZ": number_or_none(row, "accZ"),
            "gyroX": number_or_none(row, "gyroX"),
            "gyroY": number_or_none(row, "gyroY"),
            "gyroZ": number_or_none(row, "gyroZ"),
            "roll": number_or_none(row, "roll"),
            "pitch": number_or_none(row, "pitch"),
            "yaw": number_or_none(row, "yaw"),
            "cycle": number_or_none(row, "cycle"),
            "rcThr": number_or_none(row, "rcThr"),
            "rcArm": number_or_none(row, "rcArm"),
            "rcAngle": number_or_none(row, "rcAngle"),
            "vbat": number_or_none(row, "vbat"),
            "amps": number_or_none(row, "amps"),
            "diag": number_or_none(row, "diag"),
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
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.3/dist/chart.umd.min.js"></script>
<style>
@import url('https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:wght@500;600;700&family=IBM+Plex+Mono:wght@500;600;700&display=swap');
:root {{
  --paper:   #f4f5f7;
  --surface: #ffffff;
  --border:  #d7dbe2;
  --ink:     #161b22;
  --ink-mid: #4b5563;
  --ink-dim: #838c99;
  --grid:    #e7eaee;
  --font-ui:  'IBM Plex Sans', system-ui, sans-serif;
  --font-val: 'IBM Plex Mono', 'Consolas', monospace;
}}
body {{ margin: 0; font-family: var(--font-ui); background: var(--paper); color: var(--ink); }}
main {{ max-width: 1180px; margin: 0 auto; padding: 24px; }}
h1 {{ margin: 0 0 6px; font-size: 20px; font-weight: 700; }}
.run {{ color: var(--ink-dim); margin-bottom: 18px; font-family: var(--font-val); font-size: 12px; }}
.grid {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 10px; margin-bottom: 14px; }}
.card {{ background: var(--surface); border: 1px solid var(--border); border-radius: 8px; padding: 10px 12px; box-shadow: 0 1px 2px rgba(22,27,34,0.05), 0 6px 16px rgba(22,27,34,0.06); }}
.label {{ color: var(--ink-dim); font-size: 10px; text-transform: uppercase; letter-spacing: .08em; font-weight: 600; }}
.value {{ font-family: var(--font-val); font-weight: 600; font-size: 17px; margin-top: 4px; color: var(--ink); }}
.plot {{ background: var(--surface); border: 1px solid var(--border); border-radius: 8px; margin: 14px 0; padding: 14px; box-shadow: 0 1px 2px rgba(22,27,34,0.05), 0 6px 16px rgba(22,27,34,0.06); }}
.plot-title {{ display: flex; justify-content: space-between; color: var(--ink-mid); font-size: 13px; font-weight: 600; margin-bottom: 8px; }}
.chart-wrap {{ height: 360px; background: var(--surface); border-radius: 4px; padding: 6px; }}
.chart-wrap.compact {{ height: 240px; }}
canvas {{ width: 100% !important; height: 100% !important; }}
.legend {{ display: flex; gap: 14px; flex-wrap: wrap; color: var(--ink-mid); font-size: 12px; margin-top: 8px; }}
.key {{ display: inline-flex; align-items: center; gap: 6px; }}
.sw {{ width: 18px; height: 3px; border-radius: 2px; }}
.hint {{ color: var(--ink-dim); font-size: 12px; margin-top: 10px; }}
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
    <div class="chart-wrap"><canvas id="altPlot" role="img" aria-label="Altitude plot"></canvas></div>
  </section>
  <section class="plot">
    <div class="plot-title"><span>Control</span><span>left: vertical speed m/s, right: throttle us</span></div>
    <div class="chart-wrap"><canvas id="ctrlPlot" role="img" aria-label="Control plot"></canvas></div>
  </section>
  <section class="plot">
    <div class="plot-title"><span>Altitude Source</span><span>0 baro, 1 tof, 2 blend, 3 tof hold</span></div>
    <div class="chart-wrap compact"><canvas id="srcPlot" role="img" aria-label="Source plot"></canvas></div>
    <div class="hint">Red vertical markers indicate parsed altitude jumps above the configured threshold.</div>
  </section>
  <section class="plot">
    <div class="plot-title"><span>FC IMU / Attitude</span><span>raw accel, roll/pitch degrees</span></div>
    <div class="chart-wrap"><canvas id="imuPlot" role="img" aria-label="IMU plot"></canvas></div>
  </section>
</main>
<script>
const DATA = {payload_json};
// Categorical line colors — validated colorblind-safe set (dataviz skill,
// light-mode default order). Red is reserved for anomaly/jump markers only,
// never reused as a plain series color, so it never reads as a false alarm.
const COLORS = {{ alt:'#2a78d6', tof:'#1baf7a', baro:'#eda100', cbaro:'#4a3aa7', setpt:'#eb6834', thr:'#2a78d6', fV:'#1baf7a', usedV:'#008300', bfV:'#4a3aa7', desV:'#eb6834', src:'#2a78d6', accX:'#1baf7a', accY:'#eda100', accZ:'#2a78d6', roll:'#4a3aa7', pitch:'#e87ba4', jump:'#e34948' }};
const pts = DATA.points;
function finite(v) {{ return typeof v === 'number' && Number.isFinite(v); }}
function seriesData(key, scale=1) {{
  return pts.map(p => finite(p[key]) ? p[key] * scale : null);
}}
function timeLabels() {{
  return pts.map(p => (p.t ?? ((p.ms - (pts[0]?.ms ?? 0)) / 1000)).toFixed(2));
}}
const jumpLinePlugin = {{
  id: 'jumpLines',
  afterDatasetsDraw(chart, args, opts) {{
    if (!opts || !opts.enabled || !DATA.jumps.length || !pts.length) return;
    const xScale = chart.scales.x;
    const yArea = chart.chartArea;
    const firstMs = pts[0].ms;
    const ctx = chart.ctx;
    ctx.save();
    ctx.strokeStyle = COLORS.jump;
    ctx.globalAlpha = 0.45;
    ctx.lineWidth = 1;
    for (const jump of DATA.jumps) {{
      const x = xScale.getPixelForValue(((jump.to - firstMs) / 1000).toFixed(2));
      ctx.beginPath();
      ctx.moveTo(x, yArea.top);
      ctx.lineTo(x, yArea.bottom);
      ctx.stroke();
    }}
    ctx.restore();
  }}
}};
function chartOptions(yTitle, extraScales={{}}, jumpLines=false) {{
  return {{
    responsive: true,
    maintainAspectRatio: false,
    interaction: {{ mode: 'index', intersect: false }},
    plugins: {{
      legend: {{ labels: {{ color: '#4b5563', usePointStyle: true, boxWidth: 8 }} }},
      jumpLines: {{ enabled: jumpLines }},
      tooltip: {{
        callbacks: {{
          title: items => items.length ? `t=${{items[0].label}}s` : ''
        }}
      }}
    }},
    scales: Object.assign({{
      x: {{
        title: {{ display: true, text: 'time s', color: '#838c99' }},
        ticks: {{ color: '#838c99', maxTicksLimit: 10 }},
        grid: {{ color: '#e7eaee' }}
      }},
      y: {{
        title: {{ display: true, text: yTitle, color: '#838c99' }},
        ticks: {{ color: '#838c99' }},
        grid: {{ color: '#e7eaee' }}
      }}
    }}, extraScales)
  }};
}}
function dataset(label, key, color, opts={{}}) {{
  return Object.assign({{
    label,
    data: seriesData(key, opts.scale ?? 1),
    yAxisID: opts.yAxisID || 'y',
    borderColor: color,
    backgroundColor: color,
    pointRadius: opts.pointRadius ?? 0,
    borderWidth: opts.borderWidth ?? 2,
    borderDash: opts.borderDash,
    hidden: opts.hidden || false,
    tension: opts.tension ?? 0.15,
    stepped: opts.stepped || false
  }}, opts.chart || {{}});
}}
function drawChart(id, datasets, yTitle, options={{}}) {{
  const ctx = document.getElementById(id);
  if (!window.Chart || !ctx) return;
  new Chart(ctx, {{
    type: 'line',
    data: {{ labels: timeLabels(), datasets }},
    options: chartOptions(yTitle, options.scales || {{}}, !!options.jumpLines),
    plugins: [jumpLinePlugin]
  }});
}}
function drawAltitudeChart() {{
  drawChart('altPlot', [
    dataset('Altitude', 'alt', COLORS.alt),
    dataset('ToF', 'tof', COLORS.tof),
    dataset('Baro', 'baro', COLORS.baro),
    dataset('Corrected baro', 'cbaro', COLORS.cbaro),
    dataset('Setpoint', 'setpt', COLORS.setpt, {{ borderDash: [6, 4] }})
  ], 'meters', {{ jumpLines: true }});
}}
function drawControlChart() {{
  const ctx = document.getElementById('ctrlPlot');
  if (!window.Chart || !ctx) return;
  new Chart(ctx, {{
    type: 'line',
    data: {{
      labels: timeLabels(),
      datasets: [
        {{ label: 'Throttle', data: seriesData('thr'), yAxisID: 'thr', borderColor: COLORS.thr, backgroundColor: COLORS.thr, pointRadius: 0, borderWidth: 2, tension: 0.15 }},
        {{ label: 'Filtered vario', data: seriesData('fV'), yAxisID: 'v', borderColor: COLORS.fV, backgroundColor: COLORS.fV, pointRadius: 0, borderWidth: 2, tension: 0.15 }},
        {{ label: 'Used vario', data: seriesData('usedV', 0.01), yAxisID: 'v', borderColor: COLORS.usedV, backgroundColor: COLORS.usedV, pointRadius: 0, borderWidth: 2, tension: 0.15 }},
        {{ label: 'BF vario', data: seriesData('bfV', 0.01), yAxisID: 'v', borderColor: COLORS.bfV, backgroundColor: COLORS.bfV, pointRadius: 0, borderWidth: 1.5, borderDash: [5, 4], tension: 0.15, hidden: true }},
        {{ label: 'Desired speed', data: seriesData('desV'), yAxisID: 'v', borderColor: COLORS.desV, backgroundColor: COLORS.desV, pointRadius: 0, borderWidth: 2, tension: 0.15 }}
      ]
    }},
    options: {{
      responsive: true,
      maintainAspectRatio: false,
      interaction: {{ mode: 'index', intersect: false }},
      plugins: {{
        legend: {{ labels: {{ color: '#4b5563', usePointStyle: true, boxWidth: 8 }} }},
        tooltip: {{
          callbacks: {{
            title: items => items.length ? `t=${{items[0].label}}s` : '',
            label: item => {{
              const unit = item.dataset.yAxisID === 'thr' ? ' us' : ' m/s';
              return `${{item.dataset.label}}: ${{Number(item.raw).toFixed(item.dataset.yAxisID === 'thr' ? 0 : 2)}}${{unit}}`;
            }}
          }}
        }}
      }},
      scales: {{
        x: {{
          title: {{ display: true, text: 'time s', color: '#838c99' }},
          ticks: {{ color: '#838c99', maxTicksLimit: 10 }},
          grid: {{ color: '#e7eaee' }}
        }},
        v: {{
          position: 'left',
          title: {{ display: true, text: 'vertical speed m/s', color: '#838c99' }},
          ticks: {{ color: '#838c99' }},
          grid: {{ color: '#e7eaee' }},
          suggestedMin: -1.5,
          suggestedMax: 1.5
        }},
        thr: {{
          position: 'right',
          title: {{ display: true, text: 'throttle us', color: '#838c99' }},
          ticks: {{ color: '#838c99' }},
          grid: {{ drawOnChartArea: false }}
        }}
      }}
    }},
    plugins: [jumpLinePlugin]
  }});
}}
function drawSourceChart() {{
  drawChart('srcPlot', [
    dataset('Altitude source', 'src', COLORS.src, {{ stepped: true, pointRadius: 2 }})
  ], 'source id', {{
    jumpLines: true,
    scales: {{
      y: {{
        min: -0.2,
        max: 3.2,
        title: {{ display: true, text: 'source id', color: '#838c99' }},
        ticks: {{
          color: '#838c99',
          stepSize: 1,
          callback: value => ({{0:'BARO',1:'TOF',2:'BLEND',3:'HOLD'}}[value] ?? value)
        }},
        grid: {{ color: '#e7eaee' }}
      }}
    }}
  }});
}}
function drawImuChart() {{
  drawChart('imuPlot', [
    dataset('accZ', 'accZ', COLORS.accZ, {{ yAxisID: 'acc' }}),
    dataset('accX', 'accX', COLORS.accX, {{ yAxisID: 'acc', hidden: true }}),
    dataset('accY', 'accY', COLORS.accY, {{ yAxisID: 'acc', hidden: true }}),
    dataset('roll', 'roll', COLORS.roll, {{ yAxisID: 'att' }}),
    dataset('pitch', 'pitch', COLORS.pitch, {{ yAxisID: 'att' }})
  ], 'raw accel', {{
    scales: {{
      acc: {{
        position: 'left',
        title: {{ display: true, text: 'raw accel', color: '#838c99' }},
        ticks: {{ color: '#838c99' }},
        grid: {{ color: '#e7eaee' }}
      }},
      att: {{
        position: 'right',
        title: {{ display: true, text: 'attitude deg', color: '#838c99' }},
        ticks: {{ color: '#838c99' }},
        grid: {{ drawOnChartArea: false }}
      }}
    }}
  }});
}}
function stat(label, value) {{
  return `<div class="card"><div class="label">${{label}}</div><div class="value">${{value}}</div></div>`;
}}
document.getElementById('stats').innerHTML = [
  stat('Rows', pts.length),
  stat('Duration', pts.length ? ((pts[pts.length-1].ms - pts[0].ms)/1000).toFixed(2) + 's' : 'n/a'),
  stat('Source counts', DATA.sourceSummary),
  stat('Altitude jumps', DATA.jumps.length + ' >= ' + DATA.jumpThreshold.toFixed(2) + 'm'),
  stat('Diag mask', pts.length ? '0x' + (pts[pts.length-1].diag ?? 0).toString(16) : 'n/a')
].join('');
drawAltitudeChart();
drawControlChart();
drawSourceChart();
drawImuChart();
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

    for key in ("alt", "lowRel", "tof", "baro", "cbaro", "setpt", "fV", "usedV", "bfV", "derV", "vsrc", "desV", "thr",
                "accZ", "roll", "pitch", "cycle", "rcThr", "vbat"):
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
    parser.add_argument("logs", nargs="*", type=Path, help="CSV log file(s) to summarize; defaults to newest flight_logs CSV")
    parser.add_argument("--log-dir", type=Path, default=Path("flight_logs"), help="directory to scan for default/latest/new logs")
    parser.add_argument("--latest", action="store_true", help="summarize only the newest CSV in --log-dir")
    parser.add_argument("--new", action="store_true", help="summarize CSVs whose preview is missing or older than the CSV")
    parser.add_argument("--jump-threshold-m", type=float, default=0.35)
    parser.add_argument("--preview", action="store_true", help="write standalone HTML plot preview(s)")
    parser.add_argument("--open", action="store_true", help="open generated preview(s) in the default browser")
    parser.add_argument("--preview-output", type=Path, help="preview file path, or directory when parsing multiple logs")
    args = parser.parse_args()

    logs = args.logs
    if args.new:
        logs = logs_needing_preview(args.log_dir)
        if logs:
            args.preview = True
    elif args.latest or not logs:
        discovered = discover_logs(args.log_dir)
        if not discovered:
            raise SystemExit(f"No quad-flight-log-*.csv files found in {args.log_dir}")
        logs = [discovered[0]]

    if not logs:
        print(f"No new logs need previews in {args.log_dir}")
        return

    for index, path in enumerate(logs):
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
