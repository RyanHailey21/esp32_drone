const SERVICE_UUID       = 'ab0828b1-198e-4351-b779-901fa0e0371e';
const HOVER_UUID         = 'ab0828b2-198e-4351-b779-901fa0e0371e';
const SPRINT_THROT_UUID  = 'ab0828b5-198e-4351-b779-901fa0e0371e';
const SPRINT_CUTOFF_UUID = 'ab0828b6-198e-4351-b779-901fa0e0371e';
const HOLD_KP_UUID       = 'ab0828b7-198e-4351-b779-901fa0e0371e';
const HOLD_KI_UUID       = 'ab0828bc-198e-4351-b779-901fa0e0371e';
const HOLD_KD_UUID       = 'ab0828be-198e-4351-b779-901fa0e0371e';
const TARGET_ALT_UUID    = 'ab0828bf-198e-4351-b779-901fa0e0371e';
const ALT_HOLD_TARGET_UUID = 'ab0828c0-198e-4351-b779-901fa0e0371e';
const PUNCH_START_UUID   = 'ab0828b8-198e-4351-b779-901fa0e0371e';
const PUNCH_THROT_UUID   = 'ab0828b9-198e-4351-b779-901fa0e0371e';
const COMMAND_UUID       = 'ab0828ba-198e-4351-b779-901fa0e0371e';
const BENCH_MODE_UUID    = 'ab0828bb-198e-4351-b779-901fa0e0371e';
const TELEMETRY_UUID     = 'ab0828bd-198e-4351-b779-901fa0e0371e';

const STATE_NAMES  = ['IDLE','ARMING','SPRINTING','HOLDING','PUNCHING','CUT','HOVER TEST','AUTO HOVER CAL','LANDING','DONE','ALT HOLD'];
const STATE_COLORS = ['','var(--amber)','var(--amber)','var(--green)','var(--red)','var(--red)','var(--cyan)','var(--cyan)','var(--amber)','var(--text-mid)','var(--green)'];

const CMD_HOVER_TEST     = 1;
const CMD_START_MISSION  = 2;
const CMD_DISARM         = 3;
const CMD_AUTO_HOVER_CAL = 4;
const CMD_ALT_HOLD       = 5;

const CAL_LIFTOFF_CM = 15;
const CAL_MIN_THROT  = 1150;
const CAL_MAX_THROT  = 1650;

let chars = {};
let device = null, connected = false;
let benchMode = 0;
let prevStateId = -1;

// Serialise all BLE writes — prevents "GATT operation already in progress"
let _bleQ = Promise.resolve();
function bleWrite(fn) { _bleQ = _bleQ.then(fn).catch(() => {}); }

const connectBtn = document.getElementById('connect-btn');
const statusDot  = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const logEl      = document.getElementById('log');
const benchBtn   = document.getElementById('btn-bench-mode');

if (!navigator.bluetooth) {
  const ua  = navigator.userAgent;
  const ios = /iphone|ipad|ipod/i.test(ua);
  const ff  = /firefox/i.test(ua);
  const linux = /linux/i.test(ua) && !/android/i.test(ua);
  const msg = ios
    ? 'iOS blocks Web Bluetooth in all browsers (Apple restriction). Install Bluefy from the App Store — it is a free browser that adds Web BLE to iOS. Open this page inside Bluefy.'
    : ff
      ? 'Firefox does not support Web Bluetooth. Open this page in Chrome.'
      : linux
        ? 'Chrome on Linux requires a flag: open chrome://flags/#enable-experimental-web-platform-features and set it to Enabled, then relaunch Chrome.'
        : 'Web Bluetooth not available. Use Chrome on Android, Windows, or macOS (not Firefox, Safari, or iOS).';
  const warn = document.getElementById('ble-warning');
  warn.textContent = msg;
  warn.classList.add('show');
  connectBtn.disabled = true;
  connectBtn.style.opacity = '0.3';
  connectBtn.textContent = 'BLE not available — see above';
}

// ── LOGGING ─────────────────────────────────────────────
function log(msg, type = 'info') {
  const el = document.createElement('div');
  el.className = type;
  el.textContent = new Date().toTimeString().slice(0, 8) + '  ' + msg;
  logEl.appendChild(el);
  logEl.scrollTop = logEl.scrollHeight;
}

// ── STATUS ───────────────────────────────────────────────
function setStatus(s, t) {
  statusDot.className = 'status-ring' + (s ? ' ' + s : '');
  statusText.textContent = t;
}

function enableAll(on) {
  ['sprint','cutoff','target-alt','alt-hold-target','kp','ki','kd','punch-start','punch-throt'].forEach(id => {
    const el = document.getElementById('param-' + id);
    if (el) el.classList.toggle('enabled', on);
  });
  document.getElementById('mission-controls').classList.toggle('enabled', on);
  document.getElementById('params-controls').classList.toggle('enabled', on);
}

// ── TABS ─────────────────────────────────────────────────
function switchTab(paneId) {
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.toggle('active', b.dataset.tab === paneId));
  document.querySelectorAll('.tab-pane').forEach(p => p.classList.toggle('active', p.id === paneId));
}

document.querySelectorAll('.tab-btn').forEach(btn => {
  btn.addEventListener('click', () => switchTab(btn.dataset.tab));
});

// ── BLE CONNECT ──────────────────────────────────────────
connectBtn.addEventListener('click', () => connected ? disconnect() : connect());

async function connect() {
  try {
    setStatus('connecting', 'scanning...');
    log('Scanning for Quad-Tuner...');
    device = await navigator.bluetooth.requestDevice({
      filters: [{ name: 'Quad-Tuner' }],
      optionalServices: [SERVICE_UUID]
    });
    device.addEventListener('gattserverdisconnected', onDisconnected);
    const server  = await device.gatt.connect();
    const service = await server.getPrimaryService(SERVICE_UUID);

    chars.hover      = await service.getCharacteristic(HOVER_UUID);
    chars.sprint     = await service.getCharacteristic(SPRINT_THROT_UUID);
    chars.cutoff     = await service.getCharacteristic(SPRINT_CUTOFF_UUID);
    chars.kp         = await service.getCharacteristic(HOLD_KP_UUID);
    chars.ki         = await service.getCharacteristic(HOLD_KI_UUID);
    chars.kd         = await service.getCharacteristic(HOLD_KD_UUID);
    chars.targetAlt  = await service.getCharacteristic(TARGET_ALT_UUID);
    chars.altHoldTarget = await service.getCharacteristic(ALT_HOLD_TARGET_UUID);
    chars.punchStart = await service.getCharacteristic(PUNCH_START_UUID);
    chars.punchThrot = await service.getCharacteristic(PUNCH_THROT_UUID);
    chars.command    = await service.getCharacteristic(COMMAND_UUID);
    chars.benchMode  = await service.getCharacteristic(BENCH_MODE_UUID);
    chars.telemetry  = await service.getCharacteristic(TELEMETRY_UUID);

    await chars.telemetry.startNotifications();
    chars.telemetry.addEventListener('characteristicvaluechanged', onTelemetry);

    await readAll();

    connected = true;
    setStatus('connected', device.name);
    connectBtn.textContent = 'Disconnect';
    connectBtn.classList.add('active');
    enableAll(true);
    log('Connected to ' + device.name, 'ok');
  } catch(e) {
    setStatus('error', 'failed');
    log('Error: ' + e.message, 'err');
    setTimeout(() => setStatus('', 'disconnected'), 2000);
  }
}

function disconnect() {
  if (device && device.gatt.connected) device.gatt.disconnect();
  onDisconnected();
}

function onDisconnected() {
  connected = false; chars = {};
  benchMode = 0;
  _bleQ = Promise.resolve();
  setBenchButton();
  setStatus('', 'disconnected');
  connectBtn.textContent = 'Connect to Quad-Tuner';
  connectBtn.classList.remove('active');
  enableAll(false);
  log('Disconnected', 'err');
}

// ── READ ALL PARAMS ──────────────────────────────────────
async function readAll() {
  try {
    const hover      = new DataView((await chars.hover.readValue()).buffer).getUint16(0, true);
    const sprint     = new DataView((await chars.sprint.readValue()).buffer).getUint16(0, true);
    const cutoffRaw  = new DataView((await chars.cutoff.readValue()).buffer).getUint16(0, true);
    const kpRaw      = new DataView((await chars.kp.readValue()).buffer).getUint16(0, true);
    const kiRaw      = new DataView((await chars.ki.readValue()).buffer).getUint16(0, true);
    const kdRaw      = new DataView((await chars.kd.readValue()).buffer).getUint16(0, true);
    const targetAlt  = new DataView((await chars.targetAlt.readValue()).buffer).getUint16(0, true);
    const altHoldTarget = new DataView((await chars.altHoldTarget.readValue()).buffer).getUint16(0, true);
    const punchStart = new DataView((await chars.punchStart.readValue()).buffer).getUint32(0, true);
    const punchThrot = new DataView((await chars.punchThrot.readValue()).buffer).getUint16(0, true);
    benchMode        = new DataView((await chars.benchMode.readValue()).buffer).getUint8(0);

    setParam('hover',       hover,      v => v,                   v => v);
    setParam('sprint',      sprint,     v => v,                   v => v);
    setParam('cutoff',      cutoffRaw,  v => (v/100).toFixed(1),  v => v);
    setParam('kp',          kpRaw,      v => (v/10).toFixed(1),   v => v);
    setParam('ki',          kiRaw,      v => (v/10).toFixed(1),   v => v);
    setParam('kd',          kdRaw,      v => (v/10).toFixed(1),   v => v);
    setParam('target-alt',  targetAlt,  v => (v/10).toFixed(1),   v => v);
    setParam('alt-hold-target', altHoldTarget, v => (v/10).toFixed(1), v => v);
    setParam('punch-start', punchStart, v => (v/1000).toFixed(1), v => v);
    setParam('punch-throt', punchThrot, v => v,                   v => v);

    setBenchButton();
    log('Values read from ESP32', 'ok');
  } catch(e) { log('Read failed: ' + e.message, 'err'); }
}

// ── UI HELPERS ───────────────────────────────────────────
function setParam(id, raw, fmt, sliderVal) {
  const valEl    = document.getElementById('val-' + id);
  const sliderEl = document.getElementById('slider-' + id);
  if (valEl)    valEl.textContent = fmt(raw);
  if (sliderEl) { sliderEl.value = sliderVal(raw); updateFill(sliderEl); }
}

function updateFill(slider) {
  const pct = (slider.value - slider.min) / (slider.max - slider.min) * 100;
  slider.style.setProperty('--fill', pct + '%');
}

function u16buf(v) { const b = new ArrayBuffer(2); new DataView(b).setUint16(0, v, true); return b; }
function u32buf(v) { const b = new ArrayBuffer(4); new DataView(b).setUint32(0, v, true); return b; }
function u8buf(v)  { const b = new ArrayBuffer(1); new DataView(b).setUint8(0, v);        return b; }

function setBenchButton() {
  benchBtn.textContent = benchMode ? 'Bench Mode: On' : 'Bench Mode: Off';
  benchBtn.classList.toggle('bench-on', !!benchMode);
}

function sendCommand(cmd, label) {
  if (!connected) return;
  bleWrite(async () => {
    try {
      await chars.command.writeValue(u8buf(cmd));
      log(label, 'ok');
    } catch(e) {
      log('Command failed: ' + e.message, 'err');
    }
  });
}

// ── COMMAND BUTTONS ──────────────────────────────────────
document.getElementById('btn-hover-test').addEventListener('click', () =>
  sendCommand(CMD_HOVER_TEST, 'Hover test command sent'));

document.getElementById('btn-auto-hover').addEventListener('click', () =>
  sendCommand(CMD_AUTO_HOVER_CAL, 'Auto hover calibration command sent'));

document.getElementById('btn-alt-hold').addEventListener('click', () =>
  sendCommand(CMD_ALT_HOLD, 'Alt hold command sent'));

document.getElementById('btn-start-mission').addEventListener('click', () =>
  sendCommand(CMD_START_MISSION, 'Mission start command sent'));

document.getElementById('btn-disarm').addEventListener('click', () =>
  sendCommand(CMD_DISARM, 'Disarm command sent'));

document.getElementById('strip-disarm').addEventListener('click', () =>
  sendCommand(CMD_DISARM, 'Disarm command sent (strip)'));

document.getElementById('btn-sync').addEventListener('click', async () => {
  if (!connected) return;
  log('Syncing values from ESP32...');
  await readAll();
});

benchBtn.addEventListener('click', () => {
  if (!connected) return;
  const next = benchMode ? 0 : 1;
  bleWrite(async () => {
    try {
      await chars.benchMode.writeValue(u8buf(next));
      benchMode = new DataView((await chars.benchMode.readValue()).buffer).getUint8(0);
      setBenchButton();
      log(benchMode ? 'Bench mode enabled' : 'Bench mode disabled', benchMode ? 'err' : 'ok');
    } catch(e) {
      log('Bench mode failed: ' + e.message, 'err');
    }
  });
});

// ── SLIDER LISTENERS ─────────────────────────────────────
const timers = {};
function debounce(key, fn, delay = 150) {
  clearTimeout(timers[key]);
  timers[key] = setTimeout(fn, delay);
}

const SLIDERS = [
  { id: 'alt-hold-target', char: 'altHoldTarget', enc: v => u16buf(v), fmt: v => (v/10).toFixed(1), label: v => 'ALT_HOLD_TARGET -> ' + (v/10).toFixed(1) + 'm', ms: 150 },
  { id: 'hover',       char: 'hover',      enc: v => u16buf(v),  fmt: v => String(v),           label: v => 'HOVER_THROTTLE → ' + v,                        ms: 30  },
  { id: 'sprint',      char: 'sprint',     enc: v => u16buf(v),  fmt: v => String(v),           label: v => 'SPRINT_THROTTLE → ' + v,                       ms: 150 },
  { id: 'cutoff',      char: 'cutoff',     enc: v => u16buf(v),  fmt: v => (v/100).toFixed(1),  label: v => 'SPRINT_CUTOFF → ' + (v/100).toFixed(1) + 'm', ms: 150 },
  { id: 'target-alt',  char: 'targetAlt',  enc: v => u16buf(v),  fmt: v => (v/10).toFixed(1),   label: v => 'TARGET_ALT → ' + (v/10).toFixed(1) + 'm',     ms: 150 },
  { id: 'kp',          char: 'kp',         enc: v => u16buf(v),  fmt: v => (v/10).toFixed(1),   label: v => 'HOLD_KP → ' + (v/10).toFixed(1),              ms: 150 },
  { id: 'ki',          char: 'ki',         enc: v => u16buf(v),  fmt: v => (v/10).toFixed(1),   label: v => 'HOLD_KI → ' + (v/10).toFixed(1),              ms: 150 },
  { id: 'kd',          char: 'kd',         enc: v => u16buf(v),  fmt: v => (v/10).toFixed(1),   label: v => 'HOLD_KD → ' + (v/10).toFixed(1),              ms: 150 },
  { id: 'punch-start', char: 'punchStart', enc: v => u32buf(v),  fmt: v => (v/1000).toFixed(1), label: v => 'PUNCH_START → ' + (v/1000).toFixed(1) + 's',  ms: 150 },
  { id: 'punch-throt', char: 'punchThrot', enc: v => u16buf(v),  fmt: v => String(v),           label: v => 'PUNCH_THROTTLE → ' + v,                       ms: 150 },
];

SLIDERS.forEach(({ id, char: charKey, enc, fmt, label, ms }) => {
  const slider = document.getElementById('slider-' + id);
  const valEl  = document.getElementById('val-' + id);
  if (!slider) return;
  slider.addEventListener('input', function() {
    const v = parseInt(this.value);
    if (valEl) valEl.textContent = fmt(v);
    updateFill(this);
    debounce(id, () => {
      if (!connected || !chars[charKey]) return;
      bleWrite(async () => {
        try { await chars[charKey].writeValue(enc(v)); log(label(v), 'ok'); }
        catch(e) { log('Write failed: ' + e.message, 'err'); }
      });
    }, ms);
  });
});

// ── TELEMETRY ────────────────────────────────────────────
const DBG_MAX_M = 3.5;  // bar full-scale (m) — matches safety ceiling

function signedMs(cms) {
  const v = cms / 100;
  return (v >= 0 ? '+' : '') + v.toFixed(2) + ' m/s';
}
function varColor(cms) {
  if (cms >  15) return 'var(--amber)';
  if (cms < -15) return 'var(--cyan)';
  return 'var(--text-mid)';
}
function thrOffColor(off) {
  if (off >  10) return 'var(--amber)';
  if (off < -10) return 'var(--cyan)';
  return 'var(--text-mid)';
}

function el(id) { return document.getElementById(id); }

let lastTelemetryDumpMs = 0;

function onTelemetry(e) {
  const dv         = e.target.value;
  const nowMs      = performance.now();
  if (nowMs - lastTelemetryDumpMs >= 1000) {
    lastTelemetryDumpMs = nowMs;
    const bytes = Array.from(new Uint8Array(dv.buffer, dv.byteOffset, dv.byteLength))
      .map(v => v.toString(16).padStart(2, '0'))
      .join(' ');
    const mspRawHex = dv.byteLength >= 28
      ? Array.from(new Uint8Array(dv.buffer, dv.byteOffset + 22, 6))
          .map(v => v.toString(16).padStart(2, '0'))
          .join(' ')
      : '';
    console.log('Telemetry length:', dv.byteLength, 'bytes:', bytes, 'mspAlt:', mspRawHex);
  }

  const altCm      = dv.getInt32(0, true);
  const relCm      = dv.getInt32(4, true);
  const stateId    = dv.getUint8(8);
  const throttle   = dv.getUint16(9, true);
  const varioCs    = dv.byteLength >= 13 ? dv.getInt16(11, true) : 0;
  const filtVarCs  = dv.byteLength >= 15 ? dv.getInt16(13, true) : 0;
  const setptCm    = dv.byteLength >= 17 ? dv.getInt16(15, true) : 0;
  const guardPhase = dv.byteLength >= 18 ? dv.getUint8(17)       : 2;
  const fcVarioCs  = dv.byteLength >= 20 ? dv.getInt16(18, true) : varioCs;
  const derivVarCs = dv.byteLength >= 22 ? dv.getInt16(20, true) : 0;

  const altM     = (altCm / 100).toFixed(2);
  const relValid = Math.abs(relCm) <= 10000000;
  const relM     = relValid ? (relCm / 100).toFixed(2) : '---';
  const relMNum  = relValid ? relCm / 100 : 0;
  const setptM   = setptCm / 100;
  const altErrM  = setptM - relMNum;

  const isIdle    = stateId === 0;
  const isCal     = stateId === 7;
  const isAltHold = stateId === 10;
  const color     = STATE_COLORS[stateId] || 'var(--text)';

  // ── State strip ─────────────────────────────────────
  const strip = el('state-strip');
  strip.classList.toggle('active', !isIdle);
  strip.style.borderColor = isIdle ? '' : color;
  el('strip-state').textContent = STATE_NAMES[stateId] || '?';
  el('strip-state').style.color = color;
  el('strip-alt').textContent   = relValid ? 'ALT ' + relM + ' m' : 'ALT ---';
  el('strip-throt').textContent = stateId === 8
    ? 'DESCENT ' + (varioCs / 100).toFixed(2) + ' m/s'
    : 'THROT ' + throttle;

  // ── Cal progress panel ───────────────────────────────
  el('cal-panel').classList.toggle('active', isCal);
  if (isCal) {
    const altFill = el('cal-alt-fill');
    altFill.style.width      = (Math.max(0, Math.min(relCm, 50)) / 50 * 100) + '%';
    altFill.style.background = relCm >= CAL_LIFTOFF_CM ? 'var(--green)' : 'var(--cyan)';
    el('cal-alt-val').textContent   = relCm + ' cm';
    el('cal-throt-fill').style.width = Math.max(0, Math.min((throttle - CAL_MIN_THROT) / (CAL_MAX_THROT - CAL_MIN_THROT) * 100, 100)) + '%';
    el('cal-throt-val').textContent = throttle;
  }

  // ── ALTITUDE section ─────────────────────────────────
  el('d-abs').textContent = altM;
  el('d-rel').textContent = relM;
  el('d-abs').style.color = altCm !== 0 ? 'var(--green)' : 'var(--text-mid)';
  el('d-rel').style.color = !relValid   ? 'var(--text-mid)' :
                            relCm > 20  ? 'var(--green)' :
                            relCm < -20 ? 'var(--amber)'  : '#fff';

  // Setpt / alt bars — ALT_HOLD only
  el('d-bars').style.display = isAltHold ? '' : 'none';
  if (isAltHold) {
    const barPct = v => Math.max(0, Math.min(v / DBG_MAX_M * 100, 100)).toFixed(1) + '%';
    el('d-sf').style.width = barPct(setptM);
    el('d-af').style.width = barPct(Math.max(0, relMNum));
    el('d-sv').textContent = setptM.toFixed(2) + 'm';
    el('d-av').textContent = relM + 'm';
  }

  // Vario — always
  el('d-vr').textContent = signedMs(varioCs);
  el('d-vf').textContent = signedMs(filtVarCs);
  el('d-vfc').textContent = signedMs(fcVarioCs);
  el('d-vd').textContent = signedMs(derivVarCs);
  el('d-vr').style.color = varColor(varioCs);
  el('d-vf').style.color = varColor(filtVarCs);
  el('d-vfc').style.color = varColor(fcVarioCs);
  el('d-vd').style.color = varColor(derivVarCs);

  // Alt error — shown in ALT_HOLD; "—" otherwise
  if (isAltHold) {
    const sign = altErrM >= 0 ? '+' : '';
    el('d-ae').textContent = sign + altErrM.toFixed(2) + 'm';
    el('d-ae').style.color = Math.abs(altErrM) < 0.10 ? 'var(--green)'
                           : Math.abs(altErrM) < 0.50 ? 'var(--amber)' : 'var(--red)';
  } else {
    el('d-ae').textContent = '—';
    el('d-ae').style.color = 'var(--text-dim)';
  }

  // ── CONTROL section ──────────────────────────────────
  const hoverThrot = parseInt(el('slider-hover').value) || 1270;
  const thrOff     = throttle - hoverThrot;
  el('d-thr').textContent  = throttle + 'µs';
  el('d-to').textContent   = (thrOff >= 0 ? '+' : '') + thrOff + 'µs';
  el('d-to').style.color   = thrOffColor(thrOff);
  el('d-st').textContent   = STATE_NAMES[stateId] || String(stateId);
  el('d-st').style.color   = color;

  // Guard phase pills — ALT_HOLD only
  el('d-phases').style.display = isAltHold ? '' : 'none';
  if (isAltHold) {
    [0, 1, 2].forEach(i => el('d-ph' + i).classList.toggle('active', guardPhase === i));
  }

  // ── HOVER THROTTLE — always visible on FLIGHT tab ───
  el('d-hover').style.display = '';

  // ── ACTIVE GAINS — ALT_HOLD only ─────────────────────
  el('d-gains').style.display = isAltHold ? '' : 'none';
  if (isAltHold) {
    el('d-kp').textContent = (parseInt(el('slider-kp').value) / 10).toFixed(1);
    el('d-kd').textContent = (parseInt(el('slider-kd').value) / 10).toFixed(1);
    el('d-ki').textContent = (parseInt(el('slider-ki').value) / 10).toFixed(1);
  }

  // ── Tab auto-switch ───────────────────────────────────
  if ((stateId === 10 || stateId === 7) && prevStateId !== stateId) {
    switchTab('pane-flight');
  }

  // ── Cal result notification (7 → anything) ────────────
  if (prevStateId === 7 && stateId !== 7) {
    const hs = el('slider-hover');
    if (hs) { hs.value = throttle; el('val-hover').textContent = throttle; updateFill(hs); }
    el('cal-result-val').textContent = throttle + ' µs';
    el('cal-result').classList.add('active');
    el('d-hover').classList.add('live');
  }
  if (isIdle) {
    el('cal-result').classList.remove('active');
    el('d-hover').classList.remove('live');
  }

  prevStateId = stateId;
}

// Init slider fills on page load
document.querySelectorAll('input[type=range]').forEach(updateFill);
