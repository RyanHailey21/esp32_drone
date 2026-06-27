const SERVICE_UUID       = 'ab0828b1-198e-4351-b779-901fa0e0371e';
const HOVER_UUID         = 'ab0828b2-198e-4351-b779-901fa0e0371e';
const SPRINT_THROT_UUID  = 'ab0828b5-198e-4351-b779-901fa0e0371e';
const SPRINT_CUTOFF_UUID = 'ab0828b6-198e-4351-b779-901fa0e0371e';
const HOLD_KP_UUID       = 'ab0828b7-198e-4351-b779-901fa0e0371e';
const HOLD_KI_UUID       = 'ab0828bc-198e-4351-b779-901fa0e0371e';
const HOLD_KD_UUID       = 'ab0828be-198e-4351-b779-901fa0e0371e';
const TARGET_ALT_UUID    = 'ab0828bf-198e-4351-b779-901fa0e0371e';
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
  ['hover','sprint','cutoff','target-alt','kp','ki','kd','punch-start','punch-throt'].forEach(id => {
    const el = document.getElementById('param-' + id);
    if (el) el.classList.toggle('enabled', on);
  });
  document.getElementById('mission-controls').classList.toggle('enabled', on);
}

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

// Hover throttle — live (30ms debounce), min 1000 to zero throttle
document.getElementById('slider-hover').addEventListener('input', function() {
  const v = parseInt(this.value);
  document.getElementById('val-hover').textContent = v;
  updateFill(this);
  debounce('hover', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.hover.writeValue(u16buf(v)); log('HOVER_THROTTLE → ' + v, 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  }, 30);
});

document.getElementById('slider-sprint').addEventListener('input', function() {
  const v = parseInt(this.value);
  document.getElementById('val-sprint').textContent = v;
  updateFill(this);
  debounce('sprint', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.sprint.writeValue(u16buf(v)); log('SPRINT_THROTTLE → ' + v, 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  });
});

// Sprint cutoff — float x100 as uint16
document.getElementById('slider-cutoff').addEventListener('input', function() {
  const raw = parseInt(this.value);
  document.getElementById('val-cutoff').textContent = (raw / 100).toFixed(1);
  updateFill(this);
  debounce('cutoff', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.cutoff.writeValue(u16buf(raw)); log('SPRINT_CUTOFF → ' + (raw / 100).toFixed(1) + 'm', 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  });
});

// Target altitude — float x10 as uint16 (18.3m → 183)
document.getElementById('slider-target-alt').addEventListener('input', function() {
  const raw = parseInt(this.value);
  document.getElementById('val-target-alt').textContent = (raw / 10).toFixed(1);
  updateFill(this);
  debounce('targetAlt', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.targetAlt.writeValue(u16buf(raw)); log('TARGET_ALT → ' + (raw / 10).toFixed(1) + 'm', 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  });
});

// Alt loop Kp (outer P) — float x10 as uint16
document.getElementById('slider-kp').addEventListener('input', function() {
  const raw = parseInt(this.value);
  document.getElementById('val-kp').textContent = (raw / 10).toFixed(1);
  updateFill(this);
  debounce('kp', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.kp.writeValue(u16buf(raw)); log('HOLD_KP → ' + (raw / 10).toFixed(1), 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  });
});

// Hold Ki
document.getElementById('slider-ki').addEventListener('input', function() {
  const raw = parseInt(this.value);
  document.getElementById('val-ki').textContent = (raw / 10).toFixed(1);
  updateFill(this);
  debounce('ki', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.ki.writeValue(u16buf(raw)); log('HOLD_KI → ' + (raw / 10).toFixed(1), 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  });
});

// Speed loop Kp (inner P) — float x10 as uint16
document.getElementById('slider-kd').addEventListener('input', function() {
  const raw = parseInt(this.value);
  document.getElementById('val-kd').textContent = (raw / 10).toFixed(1);
  updateFill(this);
  debounce('kd', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.kd.writeValue(u16buf(raw)); log('HOLD_KD → ' + (raw / 10).toFixed(1), 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  });
});

// Punch start — uint32 ms
document.getElementById('slider-punch-start').addEventListener('input', function() {
  const v = parseInt(this.value);
  document.getElementById('val-punch-start').textContent = (v / 1000).toFixed(1);
  updateFill(this);
  debounce('punchStart', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.punchStart.writeValue(u32buf(v)); log('PUNCH_START → ' + (v / 1000).toFixed(1) + 's', 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  });
});

document.getElementById('slider-punch-throt').addEventListener('input', function() {
  const v = parseInt(this.value);
  document.getElementById('val-punch-throt').textContent = v;
  updateFill(this);
  debounce('punchThrot', () => {
    if (!connected) return;
    bleWrite(async () => {
      try { await chars.punchThrot.writeValue(u16buf(v)); log('PUNCH_THROTTLE → ' + v, 'ok'); }
      catch(e) { log('Write failed: ' + e.message, 'err'); }
    });
  });
});

// ── TELEMETRY ────────────────────────────────────────────
function onTelemetry(e) {
  const dv       = e.target.value;
  const altCm    = dv.getInt32(0, true);
  const relCm    = dv.getInt32(4, true);
  const stateId  = dv.getUint8(8);
  const throttle = dv.getUint16(9, true);
  const varioCs  = dv.byteLength >= 13 ? dv.getInt16(11, true) : 0;  // cm/s

  // Preflight panel (always visible)
  const altM = (altCm / 100).toFixed(2);
  const relValid = Math.abs(relCm) <= 10000000;  // >100 km = firmware mismatch
  const relM = relValid ? (relCm / 100).toFixed(2) : '---';
  document.getElementById('tel-alt').textContent   = altM + ' m  ↕  ' + relM + ' m';
  document.getElementById('tel-state').textContent = STATE_NAMES[stateId] ?? stateId;
  document.getElementById('tel-throt').textContent = throttle + ' µs';
  document.getElementById('tel-alt').style.color   = altCm !== 0 ? 'var(--green)' : 'var(--red)';

  const isIdle = stateId === 0;
  const isCal  = stateId === 7;
  const color  = STATE_COLORS[stateId] || 'var(--text)';

  // State strip — visible for any non-idle state
  const strip = document.getElementById('state-strip');
  strip.classList.toggle('active', !isIdle);
  strip.style.borderColor = isIdle ? '' : color;
  document.getElementById('strip-state').textContent = STATE_NAMES[stateId] || '?';
  document.getElementById('strip-state').style.color = color;
  const isLanding = stateId === 8;
  document.getElementById('strip-alt').textContent   = relValid ? 'ALT ' + relM + ' m' : 'ALT ---';
  document.getElementById('strip-throt').textContent = isLanding
    ? 'DESCENT ' + (varioCs / 100).toFixed(2) + ' m/s'
    : 'THROT ' + throttle;

  // Cal progress panel — only during AUTO_HOVER_CAL (state 7)
  document.getElementById('cal-panel').classList.toggle('active', isCal);
  if (isCal) {
    const clampedRel = Math.max(0, Math.min(relCm, 50));
    const altPct     = (clampedRel / 50) * 100;
    const altFill    = document.getElementById('cal-alt-fill');
    altFill.style.width      = altPct + '%';
    altFill.style.background = relCm >= CAL_LIFTOFF_CM ? 'var(--green)' : 'var(--cyan)';
    document.getElementById('cal-alt-val').textContent = relCm + ' cm';

    const throtPct = Math.max(0, Math.min((throttle - CAL_MIN_THROT) / (CAL_MAX_THROT - CAL_MIN_THROT) * 100, 100));
    document.getElementById('cal-throt-fill').style.width = throtPct + '%';
    document.getElementById('cal-throt-val').textContent  = throttle;
  }

  // Cal result notification — fires on 7→anything transition
  if (prevStateId === 7 && stateId !== 7) {
    const hoverSlider = document.getElementById('slider-hover');
    if (hoverSlider) {
      hoverSlider.value = throttle;
      document.getElementById('val-hover').textContent = throttle;
      updateFill(hoverSlider);
    }
    document.getElementById('cal-result-val').textContent = throttle + ' µs';
    document.getElementById('cal-result').classList.add('active');
    document.getElementById('param-hover').classList.add('live');
  }

  // Dismiss cal result when back to idle
  if (isIdle) {
    document.getElementById('cal-result').classList.remove('active');
    document.getElementById('param-hover').classList.remove('live');
  }

  prevStateId = stateId;
}

// Init slider fills on page load
document.querySelectorAll('input[type=range]').forEach(updateFill);
