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
const ANGLE_MODE_UUID    = 'ab0828c1-198e-4351-b779-901fa0e0371e';
const FLIGHT_LOG_OFFSET_UUID = 'ab0828c2-198e-4351-b779-901fa0e0371e';
const FLIGHT_LOG_CHUNK_UUID  = 'ab0828c3-198e-4351-b779-901fa0e0371e';
const ALT_RAMP_RATE_UUID     = 'ab0828c4-198e-4351-b779-901fa0e0371e';
const MAX_CLIMB_TEST_UUID    = 'ab0828c5-198e-4351-b779-901fa0e0371e';
const MAX_DESCENT_TEST_UUID  = 'ab0828c6-198e-4351-b779-901fa0e0371e';
const BF_GROUND_EFFECT_UUID  = 'ab0828c7-198e-4351-b779-901fa0e0371e';
const MISSION_TYPE_UUID      = 'ab0828c8-198e-4351-b779-901fa0e0371e';
const PUNCH_YAW_UUID         = 'ab0828ca-198e-4351-b779-901fa0e0371e';
const HOVER_TEST_YAW_UUID     = 'ab0828cb-198e-4351-b779-901fa0e0371e';

// Single source of truth for state classification — replaces what used to be
// three separate things (a name array, a color array, and inline isX booleans
// recomputed in onTelemetry every tick). `phase` keys a CSS var (--<phase>)
// used for text/border/tint color; `timeline` keys which Mission Timeline
// segment lights up (null = state isn't part of the 5-step mission sequence).
const STATE_META = [
  /* 0  IDLE           */ { name: 'IDLE',           phase: 'neutral-phase', timeline: null,     isIdle: true },
  /* 1  ARMING         */ { name: 'ARMING',         phase: 'neutral-phase', timeline: 'arm' },
  /* 2  SPRINTING      */ { name: 'SPRINTING',      phase: 'caution',       timeline: 'sprint',  isMission: true },
  /* 3  HOLDING        */ { name: 'HOLDING',        phase: 'live',          timeline: 'hold',    isMission: true },
  /* 4  PUNCHING       */ { name: 'PUNCHING',       phase: 'armed',         timeline: 'punch',   isMission: true },
  /* 5  CUT            */ { name: 'CUT',            phase: 'safe',         timeline: 'land',    isMission: true },
  /* 6  HOVER_TEST     */ { name: 'HOVER TEST',     phase: 'live',          timeline: null,      isTest: true },
  /* 7  LANDING        */ { name: 'LANDING',        phase: 'safe',          timeline: 'land',    isLanding: true },
  /* 8  DONE           */ { name: 'DONE',           phase: 'neutral-phase', timeline: null,      isDone: true },
  /* 9  ALT_HOLD       */ { name: 'ALT HOLD',       phase: 'live',          timeline: null,      isTest: true },
];
function stateMeta(id) { return STATE_META[id] || STATE_META[0]; }
function phaseVar(phase) { return 'var(--' + phase + ')'; }

const ALT_SOURCE_NAMES = ['BARO', 'TOF', 'BLEND', 'TOF HOLD'];
const VARIO_SOURCE_NAMES = ['DERIVED', 'BF VARIO', 'KF'];
const PARAM_PRESET_KEY = 'quad-tuner-param-preset-v4-4in-autorotor-1400';

const CMD_HOVER_TEST     = 1;
const CMD_START_MISSION  = 2;
const CMD_DISARM         = 3;
const CMD_ALT_HOLD       = 5;
const CMD_KILL           = 6;


let chars = {};
let device = null, connected = false;
let benchMode = 0;
let angleMode = 0;
let missionType = 1;
let prevStateId = -1;
let suppressPresetAutosave = false;
let missionStartMs = null; // performance.now() timestamp; null while not armed

// Serialise all BLE writes — prevents "GATT operation already in progress"
let _bleQ = Promise.resolve();
function bleWrite(fn) { _bleQ = _bleQ.then(fn).catch(() => {}); }

const connectBtn = document.getElementById('connect-btn');
const statusDot  = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const logEl      = document.getElementById('log');
const benchBtn   = document.getElementById('btn-bench-mode');
const angleBtn   = document.getElementById('btn-angle-mode');
const missionTypeBtn = document.getElementById('btn-mission-type');

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
  ['sprint','cutoff','target-alt','alt-hold-target','kp','ki','kd','alt-ramp','max-climb','max-descent','bf-ground-effect','punch-start','punch-throt','punch-yaw'].forEach(id => {
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
    chars.punchYaw   = await service.getCharacteristic(PUNCH_YAW_UUID);
    chars.hoverTestYaw = await service.getCharacteristic(HOVER_TEST_YAW_UUID);
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
    chars.angleMode  = await service.getCharacteristic(ANGLE_MODE_UUID);
    chars.missionType = await service.getCharacteristic(MISSION_TYPE_UUID);
    chars.telemetry  = await service.getCharacteristic(TELEMETRY_UUID);
    chars.logOffset  = await service.getCharacteristic(FLIGHT_LOG_OFFSET_UUID);
    chars.logChunk   = await service.getCharacteristic(FLIGHT_LOG_CHUNK_UUID);
    chars.altRamp    = await service.getCharacteristic(ALT_RAMP_RATE_UUID);
    chars.maxClimb   = await service.getCharacteristic(MAX_CLIMB_TEST_UUID);
    chars.maxDescent = await service.getCharacteristic(MAX_DESCENT_TEST_UUID);
    chars.bfGroundEffect = await service.getCharacteristic(BF_GROUND_EFFECT_UUID);

    await chars.telemetry.startNotifications();
    chars.telemetry.addEventListener('characteristicvaluechanged', onTelemetry);

    await readAll();

    connected = true;
    setStatus('connected', device.name);
    connectBtn.textContent = 'Disconnect';
    connectBtn.classList.add('active');
    enableAll(true);
    document.getElementById('judge-view-btn').disabled = false;
    updatePresetButtons();
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
  angleMode = 0;
  missionType = 1;
  _bleQ = Promise.resolve();
  setBenchButton();
  setAngleButton();
  setMissionTypeButton();
  setStatus('', 'disconnected');
  connectBtn.textContent = 'Connect to Quad-Tuner';
  connectBtn.classList.remove('active');
  enableAll(false);
  document.getElementById('judge-view-btn').disabled = true;
  el('judge-view').classList.remove('active');
  missionStartMs = null;
  updatePresetButtons();
  log('Disconnected', 'err');
}

// ── READ ALL PARAMS ──────────────────────────────────────
async function readAll() {
  try {
    const hover      = new DataView((await chars.hover.readValue()).buffer).getUint16(0, true);
    const sprint     = new DataView((await chars.sprint.readValue()).buffer).getUint16(0, true);
    const punchYaw   = new DataView((await chars.punchYaw.readValue()).buffer).getUint16(0, true);
    const hoverTestYaw = new DataView((await chars.hoverTestYaw.readValue()).buffer).getUint16(0, true);
    const cutoffRaw  = new DataView((await chars.cutoff.readValue()).buffer).getUint16(0, true);
    const kpRaw      = new DataView((await chars.kp.readValue()).buffer).getUint16(0, true);
    const kiRaw      = new DataView((await chars.ki.readValue()).buffer).getUint16(0, true);
    const kdRaw      = new DataView((await chars.kd.readValue()).buffer).getUint16(0, true);
    const targetAlt  = new DataView((await chars.targetAlt.readValue()).buffer).getUint16(0, true);
    const altHoldTarget = new DataView((await chars.altHoldTarget.readValue()).buffer).getUint16(0, true);
    const altRamp    = new DataView((await chars.altRamp.readValue()).buffer).getUint16(0, true);
    const maxClimb   = new DataView((await chars.maxClimb.readValue()).buffer).getUint16(0, true);
    const maxDescent = new DataView((await chars.maxDescent.readValue()).buffer).getUint16(0, true);
    const bfGroundEffect = new DataView((await chars.bfGroundEffect.readValue()).buffer).getUint16(0, true);
    const punchStart = new DataView((await chars.punchStart.readValue()).buffer).getUint32(0, true);
    const punchThrot = new DataView((await chars.punchThrot.readValue()).buffer).getUint16(0, true);
    benchMode        = new DataView((await chars.benchMode.readValue()).buffer).getUint8(0);
    angleMode        = new DataView((await chars.angleMode.readValue()).buffer).getUint8(0);
    missionType      = new DataView((await chars.missionType.readValue()).buffer).getUint8(0);

    setParam('hover',       hover,      v => v,                   v => v);
    setParam('sprint',      sprint,     v => v,                   v => v);
    setParam('punch-yaw',   punchYaw,   v => v,                   v => v);
    setParam('hover-yaw',   hoverTestYaw, v => v,                 v => v);
    setParam('cutoff',      cutoffRaw,  v => (v/100).toFixed(1),  v => v);
    setParam('kp',          kpRaw,      v => (v/10).toFixed(1),   v => v);
    setParam('ki',          kiRaw,      v => (v/10).toFixed(1),   v => v);
    setParam('kd',          kdRaw,      v => (v/10).toFixed(1),   v => v);
    setParam('target-alt',  targetAlt,  v => (v/10).toFixed(1),   v => v);
    setParam('alt-hold-target', altHoldTarget, v => (v/10).toFixed(1), v => v);
    setParam('alt-ramp',    altRamp,    v => (v/100).toFixed(2),  v => v);
    setParam('max-climb',   maxClimb,   v => (v/100).toFixed(2),  v => v);
    setParam('max-descent', maxDescent, v => (v/100).toFixed(2),  v => v);
    setParam('bf-ground-effect', bfGroundEffect, v => (v/100).toFixed(2), v => v);
    setParam('punch-start', punchStart, v => (v/1000).toFixed(1), v => v);
    setParam('punch-throt', punchThrot, v => v,                   v => v);

    setBenchButton();
    setAngleButton();
    setMissionTypeButton();
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

function setAngleButton() {
  angleBtn.textContent = angleMode ? 'Angle Mode: On' : 'Angle Mode: Off';
  angleBtn.classList.toggle('angle-on', !!angleMode);
}

function setMissionTypeButton() {
  missionTypeBtn.textContent = missionType ? 'Mission: Autorotor Cut' : 'Mission: Powered Land';
  missionTypeBtn.classList.toggle('autorotor-on', !!missionType);
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

function getSliderConfig(id) {
  return SLIDERS.find(s => s.id === id);
}

function readPreset() {
  try {
    const parsed = JSON.parse(localStorage.getItem(PARAM_PRESET_KEY) || 'null');
    return parsed && parsed.values ? parsed : null;
  } catch {
    return null;
  }
}

function updatePresetButtons() {
  const hasPreset = !!readPreset();
  const applyBtn = document.getElementById('btn-apply-preset');
  const clearBtn = document.getElementById('btn-clear-preset');
  if (applyBtn) applyBtn.disabled = !hasPreset || !connected;
  if (clearBtn) clearBtn.disabled = !hasPreset;
}

function collectPresetValues() {
  const values = {};
  SLIDERS.forEach(({ id }) => {
    const slider = document.getElementById('slider-' + id);
    if (slider) values[id] = parseInt(slider.value);
  });
  return values;
}

function saveBrowserPreset() {
  const preset = {
    savedAt: new Date().toISOString(),
    values: collectPresetValues()
  };
  localStorage.setItem(PARAM_PRESET_KEY, JSON.stringify(preset));
  updatePresetButtons();
  log('Browser preset saved', 'ok');
}

function autosaveBrowserPreset() {
  if (suppressPresetAutosave) return;
  const preset = {
    savedAt: new Date().toISOString(),
    values: collectPresetValues()
  };
  localStorage.setItem(PARAM_PRESET_KEY, JSON.stringify(preset));
  updatePresetButtons();
}

async function writeSliderValue(id, value, sourceLabel = 'preset') {
  const cfg = getSliderConfig(id);
  const slider = document.getElementById('slider-' + id);
  const valEl = document.getElementById('val-' + id);
  if (!cfg || !slider || !chars[cfg.char]) return false;
  const min = parseInt(slider.min);
  const max = parseInt(slider.max);
  const stepped = Math.max(min, Math.min(max, parseInt(value)));
  suppressPresetAutosave = true;
  slider.value = stepped;
  if (valEl) valEl.textContent = cfg.fmt(stepped);
  updateFill(slider);
  suppressPresetAutosave = false;
  await chars[cfg.char].writeValue(cfg.enc(stepped));
  log(sourceLabel + ': ' + cfg.label(stepped), 'ok');
  return true;
}

async function applyBrowserPreset() {
  if (!connected) return;
  const preset = readPreset();
  if (!preset) {
    log('No browser preset saved', 'err');
    updatePresetButtons();
    return;
  }
  const entries = Object.entries(preset.values || {});
  let count = 0;
  await bleWrite(async () => {
    for (const [id, value] of entries) {
      try {
        if (await writeSliderValue(id, value, 'preset')) count++;
      } catch(e) {
        log('Preset write failed for ' + id + ': ' + e.message, 'err');
      }
    }
  });
  log('Applied browser preset (' + count + ' values)', count ? 'ok' : 'err');
}

function clearBrowserPreset() {
  localStorage.removeItem(PARAM_PRESET_KEY);
  updatePresetButtons();
  log('Browser preset cleared', 'ok');
}

// ── COMMAND BUTTONS ──────────────────────────────────────
document.getElementById('btn-hover-test').addEventListener('click', () =>
  sendCommand(CMD_HOVER_TEST, 'Hover test command sent'));


document.getElementById('btn-alt-hold').addEventListener('click', () =>
  sendCommand(CMD_ALT_HOLD, 'Alt hold command sent'));

document.getElementById('btn-start-mission').addEventListener('click', () =>
  sendCommand(CMD_START_MISSION, 'Mission start command sent'));

document.getElementById('btn-disarm').addEventListener('click', () =>
  sendCommand(CMD_DISARM, 'Land command sent'));

document.getElementById('btn-kill').addEventListener('click', () =>
  sendCommand(CMD_KILL, 'Kill command sent'));

document.getElementById('mt-land').addEventListener('click', () =>
  sendCommand(CMD_DISARM, 'Land command sent (timeline)'));

document.getElementById('mt-kill').addEventListener('click', () =>
  sendCommand(CMD_KILL, 'Kill command sent (timeline)'));

// ── JUDGE VIEW ───────────────────────────────────────────
const judgeViewBtn = document.getElementById('judge-view-btn');
judgeViewBtn.addEventListener('click', () => el('judge-view').classList.add('active'));
document.getElementById('jv-exit').addEventListener('click', () => el('judge-view').classList.remove('active'));

document.getElementById('btn-sync').addEventListener('click', async () => {
  if (!connected) return;
  log('Syncing values from ESP32...');
  await readAll();
});

document.getElementById('btn-save-preset').addEventListener('click', saveBrowserPreset);
document.getElementById('btn-apply-preset').addEventListener('click', applyBrowserPreset);
document.getElementById('btn-clear-preset').addEventListener('click', clearBrowserPreset);

document.getElementById('btn-download-log').addEventListener('click', async () => {
  if (!connected || !chars.logOffset || !chars.logChunk) return;
  try {
    log('Downloading last flight log...');
    const decoder = new TextDecoder();
    let offset = 0;
    let text = '';
    for (let i = 0; i < 400; i++) {
      await chars.logOffset.writeValue(u16buf(offset));
      const value = await chars.logChunk.readValue();
      if (!value.byteLength) break;
      const bytes = new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
      text += decoder.decode(bytes, { stream: true });
      offset += value.byteLength;
      if (value.byteLength < 220) break;
    }
    text += decoder.decode();
    if (!text.length) {
      log('No flight log stored yet', 'err');
      return;
    }
    const blob = new Blob([text], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    const stamp = new Date().toISOString().replace(/[:.]/g, '-');
    a.href = url;
    a.download = 'quad-flight-log-' + stamp + '.csv';
    document.body.appendChild(a);
    a.click();
    a.remove();
    URL.revokeObjectURL(url);
    log('Downloaded ' + offset + ' bytes of flight log', 'ok');
  } catch(e) {
    log('Flight log download failed: ' + e.message, 'err');
  }
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

angleBtn.addEventListener('click', () => {
  if (!connected) return;
  const next = angleMode ? 0 : 1;
  bleWrite(async () => {
    try {
      await chars.angleMode.writeValue(u8buf(next));
      angleMode = new DataView((await chars.angleMode.readValue()).buffer).getUint8(0);
      setAngleButton();
      log(angleMode ? 'Angle mode enabled' : 'Angle mode disabled', 'ok');
    } catch(e) {
      log('Angle mode failed: ' + e.message, 'err');
    }
  });
});

missionTypeBtn.addEventListener('click', () => {
  if (!connected) return;
  const next = missionType ? 0 : 1;
  bleWrite(async () => {
    try {
      await chars.missionType.writeValue(u8buf(next));
      missionType = new DataView((await chars.missionType.readValue()).buffer).getUint8(0);
      setMissionTypeButton();
      log(missionType ? 'Mission type: autorotor cut' : 'Mission type: powered land', missionType ? 'err' : 'ok');
    } catch(e) {
      log('Mission type change failed: ' + e.message, 'err');
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
  { id: 'alt-ramp',    char: 'altRamp',    enc: v => u16buf(v),  fmt: v => (v/100).toFixed(2),  label: v => 'ALT_RAMP_RATE -> ' + (v/100).toFixed(2) + 'm/s', ms: 150 },
  { id: 'max-climb',   char: 'maxClimb',   enc: v => u16buf(v),  fmt: v => (v/100).toFixed(2),  label: v => 'MAX_CLIMB_TEST -> ' + (v/100).toFixed(2) + 'm/s', ms: 150 },
  { id: 'max-descent', char: 'maxDescent', enc: v => u16buf(v),  fmt: v => (v/100).toFixed(2),  label: v => 'MAX_DESCENT_TEST -> ' + (v/100).toFixed(2) + 'm/s', ms: 150 },
  { id: 'bf-ground-effect', char: 'bfGroundEffect', enc: v => u16buf(v), fmt: v => (v/100).toFixed(2), label: v => 'BF_GROUND_EFFECT -> ' + (v/100).toFixed(2) + 'm', ms: 150 },
  { id: 'hover',       char: 'hover',      enc: v => u16buf(v),  fmt: v => String(v),           label: v => 'HOVER_THROTTLE → ' + v,                        ms: 30  },
  { id: 'sprint',      char: 'sprint',     enc: v => u16buf(v),  fmt: v => String(v),           label: v => 'SPRINT_THROTTLE → ' + v,                       ms: 150 },
  { id: 'punch-yaw',   char: 'punchYaw',   enc: v => u16buf(v),  fmt: v => String(v),           label: v => 'PUNCH_YAW → ' + v + ' (CW)',                  ms: 100 },
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
    autosaveBrowserPreset();
    debounce(id, () => {
      if (!connected || !chars[charKey]) return;
      bleWrite(async () => {
        try { await chars[charKey].writeValue(enc(v)); log(label(v), 'ok'); }
        catch(e) { log('Write failed: ' + e.message, 'err'); }
      });
    }, ms);
  });
});

const hoverYawSlider = document.getElementById('slider-hover-yaw');
if (hoverYawSlider) {
  hoverYawSlider.addEventListener('input', function() {
    const v = parseInt(this.value);
    setParam('hover-yaw', v, x => x, x => x);
    debounce('hover-yaw', () => {
      if (!connected || !chars.hoverTestYaw) return;
      bleWrite(async () => {
        try { await chars.hoverTestYaw.writeValue(u16buf(v)); log('HOVER TEST YAW → ' + v + ' (CW)', 'ok'); }
        catch(e) { log('Write failed: ' + e.message, 'err'); }
      });
    }, 30);
  });
}

updatePresetButtons();

// ── TELEMETRY ────────────────────────────────────────────
const DBG_MAX_M = 3.5;  // bar full-scale (m) — matches safety ceiling

function signedMs(cms) {
  const v = cms / 100;
  return (v >= 0 ? '+' : '') + v.toFixed(2) + ' m/s';
}
function formatElapsed(startMs) {
  if (startMs === null) return '0:00.0';
  const s = (performance.now() - startMs) / 1000;
  const m = Math.floor(s / 60);
  const rem = (s - m * 60).toFixed(1).padStart(4, '0');
  return m + ':' + rem;
}
// Ticks elapsed-time readouts independently of the 5-10Hz telemetry rate
// so the stopwatch reads smoothly instead of stepping in telemetry-sized jumps.
setInterval(() => {
  if (missionStartMs === null) return;
  el('mt-time').textContent = formatElapsed(missionStartMs);
  el('jv-time').textContent = formatElapsed(missionStartMs);
}, 100);
function varColor(cms) {
  if (cms >  15) return 'var(--caution)';
  if (cms < -15) return 'var(--live)';
  return 'var(--ink-mid)';
}
function thrOffColor(off) {
  if (off >  10) return 'var(--caution)';
  if (off < -10) return 'var(--live)';
  return 'var(--ink-mid)';
}

function el(id) { return document.getElementById(id); }

let lastTelemetryDumpMs = 0;

function onTelemetry(e) {
  const dv         = e.target.value;
  const nowMs      = performance.now();
  const compactTelemetry = dv.byteLength >= 28 && dv.byteLength < 77;
  if (nowMs - lastTelemetryDumpMs >= 1000) {
    lastTelemetryDumpMs = nowMs;
    const bytes = Array.from(new Uint8Array(dv.buffer, dv.byteOffset, dv.byteLength))
      .map(v => v.toString(16).padStart(2, '0'))
      .join(' ');
    const mspRawHex = !compactTelemetry && dv.byteLength >= 28
      ? Array.from(new Uint8Array(dv.buffer, dv.byteOffset + 22, 6))
          .map(v => v.toString(16).padStart(2, '0'))
          .join(' ')
      : '';
    console.log('Telemetry length:', dv.byteLength, 'compact:', compactTelemetry, 'bytes:', bytes, 'mspAlt:', mspRawHex);
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
  const angleAux   = dv.byteLength >= 30 ? dv.getUint16(28, true) : (angleMode ? 1800 : 1000);
  const tofCm      = dv.byteLength >= 32 ? dv.getInt16(30, true) : -1;
  const compactDiagMask = compactTelemetry ? dv.getUint16(22, true) : 0;
  const compactSensorFlags = compactTelemetry ? dv.getUint8(24) : 0;
  const tofWeight  = compactTelemetry ? dv.getUint8(25) : (dv.byteLength >= 33 ? dv.getUint8(32) : 0);
  const tofValid   = compactTelemetry ? !!(compactSensorFlags & 0x02) : (dv.byteLength >= 34 ? !!dv.getUint8(33) : false);
  const baroCm     = !compactTelemetry && dv.byteLength >= 38 ? dv.getInt32(34, true) : altCm;
  const altSource  = compactTelemetry ? dv.getUint8(26) : (dv.byteLength >= 39 ? dv.getUint8(38) : (tofWeight > 70 ? 1 : (tofWeight > 0 ? 2 : 0)));
  const cbaroCm    = !compactTelemetry && dv.byteLength >= 43 ? dv.getInt32(39, true) : baroCm;
  const fcDiagMask = compactTelemetry ? compactDiagMask : (dv.byteLength >= 45 ? dv.getUint16(43, true) : 0);
  const fcAccX     = dv.byteLength >= 47 ? dv.getInt16(45, true) : 0;
  const fcAccY     = dv.byteLength >= 49 ? dv.getInt16(47, true) : 0;
  const fcAccZ     = dv.byteLength >= 51 ? dv.getInt16(49, true) : 0;
  const fcGyroX    = dv.byteLength >= 53 ? dv.getInt16(51, true) : 0;
  const fcGyroY    = dv.byteLength >= 55 ? dv.getInt16(53, true) : 0;
  const fcGyroZ    = dv.byteLength >= 57 ? dv.getInt16(55, true) : 0;
  const fcRoll     = dv.byteLength >= 59 ? dv.getInt16(57, true) / 10 : 0;
  const fcPitch    = dv.byteLength >= 61 ? dv.getInt16(59, true) / 10 : 0;
  const fcYaw      = dv.byteLength >= 63 ? dv.getInt16(61, true) : 0;
  const fcCycle    = dv.byteLength >= 65 ? dv.getUint16(63, true) : 0;
  const fcSensors  = dv.byteLength >= 67 ? dv.getUint16(65, true) : 0;
  const fcRcThr    = dv.byteLength >= 69 ? dv.getUint16(67, true) : 0;
  const fcRcArm    = dv.byteLength >= 71 ? dv.getUint16(69, true) : 0;
  const fcRcAngle  = dv.byteLength >= 73 ? dv.getUint16(71, true) : 0;
  const fcVbat     = dv.byteLength >= 74 ? dv.getUint8(73) / 10 : 0;
  const fcAmps     = dv.byteLength >= 76 ? dv.getInt16(74, true) / 100 : 0;
  const varioSource = compactTelemetry ? dv.getUint8(27) : (dv.byteLength >= 77 ? dv.getUint8(76) : 0);

  const altM     = (altCm / 100).toFixed(2);
  const relValid = Math.abs(relCm) <= 10000000;
  const relM     = relValid ? (relCm / 100).toFixed(2) : '---';
  const relMNum  = relValid ? relCm / 100 : 0;
  const setptM   = setptCm / 100;
  const altErrM  = setptM - relMNum;

  const meta      = stateMeta(stateId);
  const isIdle    = !!meta.isIdle;
  const isAltHold = stateId === 9;
  const isDone    = !!meta.isDone;
  const isLanding = !!meta.isLanding;
  const isTestMode = !!meta.isTest;
  const isMissionMode = !!meta.isMission;
  const isArmed   = !isIdle && !isDone;
  const color     = phaseVar(meta.phase);

  // ── Mission stopwatch ────────────────────────────────
  // Client-side only — starts when the state leaves IDLE/DONE, clears on
  // return. Ticked independently of telemetry rate by renderElapsed() below.
  if (isArmed && missionStartMs === null) missionStartMs = performance.now();
  if (!isArmed) missionStartMs = null;

  // ── Mission timeline ──────────────────────────────────
  const timeline = el('mission-timeline');
  timeline.classList.toggle('active', isArmed);
  timeline.style.borderColor = isArmed ? color : '';
  document.querySelectorAll('.mt-phase').forEach(p =>
    p.classList.toggle('active', p.dataset.phase === meta.timeline));
  el('mt-state').textContent = meta.name;
  el('mt-state').style.color = color;
  el('mt-time').textContent = formatElapsed(missionStartMs);
  el('mt-throttle').textContent = isLanding
    ? (varioCs / 100).toFixed(2) + ' m/s'
    : throttle + ' µs';
  const mtLand = el('mt-land');
  mtLand.disabled = !(isTestMode || isLanding);
  mtLand.textContent = isLanding ? 'Landing' : (isMissionMode ? 'No Land' : 'Land');
  el('btn-hover-test').disabled = !connected || (!isIdle && !isDone);
  el('btn-alt-hold').disabled = !connected || (!isIdle && !isDone);
  el('btn-start-mission').disabled = !connected || (!isIdle && !isDone);
  el('btn-mission-type').disabled = !connected || (!isIdle && !isDone);
  el('btn-disarm').disabled = !connected || isMissionMode;
  el('btn-kill').disabled = !connected;
  el('btn-download-log').disabled = !connected || (!isIdle && !isDone);

  // ── Judge View ────────────────────────────────────────
  el('jv-phase').textContent = meta.name;
  el('jv-phase').style.color = color;
  const jvArmed = el('jv-armed');
  jvArmed.textContent = isArmed ? 'ARMED' : 'DISARMED';
  jvArmed.classList.toggle('is-armed', isArmed);
  jvArmed.classList.toggle('is-safe', !isArmed);
  el('jv-time').textContent = formatElapsed(missionStartMs);
  el('jv-throttle').innerHTML = throttle + '<span class="jv-unit">µs</span>';
  const judgeView = el('judge-view');
  Array.from(judgeView.classList).forEach(c => { if (c.startsWith('phase-')) judgeView.classList.remove(c); });
  judgeView.classList.add('phase-' + meta.phase);

  // ── ALTITUDE section ─────────────────────────────────
  el('d-abs').textContent = altM;
  el('d-rel').textContent = relM;
  el('d-abs').style.color = altCm !== 0 ? 'var(--safe)' : 'var(--ink-mid)';
  el('d-rel').style.color = !relValid   ? 'var(--ink-mid)' :
                            relCm > 20  ? 'var(--safe)' :
                            relCm < -20 ? 'var(--caution)'  : 'var(--ink)';

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
  el('d-vsrc').textContent = VARIO_SOURCE_NAMES[varioSource] || String(varioSource);
  el('d-vr').style.color = varColor(varioCs);
  el('d-vf').style.color = varColor(filtVarCs);
  el('d-vfc').style.color = varColor(fcVarioCs);
  el('d-vd').style.color = varColor(derivVarCs);
  el('d-vsrc').style.color = varioSource === 2 ? 'var(--live)'
                            : varioSource === 1 ? 'var(--safe)' : 'var(--caution)';
  el('d-tof').textContent = compactTelemetry
    ? (tofValid ? 'OK' : 'INVALID')
    : (tofValid && tofCm >= 0 ? (tofCm / 100).toFixed(2) + 'm' : 'INVALID');
  el('d-tw').textContent = tofWeight + '%';
  el('d-baro').textContent = compactTelemetry ? 'LIVE' : (baroCm / 100).toFixed(2) + 'm';
  el('d-src').textContent = ALT_SOURCE_NAMES[altSource] || String(altSource);
  el('d-cbaro').textContent = compactTelemetry ? 'COMPACT' : (cbaroCm / 100).toFixed(2) + 'm';
  el('d-tof').style.color = tofValid ? 'var(--live)' : 'var(--ink-dim)';
  el('d-tw').style.color = tofWeight > 70 ? 'var(--live)'
                         : tofWeight > 0  ? 'var(--caution)' : 'var(--ink-dim)';
  el('d-baro').style.color = tofWeight === 0 ? 'var(--safe)' : 'var(--ink-mid)';
  el('d-src').style.color = altSource === 1 || altSource === 3 ? 'var(--live)'
                          : altSource === 2 ? 'var(--caution)' : 'var(--safe)';
  el('d-cbaro').style.color = altSource === 0 ? 'var(--safe)' : 'var(--ink-mid)';

  // Alt error — shown in ALT_HOLD; "—" otherwise
  if (isAltHold) {
    const sign = altErrM >= 0 ? '+' : '';
    el('d-ae').textContent = sign + altErrM.toFixed(2) + 'm';
    el('d-ae').style.color = Math.abs(altErrM) < 0.10 ? 'var(--safe)'
                           : Math.abs(altErrM) < 0.50 ? 'var(--caution)' : 'var(--armed)';
  } else {
    el('d-ae').textContent = '—';
    el('d-ae').style.color = 'var(--ink-dim)';
  }

  // ── CONTROL section ──────────────────────────────────
  const hoverThrot = parseInt(el('slider-hover').value) || 1400;
  const thrOff     = throttle - hoverThrot;
  el('d-thr').textContent  = throttle + 'µs';
  el('d-to').textContent   = (thrOff >= 0 ? '+' : '') + thrOff + 'µs';
  el('d-to').style.color   = thrOffColor(thrOff);
  el('d-st').textContent   = meta.name;
  el('d-st').style.color   = color;
  el('d-ang').textContent  = angleAux + 'µs';
  el('d-am').textContent   = angleAux >= 1700 ? 'ON' : 'OFF';
  el('d-am').style.color   = angleAux >= 1700 ? 'var(--live)' : 'var(--ink-mid)';
  const maskHex = '0x' + fcDiagMask.toString(16).padStart(2, '0');
  el('d-fc-msp').textContent = maskHex;
  el('d-fc-msp').style.color = fcDiagMask === 0x1f ? 'var(--safe)'
                            : fcDiagMask ? 'var(--caution)' : 'var(--armed)';
  el('d-fc-att').textContent = fcDiagMask & 0x02
    ? (compactTelemetry ? 'ATT OK' : fcRoll.toFixed(1) + ',' + fcPitch.toFixed(1) + ',' + fcYaw)
    : 'NO ATT';
  el('d-fc-acc').textContent = fcDiagMask & 0x01
    ? (compactTelemetry ? 'IMU OK' : fcAccX + ',' + fcAccY + ',' + fcAccZ)
    : 'NO IMU';
  el('d-fc-gyro').textContent = fcDiagMask & 0x01
    ? (compactTelemetry ? 'GYRO OK' : fcGyroX + ',' + fcGyroY + ',' + fcGyroZ)
    : 'NO GYRO';
  el('d-fc-rc').textContent = fcDiagMask & 0x10
    ? (compactTelemetry ? 'RC OK' : fcRcThr + '/' + fcRcArm + '/' + fcRcAngle)
    : 'NO RC';
  el('d-fc-status').textContent = compactTelemetry
    ? ((compactSensorFlags & 0x01 ? 'ALT OK ' : 'ALT STALE ')
      + (compactSensorFlags & 0x04 ? 'ATT FRESH' : 'ATT STALE'))
    : ((fcDiagMask & 0x04 ? fcCycle + 'us s=' + fcSensors.toString(16) + ' ' : 'NO STATUS ')
      + (fcDiagMask & 0x08 ? fcVbat.toFixed(1) + 'V ' + fcAmps.toFixed(1) + 'A' : ''));

  // Guard phase pills — ALT_HOLD only
  el('d-phases').style.display = isAltHold ? '' : 'none';
  if (isAltHold) {
    [0, 1, 2].forEach(i => el('d-ph' + i).classList.toggle('active', guardPhase === i));
  }

  // ── HOVER THROTTLE — always visible on FLIGHT tab ───
  el('d-hover').style.display = '';
  // Visible before arming so the test command can be preset safely. Firmware
  // applies it only in HOVER_TEST; all other states continue sending neutral yaw.
  el('d-hover-yaw').style.display = '';

  // ── ACTIVE GAINS — ALT_HOLD only ─────────────────────
  el('d-gains').style.display = isAltHold ? '' : 'none';
  if (isAltHold) {
    el('d-kp').textContent = (parseInt(el('slider-kp').value) / 10).toFixed(1);
    el('d-kd').textContent = (parseInt(el('slider-kd').value) / 10).toFixed(1);
    el('d-ki').textContent = (parseInt(el('slider-ki').value) / 10).toFixed(1);
  }

  // ── Tab auto-switch ───────────────────────────────────
  if (stateId === 9 && prevStateId !== stateId) {
    switchTab('pane-flight');
  }

  prevStateId = stateId;
}

// Init slider fills on page load
document.querySelectorAll('input[type=range]').forEach(updateFill);
