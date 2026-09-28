// Drift angle meter UI: sensor plumbing, estimator backend (Web Worker), gauge and stats.
// Add ?demo (or ?demo=4 for 4x speed) to replay a simulated session instead of sensors.

import { requestMotionPermission, startMotion, startGnss, ScreenWakeLock } from './sensors.js';

const $ = (id) => document.getElementById(id);
const params = new URLSearchParams(location.search);
const DEMO = params.has('demo');
const DEMO_SPEED = Math.max(1, Number(params.get('demo')) || 1);

// ------------------------------------------------------------------ settings

const SETTINGS_KEY = 'drift-meter.settings';
const DEFAULTS = { phoneX: 1.6, wheelbase: 2.6 };

function loadSettings() {
  try {
    const s = JSON.parse(localStorage.getItem(SETTINGS_KEY) || '{}');
    return { ...DEFAULTS, ...s };
  } catch {
    return { ...DEFAULTS };
  }
}

function saveSettings(s) {
  try { localStorage.setItem(SETTINGS_KEY, JSON.stringify(s)); } catch { /* storage unavailable */ }
}

let settings = loadSettings();

function estimatorConfig() {
  return {
    phonePos: [settings.phoneX, 0, 0.5],
    outputPoint: [settings.wheelbase / 2, 0, 0.5],
  };
}

// ------------------------------------------------------------------ estimator backend

// Starts an estimator in a module Web Worker, or on the main thread where module
// workers are unsupported. Resolves once the estimator has answered.
async function createBackend(onOut, config) {
  const start = { type: 'start', config };
  const mainThread = async (replay) => {
    const { EstimatorHost } = await import('./host.js');
    const host = new EstimatorHost(onOut);
    replay.forEach((m) => host.handle(m));
    return { post: (m) => host.handle(m), stop() {} };
  };
  if (typeof Worker === 'undefined') return mainThread([start]);
  let w;
  try {
    w = new Worker(new URL('./worker.js', import.meta.url), { type: 'module' });
  } catch {
    return mainThread([start]);
  }
  return new Promise((resolve) => {
    let settled = false;
    const sent = [start]; // replayed on the main thread if the worker fails to start
    const api = {
      post: (m) => { w.postMessage(m); if (!settled) sent.push(m); },
      stop: () => w.terminate(),
    };
    w.onmessage = (e) => {
      if (!settled) { settled = true; sent.length = 0; resolve(api); }
      onOut(e.data.out);
    };
    const giveUp = () => {
      if (settled) return;
      settled = true;
      w.terminate();
      mainThread(sent).then(resolve);
    };
    w.onerror = giveUp;
    setTimeout(giveUp, 4000); // worker never answered
    w.postMessage(start);
  });
}

// ------------------------------------------------------------------ session state

const S = {
  running: false,
  backend: null,
  queue: [],
  flushTimer: null,
  motion: null,
  gnss: null,
  wake: new ScreenWakeLock(),
  out: null,
  lastFixAt: null,
  log: [],
  logMeta: null,
  drift: null,     // current drift { t0, peak, side, calmSince }
  best: null,      // { peak, side, dur }
  last: null,
  warn: '',
  demoTimer: null,
};

const LOG_MAX_EVENTS = 250000;

function record(ev) {
  S.queue.push(ev);
  S.log.push(ev);
  if (S.log.length > LOG_MAX_EVENTS) S.log.splice(0, S.log.length - LOG_MAX_EVENTS);
}

function onOut(out) {
  S.out = out;
  trackDrift(out);
}

// A drift starts when the estimator is in slide mode above 8°, and ends after half a
// second back below 4° or out of slide mode.
function trackDrift(o) {
  const b = o.valid && o.beta !== null ? Math.abs(o.beta) : 0;
  const side = o.valid && o.beta < 0 ? 'left' : 'right';
  if (!S.drift) {
    if (o.valid && o.sliding && b >= 8) S.drift = { t0: o.t, peak: b, side, calmSince: null };
    return;
  }
  const d = S.drift;
  if (b > d.peak) { d.peak = b; d.side = side; }
  const calm = !o.valid || !o.sliding || b < 4;
  if (!calm) { d.calmSince = null; return; }
  if (d.calmSince === null) d.calmSince = o.t;
  if (o.t - d.calmSince >= 0.5) {
    const dur = d.calmSince - d.t0;
    if (dur >= 0.5) {
      S.last = { peak: d.peak, side: d.side, dur };
      if (!S.best || d.peak > S.best.peak) S.best = { ...S.last };
    }
    S.drift = null;
  }
}

// ------------------------------------------------------------------ start / stop

function showIntroError(msg) {
  $('introError').textContent = msg;
}

$('startBtn').addEventListener('click', async () => {
  showIntroError('');
  if (!DEMO) {
    if (!window.isSecureContext) {
      showIntroError('This page must be opened over https:// for the browser to allow sensor access.');
      return;
    }
    try {
      await requestMotionPermission(); // first await: iOS requires it inside the tap
    } catch (e) {
      showIntroError(`${e.message} On iPhone: Settings › Safari › Motion & Orientation Access, then reload.`);
      return;
    }
  }
  $('startBtn').disabled = true;
  try {
    await begin();
  } finally {
    $('startBtn').disabled = false;
  }
});

async function begin() {
  S.log = [];
  S.queue = [];
  S.out = null;
  S.drift = null;
  S.warn = '';
  S.lastFixAt = null;
  S.logMeta = {
    app: 'drift-angle-meter',
    started: new Date().toISOString(),
    userAgent: navigator.userAgent,
    config: estimatorConfig(),
    demo: DEMO,
    format: 'events: [0, t_ms, gx, gy, gz, ax, ay, az] IMU (rad/s, m/s²) | [1, t_ms, lat, lon, speed, course] GNSS',
  };
  S.backend = await createBackend(onOut, estimatorConfig());

  if (DEMO) {
    await startDemo();
  } else {
    S.motion = startMotion((s) => record([0, s.t, s.gyro[0], s.gyro[1], s.gyro[2], s.accel[0], s.accel[1], s.accel[2]]));
    S.gnss = startGnss(
      (f) => {
        S.lastFixAt = performance.now();
        record([1, f.t, f.lat, f.lon, f.speed, f.course]);
      },
      (err) => { S.warn = err.message; },
    );
    S.wake.start();
    setTimeout(checkSensors, 3000);
  }
  S.flushTimer = setInterval(() => {
    if (S.queue.length) {
      S.backend.post({ type: 'events', events: S.queue });
      S.queue = [];
    }
  }, 40);
  S.running = true;
  $('intro').hidden = true;
  $('live').hidden = false;
  $('gpsChip').hidden = false;
  $('qualityChip').hidden = false;
  requestAnimationFrame(render);
}

function checkSensors() {
  if (!S.running || !S.motion) return;
  const st = S.motion.stats;
  if (st.samples > 0) return;
  if (st.noGyro > 0) {
    S.warn = 'This phone or browser does not provide gyroscope data, which the drift meter needs.';
  } else {
    S.warn = 'No motion sensor data is arriving. Try another browser (Chrome on Android, Safari on iPhone).';
  }
}

function stop() {
  S.running = false;
  clearInterval(S.flushTimer);
  clearInterval(S.demoTimer);
  if (S.motion) S.motion.stop();
  if (S.gnss) S.gnss.stop();
  S.motion = S.gnss = null;
  S.wake.stop();
  if (S.backend) S.backend.stop();
  S.backend = null;
  $('live').hidden = true;
  $('intro').hidden = false;
  $('gpsChip').hidden = true;
  $('qualityChip').hidden = true;
  $('startBtn').textContent = 'Start again';
}

$('stopBtn').addEventListener('click', stop);
$('recalBtn').addEventListener('click', () => {
  if (S.backend) S.backend.post({ type: 'recalibrate' });
});

$('logBtn').addEventListener('click', () => {
  const blob = new Blob([JSON.stringify({ meta: S.logMeta, events: S.log })], { type: 'application/json' });
  const a = document.createElement('a');
  const d = new Date();
  const p = (n) => String(n).padStart(2, '0');
  a.download = `drift-log-${d.getFullYear()}${p(d.getMonth() + 1)}${p(d.getDate())}-${p(d.getHours())}${p(d.getMinutes())}.json`;
  a.href = URL.createObjectURL(blob);
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
});

// ------------------------------------------------------------------ demo (simulated session)

async function startDemo() {
  const [{ Rng }, { randomPhone, simulate }, { SCENARIOS }] = await Promise.all([
    import('../sim/rng.js'), import('../sim/phone.js'), import('../sim/scenario.js'),
  ]);
  const seed = Number(params.get('seed')) || 7;
  const rng = new Rng(seed);
  const phone = randomPhone(rng);
  const { events } = simulate(SCENARIOS.driftSession(), phone, rng, estimatorConfig());
  const t0 = performance.now();
  let i = 0;
  S.demoTimer = setInterval(() => {
    const simT = ((performance.now() - t0) / 1000) * DEMO_SPEED;
    while (i < events.length && events[i].tDel <= simT) {
      const ev = events[i++];
      if (ev.type === 'imu') {
        const d = ev.data;
        record([0, d.t, d.gyro[0], d.gyro[1], d.gyro[2], d.accel[0], d.accel[1], d.accel[2]]);
      } else {
        const d = ev.data;
        S.lastFixAt = performance.now();
        record([1, d.t, d.lat, d.lon, d.speed, d.course]);
      }
    }
    if (i >= events.length) clearInterval(S.demoTimer);
  }, 20);
}

// ------------------------------------------------------------------ gauge

const G = { cx: 160, cy: 170, r: 126, range: 70 };
const polar = (deg, r) => {
  const a = (deg / G.range) * (Math.PI / 2);
  return [G.cx + r * Math.sin(a), G.cy - r * Math.cos(a)];
};
const f1 = (x) => x.toFixed(1);

function arcPath(fromDeg, toDeg, r) {
  const [x0, y0] = polar(fromDeg, r), [x1, y1] = polar(toDeg, r);
  const sweep = toDeg > fromDeg ? 1 : 0;
  return `M${f1(x0)} ${f1(y0)} A${r} ${r} 0 0 ${sweep} ${f1(x1)} ${f1(y1)}`;
}

function buildGauge() {
  $('gTrack').setAttribute('d', arcPath(-G.range, G.range, G.r));
  const ns = 'http://www.w3.org/2000/svg';
  const g = $('gTicks');
  for (let d = -G.range; d <= G.range; d += 10) {
    const major = d % 20 === 0;
    const [x0, y0] = polar(d, G.r - 12), [x1, y1] = polar(d, G.r - (major ? 24 : 18));
    const l = document.createElementNS(ns, 'line');
    l.setAttribute('x1', f1(x0)); l.setAttribute('y1', f1(y0));
    l.setAttribute('x2', f1(x1)); l.setAttribute('y2', f1(y1));
    l.setAttribute('class', major ? 'tick major' : 'tick');
    g.appendChild(l);
    if (major) {
      const [tx, ty] = polar(d, G.r - 36);
      const t = document.createElementNS(ns, 'text');
      t.setAttribute('x', f1(tx)); t.setAttribute('y', f1(ty));
      t.setAttribute('class', 'tlabel');
      t.textContent = String(Math.abs(d));
      g.appendChild(t);
    }
  }
}

function setGauge(beta, peak) {
  const v = Math.max(-G.range, Math.min(G.range, beta));
  if (Math.abs(v) < 0.5) {
    $('gValue').setAttribute('d', '');
  } else {
    $('gValue').setAttribute('d', arcPath(0, v, G.r));
  }
  const [mx, my] = polar(v, G.r);
  $('gMarker').setAttribute('cx', f1(mx));
  $('gMarker').setAttribute('cy', f1(my));
  const pk = $('gPeak');
  if (peak) {
    const p = Math.max(-G.range, Math.min(G.range, peak));
    const [a, b] = polar(p, G.r + 9), [c, d] = polar(p - 3, G.r + 19), [e, f] = polar(p + 3, G.r + 19);
    pk.setAttribute('d', `M${f1(a)} ${f1(b)} L${f1(c)} ${f1(d)} L${f1(e)} ${f1(f)} Z`);
    pk.setAttribute('visibility', 'visible');
  } else {
    pk.setAttribute('visibility', 'hidden');
  }
}

// ------------------------------------------------------------------ render loop

const HINTS = {
  collecting: 'Drive normally and take a few turns. The meter is learning how the phone is mounted.',
  'need some accelerating and braking': 'Now accelerate and brake a few times.',
  'mount direction not yet conclusive': 'Keep driving normally, with some turns and speed changes.',
  'gyro does not match GNSS turning': 'Keep driving normally. If this lasts, check that the phone is firmly mounted.',
};

function setChip(el, level, text) {
  el.className = `chip ${level}`;
  el.querySelector('span').textContent = text;
}

function fmtDrift(d) {
  return d ? `${Math.round(d.peak)}<small>°</small>` : '–';
}

function render() {
  if (!S.running) return;
  const o = S.out;
  const now = performance.now();

  // GPS chip
  const since = S.lastFixAt === null ? Infinity : (now - S.lastFixAt) / 1000;
  if (since > 3) setChip($('gpsChip'), 'critical', S.lastFixAt === null ? 'No GPS' : 'GPS lost');
  else setChip($('gpsChip'), 'good', o && o.gnssRate ? `GPS ${o.gnssRate.toFixed(o.gnssRate < 2 ? 1 : 0)} Hz` : 'GPS');

  let statusMsg = '', statusSub = '', progress = null;
  const live = o && o.phase === 'running' && o.valid;
  if (!o) {
    statusMsg = 'Starting…';
  } else if (o.phase === 'waiting-gnss') {
    statusMsg = 'Waiting for GPS…';
    statusSub = 'Make sure location is on and the sky is visible.';
  } else if (o.phase === 'standstill') {
    statusMsg = 'Keep the car still';
    statusSub = 'Measuring gravity direction and gyro bias.';
    progress = o.progress;
  } else if (o.phase === 'drive-calibration') {
    statusMsg = 'Calibrating while you drive';
    statusSub = HINTS[o.calibrationHint] || HINTS.collecting;
    progress = o.progress;
  } else if (!live) {
    statusMsg = o.message || 'Drive faster to measure';
    statusSub = o.message ? '' : 'The angle is shown above 11 km/h.';
  }
  $('statusPanel').hidden = live && !o.notice;
  $('statusNotice').textContent = o && o.notice ? o.notice : '';
  $('statusMsg').textContent = statusMsg;
  $('statusSub').textContent = statusSub;
  $('statusBar').hidden = progress === null;
  if (progress !== null) $('statusProgress').style.width = `${Math.round(100 * progress)}%`;

  // gauge
  const gauge = $('gauge');
  if (live) {
    const b = o.beta;
    setGauge(b, S.drift ? (S.drift.side === 'left' ? -S.drift.peak : S.drift.peak) : null);
    $('gAngle').textContent = `${Math.round(Math.abs(b))}°`;
    $('gDir').textContent = Math.abs(b) < 1.5 ? 'straight' : b < 0 ? 'nose left' : 'nose right';
    gauge.classList.toggle('dim', o.quality === 'poor');
    const pm = 2 * o.betaSigma;
    if (o.quality === 'good') setChip($('qualityChip'), 'good', `±${pm.toFixed(0)}°`);
    else if (o.quality === 'fair') setChip($('qualityChip'), 'warning', `±${pm.toFixed(0)}°`);
    else setChip($('qualityChip'), 'critical', `Low accuracy ±${pm.toFixed(0)}°`);
  } else {
    setGauge(0, null);
    $('gAngle').textContent = '–';
    $('gDir').textContent = '';
    gauge.classList.add('dim');
    setChip($('qualityChip'), '', o && o.phase === 'running' ? 'Ready' : 'Calibrating');
  }

  // stats
  $('sSpeed').innerHTML = o && o.speed !== null && Number.isFinite(o.speed)
    ? `${Math.round(o.speed * 3.6)}<small> km/h</small>` : '–';
  $('sNow').innerHTML = fmtDrift(S.drift);
  $('sBest').innerHTML = fmtDrift(S.best);
  $('sBestSub').textContent = S.best ? `${S.best.side}, ${S.best.dur.toFixed(1)} s` : '';
  $('sLast').innerHTML = fmtDrift(S.last);
  $('sLastSub').textContent = S.last ? `${S.last.side}, ${S.last.dur.toFixed(1)} s` : '';

  // warnings
  let warn = S.warn;
  if (!warn && o && o.imuRate && o.imuRate < 30) {
    warn = `Motion sensor rate is low (${Math.round(o.imuRate)} Hz), so accuracy will suffer.`;
  }
  $('liveWarn').textContent = warn;

  requestAnimationFrame(render);
}

// ------------------------------------------------------------------ settings dialog

$('settingsBtn').addEventListener('click', () => {
  $('setPhoneX').value = settings.phoneX;
  $('setWheelbase').value = settings.wheelbase;
  $('settings').showModal();
});

$('settings').addEventListener('close', () => {
  if ($('settings').returnValue !== 'save') return;
  const x = Number($('setPhoneX').value), wb = Number($('setWheelbase').value);
  const next = {
    phoneX: Number.isFinite(x) ? Math.min(5, Math.max(-1, x)) : DEFAULTS.phoneX,
    wheelbase: Number.isFinite(wb) ? Math.min(4.5, Math.max(1.5, wb)) : DEFAULTS.wheelbase,
  };
  const changed = next.phoneX !== settings.phoneX || next.wheelbase !== settings.wheelbase;
  settings = next;
  saveSettings(settings);
  if (changed && S.running && S.backend) {
    S.backend.post({ type: 'start', config: estimatorConfig() }); // restarts calibration with the new geometry
    S.logMeta.config = estimatorConfig();
  }
});

buildGauge();
if (DEMO) {
  $('startBtn').textContent = `Start demo${DEMO_SPEED > 1 ? ` (${DEMO_SPEED}× speed)` : ''}`;
  if (params.has('autostart')) $('startBtn').click();
}
