// Browser front-end for the simulation harness: runs one virtual phone and plots
// true vs estimated drift angle, or runs a Monte Carlo batch.

import { runOnce, describePhone } from './run.js';
import { DEG } from '../core/math.js';

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const fmt = (x, d = 1) => (Number.isFinite(x) ? x.toFixed(d) : '–');
const nextFrame = () => new Promise((r) => setTimeout(r, 0));

let data = null;      // { t, truth, est, sig, slide, end }
let view = null;      // [x0, x1]
let hoverT = null;
let drag = null;

// ------------------------------------------------------------------ run

async function runSeed(seed) {
  $('run').disabled = true;
  $('run').textContent = 'Running…';
  await nextFrame();
  try {
    const r = runOnce({ seed });
    show(r);
    history.replaceState(null, '', `#seed=${seed}`);
  } finally {
    $('run').disabled = false;
    $('run').textContent = 'Run';
  }
}

function show(r) {
  const { rec, truth, phone, metrics: m } = r;
  const t = [], tr = [], es = [], sg = [], sl = [];
  let last = -Infinity;
  for (const x of rec) {
    if (x.t - last < 0.02) continue;
    last = x.t;
    const g = truth[Math.min(truth.length - 1, Math.round(x.t / 0.01))];
    t.push(x.t);
    tr.push(g.speed > 3 ? g.beta / DEG : NaN);
    es.push(x.valid ? x.beta : NaN);
    sg.push(x.valid ? x.sigma : NaN);
    sl.push(x.valid && x.sliding);
  }
  data = { t, truth: tr, est: es, sig: sg, slide: sl, end: t[t.length - 1] };
  view = [0, data.end];

  $('phone').textContent = `Seed ${r.seed}: ${describePhone(phone)}`;
  const faults = [];
  if (phone.accelSign < 0) faults.push('accel sign');
  if (phone.gyroSign < 0) faults.push('gyro sign');
  if (phone.gyroUnit !== 1) faults.push('gyro units');
  const faultOk = m.accelSignOk && m.gyroGainOk;
  const tiles = [
    ['Calibrated at', m.calibrated ? `${fmt(m.tRunning, 0)}<small> s</small>` : '<span class="bad">never</span>'],
    ['Drift RMSE', `${fmt(m.drift.rmse)}<small>°</small>`],
    ['Drift 95th pct error', `${fmt(m.drift.p95)}<small>°</small>`],
    ['Grip RMSE', `${fmt(m.grip.rmse)}<small>°</small>`],
    ['Overconfident', `${fmt(100 * m.overconfidence)}<small>% of time</small>`],
    ['GNSS latency', `${m.calibrated ? fmt(1000 * m.latencyEst, 0) : '–'}<small> / ${fmt(1000 * m.latencyTrue, 0)} ms true</small>`],
    ['GNSS smoothing', `${m.calibrated ? fmt(m.smoothingEst, 2) : '–'}<small> / ${fmt(phone.gnssSmoothing, 2)} s true</small>`],
    ['Sensor faults', faults.length
      ? `${faults.join(', ')}<small> ${m.calibrated ? (faultOk ? '✓ detected' : '✗ missed') : ''}</small>`
      : 'none'],
  ];
  $('tiles').innerHTML = tiles.map(([k, v]) => `<div class="tile"><div class="k">${k}</div><div class="v">${v}</div></div>`).join('');
  draw();
}

// ------------------------------------------------------------------ charts

function alpha(hex, a) {
  const h = hex.replace('#', '');
  const n = parseInt(h.length === 3 ? h.split('').map((c) => c + c).join('') : h, 16);
  return `rgba(${(n >> 16) & 255}, ${(n >> 8) & 255}, ${n & 255}, ${a})`;
}

function niceTicks(lo, hi, count) {
  const span = hi - lo;
  const raw = span / count;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 5, 10].map((m) => m * mag).find((s) => span / s <= count) || 10 * mag;
  const out = [];
  for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(10));
  return out;
}

const PAD = { l: 44, r: 12, t: 10, b: 24 };

function frame(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = Number(canvas.getAttribute('height'));
  canvas.style.height = `${h}px`;
  if (canvas.width !== Math.round(w * dpr)) canvas.width = Math.round(w * dpr);
  canvas.height = Math.round(h * dpr);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w, h };
}

function indexRange(t, x0, x1) {
  let i0 = 0, i1 = t.length - 1;
  while (i0 < t.length && t[i0] < x0) i0++;
  while (i1 > 0 && t[i1] > x1) i1--;
  return [Math.max(0, i0 - 1), Math.min(t.length - 1, i1 + 1)];
}

function plot(canvas, { lines, band, shade, yRange, zeroLine }) {
  const { ctx, w, h } = frame(canvas);
  const [x0, x1] = view;
  const { t } = data;
  const [i0, i1] = indexRange(t, x0, x1);
  const [y0, y1] = yRange(i0, i1);
  const X = (v) => PAD.l + ((v - x0) / (x1 - x0)) * (w - PAD.l - PAD.r);
  const Y = (v) => PAD.t + (1 - (v - y0) / (y1 - y0)) * (h - PAD.t - PAD.b);
  ctx.clearRect(0, 0, w, h);

  // slide-mode shading
  if (shade) {
    ctx.fillStyle = css('--shade');
    let s = null;
    for (let i = i0; i <= i1 + 1; i++) {
      const on = i <= i1 && shade[i];
      if (on && s === null) s = t[i];
      if (!on && s !== null) {
        ctx.fillRect(X(s), PAD.t, X(t[i - 1]) - X(s), h - PAD.t - PAD.b);
        s = null;
      }
    }
  }

  // grid + ticks
  ctx.font = '11px system-ui, -apple-system, "Segoe UI", sans-serif';
  ctx.lineWidth = 1;
  ctx.fillStyle = css('--text-muted');
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (const v of niceTicks(y0, y1, 5)) {
    ctx.strokeStyle = v === 0 && zeroLine ? css('--axis') : css('--grid');
    ctx.beginPath(); ctx.moveTo(PAD.l, Math.round(Y(v)) + 0.5); ctx.lineTo(w - PAD.r, Math.round(Y(v)) + 0.5); ctx.stroke();
    ctx.fillText(String(v), PAD.l - 6, Y(v));
  }
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  for (const v of niceTicks(x0, x1, Math.max(3, Math.floor((w - PAD.l) / 90)))) {
    ctx.fillText(`${v}s`, X(v), h - PAD.b + 6);
  }
  ctx.strokeStyle = css('--axis');
  ctx.beginPath(); ctx.moveTo(PAD.l, h - PAD.b + 0.5); ctx.lineTo(w - PAD.r, h - PAD.b + 0.5); ctx.stroke();

  ctx.save();
  ctx.beginPath();
  ctx.rect(PAD.l, PAD.t, w - PAD.l - PAD.r, h - PAD.t - PAD.b);
  ctx.clip();

  // confidence band
  if (band) {
    ctx.fillStyle = alpha(band.color, 0.22);
    let open = false;
    const flush = (from, to) => {
      ctx.beginPath();
      for (let i = from; i <= to; i++) ctx.lineTo(X(t[i]), Y(band.hi[i]));
      for (let i = to; i >= from; i--) ctx.lineTo(X(t[i]), Y(band.lo[i]));
      ctx.closePath();
      ctx.fill();
    };
    let s = 0;
    for (let i = i0; i <= i1 + 1; i++) {
      const ok = i <= i1 && Number.isFinite(band.lo[i]);
      if (ok && !open) { s = i; open = true; }
      if (!ok && open) { flush(s, i - 1); open = false; }
    }
  }

  // lines (2px, gaps where undefined)
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  for (const ln of lines) {
    ctx.strokeStyle = ln.color;
    ctx.beginPath();
    let pen = false;
    for (let i = i0; i <= i1; i++) {
      const v = ln.y[i];
      if (!Number.isFinite(v)) { pen = false; continue; }
      if (pen) ctx.lineTo(X(t[i]), Y(v)); else { ctx.moveTo(X(t[i]), Y(v)); pen = true; }
    }
    ctx.stroke();
  }

  // drag selection
  if (drag && drag.canvas === canvas && drag.x !== undefined) {
    ctx.fillStyle = alpha(css('--series-1'), 0.12);
    ctx.fillRect(Math.min(drag.x0, drag.x), PAD.t, Math.abs(drag.x - drag.x0), h - PAD.t - PAD.b);
  }

  // crosshair
  if (hoverT !== null && hoverT >= x0 && hoverT <= x1) {
    ctx.strokeStyle = css('--text-muted');
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(Math.round(X(hoverT)) + 0.5, PAD.t); ctx.lineTo(Math.round(X(hoverT)) + 0.5, h - PAD.b); ctx.stroke();
    const i = nearest(hoverT);
    for (const ln of lines) {
      const v = ln.y[i];
      if (!Number.isFinite(v)) continue;
      ctx.fillStyle = ln.color;
      ctx.strokeStyle = css('--surface-1');
      ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(X(t[i]), Y(v), 4.5, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    }
  }
  ctx.restore();
  return { X, Y, w, h };
}

function nearest(tq) {
  const t = data.t;
  let lo = 0, hi = t.length - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (t[mid] <= tq) lo = mid; else hi = mid;
  }
  return tq - t[lo] < t[hi] - tq ? lo : hi;
}

function draw() {
  if (!data) return;
  const s1 = css('--series-1'), s2 = css('--series-2');
  const err = data.est.map((e, i) => e - data.truth[i]);
  const lo = data.est.map((e, i) => e - 2 * data.sig[i]);
  const hi = data.est.map((e, i) => e + 2 * data.sig[i]);
  plot($('cBeta'), {
    lines: [{ y: data.truth, color: s1 }, { y: data.est, color: s2 }],
    band: { lo, hi, color: s2 },
    shade: data.slide,
    zeroLine: true,
    yRange: (i0, i1) => {
      let a = Infinity, b = -Infinity;
      for (let i = i0; i <= i1; i++) {
        for (const v of [data.truth[i], lo[i], hi[i]]) if (Number.isFinite(v)) { a = Math.min(a, v); b = Math.max(b, v); }
      }
      if (!Number.isFinite(a)) return [-10, 10];
      const pad = Math.max(2, 0.08 * (b - a));
      return [Math.min(a - pad, -5), Math.max(b + pad, 5)];
    },
  });
  plot($('cErr'), {
    lines: [{ y: err, color: s2 }],
    band: { lo: data.sig.map((s) => -2 * s), hi: data.sig.map((s) => 2 * s), color: s2 },
    shade: data.slide,
    zeroLine: true,
    yRange: (i0, i1) => {
      let m = 3;
      for (let i = i0; i <= i1; i++) {
        if (Number.isFinite(err[i])) m = Math.max(m, Math.abs(err[i]));
        if (Number.isFinite(data.sig[i])) m = Math.max(m, 2 * data.sig[i]);
      }
      m = Math.min(m * 1.1, 30);
      return [-m, m];
    },
  });
  tooltips();
}

function tooltips() {
  for (const id of ['Beta', 'Err']) {
    const tip = $(`tip${id}`), canvas = $(`c${id}`);
    if (hoverT === null || drag) { tip.style.display = 'none'; continue; }
    const i = nearest(hoverT);
    const tr = data.truth[i], es = data.est[i], sg = data.sig[i];
    tip.innerHTML =
      `<div class="muted">t = ${fmt(data.t[i], 2)} s${data.slide[i] ? ' · slide mode' : ''}</div>` +
      `<div>True <b>${fmt(tr)}°</b></div>` +
      `<div>Estimated <b>${fmt(es)}°</b> <span class="muted">±${fmt(2 * sg)}° (2σ)</span></div>` +
      `<div>Error <b>${fmt(es - tr)}°</b></div>`;
    tip.style.display = 'block';
    const w = canvas.clientWidth;
    const x = PAD.l + ((data.t[i] - view[0]) / (view[1] - view[0])) * (w - PAD.l - PAD.r);
    const tw = tip.offsetWidth;
    tip.style.left = `${x + 14 + tw > w ? x - 14 - tw : x + 14}px`;
    tip.style.top = '12px';
  }
}

function tAt(canvas, clientX) {
  const r = canvas.getBoundingClientRect();
  const w = r.width;
  const f = (clientX - r.left - PAD.l) / (w - PAD.l - PAD.r);
  return view[0] + Math.min(1, Math.max(0, f)) * (view[1] - view[0]);
}

for (const canvas of [$('cBeta'), $('cErr')]) {
  canvas.addEventListener('pointermove', (e) => {
    if (!data) return;
    hoverT = tAt(canvas, e.clientX);
    if (drag) drag.x = e.clientX - canvas.getBoundingClientRect().left;
    draw();
  });
  canvas.addEventListener('pointerleave', () => { hoverT = null; if (!drag) draw(); });
  canvas.addEventListener('pointerdown', (e) => {
    if (!data) return;
    canvas.setPointerCapture(e.pointerId);
    drag = { canvas, x0: e.clientX - canvas.getBoundingClientRect().left, t0: tAt(canvas, e.clientX) };
  });
  canvas.addEventListener('pointerup', (e) => {
    if (!drag) return;
    const t1 = tAt(canvas, e.clientX);
    const moved = Math.abs(e.clientX - canvas.getBoundingClientRect().left - drag.x0) > 6;
    if (moved) view = [Math.min(drag.t0, t1), Math.max(drag.t0, t1)];
    drag = null;
    draw();
  });
  canvas.addEventListener('dblclick', () => { if (data) { view = [0, data.end]; draw(); } });
}
window.addEventListener('resize', draw);
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', draw);

// ------------------------------------------------------------------ Monte Carlo

async function monteCarlo() {
  const n = Math.max(1, Math.min(500, Number($('mcRuns').value) || 20));
  const s0 = Number($('seed').value) || 1;
  const body = $('mcTable').querySelector('tbody');
  body.innerHTML = '';
  $('mcCard').hidden = false;
  $('mc').disabled = true;
  const res = [];
  for (let i = 0; i < n; i++) {
    const seed = s0 + i;
    $('mcSummary').textContent = `Running ${i + 1} / ${n}…`;
    await nextFrame();
    const { phone, metrics: m } = runOnce({ seed });
    res.push(m);
    const faults = [phone.accelSign < 0 && 'accel', phone.gyroSign < 0 && 'gyro sign', phone.gyroUnit !== 1 && 'gyro units']
      .filter(Boolean);
    const tr = document.createElement('tr');
    tr.innerHTML =
      `<td>${seed}</td><td>${phone.mount}, ${phone.imuRate} Hz IMU, ${phone.gnssRate} Hz GNSS</td>` +
      `<td>${fmt(m.drift.rmse)}°</td><td>${fmt(m.drift.p95)}°</td><td>${fmt(m.grip.rmse)}°</td>` +
      `<td>${fmt(100 * m.driftAvailability, 0)}%</td><td>${fmt(100 * m.overconfidence)}%</td>` +
      `<td>${m.calibrated ? fmt(m.tRunning, 0) + ' s' : '<span class="bad">never</span>'}</td>` +
      `<td>${faults.length ? faults.join(', ') + (m.accelSignOk && m.gyroGainOk ? ' ✓' : ' <span class="bad">✗</span>') : '–'}</td>`;
    tr.addEventListener('click', () => { $('seed').value = seed; runSeed(seed); window.scrollTo({ top: 0, behavior: 'smooth' }); });
    body.appendChild(tr);
  }
  const cal = res.filter((m) => m.calibrated);
  const q = (xs, p) => { const s = xs.filter(Number.isFinite).sort((a, b) => a - b); return s[Math.min(s.length - 1, Math.floor(p * s.length))]; };
  $('mcSummary').innerHTML =
    `${cal.length}/${res.length} calibrated · drift RMSE median <b>${fmt(q(cal.map((m) => m.drift.rmse), 0.5))}°</b>, ` +
    `90th percentile ${fmt(q(cal.map((m) => m.drift.rmse), 0.9))}° · grip RMSE median ${fmt(q(cal.map((m) => m.grip.rmse), 0.5))}°`;
  $('mc').disabled = false;
}

// ------------------------------------------------------------------ wiring

$('run').addEventListener('click', () => runSeed(Number($('seed').value) || 1));
$('random').addEventListener('click', () => {
  const s = 1 + Math.floor(Math.random() * 100000);
  $('seed').value = s;
  runSeed(s);
});
$('mc').addEventListener('click', monteCarlo);

const m = location.hash.match(/seed=(\d+)/);
if (m) $('seed').value = m[1];
runSeed(Number($('seed').value) || 1);
