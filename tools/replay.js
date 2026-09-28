// Replays a log saved from the app ("Save log") through the estimator and summarizes it.
//   node tools/replay.js drift-log-XXXX.json [--trace]

import { readFileSync, writeFileSync } from 'node:fs';
import { DriftEstimator } from '../js/core/estimator.js';

const file = process.argv[2];
if (!file) {
  console.error('usage: node tools/replay.js <log.json> [--trace]');
  process.exit(1);
}
const trace = process.argv.includes('--trace');
const { meta, events } = JSON.parse(readFileSync(file, 'utf8'));
// Logs from app versions before 0.2 stored the gyro as [beta, gamma, alpha]; the
// estimator now takes rotationRate in its native [alpha, beta, gamma] order.
if (meta.gyroOrder !== 'alpha-beta-gamma') {
  for (const e of events) if (e[0] === 0) [e[2], e[3], e[4]] = [e[4], e[2], e[3]];
}
const profileArg = process.argv.indexOf('--profile');
const profile = profileArg > 0 ? JSON.parse(readFileSync(process.argv[profileArg + 1], 'utf8')) : null;
// only the phone position is taken from the log; everything else is today's default
const est = new DriftEstimator({
  ...(meta.config && meta.config.phonePos ? { phonePos: meta.config.phonePos } : {}),
  ...(profile ? { device: profile.device, mount: profile.mount } : {}),
});
const imu = events.filter((e) => e[0] === 0), gnss = events.filter((e) => e[0] === 1);
const span = (events[events.length - 1][1] - events[0][1]) / 1000;
console.log(`${meta.userAgent}`);
console.log(`${span.toFixed(0)} s, IMU ${imu.length} samples (${(imu.length / span).toFixed(0)} Hz), GNSS ${gnss.length} fixes (${(gnss.length / span).toFixed(1)} Hz), no speed/course in ${gnss.filter((g) => g[4] === null).length} fixes`);

let phase = null, lastTrace = -Infinity, out = null;
let firstLive = false, sensorCheck = null, latencyShown = false;
const t0 = events[0][1];
const drifts = [];
let cur = null;
for (const e of events) {
  out = e[0] === 0
    ? est.addImu({ t: e[1], gyro: [e[2], e[3], e[4]], accel: [e[5], e[6], e[7]] })
    : est.addGnss({ t: e[1], lat: e[2], lon: e[3], speed: e[4], course: e[5] });
  const ts = (e[1] - t0) / 1000;
  if (out.phase !== phase) {
    phase = out.phase;
    console.log(`${ts.toFixed(1).padStart(7)} s  phase: ${phase}${out.notice ? ` (${out.notice})` : ''}`);
  }
  if (out.valid && !firstLive) {
    firstLive = true;
    console.log(`${ts.toFixed(1).padStart(7)} s  angle live (mount from ${est.mountSource})`);
  }
  if (out.sensorCheck !== sensorCheck) {
    sensorCheck = out.sensorCheck;
    if (sensorCheck === 'ok') console.log(`${ts.toFixed(1).padStart(7)} s  sensor check ok: ${JSON.stringify(out.profile.device)}`);
  }
  if (out.latencyConverged && !latencyShown) {
    latencyShown = true;
    console.log(`${ts.toFixed(1).padStart(7)} s  GNSS delay learned: ${(out.latency * 1000).toFixed(0)} ms`);
  }
  if (out.valid && out.sliding && out.quality !== 'poor' && Math.abs(out.beta) >= 8) {
    if (!cur) cur = { t0: ts, peak: 0 };
    cur.peak = Math.max(cur.peak, Math.abs(out.beta));
    cur.t1 = ts;
  } else if (cur && ts - cur.t1 > 0.5) {
    drifts.push(cur);
    cur = null;
  }
  if (trace && out.valid && ts - lastTrace >= 0.5) {
    lastTrace = ts;
    console.log(`${ts.toFixed(1).padStart(7)} s  beta ${out.beta.toFixed(1).padStart(6)}° ±${(2 * out.betaSigma).toFixed(1)}  ${(out.speed * 3.6).toFixed(0)} km/h ${out.sliding ? 'SLIDE' : ''}`);
  }
}
if (cur) drifts.push(cur);
console.log('\ncalibration:', JSON.stringify(out.calibration), `latency ${out.latency ? (out.latency * 1000).toFixed(0) + ' ms' : '-'}`);
const save = process.argv.indexOf('--save-profile');
if (save > 0) {
  writeFileSync(process.argv[save + 1], JSON.stringify(out.profile));
  console.log('profile saved to', process.argv[save + 1]);
}
console.log('diagnostics:', JSON.stringify(est.debugState()));
console.log(`\n${drifts.length} drifts:`);
for (const d of drifts) console.log(`  at ${d.t0.toFixed(1)} s, ${(d.t1 - d.t0).toFixed(1)} s long, peak ${d.peak.toFixed(0)}°`);
