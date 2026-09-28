// Runs the estimator against one simulated phone + scenario and scores it.

import { DriftEstimator, DEFAULT_CONFIG } from '../core/estimator.js';
import { DEG } from '../core/math.js';
import { Rng } from './rng.js';
import { randomPhone, simulate } from './phone.js';
import { SCENARIOS } from './scenario.js';

// session > 1 simulates another drive with the same phone and mount (different noise),
// starting from the profile the app stored after the previous session.
export function runOnce({
  seed = 1, scenario = 'driftSession', phone: overrides = {}, config = {}, session = 1, profile = null,
} = {}) {
  const cfg = { ...DEFAULT_CONFIG, ...config };
  if (profile) Object.assign(cfg, { device: profile.device, mount: profile.mount });
  const rng = new Rng(seed);
  const phone = randomPhone(rng, overrides);
  const sc = SCENARIOS[scenario]();
  const { events, truth } = simulate(sc, phone, session > 1 ? new Rng(seed + 1000003 * session) : rng, cfg);
  const est = new DriftEstimator(cfg);
  const rec = [];
  const t0 = performance.now();
  for (const ev of events) {
    if (ev.type === 'imu') {
      const o = est.addImu(ev.data);
      rec.push({
        t: ev.tSample, phase: o.phase, valid: o.valid, beta: o.beta, sigma: o.betaSigma,
        quality: o.quality, sliding: o.sliding,
      });
    } else {
      est.addGnss(ev.data);
    }
  }
  const cpuMs = performance.now() - t0;
  const metrics = computeMetrics(rec, truth, est, phone);
  metrics.cpuPerSimSecondMs = cpuMs / sc.end;
  return { seed, phone, metrics, rec, truth, est, profile: est.out.profile };
}

function pct(sorted, p) {
  if (!sorted.length) return NaN;
  return sorted[Math.min(sorted.length - 1, Math.floor(p * sorted.length))];
}

function errStats(errs) {
  if (!errs.length) return { n: 0, rmse: NaN, p95: NaN, max: NaN, mean: NaN };
  const abs = errs.map(Math.abs).sort((a, b) => a - b);
  return {
    n: errs.length,
    rmse: Math.sqrt(errs.reduce((s, e) => s + e * e, 0) / errs.length),
    p95: pct(abs, 0.95),
    max: abs[abs.length - 1],
    mean: errs.reduce((s, e) => s + e, 0) / errs.length,
  };
}

export function computeMetrics(rec, truth, est, phone) {
  const truthAt = (t) => truth[Math.max(0, Math.min(truth.length - 1, Math.round(t / 0.01)))];
  const firstValid = rec.find((r) => r.valid);
  const tRunning = firstValid ? firstValid.t : null;
  const moving = truth.find((x) => x.speed > 0.5);
  const tLive = tRunning !== null && moving ? tRunning - moving.t : null;
  const drift = [], grip = [], shown = [];
  let driftN = 0, driftAvail = 0, overconf = 0, consistN = 0;
  for (const r of rec) {
    if (tRunning === null || r.t < tRunning) continue;
    const tr = truthAt(r.t);
    if (tr.speed < 5) continue;
    const bt = tr.beta / DEG;
    const isDrift = Math.abs(bt) >= 10, isGrip = Math.abs(bt) < 4;
    if (isDrift) driftN++;
    if (!r.valid) continue;
    const e = r.beta - bt;
    if (isDrift) {
      if (r.quality !== 'poor') { driftAvail++; shown.push(e); }
      drift.push(e);
    } else if (isGrip) {
      grip.push(e);
    }
    consistN++;
    if (Math.abs(e) > 3 * r.sigma) overconf++;
  }
  const cal = est.cal, dev = est.device;
  return {
    calibrated: !!cal,
    tRunning,
    tLive,
    sensorsVerified: dev.sensorsVerified,
    accelVerified: dev.accelVerified,
    accelSignOk: dev.accelSign === phone.accelSign,
    gyroGainOk: Math.abs(dev.gyroGain * phone.gyroUnit * phone.gyroSign - 1) < 0.01,
    gyroMapOk: dev.gyroMap === phone.gyroMap,
    mountSource: est.mountSource,
    latencyEst: est.latency ? est.latency.L : null,
    smoothingEst: cal ? cal.smoothing : null,
    latencyTrue: phone.gnssLatency + phone.gnssJitter - phone.imuDelay - phone.imuJitter,
    drift: errStats(drift),
    driftShown: errStats(shown), // drifting readings not flagged "poor", i.e. what the gauge shows
    grip: errStats(grip),
    quirky: phone.gyroMap !== 0 || phone.accelSign < 0 || phone.gyroSign < 0 || phone.gyroUnit !== 1,
    driftAvailability: driftN ? driftAvail / driftN : NaN,
    overconfidence: consistN ? overconf / consistN : NaN,
    debug: est.debugState(),
  };
}

export function describePhone(p) {
  const faults = [];
  if (p.accelSign < 0) faults.push('accel-sign');
  if (p.gyroSign < 0) faults.push('gyro-sign');
  if (p.gyroUnit !== 1) faults.push(p.gyroUnit > 1 ? 'gyro-deg' : 'gyro-rad');
  if (p.gnssNoDoppler) faults.push('no-doppler');
  if (p.gyroMap === 1) faults.push('spec-axes');
  return `${p.mount.padEnd(14)} imu ${String(p.imuRate).padStart(3)}Hz gnss ${String(p.gnssRate).padStart(2)}Hz ` +
    `lat ${(p.gnssLatency * 1000).toFixed(0).padStart(3)}ms smooth ${p.gnssSmoothing.toFixed(1)}s ` +
    `vnoise ${p.gnssVelNoise.toFixed(2)}${faults.length ? ' [' + faults.join(',') + ']' : ''}`;
}
