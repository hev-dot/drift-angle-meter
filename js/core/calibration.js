// Self-calibration: everything that differs between phones, browsers and mounts is
// found at runtime instead of being configured.
//
// 1. Standstill (a couple of seconds): gravity direction in phone axes -> "up",
//    gyro bias, sensor noise levels.
// 2. Mount yaw (which way is forward) from the first straight acceleration, e.g. the
//    launch off the start line (MountAligner): a few seconds.
// 3. In the background while driving (DriveCalibrator), compared against GNSS:
//    - gyro axis order, units and sign, GNSS latency and smoothing, from course vs gyro yaw
//      (after ~150° of turning),
//    - accelerometer handedness (sign fault), from horizontal accel vs GNSS kinematics.
//    These are properties of the phone + browser, so they are remembered between sessions.

import {
  normalize, dot, cross, scale, fromRows, wrapPi, DEG,
} from './math.js';
import { latencyCandidates, argminRefined } from './latency.js';

// Browsers disagree on how DeviceMotionEvent.rotationRate's alpha/beta/gamma map to the
// device axes. The adapter delivers [alpha, beta, gamma] (rad/s); these are the known
// interpretations, as functions returning [x, y, z].
export const GYRO_MAPS = [
  (g) => [g[0], g[1], g[2]], // alpha = x, beta = y, gamma = z (Chrome on Android, measured)
  (g) => [g[1], g[2], g[0]], // alpha = z, beta = x, gamma = y (W3C specification text)
];

// Horizontal basis in raw phone axes: e1 = the phone axis closest to horizontal,
// projected onto the horizontal plane; e2 = up x e1.
function levelBasis(up) {
  const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  let best = axes[0];
  for (const a of axes) if (Math.abs(dot(a, up)) < Math.abs(dot(best, up))) best = a;
  const e1 = normalize(cross(cross(up, best), up));
  return { e1, e2: cross(up, e1) };
}

// Vehicle <- phone rotation from "up" and "forward" in raw accelerometer axes.
// A sign-inverted accelerometer inverts both, so both are multiplied by accelSign.
export function mountRotation(upRaw, fwdRaw, accelSign = 1) {
  const up = scale(normalize(upRaw), accelSign);
  let fwd = scale(fwdRaw, accelSign);
  fwd = normalize([fwd[0] - dot(fwd, up) * up[0], fwd[1] - dot(fwd, up) * up[1], fwd[2] - dot(fwd, up) * up[2]]);
  return fromRows(fwd, cross(up, fwd), up);
}

// Finds which way is forward from straight-line speed changes (the launch off the
// start line, or any firm straight acceleration or braking). Integrating the leveled
// accelerometer over the window gives the velocity change in phone axes; GNSS says how
// much the speed changed and that the course stayed constant. Gravity and accelerometer
// bias cancel because the standstill reading is subtracted.
// Also uses steady turns: there the (centripetal) acceleration points along the car's
// left-right axis, with the side given by the gyro's yaw direction.
export class MountAligner {
  constructor(standstill, {
    latency = 0.2, minDeltaSpeed = 2.5, maxCourseChange = 10 * DEG, gyroMap = 0, gyroGain = 1,
  } = {}) {
    this.up = standstill.up;
    this.gRef = scale(standstill.up, standstill.gravity);
    this.bias = standstill.gyroBias;
    this.gyroMap = gyroMap;
    this.gyroGain = gyroGain;
    ({ e1: this.e1, e2: this.e2 } = levelBasis(this.up));
    this.L = latency;
    this.minDs = minDeltaSpeed;
    this.maxCourse = maxCourseChange;
    this.T = []; this.c1 = []; this.c2 = []; this.cY = [];
    this._last = null;
    this.fixes = [];
    this.sum = [0, 0];
    this.n = 0;
    this.dirs = [];
    this.usedUntil = -Infinity;
    this.progress = 0;
  }

  addImu(t, accelRaw, gyroRaw) {
    const d = [accelRaw[0] - this.gRef[0], accelRaw[1] - this.gRef[1], accelRaw[2] - this.gRef[2]];
    const a1 = dot(d, this.e1), a2 = dot(d, this.e2);
    const g = GYRO_MAPS[this.gyroMap]([gyroRaw[0] - this.bias[0], gyroRaw[1] - this.bias[1], gyroRaw[2] - this.bias[2]]);
    const r = this.gyroGain * dot(g, this.up);
    const n = this.T.length;
    if (n === 0) {
      this.T.push(t); this.c1.push(0); this.c2.push(0); this.cY.push(0);
    } else {
      const dt = t - this.T[n - 1];
      if (dt <= 0) return;
      this.T.push(t);
      this.c1.push(this.c1[n - 1] + 0.5 * (a1 + this._last[0]) * dt);
      this.c2.push(this.c2[n - 1] + 0.5 * (a2 + this._last[1]) * dt);
      this.cY.push(this.cY[n - 1] + 0.5 * (r + this._last[2]) * dt);
    }
    this._last = [a1, a2, r];
    if (this.T[0] < t - 30) {
      let k = 0;
      while (this.T[k] < t - 20) k++;
      this.T.splice(0, k); this.c1.splice(0, k); this.c2.splice(0, k); this.cY.splice(0, k);
    }
  }

  // Steady turn between fixes a and b: the leveled velocity change is v * dPsi along
  // the car's left axis (towards the inside of the turn).
  _turnWindow(a, b) {
    const dt = b.t - a.t;
    if (dt < 0.9 || dt > 3 || a.speed < 2.5 || b.speed < 2.5 || Math.abs(b.speed - a.speed) > 1) return;
    // IMU-only window (no GNSS timing involved), ending at the fix minus the delay
    const t0 = a.t - this.L, t1 = b.t - this.L;
    const y0 = interp(this.T, this.cY, t0), y1 = interp(this.T, this.cY, t1);
    const p0 = interp(this.T, this.c1, t0), p1 = interp(this.T, this.c1, t1);
    const q0 = interp(this.T, this.c2, t0), q1 = interp(this.T, this.c2, t1);
    if (y0 === null || y1 === null || p0 === null || q0 === null) return;
    const dPsi = y1 - y0;
    if (Math.abs(dPsi) < 15 * DEG || Math.abs(dPsi) > 60 * DEG) return;
    // course must turn by about the same amount (grip, and a plausible gyro)
    if (a.chi === null || b.chi === null || Math.abs(wrapPi(b.chi - a.chi) - dPsi) > 0.3 * Math.abs(dPsi) + 5 * DEG) return;
    const dv = [p1 - p0, q1 - q0];
    // expected velocity change in vehicle axes: [speed change, v * dPsi to the left]
    const u = [b.speed - a.speed, 0.5 * (a.speed + b.speed) * dPsi];
    // the sideways part must dominate sensor bias and tilt errors (~0.1-0.2 m/s² x dt)
    if (Math.abs(u[1]) < 1.5 || Math.abs(u[0]) > 0.5 * Math.abs(u[1])) return;
    const expect = Math.hypot(u[0], u[1]);
    const mag = Math.hypot(dv[0], dv[1]);
    if (mag < 0.7 * expect || mag > 1.4 * expect) return;
    // forward is the leveled-phone direction that rotates u onto dv
    const ang = Math.atan2(dv[1], dv[0]) - Math.atan2(u[1], u[0]);
    const fwd = [Math.cos(ang), Math.sin(ang)];
    this.sum[0] += fwd[0] * expect;
    this.sum[1] += fwd[1] * expect;
    this.dirs.push(ang);
    this.n++;
    this.usedUntil = b.t;
  }

  // fix: { t, speed, chi } (chi: course, rad CCW from east, or null)
  addGnss(fix) {
    if (!Number.isFinite(fix.speed)) return;
    this.fixes.push(fix);
    while (this.fixes.length && this.fixes[0].t < fix.t - 12) this.fixes.shift();
    const b = fix;
    if (b.speed < 3 || b.chi === null) return;
    // turn windows of 1-3 s, longest first (gentle turns need a longer window)
    for (let i = 0; i < this.fixes.length - 1; i++) {
      const a = this.fixes[i];
      if (b.t - a.t > 3 || a.t < this.usedUntil) continue;
      if (b.t - a.t < 0.9) break;
      const n0 = this.n;
      this._turnWindow(a, b);
      if (this.n > n0) return;
    }
    let start = -1;
    for (let i = this.fixes.length - 2; i >= 0; i--) {
      const a = this.fixes[i];
      if (b.t - a.t > 8 || a.t < this.usedUntil) break;
      if (a.speed >= 3 && (a.chi === null || Math.abs(wrapPi(a.chi - b.chi)) > this.maxCourse)) break;
      const ds = Math.abs(b.speed - a.speed);
      this.progress = Math.max(this.progress, Math.min(1, ds / this.minDs));
      if (ds >= this.minDs) { start = i; break; }
    }
    if (start < 0) return;
    const a = this.fixes[start];
    const ds = b.speed - a.speed;
    const p = [interp(this.T, this.c1, a.t - this.L), interp(this.T, this.c1, b.t - this.L)];
    const q = [interp(this.T, this.c2, a.t - this.L), interp(this.T, this.c2, b.t - this.L)];
    const y = [interp(this.T, this.cY, a.t - this.L - 0.5), interp(this.T, this.cY, b.t - this.L)];
    if (p.includes(null) || q.includes(null)) return;
    // straight according to the gyro too (GNSS course says nothing at walking pace)
    if (y[0] !== null && y[1] !== null && Math.abs(y[1] - y[0]) > 5 * DEG) return;
    const dv = [p[1] - p[0], q[1] - q[0]];
    const mag = Math.hypot(dv[0], dv[1]);
    if (mag < 0.5 * Math.abs(ds) || mag > 2 * Math.abs(ds)) return; // not a clean straight-line speed change
    const s = Math.sign(ds);
    this.sum[0] += s * dv[0];
    this.sum[1] += s * dv[1];
    this.dirs.push(Math.atan2(s * dv[1], s * dv[0]));
    this.n++;
    this.usedUntil = b.t;
  }

  // { fwdRaw, sigma } once at least one clean straight speed change has been seen.
  get result() {
    if (this.n === 0) return null;
    const a = Math.atan2(this.sum[1], this.sum[0]);
    let spread = 0;
    for (const d of this.dirs) spread += wrapPi(d - a) ** 2;
    spread = this.n > 1 ? Math.sqrt(spread / (this.n - 1)) : 0;
    const c = Math.cos(a), s = Math.sin(a);
    return {
      fwdRaw: [0, 1, 2].map((k) => c * this.e1[k] + s * this.e2[k]),
      sigma: Math.max(3 * DEG / Math.sqrt(this.n), spread / Math.sqrt(this.n), 1 * DEG),
      n: this.n,
    };
  }
}

// Sliding window of recent raw samples, used for standstill detection.
export class MotionWindow {
  constructor(span = 1.0) {
    this.span = span;
    this.buf = [];
  }

  push(t, accel, gyro) {
    this.buf.push({ t, a: accel, g: gyro });
    while (this.buf.length && this.buf[0].t < t - this.span) this.buf.shift();
  }

  get duration() {
    const b = this.buf;
    return b.length > 1 ? b[b.length - 1].t - b[0].t : 0;
  }

  // accelStd: raw per-axis std (includes engine/road vibration).
  // accelDrift: spread of 4 sub-block means, i.e. low-frequency motion; vibration averages out.
  stats() {
    const b = this.buf, n = b.length;
    const mA = [0, 0, 0], mG = [0, 0, 0], sA = [0, 0, 0];
    for (const s of b) for (let k = 0; k < 3; k++) { mA[k] += s.a[k] / n; mG[k] += s.g[k] / n; }
    for (const s of b) for (let k = 0; k < 3; k++) sA[k] += (s.a[k] - mA[k]) ** 2 / n;
    const blocks = 4, per = Math.floor(n / blocks);
    let drift = 0;
    if (per > 0) {
      const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
      for (let j = 0; j < blocks; j++) {
        const m = [0, 0, 0];
        for (let i = j * per; i < (j + 1) * per; i++) for (let k = 0; k < 3; k++) m[k] += b[i].a[k] / per;
        for (let k = 0; k < 3; k++) { lo[k] = Math.min(lo[k], m[k]); hi[k] = Math.max(hi[k], m[k]); }
      }
      drift = Math.max(hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]);
    }
    const accelStd = Math.sqrt(Math.max(...sA));
    // spread of block means expected from vibration alone (white-noise approximation)
    const vibrationSpread = per > 0 ? 4 * accelStd / Math.sqrt(per) : Infinity;
    return { n, meanAccel: mA, meanGyro: mG, accelStd, accelDrift: drift, vibrationSpread };
  }
}

// Accumulates a standstill period and produces gravity direction and gyro bias (raw units).
export class StandstillCalibrator {
  constructor(minDuration = 3) {
    this.minDuration = minDuration;
    this.reset();
  }

  reset() {
    this.n = 0;
    this.t0 = null;
    this.sa = [0, 0, 0]; this.sg = [0, 0, 0];
    this.sa2 = [0, 0, 0]; this.sg2 = [0, 0, 0];
    this.tLast = null;
  }

  add(t, accel, gyro) {
    if (this.t0 === null) this.t0 = t;
    this.tLast = t;
    this.n++;
    for (let k = 0; k < 3; k++) {
      this.sa[k] += accel[k]; this.sg[k] += gyro[k];
      this.sa2[k] += accel[k] ** 2; this.sg2[k] += gyro[k] ** 2;
    }
  }

  get duration() {
    return this.t0 === null ? 0 : this.tLast - this.t0;
  }

  get done() {
    return this.duration >= this.minDuration && this.n >= 30;
  }

  result() {
    const n = this.n;
    const ma = this.sa.map((x) => x / n), mg = this.sg.map((x) => x / n);
    const sdA = Math.sqrt(Math.max(...this.sa2.map((x, k) => Math.max(x / n - ma[k] ** 2, 0))));
    const sdG = Math.sqrt(Math.max(...this.sg2.map((x, k) => Math.max(x / n - mg[k] ** 2, 0))));
    return {
      up: normalize(ma),
      gravity: Math.hypot(...ma),
      gyroBias: mg,
      accelStd: sdA,
      gyroStd: sdG,
      rate: (n - 1) / this.duration,
    };
  }
}

function interp(T, C, tq) {
  const n = T.length;
  if (n < 2 || tq < T[0] || tq > T[n - 1]) return null;
  let lo = 0, hi = n - 1;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (T[mid] <= tq) lo = mid; else hi = mid;
  }
  const f = (tq - T[lo]) / (T[hi] - T[lo] || 1);
  return C[lo] + f * (C[hi] - C[lo]);
}

// First-order low-pass of a sampled series (models GNSS smoothing).
function lpfSeries(T, X, tau) {
  if (!tau) return X;
  const Y = new Array(X.length);
  Y[0] = X[0];
  for (let i = 1; i < X.length; i++) Y[i] = Y[i - 1] + (1 - Math.exp(-(T[i] - T[i - 1]) / tau)) * (X[i] - Y[i - 1]);
  return Y;
}

const GAIN_CANDIDATES = [1, 180 / Math.PI, Math.PI / 180];
const SMOOTHING_CANDIDATES = [0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2, 1.35, 1.5];

// Background sensor check while driving. solve() returns
//   { sensors, full }
// sensors (after ~150° of turning): gyro axis order, gyro units, raw gain sign,
//   GNSS latency and smoothing.
// full (needs turns plus speed changes): additionally accelerometer handedness, the
//   corrected gyro sign and a turn-based mount estimate (the slow path used when no
//   straight acceleration has been seen).
export class DriveCalibrator {
  // standstill: StandstillCalibrator.result()
  constructor(standstill, opts = {}) {
    this.upRaw = standstill.up;
    this.biasRaw = standstill.gyroBias;
    this.opts = {
      minSpeed: 4, minTurnSensors: 100 * DEG, minPairsSensors: 4,
      minTurn: 2 * Math.PI, minPairs: 20, maxAlphaSigma: 2 * DEG, maxLag: 1.2, ...opts,
    };
    ({ e1: this.e1, e2: this.e2 } = levelBasis(this.upRaw));
    this.T = []; this.cR = GYRO_MAPS.map(() => []); this.cA1 = []; this.cA2 = [];
    this._last = null;
    this.fixes = [];
    this.status = { progress: 0, reason: 'collecting' };
    this.sensors = null;
  }

  addImu(t, gyroRaw, accelRaw) {
    const g = [gyroRaw[0] - this.biasRaw[0], gyroRaw[1] - this.biasRaw[1], gyroRaw[2] - this.biasRaw[2]];
    const r = GYRO_MAPS.map((m) => dot(m(g), this.upRaw));
    const a1 = dot(accelRaw, this.e1), a2 = dot(accelRaw, this.e2);
    const n = this.T.length;
    if (n === 0) {
      this.T.push(t); this.cA1.push(0); this.cA2.push(0);
      for (const c of this.cR) c.push(0);
    } else {
      const dt = t - this.T[n - 1];
      if (dt <= 0) return;
      const p = this._last;
      this.T.push(t);
      for (let m = 0; m < r.length; m++) this.cR[m].push(this.cR[m][n - 1] + 0.5 * (r[m] + p.r[m]) * dt);
      this.cA1.push(this.cA1[n - 1] + 0.5 * (a1 + p.a1) * dt);
      this.cA2.push(this.cA2[n - 1] + 0.5 * (a2 + p.a2) * dt);
    }
    this._last = { r, a1, a2 };
  }

  // fix: { t, speed, chi } with chi = course as math angle (rad, CCW from east) or null.
  addGnss(fix) {
    this.fixes.push(fix);
  }

  // Non-overlapping windows between fixes, at least minDt long. Long windows make smoothed
  // GNSS (common with fused location providers) behave like a pure delay; short ones give
  // more clean windows while drifting.
  _pairs(minDt = 1.9) {
    const out = [];
    const F = this.fixes, minV = this.opts.minSpeed;
    let a = null;
    for (const b of F) {
      if (b.chi === null || b.speed < minV) { a = null; continue; }
      if (a === null) { a = b; continue; }
      const dt = b.t - a.t;
      if (dt < minDt) continue;
      if (dt > 3.5) { a = b; continue; }
      const prev = a;
      a = b;
      out.push({
        t0: prev.t, t1: b.t, dt,
        dChi: wrapPi(b.chi - prev.chi),
        vbar: 0.5 * (prev.speed + b.speed),
        along: (b.speed - prev.speed) / dt,
      });
    }
    return out;
  }

  // Best (smoothing, latency, gain) for one gyro axis order: dChi ~ k * dPsi_smoothed(L).
  // Robust: windows where course and heading turned differently (drift entries, exits and
  // transitions, where the drift angle changed) are outliers; their squared residual is
  // capped so they cannot pull the fit. Steady-angle drifting still counts as good data.
  _fitMap(cR, pairs, smoothingCandidates) {
    const cand = latencyCandidates(this.opts.maxLag);
    const minN = Math.min(this.opts.minPairs, pairs.length);
    const CAP = (4 * DEG) ** 2;
    const fitLag = (tau) => {
      const fR = lpfSeries(this.T, cR, tau);
      const cost = new Float64Array(cand.length).fill(Infinity);
      const gains = new Float64Array(cand.length);
      const d = new Float64Array(pairs.length);
      for (let j = 0; j < cand.length; j++) {
        const L = cand[j];
        let sxy = 0, sxx = 0, n = 0;
        for (let i = 0; i < pairs.length; i++) {
          const p = pairs[i];
          const a = interp(this.T, fR, p.t0 - L), b = interp(this.T, fR, p.t1 - L);
          d[i] = a === null || b === null ? NaN : b - a;
          if (Number.isNaN(d[i])) continue;
          sxy += d[i] * p.dChi; sxx += d[i] * d[i]; n++;
        }
        if (n < minN || sxx <= 0) continue;
        // gain from least squares, then refitted on the windows it explains (twice)
        let k = sxy / sxx;
        for (let it = 0; it < 2; it++) {
          let ixy = 0, ixx = 0;
          for (let i = 0; i < pairs.length; i++) {
            if (Number.isNaN(d[i])) continue;
            if ((pairs[i].dChi - k * d[i]) ** 2 < CAP) { ixy += d[i] * pairs[i].dChi; ixx += d[i] * d[i]; }
          }
          if (ixx > 0) k = ixy / ixx;
        }
        let c = 0, inl = 0, inlTurn = 0;
        for (let i = 0; i < pairs.length; i++) {
          if (Number.isNaN(d[i])) continue;
          const e2 = (pairs[i].dChi - k * d[i]) ** 2;
          c += Math.min(e2, CAP);
          if (e2 < CAP) { inl++; inlTurn += Math.abs(pairs[i].dChi); }
        }
        gains[j] = k;
        cost[j] = c / n;
        inliers[j] = inl / n;
        inlierTurn[j] = inlTurn;
      }
      const { x, index } = argminRefined(cand, cost);
      return {
        cost: cost[index], latency: x, k: gains[index], tau, fR,
        inlierFraction: inliers[index], inlierTurn: inlierTurn[index],
      };
    };
    const inliers = new Float64Array(cand.length), inlierTurn = new Float64Array(cand.length);
    if (smoothingCandidates.length === 1) return fitLag(smoothingCandidates[0]);
    const fits = smoothingCandidates.map(fitLag);
    const S = smoothingCandidates;
    const iBest = argminRefined(S, fits.map((f) => f.cost));
    // argminRefined assumes an even grid; refine only between equally spaced neighbours
    const i = iBest.index;
    const evenNeighbours = i > 0 && i < fits.length - 1 &&
      Math.abs((S[i + 1] - S[i]) - (S[i] - S[i - 1])) < 1e-9;
    let best = fits[i];
    if (evenNeighbours && Math.abs(iBest.x - best.tau) > 0.01) {
      const refined = fitLag(Math.max(0, iBest.x));
      if (refined.cost < best.cost) best = refined;
    }
    return best;
  }

  // Attempt to solve (at most about every 2 s). Returns null while nothing is conclusive
  // (see this.status).
  solve() {
    const F = this.fixes;
    const tNow = F.length ? F[F.length - 1].t : 0;
    if (this._lastSolve !== undefined && tNow - this._lastSolve < 2) return null;
    this._lastSolve = tNow;
    const pairs = this._pairs();
    const turn = pairs.reduce((s, p) => s + Math.abs(p.dChi), 0);
    this.status = {
      progress: Math.min(1, Math.min(turn / this.opts.minTurnSensors, pairs.length / this.opts.minPairsSensors)),
      reason: 'collecting',
    };
    if (turn < this.opts.minTurnSensors || pairs.length < this.opts.minPairsSensors) return null;

    // 1. Gyro axis order: the order whose yaw explains the course changes, judged on 1 s
    //    turning windows and fitted with a pure GNSS delay (smoothing needs more data to
    //    separate from delay). Decided only with a clear margin and when most turning
    //    windows agree; while drifting, entries and transitions do not count.
    const turning = this._pairs(0.9).filter((p) => Math.abs(p.dChi) > 4 * DEG);
    if (turning.length < this.opts.minPairsSensors) return null;
    // A wrong axis order cannot explain the turning with a plausible gain (±1, or a
    // deg/rad unit mix-up); the right one does, on most of the turning.
    const classify = (k, tol) => {
      if (!Number.isFinite(k) || k === 0) return null;
      let unit = GAIN_CANDIDATES[0];
      for (const c of GAIN_CANDIDATES) if (Math.abs(Math.log(Math.abs(k) / c)) < Math.abs(Math.log(Math.abs(k) / unit))) unit = c;
      return Math.abs(Math.abs(k) / unit - 1) > tol ? null : unit;
    };
    const quick = this.cR.map((cR) => this._fitMap(cR, turning, [0]));
    const ok = quick.map((f) => Number.isFinite(f.cost) && classify(f.k, 0.15) !== null &&
      f.inlierFraction >= 0.35 && f.inlierTurn >= this.opts.minTurnSensors);
    let m = -1;
    for (let j = 0; j < quick.length; j++) if (ok[j] && (m < 0 || quick[j].cost < quick[m].cost)) m = j;
    const nOk = ok.filter(Boolean).length;
    const margin = m < 0 ? 0 : Math.min(...quick.filter((_, j) => j !== m).map((f) => f.cost)) / Math.max(quick[m].cost, 1e-12);
    if (m < 0 || (nOk > 1 && margin < 1.5)) {
      // Early switch: the assumed order is clearly implausible and exactly one other order
      // already explains the turning. Leaving a wrong order is urgent (its angles are
      // garbage); confirming the right one can wait for more data.
      const assumed = this.opts.assumedMap ?? 0;
      const early = quick.map((f, j) => j !== assumed && Number.isFinite(f.cost) && classify(f.k, 0.15) !== null &&
        f.inlierFraction >= 0.35 && f.inlierTurn >= 0.5 * this.opts.minTurnSensors);
      if (classify(quick[assumed].k, 0.35) === null && early.filter(Boolean).length === 1) {
        m = early.indexOf(true);
      } else {
        this.status.reason = 'checking sensor axes';
        return null;
      }
    }
    const unit = classify(quick[m].k, 0.15);
    const sensors = {
      gyroMap: m, gyroUnit: unit, kRaw: quick[m].k, latency: Math.max(0, quick[m].latency), smoothing: 0, margin,
    };
    this.sensors = sensors;
    this.status.progress = Math.min(1, Math.min(turn / this.opts.minTurn, pairs.length / this.opts.minPairs));
    if (turn < this.opts.minTurn || pairs.length < this.opts.minPairs) return { sensors, full: null };

    // With plenty of turning: GNSS smoothing and delay separately, for the chosen axis order.
    const { latency, k, tau: smoothing, fR } = this._fitMap(this.cR[m], pairs, SMOOTHING_CANDIDATES);

    // 2. Horizontal acceleration windows (IMU smoothed and shifted like the GNSS).
    const fA1 = lpfSeries(this.T, this.cA1, smoothing), fA2 = lpfSeries(this.T, this.cA2, smoothing);
    const W = [];
    for (const p of pairs) {
      const t0 = p.t0 - latency, t1 = p.t1 - latency;
      const r0 = interp(this.T, fR, t0), r1 = interp(this.T, fR, t1);
      const x0 = interp(this.T, fA1, t0), x1 = interp(this.T, fA1, t1);
      const y0 = interp(this.T, fA2, t0), y1 = interp(this.T, fA2, t1);
      if (r0 === null || r1 === null) continue;
      W.push({
        p,
        dPsiRaw: r1 - r0,
        w: [(x1 - x0) / p.dt, (y1 - y0) / p.dt],
      });
    }

    // 3. Accelerometer handedness: GNSS-only kinematics vs horizontal accel.
    //    A rotation fits a correct sensor, a reflection fits a sign-inverted one.
    //    Telling them apart needs both turning and speed changes, so the decision waits
    //    until the residual difference is clearly significant.
    let Sxx = 0, Sxy = 0, Syx = 0, Syy = 0, sw2 = 0, su2 = 0;
    for (const { p, w } of W) {
      const u = [p.along, p.vbar * p.dChi / p.dt];
      Sxx += u[0] * w[0]; Sxy += u[0] * w[1]; Syx += u[1] * w[0]; Syy += u[1] * w[1];
      sw2 += w[0] * w[0] + w[1] * w[1]; su2 += u[0] * u[0] + u[1] * u[1];
    }
    const rot = Math.hypot(Sxx + Syy, Sxy - Syx), ref = Math.hypot(Sxx - Syy, Sxy + Syx);
    const resRot = sw2 + su2 - 2 * rot, resRef = sw2 + su2 - 2 * ref;
    const noise = Math.max(Math.min(resRot, resRef), 1e-9) / Math.max(2 * W.length - 1, 1);
    const handedness = Math.abs(resRot - resRef) / noise;
    this.status.handedness = handedness;
    if (handedness < 30) {
      this.status.reason = 'need some accelerating and braking';
      return { sensors, full: null };
    }
    const accelSign = ref > rot ? -1 : 1;

    // 4. Gyro sign after the accel correction (flipping accel flips "up").
    const kCorr = k * accelSign;
    const gyroGain = Math.sign(kCorr) * unit;

    // 5. Mount yaw: rotation from vehicle [along, lateral] to leveled phone [e1, e2],
    //    lateral taken from the (timely) gyro.
    Sxx = 0; Sxy = 0; Syx = 0; Syy = 0;
    const UW = [];
    for (const { p, w, dPsiRaw } of W) {
      const u = [p.along, p.vbar * dPsiRaw * k / p.dt];
      const wc = [accelSign * w[0], w[1]];
      UW.push([u, wc]);
      Sxx += u[0] * wc[0]; Sxy += u[0] * wc[1]; Syx += u[1] * wc[0]; Syy += u[1] * wc[1];
    }
    const alpha = Math.atan2(Sxy - Syx, Sxx + Syy);
    const c = Math.cos(alpha), s = Math.sin(alpha);
    let res = 0, uu = 0;
    for (const [u, w] of UW) {
      res += (w[0] - (c * u[0] - s * u[1])) ** 2 + (w[1] - (s * u[0] + c * u[1])) ** 2;
      uu += u[0] * u[0] + u[1] * u[1];
    }
    const alphaSigma = Math.sqrt(res / Math.max(2 * UW.length - 1, 1) / uu);
    if (alphaSigma > this.opts.maxAlphaSigma) {
      this.status.reason = 'mount direction not yet conclusive';
      return { sensors, full: null };
    }

    // Forward in raw accelerometer axes (see mountRotation for the sign convention).
    const up = scale(this.upRaw, accelSign);
    const e2 = cross(up, this.e1);
    const xv = [c * this.e1[0] + s * e2[0], c * this.e1[1] + s * e2[1], c * this.e1[2] + s * e2[2]];
    this.status = { progress: 1, reason: 'done' };
    return {
      sensors,
      full: {
        gyroMap: m,
        accelSign,
        gyroGain,
        fwdRaw: scale(xv, accelSign),
        alphaSigma: Math.max(alphaSigma, 0.5 * DEG),
        latency: Math.max(0, latency),
        smoothing,
        pairs: pairs.length,
      },
    };
  }
}
