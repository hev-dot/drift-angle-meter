// Self-calibration: everything that differs between phones, browsers and mounts is
// found at runtime instead of being configured.
//
// 1. Standstill (a few seconds): gravity direction in phone axes -> "up",
//    gyro bias, sensor noise levels.
// 2. Normal driving with some turns (about a minute): compared against GNSS,
//    - GNSS latency and gyro gain (units / sign faults), from course vs gyro yaw,
//    - accelerometer handedness (sign fault), from horizontal accel vs GNSS kinematics,
//    - mount yaw (which way is forward), from horizontal accel vs [along-track, centripetal].

import {
  normalize, dot, cross, scale, fromRows, wrapPi, DEG,
} from './math.js';
import { latencyCandidates, argminRefined } from './latency.js';

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

export class DriveCalibrator {
  // standstill: StandstillCalibrator.result()
  constructor(standstill, opts = {}) {
    this.upRaw = standstill.up;
    this.biasRaw = standstill.gyroBias;
    this.opts = { minSpeed: 4, minTurn: 2 * Math.PI, minPairs: 20, maxAlphaSigma: 2 * DEG, maxLag: 1.2, ...opts };
    // e1: the phone axis closest to horizontal, projected onto the horizontal plane.
    const axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    let best = axes[0];
    for (const a of axes) if (Math.abs(dot(a, this.upRaw)) < Math.abs(dot(best, this.upRaw))) best = a;
    this.e1 = normalize(cross(cross(this.upRaw, best), this.upRaw));
    this.e2 = cross(this.upRaw, this.e1);
    this.T = []; this.cR = []; this.cA1 = []; this.cA2 = [];
    this._last = null;
    this.fixes = [];
    this.status = { progress: 0, reason: 'collecting' };
  }

  addImu(t, gyroRaw, accelRaw) {
    const g = [gyroRaw[0] - this.biasRaw[0], gyroRaw[1] - this.biasRaw[1], gyroRaw[2] - this.biasRaw[2]];
    const r = dot(g, this.upRaw);
    const a1 = dot(accelRaw, this.e1), a2 = dot(accelRaw, this.e2);
    const n = this.T.length;
    if (n === 0) {
      this.T.push(t); this.cR.push(0); this.cA1.push(0); this.cA2.push(0);
    } else {
      const dt = t - this.T[n - 1];
      if (dt <= 0) return;
      const p = this._last;
      this.T.push(t);
      this.cR.push(this.cR[n - 1] + 0.5 * (r + p.r) * dt);
      this.cA1.push(this.cA1[n - 1] + 0.5 * (a1 + p.a1) * dt);
      this.cA2.push(this.cA2[n - 1] + 0.5 * (a2 + p.a2) * dt);
    }
    this._last = { r, a1, a2 };
  }

  // fix: { t, speed, chi } with chi = course as math angle (rad, CCW from east) or null.
  addGnss(fix) {
    this.fixes.push(fix);
  }

  // Non-overlapping windows of ~2 s between fixes. Long windows make smoothed GNSS
  // (common with fused location providers) behave like a pure delay.
  _pairs() {
    const out = [];
    const F = this.fixes, minV = this.opts.minSpeed;
    let a = null;
    for (const b of F) {
      if (b.chi === null || b.speed < minV) { a = null; continue; }
      if (a === null) { a = b; continue; }
      const dt = b.t - a.t;
      if (dt < 1.9) continue;
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

  // Attempt to solve (at most about once a second). Returns null while more data is
  // needed (see this.status).
  solve() {
    const F = this.fixes;
    const tNow = F.length ? F[F.length - 1].t : 0;
    if (this._lastSolve !== undefined && tNow - this._lastSolve < 1) return null;
    this._lastSolve = tNow;
    const pairs = this._pairs();
    const turn = pairs.reduce((s, p) => s + Math.abs(p.dChi), 0);
    this.status = {
      progress: Math.min(1, Math.min(turn / this.opts.minTurn, pairs.length / this.opts.minPairs)),
      reason: 'collecting',
    };
    if (turn < this.opts.minTurn || pairs.length < this.opts.minPairs) return null;

    // 1. GNSS smoothing, latency and gyro gain: dChi ~ k * dPsi_smoothed(L), over a grid
    //    of smoothing time constants (refined), each with a latency grid (refined).
    const cand = latencyCandidates(this.opts.maxLag);
    const fitLag = (tau) => {
      const fR = lpfSeries(this.T, this.cR, tau);
      const cost = new Float64Array(cand.length).fill(Infinity);
      const gains = new Float64Array(cand.length);
      for (let j = 0; j < cand.length; j++) {
        const L = cand[j];
        let sxy = 0, sxx = 0, syy = 0, n = 0;
        for (const p of pairs) {
          const a = interp(this.T, fR, p.t0 - L), b = interp(this.T, fR, p.t1 - L);
          if (a === null || b === null) continue;
          const d = b - a;
          sxy += d * p.dChi; sxx += d * d; syy += p.dChi * p.dChi; n++;
        }
        if (n < this.opts.minPairs || sxx <= 0) continue;
        gains[j] = sxy / sxx;
        cost[j] = (syy - sxy * sxy / sxx) / n;
      }
      const { x, index } = argminRefined(cand, cost);
      return { cost: cost[index], latency: x, k: gains[index], tau, fR };
    };
    const fits = SMOOTHING_CANDIDATES.map(fitLag);
    const iBest = argminRefined(SMOOTHING_CANDIDATES, fits.map((f) => f.cost));
    // argminRefined assumes an even grid; refine only between equally spaced neighbours
    const i = iBest.index;
    const evenNeighbours = i > 0 && i < fits.length - 1 &&
      Math.abs((SMOOTHING_CANDIDATES[i + 1] - SMOOTHING_CANDIDATES[i]) - (SMOOTHING_CANDIDATES[i] - SMOOTHING_CANDIDATES[i - 1])) < 1e-9;
    let best = fits[i];
    if (evenNeighbours && Math.abs(iBest.x - best.tau) > 0.01) {
      const refined = fitLag(Math.max(0, iBest.x));
      if (refined.cost < best.cost) best = refined;
    }
    if (!Number.isFinite(best.cost)) return null;
    const { latency, k, tau: smoothing, fR } = best;
    let unit = GAIN_CANDIDATES[0];
    for (const c of GAIN_CANDIDATES) if (Math.abs(Math.log(Math.abs(k) / c)) < Math.abs(Math.log(Math.abs(k) / unit))) unit = c;
    if (Math.abs(Math.abs(k) / unit - 1) > 0.25) {
      this.status.reason = 'gyro does not match GNSS turning';
      return null;
    }

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
      return null;
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
      return null;
    }

    // Vehicle frame axes expressed in phone axes.
    const up = scale(this.upRaw, accelSign);
    const e2 = cross(up, this.e1);
    const xv = [c * this.e1[0] + s * e2[0], c * this.e1[1] + s * e2[1], c * this.e1[2] + s * e2[2]];
    const yv = cross(up, xv);
    this.status = { progress: 1, reason: 'done' };
    return {
      accelSign,
      gyroGain,
      gyroBiasRaw: this.biasRaw,
      R: fromRows(xv, yv, up), // vehicle <- phone
      alphaSigma: Math.max(alphaSigma, 0.5 * DEG),
      latency: Math.max(0, latency),
      smoothing,
      pairs: pairs.length,
      gainFit: kCorr / (Math.sign(kCorr) * unit),
    };
  }
}
