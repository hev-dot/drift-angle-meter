// GNSS latency estimation. Browsers deliver location fixes late (and often smoothed);
// the delay differs per phone, browser and OS version. It is found by comparing the
// course change between two fixes with the gyro yaw integrated over the same interval,
// shifted by each candidate lag. The gyro is (nearly) undelayed, so the lag with the
// smallest residual is the GNSS delay relative to the IMU.

import { wrapPi } from './math.js';

// Running integral of a signal (trapezoidal), optionally passed through the same
// first-order smoothing as the GNSS (tau), kept for a limited time span.
export class IntegralHistory {
  constructor(span = 5, tau = 0) {
    this.span = span;
    this.tau = tau;
    this.t = [];
    this.c = [];
    this._raw = 0;
  }

  push(t, x) {
    const n = this.t.length;
    if (n === 0) {
      this.t.push(t); this.c.push(0); this._x = x;
      return;
    }
    const dt = t - this.t[n - 1];
    if (dt <= 0) return;
    this._raw += 0.5 * (x + this._x) * dt;
    const prev = this.c[n - 1];
    this.t.push(t);
    this.c.push(this.tau ? prev + (1 - Math.exp(-dt / this.tau)) * (this._raw - prev) : this._raw);
    this._x = x;
    // drop old samples in chunks to keep shift() cost low
    if (this.t[0] < t - this.span - 1) {
      let k = 0;
      while (k < this.t.length && this.t[k] < t - this.span) k++;
      this.t.splice(0, k); this.c.splice(0, k);
    }
  }

  // Integral value at time tq, or null when outside the stored range.
  at(tq) {
    const T = this.t, n = T.length;
    if (n < 2 || tq < T[0] || tq > T[n - 1]) return null;
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (T[mid] <= tq) lo = mid; else hi = mid;
    }
    const f = (tq - T[lo]) / (T[hi] - T[lo] || 1);
    return this.c[lo] + f * (this.c[hi] - this.c[lo]);
  }

  // Mean of the integrated signal over [tq - h, tq + h].
  rateAt(tq, h = 0.05) {
    const n = this.t.length;
    if (n < 2) return null;
    const t0 = Math.max(tq - h, this.t[0]), t1 = Math.min(tq + h, this.t[n - 1]);
    if (t1 - t0 < h) return null;
    return (this.at(t1) - this.at(t0)) / (t1 - t0);
  }
}

export function latencyCandidates(maxLag = 1.2, step = 0.02) {
  const out = [];
  for (let L = 0; L <= maxLag + 1e-9; L += step) out.push(L);
  return out;
}

// Parabolic refinement of the minimum of a sampled cost curve.
export function argminRefined(xs, cost) {
  let j = 0;
  for (let i = 1; i < cost.length; i++) if (cost[i] < cost[j]) j = i;
  if (j === 0 || j === cost.length - 1) return { x: xs[j], index: j };
  const a = cost[j - 1], b = cost[j], c = cost[j + 1];
  const den = a - 2 * b + c;
  const off = den > 0 ? 0.5 * (a - c) / den : 0;
  return { x: xs[j] + off * (xs[1] - xs[0]), index: j };
}

// Online tracker used while the estimator runs (gyro already calibrated, gain = 1).
export class LatencyTracker {
  constructor({ initial = 0.15, maxLag = 1.2, step = 0.02, forget = 0.985 } = {}) {
    this.L = initial;
    this.cand = latencyCandidates(maxLag, step);
    this.cost = new Float64Array(this.cand.length);
    this.info = 0;
    this.forget = forget;
    this.updates = 0;
  }

  // dChi: course change between fixes at (arrival) times t0 -> t1, measured at the phone;
  // yaw: IntegralHistory of vehicle yaw rate; v0, v1: speeds at the fixes; px: phone distance
  // ahead of the rear axle. While gripping, the phone's course leads the heading by
  // atan(r * px / v), which is accounted for per candidate lag.
  addCourseDelta(t0, t1, dChi, yaw, v0, v1, px) {
    const d = new Float64Array(this.cand.length);
    let lo = Infinity, hi = -Infinity;
    for (let j = 0; j < this.cand.length; j++) {
      const a = yaw.at(t0 - this.cand[j]), b = yaw.at(t1 - this.cand[j]);
      const r0 = yaw.rateAt(t0 - this.cand[j]), r1 = yaw.rateAt(t1 - this.cand[j]);
      if (a === null || b === null || r0 === null || r1 === null) return false;
      d[j] = b - a + Math.atan2(r1 * px, v1) - Math.atan2(r0 * px, v0);
      if (d[j] < lo) lo = d[j];
      if (d[j] > hi) hi = d[j];
    }
    const spread = hi - lo;
    if (spread < 0.004) return false; // yaw rate nearly constant: carries no timing information
    let best = Infinity;
    for (let j = 0; j < d.length; j++) best = Math.min(best, Math.abs(wrapPi(dChi - d[j])));
    if (best > 0.2) return false; // outlier (GNSS glitch or unmodelled slip)
    for (let j = 0; j < d.length; j++) {
      const e = wrapPi(dChi - d[j]);
      this.cost[j] = this.forget * this.cost[j] + e * e;
    }
    this.info = this.forget * this.info + spread * spread;
    this.updates++;
    if (this.info > 0.02 && this.updates >= 8) {
      const { x } = argminRefined(this.cand, this.cost);
      this.L += 0.3 * (x - this.L);
    }
    return true;
  }

  get converged() {
    return this.info > 0.02 && this.updates >= 8;
  }
}
