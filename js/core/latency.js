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

// Online GNSS delay and smoothing estimate from velocity changes, used while the filter
// runs. Between two fixes the reported velocity changes by the integral of the true
// (nav-frame, horizontal) acceleration over the same interval, shifted by the delay and
// passed through the receiver's smoothing. The filter's acceleration (accelerometer
// rotated by the estimated attitude) is timely, so the (smoothing, delay) cell whose
// shifted integral best matches the GNSS velocity changes is the answer. Unlike course
// changes, this is not confounded by the drift angle, so it learns during launches,
// braking, corner entries and drifts alike.
export class VelocityLagTracker {
  constructor({
    initial = 0.2, initialSmoothing = 0, maxLag = 1.2, step = 0.02,
    taus = [0, 0.3, 0.6, 0.9, 1.2], memory = 60, span = 5,
  } = {}) {
    this.L = initial;
    this.tau = initialSmoothing;
    this.cand = latencyCandidates(maxLag, step);
    this.taus = taus;
    this.hist = taus.map((tau) => [new IntegralHistory(span, tau), new IntegralHistory(span, tau)]);
    this.cost = taus.map(() => new Float64Array(this.cand.length));
    this.memory = memory; // s
    this.info = 0;
    this.updates = 0;
    this._converged = false;
  }

  // Horizontal nav-frame acceleration (east, north), m/s², at IMU time t (s).
  push(t, aE, aN) {
    for (const [hE, hN] of this.hist) { hE.push(t, aE); hN.push(t, aN); }
  }

  // Velocity change (east, north) between consecutive fixes arriving at t0 and t1.
  addFix(t0, t1, dvE, dvN) {
    const nT = this.taus.length, nL = this.cand.length;
    const pE = new Float64Array(nT * nL), pN = new Float64Array(nT * nL);
    for (let i = 0; i < nT; i++) {
      const [hE, hN] = this.hist[i];
      // a fix can arrive a few ms after the newest IMU sample
      const end = hE.t[hE.t.length - 1];
      const q = (tq) => (tq > end && tq - end < 0.1 ? end : tq);
      for (let j = 0; j < nL; j++) {
        const L = this.cand[j];
        const a = q(t0 - L), b = q(t1 - L);
        const e0 = hE.at(a), e1 = hE.at(b), n0 = hN.at(a), n1 = hN.at(b);
        if (e0 === null || e1 === null || n0 === null || n1 === null) return false;
        pE[i * nL + j] = e1 - e0;
        pN[i * nL + j] = n1 - n0;
      }
    }
    let best = Infinity;
    for (let k = 0; k < nT * nL; k++) best = Math.min(best, (dvE - pE[k]) ** 2 + (dvN - pN[k]) ** 2);
    if (best > 4) return false; // glitch, or the filter is not tracking
    // timing information in this interval: how much the prediction depends on the delay
    let mE = 0, mN = 0;
    for (let j = 0; j < nL; j++) { mE += pE[j] / nL; mN += pN[j] / nL; }
    let spread = 0;
    for (let j = 0; j < nL; j++) spread += ((pE[j] - mE) ** 2 + (pN[j] - mN) ** 2) / nL;
    const cap = 1.5 ** 2;
    const f = Math.exp(-(t1 - t0) / this.memory); // same memory in seconds at any fix rate
    for (let i = 0; i < nT; i++) {
      const c = this.cost[i];
      for (let j = 0; j < nL; j++) {
        const k = i * nL + j;
        c[j] = f * c[j] + Math.min((dvE - pE[k]) ** 2 + (dvN - pN[k]) ** 2, cap);
      }
    }
    this.info = f * this.info + spread;
    this.updates++;
    if (this.info > 1.5 && this.updates >= 10) this._converged = true;
    if (this.converged) {
      let bi = 0, bj = 0;
      for (let i = 0; i < nT; i++) for (let j = 0; j < nL; j++) if (this.cost[i][j] < this.cost[bi][bj]) { bi = i; bj = j; }
      const { x } = argminRefined(this.cand, this.cost[bi]);
      this.tau = this.taus[bi];
      this.L += 0.3 * (x - this.L);
    }
    return true;
  }

  get converged() {
    return this._converged;
  }

  // How much better the best smoothing row explains the data than the row nearest `tau`
  // (ratio of their minimum costs, < 1 means better).
  smoothingImprovement(tau) {
    let i = 0;
    for (let k = 1; k < this.taus.length; k++) if (Math.abs(this.taus[k] - tau) < Math.abs(this.taus[i] - tau)) i = k;
    const rowMin = (r) => Math.min(...this.cost[r]);
    let b = 0;
    for (let k = 1; k < this.taus.length; k++) if (rowMin(k) < rowMin(b)) b = k;
    return { tau: this.taus[b], ratio: rowMin(b) / Math.max(rowMin(i), 1e-12) };
  }

  // Best delay given the smoothing the filter currently models (its nearest grid row).
  lagFor(tau) {
    if (!this.converged) return this.L;
    let i = 0;
    for (let k = 1; k < this.taus.length; k++) if (Math.abs(this.taus[k] - tau) < Math.abs(this.taus[i] - tau)) i = k;
    return Math.max(0, argminRefined(this.cand, this.cost[i]).x);
  }
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
    if (best > 0.07) return false; // outlier (GNSS glitch or unmodelled slip)
    const cap = 0.07 ** 2;
    for (let j = 0; j < d.length; j++) {
      const e = wrapPi(dChi - d[j]);
      this.cost[j] = this.forget * this.cost[j] + Math.min(e * e, cap);
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
