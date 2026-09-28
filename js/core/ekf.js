// Error-state EKF: strapdown INS on the phone IMU, aided by GNSS velocity and
// vehicle pseudo-measurements (non-holonomic constraint, zero vertical velocity,
// zero velocity/rate at standstill).
//
// Frames
//   nav (n): local level ENU, z up.
//   body (b): phone IMU axes after the coarse mount rotation from calibration,
//             i.e. approximately vehicle x-forward, y-left, z-up.
//   vehicle (v): body rotated about z by the residual mount yaw `alpha`.
//
// Error state: 0-2 attitude (nav frame, C_true = (I + [dθ×]) C), 3-5 velocity,
// 6-8 gyro bias, 9-11 accel bias, 12 gyro-z scale factor, 13 residual mount yaw,
// and when the GNSS is smoothed (first-order, time constant tau): 14-15 the
// smoothed horizontal velocity that the GNSS actually reports.

import {
  G, qmul, qnormalize, qFromRotVec, qToMat, m3v, m3tv, cross, invSmall, clamp,
} from './math.js';

const I_V = 3, I_BG = 6, I_BA = 9, I_S = 12, I_AL = 13, I_VS = 14;

export const DEFAULT_NOISE = {
  gyroDensity: 0.004,       // rad/s/√Hz, white gyro noise incl. vibration
  accelDensity: 0.12,       // m/s²/√Hz, white accel noise incl. road vibration
  gyroBiasWalk: 0.0003,     // rad/s/√s
  accelBiasWalk: 0.004,     // m/s²/√s
  scaleWalk: 1e-4,          // 1/√s
  alphaWalk: 2e-4,          // rad/√s
};

export class DriftEKF {
  // smoothing: GNSS velocity smoothing time constant (s); below 0.1 s it is treated as a pure delay.
  constructor(noise = {}, { smoothing = 0 } = {}) {
    this.noise = { ...DEFAULT_NOISE, ...noise };
    this.tau = smoothing >= 0.1 ? smoothing : 0;
    this.n = this.tau ? 16 : 14;
    this.q = [1, 0, 0, 0];
    this.v = [0, 0, 0];
    this.vs = [0, 0];
    this.bg = [0, 0, 0];
    this.ba = [0, 0, 0];
    this.s = 0;
    this.alpha = 0;
    this.P = new Float64Array(this.n * this.n);
    this.w = [0, 0, 0];     // last bias/scale corrected angular rate (body)
    this.wzRaw = 0;         // last gyro z minus bias, before scale
    this.f = [0, 0, 0];     // last bias corrected specific force (body)
    this._rows = this.tau ? [0, 1, 2, 3, 4, 5, 14, 15] : [0, 1, 2, 3, 4, 5];
    this._M = new Float64Array(this.n * this.n);
  }

  init({ q, v, sigma }) {
    const n = this.n;
    this.q = qnormalize(q);
    this.v = v.slice();
    this.vs = [v[0], v[1]];
    this.P.fill(0);
    const d = [
      sigma.roll, sigma.pitch, sigma.yaw,
      sigma.vel, sigma.vel, sigma.velZ ?? sigma.vel,
      sigma.gyroBias, sigma.gyroBias, sigma.gyroBias,
      sigma.accelBias, sigma.accelBias, sigma.accelBias,
      sigma.scale, sigma.alpha, sigma.vel, sigma.vel,
    ];
    for (let i = 0; i < n; i++) this.P[i * n + i] = d[i] * d[i];
  }

  get C() {
    return qToMat(this.q);
  }

  _row() {
    return new Float64Array(this.n);
  }

  // Strapdown propagation with one IMU sample (body frame, rad/s and m/s² specific force).
  propagate(dt, gyro, accel) {
    const n = this.n;
    const C = qToMat(this.q);
    const wzRaw = gyro[2] - this.bg[2];
    const w = [gyro[0] - this.bg[0], gyro[1] - this.bg[1], wzRaw * (1 + this.s)];
    const f = [accel[0] - this.ba[0], accel[1] - this.ba[1], accel[2] - this.ba[2]];
    const fn = m3v(C, f);
    const kf = this.tau ? 1 - Math.exp(-dt / this.tau) : 0;

    if (this.tau) {
      this.vs[0] += kf * (this.v[0] - this.vs[0]);
      this.vs[1] += kf * (this.v[1] - this.vs[1]);
    }
    this.q = qnormalize(qmul(this.q, qFromRotVec([w[0] * dt, w[1] * dt, w[2] * dt])));
    this.v = [this.v[0] + fn[0] * dt, this.v[1] + fn[1] * dt, this.v[2] + (fn[2] - G) * dt];
    this.w = w;
    this.wzRaw = wzRaw;
    this.f = f;

    // Sparse A = Phi - I as (row, col, value) entries.
    const A = [];
    for (let i = 0; i < 3; i++) {
      for (let k = 0; k < 3; k++) {
        A.push(i, I_BG + k, -C[i * 3 + k] * dt);       // attitude <- gyro bias
        A.push(3 + i, I_BA + k, -C[i * 3 + k] * dt);   // velocity <- accel bias
      }
      A.push(i, I_S, C[i * 3 + 2] * wzRaw * dt);      // attitude <- gyro z scale
    }
    // velocity <- attitude: -[fn x] dt
    A.push(3, 1, fn[2] * dt, 3, 2, -fn[1] * dt, 4, 0, -fn[2] * dt, 4, 2, fn[0] * dt, 5, 0, fn[1] * dt, 5, 1, -fn[0] * dt);
    if (this.tau) A.push(I_VS, I_V, kf, I_VS, I_VS, -kf, I_VS + 1, I_V + 1, kf, I_VS + 1, I_VS + 1, -kf);

    // P' = P + AP + PA^T + APA^T + Qd
    const P = this.P, M = this._M, rows = this._rows;
    for (const i of rows) M.fill(0, i * n, i * n + n);
    for (let e = 0; e < A.length; e += 3) {
      const i = A[e], k = A[e + 1], a = A[e + 2];
      for (let j = 0; j < n; j++) M[i * n + j] += a * P[k * n + j];
    }
    const MA = new Float64Array(n * n);
    for (let e = 0; e < A.length; e += 3) {
      const j = A[e], k = A[e + 1], a = A[e + 2];
      for (const i of rows) MA[i * n + j] += M[i * n + k] * a;
    }
    for (const i of rows) {
      for (let j = 0; j < n; j++) {
        const mij = M[i * n + j];
        P[i * n + j] += mij;
        P[j * n + i] += mij;
      }
      for (const j of rows) P[i * n + j] += MA[i * n + j];
    }

    const ns = this.noise;
    const qd = [
      ns.gyroDensity ** 2, ns.gyroDensity ** 2, ns.gyroDensity ** 2,
      ns.accelDensity ** 2, ns.accelDensity ** 2, ns.accelDensity ** 2,
      ns.gyroBiasWalk ** 2, ns.gyroBiasWalk ** 2, ns.gyroBiasWalk ** 2,
      ns.accelBiasWalk ** 2, ns.accelBiasWalk ** 2, ns.accelBiasWalk ** 2,
      ns.scaleWalk ** 2, ns.alphaWalk ** 2, 0, 0,
    ];
    for (let i = 0; i < n; i++) P[i * n + i] += qd[i] * dt;
  }

  // Generic update. H: array of m rows (length n), y: innovations, R: variances.
  // Returns { nis, accepted }.
  update(H, y, R, gate = Infinity) {
    const n = this.n, m = y.length, P = this.P;
    const PHt = new Float64Array(n * m);
    for (let i = 0; i < n; i++)
      for (let r = 0; r < m; r++) {
        let s = 0;
        const h = H[r];
        for (let j = 0; j < n; j++) if (h[j] !== 0) s += P[i * n + j] * h[j];
        PHt[i * m + r] = s;
      }
    const S = new Float64Array(m * m);
    for (let a = 0; a < m; a++)
      for (let b = 0; b < m; b++) {
        let s = 0;
        for (let j = 0; j < n; j++) if (H[a][j] !== 0) s += H[a][j] * PHt[j * m + b];
        S[a * m + b] = s + (a === b ? R[a] : 0);
      }
    const Si = invSmall(S, m);
    if (!Si) return { nis: Infinity, accepted: false };
    let nis = 0;
    for (let a = 0; a < m; a++) for (let b = 0; b < m; b++) nis += y[a] * Si[a * m + b] * y[b];
    if (!(nis <= gate)) return { nis, accepted: false };

    const K = new Float64Array(n * m);
    for (let i = 0; i < n; i++)
      for (let b = 0; b < m; b++) {
        let s = 0;
        for (let a = 0; a < m; a++) s += PHt[i * m + a] * Si[a * m + b];
        K[i * m + b] = s;
      }
    const dx = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let a = 0; a < m; a++) s += K[i * m + a] * y[a];
      dx[i] = s;
    }
    for (let i = 0; i < n; i++)
      for (let j = 0; j < n; j++) {
        let s = 0;
        for (let a = 0; a < m; a++) s += K[i * m + a] * PHt[j * m + a];
        P[i * n + j] -= s;
      }
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const s = 0.5 * (P[i * n + j] + P[j * n + i]);
        P[i * n + j] = s; P[j * n + i] = s;
      }
      if (P[i * n + i] < 1e-14) P[i * n + i] = 1e-14;
    }
    this._inject(dx);
    return { nis, accepted: true };
  }

  _inject(dx) {
    this.q = qnormalize(qmul(qFromRotVec([dx[0], dx[1], dx[2]]), this.q));
    for (let i = 0; i < 3; i++) {
      this.v[i] += dx[I_V + i];
      this.bg[i] = clamp(this.bg[i] + dx[I_BG + i], -0.1, 0.1);
      this.ba[i] = clamp(this.ba[i] + dx[I_BA + i], -1.5, 1.5);
    }
    this.s = clamp(this.s + dx[I_S], -0.1, 0.1);
    this.alpha = clamp(this.alpha + dx[I_AL], -0.3, 0.3);
    if (this.tau) { this.vs[0] += dx[I_VS]; this.vs[1] += dx[I_VS + 1]; }
  }

  // ---- measurements ----

  // GNSS horizontal velocity (ENU east, north), as reported (smoothed if tau > 0).
  updateGnssVelocity(vE, vN, sigma, gate) {
    const h0 = this._row(), h1 = this._row();
    const i = this.tau ? I_VS : I_V;
    h0[i] = 1; h1[i + 1] = 1;
    const cur = this.tau ? this.vs : this.v;
    return this.update([h0, h1], [vE - cur[0], vN - cur[1]], [sigma * sigma, sigma * sigma], gate);
  }

  // Current (not yet corrected) predicted GNSS velocity.
  predictedGnssVelocity() {
    return this.tau ? this.vs.slice() : [this.v[0], this.v[1]];
  }

  // Flat ground: vertical velocity ~ 0.
  updateVerticalVelocity(sigma) {
    const h = this._row();
    h[I_V + 2] = 1;
    return this.update([h], [-this.v[2]], [sigma * sigma]);
  }

  updateZeroVelocity(sigma) {
    const H = [0, 1, 2].map((k) => { const h = this._row(); h[I_V + k] = 1; return h; });
    return this.update(H, [-this.v[0], -this.v[1], -this.v[2]], [sigma * sigma, sigma * sigma, sigma * sigma]);
  }

  // Stationary: true angular rate is zero, so the (averaged) raw gyro reading is bias.
  updateZeroRate(gyroMean, sigma) {
    const H = [0, 1, 2].map((k) => { const h = this._row(); h[I_BG + k] = -1; return h; });
    const y = [0, 1, 2].map((k) => -(gyroMean[k] - this.bg[k]));
    return this.update(H, y, [sigma * sigma, sigma * sigma, sigma * sigma]);
  }

  // Non-holonomic constraint: lateral velocity at the rear axle is ~0 while the car grips.
  // p: phone position relative to rear axle centre, vehicle frame [x fwd, y left, z up] (m).
  updateNonHolonomic(p, sigma, gate) {
    const C = qToMat(this.q);
    const ca = Math.cos(this.alpha), sa = Math.sin(this.alpha);
    const vb = m3tv(C, this.v);
    const u = [ca * vb[0] - sa * vb[1], sa * vb[0] + ca * vb[1], vb[2]];
    const wv = [ca * this.w[0] - sa * this.w[1], sa * this.w[0] + ca * this.w[1], this.w[2]];
    // lateral velocity at rear axle = u_y - (w x p)_y = u_y - (w_z p_x - w_x p_z)
    const hVal = u[1] - (wv[2] * p[0] - wv[0] * p[2]);
    const Hv = m3v(C, [sa, ca, 0]);
    const Hth = cross(Hv, this.v);
    const h = this._row();
    h[0] = Hth[0]; h[1] = Hth[1]; h[2] = Hth[2];
    h[3] = Hv[0]; h[4] = Hv[1]; h[5] = Hv[2];
    h[I_BG + 2] = p[0] * (1 + this.s);
    h[I_S] = -p[0] * this.wzRaw;
    h[I_AL] = u[0];
    return this.update([h], [-hVal], [sigma * sigma], gate);
  }

  // ---- outputs ----

  // Velocity at a point (rel. rear axle) expressed in the vehicle frame, given phone position p.
  vehicleVelocityAt(point, p) {
    const C = qToMat(this.q);
    const ca = Math.cos(this.alpha), sa = Math.sin(this.alpha);
    const vb = m3tv(C, this.v);
    const u = [ca * vb[0] - sa * vb[1], sa * vb[0] + ca * vb[1], vb[2]];
    const wv = this.vehicleRate();
    const d = [point[0] - p[0], point[1] - p[1], (point[2] ?? p[2]) - p[2]];
    const wxd = cross(wv, d);
    return [u[0] + wxd[0], u[1] + wxd[1], u[2] + wxd[2]];
  }

  vehicleRate() {
    const ca = Math.cos(this.alpha), sa = Math.sin(this.alpha);
    return [ca * this.w[0] - sa * this.w[1], sa * this.w[0] + ca * this.w[1], this.w[2]];
  }

  // Vehicle-frame kinematic acceleration (gravity removed) at the phone.
  vehicleAccel() {
    const C = qToMat(this.q);
    const gb = m3tv(C, [0, 0, -G]);
    const a = [this.f[0] + gb[0], this.f[1] + gb[1], this.f[2] + gb[2]];
    const ca = Math.cos(this.alpha), sa = Math.sin(this.alpha);
    return [ca * a[0] - sa * a[1], sa * a[0] + ca * a[1], a[2]];
  }

  // Sideslip angle at a point and its 1-sigma from the covariance.
  sideslipAt(point, p) {
    const n = this.n;
    const u = this.vehicleVelocityAt(point, p);
    const sp2 = u[0] * u[0] + u[1] * u[1];
    const beta = Math.atan2(u[1], u[0]);
    if (sp2 < 1e-6) return { beta, sigma: Math.PI };
    const C = qToMat(this.q);
    const ca = Math.cos(this.alpha), sa = Math.sin(this.alpha);
    const Hx = m3v(C, [ca, -sa, 0]);
    const Hy = m3v(C, [sa, ca, 0]);
    const Tx = cross(Hx, this.v), Ty = cross(Hy, this.v);
    const J = this._row();
    const kx = -u[1] / sp2, ky = u[0] / sp2;
    for (let k = 0; k < 3; k++) {
      J[k] = kx * Tx[k] + ky * Ty[k];
      J[3 + k] = kx * Hx[k] + ky * Hy[k];
    }
    J[I_AL] = kx * -u[1] + ky * u[0];
    let s2 = 0;
    const P = this.P;
    for (let i = 0; i < n; i++) {
      if (J[i] === 0) continue;
      let r = 0;
      for (let j = 0; j < n; j++) if (J[j] !== 0) r += P[i * n + j] * J[j];
      s2 += J[i] * r;
    }
    return { beta, sigma: Math.sqrt(Math.max(s2, 0)) };
  }

  // Admit extra heading uncertainty (e.g. after a long slide the heading may have drifted).
  inflateHeading(sigma) {
    this.P[2 * this.n + 2] += sigma * sigma;
  }

  headingSigma() {
    return Math.sqrt(this.P[2 * this.n + 2]);
  }

  snapshot() {
    return {
      q: this.q.slice(), v: this.v.slice(), vs: this.vs.slice(), bg: this.bg.slice(), ba: this.ba.slice(),
      s: this.s, alpha: this.alpha, P: Float64Array.from(this.P),
      w: this.w.slice(), wzRaw: this.wzRaw, f: this.f.slice(),
    };
  }

  restore(sn) {
    this.q = sn.q.slice(); this.v = sn.v.slice(); this.vs = sn.vs.slice();
    this.bg = sn.bg.slice(); this.ba = sn.ba.slice();
    this.s = sn.s; this.alpha = sn.alpha; this.P.set(sn.P);
    this.w = sn.w.slice(); this.wzRaw = sn.wzRaw; this.f = sn.f.slice();
  }
}
