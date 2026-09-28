// Drift angle estimator: takes raw phone sensor samples, calibrates itself, and
// outputs the vehicle sideslip (drift) angle with a confidence estimate.
//
// Input units
//   addImu({ t, gyro, accel })   t: ms (monotonic clock shared with GNSS),
//                                gyro: rotationRate [alpha, beta, gamma] in rad/s (the axis
//                                order is detected, see GYRO_MAPS), accel: m/s² specific
//                                force (reads +g "up" at rest), phone axes.
//   addGnss({ t, lat, lon, speed, course })  t: ms arrival time on the same clock,
//                                speed: m/s, course: deg clockwise from north.
//                                speed/course may be null -> position differencing.
//
// Calibration flow (fast path, ~5 s of driving on a phone seen before):
//   standstill (2 s)  ->  first straight acceleration, e.g. the launch  ->  live.
// Properties of the phone + browser (gyro axis order, units, signs, GNSS latency and
// smoothing) are checked in the background during the first turns and returned in
// output.profile so the app can store them; with a stored profile and an unchanged
// mount the meter goes live as soon as the car is moving straight above 18 km/h.
//
// Sign convention: beta > 0 when the car moves to the left of where it points
// (ISO 8855). A car drifting through a left-hand corner reads negative.

import { DriftEKF } from './ekf.js';
import {
  MotionWindow, StandstillCalibrator, DriveCalibrator, MountAligner, GYRO_MAPS, mountRotation,
} from './calibration.js';
import { VelocityLagTracker } from './latency.js';
import { m3v, m3tv, qFromEuler, eulerFromMat, wrapPi, dot, normalize, cross, DEG } from './math.js';

export const PHASE = {
  WAIT_GNSS: 'waiting-gnss',
  STANDSTILL: 'standstill',
  ALIGN: 'align',
  RUNNING: 'running',
};

export const DEFAULT_CONFIG = {
  phonePos: [1.6, 0, 0.5],    // phone relative to rear axle centre (m): x fwd, y left, z up
  outputPoint: [0, 0, 0.5],   // point where the drift angle is reported: rear axle centre,
                              // where it is ~0 whenever the car grips (also in tight turns)
  minSpeed: 3,                // m/s; no angle below this
  minSlideSpeed: 5,           // m/s; a slide can only start above this (18 km/h)
  standstillAccelDrift: 0.3,  // m/s², max low-frequency accel change counted as "still" (+ vibration allowance)
  standstillSeconds: 2,
  device: null,               // stored device profile (see DEFAULT_DEVICE)
  mount: null,                // stored mount { upRaw, fwdRaw }
};

// Assumptions for a phone + browser not seen before; verified while driving.
export const DEFAULT_DEVICE = {
  gyroMap: 0, gyroGain: 1, accelSign: 1, latency: 0.2, smoothing: 0,
  sensorsVerified: false, accelVerified: false, latencyLearned: false,
};

const EARTH_R = 6371000;
const HISTORY_SPAN = 2.5; // s of IMU history kept for delayed GNSS updates
const STORED_MOUNT_MAX_TILT = 4 * DEG;

function lpf(prev, x, dt, tau) {
  return prev + (1 - Math.exp(-dt / tau)) * (x - prev);
}

export class DriftEstimator {
  constructor(config = {}) {
    this.cfg = { ...DEFAULT_CONFIG, ...config };
    this.device = { ...DEFAULT_DEVICE, ...(this.cfg.device || {}) };
    this.storedMount = this.cfg.mount || null;
    this.profileVersion = 0;
    this._reset(null);
  }

  // Start calibration over (e.g. the phone was moved in its mount).
  recalibrate(notice = null) {
    this.storedMount = null;
    this._reset(notice);
    this.profileVersion++;
    return this.out;
  }

  _reset(notice) {
    this.notice = notice;
    this._pendingReset = null;
    this.mountOff = 0;
    this.tiltTime = 0;
    this.phase = PHASE.WAIT_GNSS;
    this.win = new MotionWindow(1.0);
    this.standstill = new StandstillCalibrator(this.cfg.standstillSeconds);
    this.verifier = null;
    this.aligner = null;
    this.mountSource = null;
    this.smoothingSwitched = false;
    this.stand = null;
    this.mount = null;
    this.cal = null;
    this.ekf = null;
    this.ekfReady = false;
    this.mountSaved = false;
    this.hist = [];
    this.latency = null;
    this.lastImuT = null;
    this.lastGnss = null;
    this.posHist = [];        // recent positions for velocity by differencing
    this.origin = null;
    this.imuDt = null;
    this.gnssDt = null;
    this.gnssSigma = 0.3;
    this.gnssRejects = 0;
    this.gnssKind = null;
    this.stats = { gnssUsed: 0, gnssRejected: 0, gnssTooOld: 0, imuGaps: 0, restarts: 0 };
    // slide detector
    this.sliding = false;
    this.lastSlideT = -Infinity;
    this.aLatF = 0; this.rF = 0; this.rDotF = 0; this.bdotF = 0; this.kinTime = 0; this.bigBetaTime = 0;
    this.gripTime = 0; this.straightTime = 0;
    this.reacquireUntil = -Infinity;
    this.nhcRejectT = -Infinity;
    this.gnssBadT = -Infinity;
    this.lastPseudoT = -Infinity;
    this.gF = [0, 0, 0];
    this.gFa = [0, 0, 0];
    this.gFaSlow = [0, 0, 0];
    this.lastStillT = -Infinity;
    this.betaHist = [];
    this.kinSign = 0;  // accumulates lateral accel x yaw rate x speed (see _verify)
    this.out = this._output(0);
  }

  // ---------------------------------------------------------------- IMU

  addImu({ t, gyro, accel }) {
    t /= 1000;
    if (this.lastImuT !== null && t <= this.lastImuT) return this.out;
    const dt = this.lastImuT === null ? 0 : t - this.lastImuT;
    this.lastImuT = t;
    if (dt > 0) this.imuDt = this.imuDt === null ? dt : lpf(this.imuDt, dt, 1, 2);
    this.win.push(t, accel, gyro);
    if (this.verifier) this.verifier.addImu(t, gyro, accel);
    if (this.aligner) this.aligner.addImu(t, accel, gyro);

    switch (this.phase) {
      case PHASE.WAIT_GNSS:
        if (this.lastGnss && t - this.lastGnss.t < 3) this.phase = PHASE.STANDSTILL;
        break;
      case PHASE.STANDSTILL:
        if (this._rawStill(t)) {
          this.standstill.add(t, accel, gyro);
          if (this.standstill.done) this._afterStandstill();
        } else {
          this.standstill.reset();
        }
        break;
      case PHASE.ALIGN:
        break;
      case PHASE.RUNNING:
        this._runImu(t, dt, gyro, accel);
        break;
    }
    if (this._pendingReset) {
      const lastGnss = this.lastGnss;
      this.storedMount = null;
      this._reset(this._pendingReset);
      this.lastGnss = lastGnss;
      this.profileVersion++;
    }
    this.out = this._output(t);
    return this.out;
  }

  _afterStandstill() {
    this.stand = this.standstill.result();
    if (!this.device.sensorsVerified || !this.device.accelVerified) {
      this.verifier = new DriveCalibrator(this.stand, { assumedMap: this.device.gyroMap });
    }
    this.aligner = new MountAligner(this.stand, {
      latency: this.device.latency, gyroMap: this.device.gyroMap, gyroGain: this.device.gyroGain,
    });
    const sm = this.storedMount;
    if (sm && dot(normalize(sm.upRaw), this.stand.up) > Math.cos(STORED_MOUNT_MAX_TILT)) {
      // Same mount as last time (gravity points the same way in phone axes). The first
      // straight acceleration still double-checks the forward direction.
      this.mount = { upRaw: this.stand.up, fwdRaw: sm.fwdRaw };
      this.mountSource = 'stored';
      this._startRunning(2 * DEG);
    } else {
      this.phase = PHASE.ALIGN;
    }
  }

  _gnssSlow(t) {
    const g = this.lastGnss;
    return g !== null && t - g.t < 3 && g.speed < (g.kind === 'diff' ? 0.8 : 0.5);
  }

  _rawStill(t) {
    if (this.win.duration < 0.8 || !this._gnssSlow(t)) return false;
    const s = this.win.stats();
    return s.accelDrift < this.cfg.standstillAccelDrift + s.vibrationSpread && s.accelStd < 3;
  }

  _toBody(gyroRaw, accelRaw) {
    const c = this.cal;
    const g0 = GYRO_MAPS[c.gyroMap]([0, 1, 2].map((k) => gyroRaw[k] - c.gyroBiasRaw[k]));
    const g = [0, 1, 2].map((k) => c.gyroGain * g0[k]);
    const a = [0, 1, 2].map((k) => c.accelSign * accelRaw[k]);
    return { g: m3v(c.R, g), a: m3v(c.R, a) };
  }

  _runImu(t, dt, gyroRaw, accelRaw) {
    const { g, a } = this._toBody(gyroRaw, accelRaw);
    const ekf = this.ekf;
    if (dt > 0) {
      for (let k = 0; k < 3; k++) {
        this.gF[k] = lpf(this.gF[k], g[k], dt, 0.5);
        this.gFa[k] = lpf(this.gFa[k], a[k], dt, 0.3);
        this.gFaSlow[k] = lpf(this.gFaSlow[k], a[k], dt, 3);
      }
      const sp = this.lastGnss ? this.lastGnss.speed : 0;
      if (sp > 4 && dt < 0.2) this.kinSign += this.gFa[1] * this.gF[2] * sp * dt;
    }
    if (!this.ekfReady || dt <= 0) return;
    if (dt > 1) {
      // Sensor stream interrupted (page hidden, screen locked): restart the filter,
      // keep the calibration, re-initialise on the next good GNSS fix.
      this.stats.imuGaps++;
      this.ekfReady = false;
      this.hist = [];
      return;
    }
    if (dt > 0.2) { this.stats.imuGaps++; dt = 0.2; }

    const flags = this._decideFlags(t);
    const e = { t, dt, g, a, flags, snap: null };
    this._step(e, false);
    e.snap = ekf.snapshot();
    this.hist.push(e);
    if (this.hist.length > 50 && this.hist[0].t < t - HISTORY_SPAN - 0.5) {
      let k = 0;
      while (this.hist[k].t < t - HISTORY_SPAN) k++;
      this.hist.splice(0, k);
    }
    this._updateSlideDetector(t, dt);
    this._checkMount(t, dt, flags);
    // Slow manoeuvring (parking, reversing, tight turns at walking pace) leaves the heading
    // poorly constrained; take it again from GNSS when pulling away.
    if (this._speed() < 2.5) this.slowTime += dt;
    else if (this.slowTime > 1.5 && this._speed() > 4) { this.headingSuspect = true; this.slowTime = 0; }
    else if (this._speed() > 4) this.slowTime = 0;
    const an = m3v(ekf.C, ekf.f); // nav-frame specific force; horizontal part = acceleration
    this.latency.push(t, an[0], an[1]);
    const u = ekf.vehicleVelocityAt(this.cfg.phonePos, this.cfg.phonePos);
    this.betaHist.push([t, Math.atan2(u[1], u[0])]);
    if (this.betaHist[0][0] < t - 8) this.betaHist.splice(0, this.betaHist.findIndex((x) => x[0] >= t - 6));
  }

  // Filter's drift angle at the phone at time tq (nearest stored sample), or null.
  _betaAt(tq) {
    const B = this.betaHist;
    if (!B.length || tq < B[0][0] || tq > B[B.length - 1][0] + 0.1) return null;
    let lo = 0, hi = B.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (B[mid][0] <= tq) lo = mid; else hi = mid;
    }
    return B[lo][1];
  }

  // Detect the phone being moved in its mount: at standstill gravity no longer points
  // along the calibrated "up", or the filter's body tilt stays implausibly large.
  _checkMount(t, dt, flags) {
    if (flags.pseudo && flags.stationary) {
      const m = this.win.stats().meanAccel;
      const upP = this.cal.R.slice(6, 9);
      const s = this.cal.accelSign;
      const n = Math.hypot(m[0], m[1], m[2]);
      const c = (s * (m[0] * upP[0] + m[1] * upP[1] + m[2] * upP[2])) / n;
      this.mountOff = c < Math.cos(8 * DEG) ? this.mountOff + 1 : 0;
      if (this.mountOff >= 20) this._pendingReset = 'The phone moved in its mount. Recalibrating…';
    }
    // (With unverified gyro axes a wrong axis order also tilts the filter; the background
    // sensor check handles that case, so the tilt test waits for it.)
    if (!this.device.sensorsVerified) return;
    const { roll, pitch } = eulerFromMat(this.ekf.C);
    this.tiltTime = Math.abs(roll) > 20 * DEG || Math.abs(pitch) > 20 * DEG ? this.tiltTime + dt : 0;
    if (this.tiltTime > 2) this._pendingReset = 'The phone moved in its mount. Recalibrating…';
  }

  _speed() {
    const v = this.ekf.v;
    return Math.hypot(v[0], v[1]);
  }

  _decideFlags(t) {
    if (t - this.lastPseudoT < 0.05) return { pseudo: false };
    this.lastPseudoT = t;
    const sp = this._speed();
    // Standstill also needs quiet gyro and no horizontal acceleration: a gentle pull-away
    // can look "still" to the accelerometer-variance test for a second or two, and
    // clamping the speed to zero then would leave the filter behind.
    const ez = this.ekfReady ? this.ekf.bg : [0, 0, 0];
    const quiet = Math.hypot(this.gF[0] - ez[0], this.gF[1] - ez[1], this.gF[2] - ez[2]) < 0.03 &&
      Math.hypot(this.gFa[0] - this.gFaSlow[0], this.gFa[1] - this.gFaSlow[1]) < 0.25;
    const stationary = sp < 0.5 && quiet && this._rawStill(t);
    if (stationary) this.lastStillT = t;
    return {
      pseudo: true,
      stationary,
      gyroMean: stationary ? this.gF.slice() : null,
      nhc: !stationary && !this.sliding && sp > 2,
      speed: sp,
    };
  }

  // One IMU step plus the pseudo-measurements decided for it. Also used for replay.
  _step(e, replay) {
    const ekf = this.ekf;
    ekf.propagate(e.dt, e.g, e.a);
    const f = e.flags;
    if (!f.pseudo) return;
    if (f.stationary) {
      ekf.updateZeroVelocity(0.03);
      ekf.updateZeroRate(f.gyroMean, 0.003);
      return;
    }
    ekf.updateVerticalVelocity(0.3);
    if (f.nhc) {
      // grip slip allowance, plus the unknown phone position (±0.5 m) times yaw rate
      const sigma = Math.hypot(Math.max(0.1, f.speed * Math.tan(1.5 * DEG)), 0.5 * Math.abs(ekf.w[2]));
      const r = ekf.updateNonHolonomic(this.cfg.phonePos, sigma, 9);
      if (!replay && !r.accepted) this.nhcRejectT = e.t;
    }
  }

  _updateSlideDetector(t, dt) {
    const ekf = this.ekf, p = this.cfg.phonePos;
    const aV = ekf.vehicleAccel();
    const wv = ekf.vehicleRate();
    const u = ekf.vehicleVelocityAt(p, p);
    const sp = Math.hypot(u[0], u[1]);
    const rPrev = this.rF;
    this.aLatF = lpf(this.aLatF, aV[1], dt, 0.12);
    this.rF = lpf(this.rF, wv[2], dt, 0.12);
    this.rDotF = lpf(this.rDotF, (this.rF - rPrev) / dt, dt, 0.1);
    const bdot = sp > 3 ? (this.aLatF - this.rF * u[0] - this.rDotF * p[0]) / sp : 0;
    this.bdotF = lpf(this.bdotF, bdot, dt, 0.1);
    const bRear = Math.abs(ekf.sideslipAt([0, 0, p[2]], p).beta);

    // Drift entries change the angle at 30-60°/s; phone mounts vibrate, so the kinematic
    // indicator must exceed 15°/s for 0.15 s to count.
    const kinRaw = Math.abs(this.bdotF) > 15 * DEG || Math.abs(this.aLatF) > 6;
    this.kinTime = kinRaw ? this.kinTime + dt : 0;
    const kin = this.kinTime >= 0.15;
    const nhcRej = t - this.nhcRejectT < 0.3;
    const gnssBad = t - this.gnssBadT < 1.5;
    if (sp < this.cfg.minSpeed || (!this.sliding && sp < this.cfg.minSlideSpeed)) {
      // (at walking pace the kinematic slip indicators are mostly noise)
      this.sliding = false;
    } else if (!this.sliding) {
      const reacquiring = t < this.reacquireUntil;
      // the filter's own angle has ~2° of noise in normal driving on a real dash mount
      this.bigBetaTime = bRear > 6 * DEG ? this.bigBetaTime + dt : 0;
      if (kin || (!reacquiring && (nhcRej || gnssBad || this.bigBetaTime > 0.3))) {
        this.sliding = true;
        this.gripTime = 0; this.straightTime = 0;
      }
    } else {
      const calm = !kin && !nhcRej;
      this.gripTime = calm ? this.gripTime + dt : 0;
      this.straightTime = calm && Math.abs(this.rF) < 4 * DEG ? this.straightTime + dt : 0;
      if (this.gripTime > 1.0 && bRear < 4 * DEG) {
        this.sliding = false;
      } else if (this.straightTime > 3) {
        // Driving straight and calm for seconds while the filter still reports slip:
        // the heading has drifted during the slide. Re-acquire it from the grip constraint.
        this.sliding = false;
        this.reacquireUntil = t + 3;
        ekf.inflateHeading(Math.max(bRear, 2 * DEG));
      }
    }
    if (this.sliding) this.lastSlideT = t;
  }

  // ---------------------------------------------------------------- GNSS

  addGnss(fix) {
    const t = fix.t / 1000;
    const pre = this._preprocess(fix, t);
    if (!pre) return this.out;
    if (this.lastGnss) this.gnssDt = this.gnssDt === null ? t - this.lastGnss.t : lpf(this.gnssDt, t - this.lastGnss.t, 1, 5);
    const prev = this.lastGnss;
    this.lastGnss = { t, speed: pre.speed, chi: pre.chi, kind: pre.kind, vE: pre.vE, vN: pre.vN };
    this.gnssKind = pre.kind;
    const calFix = { t: pre.tMeas, speed: pre.speed, chi: pre.chi };
    if (this.verifier) this.verifier.addGnss(calFix);
    if (this.aligner) this.aligner.addGnss(calFix);

    switch (this.phase) {
      case PHASE.WAIT_GNSS:
        this.phase = PHASE.STANDSTILL;
        break;
      case PHASE.ALIGN:
        this._alignGnss();
        break;
      case PHASE.RUNNING:
        this._checkAligner();
        this._runGnss(pre, t, prev);
        break;
    }
    if (this.verifier && this.phase !== PHASE.STANDSTILL) this._verify();
    this.out = this._output(this.lastImuT ?? t);
    return this.out;
  }

  // Waiting for the first straight acceleration (or, failing that, the slow turn-based
  // calibration) to learn which way the phone faces.
  _alignGnss() {
    const r = this.aligner.result;
    if (r) {
      this.mount = { upRaw: this.stand.up, fwdRaw: r.fwdRaw };
      this.mountSource = 'launch';
      this.aligner = null;
      this._startRunning(Math.max(r.sigma, 2.5 * DEG));
    }
  }

  // With a stored mount, the first straight acceleration confirms the forward direction.
  _checkAligner() {
    if (!this.aligner) return;
    const r = this.aligner.result;
    if (!r) return;
    this.aligner = null;
    const up = this.stand.up;
    const a = normalize(this.mount.fwdRaw), b = r.fwdRaw;
    const angle = Math.atan2(dot(cross(a, b), up), dot(a, b));
    if (Math.abs(angle) > 6 * DEG) {
      this.mount = { upRaw: up, fwdRaw: r.fwdRaw };
      this.mountSource = 'launch';
      this.notice = 'Phone direction changed since last time. Updated.';
      this._startRunning(Math.max(r.sigma, 2.5 * DEG));
      this.profileVersion++;
    }
  }

  // Background check of the phone + browser assumptions.
  _verify() {
    const res = this.verifier.solve();
    if (!res) return;
    const d = this.device;
    let restart = false;
    if (res.sensors && !d.sensorsVerified) {
      const s = res.sensors;
      let gain = Math.sign(s.kRaw * d.accelSign) * s.gyroUnit;
      // Yaw opposite to the GNSS turning: either the gyro sign or the accelerometer sign
      // (which flips "up") is inverted. In the current body frame an inverted gyro makes
      // lateral accel and yaw rate x speed disagree in sign; an inverted accelerometer
      // mirrors both, so they still agree.
      if (gain < 0 && s.gyroMap === d.gyroMap && this.kinSign > 0) {
        d.accelSign = -d.accelSign;
        gain = -gain;
      }
      if (s.gyroMap !== d.gyroMap || gain !== d.gyroGain || (this.cal && this.cal.accelSign !== d.accelSign)) {
        restart = true;
        this.notice = 'Sensor axes corrected for this phone.';
      }
      // (the quick fit's delay is not used: it is biased by drifting, see _runGnss)
      Object.assign(d, { gyroMap: s.gyroMap, gyroGain: gain, sensorsVerified: true });
      this.profileVersion++;
    }
    if (res.full && !d.accelVerified) {
      const f = res.full;
      if (f.accelSign !== d.accelSign || f.gyroGain !== d.gyroGain || f.gyroMap !== d.gyroMap) {
        restart = true;
        this.notice = 'Sensor axes corrected for this phone.';
      }
      Object.assign(d, { gyroMap: f.gyroMap, gyroGain: f.gyroGain, accelSign: f.accelSign, accelVerified: true });
      this.verifier = null;
      this.profileVersion++;
      if (this.phase === PHASE.ALIGN) {
        // no straight acceleration seen yet: use the turn-based mount estimate
        this.mount = { upRaw: this.stand.up, fwdRaw: f.fwdRaw };
        this.mountSource = 'turns';
        this.aligner = null;
        this._startRunning(f.alphaSigma);
        return;
      }
    }
    if (restart && this.phase === PHASE.RUNNING) {
      this.latency = null; // its acceleration history came from the wrong sensor model
      this._startRunning(this.cal.alphaSigma);
    }
  }

  _preprocess(fix, t) {
    const hasPos = Number.isFinite(fix.lat) && Number.isFinite(fix.lon);
    let enu = null;
    if (hasPos) {
      if (!this.origin) this.origin = { lat: fix.lat, lon: fix.lon, c: Math.cos(fix.lat * DEG) };
      enu = [
        (fix.lon - this.origin.lon) * DEG * EARTH_R * this.origin.c,
        (fix.lat - this.origin.lat) * DEG * EARTH_R,
      ];
    }
    if (enu) {
      this.posHist.push({ t, enu });
      while (this.posHist.length && this.posHist[0].t < t - 5) this.posHist.shift();
    }

    const speed = fix.speed;
    if (Number.isFinite(speed) && speed >= 0) {
      if (Number.isFinite(fix.course) && speed > 0.3) {
        const c = fix.course * DEG;
        const vE = speed * Math.sin(c), vN = speed * Math.cos(c);
        return { kind: 'doppler', tMeas: t, vE, vN, speed, chi: Math.atan2(vN, vE) };
      }
      if (speed <= 0.5) return { kind: 'doppler', tMeas: t, vE: 0, vN: 0, speed, chi: null };
    }
    // Fallback: velocity as the least-squares slope of the last ~2 s of positions (short
    // differences turn metre-level position noise into metres per second), valid at the
    // mean time of the window.
    if (!enu) return null;
    const win = this.posHist.filter((p) => p.t >= t - 2.05);
    if (win.length < 2 || t - win[0].t < 0.9) return null;
    const tm = win.reduce((s, p) => s + p.t, 0) / win.length;
    const mE = win.reduce((s, p) => s + p.enu[0], 0) / win.length;
    const mN = win.reduce((s, p) => s + p.enu[1], 0) / win.length;
    let stt = 0, ste = 0, stn = 0;
    for (const p of win) {
      const d = p.t - tm;
      stt += d * d; ste += d * (p.enu[0] - mE); stn += d * (p.enu[1] - mN);
    }
    const vE = ste / stt, vN = stn / stt;
    const sp = Math.hypot(vE, vN);
    return { kind: 'diff', tMeas: tm, vE, vN, speed: sp, chi: sp > 1 ? Math.atan2(vN, vE) : null };
  }

  // (Re)build the body transform and filter from the current mount and device profile.
  // The filter itself initialises on the next straight GNSS fix above 18 km/h.
  _startRunning(alphaSigma) {
    const d = this.device;
    this.cal = {
      gyroMap: d.gyroMap,
      gyroGain: d.gyroGain,
      gyroBiasRaw: this.stand.gyroBias,
      accelSign: d.accelSign,
      R: mountRotation(this.mount.upRaw, this.mount.fwdRaw, d.accelSign),
      alphaSigma,
      smoothing: d.smoothing,
      latency: d.latency,
    };
    if (this.phase === PHASE.RUNNING) this.stats.restarts++;
    this.phase = PHASE.RUNNING;
    if (!this.latency) this.latency = new VelocityLagTracker({ initial: d.latency, initialSmoothing: d.smoothing });
    const st = this.stand;
    const rate = st.rate || 60;
    this.ekf = new DriftEKF({
      gyroDensity: Math.max(0.003, 2 * Math.abs(d.gyroGain) * st.gyroStd / Math.sqrt(rate)),
      accelDensity: Math.max(0.12, 2 * st.accelStd / Math.sqrt(rate)),
    }, { smoothing: d.smoothing });
    this.ekfReady = false;
    this.hist = [];
    this.betaHist = [];
    this.kinSign = 0;
    this.nisF = 2;
    this.lastConsistentT = null;
    this.lastGnssUpdateT = -Infinity;
    this.slowTime = 0;
    this.headingSuspect = false;
    this.sliding = false;
    this.mountSaved = false;
    this.profileVersion++;
  }

  _runGnss(pre, t, prev) {
    const ekf = this.ekf;
    if (!this.ekfReady) {
      if (pre.speed > 5 && pre.chi !== null && Math.abs(this.gF[2]) < 0.1) {
        // The fix is L seconds old: extrapolate the speed with the current forward
        // acceleration (gravity removed with the standstill level), e.g. during a launch.
        const L = this.latency.lagFor(this.cal.smoothing);
        const along = this.gFa[0];
        const sp = pre.speed + along * L;
        const c = Math.cos(pre.chi), s = Math.sin(pre.chi);
        ekf.init({
          q: qFromEuler(0, 0, pre.chi),
          v: [sp * c, sp * s, 0],
          sigma: {
            roll: 2 * DEG, pitch: 2 * DEG, yaw: 5 * DEG, vel: 0.5 + Math.abs(along) * 0.3, velZ: 0.2,
            gyroBias: 0.3 * DEG, accelBias: 0.2, scale: 0.01, alpha: this.cal.alphaSigma,
          },
        });
        this.ekfReady = true;
        this.hist = [];
        this.lastPseudoT = -Infinity;
      }
      return;
    }

    if (this.headingSuspect && pre.speed > 5 && pre.chi !== null && Math.abs(this.gF[2]) < 0.1) {
      const L = this.latency.lagFor(this.cal.smoothing);
      const sp = pre.speed + this.gFa[0] * L;
      ekf.resetHeading(pre.chi, sp * Math.cos(pre.chi), sp * Math.sin(pre.chi), 5 * DEG, 0.5 + Math.abs(this.gFa[0]) * 0.3);
      this.headingSuspect = false;
      this.hist = [];
      return;
    }

    // GNSS delay and smoothing from velocity changes between consecutive Doppler fixes.
    if (pre.kind === 'doppler' && prev && prev.kind === 'doppler' && t - prev.t < 2.5) {
      this.latency.addFix(prev.t, t, pre.vE - prev.vE, pre.vN - prev.vN);
      const lt = this.latency;
      if (lt.converged) {
        // A clearly better smoothing model is worth one filter restart per session (at a
        // calm moment); otherwise keep the current model and its matching delay.
        const imp = lt.smoothingImprovement(this.cal.smoothing);
        const switchModel = imp.ratio < 0.7 && Math.abs(imp.tau - this.cal.smoothing) > 0.25;
        const smoothing = switchModel ? imp.tau : this.cal.smoothing;
        const L = lt.lagFor(smoothing);
        if (Math.abs(L - this.device.latency) > 0.02 || smoothing !== this.device.smoothing || !this.device.latencyLearned) {
          Object.assign(this.device, { latency: L, smoothing, latencyLearned: true });
          this.profileVersion++;
        }
        if (switchModel && !this.smoothingSwitched && !this.sliding) {
          this.smoothingSwitched = true;
          this._startRunning(this.cal.alphaSigma);
          return;
        }
      }
    }

    // High-rate GNSS output is usually smoothed, so consecutive fixes are far from
    // independent; feeding all of them would make the filter overconfident.
    if (t - this.lastGnssUpdateT < 0.4) return;
    this.lastGnssUpdateT = t;

    // Until the GNSS delay is known, a velocity fix is uncertain by (acceleration x delay error).
    const aV = ekf.vehicleAccel();
    const sigmaL = this.latency.converged ? 0.04 : this.device.latencyLearned ? 0.08 : 0.3;
    const sigmaLag = Math.hypot(aV[0], aV[1]) * sigmaL;

    const tm = pre.tMeas - this.latency.lagFor(this.cal.smoothing);
    const H = this.hist;
    if (H.length === 0 || tm < H[0].t) { this.stats.gnssTooOld++; return; }
    let j = H.length - 1;
    while (j > 0 && H[j].t > tm) j--;
    const replay = j < H.length - 1;
    const saved = replay ? ekf.snapshot() : null;
    if (replay) ekf.restore(H[j].snap);
    // Just after a standstill the filter's velocity is tightly pinned at zero; let the
    // first moving fixes through instead of rejecting them as outliers.
    if (t - this.lastStillT < 4 && pre.speed > 1) ekf.inflateVelocity(1.5);

    const sigma = Math.hypot(pre.kind === 'diff' ? 2 * this.gnssSigma + 0.3 : this.gnssSigma, sigmaLag);
    const vp = ekf.predictedGnssVelocity();
    const yE = pre.vE - vp[0], yN = pre.vN - vp[1];
    let res = ekf.updateGnssVelocity(pre.vE, pre.vN, sigma, 25);
    // consistency monitor: normalized innovations average 2 when the filter agrees with GNSS
    this.nisF = 0.8 * this.nisF + 0.2 * Math.min(res.nis, 50);
    if (!res.accepted) {
      this.gnssRejects++;
      this.stats.gnssRejected++;
      if (this.gnssRejects >= 3) {
        res = ekf.updateGnssVelocity(pre.vE, pre.vN, 3 * sigma, Infinity);
        this.gnssRejects = 0;
      } else {
        if (replay) ekf.restore(saved);
        return;
      }
    } else {
      this.gnssRejects = 0;
      this.stats.gnssUsed++;
      if (pre.kind === 'doppler' && sigmaLag < 0.1) {
        const e2 = 0.5 * (yE * yE + yN * yN);
        this.gnssSigma = Math.min(1.2, Math.max(0.15, Math.sqrt(lpf(this.gnssSigma ** 2, e2, 1, 30))));
      }
      if (!this.sliding && res.nis > 12) this.gnssBadT = t;
    }
    H[j].snap = ekf.snapshot();
    if (replay) {
      for (let i = j + 1; i < H.length; i++) {
        this._step(H[i], true);
        H[i].snap = ekf.snapshot();
      }
    }
    if (this._recover(t)) return;
    this._maybeSaveMount();
  }

  // Recovery from a wrong forward direction. A moderate error shows up as a large
  // residual mount yaw in the filter: fold it in and restart. A gross one makes the
  // filter disagree with GNSS for a long time: find the direction again.
  _recover(t) {
    const ekf = this.ekf;
    if (this.lastConsistentT === null || this.nisF < 6 || this.sliding) this.lastConsistentT = t;
    const sa = Math.sqrt(ekf.P[13 * ekf.n + 13]);
    if (Math.abs(ekf.alpha) > 12 * DEG && sa < 4 * DEG && !this.sliding) {
      this.mount = { upRaw: this.mount.upRaw, fwdRaw: this._fwdWithAlpha() };
      this._startRunning(3 * DEG);
      return true;
    }
    if (this.device.sensorsVerified && t - this.lastConsistentT > 15) {
      this.notice = 'Re-checking which way the phone faces…';
      this.aligner = new MountAligner(this.stand, {
        latency: this.device.latency, gyroMap: this.device.gyroMap, gyroGain: this.device.gyroGain,
      });
      this.mount = null;
      this.mountSource = null;
      this.ekfReady = false;
      this.phase = PHASE.ALIGN;
      this.profileVersion++;
      return true;
    }
    return false;
  }

  // Current forward direction in raw accelerometer axes, including the filter's
  // residual mount yaw.
  _fwdWithAlpha() {
    const a = this.ekf.alpha;
    // vehicle x in body axes is Rz(-alpha) e_x; body -> phone is R^T; raw = accelSign * phone
    const xp = m3tv(this.cal.R, [Math.cos(a), -Math.sin(a), 0]);
    return xp.map((v) => v * this.cal.accelSign);
  }

  // Once the filter has pinned down the residual mount yaw, fold it into the stored mount.
  _maybeSaveMount() {
    if (this.mountSaved || !this.ekfReady) return;
    const sa = Math.sqrt(this.ekf.P[13 * this.ekf.n + 13]);
    if (sa > 1 * DEG) return;
    this.mount = { upRaw: this.mount.upRaw, fwdRaw: this._fwdWithAlpha() };
    this.mountSaved = true;
    this.profileVersion++;
  }

  // ---------------------------------------------------------------- output

  _output(t) {
    const o = {
      t,
      phase: this.phase,
      message: '',
      notice: this.phase === PHASE.RUNNING && this.ekfReady ? null : this.notice,
      calibrationHint: this.verifier ? this.verifier.status.reason : null,
      sensorCheck: this.device.sensorsVerified ? 'ok' : 'pending',
      progress: 0,
      valid: false,
      beta: null,
      betaSigma: null,
      quality: 'none',
      speed: this.lastGnss ? this.lastGnss.speed : null,
      yawRate: null,
      sliding: false,
      imuRate: this.imuDt ? 1 / this.imuDt : null,
      gnssRate: this.gnssDt ? 1 / this.gnssDt : null,
      gnssKind: this.gnssKind,
      latency: this.latency ? this.latency.L : null,
      latencyConverged: this.latency ? this.latency.converged : false,
      calibration: this.cal
        ? {
          gyroMap: this.cal.gyroMap, accelSign: this.cal.accelSign, gyroGain: this.cal.gyroGain,
          alphaSigma: this.cal.alphaSigma / DEG, smoothing: this.cal.smoothing, mount: this.mountSource,
        }
        : null,
      profile: {
        version: this.profileVersion,
        device: { ...this.device },
        mount: this.mount ? { upRaw: this.mount.upRaw.slice(), fwdRaw: this.mount.fwdRaw.slice() } : this.storedMount,
      },
    };
    switch (this.phase) {
      case PHASE.WAIT_GNSS:
        o.message = 'Waiting for GPS…';
        break;
      case PHASE.STANDSTILL:
        o.message = 'Keep the car still…';
        o.progress = Math.min(1, this.standstill.duration / this.cfg.standstillSeconds);
        break;
      case PHASE.ALIGN:
        o.message = 'Drive off straight and accelerate to about 20 km/h';
        o.progress = this.aligner ? this.aligner.progress : 0;
        break;
      case PHASE.RUNNING:
        if (!this.ekfReady) {
          o.message = 'Drive straight above 20 km/h to start';
          break;
        }
        {
          const p = this.cfg.phonePos;
          const u = this.ekf.vehicleVelocityAt(this.cfg.outputPoint, p);
          const sp = Math.hypot(u[0], u[1]);
          const { beta, sigma } = this.ekf.sideslipAt(this.cfg.outputPoint, p);
          o.speed = sp;
          o.yawRate = this.ekf.vehicleRate()[2] / DEG;
          o.sliding = this.sliding;
          o.valid = sp >= this.cfg.minSpeed;
          if (o.valid) {
            o.beta = beta / DEG;
            o.betaSigma = sigma / DEG;
            o.quality = sigma < 2 * DEG ? 'good' : sigma < 4.5 * DEG ? 'fair' : 'poor';
            // The covariance cannot know about a wrong sensor model; persistent disagreement
            // with GNSS can.
            if (this.nisF > 8 || this.headingSuspect) o.quality = 'poor';
            else if (!this.device.sensorsVerified && o.quality === 'good') o.quality = 'fair';
          }
        }
        break;
    }
    return o;
  }

  // Internal state for diagnostics.
  debugState() {
    if (!this.ekfReady) return null;
    const e = this.ekf;
    return {
      gyroBias: e.bg.map((x) => x / DEG),
      accelBias: e.ba.slice(),
      scale: e.s,
      alpha: e.alpha / DEG,
      headingSigma: e.headingSigma() / DEG,
      gnssSigma: this.gnssSigma,
      device: { ...this.device },
      mountSource: this.mountSource,
      stats: { ...this.stats },
    };
  }
}
