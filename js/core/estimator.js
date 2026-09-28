// Drift angle estimator: takes raw phone sensor samples, calibrates itself, and
// outputs the vehicle sideslip (drift) angle with a confidence estimate.
//
// Input units
//   addImu({ t, gyro, accel })   t: ms (monotonic clock shared with GNSS),
//                                gyro: rad/s phone axes, accel: m/s² specific force
//                                (reads +g "up" at rest), phone axes.
//   addGnss({ t, lat, lon, speed, course })  t: ms arrival time on the same clock,
//                                speed: m/s, course: deg clockwise from north.
//                                speed/course may be null -> position differencing.
//
// Sign convention: beta > 0 when the car moves to the left of where it points
// (ISO 8855). A car drifting through a left-hand corner reads negative.

import { DriftEKF } from './ekf.js';
import { MotionWindow, StandstillCalibrator, DriveCalibrator } from './calibration.js';
import { LatencyTracker, IntegralHistory } from './latency.js';
import { m3v, qFromEuler, eulerFromMat, wrapPi, DEG } from './math.js';

export const PHASE = {
  WAIT_GNSS: 'waiting-gnss',
  STANDSTILL: 'standstill',
  DRIVE_CAL: 'drive-calibration',
  RUNNING: 'running',
};

export const DEFAULT_CONFIG = {
  phonePos: [1.6, 0, 0.5],    // phone relative to rear axle centre (m): x fwd, y left, z up
  outputPoint: [1.3, 0, 0.5], // point where the drift angle is reported (≈ mid-wheelbase)
  minSpeed: 3,                // m/s; no angle below this
  standstillAccelDrift: 0.3,  // m/s², max low-frequency accel change counted as "still" (+ vibration allowance)
  standstillSeconds: 3,
};

const EARTH_R = 6371000;
const HISTORY_SPAN = 2.5; // s of IMU history kept for delayed GNSS updates

function lpf(prev, x, dt, tau) {
  return prev + (1 - Math.exp(-dt / tau)) * (x - prev);
}

export class DriftEstimator {
  constructor(config = {}) {
    this.cfg = { ...DEFAULT_CONFIG, ...config };
    this._reset(null);
  }

  // Start calibration over (e.g. the phone was moved in its mount).
  recalibrate(notice = null) {
    this._reset(notice);
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
    this.drive = null;
    this.stand = null;
    this.cal = null;
    this.ekf = null;
    this.ekfReady = false;
    this.hist = [];
    this.yawHist = new IntegralHistory(HISTORY_SPAN + 1);
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
    this.stats = { gnssUsed: 0, gnssRejected: 0, gnssTooOld: 0, imuGaps: 0 };
    // slide detector
    this.sliding = false;
    this.lastSlideT = -Infinity;
    this.aLatF = 0; this.rF = 0; this.rDotF = 0; this.bdotF = 0;
    this.gripTime = 0; this.straightTime = 0;
    this.reacquireUntil = -Infinity;
    this.nhcRejectT = -Infinity;
    this.gnssBadT = -Infinity;
    this.lastPseudoT = -Infinity;
    this.gF = [0, 0, 0];
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

    switch (this.phase) {
      case PHASE.WAIT_GNSS:
        if (this.lastGnss && t - this.lastGnss.t < 3) this.phase = PHASE.STANDSTILL;
        break;
      case PHASE.STANDSTILL:
        if (this._rawStill(t)) {
          this.standstill.add(t, accel, gyro);
          if (this.standstill.done) {
            this.stand = this.standstill.result();
            this.drive = new DriveCalibrator(this.stand);
            this.phase = PHASE.DRIVE_CAL;
          }
        } else {
          this.standstill.reset();
        }
        break;
      case PHASE.DRIVE_CAL:
        this.drive.addImu(t, gyro, accel);
        break;
      case PHASE.RUNNING:
        this._runImu(t, dt, gyro, accel);
        break;
    }
    if (this._pendingReset) {
      const lastGnss = this.lastGnss;
      this._reset(this._pendingReset);
      this.lastGnss = lastGnss;
    }
    this.out = this._output(t);
    return this.out;
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
    const g = [0, 1, 2].map((k) => c.gyroGain * (gyroRaw[k] - c.gyroBiasRaw[k]));
    const a = [0, 1, 2].map((k) => c.accelSign * accelRaw[k]);
    return { g: m3v(c.R, g), a: m3v(c.R, a) };
  }

  _runImu(t, dt, gyroRaw, accelRaw) {
    const { g, a } = this._toBody(gyroRaw, accelRaw);
    const ekf = this.ekf;
    const rz = this.ekfReady ? (g[2] - ekf.bg[2]) * (1 + ekf.s) : g[2];
    this.yawHist.push(t, rz);
    if (dt > 0) for (let k = 0; k < 3; k++) this.gF[k] = lpf(this.gF[k], g[k], dt, 0.5);
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
    const stationary = sp < 0.5 && this._rawStill(t);
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
      const sigma = Math.max(0.1, f.speed * Math.tan(1.5 * DEG));
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

    const kin = Math.abs(this.bdotF) > 10 * DEG || Math.abs(this.aLatF) > 5;
    const nhcRej = t - this.nhcRejectT < 0.3;
    const gnssBad = t - this.gnssBadT < 1.5;
    if (sp < this.cfg.minSpeed) {
      this.sliding = false;
    } else if (!this.sliding) {
      const reacquiring = t < this.reacquireUntil;
      if (kin || (!reacquiring && (nhcRej || gnssBad || bRear > 4 * DEG))) {
        this.sliding = true;
        this.gripTime = 0; this.straightTime = 0;
      }
    } else {
      const calm = !kin && !nhcRej;
      this.gripTime = calm ? this.gripTime + dt : 0;
      this.straightTime = calm && Math.abs(this.rF) < 4 * DEG ? this.straightTime + dt : 0;
      if (this.gripTime > 1.0 && bRear < 3 * DEG) {
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
    this.lastGnss = { t, speed: pre.speed, chi: pre.chi, kind: pre.kind };
    this.gnssKind = pre.kind;

    switch (this.phase) {
      case PHASE.WAIT_GNSS:
        this.phase = PHASE.STANDSTILL;
        break;
      case PHASE.DRIVE_CAL: {
        this.drive.addGnss({ t: pre.tMeas, speed: pre.speed, chi: pre.chi });
        const cal = this.drive.solve();
        if (cal) this._startRunning(cal);
        break;
      }
      case PHASE.RUNNING:
        this._runGnss(pre, t, prev);
        break;
    }
    this.out = this._output(this.lastImuT ?? t);
    return this.out;
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

  _startRunning(cal) {
    this.cal = cal;
    this.phase = PHASE.RUNNING;
    this.latency = new LatencyTracker({ initial: cal.latency });
    this.yawHist = new IntegralHistory(HISTORY_SPAN + 1, cal.smoothing);
    const st = this.stand;
    const rate = st.rate || 60;
    this.ekf = new DriftEKF({
      gyroDensity: Math.max(0.003, 2 * Math.abs(cal.gyroGain) * st.gyroStd / Math.sqrt(rate)),
      accelDensity: Math.max(0.12, 2 * st.accelStd / Math.sqrt(rate)),
    }, { smoothing: cal.smoothing });
    this.ekfReady = false;
  }

  _runGnss(pre, t, prev) {
    const ekf = this.ekf;
    if (!this.ekfReady) {
      if (pre.speed > 5 && pre.chi !== null && Math.abs(this.gF[2]) < 0.1) {
        ekf.init({
          q: qFromEuler(0, 0, pre.chi),
          v: [pre.vE, pre.vN, 0],
          sigma: {
            roll: 2 * DEG, pitch: 2 * DEG, yaw: 5 * DEG, vel: 0.5, velZ: 0.2,
            gyroBias: 0.3 * DEG, accelBias: 0.2, scale: 0.02, alpha: this.cal.alphaSigma,
          },
        });
        this.ekfReady = true;
        this.hist = [];
        this.lastPseudoT = -Infinity;
      }
      return;
    }

    // Latency tracking from course vs gyro, only while gripping.
    if (pre.kind === 'doppler' && prev && prev.kind === 'doppler' && pre.chi !== null && prev.chi !== null &&
        pre.speed > 5 && prev.speed > 5 && t - prev.t < 2 && t - this.lastSlideT > 2) {
      this.latency.addCourseDelta(prev.t, t, wrapPi(pre.chi - prev.chi), this.yawHist,
        prev.speed, pre.speed, this.cfg.phonePos[0]);
    }

    const tm = pre.tMeas - this.latency.L;
    const H = this.hist;
    if (H.length === 0 || tm < H[0].t) { this.stats.gnssTooOld++; return; }
    let j = H.length - 1;
    while (j > 0 && H[j].t > tm) j--;
    const replay = j < H.length - 1;
    const saved = replay ? ekf.snapshot() : null;
    if (replay) ekf.restore(H[j].snap);

    const sigma = pre.kind === 'diff' ? 2 * this.gnssSigma + 0.3 : this.gnssSigma;
    const vp = ekf.predictedGnssVelocity();
    const yE = pre.vE - vp[0], yN = pre.vN - vp[1];
    let res = ekf.updateGnssVelocity(pre.vE, pre.vN, sigma, 25);
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
      if (pre.kind === 'doppler') {
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
  }

  // ---------------------------------------------------------------- output

  _output(t) {
    const o = {
      t,
      phase: this.phase,
      message: '',
      notice: this.phase === PHASE.RUNNING && this.ekfReady ? null : this.notice,
      calibrationHint: this.drive ? this.drive.status.reason : null,
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
          accelSign: this.cal.accelSign, gyroGain: this.cal.gyroGain,
          alphaSigma: this.cal.alphaSigma / DEG, smoothing: this.cal.smoothing,
        }
        : null,
    };
    switch (this.phase) {
      case PHASE.WAIT_GNSS:
        o.message = 'Waiting for GPS…';
        break;
      case PHASE.STANDSTILL:
        o.message = 'Keep the car still…';
        o.progress = Math.min(1, this.standstill.duration / this.cfg.standstillSeconds);
        break;
      case PHASE.DRIVE_CAL:
        o.message = 'Drive normally and take a few turns…';
        o.progress = this.drive.status.progress;
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
      stats: { ...this.stats },
    };
  }
}
