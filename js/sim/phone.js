// Randomized "virtual phones": everything that differs between real devices and
// browsers (mounting, sample rates, timing, noise, bias, GNSS latency and smoothing,
// sensor sign/unit bugs) is drawn at random, then applied to vehicle truth to
// produce the event stream a browser would deliver to the estimator.

import { DEG, rotZ, rotY, rotAxis, m3mul, m3tv, normalize } from '../core/math.js';
import { VehicleSim } from './vehicle.js';

const EARTH_R = 6371000;
const LAT0 = 60.17, LON0 = 24.94;

function mountBase(kind, rng) {
  // columns: phone x, y, z axes expressed in vehicle coordinates (x fwd, y left, z up)
  let px, py, pz;
  switch (kind) {
    case 'dash-portrait':
    case 'windshield':
      px = [0, -1, 0]; py = [0, 0, 1]; pz = [-1, 0, 0];
      break;
    case 'dash-landscape':
      px = [0, 0, 1]; py = [0, 1, 0]; pz = [-1, 0, 0];
      break;
    case 'flat': {
      const y = rng.uniform(-Math.PI, Math.PI);
      px = [Math.cos(y), Math.sin(y), 0]; pz = [0, 0, 1];
      py = [-Math.sin(y), Math.cos(y), 0];
      break;
    }
  }
  return [px[0], py[0], pz[0], px[1], py[1], pz[1], px[2], py[2], pz[2]];
}

export function randomPhone(rng, overrides = {}) {
  const mount = rng.pick(['dash-portrait', 'dash-landscape', 'flat', 'windshield']);
  let R = mountBase(mount, rng);
  if (mount === 'windshield') R = m3mul(rotY(-rng.uniform(15, 35) * DEG), R);
  const axis = normalize([rng.gauss(), rng.gauss(), rng.gauss()]);
  R = m3mul(rotZ(rng.uniform(-15, 15) * DEG), m3mul(rotAxis(axis, rng.uniform(0, 10) * DEG), R));

  const p = {
    mount,
    R, // vehicle <- phone
    phonePosError: [rng.gauss(0.15), rng.gauss(0.15), rng.gauss(0.1)],
    imuRate: rng.pick([50, 60, 60, 60, 100, 120, 200]),
    imuDelay: rng.uniform(0.005, 0.04),
    imuJitter: rng.uniform(0, 0.006),
    imuDrop: rng.uniform(0, 0.02),
    gyroNoise: rng.uniform(0.002, 0.008),
    gyroBias: [0, 1, 2].map(() => rng.gauss(0.3 * DEG)),
    gyroBiasWalk: 0.0002,
    gyroScale: [0, 1, 2].map(() => rng.gauss(0.01)),
    gyroJump: rng.chance(0.3) ? { t: rng.uniform(40, 140), d: [0, 1, 2].map(() => rng.gauss(0.2 * DEG)) } : null,
    accelNoise: rng.uniform(0.02, 0.08),
    accelBias: [0, 1, 2].map(() => rng.gauss(0.08)),
    accelScale: [0, 1, 2].map(() => rng.gauss(0.01)),
    vibration: rng.uniform(0.1, 0.4), // m/s² base road/engine vibration
    gnssRate: rng.pick([1, 1, 1, 1, 2, 5, 10]),
    gnssLatency: rng.uniform(0.05, 0.9),
    gnssJitter: rng.uniform(0, 0.04),
    gnssVelNoise: rng.uniform(0.05, 0.3),
    gnssSmoothing: rng.pick([0, 0, 0.3, 0.6, 1.0]),
    gnssNoDoppler: rng.chance(0.1),
    gnssDrop: 0.02,
    accelSign: rng.chance(0.05) ? -1 : 1,
    gyroUnit: rng.chance(0.05) ? rng.pick([180 / Math.PI, Math.PI / 180]) : 1,
    gyroSign: rng.chance(0.05) ? -1 : 1,
    rollGrad: rng.uniform(0.3, 0.6) * DEG,
    pitchGrad: rng.uniform(0.2, 0.5) * DEG,
    tireSlip: rng.uniform(0.2, 0.5) * DEG,
    // how the browser labels rotationRate: 0 = alpha/beta/gamma about x/y/z (Chrome),
    // 1 = about z/x/y (W3C spec text); see GYRO_MAPS
    gyroMap: rng.chance(0.3) ? 1 : 0,
  };
  return { ...p, ...overrides };
}

// Returns { events, truth } where events are sorted by delivery time and carry the
// exact payload the estimator receives from the browser adapters.
export function simulate(scenario, phone, rng, cfg) {
  const dt = 0.001;
  const posTrue = cfg.phonePos.map((x, k) => x + phone.phonePosError[k]);
  const veh = new VehicleSim(scenario, {
    phonePos: posTrue,
    outputPoint: cfg.outputPoint,
    rollGrad: phone.rollGrad,
    pitchGrad: phone.pitchGrad,
    tireSlip: phone.tireSlip,
    chi0: rng.uniform(-Math.PI, Math.PI),
  });
  const events = [];
  const truth = [];
  const vib = [0, 0, 0], vibG = [0, 0, 0];
  const aVib = Math.exp(-2 * Math.PI * 20 * dt);
  const sumF = [0, 0, 0], sumW = [0, 0, 0];
  let nSum = 0;
  const bias = phone.gyroBias.slice();
  let nextImu = 1 / phone.imuRate;
  let lastImuDel = 0;
  let nextGnss = rng.uniform(0, 1 / phone.gnssRate);
  const vS = [0, 0];
  const posErr = [rng.gauss(2), rng.gauss(2)];
  const posErrVel = [0, 0];
  let jumped = false;
  const end = scenario.end;

  for (let i = 0; ; i++) {
    const t = i * dt;
    if (t > end) break;
    const tr = veh.step(dt);
    const sp = Math.hypot(tr.vPhone[0], tr.vPhone[1]);

    // road / engine vibration (vehicle frame), band-limited
    const sv = phone.vibration + 0.03 * sp;
    for (let k = 0; k < 3; k++) {
      vib[k] = aVib * vib[k] + Math.sqrt(1 - aVib * aVib) * rng.gauss(sv * (k === 2 ? 1.5 : 1));
      vibG[k] = aVib * vibG[k] + Math.sqrt(1 - aVib * aVib) * rng.gauss(0.01 + 0.001 * sp);
    }
    // optional: phone knocked to a new orientation in its mount at time remount.t
    const R = phone.remount && t >= phone.remount.t ? phone.remount.R : phone.R;
    const fP = m3tv(R, [tr.f[0] + vib[0], tr.f[1] + vib[1], tr.f[2] + vib[2]]);
    const wP = m3tv(R, [tr.w[0] + vibG[0], tr.w[1] + vibG[1], tr.w[2] + vibG[2]]);
    for (let k = 0; k < 3; k++) { sumF[k] += fP[k]; sumW[k] += wP[k]; }
    nSum++;

    if (t >= nextImu) {
      const tS = nextImu;
      nextImu += 1 / phone.imuRate;
      for (let k = 0; k < 3; k++) bias[k] += rng.gauss(phone.gyroBiasWalk * Math.sqrt(1 / phone.imuRate));
      if (phone.gyroJump && !jumped && tS > phone.gyroJump.t) {
        for (let k = 0; k < 3; k++) bias[k] += phone.gyroJump.d[k];
        jumped = true;
      }
      const w = [0, 1, 2].map((k) =>
        ((1 + phone.gyroScale[k]) * sumW[k] / nSum + bias[k] + rng.gauss(phone.gyroNoise)) * phone.gyroUnit * phone.gyroSign);
      // adapter output is rotationRate [alpha, beta, gamma]
      const gyro = phone.gyroMap === 1 ? [w[2], w[0], w[1]] : w;
      const accel = [0, 1, 2].map((k) =>
        ((1 + phone.accelScale[k]) * sumF[k] / nSum + phone.accelBias[k] + rng.gauss(phone.accelNoise)) * phone.accelSign);
      sumF.fill(0); sumW.fill(0); nSum = 0;
      if (!rng.chance(phone.imuDrop)) {
        const tDel = Math.max(tS + phone.imuDelay + rng.exp(phone.imuJitter + 1e-9), lastImuDel + 0.0001);
        lastImuDel = tDel;
        events.push({ type: 'imu', tDel, tSample: tS, data: { t: tDel * 1000, gyro, accel } });
      }
    }

    // GNSS: optional smoothing (fused location providers), noise, latency
    if (phone.gnssSmoothing > 0) {
      const kf = 1 - Math.exp(-dt / phone.gnssSmoothing);
      vS[0] += kf * (tr.vPhone[0] - vS[0]);
      vS[1] += kf * (tr.vPhone[1] - vS[1]);
    } else {
      vS[0] = tr.vPhone[0]; vS[1] = tr.vPhone[1];
    }
    // slowly wandering position error: integrated Gauss-Markov velocity error (~0.05 m/s)
    const kv = dt / 20, kp = dt / 60;
    for (let k = 0; k < 2; k++) {
      posErrVel[k] += -kv * posErrVel[k] + rng.gauss(0.05 * Math.sqrt(2 * kv));
      posErr[k] += posErrVel[k] * dt - kp * posErr[k];
    }
    if (t >= nextGnss) {
      nextGnss += 1 / phone.gnssRate;
      if (!rng.chance(phone.gnssDrop)) {
        const vE = vS[0] + rng.gauss(phone.gnssVelNoise), vN = vS[1] + rng.gauss(phone.gnssVelNoise);
        let speed = Math.hypot(vE, vN);
        let course = ((Math.atan2(vE, vN) / DEG) + 360) % 360;
        if (speed < 0.3) { course = NaN; }
        if (tr.speed < 0.05) { speed = Math.abs(rng.gauss(0.05)); course = NaN; }
        if (phone.gnssNoDoppler) { speed = null; course = null; }
        const E = tr.posPhone[0] + posErr[0] + rng.gauss(0.3);
        const Nn = tr.posPhone[1] + posErr[1] + rng.gauss(0.3);
        const lat = LAT0 + Nn / EARTH_R / DEG;
        const lon = LON0 + E / (EARTH_R * Math.cos(LAT0 * DEG)) / DEG;
        const tArr = t + phone.gnssLatency + rng.exp(phone.gnssJitter + 1e-9);
        events.push({ type: 'gnss', tDel: tArr, tSample: t, data: { t: tArr * 1000, lat, lon, speed, course } });
      }
    }

    if (i % 10 === 0) {
      truth.push({ t, beta: tr.beta, speed: tr.speed, scriptBeta: tr.scriptBeta, yawRate: tr.yawRate });
    }
  }
  events.sort((a, b) => a.tDel - b.tDel);
  return { events, truth };
}
