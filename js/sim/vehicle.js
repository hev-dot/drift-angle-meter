// Rigid-body vehicle truth from a scenario: attitude (yaw from course and drift angle,
// roll/pitch from body motion), angular rate, specific force and velocity at the phone,
// and the true drift angle at the reporting point.

import {
  G, rotX, rotY, rotZ, m3mul, m3v, m3tv, m3T, cross, add,
} from '../core/math.js';

const atan2Safe = (y, x) => (Math.abs(x) + Math.abs(y) < 1e-6 ? 0 : Math.atan2(y, x));

export class VehicleSim {
  // opts: { phonePos, outputPoint, rollGrad (rad per m/s²), pitchGrad, tireSlip (rad per m/s²) }
  constructor(scenario, opts) {
    this.sc = scenario;
    this.o = opts;
    this.t = 0;
    this.chi = opts.chi0 ?? 0;
    this.pos = [0, 0, 0];
    this.roll = 0;
    this.pitch = 0;
    this.Cprev = null;
    this.wPrev = [0, 0, 0];
  }

  step(dt) {
    const o = this.o;
    const k = this.sc.at(this.t);
    const aLat = k.v * k.chiDot;
    const beta = k.beta - o.tireSlip * aLat; // small rear slip angle while gripping
    this.chi += k.chiDot * dt;
    const chi = this.chi;
    const psi = chi - beta;

    const cc = Math.cos(chi), sc = Math.sin(chi);
    const vRa = [k.v * cc, k.v * sc, 0];
    const aRa = [k.vDot * cc - k.v * k.chiDot * sc, k.vDot * sc + k.v * k.chiDot * cc, 0];

    // body roll / pitch follow the acceleration in the yaw frame with some lag
    const cp = Math.cos(psi), sp = Math.sin(psi);
    const ax = cp * aRa[0] + sp * aRa[1], ay = -sp * aRa[0] + cp * aRa[1];
    const kf = 1 - Math.exp(-dt / 0.15);
    this.roll += kf * (o.rollGrad * ay - this.roll);
    this.pitch += kf * (-o.pitchGrad * ax - this.pitch);

    const C = m3mul(rotZ(psi), m3mul(rotY(this.pitch), rotX(this.roll)));
    let w = [0, 0, 0];
    if (this.Cprev) {
      const M = m3mul(m3T(this.Cprev), C);
      w = [(M[7] - M[5]) / (2 * dt), (M[2] - M[6]) / (2 * dt), (M[3] - M[1]) / (2 * dt)];
    }
    const wDot = this.Cprev ? [(w[0] - this.wPrev[0]) / dt, (w[1] - this.wPrev[1]) / dt, (w[2] - this.wPrev[2]) / dt] : [0, 0, 0];
    this.Cprev = C;
    this.wPrev = w;

    const p = o.phonePos;
    const wxp = cross(w, p);
    const aRel = add(cross(wDot, p), cross(w, wxp));
    const aPhoneN = add(aRa, m3v(C, aRel));
    const f = m3tv(C, [aPhoneN[0], aPhoneN[1], aPhoneN[2] + G]);
    const vPhone = add(vRa, m3v(C, wxp));
    this.pos = add(this.pos, [vRa[0] * dt, vRa[1] * dt, 0]);
    const posPhone = add(this.pos, m3v(C, p));

    const vOut = add(m3tv(C, vRa), cross(w, o.outputPoint));
    const out = {
      t: this.t,
      C,
      w,
      f,
      vPhone,
      posPhone,
      speed: Math.hypot(vOut[0], vOut[1]),
      beta: atan2Safe(vOut[1], vOut[0]),
      scriptBeta: k.beta,
      yawRate: w[2],
      aLat,
    };
    this.t += dt;
    return out;
  }
}
