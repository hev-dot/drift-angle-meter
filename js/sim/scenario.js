// Driving scenarios as keyframes of speed, course rate and drift angle, all defined at
// the rear axle centre. Values move between keyframes along smooth S-curves.

import { DEG } from '../core/math.js';

export class Scenario {
  constructor() {
    this.frames = [{ t: 0, v: 0, chiDot: 0, beta: 0 }];
  }

  get end() {
    return this.frames[this.frames.length - 1].t;
  }

  // Move to new values over `dur` seconds (unspecified values are kept).
  to(dur, change = {}) {
    const last = this.frames[this.frames.length - 1];
    this.frames.push({ ...last, ...change, t: last.t + dur });
    return this;
  }

  hold(dur) {
    return this.to(dur);
  }

  // Values and time derivatives at time t.
  at(t) {
    const F = this.frames;
    if (t >= this.end) {
      const l = F[F.length - 1];
      return { v: l.v, vDot: 0, chiDot: l.chiDot, beta: l.beta, betaDot: 0 };
    }
    let i = this._i ?? 0;
    if (F[i].t > t) i = 0;
    while (F[i + 1].t <= t) i++;
    this._i = i;
    const a = F[i], b = F[i + 1];
    const T = b.t - a.t;
    const x = (t - a.t) / T;
    const s = x * x * (3 - 2 * x);
    const ds = (6 * x * (1 - x)) / T;
    return {
      v: a.v + (b.v - a.v) * s,
      vDot: (b.v - a.v) * ds,
      chiDot: a.chiDot + (b.chiDot - a.chiDot) * s,
      beta: a.beta + (b.beta - a.beta) * s,
      betaDot: (b.beta - a.beta) * ds,
    };
  }
}

// A typical session: standstill, a minute of ordinary driving (calibration), then a
// mix of drifts: entries, transitions, big angle, low-grip slow drift, long varying drift.
// Left-hand turns have chiDot > 0; drifting through them gives beta < 0.
export function driftSession() {
  const s = new Scenario();
  s.hold(8);
  s.to(5, { v: 12 }).hold(2);
  const speeds = [9, 13, 11, 12, 10, 12];
  for (let i = 0; i < 6; i++) {
    s.to(1, { chiDot: 0.25 }).hold(3).to(1, { chiDot: 0 }).hold(2);
    s.to(1, { chiDot: -0.25 }).hold(3).to(1, { chiDot: 0 }).hold(2);
    s.to(2, { v: speeds[i] });
  }
  s.to(2, { v: 12 }).hold(2);

  // D1: left-hand drift
  s.to(0.8, { chiDot: 0.5, beta: -30 * DEG }).hold(4);
  s.to(1, { chiDot: 0, beta: 0 }).hold(4);
  // D2: right-hand drift, transition to left, exit
  s.to(0.8, { chiDot: -0.5, beta: 35 * DEG }).hold(3);
  s.to(1, { chiDot: 0.5, beta: -35 * DEG }).hold(3);
  s.to(1, { chiDot: 0, beta: 0 }).hold(4);
  // D3: big angle
  s.to(2, { v: 10 });
  s.to(1, { chiDot: 0.6, beta: -50 * DEG }).hold(5);
  s.to(1.2, { chiDot: 0, beta: 0 }).hold(4);
  // D4: slow low-grip drift (small lateral g)
  s.to(2, { v: 7 });
  s.to(1.5, { chiDot: 0.25, beta: -25 * DEG }).hold(6);
  s.to(1.5, { chiDot: 0, beta: 0 }).hold(4);
  // D5: long drift with varying angle
  s.to(2, { v: 11 });
  s.to(1, { chiDot: -0.45, beta: 40 * DEG }).hold(2);
  s.to(2, { beta: 30 * DEG }).to(2, { beta: 45 * DEG }).to(2, { beta: 35 * DEG }).hold(3);
  s.to(1.2, { chiDot: 0, beta: 0 }).hold(5);
  // some grip driving
  s.to(1, { chiDot: 0.3 }).hold(2).to(1, { chiDot: 0 }).hold(3);
  // D6: short left drift
  s.to(0.7, { chiDot: 0.5, beta: -25 * DEG }).hold(2);
  s.to(1, { chiDot: 0, beta: 0 }).hold(3);
  s.to(4, { v: 0 }).hold(5);
  return s;
}

export const SCENARIOS = { driftSession };
