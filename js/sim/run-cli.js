// Monte Carlo over randomized virtual phones.
//   node js/sim/run-cli.js [--runs 30] [--seed 1] [--verbose]

import { runOnce, describePhone } from './run.js';

const args = process.argv.slice(2);
const arg = (name, def) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : def;
};
const runs = Number(arg('runs', 30));
const seed0 = Number(arg('seed', 1));
const verbose = args.includes('--verbose');

const f = (x, d = 1) => (Number.isFinite(x) ? x.toFixed(d) : '  - ');
const results = [];
for (let i = 0; i < runs; i++) {
  const seed = seed0 + i;
  const { phone, metrics: m } = runOnce({ seed });
  results.push(m);
  const flags = [];
  if (!m.calibrated) flags.push('NOT CALIBRATED');
  else {
    if (!m.accelSignOk) flags.push('accel-sign wrong');
    if (!m.gyroGainOk) flags.push('gyro-gain wrong');
  }
  console.log(
    `#${String(seed).padEnd(4)} ${describePhone(phone)}\n` +
    `      run@${f(m.tRunning, 0)}s  drift rmse ${f(m.drift.rmse)}° p95 ${f(m.drift.p95)}° max ${f(m.drift.max)}°` +
    `  grip rmse ${f(m.grip.rmse)}°  avail ${f(100 * m.driftAvailability, 0)}%  overconf ${f(100 * m.overconfidence, 1)}%` +
    `  latency ${m.calibrated ? f(m.latencyEst * 1000, 0) : '-'}/${f(m.latencyTrue * 1000, 0)}ms` +
    ` smooth ${m.calibrated ? f(m.smoothingEst, 2) : '-'}s  cpu ${f(m.cpuPerSimSecondMs, 1)}ms/s` +
    (flags.length ? `  !! ${flags.join(', ')}` : ''),
  );
  if (verbose) console.log('      ', JSON.stringify(m.debug));
}

const cal = results.filter((m) => m.calibrated);
const sorted = (xs) => xs.filter(Number.isFinite).sort((a, b) => a - b);
const q = (xs, p) => (xs.length ? xs[Math.min(xs.length - 1, Math.floor(p * xs.length))] : NaN);
const dr = sorted(cal.map((m) => m.drift.rmse));
const d95 = sorted(cal.map((m) => m.drift.p95));
const gr = sorted(cal.map((m) => m.grip.rmse));
const av = sorted(cal.map((m) => m.driftAvailability));
const oc = sorted(cal.map((m) => m.overconfidence));
console.log('\n=== summary ===');
console.log(`calibrated         ${cal.length}/${results.length}`);
console.log(`faults identified  ${cal.filter((m) => m.accelSignOk && m.gyroGainOk).length}/${cal.length}`);
console.log(`drift RMSE (°)     median ${f(q(dr, 0.5))}  p90 ${f(q(dr, 0.9))}  worst ${f(dr[dr.length - 1])}`);
console.log(`drift P95 |err| (°) median ${f(q(d95, 0.5))}  p90 ${f(q(d95, 0.9))}`);
console.log(`grip RMSE (°)      median ${f(q(gr, 0.5))}  p90 ${f(q(gr, 0.9))}`);
console.log(`drift availability median ${f(100 * q(av, 0.5), 0)}%  p10 ${f(100 * q(av, 0.1), 0)}%`);
console.log(`overconfidence     median ${f(100 * q(oc, 0.5), 1)}%  p90 ${f(100 * q(oc, 0.9), 1)}%`);
