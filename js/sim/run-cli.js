// Monte Carlo over randomized virtual phones.
//   node js/sim/run-cli.js [--runs 30] [--seed 1] [--scenario driftSession|trackRun]
//                          [--sessions 2] [--verbose]
// With --sessions 2 every phone drives twice; the second drive starts from the profile
// (device + mount) the app would have stored after the first.

import { runOnce, describePhone } from './run.js';

const args = process.argv.slice(2);
const arg = (name, def) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : def;
};
const runs = Number(arg('runs', 30));
const seed0 = Number(arg('seed', 1));
const scenario = arg('scenario', 'driftSession');
const sessions = Number(arg('sessions', 1));
const verbose = args.includes('--verbose');

const f = (x, d = 1) => (Number.isFinite(x) ? x.toFixed(d) : '  - ');
const results = Array.from({ length: sessions }, () => []);
for (let i = 0; i < runs; i++) {
  const seed = seed0 + i;
  let profile = null;
  for (let s = 1; s <= sessions; s++) {
    const r = runOnce({ seed, scenario, session: s, profile });
    profile = r.profile;
    const m = r.metrics;
    results[s - 1].push(m);
    const flags = [];
    if (!m.calibrated) flags.push('NOT CALIBRATED');
    if (m.sensorsVerified && !m.gyroMapOk) flags.push('gyro-axes wrong');
    if (m.sensorsVerified && !m.gyroGainOk) flags.push('gyro-gain wrong');
    if (m.accelVerified && !m.accelSignOk) flags.push('accel-sign wrong');
    if (s === 1) console.log(`#${String(seed).padEnd(4)} ${describePhone(r.phone)}`);
    console.log(
      `   s${s} live ${f(m.tLive, 1)}s after moving (${m.mountSource ?? '-'})  drift rmse ${f(m.drift.rmse)}° p95 ${f(m.drift.p95)}° max ${f(m.drift.max)}°` +
      `  grip rmse ${f(m.grip.rmse)}°  avail ${f(100 * m.driftAvailability, 0)}%  overconf ${f(100 * m.overconfidence, 1)}%` +
      `  sensors ${m.sensorsVerified ? 'ok' : 'unchecked'}  latency ${m.latencyEst !== null ? f(m.latencyEst * 1000, 0) : '-'}/${f(m.latencyTrue * 1000, 0)}ms` +
      (flags.length ? `  !! ${flags.join(', ')}` : ''),
    );
    if (verbose) console.log('      ', JSON.stringify(m.debug));
  }
}

const sorted = (xs) => xs.filter(Number.isFinite).sort((a, b) => a - b);
const q = (xs, p) => (xs.length ? xs[Math.min(xs.length - 1, Math.floor(p * xs.length))] : NaN);
for (let s = 0; s < sessions; s++) {
  const all = results[s];
  const cal = all.filter((m) => m.calibrated);
  const dr = sorted(cal.map((m) => m.drift.rmse));
  const d95 = sorted(cal.map((m) => m.drift.p95));
  const gr = sorted(cal.map((m) => m.grip.rmse));
  const av = sorted(cal.map((m) => m.driftAvailability));
  const oc = sorted(cal.map((m) => m.overconfidence));
  const live = sorted(all.map((m) => m.tLive));
  console.log(`\n=== summary${sessions > 1 ? ` session ${s + 1}` : ''} (${scenario}) ===`);
  console.log(`live               ${live.length}/${all.length}; seconds after moving: median ${f(q(live, 0.5))}  p90 ${f(q(live, 0.9))}  worst ${f(live[live.length - 1])}`);
  console.log(`sensor check done  ${all.filter((m) => m.sensorsVerified).length}/${all.length}, wrong ${all.filter((m) => m.sensorsVerified && !(m.gyroMapOk && m.gyroGainOk)).length}`);
  console.log(`drift RMSE (°)     median ${f(q(dr, 0.5))}  p90 ${f(q(dr, 0.9))}  worst ${f(dr[dr.length - 1])}`);
  for (const [label, grp] of [['standard phones', cal.filter((m) => !m.quirky)], ['phones w/ quirks', cal.filter((m) => m.quirky)]]) {
    const s = sorted(grp.map((m) => m.driftShown.rmse));
    const a = sorted(grp.map((m) => m.driftAvailability));
    console.log(`  shown, ${label.padEnd(16)} (${grp.length}): RMSE median ${f(q(s, 0.5))}  p90 ${f(q(s, 0.9))}  worst ${f(s[s.length - 1])}; shown ${f(100 * q(a, 0.5), 0)}% of drift time (p10 ${f(100 * q(a, 0.1), 0)}%)`);
  }
  console.log(`drift P95 |err| (°) median ${f(q(d95, 0.5))}  p90 ${f(q(d95, 0.9))}`);
  console.log(`grip RMSE (°)      median ${f(q(gr, 0.5))}  p90 ${f(q(gr, 0.9))}`);
  console.log(`drift availability median ${f(100 * q(av, 0.5), 0)}%  p10 ${f(100 * q(av, 0.1), 0)}%`);
  console.log(`overconfidence     median ${f(100 * q(oc, 0.5), 1)}%  p90 ${f(100 * q(oc, 0.9), 1)}%`);
}
