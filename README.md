# Drift angle meter (browser, phone sensors)

Measures a car's drift (sideslip) angle with a phone's own sensors, running in the phone
browser with no installation. Plain ES modules, no build step, no dependencies.

## How it works

- **Estimator**: error-state EKF on the phone IMU (strapdown INS), aided by GNSS
  velocity. While the car grips, a non-holonomic constraint (no sideways motion at the
  rear axle) calibrates heading. While it slides, heading is carried by the gyro and
  corrected by GNSS velocity and centripetal acceleration.
- **Self-calibration**: nothing is configured per phone, and it is quick enough for a
  short track run:
  - 2 s standing still (e.g. on the start line): gravity direction, gyro bias, noise.
  - The launch (any firm straight acceleration or braking, or a normal corner): which
    way the phone faces. The gauge goes live a couple of seconds after pulling away.
  - In the background while driving: the phone/browser's gyro axis order, units and
    signs (from the first ~100° of turning), GNSS delay and smoothing (from velocity
    changes, including during drifts).
  - The phone profile and mount are remembered, so later sessions are ready right after
    the standstill if the phone is mounted the same way.
- **Browser realities handled**: browsers disagree on which axis `rotationRate`'s
  alpha/beta/gamma are about (Chrome on Android: x/y/z, measured; the spec text: z/x/y),
  GNSS delivered late (IMU history replay), smoothed GNSS velocity (extra filter states),
  high-rate correlated fixes, missing speed/course (least-squares velocity from positions),
  jittery or dropped IMU samples, sign/unit bugs.
- **Recovery**: heading re-taken from GNSS after slow manoeuvring; forward direction
  re-found if the filter keeps disagreeing with GNSS; readings are flagged when the
  filter's GNSS residuals show it is inconsistent.
- **Confidence**: every reading has a 1σ estimate, and the quality flag uses it.

The angle is reported at the rear axle, where it is ~0° whenever the car grips (also in
tight, slow turns). Sign convention: β > 0 when the car moves to the left of where it
points (ISO 8855), so a drift through a left-hand corner reads negative.

## The app (`index.html`)

To use it:
1. Open the page over **https**.
2. Mount the phone firmly and tap **Start**.
3. Keep the car still for 2 s.
4. Drive off with a firm straight launch (or take a normal corner).
5. First time with a phone: the sensor check completes during the first corners.

What it shows:
- **Gauge**: the drift angle and which side the nose points.
- **Accuracy chip**: the ±2σ confidence. The gauge dims when accuracy is poor.
- **Stats**: speed, peak of the current drift, session best and last drift.

What it handles:
- **Recalibration**: automatic if the phone is knocked in its mount, or via a button.
- **Gaps**: if the page is backgrounded, the filter restarts cleanly.

Settings: phone distance ahead of the rear axle, and "forget this phone's calibration".

Other features:
- **Save log** downloads the raw sensor stream. Replay it with `node tools/replay.js <file>`.
- **Demo without sensors**: open `index.html?demo=5` to replay a simulated session at 5× speed.

Browser support:
- **Android**: Chrome.
- **iPhone**: Safari. It asks for motion permission on the Start tap. If access was
  denied earlier, re-enable it under Settings › Safari.
- **Estimator thread**: runs in a module Web Worker, with a main-thread fallback for
  browsers that don't support one.

### Testing on a phone

Sensors only work on https pages.
1. Run `powershell -ExecutionPolicy Bypass -File tools/make-cert.ps1` once. It creates a
   self-signed certificate for this PC's addresses.
2. Run `node tools/serve.js --https`.
3. On the phone, which must be on the same network, open the printed `https://<pc-ip>:8443/`
   address and accept the certificate warning once.
4. Windows Firewall may ask to allow Node.

For real use, host the folder on any static https host, such as GitHub Pages, Netlify or
Cloudflare Pages. Don't upload `tools/dev-cert.pfx`.

## Layout

| Path | What |
|---|---|
| `js/core/estimator.js` | Entry point: `addImu`, `addGnss`, phases, slide detection, delayed-GNSS replay |
| `js/core/ekf.js` | Error-state EKF |
| `js/core/calibration.js` | Standstill, launch/turn mount alignment, background sensor check |
| `js/core/latency.js` | GNSS delay and smoothing tracking |
| `index.html`, `js/app/` | The app: sensor adapters, worker host, UI |
| `js/sim/` | Simulation harness: vehicle truth, randomized virtual phones, metrics |
| `sim.html` | Browser view of a simulated run and Monte Carlo |
| `tools/` | Dev server (http/https), certificate script, log replay |

## Running

```
node js/sim/run-cli.js --runs 40 --scenario trackRun --sessions 2   # short track runs, 2 sessions per phone
node js/sim/run-cli.js --runs 30 --scenario driftSession            # longer session with normal driving
node tools/replay.js drift-log-XXXX.json [--trace] [--save-profile p.json] [--profile p.json]
node tools/serve.js                                                  # then open http://localhost:8080/sim.html
```

The virtual phones randomize:
- mount orientation
- IMU rate (50–200 Hz), delay, jitter and drops
- gyro and accelerometer noise, bias and scale
- GNSS rate (1–10 Hz), latency (0.05–0.9 s), smoothing (0–1 s) and noise
- missing GNSS speed/course
- gyro axis order (30% use the spec order), sign and unit faults

## Current results

Simulated short track run (6 s on the line, launch, drifts from ~8 s later, no warm-up
driving), 40 random phones, drift angle error while shown on the gauge:

| | First session with a phone | Later sessions |
|---|---|---|
| Live after pulling away | median 2.5 s | median 2.1 s |
| Standard phones (Chrome-like sensors) | median 2.7°, worst 5.4° | median 2.0°, worst 3.1° |
| Phones with unusual sensor conventions | unreliable until the sensor check completes | mostly fine; some still unreliable |

Longer session with normal driving before the drifts: median 2.0°, worst 4.4° (all phones).

Real device (Android 10, Chrome 139, a drive on public roads, replayed): sensor axis order
detected and confirmed (alpha = x), GNSS delay ≈ 0.2–0.3 s, normal driving reads
0.4° ± 1.9° (straight) and 0.6° ± 2.3° (turning). No sustained drift was detected on
that drive, so drift accuracy on a real car is still unverified.
