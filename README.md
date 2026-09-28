# Drift angle meter (browser, phone sensors)

Measures a car's drift (sideslip) angle with a phone's own sensors, running in the phone
browser with no installation. Plain ES modules, no build step, no dependencies.

## How it works

- **Estimator**: error-state EKF on the phone IMU (strapdown INS), aided by GNSS
  velocity. While the car grips, a non-holonomic constraint (no sideways motion at the
  rear axle) calibrates heading. While it slides, heading is carried by the gyro and
  corrected by GNSS velocity and centripetal acceleration.
- **Self-calibration**: nothing is configured per phone. Calibration has two stages:
  - A few seconds standing still give gravity direction, gyro bias and noise levels.
  - About a minute of normal driving gives the rest, by comparison against GNSS:
    - mount orientation
    - GNSS latency and smoothing
    - accelerometer and gyro sign/unit faults
- **Browser realities handled**: GNSS delivered late (IMU history replay), smoothed
  GNSS velocity (modelled as extra filter states), missing speed/course (least-squares
  velocity from positions), jittery or dropped IMU samples, and axis sign/unit bugs.
- **Confidence**: every reading has a 1σ estimate, and the quality flag uses it.

Sign convention: β > 0 when the car moves to the left of where it points (ISO 8855),
so a drift through a left-hand corner reads negative.

## The app (`index.html`)

To use it:
1. Open the page over **https**.
2. Mount the phone firmly and tap **Start**.
3. Keep the car still for about 3 s.
4. Drive normally for about a minute, with turns, accelerating and braking.

What it shows:
- **Gauge**: the drift angle and which side the nose points.
- **Accuracy chip**: the ±2σ confidence. The gauge dims when accuracy is poor.
- **Stats**: speed, peak of the current drift, session best and last drift.

What it handles:
- **Recalibration**: automatic if the phone is knocked in its mount, or via a button.
- **Gaps**: if the page is backgrounded, the filter restarts cleanly.

Optional settings: phone distance ahead of the rear axle, and wheelbase. The angle is
reported at mid-wheelbase.

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
| `js/core/calibration.js` | Standstill and driving self-calibration |
| `js/core/latency.js` | GNSS latency tracking |
| `index.html`, `js/app/` | The app: sensor adapters, worker host, UI |
| `js/sim/` | Simulation harness: vehicle truth, randomized virtual phones, metrics |
| `sim.html` | Browser view of a simulated run and Monte Carlo |
| `tools/` | Dev server (http/https), certificate script, log replay |

## Running

```
node js/sim/run-cli.js --runs 100 --seed 1     # Monte Carlo over random virtual phones
node tools/serve.js                              # then open http://localhost:8080/sim.html
```

The virtual phones randomize:
- mount orientation
- IMU rate (50–200 Hz), delay, jitter and drops
- gyro and accelerometer noise, bias and scale
- GNSS rate (1–10 Hz), latency (0.05–0.9 s), smoothing (0–1 s) and noise
- missing GNSS speed/course
- sign and unit faults

## Current simulated results (100 random phones, `--seed 1`)

| Metric | Result |
|---|---|
| Calibrated | 100 / 100 |
| Sensor faults identified | 100 / 100 |
| Drift RMSE (\|β\| ≥ 10°) | median 1.7°, 90th pct 2.3°, worst 3.0° |
| Grip RMSE | median 0.8° |

These results are from simulation only. Real-device testing still has to confirm that the
simulated error sources match real phones and browsers.
