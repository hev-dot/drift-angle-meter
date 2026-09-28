// Browser sensor adapters. They convert whatever the browser delivers into the
// estimator's input format on one monotonic clock (performance.now(), ms):
//   IMU:  { t, gyro: [x, y, z] rad/s, accel: [x, y, z] m/s² incl. gravity }, device axes
//   GNSS: { t, lat, lon, speed (m/s | null), course (deg from north | null) }
// Sign or unit quirks of individual browsers are not corrected here; the estimator
// detects them during calibration.

const DEG = Math.PI / 180;

export class SensorError extends Error {
  constructor(code, message) {
    super(message);
    this.code = code;
  }
}

// Must be called directly from a user gesture (iOS shows its permission prompt then).
export async function requestMotionPermission() {
  const D = window.DeviceMotionEvent;
  if (!D) throw new SensorError('no-motion', 'This browser does not provide motion sensor data.');
  if (typeof D.requestPermission === 'function') {
    let r;
    try {
      r = await D.requestPermission();
    } catch {
      throw new SensorError('motion-denied', 'Motion sensor access was not granted.');
    }
    if (r !== 'granted') throw new SensorError('motion-denied', 'Motion sensor access was not granted.');
  }
}

// Event timestamps should be on the performance.now() clock; a few engines have used
// epoch time. Fall back to the handler time if the value is implausible.
function eventTime(e) {
  const now = performance.now();
  const ts = e.timeStamp;
  return Number.isFinite(ts) && ts > 0 && Math.abs(now - ts) < 1000 ? ts : now;
}

export function startMotion(onSample) {
  const stats = { events: 0, samples: 0, noGyro: 0 };
  const handler = (e) => {
    stats.events++;
    const a = e.accelerationIncludingGravity;
    const r = e.rotationRate;
    if (!a || a.x === null || a.y === null || a.z === null) return;
    if (!r || r.alpha === null || r.beta === null || r.gamma === null) {
      stats.noGyro++;
      return;
    }
    stats.samples++;
    // rotationRate: alpha about z, beta about x, gamma about y (deg/s per spec)
    onSample({
      t: eventTime(e),
      gyro: [r.beta * DEG, r.gamma * DEG, r.alpha * DEG],
      accel: [a.x, a.y, a.z],
    });
  };
  window.addEventListener('devicemotion', handler);
  return {
    stats,
    stop: () => window.removeEventListener('devicemotion', handler),
  };
}

export function startGnss(onFix, onError) {
  if (!('geolocation' in navigator)) {
    onError(new SensorError('no-gnss', 'This browser does not provide location.'));
    return { stop() {} };
  }
  const valid = (x) => (Number.isFinite(x) && x >= 0 ? x : null);
  const id = navigator.geolocation.watchPosition(
    (p) => {
      const c = p.coords;
      onFix({
        // arrival time: fix timestamps come from other clocks and can be seconds off
        t: performance.now(),
        lat: c.latitude,
        lon: c.longitude,
        speed: valid(c.speed),
        course: valid(c.heading),
        accuracy: c.accuracy,
      });
    },
    (err) => {
      const code = err.code === 1 ? 'gnss-denied' : 'gnss-error';
      const msg = err.code === 1 ? 'Location access was not granted.' : `Location error: ${err.message || err.code}`;
      onError(new SensorError(code, msg));
    },
    { enableHighAccuracy: true, maximumAge: 0, timeout: 15000 },
  );
  return { stop: () => navigator.geolocation.clearWatch(id) };
}

// Keeps the screen on while measuring (sensors stop when the page is hidden).
export class ScreenWakeLock {
  constructor() {
    this.lock = null;
    this.wanted = false;
    this._onVis = () => {
      if (this.wanted && document.visibilityState === 'visible') this._acquire();
    };
  }

  get supported() {
    return 'wakeLock' in navigator;
  }

  async start() {
    this.wanted = true;
    document.addEventListener('visibilitychange', this._onVis);
    await this._acquire();
  }

  async _acquire() {
    if (!this.supported || this.lock) return;
    try {
      this.lock = await navigator.wakeLock.request('screen');
      this.lock.addEventListener('release', () => { this.lock = null; });
    } catch {
      this.lock = null;
    }
  }

  stop() {
    this.wanted = false;
    document.removeEventListener('visibilitychange', this._onVis);
    if (this.lock) this.lock.release().catch(() => {});
    this.lock = null;
  }
}
