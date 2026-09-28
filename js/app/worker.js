// Runs the estimator off the main thread. Messages in:
//   { type: 'start', config }
//   { type: 'recalibrate' }
//   { type: 'events', events }  events: [0, t, gx, gy, gz, ax, ay, az] (IMU)
//                                       [1, t, lat, lon, speed, course] (GNSS)
// Messages out: { type: 'out', out } after each batch.

import { EstimatorHost } from './host.js';

const host = new EstimatorHost((out) => postMessage({ type: 'out', out }));
onmessage = (e) => host.handle(e.data);
