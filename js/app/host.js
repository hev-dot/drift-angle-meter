// Message handling around the estimator, shared by the Web Worker and the
// main-thread fallback (browsers without module workers).

import { DriftEstimator } from '../core/estimator.js';

export class EstimatorHost {
  constructor(emit) {
    this.emit = emit;
    this.est = null;
  }

  handle(m) {
    switch (m.type) {
      case 'start':
        this.est = new DriftEstimator(m.config);
        this.emit(this.est.out);
        break;
      case 'recalibrate':
        if (this.est) this.emit(this.est.recalibrate('Recalibrating…'));
        break;
      case 'events': {
        if (!this.est) return;
        let out = null;
        for (const ev of m.events) {
          out = ev[0] === 0
            ? this.est.addImu({ t: ev[1], gyro: [ev[2], ev[3], ev[4]], accel: [ev[5], ev[6], ev[7]] })
            : this.est.addGnss({ t: ev[1], lat: ev[2], lon: ev[3], speed: ev[4], course: ev[5] });
        }
        if (out) this.emit(out);
        break;
      }
    }
  }
}
