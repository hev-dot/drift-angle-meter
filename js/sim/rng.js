// Seeded random numbers so every simulated run is reproducible.

export class Rng {
  constructor(seed = 1) {
    this.s = seed >>> 0 || 1;
    this._spare = null;
  }

  // mulberry32
  next() {
    let t = (this.s = (this.s + 0x6d2b79f5) >>> 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  uniform(a = 0, b = 1) {
    return a + (b - a) * this.next();
  }

  gauss(sigma = 1) {
    if (this._spare !== null) {
      const s = this._spare;
      this._spare = null;
      return s * sigma;
    }
    let u, v, r;
    do {
      u = 2 * this.next() - 1;
      v = 2 * this.next() - 1;
      r = u * u + v * v;
    } while (r >= 1 || r === 0);
    const m = Math.sqrt((-2 * Math.log(r)) / r);
    this._spare = v * m;
    return u * m * sigma;
  }

  pick(arr) {
    return arr[Math.floor(this.next() * arr.length)];
  }

  chance(p) {
    return this.next() < p;
  }

  exp(mean) {
    return -Math.log(1 - this.next()) * mean;
  }
}
