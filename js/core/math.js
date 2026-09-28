// Small vector / rotation / matrix helpers. No dependencies.
// 3x3 matrices are flat row-major arrays of length 9. Vectors are arrays of length 3.
// Quaternions are [w, x, y, z] (Hamilton), q maps body -> nav: v_n = C(q) v_b.

export const DEG = Math.PI / 180;
export const G = 9.80665;

export function wrapPi(a) {
  a = (a + Math.PI) % (2 * Math.PI);
  if (a < 0) a += 2 * Math.PI;
  return a - Math.PI;
}

export function clamp(x, lo, hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

// ---- 3-vectors ----
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
export const norm = (a) => Math.hypot(a[0], a[1], a[2]);
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export function normalize(a) {
  const n = norm(a);
  return n > 0 ? scale(a, 1 / n) : [0, 0, 0];
}

// ---- 3x3 matrices ----
export function m3v(M, v) {
  return [
    M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
    M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
    M[6] * v[0] + M[7] * v[1] + M[8] * v[2],
  ];
}
// M^T v
export function m3tv(M, v) {
  return [
    M[0] * v[0] + M[3] * v[1] + M[6] * v[2],
    M[1] * v[0] + M[4] * v[1] + M[7] * v[2],
    M[2] * v[0] + M[5] * v[1] + M[8] * v[2],
  ];
}
export function m3mul(A, B) {
  const C = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
  return C;
}
export function m3T(A) {
  return [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
}
export function rotX(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [1, 0, 0, 0, c, -s, 0, s, c];
}
export function rotY(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, 0, s, 0, 1, 0, -s, 0, c];
}
export function rotZ(a) {
  const c = Math.cos(a), s = Math.sin(a);
  return [c, -s, 0, s, c, 0, 0, 0, 1];
}
// Rotation about a unit axis (Rodrigues).
export function rotAxis(axis, a) {
  const [x, y, z] = normalize(axis);
  const c = Math.cos(a), s = Math.sin(a), t = 1 - c;
  return [
    t * x * x + c, t * x * y - s * z, t * x * z + s * y,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c,
  ];
}
// Matrix whose rows are the given vectors.
export function fromRows(r0, r1, r2) {
  return [r0[0], r0[1], r0[2], r1[0], r1[1], r1[2], r2[0], r2[1], r2[2]];
}

// ---- quaternions ----
export function qmul(a, b) {
  return [
    a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
    a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
    a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
    a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
  ];
}
export function qnormalize(q) {
  const n = Math.hypot(q[0], q[1], q[2], q[3]);
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}
export function qFromRotVec(v) {
  const a = norm(v);
  if (a < 1e-9) return qnormalize([1, v[0] / 2, v[1] / 2, v[2] / 2]);
  const s = Math.sin(a / 2) / a;
  return [Math.cos(a / 2), v[0] * s, v[1] * s, v[2] * s];
}
export function qToMat(q) {
  const [w, x, y, z] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
  ];
}
// ZYX Euler (yaw about z, then pitch about y, then roll about x): C = Rz(yaw) Ry(pitch) Rx(roll)
export function qFromEuler(roll, pitch, yaw) {
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  return [
    cy * cp * cr + sy * sp * sr,
    cy * cp * sr - sy * sp * cr,
    cy * sp * cr + sy * cp * sr,
    sy * cp * cr - cy * sp * sr,
  ];
}
export function eulerFromMat(C) {
  return {
    roll: Math.atan2(C[7], C[8]),
    pitch: -Math.asin(clamp(C[6], -1, 1)),
    yaw: Math.atan2(C[3], C[0]),
  };
}

// ---- small dense matrices (Float64Array, row-major, n x m) ----
// Inverse of a small symmetric positive-definite matrix (m <= 3) via Gauss-Jordan.
export function invSmall(S, m) {
  const A = Float64Array.from(S);
  const I = new Float64Array(m * m);
  for (let i = 0; i < m; i++) I[i * m + i] = 1;
  for (let c = 0; c < m; c++) {
    let p = c;
    for (let r = c + 1; r < m; r++) if (Math.abs(A[r * m + c]) > Math.abs(A[p * m + c])) p = r;
    if (Math.abs(A[p * m + c]) < 1e-300) return null;
    if (p !== c) {
      for (let k = 0; k < m; k++) {
        let t = A[c * m + k]; A[c * m + k] = A[p * m + k]; A[p * m + k] = t;
        t = I[c * m + k]; I[c * m + k] = I[p * m + k]; I[p * m + k] = t;
      }
    }
    const d = A[c * m + c];
    for (let k = 0; k < m; k++) { A[c * m + k] /= d; I[c * m + k] /= d; }
    for (let r = 0; r < m; r++) {
      if (r === c) continue;
      const f = A[r * m + c];
      if (f === 0) continue;
      for (let k = 0; k < m; k++) { A[r * m + k] -= f * A[c * m + k]; I[r * m + k] -= f * I[c * m + k]; }
    }
  }
  return I;
}
