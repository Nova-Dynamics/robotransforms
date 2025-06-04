/**
 * Transformations
 *
 * All transformations are of coordinates NOT vectors
 *
 * 'sr' means shift then rotate, so that the "shift" (what I add to a vector to put it into the new coordinate reference frame) is in the initial coordinate reference frame
 * 'lr' means location then rotate, so that the "location" (where the terminal coordinate's origin is relative to the initial) is in the initial coordinate reference frame
 *
 * Types:
 *     euler   - [ yaw, pitch, roll ]
 *     quat    - [ re, i, j, k ]
 *     redquat - [ i, j, k ] (where re is assumed positive)
 *     rotvec  - [ e1, e2, e3 ] = 2*arcsin(|[i,j,k]|)* hat([i,j,k])
 *     rotmat  - 3x3
 *     homo    - 4x4
 *     srq     - [ x, y, z, i, j, k ]
 *     sre     - [ x, y, z, yaw, pitch, roll ]
 *     lrq     - [ x, y, z, i, j, k ]
 *     lre     - [ x, y, z, yaw, pitch, roll ]
 *     lrQ     - [ x, y, z, re, i, j, k ]
 *     lrrv    - [ x, y, z, e1, e2, e3 ]
 */

function identity_matrix(n) {
    const A = Array(n).fill().map(() => Array(n).fill(0));
    for ( let i = 0; i < n; i ++ ) A[i][i] = 1;
    return A
}
function _invert_matrix_subblock(A, I, offset) {
    let ri = null;
    for ( let i = offset; i < A.length; i ++ ) {
        if ( A[i][offset] != 0 ) {
            ri = i;
            break;
        }
    }

    if ( ri==null ) throw new Error("Matrix is singular");

    // Row interchange (if necessary)
    if ( ri != offset ) {
        let row;

        row = A[offset];
        A[offset] = A[ri];
        A[ri] = row;

        row = I[offset];
        I[offset] = I[ri];
        I[ri] = row;
    }

    // Normalize row to A = [ 0, ..., 0, 1, a, b, c, ... ]
    const scale = 1/A[offset][offset];
    for ( let i = 0; i < offset; i ++ ) {
        I[offset][i] *= scale;
    }
    for ( let i = offset; i < A.length; i ++ ) {
        A[offset][i] *= scale;
        I[offset][i] *= scale;
    }

    // Zero remaining rows
    for ( let i = 0; i < A.length; i ++ ) {
        if ( i == offset ) continue;
        const a = A[i][offset];
        for ( let j = 0; j < offset; j ++ ) {
            I[i][j] -= a*I[offset][j];
        }
        for ( let j = offset; j < A.length; j ++ ) {
            A[i][j] -= a*A[offset][j];
            I[i][j] -= a*I[offset][j];
        }
    }
}

function invert_matrix(_A) {
    const I = identity_matrix(_A.length);
    const A = copy_matrix(_A);
    for ( let i = 0; i < _A.length; i ++ ) _invert_matrix_subblock(A, I, i);
    return I;
}

function transpose_matrix(A) {
    const T = Array(A[0].length).fill().map(() => Array(A.length).fill(0));
    let i, j;
    for ( i = 0; i < T.length; i ++ ) {
        for ( j = 0; j < T[0].length; j ++ ) T[i][j] = A[j][i];
    }
    return T;
}
function copy_matrix(A) {
    return A.map((r) => [...r]);
}
function multiply_matrix(A, B) {
    const C = Array(A.length).fill().map(() => Array(B[0].length).fill(0));
    let i, j, k;
    for ( i = 0; i < A.length; i ++ ) {
        for ( k = 0; k < A[0].length; k ++ ) {
            for ( j = 0; j < B[0].length; j ++ ) {
                C[i][j] += A[i][k]*B[k][j];
            }
        }
    }

    return C;
}

// Helper functions
function clip(x, minValue, maxValue) {
    return Math.max(minValue, Math.min(maxValue, x));
}

function add_vectors(a, b) {
    return [ a[0] + b[0], a[1] + b[1], a[2] + b[2] ];
}

function subtract_vectors(a, b) {
    return [ a[0] - b[0], a[1] - b[1], a[2] - b[2] ];
}

function multiply_matrix_vector(R, v) {
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2]
    ];
}

// ---------------------
//  Rotation conversion
// ---------------------

function euler2rotmat(euler) {
    const sy = Math.sin(euler[0]),
          cy = Math.cos(euler[0]);
    const sp = Math.sin(euler[1]),
          cp = Math.cos(euler[1]);
    const sr = Math.sin(euler[2]),
          cr = Math.cos(euler[2]);

    return [
        [ cr * cy + sr * sp * sy, -cr * sy + sr * sp * cy, -sr * cp ],
        [ cp * sy, cp * cy, sp ],
        [ sr * cy - cr * sp * sy, -sr * sy - cr * sp * cy, cr * cp ]
    ];
}

function rotmat2euler(R) {
    return [
        Math.atan2(R[1][0], R[1][1]), // yaw
        Math.asin(clip(R[1][2], -1, 1)), // pitch
        Math.atan2(-R[0][2], R[2][2]) // roll
    ];
}

function euler2quat(euler) {
    const sy = Math.sin(euler[0] * 0.5),
          cy = Math.cos(euler[0] * 0.5);
    const sp = Math.sin(euler[1] * 0.5),
          cp = Math.cos(euler[1] * 0.5);
    const sr = Math.sin(euler[2] * 0.5),
          cr = Math.cos(euler[2] * 0.5);

    return [
        sy * sp * sr + cy * cp * cr,
        cy * sp * cr + sy * cp * sr,
        -sy * sp * cr + cy * cp * sr,
        cy * sp * sr - sy * cp * cr
    ];
}

function quat2redquat(q) {
    const sign = q[0] < 0 ? -1 : 1;
    return q.slice(1).map((v) => sign * v);
}

function redquat2quat(rq) {
    const sumSquares = rq[0] * rq[0] + rq[1] * rq[1] + rq[2] * rq[2];
    const re = Math.sqrt(clip(1 - sumSquares, 0, 1));
    return [ re, rq[0], rq[1], rq[2] ];
}

function redquat2euler(redquat) {
    return quat2euler(redquat2quat(redquat));
}

function quat2euler(quat) {
    const [ a, b, c, d ] = quat;
    return [
        Math.atan2(2 * (b * c - a * d), 2 * (a * a + c * c) - 1), // yaw
        Math.asin(clip(2 * (c * d + a * b), -1, 1)), // pitch
        Math.atan2(2 * (a * c - b * d), 2 * (a * a + d * d) - 1) // roll
    ];
}

function redquat2rotmat(redquat) {
    return quat2rotmat(redquat2quat(redquat));
}

function quat2rotmat(quat) {
    const [ a, b, c, d ] = quat;
    const asq = a * a,
          bsq = b * b,
          csq = c * c,
          dsq = d * d;
    return [
        [ asq + bsq - csq - dsq, 2 * (b * c + a * d), 2 * (b * d - a * c) ],
        [ 2 * (b * c - a * d), asq - bsq + csq - dsq, 2 * (c * d + a * b) ],
        [ 2 * (b * d + a * c), 2 * (c * d - a * b), asq - bsq - csq + dsq ]
    ];
}

function rotmat2quat(R) {
    const asq = (1 + R[0][0] + R[1][1] + R[2][2]) * 0.25;
    let b = Math.sqrt(clip((R[0][0] + 1) * 0.5 - asq, 0, 1));
    let c = Math.sqrt(clip((R[1][1] + 1) * 0.5 - asq, 0, 1));
    let d = Math.sqrt(clip((R[2][2] + 1) * 0.5 - asq, 0, 1));

    if (asq > 1e-12) {
        b *= R[1][2] - R[2][1] < 0 ? -1 : 1;
        c *= R[2][0] - R[0][2] < 0 ? -1 : 1;
        d *= R[0][1] - R[1][0] < 0 ? -1 : 1;
    } else if (b > 1e-12) {
        c *= R[0][1] + R[1][0] < 0 ? -1 : 1;
        d *= R[2][0] + R[0][2] < 0 ? -1 : 1;
    } else if (c > 1e-12) {
        d *= R[1][2] + R[2][1] < 0 ? -1 : 1;
    }

    return [ Math.sqrt(clip(asq, 0, 1)), b, c, d ];
}

function quat2rotvec(quat) {
    if (
        Math.abs(quat[0] - 1) < 1e-8 &&
        Math.abs(quat[1]) < 1e-8 &&
        Math.abs(quat[2]) < 1e-8 &&
        Math.abs(quat[3]) < 1e-8
    ) {
        return [ 0, 0, 0 ];
    }

    const qvec = quat.slice(1);
    const qvec_norm = Math.hypot(...qvec);
    let rotvec = [ 0, 0, 0 ];

    if (quat[0] >= 0) {
        rotvec = qvec.map((v) => (2 * Math.asin(qvec_norm) * v) / qvec_norm);
    } else {
        rotvec = qvec.map((v) => (2 * Math.asin(qvec_norm) * -v) / qvec_norm);
    }

    return rotvec;
}

function rotvec2quat(rotvec) {
    const rotvec_norm = Math.hypot(...rotvec);
    if (rotvec_norm < 1e-8) {
        return [ 1, 0, 0, 0 ];
    }

    const half_norm = rotvec_norm / 2;
    const sin_half_norm = Math.sin(half_norm);
    return [
        Math.cos(half_norm),
        (sin_half_norm * rotvec[0]) / rotvec_norm,
        (sin_half_norm * rotvec[1]) / rotvec_norm,
        (sin_half_norm * rotvec[2]) / rotvec_norm
    ];
}

// ------------------------------------------
//  Applications
// ------------------------------------------

function apply_rotmat(R, v) {
    return multiply_matrix_vector(R, v);
}

function apply_redquat(rq, v) {
    return apply_quat(redquat2quat(rq), v);
}

function apply_rotvec(rv, v) {
    return apply_quat(rotvec2quat(rv), v);
}

function apply_quat(q, v) {
    const a = -v[0] * q[1] - v[1] * q[2] - v[2] * q[3];
    const b = v[0] * q[0] + v[1] * q[3] - v[2] * q[2];
    const c = v[1] * q[0] + v[2] * q[1] - v[0] * q[3];
    const d = v[2] * q[0] + v[0] * q[2] - v[1] * q[1];

    return [
        q[0] * b - q[1] * a - q[2] * d + q[3] * c,
        q[0] * c - q[2] * a - q[3] * b + q[1] * d,
        q[0] * d - q[3] * a - q[1] * c + q[2] * b
    ];
}

function apply_iquat(q, v) {
    const a =  v[0] * q[1] + v[1] * q[2] + v[2] * q[3];
    const b =  v[0] * q[0] - v[1] * q[3] + v[2] * q[2];
    const c =  v[1] * q[0] - v[2] * q[1] + v[0] * q[3];
    const d =  v[2] * q[0] - v[0] * q[2] + v[1] * q[1];

    return [
        q[0] * b + q[1] * a + q[2] * d - q[3] * c,
        q[0] * c + q[2] * a + q[3] * b - q[1] * d,
        q[0] * d + q[3] * a + q[1] * c - q[2] * b
    ];
}

function apply_euler(euler, v) {
    return apply_quat(euler2quat(euler), v);
}

function apply_homo(H, v) {
    let w = 0;
    const vout = [];

    if (v.length < 4)
        v.push(1)

    // Compute the result of matrix multiplication and sum for the w component
    for (let i = 0; i < 3; i++) {
        vout[i] = H[i][0] * v[0] + H[i][1] * v[1] + H[i][2] * v[2] + H[i][3] * v[3];
        w += H[3][i] * v[i];
    }
    w += H[3][3] * v[3];

    // Divide by w component if it's not zero
    if (w !== 0) {
        for (let i = 0; i < 3; i++) {
            vout[i] /= w;
        }
    }

    return vout.slice(0, 3)
}

function apply_srq(srq, v) {
    const shifted_v = add_vectors(v, srq.slice(0, 3));
    return apply_redquat(srq.slice(3), shifted_v);
}

function apply_lrq(lrq, v) {
    const shifted_v = subtract_vectors(v, lrq.slice(0, 3));
    return apply_redquat(lrq.slice(3), shifted_v);
}

function apply_sre(sre, v) {
    const shifted_v = add_vectors(v, sre.slice(0, 3));
    return apply_euler(sre.slice(3), shifted_v);
}

function apply_lre(lre, v) {
    const shifted_v = subtract_vectors(v, lre.slice(0, 3));
    return apply_euler(lre.slice(3), shifted_v);
}

function apply_lrQ(lrQ, v) {
    const shifted_v = subtract_vectors(v, lrQ.slice(0, 3));
    return apply_quat(lrQ.slice(3), shifted_v);
}

function apply_lrrv(lrrv, v) {
    const shifted_v = subtract_vectors(v, lrrv.slice(0, 3));
    return apply_quat(rotvec2quat(lrrv.slice(3)), shifted_v);
}

// -------------
//  Inversions
// -------------

function invert_rotmat(R) {
    return transpose_matrix(R);
}

function invert_redquat(rq) {
    return rq.map((v) => -v);
}

function invert_rotvec(rv) {
    return rv.map((v) => -v);
}

function invert_quat(q) {
    return [ q[0], -q[1], -q[2], -q[3] ];
}

function invert_euler(euler) {
    return quat2euler(invert_quat(euler2quat(euler)));
}

function invert_homo(H) {
    return invert_matrix(H);
}

function invert_srq(srq) {
    const rqinv = invert_redquat(srq.slice(3));
    const shift = apply_redquat(srq.slice(3), srq.slice(0, 3)).map((v) => -v);
    return [ ...shift, ...rqinv ];
}

function invert_lrq(lrq) {
    const rqinv = invert_redquat(lrq.slice(3));
    const location = apply_redquat(lrq.slice(3), lrq.slice(0, 3)).map((v) => -v);
    return [ ...location, ...rqinv ];
}

function invert_lrQ(lrQ) {
    const rinv = invert_quat(lrQ.slice(3));
    const location = apply_quat(lrQ.slice(3), lrQ.slice(0, 3)).map((v) => -v);
    return [ ...location, ...rinv ];
}

function invert_lrrv(lrrv) {
    const rinv = invert_rotvec(lrrv.slice(3));
    const location = apply_rotvec(lrrv.slice(3), lrrv.slice(0, 3)).map((v) => -v);
    return [ ...location, ...rinv ];
}

function invert_sre(sre) {
    return homo2sre(invert_homo(sre2homo(sre)));
}

function invert_lre(lre) {
    return homo2lre(invert_homo(lre2homo(lre)));
}

// ------------
//  Euclidian
// ------------

function srq2homo(srq) {
    const shift = srq.slice(0, 3);
    const R = redquat2rotmat(srq.slice(3));

    const H = [
        [ ...R[0], 0 ],
        [ ...R[1], 0 ],
        [ ...R[2], 0 ],
        [ 0, 0, 0, 1 ]
    ];
    const shift_prime = apply_rotmat(R, shift);
    H[0][3] = shift_prime[0];
    H[1][3] = shift_prime[1];
    H[2][3] = shift_prime[2];

    return H;
}

function lrq2homo(lrq) {
    const shift = lrq.slice(0, 3).map((v) => -v);
    const R = redquat2rotmat(lrq.slice(3));

    const H = [
        [ ...R[0], 0 ],
        [ ...R[1], 0 ],
        [ ...R[2], 0 ],
        [ 0, 0, 0, 1 ]
    ];
    const shift_prime = apply_rotmat(R, shift);
    H[0][3] = shift_prime[0];
    H[1][3] = shift_prime[1];
    H[2][3] = shift_prime[2];

    return H;
}

function lrQ2homo(lrQ) {
    const shift = lrQ.slice(0, 3).map((v) => -v);
    const R = quat2rotmat(lrQ.slice(3));

    const H = [
        [ ...R[0], 0 ],
        [ ...R[1], 0 ],
        [ ...R[2], 0 ],
        [ 0, 0, 0, 1 ]
    ];
    const shift_prime = apply_rotmat(R, shift);
    H[0][3] = shift_prime[0];
    H[1][3] = shift_prime[1];
    H[2][3] = shift_prime[2];

    return H;
}

function sre2homo(sre) {
    const shift = sre.slice(0, 3);
    const R = euler2rotmat(sre.slice(3));

    const H = [
        [ ...R[0], 0 ],
        [ ...R[1], 0 ],
        [ ...R[2], 0 ],
        [ 0, 0, 0, 1 ]
    ];
    const shift_prime = apply_rotmat(R, shift);
    H[0][3] = shift_prime[0];
    H[1][3] = shift_prime[1];
    H[2][3] = shift_prime[2];

    return H;
}

function lre2homo(lre) {
    const shift = lre.slice(0, 3).map((v) => -v);
    const R = euler2rotmat(lre.slice(3));

    const H = [
        [ ...R[0], 0 ],
        [ ...R[1], 0 ],
        [ ...R[2], 0 ],
        [ 0, 0, 0, 1 ]
    ];
    const shift_prime = apply_rotmat(R, shift);
    H[0][3] = shift_prime[0];
    H[1][3] = shift_prime[1];
    H[2][3] = shift_prime[2];

    return H;
}

function homo2srq(H) {
    const R = [ H[0].slice(0, 3), H[1].slice(0, 3), H[2].slice(0, 3) ];
    const quat = rotmat2quat(R);
    const rq = quat.slice(1);
    const rq_inv = invert_redquat(rq);
    const shift_prime = [ H[0][3], H[1][3], H[2][3] ];
    const shift = apply_redquat(rq_inv, shift_prime);

    return [ ...shift, ...rq ];
}

function homo2lrq(H) {
    const R = [ H[0].slice(0, 3), H[1].slice(0, 3), H[2].slice(0, 3) ];
    const quat = rotmat2quat(R);
    const rq = quat.slice(1);
    const rq_inv = invert_redquat(rq);
    const shift_prime = [ H[0][3], H[1][3], H[2][3] ];
    const shift = apply_redquat(rq_inv, shift_prime);

    return shift.map((v) => -v).concat(rq);
}

function homo2lrQ(H) {
    const R = [ H[0].slice(0, 3), H[1].slice(0, 3), H[2].slice(0, 3) ];
    const quat = rotmat2quat(R);
    const rinv = invert_quat(quat);
    const shift_prime = [ H[0][3], H[1][3], H[2][3] ];
    const shift = apply_quat(rinv, shift_prime);

    return shift.map((v) => -v).concat(quat);
}

function homo2sre(H) {
    const R = [ H[0].slice(0, 3), H[1].slice(0, 3), H[2].slice(0, 3) ];
    const euler = rotmat2euler(R);
    const shift_prime = [ H[0][3], H[1][3], H[2][3] ];
    const shift = multiply_matrix_vector(transpose_matrix(R), shift_prime);

    return [ ...shift, ...euler ];
}

function homo2lre(H) {
    const R = [ H[0].slice(0, 3), H[1].slice(0, 3), H[2].slice(0, 3) ];
    const euler = rotmat2euler(R);
    const shift_prime = [ H[0][3], H[1][3], H[2][3] ];
    const shift = multiply_matrix_vector(transpose_matrix(R), shift_prime);

    return shift.map((v) => -v).concat(euler);
}

function lrq2lrQ(lrq) {
    const q = redquat2quat(lrq.slice(3));
    return [ ...lrq.slice(0, 3), ...q ];
}

function lrQ2lrq(lrQ) {
    const rq = quat2redquat(lrQ.slice(3));
    return [ ...lrQ.slice(0, 3), ...rq ];
}

function lrrv2lrQ(lrrv) {
    const q = rotvec2quat(lrrv.slice(3));
    return [ ...lrrv.slice(0, 3), ...q ];
}

function lrQ2lrrv(lrQ) {
    const rv = quat2rotvec(lrQ.slice(3));
    return [ ...lrQ.slice(0, 3), ...rv ];
}

function lre2lrQ(lre) {
    const q = euler2quat(lre.slice(3));
    return [ ...lre.slice(0, 3), ...q ];
}

function lrQ2lre(lrQ) {
    const euler = quat2euler(lrQ.slice(3));
    return [ ...lrQ.slice(0, 3), ...euler ];
}

// ------------------------------------------
//  Compositions
// ------------------------------------------

function compose_rotmat(R1, R2) {
    return multiply_matrix(R2, R1);
}

function compose_redquat(rq1, rq2) {
    return quat2redquat(
        compose_quat(redquat2quat(rq1), redquat2quat(rq2))
    );
}

function compose_rotvec(rv1, rv2) {
    return quat2rotvec(
        compose_quat(rotvec2quat(rv1), rotvec2quat(rv2))
    );
}

function compose_quat(q1, q2) {
    const [ a1, b1, c1, d1 ] = q1;
    const [ a2, b2, c2, d2 ] = q2;

    return [
        a1 * a2 - b1 * b2 - c1 * c2 - d1 * d2,
        a1 * b2 + b1 * a2 + c1 * d2 - d1 * c2,
        a1 * c2 + c1 * a2 + d1 * b2 - b1 * d2,
        a1 * d2 + d1 * a2 + b1 * c2 - c1 * b2
    ];
}

function compose_homo(H1, H2) {
    return multiply_matrix(H2, H1);
}

function compose_srq(srq1, srq2) {
    const rq1 = srq1.slice(3);
    const rq3 = compose_redquat(rq1, srq2.slice(3));
    const shift3 = add_vectors(
        srq1.slice(0, 3),
        apply_redquat(invert_redquat(rq1), srq2.slice(0, 3))
    );
    return [ ...shift3, ...rq3 ];
}

function compose_lrq(lrq1, lrq2) {
    const rq1 = lrq1.slice(3);
    const rq3 = compose_redquat(rq1, lrq2.slice(3));
    const location3 = add_vectors(
        lrq1.slice(0, 3),
        apply_redquat(invert_redquat(rq1), lrq2.slice(0, 3))
    );
    return [ ...location3, ...rq3 ];
}

function compose_lrrv(lrrv1, lrrv2) {
    const rv1 = lrrv1.slice(3);
    const rv3 = compose_rotvec(rv1, lrrv2.slice(3));
    const location3 = add_vectors(
        lrrv1.slice(0, 3),
        apply_rotvec(invert_rotvec(rv1), lrrv2.slice(0, 3))
    );
    return [ ...location3, ...rv3 ];
}

function compose_lrQ(lrQ1, lrQ2) {
    const q1 = lrQ1.slice(3);
    const q3 = compose_quat(q1, lrQ2.slice(3));
    const location3 = add_vectors(
        lrQ1.slice(0, 3),
        apply_iquat(q1/*inverse*/, lrQ2.slice(0, 3))
    );
    return [ ...location3, ...q3 ];
}

function compose_sre(sre1, sre2) {
    return homo2sre(compose_homo(sre2homo(sre1), sre2homo(sre2)));
}

function compose_lre(lre1, lre2) {
    return homo2lre(compose_homo(lre2homo(lre1), lre2homo(lre2)));
}

module.exports = {
    euler2rotmat,
    rotmat2euler,
    euler2quat,
    quat2redquat,
    redquat2quat,
    redquat2euler,
    quat2euler,
    redquat2rotmat,
    quat2rotmat,
    rotmat2quat,
    quat2rotvec,
    rotvec2quat,
    apply_rotmat,
    apply_redquat,
    apply_rotvec,
    apply_quat,
    apply_iquat,
    apply_euler,
    apply_homo,
    apply_srq,
    apply_lrq,
    apply_sre,
    apply_lre,
    apply_lrQ,
    apply_lrrv,
    invert_rotmat,
    invert_redquat,
    invert_rotvec,
    invert_quat,
    invert_euler,
    invert_homo,
    invert_srq,
    invert_lrq,
    invert_lrQ,
    invert_lrrv,
    invert_sre,
    invert_lre,
    srq2homo,
    lrq2homo,
    lrQ2homo,
    sre2homo,
    lre2homo,
    homo2srq,
    homo2lrq,
    homo2lrQ,
    homo2sre,
    homo2lre,
    lrq2lrQ,
    lrQ2lrq,
    lrrv2lrQ,
    lrQ2lrrv,
    lre2lrQ,
    lrQ2lre,
    compose_rotmat,
    compose_redquat,
    compose_rotvec,
    compose_quat,
    compose_homo,
    compose_srq,
    compose_lrq,
    compose_lrrv,
    compose_lrQ,
    compose_sre,
    compose_lre
};
