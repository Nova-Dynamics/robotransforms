
const t = require("./euclidean.js");

// ---------------------
//  Rotation conversion
// ---------------------


function d_euler2quat_d_euler(euler) {
    const sy = Math.sin(euler[0] * 0.5),
          cy = Math.cos(euler[0] * 0.5);
    const sp = Math.sin(euler[1] * 0.5),
          cp = Math.cos(euler[1] * 0.5);
    const sr = Math.sin(euler[2] * 0.5),
          cr = Math.cos(euler[2] * 0.5);

    const dsy = 0.5*cy,
          dcy = -0.5*sy;
    const dsp = 0.5*cp;
    dcp = -0.5*sp;
    const dsr = 0.5*cr;
    dcr = -0.5*sr;

    return [
        [  dsy * sp * sr + dcy * cp * cr,  sy * dsp * sr + cy * dcp * cr,  sy * sp * dsr + cy * cp * dcr ],
        [  dcy * sp * cr + dsy * cp * sr,  cy * dsp * cr + sy * dcp * sr,  cy * sp * dcr + sy * cp * dsr ],
        [ -dsy * sp * cr + dcy * cp * sr, -sy * dsp * cr + cy * dcp * sr, -sy * sp * dcr + cy * cp * dsr ],
        [  dcy * sp * sr - dsy * cp * cr,  cy * dsp * sr - sy * dcp * cr,  cy * sp * dsr - sy * cp * dcr ]
    ];
}

function d_quat2redquat_d_q(q) {
    const s = q[0] < 0 ? -1 : 1;
    return [
        [ 0,  s,  0,  0 ],
        [ 0,  0,  s,  0 ],
        [ 0,  0,  0,  s ]
    ];
}

/**
 *   This is a special case for generaing
 *   structure constants, since we get a
 *   really nice reduction in the form when
 *   we set rq to zero. Though, technically,
 *   you can get the same result with:
 *        mult( d_compose_quat_d_q2(q, [1,0,0,0]), d_redquat2quat_d_rq([0,0,0]) )
 *
 *   ∂ compose_quat(q, redquat2quat(rq)) |
 *   ----------------------------------- |
 *                ∂ rq                   | rq=0
 *
 *  returns 4x3
 */
function d_quat_encaps_redquat_d_redquat_at_zero(q) {
    const [ a, b, c, d ] = q;
    return [
        [ -b, -c, -d ],
        [  a, -d,  c ],
        [  d,  a, -b ],
        [ -c,  b,  a ]
    ];
};

function d_redquat2quat_d_redquat(rq) {
    const [ b, c, d ] = rq;
    const a = Math.sqrt(Math.max(1 - b*b - c*c - d*d), 0);
    const nia = -1/a
    return [
        [ b*nia,  c*nia,  d*nia ],
        [     1,      0,      0 ],
        [     0,      1,      0 ],
        [     0,      0,      1 ]
    ];
}

function d_redquat2euler_d_redquat(redquat) {
    throw new Error("todo");
}

function d_quat2euler_d_q(quat) {
    throw new Error("todo");
}

function d_quat2rotvec_d_q(quat) {
    /**
     *  Recall that quat = [ cos(|rv|/2), 0.5*sinc(|rv|/2) * r̅v̅ ]^T,
     *  but ain't nobody got time to take a derivative of a sinc function with a chain and product rule!
     *  Anyway, you can just abuse the fact that the cos derivative is zero at rv=0, and that
     *  the derivate of sinc is defined to be zero at rv=0 (which nukes the product rule), then you
     *  recall that sinc is also zero at zero, and the whole thing MASSIVELY simplifies to:
     *
     *   ∂ rotvec2quat(rq)  |         [ 0   0    0    0  ]
     *   ------------------ |       = [ 0  1/2   0    0  ]
     *         ∂ rv         | rv=0    [ 0   0   1/2   0  ]
     *                                [ 0   0    0   1/2 ]
     *
     *  In our case, we don't actually need the general derivate anyway, so I make a special function
     *  below that uses this
     */
    if (
        Math.abs(quat[0] - 1) < 1e-8 &&
        Math.abs(quat[1]) < 1e-8 &&
        Math.abs(quat[2]) < 1e-8 &&
        Math.abs(quat[3]) < 1e-8
    ) {
        return [
            [ 0,   0,   0,   0 ],
            [ 0, 0.5,   0,   0 ],
            [ 0,   0, 0.5,   0 ],
            [ 0,   0,   0, 0.5 ]
        ];
    }
    throw new Error("todo");
}

/**
 *   This is a special case for generaing
 *   structure constants, since we get a
 *   really nice reduction in the form when
 *   we set rv to zero. Additionally, the
 *   internal guts are HORRIFYINGLY ugly
 *   in the general case, so we don't even
 *   need them, so ya, just use this.
 *   (see comment on d_rotvec2quat_d_rv)
 *
 *
 *   ∂ compose_quat(q, rotvec2quat(rv))  |
 *   ----------------------------------- |
 *                ∂ rv                   | rv=0
 *
 *  As a fun side note, this is exactly equal
 *  to d_quat_encaps_rq_d_rq_at_zero(q) / 2,
 *  which shouldn't be surprising, because the
 *  vector part of q looks like sin(rv/2) ~= rv/2
 *  near zero, which means the "redquat" equivalent
 *  of this rotvec is just rv/2 near zero.
 *
 *  returns 4x3
 */
function d_quat_encaps_rotvec_d_rotvec_at_zero(q, _) {
    const ha = 0.5*q[0];
    const hb = 0.5*q[1];
    const hc = 0.5*q[2];
    const hd = 0.5*q[3];
    return [
        [ -hb, -hc, -hd ],
        [  ha, -hd,  hc ],
        [  hd,  ha, -hb ],
        [ -hc,  hb,  ha ]
    ];
};

function d_rotvec2quat_d_rotvec(rotvec) {
    throw new Error("todo");
}

// ------------------------------------------
//  Applications
// ------------------------------------------

function d_apply_redquat_d_v(rq, v) {
    return t.redquat2rotmat(rq);
}
function d_apply_redquat_d_redquat(rq, v) {
    throw new Error("todo");
}

function d_apply_rotvec_d_v(rv, v) {
    return t.rotvec2rotmat(rv);
}
function d_apply_rotvec_d_rotvec(rv, v) {
    throw new Error("todo");
}

function d_apply_quat_d_v(q, v) {
    /**
     *                          [    |            |          |    ]
     *                          [    |            |          |    ]
     *     ∂ apply_quat(q, v)   [     ∂ v        ∂ v        ∂ v   ]
     *   ----------------- =    [ R @ ----,  R @ ----,  R @ ----, ]
     *        ∂ v               [     ∂ x        ∂ y        ∂ z   ]
     *                          [    |            |          |    ]
     *                          [    |            |          |    ]
     *
     *                          [     [ 1 ]       [ 0 ]       [ 0 ] ]
     *                        = [ R @ [ 0 ],  R @ [ 1 ],  R @ [ 0 ] ]
     *                          [     [ 0 ]       [ 0 ]       [ 1 ] ]
     *
     *                        = R
     */
    return t.quat2rotmat(q);
}
function d_apply_quat_d_q(rv, v) {
    /**
     *                        [         |                      |                      |                      |             ]
     *                        [         |                      |                      |                      |             ]
     *   ∂ apply_quat(q, v)   [  ∂ quat2rotmat(q)       ∂ quat2rotmat(q)       ∂ quat2rotmat(q)       ∂ quat2rotmat(q)     ]
     *   ------------------ = [  ---------------- @ v,  ---------------- @ v,  ---------------- @ v,  ---------------- @ v ]
     *         ∂ q            [        ∂ a                    ∂ b                    ∂ c                    ∂ d            ]
     *                        [         |                      |                      |                      |             ]
     *                        [         |                      |                      |                      |             ]
     *
     *  ∂ quat2rotmat(q)     [  a   d  -c ]
     *  ---------------- = 2*[ -d   a   b ]
     *        ∂ a            [  c  -b   a ]
     *
     *  ∂ quat2rotmat(q)     [  b   c   d ]
     *  ---------------- = 2*[  c  -b   a ]
     *        ∂ b            [  d  -a  -b ]
     *
     *  ∂ quat2rotmat(q)     [ -c   b  -a ]
     *  ---------------- = 2*[  b   c   d ]
     *        ∂ c            [  a   d  -c ]
     *
     *  ∂ quat2rotmat(q)     [ -d   a   b ]
     *  ---------------- = 2*[ -a  -d   c ]
     *        ∂ d            [  b   c   d ]
     */
    const ta = 2*q[0];
    const tb = 2*q[1];
    const tc = 2*q[2];
    const td = 2*q[3];

    const x = v[0];
    const y = v[1];
    const z = v[2];
    return [
        [  ta*x  +td*y  -tc*z,   tb*x  +tc*y  +td*z,  -tc*x  +tb*y  -ta*z,  -td*x  +ta*y  +tb*z ],
        [ -td*x  +ta*y  +tb*z,   tc*x  -tb*y  +ta*z,   tb*x  +tc*y  +td*z,  -ta*x  -td*y  +tc*z ],
        [  tc*x  -tb*y  +ta*z,   td*x  -ta*y  -tb*z,   ta*x  +td*y  -tc*z,   tb*x  +tc*y  +td*z ]
    ];
}


function d_apply_euler_d_v(euler, v) {
    return t.euler2rotmat(euler);
}
function d_apply_euler_d_euler(euler, v) {
    throw new Error("todo");
}

function d_apply_lrq_d_v(lrq, v) {
    throw new Error("todo");
}
function d_apply_lrq_d_lrq(lrq, v) {
    throw new Error("todo");
}

function d_apply_lrq_d_v(lrq, v) {
    throw new Error("todo");
}
function d_apply_lrq_d_lrq(lrq, v) {
    throw new Error("todo");
}

function d_apply_lrQ_d_v(lrQ, v) {
    return t.quat2rotmat(lrQ.slice(3));
}
function d_apply_lrQ_d_lrQ(lrQ, v) {
    const M = d_apply_quat_d_q(lrQ.slice(3));
    return [
        [ 1, 0, 0,   M[0][0], M[0][1], M[0][2], M[0][3] ],
        [ 0, 1, 0,   M[1][0], M[1][1], M[1][2], M[1][3] ],
        [ 0, 0, 1,   M[2][0], M[2][1], M[2][2], M[2][3] ]
    ]
}

function d_apply_lrrv_d_v(lrrv, v) {
    throw new Error("todo");
}
function d_apply_lrrv_d_lrrv(lrrv, v) {
    throw new Error("todo");
}

// -------------
//  Inversions
// -------------

function d_invert_redquat_d_redquat(rq) {
    throw new Error("todo");
}

function d_invert_rotvec_d_rotvec(rv) {
    throw new Error("todo");
}

function d_invert_quat_d_q(q) {
    return [
        [ 1,  0,  0,  0 ],
        [ 0, -1,  0,  0 ],
        [ 0,  0  -1,  0 ],
        [ 0,  0,  0, -1 ]
    ];
}

function d_invert_euler_d_euler(euler) {
    throw new Error("todo");
}

function d_invert_lrq_d_lrq(lrq) {
    throw new Error("todo");
}

function d_invert_lrQ_d_lrQ(lrQ) {
    /**
     *    invert_lrQ(lrQ) = [ -apply_quat(q, v), invert_quat(q) ]^T
     *
     *   ∂ lrQ^-1    [  -∂apply_quat(q,v) / ∂v    -∂apply_quat(q,v) / ∂q ]
     *   --------  = [                                                   ]
     *    ∂ lrQ      [   ∂invert_quat(q) / ∂v      ∂invert_quat(q) / ∂q  ]
     *
     *  returns 7x7
     */
    const x = lrQ[0];
    const y = lrQ[1];
    const z = lrQ[2];
    const nta = -2*lrQ[3];
    const ntb = -2*lrQ[4];
    const ntc = -2*lrQ[5];
    const ntd = -2*lrQ[6];
    const R = t.quat2rotmat(lrQ.slice(3));
    return [
        [ -R[0][0], -R[0][1], -R[0][2],   nta*x  +ntd*y  -ntc*z,   ntb*x  +ntc*y  +ntd*z,  -ntc*x  +ntb*y  -nta*z,  -ntd*x  +nta*y  +ntb*z ],
        [ -R[1][0], -R[1][1], -R[1][2],  -ntd*x  +nta*y  +ntb*z,   ntc*x  -ntb*y  +nta*z,   ntb*x  +ntc*y  +ntd*z,  -nta*x  -ntd*y  +ntc*z ],
        [ -R[2][0], -R[2][1], -R[2][2],   ntc*x  -ntb*y  +nta*z,   ntd*x  -nta*y  -ntb*z,   nta*x  +ntd*y  -ntc*z,   ntb*x  +ntc*y  +ntd*z ],
        [        0,        0,        0,                       1,                       0,                       0,                       0 ],
        [        0,        0,        0,                       0,                      -1,                       0,                       0 ],
        [        0,        0,        0,                       0,                       0,                      -1,                       0 ],
        [        0,        0,        0,                       0,                       0,                       0,                      -1 ]
    ]
}

function d_invert_lrrv_d_lrrv(lrrv) {
    throw new Error("todo");
}

function d_invert_lre_d_lre(lre) {
    throw new Error("todo");
}

// ------------
//  Euclidian
// ------------


function d_lrq2lrQ_d_lrq(lrq) {
    const [ b, c, d ] = lrq.slice(3);
    const a = Math.sqrt(Math.max(1 - b*b - c*c - d*d), 0);
    const nia = -1/a
    return [
        [ 1, 0, 0,      0,      0,      0 ],
        [ 0, 1, 0,      0,      0,      0 ],
        [ 0, 0, 1,      0,      0,      0 ],

        [ 0, 0, 0,  b*nia,  c*nia,  d*nia ],
        [ 0, 0, 0,      1,      0,      0 ],
        [ 0, 0, 0,      0,      1,      0 ],
        [ 0, 0, 0,      0,      0,      1 ]
    ];
}

function d_lrQ2lrq_d_lrQ(lrQ) {
    const s = lrQ[3] < 0 ? -1 : 1;
    return [
        [ 1, 0, 0,    0,  0,  0,  0 ],
        [ 0, 1, 0,    0,  0,  0,  0 ],
        [ 0, 0, 1,    0,  0,  0,  0 ],

        [ 0, 0, 0,    0,  s,  0,  0 ],
        [ 0, 0, 0,    0,  0,  s,  0 ],
        [ 0, 0, 0,    0,  0,  0,  s ]
    ];
}

function d_lrrv2lrQ_d_lrrv(lrrv) {
    throw new Error("todo");
}

function d_lrQ2lrrv_d_lrQ(lrQ) {
    throw new Error("todo");
}

function d_lre2lrQ_d_lre(lre) {
    throw new Error("todo");
}

function d_lrQ2lre_d_lrQ(lrQ) {
    throw new Error("todo");
}

// ------------------------------------------
//  Compositions
// ------------------------------------------

function d_compose_redquat_d_redquat1(rq1, rq2) {
    throw new Error("todo");
}
function d_compose_redquat_d_redquat2(rq1, rq2) {
    throw new Error("todo");
}

function d_compose_rotvec_d_rotvec1(rv1, rv2) {
    throw new Error("todo");
}
function d_compose_rotvec_d_rotvec2(rv1, rv2) {
    throw new Error("todo");
}

function d_compose_quat_d_q1(_, q2) {
    /**
     *   ∂ compose_quat(q1, q2)
     *   ----------------------
     *           ∂ q1
     *
     *  returns 4x4
     */
    const [ a, b, c, d ] = q2;
    return [
        [ a, -b, -c, -d ],
        [ b,  a,  d, -c ],
        [ c, -d,  a,  b ],
        [ d,  c, -b,  a ]
    ];
};

function d_compose_quat_d_q2(q1, _) {
    /**
     *   ∂ compose_quat(q1, q2)
     *   ----------------------
     *           ∂ q2
     *
     *  returns 4x4
     */
    const [ a, b, c, d ] = q1;
    return [
        [ a, -b, -c, -d ],
        [ b,  a, -d,  c ],
        [ c,  d,  a, -b ],
        [ d, -c,  b,  a ]
    ];
};

function d_compose_lrq_d_lrq1(lrq1, lrq2) {
    throw new Error("todo");
}
function d_compose_lrq_d_lrq2(lrq1, lrq2) {
    throw new Error("todo");
}

function d_compose_lrrv_d_lrrv1(lrrv1, lrrv2) {
    throw new Error("todo");
}
function d_compose_lrrv_d_lrrv2(lrrv1, lrrv2) {
    throw new Error("todo");
}

function d_compose_lrQ_d_lrQ1(lrQ1, lrQ2) {
    /**
     *    compose_lrQ(lrQ1, lrQ2) = lrQ3 = [ v3, q3 ]^T = [ v1 + apply_quat(q1, v2), compose_quat(q1, q2) ]^T
     *
     *   ∂ lrQ3    [  ∂v3 / ∂v1    ∂v3 / ∂q1 ]
     *   ------- = [                         ]
     *   ∂ lrQ1    [  ∂q3 / ∂v1    ∂q3 / ∂q1 ]
     *
     *             [   1_3x3     ∂ apply_quat(q1, v2) / ∂q1  ]
     *           = [                                         ]
     *             [   0_4x3    ∂ compose_quat(q1, q2) / ∂q1 ]
     *
     *  returns 7x7
     */
    const ta1 = 2*lrQ1[3];
    const tb1 = 2*lrQ1[4];
    const tc1 = 2*lrQ1[5];
    const td1 = 2*lrQ1[6];
    const [ x2, y2, z2, a2, b2, c2, d2 ] = lrQ2;
    return [
        [ 1, 0, 0,   ta1*x2  +td1*y2  -tc1*z2,   tb1*x2  +tc1*y2  +td1*z2,  -tc1*x2  +tb1*y2  -ta1*z2,  -td1*x2  +ta1*y2  +tb1*z2 ],
        [ 0, 1, 0,  -td1*x2  +ta1*y2  +tb1*z2,   tc1*x2  -tb1*y2  +ta1*z2,   tb1*x2  +tc1*y2  +td1*z2,  -ta1*x2  -td1*y2  +tc1*z2 ],
        [ 0, 0, 1,   tc1*x2  -tb1*y2  +ta1*z2,   td1*x2  -ta1*y2  -tb1*z2,   ta1*x2  +td1*y2  -tc1*z2,   tb1*x2  +tc1*y2  +td1*z2 ],

        [ 0, 0, 0,                         a2,                        -b2,                        -c2,                        -d2 ],
        [ 0, 0, 0,                         b2,                         a2,                         d2,                        -c2 ],
        [ 0, 0, 0,                         c2,                        -d2,                         a2,                         b2 ],
        [ 0, 0, 0,                         d2,                         c2,                        -b2,                         a2 ]
    ]
};

function d_compose_lrQ_d_lrQ2(lrQ1, _) {
    /**
     *    compose_lrQ(lrQ1, lrQ2) = lrQ3 = [ v3, q3 ]^T = [ v1 + apply_quat(q1, v2), compose_quat(q1, q2) ]^T
     *
     *   ∂ lrQ3    [  ∂v3 / ∂v2    ∂v3 / ∂q2 ]
     *   ------- = [                         ]
     *   ∂ lrQ2    [  ∂q3 / ∂v2    ∂q3 / ∂q2 ]
     *
     *             [   ∂ apply_quat(q1, v2) / ∂v2                0_3x4           ]
     *           = [                                                             ]
     *             [           0_4x3                ∂ compose_quat(q1, q2) / ∂q2 ]
     *
     *  returns 7x7
     */
    const R = t.quat2rotmat(lrQ1.slice(3));
    const [ a1, b1, c1, d1 ] = lrQ1.slice(3);
    return [
        [ R[0][0], R[0][1], R[0][2],     0,   0,   0,   0 ],
        [ R[1][0], R[1][1], R[1][2],     0,   0,   0,   0 ],
        [ R[2][0], R[2][1], R[2][2],     0,   0,   0,   0 ],

        [       0,       0,       0,    a1, -b1, -c1, -d1 ],
        [       0,       0,       0,    b1,  a1, -d1,  c1 ],
        [       0,       0,       0,    c1,  d1,  a1, -b1 ],
        [       0,       0,       0,    d1, -c1,  b1,  a1 ]
    ];
};

function d_compose_lre_d_lre1(lre1, lre2) {
    throw new Error("todo");
}
function d_compose_lre_d_lre2(lre1, lre2) {
    throw new Error("todo");
}

function d_lrQ_encaps_lrrq_d_lrrq_at_zero(lrQ) {
    /**
     *   ∂ compose_lrQ(lrQ, lrrq2lrQ(lrrq))  |
     *   ----------------------------------- |
     *                ∂ lrrq                 | lrrq=0
     *
     *      ∂ compose_lrQ(lrQ, lrQ2)  |                             ∂ lrrq2lrQ(lrrq))  |
     *    = ------------------------- |                          @ ------------------- |
     *              ∂ lrQ2            | lrQ2=[0,0,0, 1,0,0,0]^T         ∂ lrrq         | lrrq=0
     *
     *      [  ∂apply_quat(q1, v2) / ∂v2 |v2=0                0_3x3                      ]   [        ∂v / ∂v |v=0                     ∂v / ∂rq| rq=0          ]
     *    = [                                                                            ] @ [                                                                 ]
     *      [              0_4x4              ∂compose_quat(q1, q2)/∂q2 | q2=[1,0,0,0]   ]   [ ∂redquat2quat(rq) / ∂v | v=0     ∂redquat2quat(rq) / ∂rq | rq=0 ]
     *
     *      [  ∂apply_quat(q1, v2) / ∂v2 |v2=0                0_3x3                      ]   [    1_3x3                  0_3x3              ]
     *    = [                                                                            ] @ [                                              ]
     *      [              0_4x4              ∂compose_quat(q1, q2)/∂q2 | q2=[1,0,0,0]   ]   [    0_4x3     ∂redquat2quat(rq) / ∂rq | rq=0  ]
     *
     *      [  ∂apply_quat(q1, v2) / ∂v2 |v2=0                                      0_3x3                                      ]
     *    = [                                                                                                                  ]
     *      [              0_4x4                   ∂compose_quat(q1, q2)/∂q2 | q2=[1,0,0,0] @  ∂redquat2quat(rq) / ∂rq | rq=0  ]
     *
     *      [  ∂apply_quat(q1, v2) / ∂v2 |v2=0                  0_3x3                 ]
     *    = [                                                                         ]
     *      [              0_4x4                   d_quat_encaps_rq_d_rq_at_zero(q1)  ]
     */
    const R = t.quat2rotmat(lrQ.slice(3));
    const [ a, b, c, d ] = lrQ.slice(3);
    return [
        [ R[0][0], R[0][1], R[0][2],    0,  0,  0 ],
        [ R[1][0], R[1][1], R[1][2],    0,  0,  0 ],
        [ R[2][0], R[2][1], R[2][2],    0,  0,  0 ],

        [       0,       0,       0,   -b, -c, -d ],
        [       0,       0,       0,    a, -d,  c ],
        [       0,       0,       0,    d,  a, -b ],
        [       0,       0,       0,   -c,  b,  a ]
    ];
};

function d_lrQ_encaps_lrrv_d_lrrv_at_zero(lrQ) {
    /**
     *   ∂ compose_lrQ(lrQ, lrrv2lrQ(lrrv))  |
     *   ----------------------------------- |
     *                ∂ lrrv                 | lrrv=0
     *
     *      ∂ compose_lrQ(lrQ, lrQ2)  |                             ∂ lrrv2lrQ(lrrv))  |
     *    = ------------------------- |                          @ ------------------- |
     *              ∂ lrQ2            | lrQ2=[0,0,0, 1,0,0,0]^T         ∂ lrrv         | lrrv=0
     *
     *      [  ∂apply_quat(q1, v2) / ∂v2 |v2=0                0_3x3                      ]   [        ∂v / ∂v |v=0                     ∂v / ∂rv| rv=0        ]
     *    = [                                                                            ] @ [                                                               ]
     *      [              0_4x4              ∂compose_quat(q1, q2)/∂q2 | q2=[1,0,0,0]   ]   [ ∂rotvec2quat(rv) / ∂v | v=0     ∂rotvec2quat(rv) / ∂rv | rv=0 ]
     *
     *      [  ∂apply_quat(q1, v2) / ∂v2 |v2=0                0_3x3                      ]   [    1_3x3                  0_3x3             ]
     *    = [                                                                            ] @ [                                             ]
     *      [              0_4x4              ∂compose_quat(q1, q2)/∂q2 | q2=[1,0,0,0]   ]   [    0_4x3     ∂rotvec2quat(rv) / ∂rv | rv=0  ]
     *
     *      [  ∂apply_quat(q1, v2) / ∂v2 |v2=0                                      0_3x3                                     ]
     *    = [                                                                                                                 ]
     *      [              0_4x4                   ∂compose_quat(q1, q2)/∂q2 | q2=[1,0,0,0] @  ∂rotvec2quat(rv) / ∂rv | rv=0  ]
     *
     *      [  ∂apply_quat(q1, v2) / ∂v2 |v2=0                  0_3x3                 ]
     *    = [                                                                         ]
     *      [              0_4x4                   d_quat_encaps_rv_d_rv_at_zero(q1)  ]
     */
    const R = t.quat2rotmat(lrQ.slice(3));
    const ha = 0.5*q[0];
    const hb = 0.5*q[1];
    const hc = 0.5*q[2];
    const hd = 0.5*q[3];
    return [
        [ R[0][0], R[0][1], R[0][2],     0,   0,   0 ],
        [ R[1][0], R[1][1], R[1][2],     0,   0,   0 ],
        [ R[2][0], R[2][1], R[2][2],     0,   0,   0 ],

        [       0,       0,       0,   -hb, -hc, -hd ],
        [       0,       0,       0,    ha, -hd,  hc ],
        [       0,       0,       0,    hd,  ha, -hb ],
        [       0,       0,       0,   -hc,  hb,  ha ]
    ];
};

module.exports = {
    d_euler2quat_d_euler,
    d_quat2redquat_d_q,
    d_quat_encaps_redquat_d_redquat_at_zero,
    d_redquat2quat_d_redquat,
    d_redquat2euler_d_redquat,
    d_quat2euler_d_q,
    d_quat2rotvec_d_q,
    d_quat_encaps_rotvec_d_rotvec_at_zero,
    d_rotvec2quat_d_rotvec,
    d_apply_redquat_d_v,
    d_apply_redquat_d_redquat,
    d_apply_rotvec_d_v,
    d_apply_rotvec_d_rotvec,
    d_apply_quat_d_v,
    d_apply_quat_d_q,
    d_apply_euler_d_v,
    d_apply_euler_d_euler,
    d_apply_lrq_d_v,
    d_apply_lrq_d_lrq,
    d_apply_lrq_d_v,
    d_apply_lrq_d_lrq,
    d_apply_lrQ_d_v,
    d_apply_lrQ_d_lrQ,
    d_apply_lrrv_d_v,
    d_apply_lrrv_d_lrrv,
    d_invert_redquat_d_redquat,
    d_invert_rotvec_d_rotvec,
    d_invert_quat_d_q,
    d_invert_euler_d_euler,
    d_invert_lrq_d_lrq,
    d_invert_lrQ_d_lrQ,
    d_invert_lrrv_d_lrrv,
    d_invert_lre_d_lre,
    d_lrq2lrQ_d_lrq,
    d_lrQ2lrq_d_lrQ,
    d_lrrv2lrQ_d_lrrv,
    d_lrQ2lrrv_d_lrQ,
    d_lre2lrQ_d_lre,
    d_lrQ2lre_d_lrQ,
    d_compose_redquat_d_redquat1,
    d_compose_redquat_d_redquat2,
    d_compose_rotvec_d_rotvec1,
    d_compose_rotvec_d_rotvec2,
    d_compose_quat_d_q1,
    d_compose_quat_d_q2,
    d_compose_lrq_d_lrq1,
    d_compose_lrq_d_lrq2,
    d_compose_lrrv_d_lrrv1,
    d_compose_lrrv_d_lrrv2,
    d_compose_lrQ_d_lrQ1,
    d_compose_lrQ_d_lrQ2,
    d_compose_lre_d_lre1,
    d_compose_lre_d_lre2,
    d_lrQ_encaps_lrrq_d_lrrq_at_zero,
    d_lrQ_encaps_lrrv_d_lrrv_at_zero
};
