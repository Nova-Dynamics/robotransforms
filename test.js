const t = require("./lib/euclidean.js");
const td = require("./lib/d_euclidean.js");

function rand(half_width=1, center=0){
    return half_width*(2*Math.random()-1) + center;
}

function rand_redquat() {

    while ( true ) {
        const b = rand(1);
        const c = rand(1);
        const d = rand(1);

        if ( b*b + c*c + d*d <= 1 ) return [ b, c, d ];
    }
}

function rand_quat() {
    const [ b, c, d ] = rand_redquat();
    const a = (Math.random()>0.5?1:-1)*Math.sqrt(Math.max(0, 1 - b*b - c*c - d*d));
    return [ a, b, c, d ];
}

function rand_lrq() {
    return [
        rand(10),
        rand(10),
        rand(10),
        ...rand_redquat()
    ]
}

function rand_lrQ() {
    return [
        rand(10),
        rand(10),
        rand(10),
        ...rand_quat()
    ]
}



function nearly_equal(a, b, tol=1e-2) {
    if ( a==0 && b==0 ) return true;

    if ( a == 0 ) return Math.abs(b) < tol;
    if ( b == 0 ) return Math.abs(a) < tol;

    return Math.abs(a-b)/Math.max(Math.abs(a), Math.abs(b)) < tol;
}

const dx = 1e-8;

const N = 1000;


console.log("apply_iquat")
for ( let idx = 0; idx < N; idx ++ ) {
    const q = rand_quat();
    const v = [ rand(10), rand(10), rand(10) ];

    const left = t.apply_iquat(q, v);
    const right = t.apply_quat(t.invert_quat(q), v);

    for ( let i = 0; i < 3; i ++ ) {
        if ( !nearly_equal(left[i], right[i]) ) console.log("ACK");
    }
}


let label;
let diff;
let func;
let D_DP;
let D_NUMERATOR;
let anybad;


/**
 *  ------------------------------------------------------------------
 */
label = "euler2quat";
diff = td.d_euler2quat_d_euler;
func = t.euler2quat;
D_DP = 3;
D_NUMERATOR = 4;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const input = t.quat2euler(rand_quat());
    const OUT = diff(input);
    const FX = func(input);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...input];
            step[j] += dx;

            const fxpdx = func(step)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");



/**
 *  ------------------------------------------------------------------
 */
label = "quat2redquat";
diff = td.d_quat2redquat_d_q
func = t.quat2redquat;
D_DP = 4;
D_NUMERATOR = 3;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const input = rand_quat();
    const OUT = diff(input);
    const FX = func(input);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...input];
            step[j] += dx;

            const fxpdx = func(step)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_quat_encaps_redquat_d_redquat_at_zero";
diff = td.d_quat_encaps_redquat_d_redquat_at_zero
func = (q, rq) => t.compose_quat(q, t.redquat2quat(rq));
D_DP = 3;
D_NUMERATOR = 4;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_quat();
    const inputb = [ 0, 0, 0 ];
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputb];
            step[j] += dx;

            const fxpdx = func(inputa, step)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");



/**
 *  ------------------------------------------------------------------
 */
label = "d_redquat2quat_d_redquat";
diff = td.d_redquat2quat_d_redquat;
func = t.redquat2quat
D_DP = 3;
D_NUMERATOR = 4;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_redquat();
    // const inputb = [ 0, 0, 0 ];
    const OUT = diff(inputa);//, inputb);
    const FX = func(inputa);//, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            const fxpdx = func(/*inputa, */step)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


//  Has bad differential qualities -- cannot test
// /**
//  *  ------------------------------------------------------------------
//  */
// label = "d_quat_encaps_rotvec_d_rotvec_at_zero";
// diff = td.d_quat_encaps_rotvec_d_rotvec_at_zero;
// func = (q, rv) => t.compose_quat(q, t.rotvec2quat(rv));
// D_DP = 3;
// D_NUMERATOR = 4;
//
//
// console.log(label);
// anybad = false;
// for ( let idx = 0; idx < N; idx ++ ) {
//
//     const inputa = rand_quat();
//     const inputb = [ 0, 0, 0 ];
//     const OUT = diff(inputa, inputb);
//     const FX = func(inputa, inputb);
//     const DFDX = [];
//     const bad = [];
//     for ( let i = 0; i < D_NUMERATOR; i ++ ) {
//         DFDX.push([]);
//         const fx = FX[i];
//         for ( let j = 0; j < D_DP; j ++ ) {
//             // const step = [...inputa];
//             const step = [...inputb];
//             step[j] += dx;
//
//             const fxpdx = func(inputa, step)[i];
//
//             const dfdx = (fxpdx - fx) / dx;
//             DFDX[i].push(dfdx);
//
//             if ( !nearly_equal(dfdx, OUT[i][j]) ) {
//                 bad.push([ i, j, dfdx ]);
//             }
//         }
//     }
//
//
//     if ( bad.length ) {
//         anybad = true;
//         console.log(`Failed ${label}`);
//         console.log("input", FX);
//         console.log("diff");
//         console.table(OUT);
//         console.log("ndiff");
//         console.table(DFDX);
//         console.log("at");
//         for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
//     }
// }
// if ( !anybad ) console.log("    Good");

/**
 *  ------------------------------------------------------------------
 */
label = "d_apply_quat_d_v";
diff = td.d_apply_quat_d_v;
func = t.apply_quat
D_DP = 3;
D_NUMERATOR = 3;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_quat();
    const inputb = [ rand(10), rand(10), rand(10) ];
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            // const step = [...inputa];
            const step = [...inputb];
            step[j] += dx;

            const fxpdx = func(inputa, step)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_apply_quat_d_q";
diff = td.d_apply_quat_d_q;
func = t.apply_quat
D_DP = 4;
D_NUMERATOR = 3;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_quat();
    const inputb = [ rand(10), rand(10), rand(10) ];
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            // const fxpdx = func(step)[i];
            // const fxpdx = func(inputa, step)[i];
            const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");



/**
 *  ------------------------------------------------------------------
 */
label = "d_apply_iquat_d_v";
diff = td.d_apply_iquat_d_v;
func = t.apply_iquat
D_DP = 3;
D_NUMERATOR = 3;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_quat();
    const inputb = [ rand(10), rand(10), rand(10) ];
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            // const step = [...inputa];
            const step = [...inputb];
            step[j] += dx;

            const fxpdx = func(inputa, step)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_apply_iquat_d_q";
diff = td.d_apply_iquat_d_q;
func = t.apply_iquat
D_DP = 4;
D_NUMERATOR = 3;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_quat();
    const inputb = [ rand(10), rand(10), rand(10) ];
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            // const fxpdx = func(step)[i];
            // const fxpdx = func(inputa, step)[i];
            const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");

/**
 *  ------------------------------------------------------------------
 */
label = "d_invert_quat_d_q";
diff = td.d_invert_quat_d_q;
func = t.invert_quat
D_DP = 4;
D_NUMERATOR = 4;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_quat();
    // const inputb = [ rand(10), rand(10), rand(10) ];
    const OUT = diff(inputa);//, inputb);
    const FX = func(inputa);//, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            const fxpdx = func(step)[i];
            // const fxpdx = func(inputa, step)[i];
            // const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");



/**
 *  ------------------------------------------------------------------
 */
label = "d_invert_lrQ_d_lrQ";
diff = td.d_invert_lrQ_d_lrQ;
func = t.invert_lrQ
D_DP = 7;
D_NUMERATOR = 7;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_lrQ();
    // const inputb = [ rand(10), rand(10), rand(10) ];
    const OUT = diff(inputa);//, inputb);
    const FX = func(inputa);//, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            const fxpdx = func(step)[i];
            // const fxpdx = func(inputa, step)[i];
            // const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_lrq2lrQ_d_lrq";
diff = td.d_lrq2lrQ_d_lrq;
func = t.lrq2lrQ
D_DP = 6;
D_NUMERATOR = 7;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_lrq();
    // const inputb = [ rand(10), rand(10), rand(10) ];
    const OUT = diff(inputa);//, inputb);
    const FX = func(inputa);//, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            const fxpdx = func(step)[i];
            // const fxpdx = func(inputa, step)[i];
            // const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");

/**
 *  ------------------------------------------------------------------
 */
label = "d_lrQ2lrq_d_lrQ";
diff = td.d_lrQ2lrq_d_lrQ;
func = t.lrQ2lrq
D_DP = 7;
D_NUMERATOR = 6;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_lrQ();
    // const inputb = [ rand(10), rand(10), rand(10) ];
    const OUT = diff(inputa);//, inputb);
    const FX = func(inputa);//, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            const fxpdx = func(step)[i];
            // const fxpdx = func(inputa, step)[i];
            // const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_compose_quat_d_q1";
diff = td.d_compose_quat_d_q1;
func = t.compose_quat
D_DP = 4;
D_NUMERATOR = 4;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_quat();
    const inputb = rand_quat();
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            // const fxpdx = func(step)[i];
            // const fxpdx = func(inputa, step)[i];
            const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_compose_quat_d_q2";
diff = td.d_compose_quat_d_q2;
func = t.compose_quat
D_DP = 4;
D_NUMERATOR = 4;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_quat();
    const inputb = rand_quat();
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            // const step = [...inputa];
            const step = [...inputb];
            step[j] += dx;

            // const fxpdx = func(step)[i];
            const fxpdx = func(inputa, step)[i];
            // const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_compose_lrQ_d_lrQ1";
diff = td.d_compose_lrQ_d_lrQ1;
func = t.compose_lrQ
D_DP = 7;
D_NUMERATOR = 7;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_lrQ();
    const inputb = rand_lrQ();
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            const step = [...inputa];
            // const step = [...inputb];
            step[j] += dx;

            // const fxpdx = func(step)[i];
            // const fxpdx = func(inputa, step)[i];
            const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_compose_lrQ_d_lrQ2";
diff = td.d_compose_lrQ_d_lrQ2;
func = t.compose_lrQ
D_DP = 7;
D_NUMERATOR = 7;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_lrQ();
    const inputb = rand_lrQ();
    const OUT = diff(inputa, inputb);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            // const step = [...inputa];
            const step = [...inputb];
            step[j] += dx;

            // const fxpdx = func(step)[i];
            const fxpdx = func(inputa, step)[i];
            // const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_lrQ_encaps_lrq_d_lrq_at_zero";
diff = td.d_lrQ_encaps_lrq_d_lrq_at_zero;
func = (lrQ, lrq) => t.compose_lrQ(lrQ, t.lrq2lrQ(lrq));
D_DP = 6;
D_NUMERATOR = 7;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_lrQ();
    const inputb = [ 0, 0, 0,  0, 0, 0 ];
    const OUT = diff(inputa/*, inputb*/);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            // const step = [...inputa];
            const step = [...inputb];
            step[j] += dx;

            // const fxpdx = func(step)[i];
            const fxpdx = func(inputa, step)[i];
            // const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");


/**
 *  ------------------------------------------------------------------
 */
label = "d_lrQ_encaps_lrrv_d_lrrv_at_zero";
diff = td.d_lrQ_encaps_lrrv_d_lrrv_at_zero;
func = (lrQ, lrrv) => t.compose_lrQ(lrQ, t.lrrv2lrQ(lrrv));
D_DP = 6;
D_NUMERATOR = 7;


console.log(label);
anybad = false;
for ( let idx = 0; idx < N; idx ++ ) {

    const inputa = rand_lrQ();
    const inputb = [ 0, 0, 0,  0, 0, 0 ];
    const OUT = diff(inputa/*, inputb*/);
    const FX = func(inputa, inputb);
    const DFDX = [];
    const bad = [];
    for ( let i = 0; i < D_NUMERATOR; i ++ ) {
        DFDX.push([]);
        const fx = FX[i];
        for ( let j = 0; j < D_DP; j ++ ) {
            // const step = [...inputa];
            const step = [...inputb];
            step[j] += dx;

            // const fxpdx = func(step)[i];
            const fxpdx = func(inputa, step)[i];
            // const fxpdx = func(step, inputb)[i];

            const dfdx = (fxpdx - fx) / dx;
            DFDX[i].push(dfdx);

            if ( !nearly_equal(dfdx, OUT[i][j]) ) {
                bad.push([ i, j, dfdx ]);
            }
        }
    }


    if ( bad.length ) {
        anybad = true;
        console.log(`Failed ${label}`);
        console.log("input", FX);
        console.log("diff");
        console.table(OUT);
        console.log("ndiff");
        console.table(DFDX);
        console.log("at");
        for ( const [ i, j, dfdx ] of bad ) console.log(i, j, dfdx, "!=", OUT[i][j]);
    }
}
if ( !anybad ) console.log("    Good");
