/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
acadoWorkspace.state[0] = acadoVariables.x[0];
acadoWorkspace.state[1] = acadoVariables.x[1];
acadoWorkspace.state[2] = acadoVariables.x[2];
acadoWorkspace.state[15] = acadoVariables.u[0];
acadoWorkspace.state[16] = acadoVariables.od[0];
acadoWorkspace.state[17] = acadoVariables.od[1];
acadoWorkspace.state[18] = acadoVariables.od[2];
acadoWorkspace.state[19] = acadoVariables.od[3];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{

acadoWorkspace.state[15] = acadoVariables.u[lRun1];
acadoWorkspace.state[16] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.state[17] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.state[18] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.state[19] = acadoVariables.od[lRun1 * 4 + 3];

ret = acado_integrate(acadoWorkspace.state, lRun1 == 0);

acadoVariables.x[lRun1 * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[lRun1 * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[lRun1 * 3 + 5] = acadoWorkspace.state[2];

acadoWorkspace.evGx[lRun1 * 9] = acadoWorkspace.state[3];
acadoWorkspace.evGx[lRun1 * 9 + 1] = acadoWorkspace.state[4];
acadoWorkspace.evGx[lRun1 * 9 + 2] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 9 + 3] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 9 + 4] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 9 + 5] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 9 + 6] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 9 + 7] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 9 + 8] = acadoWorkspace.state[11];

acadoWorkspace.evGu[lRun1 * 3] = acadoWorkspace.state[12];
acadoWorkspace.evGu[lRun1 * 3 + 1] = acadoWorkspace.state[13];
acadoWorkspace.evGu[lRun1 * 3 + 2] = acadoWorkspace.state[14];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
const real_t* od = in + 4;
/* Vector of auxiliary variables; number of elements: 24. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(od[2]));
a[1] = (sin(od[2]));
a[2] = (((xd[0]-od[0])*a[0])+((xd[1]-od[1])*a[1]));
a[3] = (sin(od[2]));
a[4] = (cos(od[2]));
a[5] = ((((real_t)(0.0000000000000000e+00)-(xd[0]-od[0]))*a[3])+((xd[1]-od[1])*a[4]));
a[6] = (xd[2]-od[2]);
a[7] = (tan(u[0]));
a[8] = (((od[3]*od[3])*a[7])/(real_t)(3.1000000000000001e+00));
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[3]);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(1.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[0])),2)));
a[22] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.1000000000000001e+00));
a[23] = (((od[3]*od[3])*a[21])*a[22]);

/* Compute outputs: */
out[0] = a[2];
out[1] = a[5];
out[2] = a[6];
out[3] = u[0];
out[4] = a[8];
out[5] = a[0];
out[6] = a[1];
out[7] = a[9];
out[8] = a[10];
out[9] = a[4];
out[10] = a[11];
out[11] = a[12];
out[12] = a[13];
out[13] = a[14];
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = a[15];
out[18] = a[16];
out[19] = a[17];
out[20] = a[18];
out[21] = a[19];
out[22] = a[20];
out[23] = (real_t)(1.0000000000000000e+00);
out[24] = a[23];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 3;
/* Vector of auxiliary variables; number of elements: 13. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(od[2]));
a[1] = (sin(od[2]));
a[2] = (((xd[0]-od[0])*a[0])+((xd[1]-od[1])*a[1]));
a[3] = (sin(od[2]));
a[4] = (cos(od[2]));
a[5] = ((((real_t)(0.0000000000000000e+00)-(xd[0]-od[0]))*a[3])+((xd[1]-od[1])*a[4]));
a[6] = (xd[2]-od[2]);
a[7] = (real_t)(0.0000000000000000e+00);
a[8] = (((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*a[3]);
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(1.0000000000000000e+00);

/* Compute outputs: */
out[0] = a[2];
out[1] = a[5];
out[2] = a[6];
out[3] = a[0];
out[4] = a[1];
out[5] = a[7];
out[6] = a[8];
out[7] = a[4];
out[8] = a[9];
out[9] = a[10];
out[10] = a[11];
out[11] = a[12];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0]*(real_t)2.0000000000000000e+00;
tmpQ2[1] = + tmpFx[3]*(real_t)2.0000000000000000e+00;
tmpQ2[2] = + tmpFx[6]*(real_t)2.0000000000000000e+00;
tmpQ2[3] = + tmpFx[9]*(real_t)3.0000000000000000e+00;
tmpQ2[4] = + tmpFx[12];
tmpQ2[5] = + tmpFx[1]*(real_t)2.0000000000000000e+00;
tmpQ2[6] = + tmpFx[4]*(real_t)2.0000000000000000e+00;
tmpQ2[7] = + tmpFx[7]*(real_t)2.0000000000000000e+00;
tmpQ2[8] = + tmpFx[10]*(real_t)3.0000000000000000e+00;
tmpQ2[9] = + tmpFx[13];
tmpQ2[10] = + tmpFx[2]*(real_t)2.0000000000000000e+00;
tmpQ2[11] = + tmpFx[5]*(real_t)2.0000000000000000e+00;
tmpQ2[12] = + tmpFx[8]*(real_t)2.0000000000000000e+00;
tmpQ2[13] = + tmpFx[11]*(real_t)3.0000000000000000e+00;
tmpQ2[14] = + tmpFx[14];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[3] + tmpQ2[2]*tmpFx[6] + tmpQ2[3]*tmpFx[9] + tmpQ2[4]*tmpFx[12];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[4] + tmpQ2[2]*tmpFx[7] + tmpQ2[3]*tmpFx[10] + tmpQ2[4]*tmpFx[13];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[5] + tmpQ2[2]*tmpFx[8] + tmpQ2[3]*tmpFx[11] + tmpQ2[4]*tmpFx[14];
tmpQ1[3] = + tmpQ2[5]*tmpFx[0] + tmpQ2[6]*tmpFx[3] + tmpQ2[7]*tmpFx[6] + tmpQ2[8]*tmpFx[9] + tmpQ2[9]*tmpFx[12];
tmpQ1[4] = + tmpQ2[5]*tmpFx[1] + tmpQ2[6]*tmpFx[4] + tmpQ2[7]*tmpFx[7] + tmpQ2[8]*tmpFx[10] + tmpQ2[9]*tmpFx[13];
tmpQ1[5] = + tmpQ2[5]*tmpFx[2] + tmpQ2[6]*tmpFx[5] + tmpQ2[7]*tmpFx[8] + tmpQ2[8]*tmpFx[11] + tmpQ2[9]*tmpFx[14];
tmpQ1[6] = + tmpQ2[10]*tmpFx[0] + tmpQ2[11]*tmpFx[3] + tmpQ2[12]*tmpFx[6] + tmpQ2[13]*tmpFx[9] + tmpQ2[14]*tmpFx[12];
tmpQ1[7] = + tmpQ2[10]*tmpFx[1] + tmpQ2[11]*tmpFx[4] + tmpQ2[12]*tmpFx[7] + tmpQ2[13]*tmpFx[10] + tmpQ2[14]*tmpFx[13];
tmpQ1[8] = + tmpQ2[10]*tmpFx[2] + tmpQ2[11]*tmpFx[5] + tmpQ2[12]*tmpFx[8] + tmpQ2[13]*tmpFx[11] + tmpQ2[14]*tmpFx[14];
}

void acado_setObjR1R2( real_t* const tmpFu, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0]*(real_t)2.0000000000000000e+00;
tmpR2[1] = + tmpFu[1]*(real_t)2.0000000000000000e+00;
tmpR2[2] = + tmpFu[2]*(real_t)2.0000000000000000e+00;
tmpR2[3] = + tmpFu[3]*(real_t)3.0000000000000000e+00;
tmpR2[4] = + tmpFu[4];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[1] + tmpR2[2]*tmpFu[2] + tmpR2[3]*tmpFu[3] + tmpR2[4]*tmpFu[4];
}

void acado_setObjQN1QN2( real_t* const tmpFx, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = + tmpFx[0]*(real_t)6.0000000000000000e+00;
tmpQN2[1] = + tmpFx[3]*(real_t)6.0000000000000000e+00;
tmpQN2[2] = + tmpFx[6]*(real_t)6.0000000000000000e+00;
tmpQN2[3] = + tmpFx[1]*(real_t)6.0000000000000000e+00;
tmpQN2[4] = + tmpFx[4]*(real_t)6.0000000000000000e+00;
tmpQN2[5] = + tmpFx[7]*(real_t)6.0000000000000000e+00;
tmpQN2[6] = + tmpFx[2]*(real_t)6.0000000000000000e+00;
tmpQN2[7] = + tmpFx[5]*(real_t)6.0000000000000000e+00;
tmpQN2[8] = + tmpFx[8]*(real_t)6.0000000000000000e+00;
tmpQN1[0] = + tmpQN2[0]*tmpFx[0] + tmpQN2[1]*tmpFx[3] + tmpQN2[2]*tmpFx[6];
tmpQN1[1] = + tmpQN2[0]*tmpFx[1] + tmpQN2[1]*tmpFx[4] + tmpQN2[2]*tmpFx[7];
tmpQN1[2] = + tmpQN2[0]*tmpFx[2] + tmpQN2[1]*tmpFx[5] + tmpQN2[2]*tmpFx[8];
tmpQN1[3] = + tmpQN2[3]*tmpFx[0] + tmpQN2[4]*tmpFx[3] + tmpQN2[5]*tmpFx[6];
tmpQN1[4] = + tmpQN2[3]*tmpFx[1] + tmpQN2[4]*tmpFx[4] + tmpQN2[5]*tmpFx[7];
tmpQN1[5] = + tmpQN2[3]*tmpFx[2] + tmpQN2[4]*tmpFx[5] + tmpQN2[5]*tmpFx[8];
tmpQN1[6] = + tmpQN2[6]*tmpFx[0] + tmpQN2[7]*tmpFx[3] + tmpQN2[8]*tmpFx[6];
tmpQN1[7] = + tmpQN2[6]*tmpFx[1] + tmpQN2[7]*tmpFx[4] + tmpQN2[8]*tmpFx[7];
tmpQN1[8] = + tmpQN2[6]*tmpFx[2] + tmpQN2[7]*tmpFx[5] + tmpQN2[8]*tmpFx[8];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[runObj];
acadoWorkspace.objValueIn[4] = acadoVariables.od[runObj * 4];
acadoWorkspace.objValueIn[5] = acadoVariables.od[runObj * 4 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[runObj * 4 + 2];
acadoWorkspace.objValueIn[7] = acadoVariables.od[runObj * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 5] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 5 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 5 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 5 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 5 + 4] = acadoWorkspace.objValueOut[4];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 5 ]), &(acadoWorkspace.Q1[ runObj * 9 ]), &(acadoWorkspace.Q2[ runObj * 15 ]) );

acado_setObjR1R2( &(acadoWorkspace.objValueOut[ 20 ]), &(acadoWorkspace.R1[ runObj ]), &(acadoWorkspace.R2[ runObj * 5 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[60];
acadoWorkspace.objValueIn[1] = acadoVariables.x[61];
acadoWorkspace.objValueIn[2] = acadoVariables.x[62];
acadoWorkspace.objValueIn[3] = acadoVariables.od[80];
acadoWorkspace.objValueIn[4] = acadoVariables.od[81];
acadoWorkspace.objValueIn[5] = acadoVariables.od[82];
acadoWorkspace.objValueIn[6] = acadoVariables.od[83];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];

acado_setObjQN1QN2( &(acadoWorkspace.objValueOut[ 3 ]), acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] += + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] += + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[3] + Gx1[2]*Gx2[6];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[4] + Gx1[2]*Gx2[7];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[8];
Gx3[3] = + Gx1[3]*Gx2[0] + Gx1[4]*Gx2[3] + Gx1[5]*Gx2[6];
Gx3[4] = + Gx1[3]*Gx2[1] + Gx1[4]*Gx2[4] + Gx1[5]*Gx2[7];
Gx3[5] = + Gx1[3]*Gx2[2] + Gx1[4]*Gx2[5] + Gx1[5]*Gx2[8];
Gx3[6] = + Gx1[6]*Gx2[0] + Gx1[7]*Gx2[3] + Gx1[8]*Gx2[6];
Gx3[7] = + Gx1[6]*Gx2[1] + Gx1[7]*Gx2[4] + Gx1[8]*Gx2[7];
Gx3[8] = + Gx1[6]*Gx2[2] + Gx1[7]*Gx2[5] + Gx1[8]*Gx2[8];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[1] + Gx1[2]*Gu1[2];
Gu2[1] = + Gx1[3]*Gu1[0] + Gx1[4]*Gu1[1] + Gx1[5]*Gu1[2];
Gu2[2] = + Gx1[6]*Gu1[0] + Gx1[7]*Gu1[1] + Gx1[8]*Gu1[2];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] += + Gu1[0]*Gu2[0] + Gu1[1]*Gu2[1] + Gu1[2]*Gu2[2];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] = R11[0];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 20) + (iCol)] = acadoWorkspace.H[(iCol * 20) + (iRow)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2];
dNew[1] = + Gx1[3]*dOld[0] + Gx1[4]*dOld[1] + Gx1[5]*dOld[2];
dNew[2] = + Gx1[6]*dOld[0] + Gx1[7]*dOld[1] + Gx1[8]*dOld[2];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4];
QDy1[1] = + Q2[5]*Dy1[0] + Q2[6]*Dy1[1] + Q2[7]*Dy1[2] + Q2[8]*Dy1[3] + Q2[9]*Dy1[4];
QDy1[2] = + Q2[10]*Dy1[0] + Q2[11]*Dy1[1] + Q2[12]*Dy1[2] + Q2[13]*Dy1[3] + Q2[14]*Dy1[4];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[1]*QDy1[1] + E1[2]*QDy1[2];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[1]*Gx1[3] + E1[2]*Gx1[6];
H101[1] += + E1[0]*Gx1[1] + E1[1]*Gx1[4] + E1[2]*Gx1[7];
H101[2] += + E1[0]*Gx1[2] + E1[1]*Gx1[5] + E1[2]*Gx1[8];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 3; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0];
dNew[1] += + E1[1]*U1[0];
dNew[2] += + E1[2]*U1[0];
}

void acado_multHxC( real_t* const Hx, real_t* const Gx, real_t* const A01 )
{
A01[0] = + Hx[0]*Gx[0] + Hx[1]*Gx[3] + Hx[2]*Gx[6];
A01[1] = + Hx[0]*Gx[1] + Hx[1]*Gx[4] + Hx[2]*Gx[7];
A01[2] = + Hx[0]*Gx[2] + Hx[1]*Gx[5] + Hx[2]*Gx[8];
A01[3] = + Hx[3]*Gx[0] + Hx[4]*Gx[3] + Hx[5]*Gx[6];
A01[4] = + Hx[3]*Gx[1] + Hx[4]*Gx[4] + Hx[5]*Gx[7];
A01[5] = + Hx[3]*Gx[2] + Hx[4]*Gx[5] + Hx[5]*Gx[8];
A01[6] = + Hx[6]*Gx[0] + Hx[7]*Gx[3] + Hx[8]*Gx[6];
A01[7] = + Hx[6]*Gx[1] + Hx[7]*Gx[4] + Hx[8]*Gx[7];
A01[8] = + Hx[6]*Gx[2] + Hx[7]*Gx[5] + Hx[8]*Gx[8];
A01[9] = + Hx[9]*Gx[0] + Hx[10]*Gx[3] + Hx[11]*Gx[6];
A01[10] = + Hx[9]*Gx[1] + Hx[10]*Gx[4] + Hx[11]*Gx[7];
A01[11] = + Hx[9]*Gx[2] + Hx[10]*Gx[5] + Hx[11]*Gx[8];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 80) + (col)] = + Hx[0]*E[0] + Hx[1]*E[1] + Hx[2]*E[2];
acadoWorkspace.A[(row * 80 + 20) + (col)] = + Hx[3]*E[0] + Hx[4]*E[1] + Hx[5]*E[2];
acadoWorkspace.A[(row * 80 + 40) + (col)] = + Hx[6]*E[0] + Hx[7]*E[1] + Hx[8]*E[2];
acadoWorkspace.A[(row * 80 + 60) + (col)] = + Hx[9]*E[0] + Hx[10]*E[1] + Hx[11]*E[2];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;
const real_t* od = in + 4;
/* Vector of auxiliary variables; number of elements: 30. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (cos(od[2]));
a[1] = (sin(od[2]));
a[2] = (((xd[0]-od[0])*a[0])+((xd[1]-od[1])*a[1]));
a[3] = (sin(od[2]));
a[4] = (cos(od[2]));
a[5] = ((((real_t)(0.0000000000000000e+00)-(xd[0]-od[0]))*a[3])+((xd[1]-od[1])*a[4]));
a[6] = (xd[2]-od[2]);
a[7] = (tan(u[0]));
a[8] = (((od[3]*od[3])*a[7])/(real_t)(3.1000000000000001e+00));
a[9] = (cos(od[2]));
a[10] = (sin(od[2]));
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(-1.0000000000000000e+00);
a[13] = (a[12]*a[3]);
a[14] = (cos(od[2]));
a[15] = (real_t)(0.0000000000000000e+00);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(1.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = ((real_t)(1.0000000000000000e+00)/(pow((cos(u[0])),2)));
a[26] = (od[3]*od[3]);
a[27] = (a[25]*a[26]);
a[28] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.1000000000000001e+00));
a[29] = (a[27]*a[28]);

/* Compute outputs: */
out[0] = a[2];
out[1] = a[5];
out[2] = a[6];
out[3] = a[8];
out[4] = a[9];
out[5] = a[10];
out[6] = a[11];
out[7] = a[13];
out[8] = a[14];
out[9] = a[15];
out[10] = a[16];
out[11] = a[17];
out[12] = a[18];
out[13] = a[19];
out[14] = a[20];
out[15] = a[21];
out[16] = a[22];
out[17] = a[23];
out[18] = a[24];
out[19] = a[29];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
}

void acado_condensePrep(  )
{
int lRun1;
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 9 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 3 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 3 ]), &(acadoWorkspace.E[ 6 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.evGx[ 18 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.E[ 9 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.E[ 12 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 6 ]), &(acadoWorkspace.E[ 15 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.evGx[ 27 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.E[ 18 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.E[ 21 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.E[ 24 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 9 ]), &(acadoWorkspace.E[ 27 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.evGx[ 36 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.E[ 30 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.E[ 33 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.E[ 36 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.E[ 39 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 12 ]), &(acadoWorkspace.E[ 42 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.evGx[ 45 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 45 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.E[ 48 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.E[ 51 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.E[ 54 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.E[ 57 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 15 ]), &(acadoWorkspace.E[ 60 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.evGx[ 54 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.E[ 63 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.E[ 66 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.E[ 69 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.E[ 72 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.E[ 75 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 78 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 18 ]), &(acadoWorkspace.E[ 81 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.evGx[ 63 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.E[ 84 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.E[ 87 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.E[ 90 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.E[ 93 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.E[ 96 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.E[ 99 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.E[ 102 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 21 ]), &(acadoWorkspace.E[ 105 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.evGx[ 72 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.E[ 108 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.E[ 111 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.E[ 114 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.E[ 117 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.E[ 123 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.E[ 126 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.E[ 129 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 24 ]), &(acadoWorkspace.E[ 132 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.evGx[ 81 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 135 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.E[ 138 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.E[ 141 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.E[ 144 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 147 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.E[ 150 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.E[ 153 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.E[ 156 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.E[ 159 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.E[ 162 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 90 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGx[ 90 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.E[ 165 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.E[ 168 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.E[ 171 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.E[ 174 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.E[ 177 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.E[ 180 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.E[ 183 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.E[ 186 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.E[ 189 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.E[ 192 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.E[ 195 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 99 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.evGx[ 99 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.E[ 198 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.E[ 201 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.E[ 204 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.E[ 207 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.E[ 210 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.E[ 213 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.E[ 219 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.E[ 222 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.E[ 225 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 195 ]), &(acadoWorkspace.E[ 228 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 33 ]), &(acadoWorkspace.E[ 231 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.evGx[ 108 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.E[ 234 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.E[ 237 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.E[ 243 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.E[ 246 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.E[ 249 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 252 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.E[ 255 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.E[ 258 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 225 ]), &(acadoWorkspace.E[ 261 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.E[ 264 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 231 ]), &(acadoWorkspace.E[ 267 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 36 ]), &(acadoWorkspace.E[ 270 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 117 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.evGx[ 117 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.E[ 273 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.E[ 276 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 279 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.E[ 282 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.E[ 285 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.E[ 288 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.E[ 291 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.E[ 294 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.E[ 297 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 261 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.E[ 303 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 267 ]), &(acadoWorkspace.E[ 306 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.E[ 309 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 39 ]), &(acadoWorkspace.E[ 312 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 126 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.evGx[ 126 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.E[ 315 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.E[ 318 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.E[ 321 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.E[ 324 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.E[ 327 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.E[ 330 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.E[ 333 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.E[ 336 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.E[ 339 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.E[ 342 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 303 ]), &(acadoWorkspace.E[ 345 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.E[ 348 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 309 ]), &(acadoWorkspace.E[ 351 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.E[ 354 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 42 ]), &(acadoWorkspace.E[ 357 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 135 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.evGx[ 135 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.E[ 363 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.E[ 366 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.E[ 369 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.E[ 372 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.E[ 375 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.E[ 378 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.E[ 381 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.E[ 384 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.E[ 387 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.E[ 390 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.E[ 393 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.E[ 396 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 354 ]), &(acadoWorkspace.E[ 399 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 357 ]), &(acadoWorkspace.E[ 402 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 45 ]), &(acadoWorkspace.E[ 405 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.evGx[ 144 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 408 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.E[ 411 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.E[ 414 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.E[ 417 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.E[ 420 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.E[ 423 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.E[ 426 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.E[ 429 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.E[ 432 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.E[ 435 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.E[ 438 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.E[ 441 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.E[ 444 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 399 ]), &(acadoWorkspace.E[ 447 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 402 ]), &(acadoWorkspace.E[ 450 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.E[ 453 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 48 ]), &(acadoWorkspace.E[ 456 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 153 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.evGx[ 153 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.E[ 459 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.E[ 462 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.E[ 465 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.E[ 468 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.E[ 471 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.E[ 474 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.E[ 477 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.E[ 480 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 483 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.E[ 486 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.E[ 489 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.E[ 492 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.E[ 495 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 447 ]), &(acadoWorkspace.E[ 498 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.E[ 501 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 453 ]), &(acadoWorkspace.E[ 504 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.E[ 507 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 51 ]), &(acadoWorkspace.E[ 510 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.evGx[ 162 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.E[ 513 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.E[ 516 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.E[ 519 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.E[ 522 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.E[ 525 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.E[ 528 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.E[ 531 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.E[ 534 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.E[ 537 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.E[ 540 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.E[ 543 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.E[ 546 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.E[ 549 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.E[ 552 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 501 ]), &(acadoWorkspace.E[ 555 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.E[ 558 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 507 ]), &(acadoWorkspace.E[ 561 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.E[ 564 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.E[ 567 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 171 ]), acadoWorkspace.T );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGx[ 171 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.E[ 570 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.E[ 573 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.E[ 576 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.E[ 579 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.E[ 582 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.E[ 585 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.E[ 588 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.E[ 591 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.E[ 594 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.E[ 597 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.E[ 600 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.E[ 603 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.E[ 606 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.E[ 609 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.E[ 612 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 558 ]), &(acadoWorkspace.E[ 615 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 561 ]), &(acadoWorkspace.E[ 618 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.E[ 621 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.E[ 624 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 57 ]), &(acadoWorkspace.E[ 627 ]) );

acado_multGxGu( &(acadoWorkspace.Q1[ 9 ]), acadoWorkspace.E, acadoWorkspace.QE );
acado_multGxGu( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.QE[ 3 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 18 ]), &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QE[ 9 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 27 ]), &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.QE[ 15 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QE[ 21 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 36 ]), &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 33 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 45 ]), &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 45 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 51 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 54 ]), &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 63 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 69 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 63 ]), &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 87 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 72 ]), &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 111 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 135 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 141 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 147 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 90 ]), &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 165 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 171 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QE[ 177 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.QE[ 183 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 99 ]), &(acadoWorkspace.E[ 195 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 201 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 207 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QE[ 213 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 225 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 108 ]), &(acadoWorkspace.E[ 231 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 237 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 249 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 261 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 267 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 117 ]), &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 273 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 279 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 282 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 285 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 291 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 303 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 309 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 126 ]), &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 315 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 318 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 321 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 327 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 333 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 354 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 135 ]), &(acadoWorkspace.E[ 357 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 363 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 366 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 369 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 375 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 399 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 402 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 144 ]), &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 411 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 414 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 417 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 423 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 426 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 447 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 453 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 153 ]), &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 459 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 462 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 465 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 471 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 474 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 477 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 501 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 507 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 513 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 519 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 522 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 525 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 531 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 558 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 561 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 171 ]), &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 570 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 573 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 579 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 582 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 585 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 588 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 591 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 594 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 597 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 603 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 606 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QE[ 609 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 612 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 615 ]), &(acadoWorkspace.QE[ 615 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 618 ]), &(acadoWorkspace.QE[ 618 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 621 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 627 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 3 ]), &(acadoWorkspace.evGx[ 9 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 9 ]), &(acadoWorkspace.evGx[ 18 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 18 ]), &(acadoWorkspace.evGx[ 27 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 30 ]), &(acadoWorkspace.evGx[ 36 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 45 ]), &(acadoWorkspace.evGx[ 45 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 63 ]), &(acadoWorkspace.evGx[ 54 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 84 ]), &(acadoWorkspace.evGx[ 63 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 72 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 135 ]), &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 165 ]), &(acadoWorkspace.evGx[ 90 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 198 ]), &(acadoWorkspace.evGx[ 99 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 234 ]), &(acadoWorkspace.evGx[ 108 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 273 ]), &(acadoWorkspace.evGx[ 117 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 315 ]), &(acadoWorkspace.evGx[ 126 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 360 ]), &(acadoWorkspace.evGx[ 135 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 408 ]), &(acadoWorkspace.evGx[ 144 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 459 ]), &(acadoWorkspace.evGx[ 153 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 513 ]), &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 570 ]), &(acadoWorkspace.evGx[ 171 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 6 ]), &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 21 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 33 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 87 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 111 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 138 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 201 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 237 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 318 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 363 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 411 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 462 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 516 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 573 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 3 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 15 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 51 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 69 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 114 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 141 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 171 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 279 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 321 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 366 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 414 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 465 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 519 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 6 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 27 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 39 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 93 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 117 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 174 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 207 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 243 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 282 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 369 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 417 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 468 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 522 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 579 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 9 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 57 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 75 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 147 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 177 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 246 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 285 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 327 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 372 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 471 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 525 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 582 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 12 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 99 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 123 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 213 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 249 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 330 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 375 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 423 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 474 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 585 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 15 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 81 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 126 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 153 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 183 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 291 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 333 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 378 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 426 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 477 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 531 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 588 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 18 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 105 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 129 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 186 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 219 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 255 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 294 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 381 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 429 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 534 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 591 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 21 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 159 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 189 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 222 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 258 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 297 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 339 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 483 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 537 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 594 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 24 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 225 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 261 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 342 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 387 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 435 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 486 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 597 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 195 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 303 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 345 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 390 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 438 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 489 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 543 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 30 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 231 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 267 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 306 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 348 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 393 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 441 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 492 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 546 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 603 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 33 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 309 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 351 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 396 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 444 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 495 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 549 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 606 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 36 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 39 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.H10[ 39 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 354 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 39 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 399 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 39 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 447 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 39 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 498 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 39 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 39 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 609 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 39 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 357 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 402 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 450 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 501 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 555 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 612 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 42 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 45 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 405 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.H10[ 45 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 453 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 45 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 45 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 558 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 45 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 615 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 45 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 507 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 561 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 618 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 48 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 51 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 510 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.H10[ 51 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 564 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 51 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 621 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 51 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 567 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 57 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 627 ]), &(acadoWorkspace.evGx[ 171 ]), &(acadoWorkspace.H10[ 57 ]) );

acado_setBlockH11_R1( 0, 0, acadoWorkspace.R1 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.QE[ 3 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QE[ 9 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 18 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 30 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 45 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 63 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 84 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 135 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 165 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 198 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 234 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 273 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 315 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 360 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 408 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 459 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 513 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 570 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 21 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 33 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 87 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 111 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 201 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 237 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 318 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 363 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 411 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 462 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 573 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QE[ 15 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 51 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 69 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 141 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 171 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 279 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 321 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 366 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 414 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 465 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 519 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 576 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 207 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 282 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 369 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 417 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 522 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 579 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 147 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 177 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 285 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 327 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 471 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 525 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 582 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 213 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 249 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 375 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 423 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 474 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 585 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 183 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 291 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 333 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 426 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 477 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 531 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 591 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 0, 10 );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 0, 10, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 0, 11 );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 0, 11, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 0, 12 );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 0, 12, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 0, 13 );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 0, 13, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 0, 14 );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 0, 14, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 0, 15 );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 0, 15, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 0, 16 );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 0, 16, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 0, 17 );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 0, 17, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 0, 18 );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 0, 18, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 0, 19 );
acado_setBlockH11( 0, 19, &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 1, 1, &(acadoWorkspace.R1[ 1 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QE[ 6 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 12 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QE[ 21 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 33 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 48 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 66 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 87 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 111 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 138 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 168 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 201 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 237 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 276 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 318 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 363 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 411 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 462 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 516 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 573 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QE[ 15 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 51 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 69 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 141 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 171 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 279 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 321 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 366 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 414 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 465 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 519 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 576 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 207 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 282 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 369 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 417 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 522 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 579 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 147 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 177 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 285 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 327 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 471 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 525 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 582 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 213 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 249 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 375 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 423 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 474 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 585 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 183 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 291 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 333 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 426 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 477 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 531 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 591 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 1, 10 );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 1, 10, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 1, 11 );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 1, 11, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 1, 12 );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 1, 12, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 1, 13 );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 1, 13, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 1, 14 );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 1, 14, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 1, 15 );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 1, 15, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 1, 16 );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 1, 16, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 1, 17 );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 1, 17, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 1, 18 );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 1, 18, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 1, 19 );
acado_setBlockH11( 1, 19, &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 2, 2, &(acadoWorkspace.R1[ 2 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.QE[ 15 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 24 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 36 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 51 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 69 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 90 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 114 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 141 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 171 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 204 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 240 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 279 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 321 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 366 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 414 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 465 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 519 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 576 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 207 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 282 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 369 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 417 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 522 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 579 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 147 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 177 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 285 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 327 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 471 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 525 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 582 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 213 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 249 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 375 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 423 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 474 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 585 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 183 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 291 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 333 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 426 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 477 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 531 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 591 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 2, 10 );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 2, 10, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 2, 11 );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 2, 11, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 2, 12 );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 2, 12, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 2, 13 );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 2, 13, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 2, 14 );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 2, 14, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 2, 15 );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 2, 15, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 2, 16 );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 2, 16, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 2, 17 );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 2, 17, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 2, 18 );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 2, 18, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 2, 19 );
acado_setBlockH11( 2, 19, &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 3, 3, &(acadoWorkspace.R1[ 3 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.QE[ 39 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 72 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 93 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 117 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 144 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 174 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 207 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 282 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 369 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 417 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 468 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 522 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 579 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 147 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 177 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 285 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 327 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 471 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 525 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 582 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 213 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 249 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 375 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 423 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 474 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 585 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 183 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 291 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 333 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 426 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 477 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 531 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 591 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 3, 10 );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 3, 10, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 3, 11 );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 3, 11, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 3, 12 );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 3, 12, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 3, 13 );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 3, 13, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 3, 14 );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 3, 14, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 3, 15 );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 3, 15, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 3, 16 );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 3, 16, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 3, 17 );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 3, 17, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 3, 18 );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 3, 18, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 3, 19 );
acado_setBlockH11( 3, 19, &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 4, 4, &(acadoWorkspace.R1[ 4 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QE[ 42 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.QE[ 57 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QE[ 75 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 96 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 120 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 147 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QE[ 177 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 210 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 246 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 285 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 327 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 372 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 420 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 471 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 525 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 582 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 213 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 249 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 375 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 423 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 474 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 585 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QE[ 183 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 291 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 333 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 426 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 477 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 531 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 591 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 4, 10 );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 4, 10, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 4, 11 );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 4, 11, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 4, 12 );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 4, 12, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 4, 13 );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 4, 13, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 4, 14 );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 4, 14, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 4, 15 );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 4, 15, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 4, 16 );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 4, 16, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 4, 17 );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 4, 17, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 4, 18 );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 4, 18, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 4, 19 );
acado_setBlockH11( 4, 19, &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 5, 5, &(acadoWorkspace.R1[ 5 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QE[ 60 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 78 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QE[ 99 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 123 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 150 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 180 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QE[ 213 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 249 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 288 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 330 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 375 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 423 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 474 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 528 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 585 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 183 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 291 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 333 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 426 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 477 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 531 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 591 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 5, 10 );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 5, 10, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 5, 11 );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 5, 11, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 5, 12 );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 5, 12, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 5, 13 );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 5, 13, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 5, 14 );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 5, 14, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 5, 15 );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 5, 15, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 5, 16 );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 5, 16, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 5, 17 );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 5, 17, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 5, 18 );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 5, 18, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 5, 19 );
acado_setBlockH11( 5, 19, &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 6, 6, &(acadoWorkspace.R1[ 6 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 102 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 126 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 153 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.QE[ 183 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 252 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 291 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 333 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 426 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 477 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 531 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 588 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 591 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 6, 10 );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 6, 10, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 6, 11 );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 6, 11, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 6, 12 );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 6, 12, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 6, 13 );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 6, 13, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 6, 14 );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 6, 14, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 6, 15 );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 6, 15, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 6, 16 );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 6, 16, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 6, 17 );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 6, 17, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 6, 18 );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 6, 18, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 6, 19 );
acado_setBlockH11( 6, 19, &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 7, 7, &(acadoWorkspace.R1[ 7 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.QE[ 105 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.QE[ 129 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 156 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 186 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.QE[ 219 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.QE[ 255 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 294 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 336 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 381 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 429 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 480 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 534 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 591 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 7, 10 );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 7, 10, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 7, 11 );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 7, 11, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 7, 12 );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 7, 12, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 7, 13 );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 7, 13, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 7, 14 );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 7, 14, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 7, 15 );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 7, 15, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 7, 16 );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 7, 16, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 7, 17 );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 7, 17, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 7, 18 );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 7, 18, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 7, 19 );
acado_setBlockH11( 7, 19, &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 8, 8, &(acadoWorkspace.R1[ 8 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QE[ 132 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.QE[ 159 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 222 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 258 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QE[ 339 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 384 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 483 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 537 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 594 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 8, 10 );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 8, 10, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 8, 11 );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 8, 11, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 8, 12 );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 8, 12, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 8, 13 );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 8, 13, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 8, 14 );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 8, 14, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 8, 15 );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 8, 15, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 8, 16 );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 8, 16, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 8, 17 );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 8, 17, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 8, 18 );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 8, 18, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 8, 19 );
acado_setBlockH11( 8, 19, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 9, 9, &(acadoWorkspace.R1[ 9 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 192 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 225 ]), &(acadoWorkspace.QE[ 225 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 261 ]), &(acadoWorkspace.QE[ 261 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 300 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.QE[ 342 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QE[ 387 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 435 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 597 ]) );

acado_zeroBlockH11( 9, 10 );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 225 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 261 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 9, 10, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 9, 11 );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 225 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 261 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 9, 11, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 9, 12 );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 261 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 9, 12, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 9, 13 );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 9, 13, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 9, 14 );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 9, 14, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 9, 15 );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 9, 15, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 9, 16 );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 9, 16, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 9, 17 );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 9, 17, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 9, 18 );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 9, 18, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 9, 19 );
acado_setBlockH11( 9, 19, &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 10, 10, &(acadoWorkspace.R1[ 10 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 195 ]), &(acadoWorkspace.QE[ 195 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 228 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 264 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 303 ]), &(acadoWorkspace.QE[ 303 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.QE[ 345 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 390 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QE[ 438 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 489 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 543 ]) );
acado_setBlockH11( 10, 10, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 600 ]) );

acado_zeroBlockH11( 10, 11 );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 303 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 10, 11, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 10, 12 );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 303 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 10, 12, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 10, 13 );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 303 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 10, 13, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 10, 14 );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 10, 14, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 10, 15 );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 10, 15, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 10, 16 );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 10, 16, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 10, 17 );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 10, 17, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 10, 18 );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 10, 18, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 10, 19 );
acado_setBlockH11( 10, 19, &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 11, 11, &(acadoWorkspace.R1[ 11 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 231 ]), &(acadoWorkspace.QE[ 231 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 267 ]), &(acadoWorkspace.QE[ 267 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 306 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 348 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.QE[ 393 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.QE[ 441 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 492 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 546 ]) );
acado_setBlockH11( 11, 11, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 603 ]) );

acado_zeroBlockH11( 11, 12 );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 267 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 11, 12, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 11, 13 );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 11, 13, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 11, 14 );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 11, 14, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 11, 15 );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 11, 15, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 11, 16 );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 11, 16, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 11, 17 );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 11, 17, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 11, 18 );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 11, 18, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 11, 19 );
acado_setBlockH11( 11, 19, &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 12, 12, &(acadoWorkspace.R1[ 12 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 309 ]), &(acadoWorkspace.QE[ 309 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 396 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 444 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.QE[ 495 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QE[ 549 ]) );
acado_setBlockH11( 12, 12, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 606 ]) );

acado_zeroBlockH11( 12, 13 );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 309 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 12, 13, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 12, 14 );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 12, 14, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 12, 15 );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 12, 15, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 12, 16 );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 12, 16, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 12, 17 );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 12, 17, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 12, 18 );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 12, 18, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 12, 19 );
acado_setBlockH11( 12, 19, &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 13, 13, &(acadoWorkspace.R1[ 13 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QE[ 312 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 354 ]), &(acadoWorkspace.QE[ 354 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 399 ]), &(acadoWorkspace.QE[ 399 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 447 ]), &(acadoWorkspace.QE[ 447 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.QE[ 498 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 552 ]) );
acado_setBlockH11( 13, 13, &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QE[ 609 ]) );

acado_zeroBlockH11( 13, 14 );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 354 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 399 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 447 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 13, 14, &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 13, 15 );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 399 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 447 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 13, 15, &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 13, 16 );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 447 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 13, 16, &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 13, 17 );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 13, 17, &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 13, 18 );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 13, 18, &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 13, 19 );
acado_setBlockH11( 13, 19, &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 14, 14, &(acadoWorkspace.R1[ 14 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 357 ]), &(acadoWorkspace.QE[ 357 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 402 ]), &(acadoWorkspace.QE[ 402 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 450 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 501 ]), &(acadoWorkspace.QE[ 501 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.QE[ 555 ]) );
acado_setBlockH11( 14, 14, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 612 ]) );

acado_zeroBlockH11( 14, 15 );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 402 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 501 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 14, 15, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 14, 16 );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 501 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 14, 16, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 14, 17 );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 501 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 14, 17, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 14, 18 );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 14, 18, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 14, 19 );
acado_setBlockH11( 14, 19, &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 15, 15, &(acadoWorkspace.R1[ 15 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 453 ]), &(acadoWorkspace.QE[ 453 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 504 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 558 ]), &(acadoWorkspace.QE[ 558 ]) );
acado_setBlockH11( 15, 15, &(acadoWorkspace.E[ 615 ]), &(acadoWorkspace.QE[ 615 ]) );

acado_zeroBlockH11( 15, 16 );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 453 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 558 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 15, 16, &(acadoWorkspace.E[ 615 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 15, 17 );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 558 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 15, 17, &(acadoWorkspace.E[ 615 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 15, 18 );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 558 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 15, 18, &(acadoWorkspace.E[ 615 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 15, 19 );
acado_setBlockH11( 15, 19, &(acadoWorkspace.E[ 615 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 16, 16, &(acadoWorkspace.R1[ 16 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QE[ 456 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 507 ]), &(acadoWorkspace.QE[ 507 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 561 ]), &(acadoWorkspace.QE[ 561 ]) );
acado_setBlockH11( 16, 16, &(acadoWorkspace.E[ 618 ]), &(acadoWorkspace.QE[ 618 ]) );

acado_zeroBlockH11( 16, 17 );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 507 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 561 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 16, 17, &(acadoWorkspace.E[ 618 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 16, 18 );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 561 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 16, 18, &(acadoWorkspace.E[ 618 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 16, 19 );
acado_setBlockH11( 16, 19, &(acadoWorkspace.E[ 618 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 17, 17, &(acadoWorkspace.R1[ 17 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QE[ 510 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 564 ]) );
acado_setBlockH11( 17, 17, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 621 ]) );

acado_zeroBlockH11( 17, 18 );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 17, 18, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 17, 19 );
acado_setBlockH11( 17, 19, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 18, 18, &(acadoWorkspace.R1[ 18 ]) );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 18, 18, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 624 ]) );

acado_zeroBlockH11( 18, 19 );
acado_setBlockH11( 18, 19, &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QE[ 627 ]) );

acado_setBlockH11_R1( 19, 19, &(acadoWorkspace.R1[ 19 ]) );
acado_setBlockH11( 19, 19, &(acadoWorkspace.E[ 627 ]), &(acadoWorkspace.QE[ 627 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );
acado_copyHTH( 10, 0 );
acado_copyHTH( 10, 1 );
acado_copyHTH( 10, 2 );
acado_copyHTH( 10, 3 );
acado_copyHTH( 10, 4 );
acado_copyHTH( 10, 5 );
acado_copyHTH( 10, 6 );
acado_copyHTH( 10, 7 );
acado_copyHTH( 10, 8 );
acado_copyHTH( 10, 9 );
acado_copyHTH( 11, 0 );
acado_copyHTH( 11, 1 );
acado_copyHTH( 11, 2 );
acado_copyHTH( 11, 3 );
acado_copyHTH( 11, 4 );
acado_copyHTH( 11, 5 );
acado_copyHTH( 11, 6 );
acado_copyHTH( 11, 7 );
acado_copyHTH( 11, 8 );
acado_copyHTH( 11, 9 );
acado_copyHTH( 11, 10 );
acado_copyHTH( 12, 0 );
acado_copyHTH( 12, 1 );
acado_copyHTH( 12, 2 );
acado_copyHTH( 12, 3 );
acado_copyHTH( 12, 4 );
acado_copyHTH( 12, 5 );
acado_copyHTH( 12, 6 );
acado_copyHTH( 12, 7 );
acado_copyHTH( 12, 8 );
acado_copyHTH( 12, 9 );
acado_copyHTH( 12, 10 );
acado_copyHTH( 12, 11 );
acado_copyHTH( 13, 0 );
acado_copyHTH( 13, 1 );
acado_copyHTH( 13, 2 );
acado_copyHTH( 13, 3 );
acado_copyHTH( 13, 4 );
acado_copyHTH( 13, 5 );
acado_copyHTH( 13, 6 );
acado_copyHTH( 13, 7 );
acado_copyHTH( 13, 8 );
acado_copyHTH( 13, 9 );
acado_copyHTH( 13, 10 );
acado_copyHTH( 13, 11 );
acado_copyHTH( 13, 12 );
acado_copyHTH( 14, 0 );
acado_copyHTH( 14, 1 );
acado_copyHTH( 14, 2 );
acado_copyHTH( 14, 3 );
acado_copyHTH( 14, 4 );
acado_copyHTH( 14, 5 );
acado_copyHTH( 14, 6 );
acado_copyHTH( 14, 7 );
acado_copyHTH( 14, 8 );
acado_copyHTH( 14, 9 );
acado_copyHTH( 14, 10 );
acado_copyHTH( 14, 11 );
acado_copyHTH( 14, 12 );
acado_copyHTH( 14, 13 );
acado_copyHTH( 15, 0 );
acado_copyHTH( 15, 1 );
acado_copyHTH( 15, 2 );
acado_copyHTH( 15, 3 );
acado_copyHTH( 15, 4 );
acado_copyHTH( 15, 5 );
acado_copyHTH( 15, 6 );
acado_copyHTH( 15, 7 );
acado_copyHTH( 15, 8 );
acado_copyHTH( 15, 9 );
acado_copyHTH( 15, 10 );
acado_copyHTH( 15, 11 );
acado_copyHTH( 15, 12 );
acado_copyHTH( 15, 13 );
acado_copyHTH( 15, 14 );
acado_copyHTH( 16, 0 );
acado_copyHTH( 16, 1 );
acado_copyHTH( 16, 2 );
acado_copyHTH( 16, 3 );
acado_copyHTH( 16, 4 );
acado_copyHTH( 16, 5 );
acado_copyHTH( 16, 6 );
acado_copyHTH( 16, 7 );
acado_copyHTH( 16, 8 );
acado_copyHTH( 16, 9 );
acado_copyHTH( 16, 10 );
acado_copyHTH( 16, 11 );
acado_copyHTH( 16, 12 );
acado_copyHTH( 16, 13 );
acado_copyHTH( 16, 14 );
acado_copyHTH( 16, 15 );
acado_copyHTH( 17, 0 );
acado_copyHTH( 17, 1 );
acado_copyHTH( 17, 2 );
acado_copyHTH( 17, 3 );
acado_copyHTH( 17, 4 );
acado_copyHTH( 17, 5 );
acado_copyHTH( 17, 6 );
acado_copyHTH( 17, 7 );
acado_copyHTH( 17, 8 );
acado_copyHTH( 17, 9 );
acado_copyHTH( 17, 10 );
acado_copyHTH( 17, 11 );
acado_copyHTH( 17, 12 );
acado_copyHTH( 17, 13 );
acado_copyHTH( 17, 14 );
acado_copyHTH( 17, 15 );
acado_copyHTH( 17, 16 );
acado_copyHTH( 18, 0 );
acado_copyHTH( 18, 1 );
acado_copyHTH( 18, 2 );
acado_copyHTH( 18, 3 );
acado_copyHTH( 18, 4 );
acado_copyHTH( 18, 5 );
acado_copyHTH( 18, 6 );
acado_copyHTH( 18, 7 );
acado_copyHTH( 18, 8 );
acado_copyHTH( 18, 9 );
acado_copyHTH( 18, 10 );
acado_copyHTH( 18, 11 );
acado_copyHTH( 18, 12 );
acado_copyHTH( 18, 13 );
acado_copyHTH( 18, 14 );
acado_copyHTH( 18, 15 );
acado_copyHTH( 18, 16 );
acado_copyHTH( 18, 17 );
acado_copyHTH( 19, 0 );
acado_copyHTH( 19, 1 );
acado_copyHTH( 19, 2 );
acado_copyHTH( 19, 3 );
acado_copyHTH( 19, 4 );
acado_copyHTH( 19, 5 );
acado_copyHTH( 19, 6 );
acado_copyHTH( 19, 7 );
acado_copyHTH( 19, 8 );
acado_copyHTH( 19, 9 );
acado_copyHTH( 19, 10 );
acado_copyHTH( 19, 11 );
acado_copyHTH( 19, 12 );
acado_copyHTH( 19, 13 );
acado_copyHTH( 19, 14 );
acado_copyHTH( 19, 15 );
acado_copyHTH( 19, 16 );
acado_copyHTH( 19, 17 );
acado_copyHTH( 19, 18 );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 3 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 9 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 18 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 30 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 45 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 63 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 84 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 135 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 165 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 198 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 234 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 273 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 315 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 360 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 408 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 459 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 513 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 570 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 6 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 12 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 21 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 33 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 48 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 66 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 87 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 111 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 138 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 168 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 201 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 237 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 276 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 318 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 363 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 411 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 462 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 516 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 573 ]), &(acadoWorkspace.g[ 1 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 15 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 36 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 51 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 69 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 90 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 114 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 141 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 171 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 204 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 240 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 279 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 321 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 366 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 414 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 465 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 519 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 576 ]), &(acadoWorkspace.g[ 2 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 27 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 39 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 72 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 93 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 117 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 144 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 174 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 207 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 243 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 282 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 369 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 417 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 468 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 522 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 579 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 42 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 57 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 75 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 96 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 120 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 147 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 177 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 210 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 246 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 285 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 327 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 372 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 420 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 471 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 525 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 582 ]), &(acadoWorkspace.g[ 4 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 60 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 78 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 99 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 123 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 150 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 180 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 213 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 249 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 288 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 330 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 375 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 423 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 474 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 528 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 585 ]), &(acadoWorkspace.g[ 5 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 81 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 102 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 126 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 153 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 183 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 252 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 291 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 333 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 378 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 426 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 477 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 531 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 588 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 105 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 129 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 156 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 186 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 219 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 255 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 294 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 336 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 381 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 429 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 480 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 534 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 591 ]), &(acadoWorkspace.g[ 7 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 132 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 159 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 189 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 222 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 258 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 297 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 339 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 384 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 483 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 537 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 594 ]), &(acadoWorkspace.g[ 8 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 192 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 225 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 261 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 300 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 342 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 387 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 435 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 486 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 597 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 195 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 228 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 264 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 303 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 345 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 390 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 438 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 489 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 543 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 600 ]), &(acadoWorkspace.g[ 10 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 231 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 267 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 306 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 348 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 393 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 441 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 492 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 546 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 603 ]), &(acadoWorkspace.g[ 11 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 309 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 351 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 396 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 444 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 495 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 549 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 606 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 312 ]), &(acadoWorkspace.g[ 13 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 354 ]), &(acadoWorkspace.g[ 13 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 399 ]), &(acadoWorkspace.g[ 13 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 447 ]), &(acadoWorkspace.g[ 13 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 498 ]), &(acadoWorkspace.g[ 13 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 552 ]), &(acadoWorkspace.g[ 13 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 609 ]), &(acadoWorkspace.g[ 13 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 357 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 402 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 450 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 501 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 555 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 612 ]), &(acadoWorkspace.g[ 14 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 405 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 453 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 504 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 558 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 615 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 456 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 507 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 561 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 618 ]), &(acadoWorkspace.g[ 16 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 510 ]), &(acadoWorkspace.g[ 17 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 564 ]), &(acadoWorkspace.g[ 17 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 621 ]), &(acadoWorkspace.g[ 17 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 567 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 624 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 627 ]), &(acadoWorkspace.g[ 19 ]) );
acadoWorkspace.lb[0] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.ub[0] = (real_t)6.5000000000000002e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)6.5000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)6.5000000000000002e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)6.5000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)6.5000000000000002e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)6.5000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)6.5000000000000002e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)6.5000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)6.5000000000000002e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)6.5000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)6.5000000000000002e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)6.5000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)6.5000000000000002e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)6.5000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)6.5000000000000002e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)6.5000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)6.5000000000000002e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)6.5000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)6.5000000000000002e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)6.5000000000000002e-01 - acadoVariables.u[19];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.u[lRun1];
acadoWorkspace.conValueIn[4] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.conValueIn[5] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.conValueIn[6] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.conValueIn[7] = acadoVariables.od[lRun1 * 4 + 3];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 4] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evH[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[3];

acadoWorkspace.evHx[lRun1 * 12] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evHx[lRun1 * 12 + 1] = acadoWorkspace.conValueOut[5];
acadoWorkspace.evHx[lRun1 * 12 + 2] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 12 + 3] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 12 + 4] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 12 + 5] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 12 + 6] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 12 + 7] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 12 + 8] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 12 + 9] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 12 + 10] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 12 + 11] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHu[lRun1 * 4] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHu[lRun1 * 4 + 1] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHu[lRun1 * 4 + 2] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHu[lRun1 * 4 + 3] = acadoWorkspace.conValueOut[19];
}

acadoWorkspace.A01[0] = acadoWorkspace.evHx[0];
acadoWorkspace.A01[1] = acadoWorkspace.evHx[1];
acadoWorkspace.A01[2] = acadoWorkspace.evHx[2];
acadoWorkspace.A01[3] = acadoWorkspace.evHx[3];
acadoWorkspace.A01[4] = acadoWorkspace.evHx[4];
acadoWorkspace.A01[5] = acadoWorkspace.evHx[5];
acadoWorkspace.A01[6] = acadoWorkspace.evHx[6];
acadoWorkspace.A01[7] = acadoWorkspace.evHx[7];
acadoWorkspace.A01[8] = acadoWorkspace.evHx[8];
acadoWorkspace.A01[9] = acadoWorkspace.evHx[9];
acadoWorkspace.A01[10] = acadoWorkspace.evHx[10];
acadoWorkspace.A01[11] = acadoWorkspace.evHx[11];

acado_multHxC( &(acadoWorkspace.evHx[ 12 ]), acadoWorkspace.evGx, &(acadoWorkspace.A01[ 12 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.evGx[ 9 ]), &(acadoWorkspace.A01[ 24 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.evGx[ 18 ]), &(acadoWorkspace.A01[ 36 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.evGx[ 27 ]), &(acadoWorkspace.A01[ 48 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.evGx[ 36 ]), &(acadoWorkspace.A01[ 60 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.evGx[ 45 ]), &(acadoWorkspace.A01[ 72 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.evGx[ 54 ]), &(acadoWorkspace.A01[ 84 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.evGx[ 63 ]), &(acadoWorkspace.A01[ 96 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.evGx[ 72 ]), &(acadoWorkspace.A01[ 108 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.A01[ 120 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.evGx[ 90 ]), &(acadoWorkspace.A01[ 132 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.evGx[ 99 ]), &(acadoWorkspace.A01[ 144 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.evGx[ 108 ]), &(acadoWorkspace.A01[ 156 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.evGx[ 117 ]), &(acadoWorkspace.A01[ 168 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.evGx[ 126 ]), &(acadoWorkspace.A01[ 180 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.evGx[ 135 ]), &(acadoWorkspace.A01[ 192 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.evGx[ 144 ]), &(acadoWorkspace.A01[ 204 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.evGx[ 153 ]), &(acadoWorkspace.A01[ 216 ]) );
acado_multHxC( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.A01[ 228 ]) );

acado_multHxE( &(acadoWorkspace.evHx[ 12 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 3 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 24 ]), &(acadoWorkspace.E[ 6 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 9 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 12 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 36 ]), &(acadoWorkspace.E[ 15 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 18 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 21 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 24 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 48 ]), &(acadoWorkspace.E[ 27 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 30 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 33 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 36 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 39 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 42 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 45 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 48 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 51 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 54 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 57 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 72 ]), &(acadoWorkspace.E[ 60 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 63 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 66 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 69 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 72 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 75 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 78 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 84 ]), &(acadoWorkspace.E[ 81 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 84 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 87 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 90 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 93 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 96 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 99 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 102 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 96 ]), &(acadoWorkspace.E[ 105 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 108 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 111 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 114 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 117 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 120 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 123 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 126 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 129 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 108 ]), &(acadoWorkspace.E[ 132 ]), 9, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 135 ]), 10, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 138 ]), 10, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 141 ]), 10, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 144 ]), 10, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 147 ]), 10, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 150 ]), 10, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 153 ]), 10, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 156 ]), 10, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 159 ]), 10, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 162 ]), 10, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 165 ]), 11, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 168 ]), 11, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 171 ]), 11, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 174 ]), 11, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 177 ]), 11, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 180 ]), 11, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 183 ]), 11, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 186 ]), 11, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 189 ]), 11, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 192 ]), 11, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 132 ]), &(acadoWorkspace.E[ 195 ]), 11, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 198 ]), 12, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 201 ]), 12, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 204 ]), 12, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 207 ]), 12, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 210 ]), 12, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 213 ]), 12, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 216 ]), 12, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 219 ]), 12, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 222 ]), 12, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 225 ]), 12, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 228 ]), 12, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 144 ]), &(acadoWorkspace.E[ 231 ]), 12, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 234 ]), 13, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 237 ]), 13, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 240 ]), 13, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 243 ]), 13, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 246 ]), 13, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 249 ]), 13, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 252 ]), 13, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 255 ]), 13, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 258 ]), 13, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 261 ]), 13, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 264 ]), 13, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 267 ]), 13, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 156 ]), &(acadoWorkspace.E[ 270 ]), 13, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 273 ]), 14, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 276 ]), 14, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 279 ]), 14, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 282 ]), 14, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 285 ]), 14, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 288 ]), 14, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 291 ]), 14, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 294 ]), 14, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 297 ]), 14, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 300 ]), 14, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 303 ]), 14, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 306 ]), 14, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 309 ]), 14, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 168 ]), &(acadoWorkspace.E[ 312 ]), 14, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 315 ]), 15, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 318 ]), 15, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 321 ]), 15, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 324 ]), 15, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 327 ]), 15, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 330 ]), 15, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 333 ]), 15, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 336 ]), 15, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 339 ]), 15, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 342 ]), 15, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 345 ]), 15, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 348 ]), 15, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 351 ]), 15, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 354 ]), 15, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 357 ]), 15, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 360 ]), 16, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 363 ]), 16, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 366 ]), 16, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 369 ]), 16, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 372 ]), 16, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 375 ]), 16, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 378 ]), 16, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 381 ]), 16, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 384 ]), 16, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 387 ]), 16, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 390 ]), 16, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 393 ]), 16, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 396 ]), 16, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 399 ]), 16, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 402 ]), 16, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 192 ]), &(acadoWorkspace.E[ 405 ]), 16, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 408 ]), 17, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 411 ]), 17, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 414 ]), 17, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 417 ]), 17, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 420 ]), 17, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 423 ]), 17, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 426 ]), 17, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 429 ]), 17, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 432 ]), 17, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 435 ]), 17, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 438 ]), 17, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 441 ]), 17, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 444 ]), 17, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 447 ]), 17, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 450 ]), 17, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 453 ]), 17, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 204 ]), &(acadoWorkspace.E[ 456 ]), 17, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 459 ]), 18, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 462 ]), 18, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 465 ]), 18, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 468 ]), 18, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 471 ]), 18, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 474 ]), 18, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 477 ]), 18, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 480 ]), 18, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 483 ]), 18, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 486 ]), 18, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 489 ]), 18, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 492 ]), 18, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 495 ]), 18, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 498 ]), 18, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 501 ]), 18, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 504 ]), 18, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 507 ]), 18, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 216 ]), &(acadoWorkspace.E[ 510 ]), 18, 17 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 513 ]), 19, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 516 ]), 19, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 519 ]), 19, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 522 ]), 19, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 525 ]), 19, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 528 ]), 19, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 531 ]), 19, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 534 ]), 19, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 537 ]), 19, 8 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 540 ]), 19, 9 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 543 ]), 19, 10 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 546 ]), 19, 11 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 549 ]), 19, 12 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 552 ]), 19, 13 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 555 ]), 19, 14 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 558 ]), 19, 15 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 561 ]), 19, 16 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 564 ]), 19, 17 );
acado_multHxE( &(acadoWorkspace.evHx[ 228 ]), &(acadoWorkspace.E[ 567 ]), 19, 18 );

acadoWorkspace.A[0] = acadoWorkspace.evHu[0];
acadoWorkspace.A[20] = acadoWorkspace.evHu[1];
acadoWorkspace.A[40] = acadoWorkspace.evHu[2];
acadoWorkspace.A[60] = acadoWorkspace.evHu[3];
acadoWorkspace.A[81] = acadoWorkspace.evHu[4];
acadoWorkspace.A[101] = acadoWorkspace.evHu[5];
acadoWorkspace.A[121] = acadoWorkspace.evHu[6];
acadoWorkspace.A[141] = acadoWorkspace.evHu[7];
acadoWorkspace.A[162] = acadoWorkspace.evHu[8];
acadoWorkspace.A[182] = acadoWorkspace.evHu[9];
acadoWorkspace.A[202] = acadoWorkspace.evHu[10];
acadoWorkspace.A[222] = acadoWorkspace.evHu[11];
acadoWorkspace.A[243] = acadoWorkspace.evHu[12];
acadoWorkspace.A[263] = acadoWorkspace.evHu[13];
acadoWorkspace.A[283] = acadoWorkspace.evHu[14];
acadoWorkspace.A[303] = acadoWorkspace.evHu[15];
acadoWorkspace.A[324] = acadoWorkspace.evHu[16];
acadoWorkspace.A[344] = acadoWorkspace.evHu[17];
acadoWorkspace.A[364] = acadoWorkspace.evHu[18];
acadoWorkspace.A[384] = acadoWorkspace.evHu[19];
acadoWorkspace.A[405] = acadoWorkspace.evHu[20];
acadoWorkspace.A[425] = acadoWorkspace.evHu[21];
acadoWorkspace.A[445] = acadoWorkspace.evHu[22];
acadoWorkspace.A[465] = acadoWorkspace.evHu[23];
acadoWorkspace.A[486] = acadoWorkspace.evHu[24];
acadoWorkspace.A[506] = acadoWorkspace.evHu[25];
acadoWorkspace.A[526] = acadoWorkspace.evHu[26];
acadoWorkspace.A[546] = acadoWorkspace.evHu[27];
acadoWorkspace.A[567] = acadoWorkspace.evHu[28];
acadoWorkspace.A[587] = acadoWorkspace.evHu[29];
acadoWorkspace.A[607] = acadoWorkspace.evHu[30];
acadoWorkspace.A[627] = acadoWorkspace.evHu[31];
acadoWorkspace.A[648] = acadoWorkspace.evHu[32];
acadoWorkspace.A[668] = acadoWorkspace.evHu[33];
acadoWorkspace.A[688] = acadoWorkspace.evHu[34];
acadoWorkspace.A[708] = acadoWorkspace.evHu[35];
acadoWorkspace.A[729] = acadoWorkspace.evHu[36];
acadoWorkspace.A[749] = acadoWorkspace.evHu[37];
acadoWorkspace.A[769] = acadoWorkspace.evHu[38];
acadoWorkspace.A[789] = acadoWorkspace.evHu[39];
acadoWorkspace.A[810] = acadoWorkspace.evHu[40];
acadoWorkspace.A[830] = acadoWorkspace.evHu[41];
acadoWorkspace.A[850] = acadoWorkspace.evHu[42];
acadoWorkspace.A[870] = acadoWorkspace.evHu[43];
acadoWorkspace.A[891] = acadoWorkspace.evHu[44];
acadoWorkspace.A[911] = acadoWorkspace.evHu[45];
acadoWorkspace.A[931] = acadoWorkspace.evHu[46];
acadoWorkspace.A[951] = acadoWorkspace.evHu[47];
acadoWorkspace.A[972] = acadoWorkspace.evHu[48];
acadoWorkspace.A[992] = acadoWorkspace.evHu[49];
acadoWorkspace.A[1012] = acadoWorkspace.evHu[50];
acadoWorkspace.A[1032] = acadoWorkspace.evHu[51];
acadoWorkspace.A[1053] = acadoWorkspace.evHu[52];
acadoWorkspace.A[1073] = acadoWorkspace.evHu[53];
acadoWorkspace.A[1093] = acadoWorkspace.evHu[54];
acadoWorkspace.A[1113] = acadoWorkspace.evHu[55];
acadoWorkspace.A[1134] = acadoWorkspace.evHu[56];
acadoWorkspace.A[1154] = acadoWorkspace.evHu[57];
acadoWorkspace.A[1174] = acadoWorkspace.evHu[58];
acadoWorkspace.A[1194] = acadoWorkspace.evHu[59];
acadoWorkspace.A[1215] = acadoWorkspace.evHu[60];
acadoWorkspace.A[1235] = acadoWorkspace.evHu[61];
acadoWorkspace.A[1255] = acadoWorkspace.evHu[62];
acadoWorkspace.A[1275] = acadoWorkspace.evHu[63];
acadoWorkspace.A[1296] = acadoWorkspace.evHu[64];
acadoWorkspace.A[1316] = acadoWorkspace.evHu[65];
acadoWorkspace.A[1336] = acadoWorkspace.evHu[66];
acadoWorkspace.A[1356] = acadoWorkspace.evHu[67];
acadoWorkspace.A[1377] = acadoWorkspace.evHu[68];
acadoWorkspace.A[1397] = acadoWorkspace.evHu[69];
acadoWorkspace.A[1417] = acadoWorkspace.evHu[70];
acadoWorkspace.A[1437] = acadoWorkspace.evHu[71];
acadoWorkspace.A[1458] = acadoWorkspace.evHu[72];
acadoWorkspace.A[1478] = acadoWorkspace.evHu[73];
acadoWorkspace.A[1498] = acadoWorkspace.evHu[74];
acadoWorkspace.A[1518] = acadoWorkspace.evHu[75];
acadoWorkspace.A[1539] = acadoWorkspace.evHu[76];
acadoWorkspace.A[1559] = acadoWorkspace.evHu[77];
acadoWorkspace.A[1579] = acadoWorkspace.evHu[78];
acadoWorkspace.A[1599] = acadoWorkspace.evHu[79];
acadoWorkspace.lbA[0] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[1] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[2] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[3] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[4] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[5] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[6] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[7] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[8] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[9] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[10] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[11] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[12] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[13] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[14] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[15] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[16] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[17] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[18] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[19] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[20] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[21] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[22] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[23] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[24] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[25] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[26] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[27] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[28] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[29] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[29];
acadoWorkspace.lbA[30] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[30];
acadoWorkspace.lbA[31] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[31];
acadoWorkspace.lbA[32] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[32];
acadoWorkspace.lbA[33] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[33];
acadoWorkspace.lbA[34] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[34];
acadoWorkspace.lbA[35] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[35];
acadoWorkspace.lbA[36] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[36];
acadoWorkspace.lbA[37] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[37];
acadoWorkspace.lbA[38] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[38];
acadoWorkspace.lbA[39] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[39];
acadoWorkspace.lbA[40] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[40];
acadoWorkspace.lbA[41] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[41];
acadoWorkspace.lbA[42] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[42];
acadoWorkspace.lbA[43] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[43];
acadoWorkspace.lbA[44] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[44];
acadoWorkspace.lbA[45] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[45];
acadoWorkspace.lbA[46] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[46];
acadoWorkspace.lbA[47] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[47];
acadoWorkspace.lbA[48] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[48];
acadoWorkspace.lbA[49] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[49];
acadoWorkspace.lbA[50] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[50];
acadoWorkspace.lbA[51] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[51];
acadoWorkspace.lbA[52] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[52];
acadoWorkspace.lbA[53] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[53];
acadoWorkspace.lbA[54] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[54];
acadoWorkspace.lbA[55] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[55];
acadoWorkspace.lbA[56] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[56];
acadoWorkspace.lbA[57] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[57];
acadoWorkspace.lbA[58] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[58];
acadoWorkspace.lbA[59] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[59];
acadoWorkspace.lbA[60] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[60];
acadoWorkspace.lbA[61] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[61];
acadoWorkspace.lbA[62] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[62];
acadoWorkspace.lbA[63] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[63];
acadoWorkspace.lbA[64] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[64];
acadoWorkspace.lbA[65] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[65];
acadoWorkspace.lbA[66] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[66];
acadoWorkspace.lbA[67] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[67];
acadoWorkspace.lbA[68] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[68];
acadoWorkspace.lbA[69] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[69];
acadoWorkspace.lbA[70] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[70];
acadoWorkspace.lbA[71] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[71];
acadoWorkspace.lbA[72] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[72];
acadoWorkspace.lbA[73] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[73];
acadoWorkspace.lbA[74] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[74];
acadoWorkspace.lbA[75] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[75];
acadoWorkspace.lbA[76] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[76];
acadoWorkspace.lbA[77] = (real_t)-2.0000000000000000e+00 - acadoWorkspace.evH[77];
acadoWorkspace.lbA[78] = (real_t)-7.8539816339744828e-01 - acadoWorkspace.evH[78];
acadoWorkspace.lbA[79] = (real_t)-4.1923295895657581e+01 - acadoWorkspace.evH[79];

acadoWorkspace.ubA[0] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[1] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[2] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[3] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[4] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[5] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[6] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[7] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[8] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[9] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[10] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[11] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[12] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[13] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[14] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[15] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[16] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[17] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[18] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[19] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[20] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[21] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[22] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[23] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[24] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[25] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[26] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[27] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[28] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[29] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[30] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[31] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[32] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[33] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[34] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[35] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[36] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[37] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[38] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[39] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[39];
acadoWorkspace.ubA[40] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[40];
acadoWorkspace.ubA[41] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[41];
acadoWorkspace.ubA[42] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[42];
acadoWorkspace.ubA[43] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[43];
acadoWorkspace.ubA[44] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[44];
acadoWorkspace.ubA[45] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[45];
acadoWorkspace.ubA[46] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[46];
acadoWorkspace.ubA[47] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[47];
acadoWorkspace.ubA[48] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[48];
acadoWorkspace.ubA[49] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[49];
acadoWorkspace.ubA[50] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[50];
acadoWorkspace.ubA[51] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[51];
acadoWorkspace.ubA[52] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[52];
acadoWorkspace.ubA[53] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[53];
acadoWorkspace.ubA[54] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[54];
acadoWorkspace.ubA[55] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[55];
acadoWorkspace.ubA[56] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[56];
acadoWorkspace.ubA[57] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[57];
acadoWorkspace.ubA[58] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[58];
acadoWorkspace.ubA[59] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[59];
acadoWorkspace.ubA[60] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[60];
acadoWorkspace.ubA[61] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[61];
acadoWorkspace.ubA[62] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[62];
acadoWorkspace.ubA[63] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[63];
acadoWorkspace.ubA[64] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[64];
acadoWorkspace.ubA[65] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[65];
acadoWorkspace.ubA[66] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[66];
acadoWorkspace.ubA[67] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[67];
acadoWorkspace.ubA[68] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[68];
acadoWorkspace.ubA[69] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[69];
acadoWorkspace.ubA[70] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[70];
acadoWorkspace.ubA[71] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[71];
acadoWorkspace.ubA[72] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[72];
acadoWorkspace.ubA[73] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[73];
acadoWorkspace.ubA[74] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[74];
acadoWorkspace.ubA[75] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[75];
acadoWorkspace.ubA[76] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[76];
acadoWorkspace.ubA[77] = (real_t)2.0000000000000000e+00 - acadoWorkspace.evH[77];
acadoWorkspace.ubA[78] = (real_t)7.8539816339744828e-01 - acadoWorkspace.evH[78];
acadoWorkspace.ubA[79] = (real_t)4.1923295895657581e+01 - acadoWorkspace.evH[79];

}

void acado_condenseFdb(  )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.Dy[90] -= acadoVariables.y[90];
acadoWorkspace.Dy[91] -= acadoVariables.y[91];
acadoWorkspace.Dy[92] -= acadoVariables.y[92];
acadoWorkspace.Dy[93] -= acadoVariables.y[93];
acadoWorkspace.Dy[94] -= acadoVariables.y[94];
acadoWorkspace.Dy[95] -= acadoVariables.y[95];
acadoWorkspace.Dy[96] -= acadoVariables.y[96];
acadoWorkspace.Dy[97] -= acadoVariables.y[97];
acadoWorkspace.Dy[98] -= acadoVariables.y[98];
acadoWorkspace.Dy[99] -= acadoVariables.y[99];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 5 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 10 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 15 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 20 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 25 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 30 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 35 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 40 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 45 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 50 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 55 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 60 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 65 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 70 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 75 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 80 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 85 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 90 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 95 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 19 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 15 ]), &(acadoWorkspace.Dy[ 5 ]), &(acadoWorkspace.QDy[ 3 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 30 ]), &(acadoWorkspace.Dy[ 10 ]), &(acadoWorkspace.QDy[ 6 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 45 ]), &(acadoWorkspace.Dy[ 15 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 60 ]), &(acadoWorkspace.Dy[ 20 ]), &(acadoWorkspace.QDy[ 12 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 75 ]), &(acadoWorkspace.Dy[ 25 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 90 ]), &(acadoWorkspace.Dy[ 30 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 105 ]), &(acadoWorkspace.Dy[ 35 ]), &(acadoWorkspace.QDy[ 21 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 120 ]), &(acadoWorkspace.Dy[ 40 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 135 ]), &(acadoWorkspace.Dy[ 45 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 150 ]), &(acadoWorkspace.Dy[ 50 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 165 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 33 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 195 ]), &(acadoWorkspace.Dy[ 65 ]), &(acadoWorkspace.QDy[ 39 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 210 ]), &(acadoWorkspace.Dy[ 70 ]), &(acadoWorkspace.QDy[ 42 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 225 ]), &(acadoWorkspace.Dy[ 75 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 240 ]), &(acadoWorkspace.Dy[ 80 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 255 ]), &(acadoWorkspace.Dy[ 85 ]), &(acadoWorkspace.QDy[ 51 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 270 ]), &(acadoWorkspace.Dy[ 90 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 285 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 57 ]) );

acadoWorkspace.QDy[60] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[61] = + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[2];
acadoWorkspace.QDy[62] = + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[2];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 3 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 3 ]), &(acadoWorkspace.QDy[ 6 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 9 ]), &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 18 ]), &(acadoWorkspace.QDy[ 12 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 45 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 63 ]), &(acadoWorkspace.QDy[ 21 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 84 ]), &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 165 ]), &(acadoWorkspace.QDy[ 33 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 198 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 234 ]), &(acadoWorkspace.QDy[ 39 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 273 ]), &(acadoWorkspace.QDy[ 42 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 315 ]), &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 408 ]), &(acadoWorkspace.QDy[ 51 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QDy[ 57 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 570 ]), &(acadoWorkspace.QDy[ 60 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.QDy[ 6 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 1 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.QDy[ 9 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QDy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.QDy[ 15 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 5 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QDy[ 21 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.QDy[ 24 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 7 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QDy[ 30 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 225 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 261 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 195 ]), &(acadoWorkspace.QDy[ 33 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 303 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 231 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 267 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 11 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QDy[ 39 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 309 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.QDy[ 42 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 354 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 399 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 447 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 13 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 357 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 402 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 501 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QDy[ 48 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 453 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 558 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 615 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.QDy[ 51 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 507 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 561 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 618 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 17 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QDy[ 57 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 627 ]), &(acadoWorkspace.QDy[ 60 ]), &(acadoWorkspace.g[ 19 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[1] += + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[2] += + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[3] += + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[4] += + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[5] += + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[6] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[7] += + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[8] += + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[9] += + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[10] += + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[11] += + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[12] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[13] += + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[14] += + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[15] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[16] += + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[17] += + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[18] += + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[2];
acadoWorkspace.g[19] += + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[2];

acadoWorkspace.pacA01Dx0[0] = + acadoWorkspace.A01[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[2]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[1] = + acadoWorkspace.A01[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[5]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[2] = + acadoWorkspace.A01[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[8]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[3] = + acadoWorkspace.A01[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[11]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[4] = + acadoWorkspace.A01[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[14]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[5] = + acadoWorkspace.A01[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[17]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[6] = + acadoWorkspace.A01[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[20]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[7] = + acadoWorkspace.A01[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[23]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[8] = + acadoWorkspace.A01[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[26]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[9] = + acadoWorkspace.A01[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[29]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[10] = + acadoWorkspace.A01[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[32]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[11] = + acadoWorkspace.A01[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[35]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[12] = + acadoWorkspace.A01[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[38]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[13] = + acadoWorkspace.A01[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[41]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[14] = + acadoWorkspace.A01[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[44]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[15] = + acadoWorkspace.A01[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[47]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[16] = + acadoWorkspace.A01[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[50]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[17] = + acadoWorkspace.A01[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[53]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[18] = + acadoWorkspace.A01[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[56]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[19] = + acadoWorkspace.A01[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[59]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[20] = + acadoWorkspace.A01[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[62]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[21] = + acadoWorkspace.A01[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[65]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[22] = + acadoWorkspace.A01[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[68]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[23] = + acadoWorkspace.A01[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[71]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[24] = + acadoWorkspace.A01[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[74]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[25] = + acadoWorkspace.A01[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[77]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[26] = + acadoWorkspace.A01[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[80]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[27] = + acadoWorkspace.A01[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[83]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[28] = + acadoWorkspace.A01[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[86]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[29] = + acadoWorkspace.A01[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[89]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[30] = + acadoWorkspace.A01[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[92]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[31] = + acadoWorkspace.A01[93]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[94]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[95]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[32] = + acadoWorkspace.A01[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[98]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[33] = + acadoWorkspace.A01[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[101]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[34] = + acadoWorkspace.A01[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[104]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[35] = + acadoWorkspace.A01[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[107]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[36] = + acadoWorkspace.A01[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[110]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[37] = + acadoWorkspace.A01[111]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[112]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[113]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[38] = + acadoWorkspace.A01[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[116]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[39] = + acadoWorkspace.A01[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[119]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[40] = + acadoWorkspace.A01[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[122]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[41] = + acadoWorkspace.A01[123]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[124]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[125]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[42] = + acadoWorkspace.A01[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[128]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[43] = + acadoWorkspace.A01[129]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[130]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[131]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[44] = + acadoWorkspace.A01[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[134]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[45] = + acadoWorkspace.A01[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[137]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[46] = + acadoWorkspace.A01[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[140]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[47] = + acadoWorkspace.A01[141]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[142]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[143]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[48] = + acadoWorkspace.A01[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[146]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[49] = + acadoWorkspace.A01[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[149]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[50] = + acadoWorkspace.A01[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[152]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[51] = + acadoWorkspace.A01[153]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[154]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[155]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[52] = + acadoWorkspace.A01[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[158]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[53] = + acadoWorkspace.A01[159]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[160]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[161]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[54] = + acadoWorkspace.A01[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[164]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[55] = + acadoWorkspace.A01[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[167]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[56] = + acadoWorkspace.A01[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[170]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[57] = + acadoWorkspace.A01[171]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[172]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[173]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[58] = + acadoWorkspace.A01[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[176]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[59] = + acadoWorkspace.A01[177]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[178]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[179]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[60] = + acadoWorkspace.A01[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[182]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[61] = + acadoWorkspace.A01[183]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[184]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[185]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[62] = + acadoWorkspace.A01[186]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[187]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[188]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[63] = + acadoWorkspace.A01[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[191]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[64] = + acadoWorkspace.A01[192]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[193]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[194]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[65] = + acadoWorkspace.A01[195]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[196]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[197]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[66] = + acadoWorkspace.A01[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[200]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[67] = + acadoWorkspace.A01[201]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[202]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[203]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[68] = + acadoWorkspace.A01[204]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[205]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[206]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[69] = + acadoWorkspace.A01[207]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[208]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[209]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[70] = + acadoWorkspace.A01[210]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[211]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[212]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[71] = + acadoWorkspace.A01[213]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[214]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[215]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[72] = + acadoWorkspace.A01[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[218]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[73] = + acadoWorkspace.A01[219]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[220]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[221]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[74] = + acadoWorkspace.A01[222]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[223]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[224]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[75] = + acadoWorkspace.A01[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[227]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[76] = + acadoWorkspace.A01[228]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[229]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[230]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[77] = + acadoWorkspace.A01[231]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[232]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[233]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[78] = + acadoWorkspace.A01[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[236]*acadoWorkspace.Dx0[2];
acadoWorkspace.pacA01Dx0[79] = + acadoWorkspace.A01[237]*acadoWorkspace.Dx0[0] + acadoWorkspace.A01[238]*acadoWorkspace.Dx0[1] + acadoWorkspace.A01[239]*acadoWorkspace.Dx0[2];
acadoWorkspace.lbA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.lbA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.lbA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.lbA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.lbA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.lbA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.lbA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.lbA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.lbA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.lbA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.lbA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.lbA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.lbA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.lbA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.lbA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.lbA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.lbA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.lbA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.lbA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.lbA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.lbA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.lbA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.lbA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.lbA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.lbA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.lbA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.lbA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.lbA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.lbA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.lbA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.lbA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.lbA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.lbA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.lbA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.lbA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.lbA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.lbA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.lbA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.lbA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.lbA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.lbA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.lbA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.lbA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.lbA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.lbA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.lbA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.lbA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.lbA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.lbA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.lbA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.lbA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.lbA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.lbA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.lbA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.lbA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.lbA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.lbA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.lbA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.lbA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.lbA[59] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.lbA[60] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.lbA[61] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.lbA[62] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.lbA[63] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.lbA[64] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.lbA[65] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.lbA[66] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.lbA[67] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.lbA[68] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.lbA[69] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.lbA[70] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.lbA[71] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.lbA[72] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.lbA[73] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.lbA[74] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.lbA[75] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.lbA[76] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.lbA[77] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.lbA[78] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.lbA[79] -= acadoWorkspace.pacA01Dx0[79];

acadoWorkspace.ubA[0] -= acadoWorkspace.pacA01Dx0[0];
acadoWorkspace.ubA[1] -= acadoWorkspace.pacA01Dx0[1];
acadoWorkspace.ubA[2] -= acadoWorkspace.pacA01Dx0[2];
acadoWorkspace.ubA[3] -= acadoWorkspace.pacA01Dx0[3];
acadoWorkspace.ubA[4] -= acadoWorkspace.pacA01Dx0[4];
acadoWorkspace.ubA[5] -= acadoWorkspace.pacA01Dx0[5];
acadoWorkspace.ubA[6] -= acadoWorkspace.pacA01Dx0[6];
acadoWorkspace.ubA[7] -= acadoWorkspace.pacA01Dx0[7];
acadoWorkspace.ubA[8] -= acadoWorkspace.pacA01Dx0[8];
acadoWorkspace.ubA[9] -= acadoWorkspace.pacA01Dx0[9];
acadoWorkspace.ubA[10] -= acadoWorkspace.pacA01Dx0[10];
acadoWorkspace.ubA[11] -= acadoWorkspace.pacA01Dx0[11];
acadoWorkspace.ubA[12] -= acadoWorkspace.pacA01Dx0[12];
acadoWorkspace.ubA[13] -= acadoWorkspace.pacA01Dx0[13];
acadoWorkspace.ubA[14] -= acadoWorkspace.pacA01Dx0[14];
acadoWorkspace.ubA[15] -= acadoWorkspace.pacA01Dx0[15];
acadoWorkspace.ubA[16] -= acadoWorkspace.pacA01Dx0[16];
acadoWorkspace.ubA[17] -= acadoWorkspace.pacA01Dx0[17];
acadoWorkspace.ubA[18] -= acadoWorkspace.pacA01Dx0[18];
acadoWorkspace.ubA[19] -= acadoWorkspace.pacA01Dx0[19];
acadoWorkspace.ubA[20] -= acadoWorkspace.pacA01Dx0[20];
acadoWorkspace.ubA[21] -= acadoWorkspace.pacA01Dx0[21];
acadoWorkspace.ubA[22] -= acadoWorkspace.pacA01Dx0[22];
acadoWorkspace.ubA[23] -= acadoWorkspace.pacA01Dx0[23];
acadoWorkspace.ubA[24] -= acadoWorkspace.pacA01Dx0[24];
acadoWorkspace.ubA[25] -= acadoWorkspace.pacA01Dx0[25];
acadoWorkspace.ubA[26] -= acadoWorkspace.pacA01Dx0[26];
acadoWorkspace.ubA[27] -= acadoWorkspace.pacA01Dx0[27];
acadoWorkspace.ubA[28] -= acadoWorkspace.pacA01Dx0[28];
acadoWorkspace.ubA[29] -= acadoWorkspace.pacA01Dx0[29];
acadoWorkspace.ubA[30] -= acadoWorkspace.pacA01Dx0[30];
acadoWorkspace.ubA[31] -= acadoWorkspace.pacA01Dx0[31];
acadoWorkspace.ubA[32] -= acadoWorkspace.pacA01Dx0[32];
acadoWorkspace.ubA[33] -= acadoWorkspace.pacA01Dx0[33];
acadoWorkspace.ubA[34] -= acadoWorkspace.pacA01Dx0[34];
acadoWorkspace.ubA[35] -= acadoWorkspace.pacA01Dx0[35];
acadoWorkspace.ubA[36] -= acadoWorkspace.pacA01Dx0[36];
acadoWorkspace.ubA[37] -= acadoWorkspace.pacA01Dx0[37];
acadoWorkspace.ubA[38] -= acadoWorkspace.pacA01Dx0[38];
acadoWorkspace.ubA[39] -= acadoWorkspace.pacA01Dx0[39];
acadoWorkspace.ubA[40] -= acadoWorkspace.pacA01Dx0[40];
acadoWorkspace.ubA[41] -= acadoWorkspace.pacA01Dx0[41];
acadoWorkspace.ubA[42] -= acadoWorkspace.pacA01Dx0[42];
acadoWorkspace.ubA[43] -= acadoWorkspace.pacA01Dx0[43];
acadoWorkspace.ubA[44] -= acadoWorkspace.pacA01Dx0[44];
acadoWorkspace.ubA[45] -= acadoWorkspace.pacA01Dx0[45];
acadoWorkspace.ubA[46] -= acadoWorkspace.pacA01Dx0[46];
acadoWorkspace.ubA[47] -= acadoWorkspace.pacA01Dx0[47];
acadoWorkspace.ubA[48] -= acadoWorkspace.pacA01Dx0[48];
acadoWorkspace.ubA[49] -= acadoWorkspace.pacA01Dx0[49];
acadoWorkspace.ubA[50] -= acadoWorkspace.pacA01Dx0[50];
acadoWorkspace.ubA[51] -= acadoWorkspace.pacA01Dx0[51];
acadoWorkspace.ubA[52] -= acadoWorkspace.pacA01Dx0[52];
acadoWorkspace.ubA[53] -= acadoWorkspace.pacA01Dx0[53];
acadoWorkspace.ubA[54] -= acadoWorkspace.pacA01Dx0[54];
acadoWorkspace.ubA[55] -= acadoWorkspace.pacA01Dx0[55];
acadoWorkspace.ubA[56] -= acadoWorkspace.pacA01Dx0[56];
acadoWorkspace.ubA[57] -= acadoWorkspace.pacA01Dx0[57];
acadoWorkspace.ubA[58] -= acadoWorkspace.pacA01Dx0[58];
acadoWorkspace.ubA[59] -= acadoWorkspace.pacA01Dx0[59];
acadoWorkspace.ubA[60] -= acadoWorkspace.pacA01Dx0[60];
acadoWorkspace.ubA[61] -= acadoWorkspace.pacA01Dx0[61];
acadoWorkspace.ubA[62] -= acadoWorkspace.pacA01Dx0[62];
acadoWorkspace.ubA[63] -= acadoWorkspace.pacA01Dx0[63];
acadoWorkspace.ubA[64] -= acadoWorkspace.pacA01Dx0[64];
acadoWorkspace.ubA[65] -= acadoWorkspace.pacA01Dx0[65];
acadoWorkspace.ubA[66] -= acadoWorkspace.pacA01Dx0[66];
acadoWorkspace.ubA[67] -= acadoWorkspace.pacA01Dx0[67];
acadoWorkspace.ubA[68] -= acadoWorkspace.pacA01Dx0[68];
acadoWorkspace.ubA[69] -= acadoWorkspace.pacA01Dx0[69];
acadoWorkspace.ubA[70] -= acadoWorkspace.pacA01Dx0[70];
acadoWorkspace.ubA[71] -= acadoWorkspace.pacA01Dx0[71];
acadoWorkspace.ubA[72] -= acadoWorkspace.pacA01Dx0[72];
acadoWorkspace.ubA[73] -= acadoWorkspace.pacA01Dx0[73];
acadoWorkspace.ubA[74] -= acadoWorkspace.pacA01Dx0[74];
acadoWorkspace.ubA[75] -= acadoWorkspace.pacA01Dx0[75];
acadoWorkspace.ubA[76] -= acadoWorkspace.pacA01Dx0[76];
acadoWorkspace.ubA[77] -= acadoWorkspace.pacA01Dx0[77];
acadoWorkspace.ubA[78] -= acadoWorkspace.pacA01Dx0[78];
acadoWorkspace.ubA[79] -= acadoWorkspace.pacA01Dx0[79];

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];

acadoVariables.x[3] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2];
acadoVariables.x[4] += + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[2];
acadoVariables.x[5] += + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[2];
acadoVariables.x[6] += + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[2];
acadoVariables.x[7] += + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[2];
acadoVariables.x[8] += + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[2];
acadoVariables.x[9] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2];
acadoVariables.x[10] += + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[2];
acadoVariables.x[11] += + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[2];
acadoVariables.x[12] += + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[2];
acadoVariables.x[13] += + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[2];
acadoVariables.x[14] += + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[2];
acadoVariables.x[15] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2];
acadoVariables.x[16] += + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[2];
acadoVariables.x[17] += + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[2];
acadoVariables.x[18] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2];
acadoVariables.x[19] += + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[2];
acadoVariables.x[20] += + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[2];
acadoVariables.x[21] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2];
acadoVariables.x[22] += + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[2];
acadoVariables.x[23] += + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[2];
acadoVariables.x[24] += + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[2];
acadoVariables.x[25] += + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[2];
acadoVariables.x[26] += + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[2];
acadoVariables.x[27] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2];
acadoVariables.x[28] += + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[2];
acadoVariables.x[29] += + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[2];
acadoVariables.x[30] += + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[2];
acadoVariables.x[31] += + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[2];
acadoVariables.x[32] += + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[2];
acadoVariables.x[33] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2];
acadoVariables.x[34] += + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[2];
acadoVariables.x[35] += + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[2];
acadoVariables.x[36] += + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[2];
acadoVariables.x[37] += + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[2];
acadoVariables.x[38] += + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[2];
acadoVariables.x[39] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2];
acadoVariables.x[40] += + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[2];
acadoVariables.x[41] += + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[2];
acadoVariables.x[42] += + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[2];
acadoVariables.x[43] += + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[2];
acadoVariables.x[44] += + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[2];
acadoVariables.x[45] += + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[2];
acadoVariables.x[46] += + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[2];
acadoVariables.x[47] += + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[2];
acadoVariables.x[48] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2];
acadoVariables.x[49] += + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[2];
acadoVariables.x[50] += + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[2];
acadoVariables.x[51] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2];
acadoVariables.x[52] += + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[2];
acadoVariables.x[53] += + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[2];
acadoVariables.x[54] += + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[2];
acadoVariables.x[55] += + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[2];
acadoVariables.x[56] += + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[2];
acadoVariables.x[57] += + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[2];
acadoVariables.x[58] += + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[2];
acadoVariables.x[59] += + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[2];
acadoVariables.x[60] += + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[2];
acadoVariables.x[61] += + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[2];
acadoVariables.x[62] += + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[2];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 3 ]) );
acado_multEDu( &(acadoWorkspace.E[ 3 ]), acadoWorkspace.x, &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 6 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 6 ]) );
acado_multEDu( &(acadoWorkspace.E[ 9 ]), acadoWorkspace.x, &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 12 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 15 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 18 ]), acadoWorkspace.x, &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 21 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 24 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 12 ]) );
acado_multEDu( &(acadoWorkspace.E[ 30 ]), acadoWorkspace.x, &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 33 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 36 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 39 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 42 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 15 ]) );
acado_multEDu( &(acadoWorkspace.E[ 45 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 48 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 51 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 57 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 63 ]), acadoWorkspace.x, &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 66 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 69 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 72 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 75 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 78 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 21 ]) );
acado_multEDu( &(acadoWorkspace.E[ 84 ]), acadoWorkspace.x, &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 87 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 90 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 93 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 96 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 99 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 102 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 105 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 24 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), acadoWorkspace.x, &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 111 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 114 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 117 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 123 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 126 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 129 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 132 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 135 ]), acadoWorkspace.x, &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 138 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 141 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 144 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 147 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 153 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 156 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 159 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 30 ]) );
acado_multEDu( &(acadoWorkspace.E[ 165 ]), acadoWorkspace.x, &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 168 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 171 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 174 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 177 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 180 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 183 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 186 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 192 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 195 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 33 ]) );
acado_multEDu( &(acadoWorkspace.E[ 198 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 201 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 204 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 207 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 213 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 219 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 222 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 225 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 228 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 231 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 234 ]), acadoWorkspace.x, &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 237 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 246 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 249 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 252 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 255 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 258 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 261 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 264 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 267 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 39 ]) );
acado_multEDu( &(acadoWorkspace.E[ 273 ]), acadoWorkspace.x, &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 276 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 279 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 282 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 285 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 288 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 291 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 294 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 303 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 306 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 309 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 312 ]), &(acadoWorkspace.x[ 13 ]), &(acadoVariables.x[ 42 ]) );
acado_multEDu( &(acadoWorkspace.E[ 315 ]), acadoWorkspace.x, &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 318 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 321 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 327 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 330 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 333 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 336 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 339 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 342 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 345 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 348 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 354 ]), &(acadoWorkspace.x[ 13 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 357 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 360 ]), acadoWorkspace.x, &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 363 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 366 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 369 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 372 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 375 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 381 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 384 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 387 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 390 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 393 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 396 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 399 ]), &(acadoWorkspace.x[ 13 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 402 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 48 ]) );
acado_multEDu( &(acadoWorkspace.E[ 408 ]), acadoWorkspace.x, &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 411 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 414 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 417 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 423 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 426 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 429 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 435 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 438 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 441 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 444 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 447 ]), &(acadoWorkspace.x[ 13 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 453 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 456 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 51 ]) );
acado_multEDu( &(acadoWorkspace.E[ 459 ]), acadoWorkspace.x, &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 462 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 465 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 468 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 471 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 474 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 477 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 480 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 483 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 489 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 492 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 495 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 498 ]), &(acadoWorkspace.x[ 13 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 501 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 504 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 507 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 510 ]), &(acadoWorkspace.x[ 17 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 513 ]), acadoWorkspace.x, &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 516 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 519 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 522 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 525 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 528 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 531 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 534 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 537 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 543 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 546 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 549 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 552 ]), &(acadoWorkspace.x[ 13 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 555 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 558 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 561 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 564 ]), &(acadoWorkspace.x[ 17 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 57 ]) );
acado_multEDu( &(acadoWorkspace.E[ 570 ]), acadoWorkspace.x, &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 573 ]), &(acadoWorkspace.x[ 1 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 576 ]), &(acadoWorkspace.x[ 2 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 579 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 582 ]), &(acadoWorkspace.x[ 4 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 585 ]), &(acadoWorkspace.x[ 5 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 588 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 591 ]), &(acadoWorkspace.x[ 7 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.x[ 8 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 597 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 600 ]), &(acadoWorkspace.x[ 10 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 603 ]), &(acadoWorkspace.x[ 11 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 606 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 609 ]), &(acadoWorkspace.x[ 13 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 612 ]), &(acadoWorkspace.x[ 14 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 615 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 618 ]), &(acadoWorkspace.x[ 16 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.x[ 17 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 624 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 60 ]) );
acado_multEDu( &(acadoWorkspace.E[ 627 ]), &(acadoWorkspace.x[ 19 ]), &(acadoVariables.x[ 60 ]) );
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 3];
acadoWorkspace.state[1] = acadoVariables.x[index * 3 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 3 + 2];
acadoWorkspace.state[15] = acadoVariables.u[index];
acadoWorkspace.state[16] = acadoVariables.od[index * 4];
acadoWorkspace.state[17] = acadoVariables.od[index * 4 + 1];
acadoWorkspace.state[18] = acadoVariables.od[index * 4 + 2];
acadoWorkspace.state[19] = acadoVariables.od[index * 4 + 3];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 3 + 3] = acadoWorkspace.state[0];
acadoVariables.x[index * 3 + 4] = acadoWorkspace.state[1];
acadoVariables.x[index * 3 + 5] = acadoWorkspace.state[2];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 3] = acadoVariables.x[index * 3 + 3];
acadoVariables.x[index * 3 + 1] = acadoVariables.x[index * 3 + 4];
acadoVariables.x[index * 3 + 2] = acadoVariables.x[index * 3 + 5];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[60] = xEnd[0];
acadoVariables.x[61] = xEnd[1];
acadoVariables.x[62] = xEnd[2];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[60];
acadoWorkspace.state[1] = acadoVariables.x[61];
acadoWorkspace.state[2] = acadoVariables.x[62];
if (uEnd != 0)
{
acadoWorkspace.state[15] = uEnd[0];
}
else
{
acadoWorkspace.state[15] = acadoVariables.u[19];
}
acadoWorkspace.state[16] = acadoVariables.od[80];
acadoWorkspace.state[17] = acadoVariables.od[81];
acadoWorkspace.state[18] = acadoVariables.od[82];
acadoWorkspace.state[19] = acadoVariables.od[83];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[60] = acadoWorkspace.state[0];
acadoVariables.x[61] = acadoWorkspace.state[1];
acadoVariables.x[62] = acadoWorkspace.state[2];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index] = acadoVariables.u[index + 1];
}

if (uEnd != 0)
{
acadoVariables.u[19] = uEnd[0];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 5 */
real_t tmpDy[ 5 ];

/** Row vector of size: 3 */
real_t tmpDyN[ 3 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 3];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.u[lRun1];
acadoWorkspace.objValueIn[4] = acadoVariables.od[lRun1 * 4];
acadoWorkspace.objValueIn[5] = acadoVariables.od[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[6] = acadoVariables.od[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[7] = acadoVariables.od[lRun1 * 4 + 3];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 5] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 5];
acadoWorkspace.Dy[lRun1 * 5 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 5 + 1];
acadoWorkspace.Dy[lRun1 * 5 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 5 + 2];
acadoWorkspace.Dy[lRun1 * 5 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 5 + 3];
acadoWorkspace.Dy[lRun1 * 5 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 5 + 4];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[60];
acadoWorkspace.objValueIn[1] = acadoVariables.x[61];
acadoWorkspace.objValueIn[2] = acadoVariables.x[62];
acadoWorkspace.objValueIn[3] = acadoVariables.od[80];
acadoWorkspace.objValueIn[4] = acadoVariables.od[81];
acadoWorkspace.objValueIn[5] = acadoVariables.od[82];
acadoWorkspace.objValueIn[6] = acadoVariables.od[83];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 5]*(real_t)2.0000000000000000e+00;
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 5 + 1]*(real_t)2.0000000000000000e+00;
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 5 + 2]*(real_t)2.0000000000000000e+00;
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 5 + 3]*(real_t)3.0000000000000000e+00;
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 5 + 4];
objVal += + acadoWorkspace.Dy[lRun1 * 5]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 5 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 5 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 5 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 5 + 4]*tmpDy[4];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)6.0000000000000000e+00;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)6.0000000000000000e+00;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)6.0000000000000000e+00;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2];

objVal *= 0.5;
return objVal;
}

