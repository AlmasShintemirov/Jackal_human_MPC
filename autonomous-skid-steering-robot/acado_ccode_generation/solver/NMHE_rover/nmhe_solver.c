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


#include "nmhe_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int nmhe_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
nmheWorkspace.state[0] = nmheVariables.x[lRun1 * 8];
nmheWorkspace.state[1] = nmheVariables.x[lRun1 * 8 + 1];
nmheWorkspace.state[2] = nmheVariables.x[lRun1 * 8 + 2];
nmheWorkspace.state[3] = nmheVariables.x[lRun1 * 8 + 3];
nmheWorkspace.state[4] = nmheVariables.x[lRun1 * 8 + 4];
nmheWorkspace.state[5] = nmheVariables.x[lRun1 * 8 + 5];
nmheWorkspace.state[6] = nmheVariables.x[lRun1 * 8 + 6];
nmheWorkspace.state[7] = nmheVariables.x[lRun1 * 8 + 7];

nmheWorkspace.state[88] = nmheVariables.u[lRun1 * 2];
nmheWorkspace.state[89] = nmheVariables.u[lRun1 * 2 + 1];

ret = nmhe_integrate(nmheWorkspace.state, 1);

nmheWorkspace.d[lRun1 * 8] = nmheWorkspace.state[0] - nmheVariables.x[lRun1 * 8 + 8];
nmheWorkspace.d[lRun1 * 8 + 1] = nmheWorkspace.state[1] - nmheVariables.x[lRun1 * 8 + 9];
nmheWorkspace.d[lRun1 * 8 + 2] = nmheWorkspace.state[2] - nmheVariables.x[lRun1 * 8 + 10];
nmheWorkspace.d[lRun1 * 8 + 3] = nmheWorkspace.state[3] - nmheVariables.x[lRun1 * 8 + 11];
nmheWorkspace.d[lRun1 * 8 + 4] = nmheWorkspace.state[4] - nmheVariables.x[lRun1 * 8 + 12];
nmheWorkspace.d[lRun1 * 8 + 5] = nmheWorkspace.state[5] - nmheVariables.x[lRun1 * 8 + 13];
nmheWorkspace.d[lRun1 * 8 + 6] = nmheWorkspace.state[6] - nmheVariables.x[lRun1 * 8 + 14];
nmheWorkspace.d[lRun1 * 8 + 7] = nmheWorkspace.state[7] - nmheVariables.x[lRun1 * 8 + 15];

nmheWorkspace.evGx[lRun1 * 64] = nmheWorkspace.state[8];
nmheWorkspace.evGx[lRun1 * 64 + 1] = nmheWorkspace.state[9];
nmheWorkspace.evGx[lRun1 * 64 + 2] = nmheWorkspace.state[10];
nmheWorkspace.evGx[lRun1 * 64 + 3] = nmheWorkspace.state[11];
nmheWorkspace.evGx[lRun1 * 64 + 4] = nmheWorkspace.state[12];
nmheWorkspace.evGx[lRun1 * 64 + 5] = nmheWorkspace.state[13];
nmheWorkspace.evGx[lRun1 * 64 + 6] = nmheWorkspace.state[14];
nmheWorkspace.evGx[lRun1 * 64 + 7] = nmheWorkspace.state[15];
nmheWorkspace.evGx[lRun1 * 64 + 8] = nmheWorkspace.state[16];
nmheWorkspace.evGx[lRun1 * 64 + 9] = nmheWorkspace.state[17];
nmheWorkspace.evGx[lRun1 * 64 + 10] = nmheWorkspace.state[18];
nmheWorkspace.evGx[lRun1 * 64 + 11] = nmheWorkspace.state[19];
nmheWorkspace.evGx[lRun1 * 64 + 12] = nmheWorkspace.state[20];
nmheWorkspace.evGx[lRun1 * 64 + 13] = nmheWorkspace.state[21];
nmheWorkspace.evGx[lRun1 * 64 + 14] = nmheWorkspace.state[22];
nmheWorkspace.evGx[lRun1 * 64 + 15] = nmheWorkspace.state[23];
nmheWorkspace.evGx[lRun1 * 64 + 16] = nmheWorkspace.state[24];
nmheWorkspace.evGx[lRun1 * 64 + 17] = nmheWorkspace.state[25];
nmheWorkspace.evGx[lRun1 * 64 + 18] = nmheWorkspace.state[26];
nmheWorkspace.evGx[lRun1 * 64 + 19] = nmheWorkspace.state[27];
nmheWorkspace.evGx[lRun1 * 64 + 20] = nmheWorkspace.state[28];
nmheWorkspace.evGx[lRun1 * 64 + 21] = nmheWorkspace.state[29];
nmheWorkspace.evGx[lRun1 * 64 + 22] = nmheWorkspace.state[30];
nmheWorkspace.evGx[lRun1 * 64 + 23] = nmheWorkspace.state[31];
nmheWorkspace.evGx[lRun1 * 64 + 24] = nmheWorkspace.state[32];
nmheWorkspace.evGx[lRun1 * 64 + 25] = nmheWorkspace.state[33];
nmheWorkspace.evGx[lRun1 * 64 + 26] = nmheWorkspace.state[34];
nmheWorkspace.evGx[lRun1 * 64 + 27] = nmheWorkspace.state[35];
nmheWorkspace.evGx[lRun1 * 64 + 28] = nmheWorkspace.state[36];
nmheWorkspace.evGx[lRun1 * 64 + 29] = nmheWorkspace.state[37];
nmheWorkspace.evGx[lRun1 * 64 + 30] = nmheWorkspace.state[38];
nmheWorkspace.evGx[lRun1 * 64 + 31] = nmheWorkspace.state[39];
nmheWorkspace.evGx[lRun1 * 64 + 32] = nmheWorkspace.state[40];
nmheWorkspace.evGx[lRun1 * 64 + 33] = nmheWorkspace.state[41];
nmheWorkspace.evGx[lRun1 * 64 + 34] = nmheWorkspace.state[42];
nmheWorkspace.evGx[lRun1 * 64 + 35] = nmheWorkspace.state[43];
nmheWorkspace.evGx[lRun1 * 64 + 36] = nmheWorkspace.state[44];
nmheWorkspace.evGx[lRun1 * 64 + 37] = nmheWorkspace.state[45];
nmheWorkspace.evGx[lRun1 * 64 + 38] = nmheWorkspace.state[46];
nmheWorkspace.evGx[lRun1 * 64 + 39] = nmheWorkspace.state[47];
nmheWorkspace.evGx[lRun1 * 64 + 40] = nmheWorkspace.state[48];
nmheWorkspace.evGx[lRun1 * 64 + 41] = nmheWorkspace.state[49];
nmheWorkspace.evGx[lRun1 * 64 + 42] = nmheWorkspace.state[50];
nmheWorkspace.evGx[lRun1 * 64 + 43] = nmheWorkspace.state[51];
nmheWorkspace.evGx[lRun1 * 64 + 44] = nmheWorkspace.state[52];
nmheWorkspace.evGx[lRun1 * 64 + 45] = nmheWorkspace.state[53];
nmheWorkspace.evGx[lRun1 * 64 + 46] = nmheWorkspace.state[54];
nmheWorkspace.evGx[lRun1 * 64 + 47] = nmheWorkspace.state[55];
nmheWorkspace.evGx[lRun1 * 64 + 48] = nmheWorkspace.state[56];
nmheWorkspace.evGx[lRun1 * 64 + 49] = nmheWorkspace.state[57];
nmheWorkspace.evGx[lRun1 * 64 + 50] = nmheWorkspace.state[58];
nmheWorkspace.evGx[lRun1 * 64 + 51] = nmheWorkspace.state[59];
nmheWorkspace.evGx[lRun1 * 64 + 52] = nmheWorkspace.state[60];
nmheWorkspace.evGx[lRun1 * 64 + 53] = nmheWorkspace.state[61];
nmheWorkspace.evGx[lRun1 * 64 + 54] = nmheWorkspace.state[62];
nmheWorkspace.evGx[lRun1 * 64 + 55] = nmheWorkspace.state[63];
nmheWorkspace.evGx[lRun1 * 64 + 56] = nmheWorkspace.state[64];
nmheWorkspace.evGx[lRun1 * 64 + 57] = nmheWorkspace.state[65];
nmheWorkspace.evGx[lRun1 * 64 + 58] = nmheWorkspace.state[66];
nmheWorkspace.evGx[lRun1 * 64 + 59] = nmheWorkspace.state[67];
nmheWorkspace.evGx[lRun1 * 64 + 60] = nmheWorkspace.state[68];
nmheWorkspace.evGx[lRun1 * 64 + 61] = nmheWorkspace.state[69];
nmheWorkspace.evGx[lRun1 * 64 + 62] = nmheWorkspace.state[70];
nmheWorkspace.evGx[lRun1 * 64 + 63] = nmheWorkspace.state[71];

nmheWorkspace.evGu[lRun1 * 16] = nmheWorkspace.state[72];
nmheWorkspace.evGu[lRun1 * 16 + 1] = nmheWorkspace.state[73];
nmheWorkspace.evGu[lRun1 * 16 + 2] = nmheWorkspace.state[74];
nmheWorkspace.evGu[lRun1 * 16 + 3] = nmheWorkspace.state[75];
nmheWorkspace.evGu[lRun1 * 16 + 4] = nmheWorkspace.state[76];
nmheWorkspace.evGu[lRun1 * 16 + 5] = nmheWorkspace.state[77];
nmheWorkspace.evGu[lRun1 * 16 + 6] = nmheWorkspace.state[78];
nmheWorkspace.evGu[lRun1 * 16 + 7] = nmheWorkspace.state[79];
nmheWorkspace.evGu[lRun1 * 16 + 8] = nmheWorkspace.state[80];
nmheWorkspace.evGu[lRun1 * 16 + 9] = nmheWorkspace.state[81];
nmheWorkspace.evGu[lRun1 * 16 + 10] = nmheWorkspace.state[82];
nmheWorkspace.evGu[lRun1 * 16 + 11] = nmheWorkspace.state[83];
nmheWorkspace.evGu[lRun1 * 16 + 12] = nmheWorkspace.state[84];
nmheWorkspace.evGu[lRun1 * 16 + 13] = nmheWorkspace.state[85];
nmheWorkspace.evGu[lRun1 * 16 + 14] = nmheWorkspace.state[86];
nmheWorkspace.evGu[lRun1 * 16 + 15] = nmheWorkspace.state[87];
}
return ret;
}

void nmhe_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 8;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = u[0];
out[7] = u[1];
}

void nmhe_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
}

void nmhe_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = 0.0;
;
tmpQ2[49] = 0.0;
;
tmpQ2[50] = 0.0;
;
tmpQ2[51] = 0.0;
;
tmpQ2[52] = 0.0;
;
tmpQ2[53] = 0.0;
;
tmpQ2[54] = 0.0;
;
tmpQ2[55] = 0.0;
;
tmpQ2[56] = 0.0;
;
tmpQ2[57] = 0.0;
;
tmpQ2[58] = 0.0;
;
tmpQ2[59] = 0.0;
;
tmpQ2[60] = 0.0;
;
tmpQ2[61] = 0.0;
;
tmpQ2[62] = 0.0;
;
tmpQ2[63] = 0.0;
;
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = 0.0;
;
tmpQ1[7] = 0.0;
;
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = + tmpQ2[9];
tmpQ1[10] = + tmpQ2[10];
tmpQ1[11] = + tmpQ2[11];
tmpQ1[12] = + tmpQ2[12];
tmpQ1[13] = + tmpQ2[13];
tmpQ1[14] = 0.0;
;
tmpQ1[15] = 0.0;
;
tmpQ1[16] = + tmpQ2[16];
tmpQ1[17] = + tmpQ2[17];
tmpQ1[18] = + tmpQ2[18];
tmpQ1[19] = + tmpQ2[19];
tmpQ1[20] = + tmpQ2[20];
tmpQ1[21] = + tmpQ2[21];
tmpQ1[22] = 0.0;
;
tmpQ1[23] = 0.0;
;
tmpQ1[24] = + tmpQ2[24];
tmpQ1[25] = + tmpQ2[25];
tmpQ1[26] = + tmpQ2[26];
tmpQ1[27] = + tmpQ2[27];
tmpQ1[28] = + tmpQ2[28];
tmpQ1[29] = + tmpQ2[29];
tmpQ1[30] = 0.0;
;
tmpQ1[31] = 0.0;
;
tmpQ1[32] = + tmpQ2[32];
tmpQ1[33] = + tmpQ2[33];
tmpQ1[34] = + tmpQ2[34];
tmpQ1[35] = + tmpQ2[35];
tmpQ1[36] = + tmpQ2[36];
tmpQ1[37] = + tmpQ2[37];
tmpQ1[38] = 0.0;
;
tmpQ1[39] = 0.0;
;
tmpQ1[40] = + tmpQ2[40];
tmpQ1[41] = + tmpQ2[41];
tmpQ1[42] = + tmpQ2[42];
tmpQ1[43] = + tmpQ2[43];
tmpQ1[44] = + tmpQ2[44];
tmpQ1[45] = + tmpQ2[45];
tmpQ1[46] = 0.0;
;
tmpQ1[47] = 0.0;
;
tmpQ1[48] = + tmpQ2[48];
tmpQ1[49] = + tmpQ2[49];
tmpQ1[50] = + tmpQ2[50];
tmpQ1[51] = + tmpQ2[51];
tmpQ1[52] = + tmpQ2[52];
tmpQ1[53] = + tmpQ2[53];
tmpQ1[54] = 0.0;
;
tmpQ1[55] = 0.0;
;
tmpQ1[56] = + tmpQ2[56];
tmpQ1[57] = + tmpQ2[57];
tmpQ1[58] = + tmpQ2[58];
tmpQ1[59] = + tmpQ2[59];
tmpQ1[60] = + tmpQ2[60];
tmpQ1[61] = + tmpQ2[61];
tmpQ1[62] = 0.0;
;
tmpQ1[63] = 0.0;
;
}

void nmhe_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[48];
tmpR2[1] = +tmpObjS[49];
tmpR2[2] = +tmpObjS[50];
tmpR2[3] = +tmpObjS[51];
tmpR2[4] = +tmpObjS[52];
tmpR2[5] = +tmpObjS[53];
tmpR2[6] = +tmpObjS[54];
tmpR2[7] = +tmpObjS[55];
tmpR2[8] = +tmpObjS[56];
tmpR2[9] = +tmpObjS[57];
tmpR2[10] = +tmpObjS[58];
tmpR2[11] = +tmpObjS[59];
tmpR2[12] = +tmpObjS[60];
tmpR2[13] = +tmpObjS[61];
tmpR2[14] = +tmpObjS[62];
tmpR2[15] = +tmpObjS[63];
tmpR1[0] = + tmpR2[6];
tmpR1[1] = + tmpR2[7];
tmpR1[2] = + tmpR2[14];
tmpR1[3] = + tmpR2[15];
}

void nmhe_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = 0.0;
;
tmpQN2[37] = 0.0;
;
tmpQN2[38] = 0.0;
;
tmpQN2[39] = 0.0;
;
tmpQN2[40] = 0.0;
;
tmpQN2[41] = 0.0;
;
tmpQN2[42] = 0.0;
;
tmpQN2[43] = 0.0;
;
tmpQN2[44] = 0.0;
;
tmpQN2[45] = 0.0;
;
tmpQN2[46] = 0.0;
;
tmpQN2[47] = 0.0;
;
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = 0.0;
;
tmpQN1[7] = 0.0;
;
tmpQN1[8] = + tmpQN2[6];
tmpQN1[9] = + tmpQN2[7];
tmpQN1[10] = + tmpQN2[8];
tmpQN1[11] = + tmpQN2[9];
tmpQN1[12] = + tmpQN2[10];
tmpQN1[13] = + tmpQN2[11];
tmpQN1[14] = 0.0;
;
tmpQN1[15] = 0.0;
;
tmpQN1[16] = + tmpQN2[12];
tmpQN1[17] = + tmpQN2[13];
tmpQN1[18] = + tmpQN2[14];
tmpQN1[19] = + tmpQN2[15];
tmpQN1[20] = + tmpQN2[16];
tmpQN1[21] = + tmpQN2[17];
tmpQN1[22] = 0.0;
;
tmpQN1[23] = 0.0;
;
tmpQN1[24] = + tmpQN2[18];
tmpQN1[25] = + tmpQN2[19];
tmpQN1[26] = + tmpQN2[20];
tmpQN1[27] = + tmpQN2[21];
tmpQN1[28] = + tmpQN2[22];
tmpQN1[29] = + tmpQN2[23];
tmpQN1[30] = 0.0;
;
tmpQN1[31] = 0.0;
;
tmpQN1[32] = + tmpQN2[24];
tmpQN1[33] = + tmpQN2[25];
tmpQN1[34] = + tmpQN2[26];
tmpQN1[35] = + tmpQN2[27];
tmpQN1[36] = + tmpQN2[28];
tmpQN1[37] = + tmpQN2[29];
tmpQN1[38] = 0.0;
;
tmpQN1[39] = 0.0;
;
tmpQN1[40] = + tmpQN2[30];
tmpQN1[41] = + tmpQN2[31];
tmpQN1[42] = + tmpQN2[32];
tmpQN1[43] = + tmpQN2[33];
tmpQN1[44] = + tmpQN2[34];
tmpQN1[45] = + tmpQN2[35];
tmpQN1[46] = 0.0;
;
tmpQN1[47] = 0.0;
;
tmpQN1[48] = + tmpQN2[36];
tmpQN1[49] = + tmpQN2[37];
tmpQN1[50] = + tmpQN2[38];
tmpQN1[51] = + tmpQN2[39];
tmpQN1[52] = + tmpQN2[40];
tmpQN1[53] = + tmpQN2[41];
tmpQN1[54] = 0.0;
;
tmpQN1[55] = 0.0;
;
tmpQN1[56] = + tmpQN2[42];
tmpQN1[57] = + tmpQN2[43];
tmpQN1[58] = + tmpQN2[44];
tmpQN1[59] = + tmpQN2[45];
tmpQN1[60] = + tmpQN2[46];
tmpQN1[61] = + tmpQN2[47];
tmpQN1[62] = 0.0;
;
tmpQN1[63] = 0.0;
;
}

void nmhe_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 100; ++runObj)
{
nmheWorkspace.objValueIn[0] = nmheVariables.x[runObj * 8];
nmheWorkspace.objValueIn[1] = nmheVariables.x[runObj * 8 + 1];
nmheWorkspace.objValueIn[2] = nmheVariables.x[runObj * 8 + 2];
nmheWorkspace.objValueIn[3] = nmheVariables.x[runObj * 8 + 3];
nmheWorkspace.objValueIn[4] = nmheVariables.x[runObj * 8 + 4];
nmheWorkspace.objValueIn[5] = nmheVariables.x[runObj * 8 + 5];
nmheWorkspace.objValueIn[6] = nmheVariables.x[runObj * 8 + 6];
nmheWorkspace.objValueIn[7] = nmheVariables.x[runObj * 8 + 7];
nmheWorkspace.objValueIn[8] = nmheVariables.u[runObj * 2];
nmheWorkspace.objValueIn[9] = nmheVariables.u[runObj * 2 + 1];

nmhe_evaluateLSQ( nmheWorkspace.objValueIn, nmheWorkspace.objValueOut );
nmheWorkspace.Dy[runObj * 8] = nmheWorkspace.objValueOut[0];
nmheWorkspace.Dy[runObj * 8 + 1] = nmheWorkspace.objValueOut[1];
nmheWorkspace.Dy[runObj * 8 + 2] = nmheWorkspace.objValueOut[2];
nmheWorkspace.Dy[runObj * 8 + 3] = nmheWorkspace.objValueOut[3];
nmheWorkspace.Dy[runObj * 8 + 4] = nmheWorkspace.objValueOut[4];
nmheWorkspace.Dy[runObj * 8 + 5] = nmheWorkspace.objValueOut[5];
nmheWorkspace.Dy[runObj * 8 + 6] = nmheWorkspace.objValueOut[6];
nmheWorkspace.Dy[runObj * 8 + 7] = nmheWorkspace.objValueOut[7];

nmhe_setObjQ1Q2( nmheVariables.W, &(nmheWorkspace.Q1[ runObj * 64 ]), &(nmheWorkspace.Q2[ runObj * 64 ]) );

nmhe_setObjR1R2( nmheVariables.W, &(nmheWorkspace.R1[ runObj * 4 ]), &(nmheWorkspace.R2[ runObj * 16 ]) );

}
nmheWorkspace.objValueIn[0] = nmheVariables.x[800];
nmheWorkspace.objValueIn[1] = nmheVariables.x[801];
nmheWorkspace.objValueIn[2] = nmheVariables.x[802];
nmheWorkspace.objValueIn[3] = nmheVariables.x[803];
nmheWorkspace.objValueIn[4] = nmheVariables.x[804];
nmheWorkspace.objValueIn[5] = nmheVariables.x[805];
nmheWorkspace.objValueIn[6] = nmheVariables.x[806];
nmheWorkspace.objValueIn[7] = nmheVariables.x[807];
nmhe_evaluateLSQEndTerm( nmheWorkspace.objValueIn, nmheWorkspace.objValueOut );

nmheWorkspace.DyN[0] = nmheWorkspace.objValueOut[0];
nmheWorkspace.DyN[1] = nmheWorkspace.objValueOut[1];
nmheWorkspace.DyN[2] = nmheWorkspace.objValueOut[2];
nmheWorkspace.DyN[3] = nmheWorkspace.objValueOut[3];
nmheWorkspace.DyN[4] = nmheWorkspace.objValueOut[4];
nmheWorkspace.DyN[5] = nmheWorkspace.objValueOut[5];

nmhe_setObjQN1QN2( nmheVariables.WN, nmheWorkspace.QN1, nmheWorkspace.QN2 );

}

void nmhe_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7];
dNew[1] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3] + Gx1[12]*dOld[4] + Gx1[13]*dOld[5] + Gx1[14]*dOld[6] + Gx1[15]*dOld[7];
dNew[2] += + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7];
dNew[3] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7];
dNew[4] += + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7];
dNew[5] += + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7];
dNew[6] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7];
dNew[7] += + Gx1[56]*dOld[0] + Gx1[57]*dOld[1] + Gx1[58]*dOld[2] + Gx1[59]*dOld[3] + Gx1[60]*dOld[4] + Gx1[61]*dOld[5] + Gx1[62]*dOld[6] + Gx1[63]*dOld[7];
}

void nmhe_moveGxT( real_t* const Gx1, real_t* const Gx2 )
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
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
}

void nmhe_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[24] + Gx1[4]*Gx2[32] + Gx1[5]*Gx2[40] + Gx1[6]*Gx2[48] + Gx1[7]*Gx2[56];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[25] + Gx1[4]*Gx2[33] + Gx1[5]*Gx2[41] + Gx1[6]*Gx2[49] + Gx1[7]*Gx2[57];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[26] + Gx1[4]*Gx2[34] + Gx1[5]*Gx2[42] + Gx1[6]*Gx2[50] + Gx1[7]*Gx2[58];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[35] + Gx1[5]*Gx2[43] + Gx1[6]*Gx2[51] + Gx1[7]*Gx2[59];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[28] + Gx1[4]*Gx2[36] + Gx1[5]*Gx2[44] + Gx1[6]*Gx2[52] + Gx1[7]*Gx2[60];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[29] + Gx1[4]*Gx2[37] + Gx1[5]*Gx2[45] + Gx1[6]*Gx2[53] + Gx1[7]*Gx2[61];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[38] + Gx1[5]*Gx2[46] + Gx1[6]*Gx2[54] + Gx1[7]*Gx2[62];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[39] + Gx1[5]*Gx2[47] + Gx1[6]*Gx2[55] + Gx1[7]*Gx2[63];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[10]*Gx2[16] + Gx1[11]*Gx2[24] + Gx1[12]*Gx2[32] + Gx1[13]*Gx2[40] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[56];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[10]*Gx2[17] + Gx1[11]*Gx2[25] + Gx1[12]*Gx2[33] + Gx1[13]*Gx2[41] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[57];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[10]*Gx2[18] + Gx1[11]*Gx2[26] + Gx1[12]*Gx2[34] + Gx1[13]*Gx2[42] + Gx1[14]*Gx2[50] + Gx1[15]*Gx2[58];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[10]*Gx2[19] + Gx1[11]*Gx2[27] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[43] + Gx1[14]*Gx2[51] + Gx1[15]*Gx2[59];
Gx3[12] = + Gx1[8]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[10]*Gx2[20] + Gx1[11]*Gx2[28] + Gx1[12]*Gx2[36] + Gx1[13]*Gx2[44] + Gx1[14]*Gx2[52] + Gx1[15]*Gx2[60];
Gx3[13] = + Gx1[8]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[10]*Gx2[21] + Gx1[11]*Gx2[29] + Gx1[12]*Gx2[37] + Gx1[13]*Gx2[45] + Gx1[14]*Gx2[53] + Gx1[15]*Gx2[61];
Gx3[14] = + Gx1[8]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[10]*Gx2[22] + Gx1[11]*Gx2[30] + Gx1[12]*Gx2[38] + Gx1[13]*Gx2[46] + Gx1[14]*Gx2[54] + Gx1[15]*Gx2[62];
Gx3[15] = + Gx1[8]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[10]*Gx2[23] + Gx1[11]*Gx2[31] + Gx1[12]*Gx2[39] + Gx1[13]*Gx2[47] + Gx1[14]*Gx2[55] + Gx1[15]*Gx2[63];
Gx3[16] = + Gx1[16]*Gx2[0] + Gx1[17]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[24] + Gx1[20]*Gx2[32] + Gx1[21]*Gx2[40] + Gx1[22]*Gx2[48] + Gx1[23]*Gx2[56];
Gx3[17] = + Gx1[16]*Gx2[1] + Gx1[17]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[25] + Gx1[20]*Gx2[33] + Gx1[21]*Gx2[41] + Gx1[22]*Gx2[49] + Gx1[23]*Gx2[57];
Gx3[18] = + Gx1[16]*Gx2[2] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[26] + Gx1[20]*Gx2[34] + Gx1[21]*Gx2[42] + Gx1[22]*Gx2[50] + Gx1[23]*Gx2[58];
Gx3[19] = + Gx1[16]*Gx2[3] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[27] + Gx1[20]*Gx2[35] + Gx1[21]*Gx2[43] + Gx1[22]*Gx2[51] + Gx1[23]*Gx2[59];
Gx3[20] = + Gx1[16]*Gx2[4] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[19]*Gx2[28] + Gx1[20]*Gx2[36] + Gx1[21]*Gx2[44] + Gx1[22]*Gx2[52] + Gx1[23]*Gx2[60];
Gx3[21] = + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[19]*Gx2[29] + Gx1[20]*Gx2[37] + Gx1[21]*Gx2[45] + Gx1[22]*Gx2[53] + Gx1[23]*Gx2[61];
Gx3[22] = + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[19]*Gx2[30] + Gx1[20]*Gx2[38] + Gx1[21]*Gx2[46] + Gx1[22]*Gx2[54] + Gx1[23]*Gx2[62];
Gx3[23] = + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[19]*Gx2[31] + Gx1[20]*Gx2[39] + Gx1[21]*Gx2[47] + Gx1[22]*Gx2[55] + Gx1[23]*Gx2[63];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[28]*Gx2[32] + Gx1[29]*Gx2[40] + Gx1[30]*Gx2[48] + Gx1[31]*Gx2[56];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[28]*Gx2[33] + Gx1[29]*Gx2[41] + Gx1[30]*Gx2[49] + Gx1[31]*Gx2[57];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[28]*Gx2[34] + Gx1[29]*Gx2[42] + Gx1[30]*Gx2[50] + Gx1[31]*Gx2[58];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[28]*Gx2[35] + Gx1[29]*Gx2[43] + Gx1[30]*Gx2[51] + Gx1[31]*Gx2[59];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[28]*Gx2[36] + Gx1[29]*Gx2[44] + Gx1[30]*Gx2[52] + Gx1[31]*Gx2[60];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[28]*Gx2[37] + Gx1[29]*Gx2[45] + Gx1[30]*Gx2[53] + Gx1[31]*Gx2[61];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[28]*Gx2[38] + Gx1[29]*Gx2[46] + Gx1[30]*Gx2[54] + Gx1[31]*Gx2[62];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[28]*Gx2[39] + Gx1[29]*Gx2[47] + Gx1[30]*Gx2[55] + Gx1[31]*Gx2[63];
Gx3[32] = + Gx1[32]*Gx2[0] + Gx1[33]*Gx2[8] + Gx1[34]*Gx2[16] + Gx1[35]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[37]*Gx2[40] + Gx1[38]*Gx2[48] + Gx1[39]*Gx2[56];
Gx3[33] = + Gx1[32]*Gx2[1] + Gx1[33]*Gx2[9] + Gx1[34]*Gx2[17] + Gx1[35]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[37]*Gx2[41] + Gx1[38]*Gx2[49] + Gx1[39]*Gx2[57];
Gx3[34] = + Gx1[32]*Gx2[2] + Gx1[33]*Gx2[10] + Gx1[34]*Gx2[18] + Gx1[35]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[37]*Gx2[42] + Gx1[38]*Gx2[50] + Gx1[39]*Gx2[58];
Gx3[35] = + Gx1[32]*Gx2[3] + Gx1[33]*Gx2[11] + Gx1[34]*Gx2[19] + Gx1[35]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[37]*Gx2[43] + Gx1[38]*Gx2[51] + Gx1[39]*Gx2[59];
Gx3[36] = + Gx1[32]*Gx2[4] + Gx1[33]*Gx2[12] + Gx1[34]*Gx2[20] + Gx1[35]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[37]*Gx2[44] + Gx1[38]*Gx2[52] + Gx1[39]*Gx2[60];
Gx3[37] = + Gx1[32]*Gx2[5] + Gx1[33]*Gx2[13] + Gx1[34]*Gx2[21] + Gx1[35]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[37]*Gx2[45] + Gx1[38]*Gx2[53] + Gx1[39]*Gx2[61];
Gx3[38] = + Gx1[32]*Gx2[6] + Gx1[33]*Gx2[14] + Gx1[34]*Gx2[22] + Gx1[35]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[37]*Gx2[46] + Gx1[38]*Gx2[54] + Gx1[39]*Gx2[62];
Gx3[39] = + Gx1[32]*Gx2[7] + Gx1[33]*Gx2[15] + Gx1[34]*Gx2[23] + Gx1[35]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[37]*Gx2[47] + Gx1[38]*Gx2[55] + Gx1[39]*Gx2[63];
Gx3[40] = + Gx1[40]*Gx2[0] + Gx1[41]*Gx2[8] + Gx1[42]*Gx2[16] + Gx1[43]*Gx2[24] + Gx1[44]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[46]*Gx2[48] + Gx1[47]*Gx2[56];
Gx3[41] = + Gx1[40]*Gx2[1] + Gx1[41]*Gx2[9] + Gx1[42]*Gx2[17] + Gx1[43]*Gx2[25] + Gx1[44]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[46]*Gx2[49] + Gx1[47]*Gx2[57];
Gx3[42] = + Gx1[40]*Gx2[2] + Gx1[41]*Gx2[10] + Gx1[42]*Gx2[18] + Gx1[43]*Gx2[26] + Gx1[44]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[46]*Gx2[50] + Gx1[47]*Gx2[58];
Gx3[43] = + Gx1[40]*Gx2[3] + Gx1[41]*Gx2[11] + Gx1[42]*Gx2[19] + Gx1[43]*Gx2[27] + Gx1[44]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[46]*Gx2[51] + Gx1[47]*Gx2[59];
Gx3[44] = + Gx1[40]*Gx2[4] + Gx1[41]*Gx2[12] + Gx1[42]*Gx2[20] + Gx1[43]*Gx2[28] + Gx1[44]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[46]*Gx2[52] + Gx1[47]*Gx2[60];
Gx3[45] = + Gx1[40]*Gx2[5] + Gx1[41]*Gx2[13] + Gx1[42]*Gx2[21] + Gx1[43]*Gx2[29] + Gx1[44]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[46]*Gx2[53] + Gx1[47]*Gx2[61];
Gx3[46] = + Gx1[40]*Gx2[6] + Gx1[41]*Gx2[14] + Gx1[42]*Gx2[22] + Gx1[43]*Gx2[30] + Gx1[44]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[46]*Gx2[54] + Gx1[47]*Gx2[62];
Gx3[47] = + Gx1[40]*Gx2[7] + Gx1[41]*Gx2[15] + Gx1[42]*Gx2[23] + Gx1[43]*Gx2[31] + Gx1[44]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[46]*Gx2[55] + Gx1[47]*Gx2[63];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[8] + Gx1[50]*Gx2[16] + Gx1[51]*Gx2[24] + Gx1[52]*Gx2[32] + Gx1[53]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[55]*Gx2[56];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[9] + Gx1[50]*Gx2[17] + Gx1[51]*Gx2[25] + Gx1[52]*Gx2[33] + Gx1[53]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[55]*Gx2[57];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[10] + Gx1[50]*Gx2[18] + Gx1[51]*Gx2[26] + Gx1[52]*Gx2[34] + Gx1[53]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[55]*Gx2[58];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[11] + Gx1[50]*Gx2[19] + Gx1[51]*Gx2[27] + Gx1[52]*Gx2[35] + Gx1[53]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[55]*Gx2[59];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[20] + Gx1[51]*Gx2[28] + Gx1[52]*Gx2[36] + Gx1[53]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[55]*Gx2[60];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[21] + Gx1[51]*Gx2[29] + Gx1[52]*Gx2[37] + Gx1[53]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[55]*Gx2[61];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[22] + Gx1[51]*Gx2[30] + Gx1[52]*Gx2[38] + Gx1[53]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[55]*Gx2[62];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[23] + Gx1[51]*Gx2[31] + Gx1[52]*Gx2[39] + Gx1[53]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[55]*Gx2[63];
Gx3[56] = + Gx1[56]*Gx2[0] + Gx1[57]*Gx2[8] + Gx1[58]*Gx2[16] + Gx1[59]*Gx2[24] + Gx1[60]*Gx2[32] + Gx1[61]*Gx2[40] + Gx1[62]*Gx2[48] + Gx1[63]*Gx2[56];
Gx3[57] = + Gx1[56]*Gx2[1] + Gx1[57]*Gx2[9] + Gx1[58]*Gx2[17] + Gx1[59]*Gx2[25] + Gx1[60]*Gx2[33] + Gx1[61]*Gx2[41] + Gx1[62]*Gx2[49] + Gx1[63]*Gx2[57];
Gx3[58] = + Gx1[56]*Gx2[2] + Gx1[57]*Gx2[10] + Gx1[58]*Gx2[18] + Gx1[59]*Gx2[26] + Gx1[60]*Gx2[34] + Gx1[61]*Gx2[42] + Gx1[62]*Gx2[50] + Gx1[63]*Gx2[58];
Gx3[59] = + Gx1[56]*Gx2[3] + Gx1[57]*Gx2[11] + Gx1[58]*Gx2[19] + Gx1[59]*Gx2[27] + Gx1[60]*Gx2[35] + Gx1[61]*Gx2[43] + Gx1[62]*Gx2[51] + Gx1[63]*Gx2[59];
Gx3[60] = + Gx1[56]*Gx2[4] + Gx1[57]*Gx2[12] + Gx1[58]*Gx2[20] + Gx1[59]*Gx2[28] + Gx1[60]*Gx2[36] + Gx1[61]*Gx2[44] + Gx1[62]*Gx2[52] + Gx1[63]*Gx2[60];
Gx3[61] = + Gx1[56]*Gx2[5] + Gx1[57]*Gx2[13] + Gx1[58]*Gx2[21] + Gx1[59]*Gx2[29] + Gx1[60]*Gx2[37] + Gx1[61]*Gx2[45] + Gx1[62]*Gx2[53] + Gx1[63]*Gx2[61];
Gx3[62] = + Gx1[56]*Gx2[6] + Gx1[57]*Gx2[14] + Gx1[58]*Gx2[22] + Gx1[59]*Gx2[30] + Gx1[60]*Gx2[38] + Gx1[61]*Gx2[46] + Gx1[62]*Gx2[54] + Gx1[63]*Gx2[62];
Gx3[63] = + Gx1[56]*Gx2[7] + Gx1[57]*Gx2[15] + Gx1[58]*Gx2[23] + Gx1[59]*Gx2[31] + Gx1[60]*Gx2[39] + Gx1[61]*Gx2[47] + Gx1[62]*Gx2[55] + Gx1[63]*Gx2[63];
}

void nmhe_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8] + Gx1[5]*Gu1[10] + Gx1[6]*Gu1[12] + Gx1[7]*Gu1[14];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9] + Gx1[5]*Gu1[11] + Gx1[6]*Gu1[13] + Gx1[7]*Gu1[15];
Gu2[2] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[8] + Gx1[13]*Gu1[10] + Gx1[14]*Gu1[12] + Gx1[15]*Gu1[14];
Gu2[3] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[11] + Gx1[14]*Gu1[13] + Gx1[15]*Gu1[15];
Gu2[4] = + Gx1[16]*Gu1[0] + Gx1[17]*Gu1[2] + Gx1[18]*Gu1[4] + Gx1[19]*Gu1[6] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[14];
Gu2[5] = + Gx1[16]*Gu1[1] + Gx1[17]*Gu1[3] + Gx1[18]*Gu1[5] + Gx1[19]*Gu1[7] + Gx1[20]*Gu1[9] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[15];
Gu2[6] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[2] + Gx1[26]*Gu1[4] + Gx1[27]*Gu1[6] + Gx1[28]*Gu1[8] + Gx1[29]*Gu1[10] + Gx1[30]*Gu1[12] + Gx1[31]*Gu1[14];
Gu2[7] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[3] + Gx1[26]*Gu1[5] + Gx1[27]*Gu1[7] + Gx1[28]*Gu1[9] + Gx1[29]*Gu1[11] + Gx1[30]*Gu1[13] + Gx1[31]*Gu1[15];
Gu2[8] = + Gx1[32]*Gu1[0] + Gx1[33]*Gu1[2] + Gx1[34]*Gu1[4] + Gx1[35]*Gu1[6] + Gx1[36]*Gu1[8] + Gx1[37]*Gu1[10] + Gx1[38]*Gu1[12] + Gx1[39]*Gu1[14];
Gu2[9] = + Gx1[32]*Gu1[1] + Gx1[33]*Gu1[3] + Gx1[34]*Gu1[5] + Gx1[35]*Gu1[7] + Gx1[36]*Gu1[9] + Gx1[37]*Gu1[11] + Gx1[38]*Gu1[13] + Gx1[39]*Gu1[15];
Gu2[10] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[2] + Gx1[42]*Gu1[4] + Gx1[43]*Gu1[6] + Gx1[44]*Gu1[8] + Gx1[45]*Gu1[10] + Gx1[46]*Gu1[12] + Gx1[47]*Gu1[14];
Gu2[11] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[3] + Gx1[42]*Gu1[5] + Gx1[43]*Gu1[7] + Gx1[44]*Gu1[9] + Gx1[45]*Gu1[11] + Gx1[46]*Gu1[13] + Gx1[47]*Gu1[15];
Gu2[12] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[2] + Gx1[50]*Gu1[4] + Gx1[51]*Gu1[6] + Gx1[52]*Gu1[8] + Gx1[53]*Gu1[10] + Gx1[54]*Gu1[12] + Gx1[55]*Gu1[14];
Gu2[13] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[3] + Gx1[50]*Gu1[5] + Gx1[51]*Gu1[7] + Gx1[52]*Gu1[9] + Gx1[53]*Gu1[11] + Gx1[54]*Gu1[13] + Gx1[55]*Gu1[15];
Gu2[14] = + Gx1[56]*Gu1[0] + Gx1[57]*Gu1[2] + Gx1[58]*Gu1[4] + Gx1[59]*Gu1[6] + Gx1[60]*Gu1[8] + Gx1[61]*Gu1[10] + Gx1[62]*Gu1[12] + Gx1[63]*Gu1[14];
Gu2[15] = + Gx1[56]*Gu1[1] + Gx1[57]*Gu1[3] + Gx1[58]*Gu1[5] + Gx1[59]*Gu1[7] + Gx1[60]*Gu1[9] + Gx1[61]*Gu1[11] + Gx1[62]*Gu1[13] + Gx1[63]*Gu1[15];
}

void nmhe_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
}

void nmhe_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmheWorkspace.H[(iRow * 416 + 1664) + (iCol * 2 + 8)] += + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + Gu1[10]*Gu2[10] + Gu1[12]*Gu2[12] + Gu1[14]*Gu2[14];
nmheWorkspace.H[(iRow * 416 + 1664) + (iCol * 2 + 9)] += + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + Gu1[10]*Gu2[11] + Gu1[12]*Gu2[13] + Gu1[14]*Gu2[15];
nmheWorkspace.H[(iRow * 416 + 1872) + (iCol * 2 + 8)] += + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + Gu1[11]*Gu2[10] + Gu1[13]*Gu2[12] + Gu1[15]*Gu2[14];
nmheWorkspace.H[(iRow * 416 + 1872) + (iCol * 2 + 9)] += + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + Gu1[11]*Gu2[11] + Gu1[13]*Gu2[13] + Gu1[15]*Gu2[15];
}

void nmhe_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmheWorkspace.H[(iRow * 416 + 1664) + (iCol * 2 + 8)] = R11[0];
nmheWorkspace.H[(iRow * 416 + 1664) + (iCol * 2 + 9)] = R11[1];
nmheWorkspace.H[(iRow * 416 + 1872) + (iCol * 2 + 8)] = R11[2];
nmheWorkspace.H[(iRow * 416 + 1872) + (iCol * 2 + 9)] = R11[3];
}

void nmhe_zeroBlockH11( int iRow, int iCol )
{
nmheWorkspace.H[(iRow * 416 + 1664) + (iCol * 2 + 8)] = 0.0000000000000000e+00;
nmheWorkspace.H[(iRow * 416 + 1664) + (iCol * 2 + 9)] = 0.0000000000000000e+00;
nmheWorkspace.H[(iRow * 416 + 1872) + (iCol * 2 + 8)] = 0.0000000000000000e+00;
nmheWorkspace.H[(iRow * 416 + 1872) + (iCol * 2 + 9)] = 0.0000000000000000e+00;
}

void nmhe_copyHTH( int iRow, int iCol )
{
nmheWorkspace.H[(iRow * 416 + 1664) + (iCol * 2 + 8)] = nmheWorkspace.H[(iCol * 416 + 1664) + (iRow * 2 + 8)];
nmheWorkspace.H[(iRow * 416 + 1664) + (iCol * 2 + 9)] = nmheWorkspace.H[(iCol * 416 + 1872) + (iRow * 2 + 8)];
nmheWorkspace.H[(iRow * 416 + 1872) + (iCol * 2 + 8)] = nmheWorkspace.H[(iCol * 416 + 1664) + (iRow * 2 + 9)];
nmheWorkspace.H[(iRow * 416 + 1872) + (iCol * 2 + 9)] = nmheWorkspace.H[(iCol * 416 + 1872) + (iRow * 2 + 9)];
}

void nmhe_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7];
dNew[1] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3] + Gx1[12]*dOld[4] + Gx1[13]*dOld[5] + Gx1[14]*dOld[6] + Gx1[15]*dOld[7];
dNew[2] = + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7];
dNew[3] = + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7];
dNew[4] = + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7];
dNew[5] = + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7];
dNew[6] = + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7];
dNew[7] = + Gx1[56]*dOld[0] + Gx1[57]*dOld[1] + Gx1[58]*dOld[2] + Gx1[59]*dOld[3] + Gx1[60]*dOld[4] + Gx1[61]*dOld[5] + Gx1[62]*dOld[6] + Gx1[63]*dOld[7];
}

void nmhe_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmheWorkspace.QN1[0]*dOld[0] + nmheWorkspace.QN1[1]*dOld[1] + nmheWorkspace.QN1[2]*dOld[2] + nmheWorkspace.QN1[3]*dOld[3] + nmheWorkspace.QN1[4]*dOld[4] + nmheWorkspace.QN1[5]*dOld[5] + nmheWorkspace.QN1[6]*dOld[6] + nmheWorkspace.QN1[7]*dOld[7];
dNew[1] = + nmheWorkspace.QN1[8]*dOld[0] + nmheWorkspace.QN1[9]*dOld[1] + nmheWorkspace.QN1[10]*dOld[2] + nmheWorkspace.QN1[11]*dOld[3] + nmheWorkspace.QN1[12]*dOld[4] + nmheWorkspace.QN1[13]*dOld[5] + nmheWorkspace.QN1[14]*dOld[6] + nmheWorkspace.QN1[15]*dOld[7];
dNew[2] = + nmheWorkspace.QN1[16]*dOld[0] + nmheWorkspace.QN1[17]*dOld[1] + nmheWorkspace.QN1[18]*dOld[2] + nmheWorkspace.QN1[19]*dOld[3] + nmheWorkspace.QN1[20]*dOld[4] + nmheWorkspace.QN1[21]*dOld[5] + nmheWorkspace.QN1[22]*dOld[6] + nmheWorkspace.QN1[23]*dOld[7];
dNew[3] = + nmheWorkspace.QN1[24]*dOld[0] + nmheWorkspace.QN1[25]*dOld[1] + nmheWorkspace.QN1[26]*dOld[2] + nmheWorkspace.QN1[27]*dOld[3] + nmheWorkspace.QN1[28]*dOld[4] + nmheWorkspace.QN1[29]*dOld[5] + nmheWorkspace.QN1[30]*dOld[6] + nmheWorkspace.QN1[31]*dOld[7];
dNew[4] = + nmheWorkspace.QN1[32]*dOld[0] + nmheWorkspace.QN1[33]*dOld[1] + nmheWorkspace.QN1[34]*dOld[2] + nmheWorkspace.QN1[35]*dOld[3] + nmheWorkspace.QN1[36]*dOld[4] + nmheWorkspace.QN1[37]*dOld[5] + nmheWorkspace.QN1[38]*dOld[6] + nmheWorkspace.QN1[39]*dOld[7];
dNew[5] = + nmheWorkspace.QN1[40]*dOld[0] + nmheWorkspace.QN1[41]*dOld[1] + nmheWorkspace.QN1[42]*dOld[2] + nmheWorkspace.QN1[43]*dOld[3] + nmheWorkspace.QN1[44]*dOld[4] + nmheWorkspace.QN1[45]*dOld[5] + nmheWorkspace.QN1[46]*dOld[6] + nmheWorkspace.QN1[47]*dOld[7];
dNew[6] = + nmheWorkspace.QN1[48]*dOld[0] + nmheWorkspace.QN1[49]*dOld[1] + nmheWorkspace.QN1[50]*dOld[2] + nmheWorkspace.QN1[51]*dOld[3] + nmheWorkspace.QN1[52]*dOld[4] + nmheWorkspace.QN1[53]*dOld[5] + nmheWorkspace.QN1[54]*dOld[6] + nmheWorkspace.QN1[55]*dOld[7];
dNew[7] = + nmheWorkspace.QN1[56]*dOld[0] + nmheWorkspace.QN1[57]*dOld[1] + nmheWorkspace.QN1[58]*dOld[2] + nmheWorkspace.QN1[59]*dOld[3] + nmheWorkspace.QN1[60]*dOld[4] + nmheWorkspace.QN1[61]*dOld[5] + nmheWorkspace.QN1[62]*dOld[6] + nmheWorkspace.QN1[63]*dOld[7];
}

void nmhe_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7];
RDy1[1] = + R2[8]*Dy1[0] + R2[9]*Dy1[1] + R2[10]*Dy1[2] + R2[11]*Dy1[3] + R2[12]*Dy1[4] + R2[13]*Dy1[5] + R2[14]*Dy1[6] + R2[15]*Dy1[7];
}

void nmhe_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7];
QDy1[1] = + Q2[8]*Dy1[0] + Q2[9]*Dy1[1] + Q2[10]*Dy1[2] + Q2[11]*Dy1[3] + Q2[12]*Dy1[4] + Q2[13]*Dy1[5] + Q2[14]*Dy1[6] + Q2[15]*Dy1[7];
QDy1[2] = + Q2[16]*Dy1[0] + Q2[17]*Dy1[1] + Q2[18]*Dy1[2] + Q2[19]*Dy1[3] + Q2[20]*Dy1[4] + Q2[21]*Dy1[5] + Q2[22]*Dy1[6] + Q2[23]*Dy1[7];
QDy1[3] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7];
QDy1[4] = + Q2[32]*Dy1[0] + Q2[33]*Dy1[1] + Q2[34]*Dy1[2] + Q2[35]*Dy1[3] + Q2[36]*Dy1[4] + Q2[37]*Dy1[5] + Q2[38]*Dy1[6] + Q2[39]*Dy1[7];
QDy1[5] = + Q2[40]*Dy1[0] + Q2[41]*Dy1[1] + Q2[42]*Dy1[2] + Q2[43]*Dy1[3] + Q2[44]*Dy1[4] + Q2[45]*Dy1[5] + Q2[46]*Dy1[6] + Q2[47]*Dy1[7];
QDy1[6] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5] + Q2[54]*Dy1[6] + Q2[55]*Dy1[7];
QDy1[7] = + Q2[56]*Dy1[0] + Q2[57]*Dy1[1] + Q2[58]*Dy1[2] + Q2[59]*Dy1[3] + Q2[60]*Dy1[4] + Q2[61]*Dy1[5] + Q2[62]*Dy1[6] + Q2[63]*Dy1[7];
}

void nmhe_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[2]*QDy1[1] + E1[4]*QDy1[2] + E1[6]*QDy1[3] + E1[8]*QDy1[4] + E1[10]*QDy1[5] + E1[12]*QDy1[6] + E1[14]*QDy1[7];
U1[1] += + E1[1]*QDy1[0] + E1[3]*QDy1[1] + E1[5]*QDy1[2] + E1[7]*QDy1[3] + E1[9]*QDy1[4] + E1[11]*QDy1[5] + E1[13]*QDy1[6] + E1[15]*QDy1[7];
}

void nmhe_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[2]*Gx1[8] + E1[4]*Gx1[16] + E1[6]*Gx1[24] + E1[8]*Gx1[32] + E1[10]*Gx1[40] + E1[12]*Gx1[48] + E1[14]*Gx1[56];
H101[1] += + E1[0]*Gx1[1] + E1[2]*Gx1[9] + E1[4]*Gx1[17] + E1[6]*Gx1[25] + E1[8]*Gx1[33] + E1[10]*Gx1[41] + E1[12]*Gx1[49] + E1[14]*Gx1[57];
H101[2] += + E1[0]*Gx1[2] + E1[2]*Gx1[10] + E1[4]*Gx1[18] + E1[6]*Gx1[26] + E1[8]*Gx1[34] + E1[10]*Gx1[42] + E1[12]*Gx1[50] + E1[14]*Gx1[58];
H101[3] += + E1[0]*Gx1[3] + E1[2]*Gx1[11] + E1[4]*Gx1[19] + E1[6]*Gx1[27] + E1[8]*Gx1[35] + E1[10]*Gx1[43] + E1[12]*Gx1[51] + E1[14]*Gx1[59];
H101[4] += + E1[0]*Gx1[4] + E1[2]*Gx1[12] + E1[4]*Gx1[20] + E1[6]*Gx1[28] + E1[8]*Gx1[36] + E1[10]*Gx1[44] + E1[12]*Gx1[52] + E1[14]*Gx1[60];
H101[5] += + E1[0]*Gx1[5] + E1[2]*Gx1[13] + E1[4]*Gx1[21] + E1[6]*Gx1[29] + E1[8]*Gx1[37] + E1[10]*Gx1[45] + E1[12]*Gx1[53] + E1[14]*Gx1[61];
H101[6] += + E1[0]*Gx1[6] + E1[2]*Gx1[14] + E1[4]*Gx1[22] + E1[6]*Gx1[30] + E1[8]*Gx1[38] + E1[10]*Gx1[46] + E1[12]*Gx1[54] + E1[14]*Gx1[62];
H101[7] += + E1[0]*Gx1[7] + E1[2]*Gx1[15] + E1[4]*Gx1[23] + E1[6]*Gx1[31] + E1[8]*Gx1[39] + E1[10]*Gx1[47] + E1[12]*Gx1[55] + E1[14]*Gx1[63];
H101[8] += + E1[1]*Gx1[0] + E1[3]*Gx1[8] + E1[5]*Gx1[16] + E1[7]*Gx1[24] + E1[9]*Gx1[32] + E1[11]*Gx1[40] + E1[13]*Gx1[48] + E1[15]*Gx1[56];
H101[9] += + E1[1]*Gx1[1] + E1[3]*Gx1[9] + E1[5]*Gx1[17] + E1[7]*Gx1[25] + E1[9]*Gx1[33] + E1[11]*Gx1[41] + E1[13]*Gx1[49] + E1[15]*Gx1[57];
H101[10] += + E1[1]*Gx1[2] + E1[3]*Gx1[10] + E1[5]*Gx1[18] + E1[7]*Gx1[26] + E1[9]*Gx1[34] + E1[11]*Gx1[42] + E1[13]*Gx1[50] + E1[15]*Gx1[58];
H101[11] += + E1[1]*Gx1[3] + E1[3]*Gx1[11] + E1[5]*Gx1[19] + E1[7]*Gx1[27] + E1[9]*Gx1[35] + E1[11]*Gx1[43] + E1[13]*Gx1[51] + E1[15]*Gx1[59];
H101[12] += + E1[1]*Gx1[4] + E1[3]*Gx1[12] + E1[5]*Gx1[20] + E1[7]*Gx1[28] + E1[9]*Gx1[36] + E1[11]*Gx1[44] + E1[13]*Gx1[52] + E1[15]*Gx1[60];
H101[13] += + E1[1]*Gx1[5] + E1[3]*Gx1[13] + E1[5]*Gx1[21] + E1[7]*Gx1[29] + E1[9]*Gx1[37] + E1[11]*Gx1[45] + E1[13]*Gx1[53] + E1[15]*Gx1[61];
H101[14] += + E1[1]*Gx1[6] + E1[3]*Gx1[14] + E1[5]*Gx1[22] + E1[7]*Gx1[30] + E1[9]*Gx1[38] + E1[11]*Gx1[46] + E1[13]*Gx1[54] + E1[15]*Gx1[62];
H101[15] += + E1[1]*Gx1[7] + E1[3]*Gx1[15] + E1[5]*Gx1[23] + E1[7]*Gx1[31] + E1[9]*Gx1[39] + E1[11]*Gx1[47] + E1[13]*Gx1[55] + E1[15]*Gx1[63];
}

void nmhe_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 16; lCopy++) H101[ lCopy ] = 0; }
}

void nmhe_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1];
dNew[1] += + E1[2]*U1[0] + E1[3]*U1[1];
dNew[2] += + E1[4]*U1[0] + E1[5]*U1[1];
dNew[3] += + E1[6]*U1[0] + E1[7]*U1[1];
dNew[4] += + E1[8]*U1[0] + E1[9]*U1[1];
dNew[5] += + E1[10]*U1[0] + E1[11]*U1[1];
dNew[6] += + E1[12]*U1[0] + E1[13]*U1[1];
dNew[7] += + E1[14]*U1[0] + E1[15]*U1[1];
}

void nmhe_zeroBlockH00(  )
{
nmheWorkspace.H[0] = 0.0000000000000000e+00;
nmheWorkspace.H[1] = 0.0000000000000000e+00;
nmheWorkspace.H[2] = 0.0000000000000000e+00;
nmheWorkspace.H[3] = 0.0000000000000000e+00;
nmheWorkspace.H[4] = 0.0000000000000000e+00;
nmheWorkspace.H[5] = 0.0000000000000000e+00;
nmheWorkspace.H[6] = 0.0000000000000000e+00;
nmheWorkspace.H[7] = 0.0000000000000000e+00;
nmheWorkspace.H[208] = 0.0000000000000000e+00;
nmheWorkspace.H[209] = 0.0000000000000000e+00;
nmheWorkspace.H[210] = 0.0000000000000000e+00;
nmheWorkspace.H[211] = 0.0000000000000000e+00;
nmheWorkspace.H[212] = 0.0000000000000000e+00;
nmheWorkspace.H[213] = 0.0000000000000000e+00;
nmheWorkspace.H[214] = 0.0000000000000000e+00;
nmheWorkspace.H[215] = 0.0000000000000000e+00;
nmheWorkspace.H[416] = 0.0000000000000000e+00;
nmheWorkspace.H[417] = 0.0000000000000000e+00;
nmheWorkspace.H[418] = 0.0000000000000000e+00;
nmheWorkspace.H[419] = 0.0000000000000000e+00;
nmheWorkspace.H[420] = 0.0000000000000000e+00;
nmheWorkspace.H[421] = 0.0000000000000000e+00;
nmheWorkspace.H[422] = 0.0000000000000000e+00;
nmheWorkspace.H[423] = 0.0000000000000000e+00;
nmheWorkspace.H[624] = 0.0000000000000000e+00;
nmheWorkspace.H[625] = 0.0000000000000000e+00;
nmheWorkspace.H[626] = 0.0000000000000000e+00;
nmheWorkspace.H[627] = 0.0000000000000000e+00;
nmheWorkspace.H[628] = 0.0000000000000000e+00;
nmheWorkspace.H[629] = 0.0000000000000000e+00;
nmheWorkspace.H[630] = 0.0000000000000000e+00;
nmheWorkspace.H[631] = 0.0000000000000000e+00;
nmheWorkspace.H[832] = 0.0000000000000000e+00;
nmheWorkspace.H[833] = 0.0000000000000000e+00;
nmheWorkspace.H[834] = 0.0000000000000000e+00;
nmheWorkspace.H[835] = 0.0000000000000000e+00;
nmheWorkspace.H[836] = 0.0000000000000000e+00;
nmheWorkspace.H[837] = 0.0000000000000000e+00;
nmheWorkspace.H[838] = 0.0000000000000000e+00;
nmheWorkspace.H[839] = 0.0000000000000000e+00;
nmheWorkspace.H[1040] = 0.0000000000000000e+00;
nmheWorkspace.H[1041] = 0.0000000000000000e+00;
nmheWorkspace.H[1042] = 0.0000000000000000e+00;
nmheWorkspace.H[1043] = 0.0000000000000000e+00;
nmheWorkspace.H[1044] = 0.0000000000000000e+00;
nmheWorkspace.H[1045] = 0.0000000000000000e+00;
nmheWorkspace.H[1046] = 0.0000000000000000e+00;
nmheWorkspace.H[1047] = 0.0000000000000000e+00;
nmheWorkspace.H[1248] = 0.0000000000000000e+00;
nmheWorkspace.H[1249] = 0.0000000000000000e+00;
nmheWorkspace.H[1250] = 0.0000000000000000e+00;
nmheWorkspace.H[1251] = 0.0000000000000000e+00;
nmheWorkspace.H[1252] = 0.0000000000000000e+00;
nmheWorkspace.H[1253] = 0.0000000000000000e+00;
nmheWorkspace.H[1254] = 0.0000000000000000e+00;
nmheWorkspace.H[1255] = 0.0000000000000000e+00;
nmheWorkspace.H[1456] = 0.0000000000000000e+00;
nmheWorkspace.H[1457] = 0.0000000000000000e+00;
nmheWorkspace.H[1458] = 0.0000000000000000e+00;
nmheWorkspace.H[1459] = 0.0000000000000000e+00;
nmheWorkspace.H[1460] = 0.0000000000000000e+00;
nmheWorkspace.H[1461] = 0.0000000000000000e+00;
nmheWorkspace.H[1462] = 0.0000000000000000e+00;
nmheWorkspace.H[1463] = 0.0000000000000000e+00;
}

void nmhe_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
nmheWorkspace.H[0] += + Gx1[0]*Gx2[0] + Gx1[8]*Gx2[8] + Gx1[16]*Gx2[16] + Gx1[24]*Gx2[24] + Gx1[32]*Gx2[32] + Gx1[40]*Gx2[40] + Gx1[48]*Gx2[48] + Gx1[56]*Gx2[56];
nmheWorkspace.H[1] += + Gx1[0]*Gx2[1] + Gx1[8]*Gx2[9] + Gx1[16]*Gx2[17] + Gx1[24]*Gx2[25] + Gx1[32]*Gx2[33] + Gx1[40]*Gx2[41] + Gx1[48]*Gx2[49] + Gx1[56]*Gx2[57];
nmheWorkspace.H[2] += + Gx1[0]*Gx2[2] + Gx1[8]*Gx2[10] + Gx1[16]*Gx2[18] + Gx1[24]*Gx2[26] + Gx1[32]*Gx2[34] + Gx1[40]*Gx2[42] + Gx1[48]*Gx2[50] + Gx1[56]*Gx2[58];
nmheWorkspace.H[3] += + Gx1[0]*Gx2[3] + Gx1[8]*Gx2[11] + Gx1[16]*Gx2[19] + Gx1[24]*Gx2[27] + Gx1[32]*Gx2[35] + Gx1[40]*Gx2[43] + Gx1[48]*Gx2[51] + Gx1[56]*Gx2[59];
nmheWorkspace.H[4] += + Gx1[0]*Gx2[4] + Gx1[8]*Gx2[12] + Gx1[16]*Gx2[20] + Gx1[24]*Gx2[28] + Gx1[32]*Gx2[36] + Gx1[40]*Gx2[44] + Gx1[48]*Gx2[52] + Gx1[56]*Gx2[60];
nmheWorkspace.H[5] += + Gx1[0]*Gx2[5] + Gx1[8]*Gx2[13] + Gx1[16]*Gx2[21] + Gx1[24]*Gx2[29] + Gx1[32]*Gx2[37] + Gx1[40]*Gx2[45] + Gx1[48]*Gx2[53] + Gx1[56]*Gx2[61];
nmheWorkspace.H[6] += + Gx1[0]*Gx2[6] + Gx1[8]*Gx2[14] + Gx1[16]*Gx2[22] + Gx1[24]*Gx2[30] + Gx1[32]*Gx2[38] + Gx1[40]*Gx2[46] + Gx1[48]*Gx2[54] + Gx1[56]*Gx2[62];
nmheWorkspace.H[7] += + Gx1[0]*Gx2[7] + Gx1[8]*Gx2[15] + Gx1[16]*Gx2[23] + Gx1[24]*Gx2[31] + Gx1[32]*Gx2[39] + Gx1[40]*Gx2[47] + Gx1[48]*Gx2[55] + Gx1[56]*Gx2[63];
nmheWorkspace.H[208] += + Gx1[1]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[17]*Gx2[16] + Gx1[25]*Gx2[24] + Gx1[33]*Gx2[32] + Gx1[41]*Gx2[40] + Gx1[49]*Gx2[48] + Gx1[57]*Gx2[56];
nmheWorkspace.H[209] += + Gx1[1]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[17]*Gx2[17] + Gx1[25]*Gx2[25] + Gx1[33]*Gx2[33] + Gx1[41]*Gx2[41] + Gx1[49]*Gx2[49] + Gx1[57]*Gx2[57];
nmheWorkspace.H[210] += + Gx1[1]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[17]*Gx2[18] + Gx1[25]*Gx2[26] + Gx1[33]*Gx2[34] + Gx1[41]*Gx2[42] + Gx1[49]*Gx2[50] + Gx1[57]*Gx2[58];
nmheWorkspace.H[211] += + Gx1[1]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[17]*Gx2[19] + Gx1[25]*Gx2[27] + Gx1[33]*Gx2[35] + Gx1[41]*Gx2[43] + Gx1[49]*Gx2[51] + Gx1[57]*Gx2[59];
nmheWorkspace.H[212] += + Gx1[1]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[17]*Gx2[20] + Gx1[25]*Gx2[28] + Gx1[33]*Gx2[36] + Gx1[41]*Gx2[44] + Gx1[49]*Gx2[52] + Gx1[57]*Gx2[60];
nmheWorkspace.H[213] += + Gx1[1]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[17]*Gx2[21] + Gx1[25]*Gx2[29] + Gx1[33]*Gx2[37] + Gx1[41]*Gx2[45] + Gx1[49]*Gx2[53] + Gx1[57]*Gx2[61];
nmheWorkspace.H[214] += + Gx1[1]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[17]*Gx2[22] + Gx1[25]*Gx2[30] + Gx1[33]*Gx2[38] + Gx1[41]*Gx2[46] + Gx1[49]*Gx2[54] + Gx1[57]*Gx2[62];
nmheWorkspace.H[215] += + Gx1[1]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[17]*Gx2[23] + Gx1[25]*Gx2[31] + Gx1[33]*Gx2[39] + Gx1[41]*Gx2[47] + Gx1[49]*Gx2[55] + Gx1[57]*Gx2[63];
nmheWorkspace.H[416] += + Gx1[2]*Gx2[0] + Gx1[10]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[26]*Gx2[24] + Gx1[34]*Gx2[32] + Gx1[42]*Gx2[40] + Gx1[50]*Gx2[48] + Gx1[58]*Gx2[56];
nmheWorkspace.H[417] += + Gx1[2]*Gx2[1] + Gx1[10]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[26]*Gx2[25] + Gx1[34]*Gx2[33] + Gx1[42]*Gx2[41] + Gx1[50]*Gx2[49] + Gx1[58]*Gx2[57];
nmheWorkspace.H[418] += + Gx1[2]*Gx2[2] + Gx1[10]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[26]*Gx2[26] + Gx1[34]*Gx2[34] + Gx1[42]*Gx2[42] + Gx1[50]*Gx2[50] + Gx1[58]*Gx2[58];
nmheWorkspace.H[419] += + Gx1[2]*Gx2[3] + Gx1[10]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[26]*Gx2[27] + Gx1[34]*Gx2[35] + Gx1[42]*Gx2[43] + Gx1[50]*Gx2[51] + Gx1[58]*Gx2[59];
nmheWorkspace.H[420] += + Gx1[2]*Gx2[4] + Gx1[10]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[26]*Gx2[28] + Gx1[34]*Gx2[36] + Gx1[42]*Gx2[44] + Gx1[50]*Gx2[52] + Gx1[58]*Gx2[60];
nmheWorkspace.H[421] += + Gx1[2]*Gx2[5] + Gx1[10]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[26]*Gx2[29] + Gx1[34]*Gx2[37] + Gx1[42]*Gx2[45] + Gx1[50]*Gx2[53] + Gx1[58]*Gx2[61];
nmheWorkspace.H[422] += + Gx1[2]*Gx2[6] + Gx1[10]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[26]*Gx2[30] + Gx1[34]*Gx2[38] + Gx1[42]*Gx2[46] + Gx1[50]*Gx2[54] + Gx1[58]*Gx2[62];
nmheWorkspace.H[423] += + Gx1[2]*Gx2[7] + Gx1[10]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[26]*Gx2[31] + Gx1[34]*Gx2[39] + Gx1[42]*Gx2[47] + Gx1[50]*Gx2[55] + Gx1[58]*Gx2[63];
nmheWorkspace.H[624] += + Gx1[3]*Gx2[0] + Gx1[11]*Gx2[8] + Gx1[19]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[35]*Gx2[32] + Gx1[43]*Gx2[40] + Gx1[51]*Gx2[48] + Gx1[59]*Gx2[56];
nmheWorkspace.H[625] += + Gx1[3]*Gx2[1] + Gx1[11]*Gx2[9] + Gx1[19]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[35]*Gx2[33] + Gx1[43]*Gx2[41] + Gx1[51]*Gx2[49] + Gx1[59]*Gx2[57];
nmheWorkspace.H[626] += + Gx1[3]*Gx2[2] + Gx1[11]*Gx2[10] + Gx1[19]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[35]*Gx2[34] + Gx1[43]*Gx2[42] + Gx1[51]*Gx2[50] + Gx1[59]*Gx2[58];
nmheWorkspace.H[627] += + Gx1[3]*Gx2[3] + Gx1[11]*Gx2[11] + Gx1[19]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[35]*Gx2[35] + Gx1[43]*Gx2[43] + Gx1[51]*Gx2[51] + Gx1[59]*Gx2[59];
nmheWorkspace.H[628] += + Gx1[3]*Gx2[4] + Gx1[11]*Gx2[12] + Gx1[19]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[35]*Gx2[36] + Gx1[43]*Gx2[44] + Gx1[51]*Gx2[52] + Gx1[59]*Gx2[60];
nmheWorkspace.H[629] += + Gx1[3]*Gx2[5] + Gx1[11]*Gx2[13] + Gx1[19]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[35]*Gx2[37] + Gx1[43]*Gx2[45] + Gx1[51]*Gx2[53] + Gx1[59]*Gx2[61];
nmheWorkspace.H[630] += + Gx1[3]*Gx2[6] + Gx1[11]*Gx2[14] + Gx1[19]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[35]*Gx2[38] + Gx1[43]*Gx2[46] + Gx1[51]*Gx2[54] + Gx1[59]*Gx2[62];
nmheWorkspace.H[631] += + Gx1[3]*Gx2[7] + Gx1[11]*Gx2[15] + Gx1[19]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[35]*Gx2[39] + Gx1[43]*Gx2[47] + Gx1[51]*Gx2[55] + Gx1[59]*Gx2[63];
nmheWorkspace.H[832] += + Gx1[4]*Gx2[0] + Gx1[12]*Gx2[8] + Gx1[20]*Gx2[16] + Gx1[28]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[44]*Gx2[40] + Gx1[52]*Gx2[48] + Gx1[60]*Gx2[56];
nmheWorkspace.H[833] += + Gx1[4]*Gx2[1] + Gx1[12]*Gx2[9] + Gx1[20]*Gx2[17] + Gx1[28]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[44]*Gx2[41] + Gx1[52]*Gx2[49] + Gx1[60]*Gx2[57];
nmheWorkspace.H[834] += + Gx1[4]*Gx2[2] + Gx1[12]*Gx2[10] + Gx1[20]*Gx2[18] + Gx1[28]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[44]*Gx2[42] + Gx1[52]*Gx2[50] + Gx1[60]*Gx2[58];
nmheWorkspace.H[835] += + Gx1[4]*Gx2[3] + Gx1[12]*Gx2[11] + Gx1[20]*Gx2[19] + Gx1[28]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[44]*Gx2[43] + Gx1[52]*Gx2[51] + Gx1[60]*Gx2[59];
nmheWorkspace.H[836] += + Gx1[4]*Gx2[4] + Gx1[12]*Gx2[12] + Gx1[20]*Gx2[20] + Gx1[28]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[44]*Gx2[44] + Gx1[52]*Gx2[52] + Gx1[60]*Gx2[60];
nmheWorkspace.H[837] += + Gx1[4]*Gx2[5] + Gx1[12]*Gx2[13] + Gx1[20]*Gx2[21] + Gx1[28]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[44]*Gx2[45] + Gx1[52]*Gx2[53] + Gx1[60]*Gx2[61];
nmheWorkspace.H[838] += + Gx1[4]*Gx2[6] + Gx1[12]*Gx2[14] + Gx1[20]*Gx2[22] + Gx1[28]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[44]*Gx2[46] + Gx1[52]*Gx2[54] + Gx1[60]*Gx2[62];
nmheWorkspace.H[839] += + Gx1[4]*Gx2[7] + Gx1[12]*Gx2[15] + Gx1[20]*Gx2[23] + Gx1[28]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[44]*Gx2[47] + Gx1[52]*Gx2[55] + Gx1[60]*Gx2[63];
nmheWorkspace.H[1040] += + Gx1[5]*Gx2[0] + Gx1[13]*Gx2[8] + Gx1[21]*Gx2[16] + Gx1[29]*Gx2[24] + Gx1[37]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[53]*Gx2[48] + Gx1[61]*Gx2[56];
nmheWorkspace.H[1041] += + Gx1[5]*Gx2[1] + Gx1[13]*Gx2[9] + Gx1[21]*Gx2[17] + Gx1[29]*Gx2[25] + Gx1[37]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[53]*Gx2[49] + Gx1[61]*Gx2[57];
nmheWorkspace.H[1042] += + Gx1[5]*Gx2[2] + Gx1[13]*Gx2[10] + Gx1[21]*Gx2[18] + Gx1[29]*Gx2[26] + Gx1[37]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[53]*Gx2[50] + Gx1[61]*Gx2[58];
nmheWorkspace.H[1043] += + Gx1[5]*Gx2[3] + Gx1[13]*Gx2[11] + Gx1[21]*Gx2[19] + Gx1[29]*Gx2[27] + Gx1[37]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[53]*Gx2[51] + Gx1[61]*Gx2[59];
nmheWorkspace.H[1044] += + Gx1[5]*Gx2[4] + Gx1[13]*Gx2[12] + Gx1[21]*Gx2[20] + Gx1[29]*Gx2[28] + Gx1[37]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[53]*Gx2[52] + Gx1[61]*Gx2[60];
nmheWorkspace.H[1045] += + Gx1[5]*Gx2[5] + Gx1[13]*Gx2[13] + Gx1[21]*Gx2[21] + Gx1[29]*Gx2[29] + Gx1[37]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[53]*Gx2[53] + Gx1[61]*Gx2[61];
nmheWorkspace.H[1046] += + Gx1[5]*Gx2[6] + Gx1[13]*Gx2[14] + Gx1[21]*Gx2[22] + Gx1[29]*Gx2[30] + Gx1[37]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[53]*Gx2[54] + Gx1[61]*Gx2[62];
nmheWorkspace.H[1047] += + Gx1[5]*Gx2[7] + Gx1[13]*Gx2[15] + Gx1[21]*Gx2[23] + Gx1[29]*Gx2[31] + Gx1[37]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[53]*Gx2[55] + Gx1[61]*Gx2[63];
nmheWorkspace.H[1248] += + Gx1[6]*Gx2[0] + Gx1[14]*Gx2[8] + Gx1[22]*Gx2[16] + Gx1[30]*Gx2[24] + Gx1[38]*Gx2[32] + Gx1[46]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[62]*Gx2[56];
nmheWorkspace.H[1249] += + Gx1[6]*Gx2[1] + Gx1[14]*Gx2[9] + Gx1[22]*Gx2[17] + Gx1[30]*Gx2[25] + Gx1[38]*Gx2[33] + Gx1[46]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[62]*Gx2[57];
nmheWorkspace.H[1250] += + Gx1[6]*Gx2[2] + Gx1[14]*Gx2[10] + Gx1[22]*Gx2[18] + Gx1[30]*Gx2[26] + Gx1[38]*Gx2[34] + Gx1[46]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[62]*Gx2[58];
nmheWorkspace.H[1251] += + Gx1[6]*Gx2[3] + Gx1[14]*Gx2[11] + Gx1[22]*Gx2[19] + Gx1[30]*Gx2[27] + Gx1[38]*Gx2[35] + Gx1[46]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[62]*Gx2[59];
nmheWorkspace.H[1252] += + Gx1[6]*Gx2[4] + Gx1[14]*Gx2[12] + Gx1[22]*Gx2[20] + Gx1[30]*Gx2[28] + Gx1[38]*Gx2[36] + Gx1[46]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[62]*Gx2[60];
nmheWorkspace.H[1253] += + Gx1[6]*Gx2[5] + Gx1[14]*Gx2[13] + Gx1[22]*Gx2[21] + Gx1[30]*Gx2[29] + Gx1[38]*Gx2[37] + Gx1[46]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[62]*Gx2[61];
nmheWorkspace.H[1254] += + Gx1[6]*Gx2[6] + Gx1[14]*Gx2[14] + Gx1[22]*Gx2[22] + Gx1[30]*Gx2[30] + Gx1[38]*Gx2[38] + Gx1[46]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[62]*Gx2[62];
nmheWorkspace.H[1255] += + Gx1[6]*Gx2[7] + Gx1[14]*Gx2[15] + Gx1[22]*Gx2[23] + Gx1[30]*Gx2[31] + Gx1[38]*Gx2[39] + Gx1[46]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[62]*Gx2[63];
nmheWorkspace.H[1456] += + Gx1[7]*Gx2[0] + Gx1[15]*Gx2[8] + Gx1[23]*Gx2[16] + Gx1[31]*Gx2[24] + Gx1[39]*Gx2[32] + Gx1[47]*Gx2[40] + Gx1[55]*Gx2[48] + Gx1[63]*Gx2[56];
nmheWorkspace.H[1457] += + Gx1[7]*Gx2[1] + Gx1[15]*Gx2[9] + Gx1[23]*Gx2[17] + Gx1[31]*Gx2[25] + Gx1[39]*Gx2[33] + Gx1[47]*Gx2[41] + Gx1[55]*Gx2[49] + Gx1[63]*Gx2[57];
nmheWorkspace.H[1458] += + Gx1[7]*Gx2[2] + Gx1[15]*Gx2[10] + Gx1[23]*Gx2[18] + Gx1[31]*Gx2[26] + Gx1[39]*Gx2[34] + Gx1[47]*Gx2[42] + Gx1[55]*Gx2[50] + Gx1[63]*Gx2[58];
nmheWorkspace.H[1459] += + Gx1[7]*Gx2[3] + Gx1[15]*Gx2[11] + Gx1[23]*Gx2[19] + Gx1[31]*Gx2[27] + Gx1[39]*Gx2[35] + Gx1[47]*Gx2[43] + Gx1[55]*Gx2[51] + Gx1[63]*Gx2[59];
nmheWorkspace.H[1460] += + Gx1[7]*Gx2[4] + Gx1[15]*Gx2[12] + Gx1[23]*Gx2[20] + Gx1[31]*Gx2[28] + Gx1[39]*Gx2[36] + Gx1[47]*Gx2[44] + Gx1[55]*Gx2[52] + Gx1[63]*Gx2[60];
nmheWorkspace.H[1461] += + Gx1[7]*Gx2[5] + Gx1[15]*Gx2[13] + Gx1[23]*Gx2[21] + Gx1[31]*Gx2[29] + Gx1[39]*Gx2[37] + Gx1[47]*Gx2[45] + Gx1[55]*Gx2[53] + Gx1[63]*Gx2[61];
nmheWorkspace.H[1462] += + Gx1[7]*Gx2[6] + Gx1[15]*Gx2[14] + Gx1[23]*Gx2[22] + Gx1[31]*Gx2[30] + Gx1[39]*Gx2[38] + Gx1[47]*Gx2[46] + Gx1[55]*Gx2[54] + Gx1[63]*Gx2[62];
nmheWorkspace.H[1463] += + Gx1[7]*Gx2[7] + Gx1[15]*Gx2[15] + Gx1[23]*Gx2[23] + Gx1[31]*Gx2[31] + Gx1[39]*Gx2[39] + Gx1[47]*Gx2[47] + Gx1[55]*Gx2[55] + Gx1[63]*Gx2[63];
}

void nmhe_macCTSlx( real_t* const C0, real_t* const g0 )
{
g0[0] += 0.0;
;
g0[1] += 0.0;
;
g0[2] += 0.0;
;
g0[3] += 0.0;
;
g0[4] += 0.0;
;
g0[5] += 0.0;
;
g0[6] += 0.0;
;
g0[7] += 0.0;
;
}

void nmhe_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
}

void nmhe_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 200 */
static const int xBoundIndices[ 200 ] = 
{ 14, 15, 22, 23, 30, 31, 38, 39, 46, 47, 54, 55, 62, 63, 70, 71, 78, 79, 86, 87, 94, 95, 102, 103, 110, 111, 118, 119, 126, 127, 134, 135, 142, 143, 150, 151, 158, 159, 166, 167, 174, 175, 182, 183, 190, 191, 198, 199, 206, 207, 214, 215, 222, 223, 230, 231, 238, 239, 246, 247, 254, 255, 262, 263, 270, 271, 278, 279, 286, 287, 294, 295, 302, 303, 310, 311, 318, 319, 326, 327, 334, 335, 342, 343, 350, 351, 358, 359, 366, 367, 374, 375, 382, 383, 390, 391, 398, 399, 406, 407, 414, 415, 422, 423, 430, 431, 438, 439, 446, 447, 454, 455, 462, 463, 470, 471, 478, 479, 486, 487, 494, 495, 502, 503, 510, 511, 518, 519, 526, 527, 534, 535, 542, 543, 550, 551, 558, 559, 566, 567, 574, 575, 582, 583, 590, 591, 598, 599, 606, 607, 614, 615, 622, 623, 630, 631, 638, 639, 646, 647, 654, 655, 662, 663, 670, 671, 678, 679, 686, 687, 694, 695, 702, 703, 710, 711, 718, 719, 726, 727, 734, 735, 742, 743, 750, 751, 758, 759, 766, 767, 774, 775, 782, 783, 790, 791, 798, 799, 806, 807 };
nmhe_moveGuE( nmheWorkspace.evGu, nmheWorkspace.E );
for (lRun1 = 1; lRun1 < 100; ++lRun1)
{
nmhe_moveGxT( &(nmheWorkspace.evGx[ lRun1 * 64 ]), nmheWorkspace.T );
nmhe_multGxd( &(nmheWorkspace.d[ lRun1 * 8-8 ]), &(nmheWorkspace.evGx[ lRun1 * 64 ]), &(nmheWorkspace.d[ lRun1 * 8 ]) );
nmhe_multGxGx( nmheWorkspace.T, &(nmheWorkspace.evGx[ lRun1 * 64-64 ]), &(nmheWorkspace.evGx[ lRun1 * 64 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmhe_multGxGu( nmheWorkspace.T, &(nmheWorkspace.E[ lRun4 * 16 ]), &(nmheWorkspace.E[ lRun3 * 16 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmhe_moveGuE( &(nmheWorkspace.evGu[ lRun1 * 16 ]), &(nmheWorkspace.E[ lRun3 * 16 ]) );
}

nmhe_multGxGx( &(nmheWorkspace.Q1[ 64 ]), nmheWorkspace.evGx, nmheWorkspace.QGx );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 128 ]), &(nmheWorkspace.evGx[ 64 ]), &(nmheWorkspace.QGx[ 64 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 192 ]), &(nmheWorkspace.evGx[ 128 ]), &(nmheWorkspace.QGx[ 128 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 256 ]), &(nmheWorkspace.evGx[ 192 ]), &(nmheWorkspace.QGx[ 192 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 320 ]), &(nmheWorkspace.evGx[ 256 ]), &(nmheWorkspace.QGx[ 256 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 384 ]), &(nmheWorkspace.evGx[ 320 ]), &(nmheWorkspace.QGx[ 320 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 448 ]), &(nmheWorkspace.evGx[ 384 ]), &(nmheWorkspace.QGx[ 384 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 512 ]), &(nmheWorkspace.evGx[ 448 ]), &(nmheWorkspace.QGx[ 448 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 576 ]), &(nmheWorkspace.evGx[ 512 ]), &(nmheWorkspace.QGx[ 512 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 640 ]), &(nmheWorkspace.evGx[ 576 ]), &(nmheWorkspace.QGx[ 576 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 704 ]), &(nmheWorkspace.evGx[ 640 ]), &(nmheWorkspace.QGx[ 640 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 768 ]), &(nmheWorkspace.evGx[ 704 ]), &(nmheWorkspace.QGx[ 704 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 832 ]), &(nmheWorkspace.evGx[ 768 ]), &(nmheWorkspace.QGx[ 768 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 896 ]), &(nmheWorkspace.evGx[ 832 ]), &(nmheWorkspace.QGx[ 832 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 960 ]), &(nmheWorkspace.evGx[ 896 ]), &(nmheWorkspace.QGx[ 896 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1024 ]), &(nmheWorkspace.evGx[ 960 ]), &(nmheWorkspace.QGx[ 960 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1088 ]), &(nmheWorkspace.evGx[ 1024 ]), &(nmheWorkspace.QGx[ 1024 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1152 ]), &(nmheWorkspace.evGx[ 1088 ]), &(nmheWorkspace.QGx[ 1088 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1216 ]), &(nmheWorkspace.evGx[ 1152 ]), &(nmheWorkspace.QGx[ 1152 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1280 ]), &(nmheWorkspace.evGx[ 1216 ]), &(nmheWorkspace.QGx[ 1216 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1344 ]), &(nmheWorkspace.evGx[ 1280 ]), &(nmheWorkspace.QGx[ 1280 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1408 ]), &(nmheWorkspace.evGx[ 1344 ]), &(nmheWorkspace.QGx[ 1344 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1472 ]), &(nmheWorkspace.evGx[ 1408 ]), &(nmheWorkspace.QGx[ 1408 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1536 ]), &(nmheWorkspace.evGx[ 1472 ]), &(nmheWorkspace.QGx[ 1472 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1600 ]), &(nmheWorkspace.evGx[ 1536 ]), &(nmheWorkspace.QGx[ 1536 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1664 ]), &(nmheWorkspace.evGx[ 1600 ]), &(nmheWorkspace.QGx[ 1600 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1728 ]), &(nmheWorkspace.evGx[ 1664 ]), &(nmheWorkspace.QGx[ 1664 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1792 ]), &(nmheWorkspace.evGx[ 1728 ]), &(nmheWorkspace.QGx[ 1728 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1856 ]), &(nmheWorkspace.evGx[ 1792 ]), &(nmheWorkspace.QGx[ 1792 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1920 ]), &(nmheWorkspace.evGx[ 1856 ]), &(nmheWorkspace.QGx[ 1856 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 1984 ]), &(nmheWorkspace.evGx[ 1920 ]), &(nmheWorkspace.QGx[ 1920 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2048 ]), &(nmheWorkspace.evGx[ 1984 ]), &(nmheWorkspace.QGx[ 1984 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2112 ]), &(nmheWorkspace.evGx[ 2048 ]), &(nmheWorkspace.QGx[ 2048 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2176 ]), &(nmheWorkspace.evGx[ 2112 ]), &(nmheWorkspace.QGx[ 2112 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2240 ]), &(nmheWorkspace.evGx[ 2176 ]), &(nmheWorkspace.QGx[ 2176 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2304 ]), &(nmheWorkspace.evGx[ 2240 ]), &(nmheWorkspace.QGx[ 2240 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2368 ]), &(nmheWorkspace.evGx[ 2304 ]), &(nmheWorkspace.QGx[ 2304 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2432 ]), &(nmheWorkspace.evGx[ 2368 ]), &(nmheWorkspace.QGx[ 2368 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2496 ]), &(nmheWorkspace.evGx[ 2432 ]), &(nmheWorkspace.QGx[ 2432 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2560 ]), &(nmheWorkspace.evGx[ 2496 ]), &(nmheWorkspace.QGx[ 2496 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2624 ]), &(nmheWorkspace.evGx[ 2560 ]), &(nmheWorkspace.QGx[ 2560 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2688 ]), &(nmheWorkspace.evGx[ 2624 ]), &(nmheWorkspace.QGx[ 2624 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2752 ]), &(nmheWorkspace.evGx[ 2688 ]), &(nmheWorkspace.QGx[ 2688 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2816 ]), &(nmheWorkspace.evGx[ 2752 ]), &(nmheWorkspace.QGx[ 2752 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2880 ]), &(nmheWorkspace.evGx[ 2816 ]), &(nmheWorkspace.QGx[ 2816 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 2944 ]), &(nmheWorkspace.evGx[ 2880 ]), &(nmheWorkspace.QGx[ 2880 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3008 ]), &(nmheWorkspace.evGx[ 2944 ]), &(nmheWorkspace.QGx[ 2944 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3072 ]), &(nmheWorkspace.evGx[ 3008 ]), &(nmheWorkspace.QGx[ 3008 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3136 ]), &(nmheWorkspace.evGx[ 3072 ]), &(nmheWorkspace.QGx[ 3072 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3200 ]), &(nmheWorkspace.evGx[ 3136 ]), &(nmheWorkspace.QGx[ 3136 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3264 ]), &(nmheWorkspace.evGx[ 3200 ]), &(nmheWorkspace.QGx[ 3200 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3328 ]), &(nmheWorkspace.evGx[ 3264 ]), &(nmheWorkspace.QGx[ 3264 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3392 ]), &(nmheWorkspace.evGx[ 3328 ]), &(nmheWorkspace.QGx[ 3328 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3456 ]), &(nmheWorkspace.evGx[ 3392 ]), &(nmheWorkspace.QGx[ 3392 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3520 ]), &(nmheWorkspace.evGx[ 3456 ]), &(nmheWorkspace.QGx[ 3456 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3584 ]), &(nmheWorkspace.evGx[ 3520 ]), &(nmheWorkspace.QGx[ 3520 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3648 ]), &(nmheWorkspace.evGx[ 3584 ]), &(nmheWorkspace.QGx[ 3584 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3712 ]), &(nmheWorkspace.evGx[ 3648 ]), &(nmheWorkspace.QGx[ 3648 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3776 ]), &(nmheWorkspace.evGx[ 3712 ]), &(nmheWorkspace.QGx[ 3712 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3840 ]), &(nmheWorkspace.evGx[ 3776 ]), &(nmheWorkspace.QGx[ 3776 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3904 ]), &(nmheWorkspace.evGx[ 3840 ]), &(nmheWorkspace.QGx[ 3840 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 3968 ]), &(nmheWorkspace.evGx[ 3904 ]), &(nmheWorkspace.QGx[ 3904 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4032 ]), &(nmheWorkspace.evGx[ 3968 ]), &(nmheWorkspace.QGx[ 3968 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4096 ]), &(nmheWorkspace.evGx[ 4032 ]), &(nmheWorkspace.QGx[ 4032 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4160 ]), &(nmheWorkspace.evGx[ 4096 ]), &(nmheWorkspace.QGx[ 4096 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4224 ]), &(nmheWorkspace.evGx[ 4160 ]), &(nmheWorkspace.QGx[ 4160 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4288 ]), &(nmheWorkspace.evGx[ 4224 ]), &(nmheWorkspace.QGx[ 4224 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4352 ]), &(nmheWorkspace.evGx[ 4288 ]), &(nmheWorkspace.QGx[ 4288 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4416 ]), &(nmheWorkspace.evGx[ 4352 ]), &(nmheWorkspace.QGx[ 4352 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4480 ]), &(nmheWorkspace.evGx[ 4416 ]), &(nmheWorkspace.QGx[ 4416 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4544 ]), &(nmheWorkspace.evGx[ 4480 ]), &(nmheWorkspace.QGx[ 4480 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4608 ]), &(nmheWorkspace.evGx[ 4544 ]), &(nmheWorkspace.QGx[ 4544 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4672 ]), &(nmheWorkspace.evGx[ 4608 ]), &(nmheWorkspace.QGx[ 4608 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4736 ]), &(nmheWorkspace.evGx[ 4672 ]), &(nmheWorkspace.QGx[ 4672 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4800 ]), &(nmheWorkspace.evGx[ 4736 ]), &(nmheWorkspace.QGx[ 4736 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4864 ]), &(nmheWorkspace.evGx[ 4800 ]), &(nmheWorkspace.QGx[ 4800 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4928 ]), &(nmheWorkspace.evGx[ 4864 ]), &(nmheWorkspace.QGx[ 4864 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 4992 ]), &(nmheWorkspace.evGx[ 4928 ]), &(nmheWorkspace.QGx[ 4928 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5056 ]), &(nmheWorkspace.evGx[ 4992 ]), &(nmheWorkspace.QGx[ 4992 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5120 ]), &(nmheWorkspace.evGx[ 5056 ]), &(nmheWorkspace.QGx[ 5056 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5184 ]), &(nmheWorkspace.evGx[ 5120 ]), &(nmheWorkspace.QGx[ 5120 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5248 ]), &(nmheWorkspace.evGx[ 5184 ]), &(nmheWorkspace.QGx[ 5184 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5312 ]), &(nmheWorkspace.evGx[ 5248 ]), &(nmheWorkspace.QGx[ 5248 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5376 ]), &(nmheWorkspace.evGx[ 5312 ]), &(nmheWorkspace.QGx[ 5312 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5440 ]), &(nmheWorkspace.evGx[ 5376 ]), &(nmheWorkspace.QGx[ 5376 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5504 ]), &(nmheWorkspace.evGx[ 5440 ]), &(nmheWorkspace.QGx[ 5440 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5568 ]), &(nmheWorkspace.evGx[ 5504 ]), &(nmheWorkspace.QGx[ 5504 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5632 ]), &(nmheWorkspace.evGx[ 5568 ]), &(nmheWorkspace.QGx[ 5568 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5696 ]), &(nmheWorkspace.evGx[ 5632 ]), &(nmheWorkspace.QGx[ 5632 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5760 ]), &(nmheWorkspace.evGx[ 5696 ]), &(nmheWorkspace.QGx[ 5696 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5824 ]), &(nmheWorkspace.evGx[ 5760 ]), &(nmheWorkspace.QGx[ 5760 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5888 ]), &(nmheWorkspace.evGx[ 5824 ]), &(nmheWorkspace.QGx[ 5824 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 5952 ]), &(nmheWorkspace.evGx[ 5888 ]), &(nmheWorkspace.QGx[ 5888 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 6016 ]), &(nmheWorkspace.evGx[ 5952 ]), &(nmheWorkspace.QGx[ 5952 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 6080 ]), &(nmheWorkspace.evGx[ 6016 ]), &(nmheWorkspace.QGx[ 6016 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 6144 ]), &(nmheWorkspace.evGx[ 6080 ]), &(nmheWorkspace.QGx[ 6080 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 6208 ]), &(nmheWorkspace.evGx[ 6144 ]), &(nmheWorkspace.QGx[ 6144 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 6272 ]), &(nmheWorkspace.evGx[ 6208 ]), &(nmheWorkspace.QGx[ 6208 ]) );
nmhe_multGxGx( &(nmheWorkspace.Q1[ 6336 ]), &(nmheWorkspace.evGx[ 6272 ]), &(nmheWorkspace.QGx[ 6272 ]) );
nmhe_multGxGx( nmheWorkspace.QN1, &(nmheWorkspace.evGx[ 6336 ]), &(nmheWorkspace.QGx[ 6336 ]) );

for (lRun1 = 0; lRun1 < 99; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmhe_multGxGu( &(nmheWorkspace.Q1[ lRun1 * 64 + 64 ]), &(nmheWorkspace.E[ lRun3 * 16 ]), &(nmheWorkspace.QE[ lRun3 * 16 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmhe_multGxGu( nmheWorkspace.QN1, &(nmheWorkspace.E[ lRun3 * 16 ]), &(nmheWorkspace.QE[ lRun3 * 16 ]) );
}

nmhe_zeroBlockH00(  );
nmhe_multCTQC( nmheWorkspace.evGx, nmheWorkspace.QGx );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 64 ]), &(nmheWorkspace.QGx[ 64 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 128 ]), &(nmheWorkspace.QGx[ 128 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 192 ]), &(nmheWorkspace.QGx[ 192 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 256 ]), &(nmheWorkspace.QGx[ 256 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 320 ]), &(nmheWorkspace.QGx[ 320 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 384 ]), &(nmheWorkspace.QGx[ 384 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 448 ]), &(nmheWorkspace.QGx[ 448 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 512 ]), &(nmheWorkspace.QGx[ 512 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 576 ]), &(nmheWorkspace.QGx[ 576 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 640 ]), &(nmheWorkspace.QGx[ 640 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 704 ]), &(nmheWorkspace.QGx[ 704 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 768 ]), &(nmheWorkspace.QGx[ 768 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 832 ]), &(nmheWorkspace.QGx[ 832 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 896 ]), &(nmheWorkspace.QGx[ 896 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 960 ]), &(nmheWorkspace.QGx[ 960 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1024 ]), &(nmheWorkspace.QGx[ 1024 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1088 ]), &(nmheWorkspace.QGx[ 1088 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1152 ]), &(nmheWorkspace.QGx[ 1152 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1216 ]), &(nmheWorkspace.QGx[ 1216 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1280 ]), &(nmheWorkspace.QGx[ 1280 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1344 ]), &(nmheWorkspace.QGx[ 1344 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1408 ]), &(nmheWorkspace.QGx[ 1408 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1472 ]), &(nmheWorkspace.QGx[ 1472 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1536 ]), &(nmheWorkspace.QGx[ 1536 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1600 ]), &(nmheWorkspace.QGx[ 1600 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1664 ]), &(nmheWorkspace.QGx[ 1664 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1728 ]), &(nmheWorkspace.QGx[ 1728 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1792 ]), &(nmheWorkspace.QGx[ 1792 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1856 ]), &(nmheWorkspace.QGx[ 1856 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1920 ]), &(nmheWorkspace.QGx[ 1920 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 1984 ]), &(nmheWorkspace.QGx[ 1984 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2048 ]), &(nmheWorkspace.QGx[ 2048 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2112 ]), &(nmheWorkspace.QGx[ 2112 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2176 ]), &(nmheWorkspace.QGx[ 2176 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2240 ]), &(nmheWorkspace.QGx[ 2240 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2304 ]), &(nmheWorkspace.QGx[ 2304 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2368 ]), &(nmheWorkspace.QGx[ 2368 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2432 ]), &(nmheWorkspace.QGx[ 2432 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2496 ]), &(nmheWorkspace.QGx[ 2496 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2560 ]), &(nmheWorkspace.QGx[ 2560 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2624 ]), &(nmheWorkspace.QGx[ 2624 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2688 ]), &(nmheWorkspace.QGx[ 2688 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2752 ]), &(nmheWorkspace.QGx[ 2752 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2816 ]), &(nmheWorkspace.QGx[ 2816 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2880 ]), &(nmheWorkspace.QGx[ 2880 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 2944 ]), &(nmheWorkspace.QGx[ 2944 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3008 ]), &(nmheWorkspace.QGx[ 3008 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3072 ]), &(nmheWorkspace.QGx[ 3072 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3136 ]), &(nmheWorkspace.QGx[ 3136 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3200 ]), &(nmheWorkspace.QGx[ 3200 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3264 ]), &(nmheWorkspace.QGx[ 3264 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3328 ]), &(nmheWorkspace.QGx[ 3328 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3392 ]), &(nmheWorkspace.QGx[ 3392 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3456 ]), &(nmheWorkspace.QGx[ 3456 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3520 ]), &(nmheWorkspace.QGx[ 3520 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3584 ]), &(nmheWorkspace.QGx[ 3584 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3648 ]), &(nmheWorkspace.QGx[ 3648 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3712 ]), &(nmheWorkspace.QGx[ 3712 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3776 ]), &(nmheWorkspace.QGx[ 3776 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3840 ]), &(nmheWorkspace.QGx[ 3840 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3904 ]), &(nmheWorkspace.QGx[ 3904 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 3968 ]), &(nmheWorkspace.QGx[ 3968 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4032 ]), &(nmheWorkspace.QGx[ 4032 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4096 ]), &(nmheWorkspace.QGx[ 4096 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4160 ]), &(nmheWorkspace.QGx[ 4160 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4224 ]), &(nmheWorkspace.QGx[ 4224 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4288 ]), &(nmheWorkspace.QGx[ 4288 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4352 ]), &(nmheWorkspace.QGx[ 4352 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4416 ]), &(nmheWorkspace.QGx[ 4416 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4480 ]), &(nmheWorkspace.QGx[ 4480 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4544 ]), &(nmheWorkspace.QGx[ 4544 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4608 ]), &(nmheWorkspace.QGx[ 4608 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4672 ]), &(nmheWorkspace.QGx[ 4672 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4736 ]), &(nmheWorkspace.QGx[ 4736 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4800 ]), &(nmheWorkspace.QGx[ 4800 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4864 ]), &(nmheWorkspace.QGx[ 4864 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4928 ]), &(nmheWorkspace.QGx[ 4928 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 4992 ]), &(nmheWorkspace.QGx[ 4992 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5056 ]), &(nmheWorkspace.QGx[ 5056 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5120 ]), &(nmheWorkspace.QGx[ 5120 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5184 ]), &(nmheWorkspace.QGx[ 5184 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5248 ]), &(nmheWorkspace.QGx[ 5248 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5312 ]), &(nmheWorkspace.QGx[ 5312 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5376 ]), &(nmheWorkspace.QGx[ 5376 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5440 ]), &(nmheWorkspace.QGx[ 5440 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5504 ]), &(nmheWorkspace.QGx[ 5504 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5568 ]), &(nmheWorkspace.QGx[ 5568 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5632 ]), &(nmheWorkspace.QGx[ 5632 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5696 ]), &(nmheWorkspace.QGx[ 5696 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5760 ]), &(nmheWorkspace.QGx[ 5760 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5824 ]), &(nmheWorkspace.QGx[ 5824 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5888 ]), &(nmheWorkspace.QGx[ 5888 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 5952 ]), &(nmheWorkspace.QGx[ 5952 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 6016 ]), &(nmheWorkspace.QGx[ 6016 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 6080 ]), &(nmheWorkspace.QGx[ 6080 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 6144 ]), &(nmheWorkspace.QGx[ 6144 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 6208 ]), &(nmheWorkspace.QGx[ 6208 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 6272 ]), &(nmheWorkspace.QGx[ 6272 ]) );
nmhe_multCTQC( &(nmheWorkspace.evGx[ 6336 ]), &(nmheWorkspace.QGx[ 6336 ]) );

nmheWorkspace.H[0] += nmheWorkspace.Q1[0];
nmheWorkspace.H[1] += nmheWorkspace.Q1[1];
nmheWorkspace.H[2] += nmheWorkspace.Q1[2];
nmheWorkspace.H[3] += nmheWorkspace.Q1[3];
nmheWorkspace.H[4] += nmheWorkspace.Q1[4];
nmheWorkspace.H[5] += nmheWorkspace.Q1[5];
nmheWorkspace.H[6] += nmheWorkspace.Q1[6];
nmheWorkspace.H[7] += nmheWorkspace.Q1[7];
nmheWorkspace.H[208] += nmheWorkspace.Q1[8];
nmheWorkspace.H[209] += nmheWorkspace.Q1[9];
nmheWorkspace.H[210] += nmheWorkspace.Q1[10];
nmheWorkspace.H[211] += nmheWorkspace.Q1[11];
nmheWorkspace.H[212] += nmheWorkspace.Q1[12];
nmheWorkspace.H[213] += nmheWorkspace.Q1[13];
nmheWorkspace.H[214] += nmheWorkspace.Q1[14];
nmheWorkspace.H[215] += nmheWorkspace.Q1[15];
nmheWorkspace.H[416] += nmheWorkspace.Q1[16];
nmheWorkspace.H[417] += nmheWorkspace.Q1[17];
nmheWorkspace.H[418] += nmheWorkspace.Q1[18];
nmheWorkspace.H[419] += nmheWorkspace.Q1[19];
nmheWorkspace.H[420] += nmheWorkspace.Q1[20];
nmheWorkspace.H[421] += nmheWorkspace.Q1[21];
nmheWorkspace.H[422] += nmheWorkspace.Q1[22];
nmheWorkspace.H[423] += nmheWorkspace.Q1[23];
nmheWorkspace.H[624] += nmheWorkspace.Q1[24];
nmheWorkspace.H[625] += nmheWorkspace.Q1[25];
nmheWorkspace.H[626] += nmheWorkspace.Q1[26];
nmheWorkspace.H[627] += nmheWorkspace.Q1[27];
nmheWorkspace.H[628] += nmheWorkspace.Q1[28];
nmheWorkspace.H[629] += nmheWorkspace.Q1[29];
nmheWorkspace.H[630] += nmheWorkspace.Q1[30];
nmheWorkspace.H[631] += nmheWorkspace.Q1[31];
nmheWorkspace.H[832] += nmheWorkspace.Q1[32];
nmheWorkspace.H[833] += nmheWorkspace.Q1[33];
nmheWorkspace.H[834] += nmheWorkspace.Q1[34];
nmheWorkspace.H[835] += nmheWorkspace.Q1[35];
nmheWorkspace.H[836] += nmheWorkspace.Q1[36];
nmheWorkspace.H[837] += nmheWorkspace.Q1[37];
nmheWorkspace.H[838] += nmheWorkspace.Q1[38];
nmheWorkspace.H[839] += nmheWorkspace.Q1[39];
nmheWorkspace.H[1040] += nmheWorkspace.Q1[40];
nmheWorkspace.H[1041] += nmheWorkspace.Q1[41];
nmheWorkspace.H[1042] += nmheWorkspace.Q1[42];
nmheWorkspace.H[1043] += nmheWorkspace.Q1[43];
nmheWorkspace.H[1044] += nmheWorkspace.Q1[44];
nmheWorkspace.H[1045] += nmheWorkspace.Q1[45];
nmheWorkspace.H[1046] += nmheWorkspace.Q1[46];
nmheWorkspace.H[1047] += nmheWorkspace.Q1[47];
nmheWorkspace.H[1248] += nmheWorkspace.Q1[48];
nmheWorkspace.H[1249] += nmheWorkspace.Q1[49];
nmheWorkspace.H[1250] += nmheWorkspace.Q1[50];
nmheWorkspace.H[1251] += nmheWorkspace.Q1[51];
nmheWorkspace.H[1252] += nmheWorkspace.Q1[52];
nmheWorkspace.H[1253] += nmheWorkspace.Q1[53];
nmheWorkspace.H[1254] += nmheWorkspace.Q1[54];
nmheWorkspace.H[1255] += nmheWorkspace.Q1[55];
nmheWorkspace.H[1456] += nmheWorkspace.Q1[56];
nmheWorkspace.H[1457] += nmheWorkspace.Q1[57];
nmheWorkspace.H[1458] += nmheWorkspace.Q1[58];
nmheWorkspace.H[1459] += nmheWorkspace.Q1[59];
nmheWorkspace.H[1460] += nmheWorkspace.Q1[60];
nmheWorkspace.H[1461] += nmheWorkspace.Q1[61];
nmheWorkspace.H[1462] += nmheWorkspace.Q1[62];
nmheWorkspace.H[1463] += nmheWorkspace.Q1[63];
nmheWorkspace.H[0] += nmheVariables.SAC[0];
nmheWorkspace.H[1] += nmheVariables.SAC[1];
nmheWorkspace.H[2] += nmheVariables.SAC[2];
nmheWorkspace.H[3] += nmheVariables.SAC[3];
nmheWorkspace.H[4] += nmheVariables.SAC[4];
nmheWorkspace.H[5] += nmheVariables.SAC[5];
nmheWorkspace.H[6] += nmheVariables.SAC[6];
nmheWorkspace.H[7] += nmheVariables.SAC[7];
nmheWorkspace.H[208] += nmheVariables.SAC[8];
nmheWorkspace.H[209] += nmheVariables.SAC[9];
nmheWorkspace.H[210] += nmheVariables.SAC[10];
nmheWorkspace.H[211] += nmheVariables.SAC[11];
nmheWorkspace.H[212] += nmheVariables.SAC[12];
nmheWorkspace.H[213] += nmheVariables.SAC[13];
nmheWorkspace.H[214] += nmheVariables.SAC[14];
nmheWorkspace.H[215] += nmheVariables.SAC[15];
nmheWorkspace.H[416] += nmheVariables.SAC[16];
nmheWorkspace.H[417] += nmheVariables.SAC[17];
nmheWorkspace.H[418] += nmheVariables.SAC[18];
nmheWorkspace.H[419] += nmheVariables.SAC[19];
nmheWorkspace.H[420] += nmheVariables.SAC[20];
nmheWorkspace.H[421] += nmheVariables.SAC[21];
nmheWorkspace.H[422] += nmheVariables.SAC[22];
nmheWorkspace.H[423] += nmheVariables.SAC[23];
nmheWorkspace.H[624] += nmheVariables.SAC[24];
nmheWorkspace.H[625] += nmheVariables.SAC[25];
nmheWorkspace.H[626] += nmheVariables.SAC[26];
nmheWorkspace.H[627] += nmheVariables.SAC[27];
nmheWorkspace.H[628] += nmheVariables.SAC[28];
nmheWorkspace.H[629] += nmheVariables.SAC[29];
nmheWorkspace.H[630] += nmheVariables.SAC[30];
nmheWorkspace.H[631] += nmheVariables.SAC[31];
nmheWorkspace.H[832] += nmheVariables.SAC[32];
nmheWorkspace.H[833] += nmheVariables.SAC[33];
nmheWorkspace.H[834] += nmheVariables.SAC[34];
nmheWorkspace.H[835] += nmheVariables.SAC[35];
nmheWorkspace.H[836] += nmheVariables.SAC[36];
nmheWorkspace.H[837] += nmheVariables.SAC[37];
nmheWorkspace.H[838] += nmheVariables.SAC[38];
nmheWorkspace.H[839] += nmheVariables.SAC[39];
nmheWorkspace.H[1040] += nmheVariables.SAC[40];
nmheWorkspace.H[1041] += nmheVariables.SAC[41];
nmheWorkspace.H[1042] += nmheVariables.SAC[42];
nmheWorkspace.H[1043] += nmheVariables.SAC[43];
nmheWorkspace.H[1044] += nmheVariables.SAC[44];
nmheWorkspace.H[1045] += nmheVariables.SAC[45];
nmheWorkspace.H[1046] += nmheVariables.SAC[46];
nmheWorkspace.H[1047] += nmheVariables.SAC[47];
nmheWorkspace.H[1248] += nmheVariables.SAC[48];
nmheWorkspace.H[1249] += nmheVariables.SAC[49];
nmheWorkspace.H[1250] += nmheVariables.SAC[50];
nmheWorkspace.H[1251] += nmheVariables.SAC[51];
nmheWorkspace.H[1252] += nmheVariables.SAC[52];
nmheWorkspace.H[1253] += nmheVariables.SAC[53];
nmheWorkspace.H[1254] += nmheVariables.SAC[54];
nmheWorkspace.H[1255] += nmheVariables.SAC[55];
nmheWorkspace.H[1456] += nmheVariables.SAC[56];
nmheWorkspace.H[1457] += nmheVariables.SAC[57];
nmheWorkspace.H[1458] += nmheVariables.SAC[58];
nmheWorkspace.H[1459] += nmheVariables.SAC[59];
nmheWorkspace.H[1460] += nmheVariables.SAC[60];
nmheWorkspace.H[1461] += nmheVariables.SAC[61];
nmheWorkspace.H[1462] += nmheVariables.SAC[62];
nmheWorkspace.H[1463] += nmheVariables.SAC[63];
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
nmhe_zeroBlockH10( &(nmheWorkspace.H10[ lRun1 * 16 ]) );
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmhe_multQETGx( &(nmheWorkspace.QE[ lRun3 * 16 ]), &(nmheWorkspace.evGx[ lRun2 * 64 ]), &(nmheWorkspace.H10[ lRun1 * 16 ]) );
}
}

for (lRun1 = 0;lRun1 < 8; ++lRun1)
for (lRun2 = 0;lRun2 < 200; ++lRun2)
nmheWorkspace.H[(lRun1 * 208) + (lRun2 + 8)] = nmheWorkspace.H10[(lRun2 * 8) + (lRun1)];

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
nmhe_setBlockH11_R1( lRun1, lRun1, &(nmheWorkspace.R1[ lRun1 * 4 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 100; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmhe_setBlockH11( lRun1, lRun2, &(nmheWorkspace.E[ lRun4 * 16 ]), &(nmheWorkspace.QE[ lRun5 * 16 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 100; ++lRun2)
{
nmhe_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 100; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmhe_setBlockH11( lRun1, lRun2, &(nmheWorkspace.E[ lRun4 * 16 ]), &(nmheWorkspace.QE[ lRun5 * 16 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
nmhe_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0;lRun1 < 200; ++lRun1)
for (lRun2 = 0;lRun2 < 8; ++lRun2)
nmheWorkspace.H[(lRun1 * 208 + 1664) + (lRun2)] = nmheWorkspace.H10[(lRun1 * 8) + (lRun2)];

nmhe_multQ1d( &(nmheWorkspace.Q1[ 64 ]), nmheWorkspace.d, nmheWorkspace.Qd );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 128 ]), &(nmheWorkspace.d[ 8 ]), &(nmheWorkspace.Qd[ 8 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 192 ]), &(nmheWorkspace.d[ 16 ]), &(nmheWorkspace.Qd[ 16 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 256 ]), &(nmheWorkspace.d[ 24 ]), &(nmheWorkspace.Qd[ 24 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 320 ]), &(nmheWorkspace.d[ 32 ]), &(nmheWorkspace.Qd[ 32 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 384 ]), &(nmheWorkspace.d[ 40 ]), &(nmheWorkspace.Qd[ 40 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 448 ]), &(nmheWorkspace.d[ 48 ]), &(nmheWorkspace.Qd[ 48 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 512 ]), &(nmheWorkspace.d[ 56 ]), &(nmheWorkspace.Qd[ 56 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 576 ]), &(nmheWorkspace.d[ 64 ]), &(nmheWorkspace.Qd[ 64 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 640 ]), &(nmheWorkspace.d[ 72 ]), &(nmheWorkspace.Qd[ 72 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 704 ]), &(nmheWorkspace.d[ 80 ]), &(nmheWorkspace.Qd[ 80 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 768 ]), &(nmheWorkspace.d[ 88 ]), &(nmheWorkspace.Qd[ 88 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 832 ]), &(nmheWorkspace.d[ 96 ]), &(nmheWorkspace.Qd[ 96 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 896 ]), &(nmheWorkspace.d[ 104 ]), &(nmheWorkspace.Qd[ 104 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 960 ]), &(nmheWorkspace.d[ 112 ]), &(nmheWorkspace.Qd[ 112 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1024 ]), &(nmheWorkspace.d[ 120 ]), &(nmheWorkspace.Qd[ 120 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1088 ]), &(nmheWorkspace.d[ 128 ]), &(nmheWorkspace.Qd[ 128 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1152 ]), &(nmheWorkspace.d[ 136 ]), &(nmheWorkspace.Qd[ 136 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1216 ]), &(nmheWorkspace.d[ 144 ]), &(nmheWorkspace.Qd[ 144 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1280 ]), &(nmheWorkspace.d[ 152 ]), &(nmheWorkspace.Qd[ 152 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1344 ]), &(nmheWorkspace.d[ 160 ]), &(nmheWorkspace.Qd[ 160 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1408 ]), &(nmheWorkspace.d[ 168 ]), &(nmheWorkspace.Qd[ 168 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1472 ]), &(nmheWorkspace.d[ 176 ]), &(nmheWorkspace.Qd[ 176 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1536 ]), &(nmheWorkspace.d[ 184 ]), &(nmheWorkspace.Qd[ 184 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1600 ]), &(nmheWorkspace.d[ 192 ]), &(nmheWorkspace.Qd[ 192 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1664 ]), &(nmheWorkspace.d[ 200 ]), &(nmheWorkspace.Qd[ 200 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1728 ]), &(nmheWorkspace.d[ 208 ]), &(nmheWorkspace.Qd[ 208 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1792 ]), &(nmheWorkspace.d[ 216 ]), &(nmheWorkspace.Qd[ 216 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1856 ]), &(nmheWorkspace.d[ 224 ]), &(nmheWorkspace.Qd[ 224 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1920 ]), &(nmheWorkspace.d[ 232 ]), &(nmheWorkspace.Qd[ 232 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 1984 ]), &(nmheWorkspace.d[ 240 ]), &(nmheWorkspace.Qd[ 240 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2048 ]), &(nmheWorkspace.d[ 248 ]), &(nmheWorkspace.Qd[ 248 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2112 ]), &(nmheWorkspace.d[ 256 ]), &(nmheWorkspace.Qd[ 256 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2176 ]), &(nmheWorkspace.d[ 264 ]), &(nmheWorkspace.Qd[ 264 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2240 ]), &(nmheWorkspace.d[ 272 ]), &(nmheWorkspace.Qd[ 272 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2304 ]), &(nmheWorkspace.d[ 280 ]), &(nmheWorkspace.Qd[ 280 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2368 ]), &(nmheWorkspace.d[ 288 ]), &(nmheWorkspace.Qd[ 288 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2432 ]), &(nmheWorkspace.d[ 296 ]), &(nmheWorkspace.Qd[ 296 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2496 ]), &(nmheWorkspace.d[ 304 ]), &(nmheWorkspace.Qd[ 304 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2560 ]), &(nmheWorkspace.d[ 312 ]), &(nmheWorkspace.Qd[ 312 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2624 ]), &(nmheWorkspace.d[ 320 ]), &(nmheWorkspace.Qd[ 320 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2688 ]), &(nmheWorkspace.d[ 328 ]), &(nmheWorkspace.Qd[ 328 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2752 ]), &(nmheWorkspace.d[ 336 ]), &(nmheWorkspace.Qd[ 336 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2816 ]), &(nmheWorkspace.d[ 344 ]), &(nmheWorkspace.Qd[ 344 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2880 ]), &(nmheWorkspace.d[ 352 ]), &(nmheWorkspace.Qd[ 352 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 2944 ]), &(nmheWorkspace.d[ 360 ]), &(nmheWorkspace.Qd[ 360 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3008 ]), &(nmheWorkspace.d[ 368 ]), &(nmheWorkspace.Qd[ 368 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3072 ]), &(nmheWorkspace.d[ 376 ]), &(nmheWorkspace.Qd[ 376 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3136 ]), &(nmheWorkspace.d[ 384 ]), &(nmheWorkspace.Qd[ 384 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3200 ]), &(nmheWorkspace.d[ 392 ]), &(nmheWorkspace.Qd[ 392 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3264 ]), &(nmheWorkspace.d[ 400 ]), &(nmheWorkspace.Qd[ 400 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3328 ]), &(nmheWorkspace.d[ 408 ]), &(nmheWorkspace.Qd[ 408 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3392 ]), &(nmheWorkspace.d[ 416 ]), &(nmheWorkspace.Qd[ 416 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3456 ]), &(nmheWorkspace.d[ 424 ]), &(nmheWorkspace.Qd[ 424 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3520 ]), &(nmheWorkspace.d[ 432 ]), &(nmheWorkspace.Qd[ 432 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3584 ]), &(nmheWorkspace.d[ 440 ]), &(nmheWorkspace.Qd[ 440 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3648 ]), &(nmheWorkspace.d[ 448 ]), &(nmheWorkspace.Qd[ 448 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3712 ]), &(nmheWorkspace.d[ 456 ]), &(nmheWorkspace.Qd[ 456 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3776 ]), &(nmheWorkspace.d[ 464 ]), &(nmheWorkspace.Qd[ 464 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3840 ]), &(nmheWorkspace.d[ 472 ]), &(nmheWorkspace.Qd[ 472 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3904 ]), &(nmheWorkspace.d[ 480 ]), &(nmheWorkspace.Qd[ 480 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 3968 ]), &(nmheWorkspace.d[ 488 ]), &(nmheWorkspace.Qd[ 488 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4032 ]), &(nmheWorkspace.d[ 496 ]), &(nmheWorkspace.Qd[ 496 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4096 ]), &(nmheWorkspace.d[ 504 ]), &(nmheWorkspace.Qd[ 504 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4160 ]), &(nmheWorkspace.d[ 512 ]), &(nmheWorkspace.Qd[ 512 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4224 ]), &(nmheWorkspace.d[ 520 ]), &(nmheWorkspace.Qd[ 520 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4288 ]), &(nmheWorkspace.d[ 528 ]), &(nmheWorkspace.Qd[ 528 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4352 ]), &(nmheWorkspace.d[ 536 ]), &(nmheWorkspace.Qd[ 536 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4416 ]), &(nmheWorkspace.d[ 544 ]), &(nmheWorkspace.Qd[ 544 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4480 ]), &(nmheWorkspace.d[ 552 ]), &(nmheWorkspace.Qd[ 552 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4544 ]), &(nmheWorkspace.d[ 560 ]), &(nmheWorkspace.Qd[ 560 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4608 ]), &(nmheWorkspace.d[ 568 ]), &(nmheWorkspace.Qd[ 568 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4672 ]), &(nmheWorkspace.d[ 576 ]), &(nmheWorkspace.Qd[ 576 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4736 ]), &(nmheWorkspace.d[ 584 ]), &(nmheWorkspace.Qd[ 584 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4800 ]), &(nmheWorkspace.d[ 592 ]), &(nmheWorkspace.Qd[ 592 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4864 ]), &(nmheWorkspace.d[ 600 ]), &(nmheWorkspace.Qd[ 600 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4928 ]), &(nmheWorkspace.d[ 608 ]), &(nmheWorkspace.Qd[ 608 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 4992 ]), &(nmheWorkspace.d[ 616 ]), &(nmheWorkspace.Qd[ 616 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5056 ]), &(nmheWorkspace.d[ 624 ]), &(nmheWorkspace.Qd[ 624 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5120 ]), &(nmheWorkspace.d[ 632 ]), &(nmheWorkspace.Qd[ 632 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5184 ]), &(nmheWorkspace.d[ 640 ]), &(nmheWorkspace.Qd[ 640 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5248 ]), &(nmheWorkspace.d[ 648 ]), &(nmheWorkspace.Qd[ 648 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5312 ]), &(nmheWorkspace.d[ 656 ]), &(nmheWorkspace.Qd[ 656 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5376 ]), &(nmheWorkspace.d[ 664 ]), &(nmheWorkspace.Qd[ 664 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5440 ]), &(nmheWorkspace.d[ 672 ]), &(nmheWorkspace.Qd[ 672 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5504 ]), &(nmheWorkspace.d[ 680 ]), &(nmheWorkspace.Qd[ 680 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5568 ]), &(nmheWorkspace.d[ 688 ]), &(nmheWorkspace.Qd[ 688 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5632 ]), &(nmheWorkspace.d[ 696 ]), &(nmheWorkspace.Qd[ 696 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5696 ]), &(nmheWorkspace.d[ 704 ]), &(nmheWorkspace.Qd[ 704 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5760 ]), &(nmheWorkspace.d[ 712 ]), &(nmheWorkspace.Qd[ 712 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5824 ]), &(nmheWorkspace.d[ 720 ]), &(nmheWorkspace.Qd[ 720 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5888 ]), &(nmheWorkspace.d[ 728 ]), &(nmheWorkspace.Qd[ 728 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 5952 ]), &(nmheWorkspace.d[ 736 ]), &(nmheWorkspace.Qd[ 736 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 6016 ]), &(nmheWorkspace.d[ 744 ]), &(nmheWorkspace.Qd[ 744 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 6080 ]), &(nmheWorkspace.d[ 752 ]), &(nmheWorkspace.Qd[ 752 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 6144 ]), &(nmheWorkspace.d[ 760 ]), &(nmheWorkspace.Qd[ 760 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 6208 ]), &(nmheWorkspace.d[ 768 ]), &(nmheWorkspace.Qd[ 768 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 6272 ]), &(nmheWorkspace.d[ 776 ]), &(nmheWorkspace.Qd[ 776 ]) );
nmhe_multQ1d( &(nmheWorkspace.Q1[ 6336 ]), &(nmheWorkspace.d[ 784 ]), &(nmheWorkspace.Qd[ 784 ]) );
nmhe_multQN1d( nmheWorkspace.QN1, &(nmheWorkspace.d[ 792 ]), &(nmheWorkspace.Qd[ 792 ]) );

nmhe_macCTSlx( nmheWorkspace.evGx, nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 64 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 128 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 192 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 256 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 320 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 384 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 448 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 512 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 576 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 640 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 704 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 768 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 832 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 896 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 960 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1024 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1088 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1152 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1216 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1280 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1344 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1408 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1472 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1536 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1600 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1664 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1728 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1792 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1856 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1920 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 1984 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2048 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2112 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2176 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2240 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2304 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2368 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2432 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2496 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2560 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2624 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2688 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2752 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2816 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2880 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 2944 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3008 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3072 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3136 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3200 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3264 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3328 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3392 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3456 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3520 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3584 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3648 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3712 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3776 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3840 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3904 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 3968 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4032 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4096 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4160 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4224 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4288 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4352 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4416 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4480 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4544 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4608 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4672 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4736 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4800 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4864 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4928 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 4992 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5056 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5120 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5184 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5248 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5312 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5376 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5440 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5504 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5568 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5632 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5696 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5760 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5824 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5888 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 5952 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 6016 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 6080 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 6144 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 6208 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 6272 ]), nmheWorkspace.g );
nmhe_macCTSlx( &(nmheWorkspace.evGx[ 6336 ]), nmheWorkspace.g );
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmhe_macETSlu( &(nmheWorkspace.QE[ lRun3 * 16 ]), &(nmheWorkspace.g[ lRun1 * 2 + 8 ]) );
}
}
nmheWorkspace.lb[0] = (real_t)-1.0000000000000000e+12 - nmheVariables.x[0];
nmheWorkspace.lb[1] = (real_t)-1.0000000000000000e+12 - nmheVariables.x[1];
nmheWorkspace.lb[2] = (real_t)-1.0000000000000000e+12 - nmheVariables.x[2];
nmheWorkspace.lb[3] = (real_t)-1.0000000000000000e+12 - nmheVariables.x[3];
nmheWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - nmheVariables.x[4];
nmheWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - nmheVariables.x[5];
nmheWorkspace.lb[6] = (real_t)1.0000000000000001e-01 - nmheVariables.x[6];
nmheWorkspace.lb[7] = (real_t)1.0000000000000001e-01 - nmheVariables.x[7];
nmheWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[0];
nmheWorkspace.lb[9] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[1];
nmheWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[2];
nmheWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[3];
nmheWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[4];
nmheWorkspace.lb[13] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[5];
nmheWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[6];
nmheWorkspace.lb[15] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[7];
nmheWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[8];
nmheWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[9];
nmheWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[10];
nmheWorkspace.lb[19] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[11];
nmheWorkspace.lb[20] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[12];
nmheWorkspace.lb[21] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[13];
nmheWorkspace.lb[22] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[14];
nmheWorkspace.lb[23] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[15];
nmheWorkspace.lb[24] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[16];
nmheWorkspace.lb[25] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[17];
nmheWorkspace.lb[26] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[18];
nmheWorkspace.lb[27] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[19];
nmheWorkspace.lb[28] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[20];
nmheWorkspace.lb[29] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[21];
nmheWorkspace.lb[30] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[22];
nmheWorkspace.lb[31] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[23];
nmheWorkspace.lb[32] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[24];
nmheWorkspace.lb[33] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[25];
nmheWorkspace.lb[34] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[26];
nmheWorkspace.lb[35] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[27];
nmheWorkspace.lb[36] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[28];
nmheWorkspace.lb[37] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[29];
nmheWorkspace.lb[38] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[30];
nmheWorkspace.lb[39] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[31];
nmheWorkspace.lb[40] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[32];
nmheWorkspace.lb[41] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[33];
nmheWorkspace.lb[42] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[34];
nmheWorkspace.lb[43] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[35];
nmheWorkspace.lb[44] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[36];
nmheWorkspace.lb[45] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[37];
nmheWorkspace.lb[46] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[38];
nmheWorkspace.lb[47] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[39];
nmheWorkspace.lb[48] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[40];
nmheWorkspace.lb[49] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[41];
nmheWorkspace.lb[50] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[42];
nmheWorkspace.lb[51] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[43];
nmheWorkspace.lb[52] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[44];
nmheWorkspace.lb[53] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[45];
nmheWorkspace.lb[54] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[46];
nmheWorkspace.lb[55] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[47];
nmheWorkspace.lb[56] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[48];
nmheWorkspace.lb[57] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[49];
nmheWorkspace.lb[58] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[50];
nmheWorkspace.lb[59] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[51];
nmheWorkspace.lb[60] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[52];
nmheWorkspace.lb[61] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[53];
nmheWorkspace.lb[62] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[54];
nmheWorkspace.lb[63] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[55];
nmheWorkspace.lb[64] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[56];
nmheWorkspace.lb[65] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[57];
nmheWorkspace.lb[66] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[58];
nmheWorkspace.lb[67] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[59];
nmheWorkspace.lb[68] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[60];
nmheWorkspace.lb[69] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[61];
nmheWorkspace.lb[70] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[62];
nmheWorkspace.lb[71] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[63];
nmheWorkspace.lb[72] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[64];
nmheWorkspace.lb[73] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[65];
nmheWorkspace.lb[74] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[66];
nmheWorkspace.lb[75] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[67];
nmheWorkspace.lb[76] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[68];
nmheWorkspace.lb[77] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[69];
nmheWorkspace.lb[78] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[70];
nmheWorkspace.lb[79] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[71];
nmheWorkspace.lb[80] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[72];
nmheWorkspace.lb[81] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[73];
nmheWorkspace.lb[82] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[74];
nmheWorkspace.lb[83] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[75];
nmheWorkspace.lb[84] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[76];
nmheWorkspace.lb[85] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[77];
nmheWorkspace.lb[86] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[78];
nmheWorkspace.lb[87] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[79];
nmheWorkspace.lb[88] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[80];
nmheWorkspace.lb[89] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[81];
nmheWorkspace.lb[90] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[82];
nmheWorkspace.lb[91] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[83];
nmheWorkspace.lb[92] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[84];
nmheWorkspace.lb[93] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[85];
nmheWorkspace.lb[94] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[86];
nmheWorkspace.lb[95] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[87];
nmheWorkspace.lb[96] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[88];
nmheWorkspace.lb[97] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[89];
nmheWorkspace.lb[98] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[90];
nmheWorkspace.lb[99] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[91];
nmheWorkspace.lb[100] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[92];
nmheWorkspace.lb[101] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[93];
nmheWorkspace.lb[102] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[94];
nmheWorkspace.lb[103] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[95];
nmheWorkspace.lb[104] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[96];
nmheWorkspace.lb[105] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[97];
nmheWorkspace.lb[106] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[98];
nmheWorkspace.lb[107] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[99];
nmheWorkspace.lb[108] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[100];
nmheWorkspace.lb[109] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[101];
nmheWorkspace.lb[110] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[102];
nmheWorkspace.lb[111] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[103];
nmheWorkspace.lb[112] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[104];
nmheWorkspace.lb[113] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[105];
nmheWorkspace.lb[114] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[106];
nmheWorkspace.lb[115] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[107];
nmheWorkspace.lb[116] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[108];
nmheWorkspace.lb[117] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[109];
nmheWorkspace.lb[118] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[110];
nmheWorkspace.lb[119] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[111];
nmheWorkspace.lb[120] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[112];
nmheWorkspace.lb[121] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[113];
nmheWorkspace.lb[122] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[114];
nmheWorkspace.lb[123] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[115];
nmheWorkspace.lb[124] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[116];
nmheWorkspace.lb[125] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[117];
nmheWorkspace.lb[126] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[118];
nmheWorkspace.lb[127] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[119];
nmheWorkspace.lb[128] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[120];
nmheWorkspace.lb[129] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[121];
nmheWorkspace.lb[130] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[122];
nmheWorkspace.lb[131] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[123];
nmheWorkspace.lb[132] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[124];
nmheWorkspace.lb[133] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[125];
nmheWorkspace.lb[134] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[126];
nmheWorkspace.lb[135] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[127];
nmheWorkspace.lb[136] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[128];
nmheWorkspace.lb[137] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[129];
nmheWorkspace.lb[138] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[130];
nmheWorkspace.lb[139] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[131];
nmheWorkspace.lb[140] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[132];
nmheWorkspace.lb[141] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[133];
nmheWorkspace.lb[142] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[134];
nmheWorkspace.lb[143] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[135];
nmheWorkspace.lb[144] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[136];
nmheWorkspace.lb[145] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[137];
nmheWorkspace.lb[146] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[138];
nmheWorkspace.lb[147] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[139];
nmheWorkspace.lb[148] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[140];
nmheWorkspace.lb[149] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[141];
nmheWorkspace.lb[150] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[142];
nmheWorkspace.lb[151] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[143];
nmheWorkspace.lb[152] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[144];
nmheWorkspace.lb[153] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[145];
nmheWorkspace.lb[154] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[146];
nmheWorkspace.lb[155] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[147];
nmheWorkspace.lb[156] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[148];
nmheWorkspace.lb[157] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[149];
nmheWorkspace.lb[158] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[150];
nmheWorkspace.lb[159] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[151];
nmheWorkspace.lb[160] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[152];
nmheWorkspace.lb[161] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[153];
nmheWorkspace.lb[162] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[154];
nmheWorkspace.lb[163] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[155];
nmheWorkspace.lb[164] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[156];
nmheWorkspace.lb[165] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[157];
nmheWorkspace.lb[166] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[158];
nmheWorkspace.lb[167] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[159];
nmheWorkspace.lb[168] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[160];
nmheWorkspace.lb[169] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[161];
nmheWorkspace.lb[170] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[162];
nmheWorkspace.lb[171] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[163];
nmheWorkspace.lb[172] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[164];
nmheWorkspace.lb[173] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[165];
nmheWorkspace.lb[174] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[166];
nmheWorkspace.lb[175] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[167];
nmheWorkspace.lb[176] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[168];
nmheWorkspace.lb[177] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[169];
nmheWorkspace.lb[178] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[170];
nmheWorkspace.lb[179] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[171];
nmheWorkspace.lb[180] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[172];
nmheWorkspace.lb[181] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[173];
nmheWorkspace.lb[182] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[174];
nmheWorkspace.lb[183] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[175];
nmheWorkspace.lb[184] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[176];
nmheWorkspace.lb[185] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[177];
nmheWorkspace.lb[186] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[178];
nmheWorkspace.lb[187] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[179];
nmheWorkspace.lb[188] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[180];
nmheWorkspace.lb[189] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[181];
nmheWorkspace.lb[190] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[182];
nmheWorkspace.lb[191] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[183];
nmheWorkspace.lb[192] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[184];
nmheWorkspace.lb[193] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[185];
nmheWorkspace.lb[194] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[186];
nmheWorkspace.lb[195] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[187];
nmheWorkspace.lb[196] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[188];
nmheWorkspace.lb[197] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[189];
nmheWorkspace.lb[198] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[190];
nmheWorkspace.lb[199] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[191];
nmheWorkspace.lb[200] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[192];
nmheWorkspace.lb[201] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[193];
nmheWorkspace.lb[202] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[194];
nmheWorkspace.lb[203] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[195];
nmheWorkspace.lb[204] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[196];
nmheWorkspace.lb[205] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[197];
nmheWorkspace.lb[206] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[198];
nmheWorkspace.lb[207] = (real_t)-1.0000000000000000e+12 - nmheVariables.u[199];
nmheWorkspace.ub[0] = (real_t)1.0000000000000000e+12 - nmheVariables.x[0];
nmheWorkspace.ub[1] = (real_t)1.0000000000000000e+12 - nmheVariables.x[1];
nmheWorkspace.ub[2] = (real_t)1.0000000000000000e+12 - nmheVariables.x[2];
nmheWorkspace.ub[3] = (real_t)1.0000000000000000e+12 - nmheVariables.x[3];
nmheWorkspace.ub[4] = (real_t)1.0000000000000000e+12 - nmheVariables.x[4];
nmheWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - nmheVariables.x[5];
nmheWorkspace.ub[6] = (real_t)1.0000000000000000e+00 - nmheVariables.x[6];
nmheWorkspace.ub[7] = (real_t)1.0000000000000000e+00 - nmheVariables.x[7];
nmheWorkspace.ub[8] = (real_t)1.0000000000000000e+12 - nmheVariables.u[0];
nmheWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - nmheVariables.u[1];
nmheWorkspace.ub[10] = (real_t)1.0000000000000000e+12 - nmheVariables.u[2];
nmheWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - nmheVariables.u[3];
nmheWorkspace.ub[12] = (real_t)1.0000000000000000e+12 - nmheVariables.u[4];
nmheWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - nmheVariables.u[5];
nmheWorkspace.ub[14] = (real_t)1.0000000000000000e+12 - nmheVariables.u[6];
nmheWorkspace.ub[15] = (real_t)1.0000000000000000e+12 - nmheVariables.u[7];
nmheWorkspace.ub[16] = (real_t)1.0000000000000000e+12 - nmheVariables.u[8];
nmheWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - nmheVariables.u[9];
nmheWorkspace.ub[18] = (real_t)1.0000000000000000e+12 - nmheVariables.u[10];
nmheWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - nmheVariables.u[11];
nmheWorkspace.ub[20] = (real_t)1.0000000000000000e+12 - nmheVariables.u[12];
nmheWorkspace.ub[21] = (real_t)1.0000000000000000e+12 - nmheVariables.u[13];
nmheWorkspace.ub[22] = (real_t)1.0000000000000000e+12 - nmheVariables.u[14];
nmheWorkspace.ub[23] = (real_t)1.0000000000000000e+12 - nmheVariables.u[15];
nmheWorkspace.ub[24] = (real_t)1.0000000000000000e+12 - nmheVariables.u[16];
nmheWorkspace.ub[25] = (real_t)1.0000000000000000e+12 - nmheVariables.u[17];
nmheWorkspace.ub[26] = (real_t)1.0000000000000000e+12 - nmheVariables.u[18];
nmheWorkspace.ub[27] = (real_t)1.0000000000000000e+12 - nmheVariables.u[19];
nmheWorkspace.ub[28] = (real_t)1.0000000000000000e+12 - nmheVariables.u[20];
nmheWorkspace.ub[29] = (real_t)1.0000000000000000e+12 - nmheVariables.u[21];
nmheWorkspace.ub[30] = (real_t)1.0000000000000000e+12 - nmheVariables.u[22];
nmheWorkspace.ub[31] = (real_t)1.0000000000000000e+12 - nmheVariables.u[23];
nmheWorkspace.ub[32] = (real_t)1.0000000000000000e+12 - nmheVariables.u[24];
nmheWorkspace.ub[33] = (real_t)1.0000000000000000e+12 - nmheVariables.u[25];
nmheWorkspace.ub[34] = (real_t)1.0000000000000000e+12 - nmheVariables.u[26];
nmheWorkspace.ub[35] = (real_t)1.0000000000000000e+12 - nmheVariables.u[27];
nmheWorkspace.ub[36] = (real_t)1.0000000000000000e+12 - nmheVariables.u[28];
nmheWorkspace.ub[37] = (real_t)1.0000000000000000e+12 - nmheVariables.u[29];
nmheWorkspace.ub[38] = (real_t)1.0000000000000000e+12 - nmheVariables.u[30];
nmheWorkspace.ub[39] = (real_t)1.0000000000000000e+12 - nmheVariables.u[31];
nmheWorkspace.ub[40] = (real_t)1.0000000000000000e+12 - nmheVariables.u[32];
nmheWorkspace.ub[41] = (real_t)1.0000000000000000e+12 - nmheVariables.u[33];
nmheWorkspace.ub[42] = (real_t)1.0000000000000000e+12 - nmheVariables.u[34];
nmheWorkspace.ub[43] = (real_t)1.0000000000000000e+12 - nmheVariables.u[35];
nmheWorkspace.ub[44] = (real_t)1.0000000000000000e+12 - nmheVariables.u[36];
nmheWorkspace.ub[45] = (real_t)1.0000000000000000e+12 - nmheVariables.u[37];
nmheWorkspace.ub[46] = (real_t)1.0000000000000000e+12 - nmheVariables.u[38];
nmheWorkspace.ub[47] = (real_t)1.0000000000000000e+12 - nmheVariables.u[39];
nmheWorkspace.ub[48] = (real_t)1.0000000000000000e+12 - nmheVariables.u[40];
nmheWorkspace.ub[49] = (real_t)1.0000000000000000e+12 - nmheVariables.u[41];
nmheWorkspace.ub[50] = (real_t)1.0000000000000000e+12 - nmheVariables.u[42];
nmheWorkspace.ub[51] = (real_t)1.0000000000000000e+12 - nmheVariables.u[43];
nmheWorkspace.ub[52] = (real_t)1.0000000000000000e+12 - nmheVariables.u[44];
nmheWorkspace.ub[53] = (real_t)1.0000000000000000e+12 - nmheVariables.u[45];
nmheWorkspace.ub[54] = (real_t)1.0000000000000000e+12 - nmheVariables.u[46];
nmheWorkspace.ub[55] = (real_t)1.0000000000000000e+12 - nmheVariables.u[47];
nmheWorkspace.ub[56] = (real_t)1.0000000000000000e+12 - nmheVariables.u[48];
nmheWorkspace.ub[57] = (real_t)1.0000000000000000e+12 - nmheVariables.u[49];
nmheWorkspace.ub[58] = (real_t)1.0000000000000000e+12 - nmheVariables.u[50];
nmheWorkspace.ub[59] = (real_t)1.0000000000000000e+12 - nmheVariables.u[51];
nmheWorkspace.ub[60] = (real_t)1.0000000000000000e+12 - nmheVariables.u[52];
nmheWorkspace.ub[61] = (real_t)1.0000000000000000e+12 - nmheVariables.u[53];
nmheWorkspace.ub[62] = (real_t)1.0000000000000000e+12 - nmheVariables.u[54];
nmheWorkspace.ub[63] = (real_t)1.0000000000000000e+12 - nmheVariables.u[55];
nmheWorkspace.ub[64] = (real_t)1.0000000000000000e+12 - nmheVariables.u[56];
nmheWorkspace.ub[65] = (real_t)1.0000000000000000e+12 - nmheVariables.u[57];
nmheWorkspace.ub[66] = (real_t)1.0000000000000000e+12 - nmheVariables.u[58];
nmheWorkspace.ub[67] = (real_t)1.0000000000000000e+12 - nmheVariables.u[59];
nmheWorkspace.ub[68] = (real_t)1.0000000000000000e+12 - nmheVariables.u[60];
nmheWorkspace.ub[69] = (real_t)1.0000000000000000e+12 - nmheVariables.u[61];
nmheWorkspace.ub[70] = (real_t)1.0000000000000000e+12 - nmheVariables.u[62];
nmheWorkspace.ub[71] = (real_t)1.0000000000000000e+12 - nmheVariables.u[63];
nmheWorkspace.ub[72] = (real_t)1.0000000000000000e+12 - nmheVariables.u[64];
nmheWorkspace.ub[73] = (real_t)1.0000000000000000e+12 - nmheVariables.u[65];
nmheWorkspace.ub[74] = (real_t)1.0000000000000000e+12 - nmheVariables.u[66];
nmheWorkspace.ub[75] = (real_t)1.0000000000000000e+12 - nmheVariables.u[67];
nmheWorkspace.ub[76] = (real_t)1.0000000000000000e+12 - nmheVariables.u[68];
nmheWorkspace.ub[77] = (real_t)1.0000000000000000e+12 - nmheVariables.u[69];
nmheWorkspace.ub[78] = (real_t)1.0000000000000000e+12 - nmheVariables.u[70];
nmheWorkspace.ub[79] = (real_t)1.0000000000000000e+12 - nmheVariables.u[71];
nmheWorkspace.ub[80] = (real_t)1.0000000000000000e+12 - nmheVariables.u[72];
nmheWorkspace.ub[81] = (real_t)1.0000000000000000e+12 - nmheVariables.u[73];
nmheWorkspace.ub[82] = (real_t)1.0000000000000000e+12 - nmheVariables.u[74];
nmheWorkspace.ub[83] = (real_t)1.0000000000000000e+12 - nmheVariables.u[75];
nmheWorkspace.ub[84] = (real_t)1.0000000000000000e+12 - nmheVariables.u[76];
nmheWorkspace.ub[85] = (real_t)1.0000000000000000e+12 - nmheVariables.u[77];
nmheWorkspace.ub[86] = (real_t)1.0000000000000000e+12 - nmheVariables.u[78];
nmheWorkspace.ub[87] = (real_t)1.0000000000000000e+12 - nmheVariables.u[79];
nmheWorkspace.ub[88] = (real_t)1.0000000000000000e+12 - nmheVariables.u[80];
nmheWorkspace.ub[89] = (real_t)1.0000000000000000e+12 - nmheVariables.u[81];
nmheWorkspace.ub[90] = (real_t)1.0000000000000000e+12 - nmheVariables.u[82];
nmheWorkspace.ub[91] = (real_t)1.0000000000000000e+12 - nmheVariables.u[83];
nmheWorkspace.ub[92] = (real_t)1.0000000000000000e+12 - nmheVariables.u[84];
nmheWorkspace.ub[93] = (real_t)1.0000000000000000e+12 - nmheVariables.u[85];
nmheWorkspace.ub[94] = (real_t)1.0000000000000000e+12 - nmheVariables.u[86];
nmheWorkspace.ub[95] = (real_t)1.0000000000000000e+12 - nmheVariables.u[87];
nmheWorkspace.ub[96] = (real_t)1.0000000000000000e+12 - nmheVariables.u[88];
nmheWorkspace.ub[97] = (real_t)1.0000000000000000e+12 - nmheVariables.u[89];
nmheWorkspace.ub[98] = (real_t)1.0000000000000000e+12 - nmheVariables.u[90];
nmheWorkspace.ub[99] = (real_t)1.0000000000000000e+12 - nmheVariables.u[91];
nmheWorkspace.ub[100] = (real_t)1.0000000000000000e+12 - nmheVariables.u[92];
nmheWorkspace.ub[101] = (real_t)1.0000000000000000e+12 - nmheVariables.u[93];
nmheWorkspace.ub[102] = (real_t)1.0000000000000000e+12 - nmheVariables.u[94];
nmheWorkspace.ub[103] = (real_t)1.0000000000000000e+12 - nmheVariables.u[95];
nmheWorkspace.ub[104] = (real_t)1.0000000000000000e+12 - nmheVariables.u[96];
nmheWorkspace.ub[105] = (real_t)1.0000000000000000e+12 - nmheVariables.u[97];
nmheWorkspace.ub[106] = (real_t)1.0000000000000000e+12 - nmheVariables.u[98];
nmheWorkspace.ub[107] = (real_t)1.0000000000000000e+12 - nmheVariables.u[99];
nmheWorkspace.ub[108] = (real_t)1.0000000000000000e+12 - nmheVariables.u[100];
nmheWorkspace.ub[109] = (real_t)1.0000000000000000e+12 - nmheVariables.u[101];
nmheWorkspace.ub[110] = (real_t)1.0000000000000000e+12 - nmheVariables.u[102];
nmheWorkspace.ub[111] = (real_t)1.0000000000000000e+12 - nmheVariables.u[103];
nmheWorkspace.ub[112] = (real_t)1.0000000000000000e+12 - nmheVariables.u[104];
nmheWorkspace.ub[113] = (real_t)1.0000000000000000e+12 - nmheVariables.u[105];
nmheWorkspace.ub[114] = (real_t)1.0000000000000000e+12 - nmheVariables.u[106];
nmheWorkspace.ub[115] = (real_t)1.0000000000000000e+12 - nmheVariables.u[107];
nmheWorkspace.ub[116] = (real_t)1.0000000000000000e+12 - nmheVariables.u[108];
nmheWorkspace.ub[117] = (real_t)1.0000000000000000e+12 - nmheVariables.u[109];
nmheWorkspace.ub[118] = (real_t)1.0000000000000000e+12 - nmheVariables.u[110];
nmheWorkspace.ub[119] = (real_t)1.0000000000000000e+12 - nmheVariables.u[111];
nmheWorkspace.ub[120] = (real_t)1.0000000000000000e+12 - nmheVariables.u[112];
nmheWorkspace.ub[121] = (real_t)1.0000000000000000e+12 - nmheVariables.u[113];
nmheWorkspace.ub[122] = (real_t)1.0000000000000000e+12 - nmheVariables.u[114];
nmheWorkspace.ub[123] = (real_t)1.0000000000000000e+12 - nmheVariables.u[115];
nmheWorkspace.ub[124] = (real_t)1.0000000000000000e+12 - nmheVariables.u[116];
nmheWorkspace.ub[125] = (real_t)1.0000000000000000e+12 - nmheVariables.u[117];
nmheWorkspace.ub[126] = (real_t)1.0000000000000000e+12 - nmheVariables.u[118];
nmheWorkspace.ub[127] = (real_t)1.0000000000000000e+12 - nmheVariables.u[119];
nmheWorkspace.ub[128] = (real_t)1.0000000000000000e+12 - nmheVariables.u[120];
nmheWorkspace.ub[129] = (real_t)1.0000000000000000e+12 - nmheVariables.u[121];
nmheWorkspace.ub[130] = (real_t)1.0000000000000000e+12 - nmheVariables.u[122];
nmheWorkspace.ub[131] = (real_t)1.0000000000000000e+12 - nmheVariables.u[123];
nmheWorkspace.ub[132] = (real_t)1.0000000000000000e+12 - nmheVariables.u[124];
nmheWorkspace.ub[133] = (real_t)1.0000000000000000e+12 - nmheVariables.u[125];
nmheWorkspace.ub[134] = (real_t)1.0000000000000000e+12 - nmheVariables.u[126];
nmheWorkspace.ub[135] = (real_t)1.0000000000000000e+12 - nmheVariables.u[127];
nmheWorkspace.ub[136] = (real_t)1.0000000000000000e+12 - nmheVariables.u[128];
nmheWorkspace.ub[137] = (real_t)1.0000000000000000e+12 - nmheVariables.u[129];
nmheWorkspace.ub[138] = (real_t)1.0000000000000000e+12 - nmheVariables.u[130];
nmheWorkspace.ub[139] = (real_t)1.0000000000000000e+12 - nmheVariables.u[131];
nmheWorkspace.ub[140] = (real_t)1.0000000000000000e+12 - nmheVariables.u[132];
nmheWorkspace.ub[141] = (real_t)1.0000000000000000e+12 - nmheVariables.u[133];
nmheWorkspace.ub[142] = (real_t)1.0000000000000000e+12 - nmheVariables.u[134];
nmheWorkspace.ub[143] = (real_t)1.0000000000000000e+12 - nmheVariables.u[135];
nmheWorkspace.ub[144] = (real_t)1.0000000000000000e+12 - nmheVariables.u[136];
nmheWorkspace.ub[145] = (real_t)1.0000000000000000e+12 - nmheVariables.u[137];
nmheWorkspace.ub[146] = (real_t)1.0000000000000000e+12 - nmheVariables.u[138];
nmheWorkspace.ub[147] = (real_t)1.0000000000000000e+12 - nmheVariables.u[139];
nmheWorkspace.ub[148] = (real_t)1.0000000000000000e+12 - nmheVariables.u[140];
nmheWorkspace.ub[149] = (real_t)1.0000000000000000e+12 - nmheVariables.u[141];
nmheWorkspace.ub[150] = (real_t)1.0000000000000000e+12 - nmheVariables.u[142];
nmheWorkspace.ub[151] = (real_t)1.0000000000000000e+12 - nmheVariables.u[143];
nmheWorkspace.ub[152] = (real_t)1.0000000000000000e+12 - nmheVariables.u[144];
nmheWorkspace.ub[153] = (real_t)1.0000000000000000e+12 - nmheVariables.u[145];
nmheWorkspace.ub[154] = (real_t)1.0000000000000000e+12 - nmheVariables.u[146];
nmheWorkspace.ub[155] = (real_t)1.0000000000000000e+12 - nmheVariables.u[147];
nmheWorkspace.ub[156] = (real_t)1.0000000000000000e+12 - nmheVariables.u[148];
nmheWorkspace.ub[157] = (real_t)1.0000000000000000e+12 - nmheVariables.u[149];
nmheWorkspace.ub[158] = (real_t)1.0000000000000000e+12 - nmheVariables.u[150];
nmheWorkspace.ub[159] = (real_t)1.0000000000000000e+12 - nmheVariables.u[151];
nmheWorkspace.ub[160] = (real_t)1.0000000000000000e+12 - nmheVariables.u[152];
nmheWorkspace.ub[161] = (real_t)1.0000000000000000e+12 - nmheVariables.u[153];
nmheWorkspace.ub[162] = (real_t)1.0000000000000000e+12 - nmheVariables.u[154];
nmheWorkspace.ub[163] = (real_t)1.0000000000000000e+12 - nmheVariables.u[155];
nmheWorkspace.ub[164] = (real_t)1.0000000000000000e+12 - nmheVariables.u[156];
nmheWorkspace.ub[165] = (real_t)1.0000000000000000e+12 - nmheVariables.u[157];
nmheWorkspace.ub[166] = (real_t)1.0000000000000000e+12 - nmheVariables.u[158];
nmheWorkspace.ub[167] = (real_t)1.0000000000000000e+12 - nmheVariables.u[159];
nmheWorkspace.ub[168] = (real_t)1.0000000000000000e+12 - nmheVariables.u[160];
nmheWorkspace.ub[169] = (real_t)1.0000000000000000e+12 - nmheVariables.u[161];
nmheWorkspace.ub[170] = (real_t)1.0000000000000000e+12 - nmheVariables.u[162];
nmheWorkspace.ub[171] = (real_t)1.0000000000000000e+12 - nmheVariables.u[163];
nmheWorkspace.ub[172] = (real_t)1.0000000000000000e+12 - nmheVariables.u[164];
nmheWorkspace.ub[173] = (real_t)1.0000000000000000e+12 - nmheVariables.u[165];
nmheWorkspace.ub[174] = (real_t)1.0000000000000000e+12 - nmheVariables.u[166];
nmheWorkspace.ub[175] = (real_t)1.0000000000000000e+12 - nmheVariables.u[167];
nmheWorkspace.ub[176] = (real_t)1.0000000000000000e+12 - nmheVariables.u[168];
nmheWorkspace.ub[177] = (real_t)1.0000000000000000e+12 - nmheVariables.u[169];
nmheWorkspace.ub[178] = (real_t)1.0000000000000000e+12 - nmheVariables.u[170];
nmheWorkspace.ub[179] = (real_t)1.0000000000000000e+12 - nmheVariables.u[171];
nmheWorkspace.ub[180] = (real_t)1.0000000000000000e+12 - nmheVariables.u[172];
nmheWorkspace.ub[181] = (real_t)1.0000000000000000e+12 - nmheVariables.u[173];
nmheWorkspace.ub[182] = (real_t)1.0000000000000000e+12 - nmheVariables.u[174];
nmheWorkspace.ub[183] = (real_t)1.0000000000000000e+12 - nmheVariables.u[175];
nmheWorkspace.ub[184] = (real_t)1.0000000000000000e+12 - nmheVariables.u[176];
nmheWorkspace.ub[185] = (real_t)1.0000000000000000e+12 - nmheVariables.u[177];
nmheWorkspace.ub[186] = (real_t)1.0000000000000000e+12 - nmheVariables.u[178];
nmheWorkspace.ub[187] = (real_t)1.0000000000000000e+12 - nmheVariables.u[179];
nmheWorkspace.ub[188] = (real_t)1.0000000000000000e+12 - nmheVariables.u[180];
nmheWorkspace.ub[189] = (real_t)1.0000000000000000e+12 - nmheVariables.u[181];
nmheWorkspace.ub[190] = (real_t)1.0000000000000000e+12 - nmheVariables.u[182];
nmheWorkspace.ub[191] = (real_t)1.0000000000000000e+12 - nmheVariables.u[183];
nmheWorkspace.ub[192] = (real_t)1.0000000000000000e+12 - nmheVariables.u[184];
nmheWorkspace.ub[193] = (real_t)1.0000000000000000e+12 - nmheVariables.u[185];
nmheWorkspace.ub[194] = (real_t)1.0000000000000000e+12 - nmheVariables.u[186];
nmheWorkspace.ub[195] = (real_t)1.0000000000000000e+12 - nmheVariables.u[187];
nmheWorkspace.ub[196] = (real_t)1.0000000000000000e+12 - nmheVariables.u[188];
nmheWorkspace.ub[197] = (real_t)1.0000000000000000e+12 - nmheVariables.u[189];
nmheWorkspace.ub[198] = (real_t)1.0000000000000000e+12 - nmheVariables.u[190];
nmheWorkspace.ub[199] = (real_t)1.0000000000000000e+12 - nmheVariables.u[191];
nmheWorkspace.ub[200] = (real_t)1.0000000000000000e+12 - nmheVariables.u[192];
nmheWorkspace.ub[201] = (real_t)1.0000000000000000e+12 - nmheVariables.u[193];
nmheWorkspace.ub[202] = (real_t)1.0000000000000000e+12 - nmheVariables.u[194];
nmheWorkspace.ub[203] = (real_t)1.0000000000000000e+12 - nmheVariables.u[195];
nmheWorkspace.ub[204] = (real_t)1.0000000000000000e+12 - nmheVariables.u[196];
nmheWorkspace.ub[205] = (real_t)1.0000000000000000e+12 - nmheVariables.u[197];
nmheWorkspace.ub[206] = (real_t)1.0000000000000000e+12 - nmheVariables.u[198];
nmheWorkspace.ub[207] = (real_t)1.0000000000000000e+12 - nmheVariables.u[199];

for (lRun1 = 0; lRun1 < 200; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 8;
lRun4 = ((lRun3) / (8)) + (1);
nmheWorkspace.A[lRun1 * 208] = nmheWorkspace.evGx[lRun3 * 8];
nmheWorkspace.A[lRun1 * 208 + 1] = nmheWorkspace.evGx[lRun3 * 8 + 1];
nmheWorkspace.A[lRun1 * 208 + 2] = nmheWorkspace.evGx[lRun3 * 8 + 2];
nmheWorkspace.A[lRun1 * 208 + 3] = nmheWorkspace.evGx[lRun3 * 8 + 3];
nmheWorkspace.A[lRun1 * 208 + 4] = nmheWorkspace.evGx[lRun3 * 8 + 4];
nmheWorkspace.A[lRun1 * 208 + 5] = nmheWorkspace.evGx[lRun3 * 8 + 5];
nmheWorkspace.A[lRun1 * 208 + 6] = nmheWorkspace.evGx[lRun3 * 8 + 6];
nmheWorkspace.A[lRun1 * 208 + 7] = nmheWorkspace.evGx[lRun3 * 8 + 7];
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = (((((lRun4) * (lRun4-1)) / (2)) + (lRun2)) * (8)) + ((lRun3) % (8));
nmheWorkspace.A[(lRun1 * 208) + (lRun2 * 2 + 8)] = nmheWorkspace.E[lRun5 * 2];
nmheWorkspace.A[(lRun1 * 208) + (lRun2 * 2 + 9)] = nmheWorkspace.E[lRun5 * 2 + 1];
}
}

}

void nmhe_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
real_t tmp;

for (lRun2 = 0; lRun2 < 800; ++lRun2)
nmheWorkspace.Dy[lRun2] -= nmheVariables.y[lRun2];

nmheWorkspace.DyN[0] -= nmheVariables.yN[0];
nmheWorkspace.DyN[1] -= nmheVariables.yN[1];
nmheWorkspace.DyN[2] -= nmheVariables.yN[2];
nmheWorkspace.DyN[3] -= nmheVariables.yN[3];
nmheWorkspace.DyN[4] -= nmheVariables.yN[4];
nmheWorkspace.DyN[5] -= nmheVariables.yN[5];

nmhe_multRDy( nmheWorkspace.R2, nmheWorkspace.Dy, &(nmheWorkspace.g[ 8 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 16 ]), &(nmheWorkspace.Dy[ 8 ]), &(nmheWorkspace.g[ 10 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 32 ]), &(nmheWorkspace.Dy[ 16 ]), &(nmheWorkspace.g[ 12 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 48 ]), &(nmheWorkspace.Dy[ 24 ]), &(nmheWorkspace.g[ 14 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 64 ]), &(nmheWorkspace.Dy[ 32 ]), &(nmheWorkspace.g[ 16 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 80 ]), &(nmheWorkspace.Dy[ 40 ]), &(nmheWorkspace.g[ 18 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 96 ]), &(nmheWorkspace.Dy[ 48 ]), &(nmheWorkspace.g[ 20 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 112 ]), &(nmheWorkspace.Dy[ 56 ]), &(nmheWorkspace.g[ 22 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 128 ]), &(nmheWorkspace.Dy[ 64 ]), &(nmheWorkspace.g[ 24 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 144 ]), &(nmheWorkspace.Dy[ 72 ]), &(nmheWorkspace.g[ 26 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 160 ]), &(nmheWorkspace.Dy[ 80 ]), &(nmheWorkspace.g[ 28 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 176 ]), &(nmheWorkspace.Dy[ 88 ]), &(nmheWorkspace.g[ 30 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 192 ]), &(nmheWorkspace.Dy[ 96 ]), &(nmheWorkspace.g[ 32 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 208 ]), &(nmheWorkspace.Dy[ 104 ]), &(nmheWorkspace.g[ 34 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 224 ]), &(nmheWorkspace.Dy[ 112 ]), &(nmheWorkspace.g[ 36 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 240 ]), &(nmheWorkspace.Dy[ 120 ]), &(nmheWorkspace.g[ 38 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 256 ]), &(nmheWorkspace.Dy[ 128 ]), &(nmheWorkspace.g[ 40 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 272 ]), &(nmheWorkspace.Dy[ 136 ]), &(nmheWorkspace.g[ 42 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 288 ]), &(nmheWorkspace.Dy[ 144 ]), &(nmheWorkspace.g[ 44 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 304 ]), &(nmheWorkspace.Dy[ 152 ]), &(nmheWorkspace.g[ 46 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 320 ]), &(nmheWorkspace.Dy[ 160 ]), &(nmheWorkspace.g[ 48 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 336 ]), &(nmheWorkspace.Dy[ 168 ]), &(nmheWorkspace.g[ 50 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 352 ]), &(nmheWorkspace.Dy[ 176 ]), &(nmheWorkspace.g[ 52 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 368 ]), &(nmheWorkspace.Dy[ 184 ]), &(nmheWorkspace.g[ 54 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 384 ]), &(nmheWorkspace.Dy[ 192 ]), &(nmheWorkspace.g[ 56 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 400 ]), &(nmheWorkspace.Dy[ 200 ]), &(nmheWorkspace.g[ 58 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 416 ]), &(nmheWorkspace.Dy[ 208 ]), &(nmheWorkspace.g[ 60 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 432 ]), &(nmheWorkspace.Dy[ 216 ]), &(nmheWorkspace.g[ 62 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 448 ]), &(nmheWorkspace.Dy[ 224 ]), &(nmheWorkspace.g[ 64 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 464 ]), &(nmheWorkspace.Dy[ 232 ]), &(nmheWorkspace.g[ 66 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 480 ]), &(nmheWorkspace.Dy[ 240 ]), &(nmheWorkspace.g[ 68 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 496 ]), &(nmheWorkspace.Dy[ 248 ]), &(nmheWorkspace.g[ 70 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 512 ]), &(nmheWorkspace.Dy[ 256 ]), &(nmheWorkspace.g[ 72 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 528 ]), &(nmheWorkspace.Dy[ 264 ]), &(nmheWorkspace.g[ 74 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 544 ]), &(nmheWorkspace.Dy[ 272 ]), &(nmheWorkspace.g[ 76 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 560 ]), &(nmheWorkspace.Dy[ 280 ]), &(nmheWorkspace.g[ 78 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 576 ]), &(nmheWorkspace.Dy[ 288 ]), &(nmheWorkspace.g[ 80 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 592 ]), &(nmheWorkspace.Dy[ 296 ]), &(nmheWorkspace.g[ 82 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 608 ]), &(nmheWorkspace.Dy[ 304 ]), &(nmheWorkspace.g[ 84 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 624 ]), &(nmheWorkspace.Dy[ 312 ]), &(nmheWorkspace.g[ 86 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 640 ]), &(nmheWorkspace.Dy[ 320 ]), &(nmheWorkspace.g[ 88 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 656 ]), &(nmheWorkspace.Dy[ 328 ]), &(nmheWorkspace.g[ 90 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 672 ]), &(nmheWorkspace.Dy[ 336 ]), &(nmheWorkspace.g[ 92 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 688 ]), &(nmheWorkspace.Dy[ 344 ]), &(nmheWorkspace.g[ 94 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 704 ]), &(nmheWorkspace.Dy[ 352 ]), &(nmheWorkspace.g[ 96 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 720 ]), &(nmheWorkspace.Dy[ 360 ]), &(nmheWorkspace.g[ 98 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 736 ]), &(nmheWorkspace.Dy[ 368 ]), &(nmheWorkspace.g[ 100 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 752 ]), &(nmheWorkspace.Dy[ 376 ]), &(nmheWorkspace.g[ 102 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 768 ]), &(nmheWorkspace.Dy[ 384 ]), &(nmheWorkspace.g[ 104 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 784 ]), &(nmheWorkspace.Dy[ 392 ]), &(nmheWorkspace.g[ 106 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 800 ]), &(nmheWorkspace.Dy[ 400 ]), &(nmheWorkspace.g[ 108 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 816 ]), &(nmheWorkspace.Dy[ 408 ]), &(nmheWorkspace.g[ 110 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 832 ]), &(nmheWorkspace.Dy[ 416 ]), &(nmheWorkspace.g[ 112 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 848 ]), &(nmheWorkspace.Dy[ 424 ]), &(nmheWorkspace.g[ 114 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 864 ]), &(nmheWorkspace.Dy[ 432 ]), &(nmheWorkspace.g[ 116 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 880 ]), &(nmheWorkspace.Dy[ 440 ]), &(nmheWorkspace.g[ 118 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 896 ]), &(nmheWorkspace.Dy[ 448 ]), &(nmheWorkspace.g[ 120 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 912 ]), &(nmheWorkspace.Dy[ 456 ]), &(nmheWorkspace.g[ 122 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 928 ]), &(nmheWorkspace.Dy[ 464 ]), &(nmheWorkspace.g[ 124 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 944 ]), &(nmheWorkspace.Dy[ 472 ]), &(nmheWorkspace.g[ 126 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 960 ]), &(nmheWorkspace.Dy[ 480 ]), &(nmheWorkspace.g[ 128 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 976 ]), &(nmheWorkspace.Dy[ 488 ]), &(nmheWorkspace.g[ 130 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 992 ]), &(nmheWorkspace.Dy[ 496 ]), &(nmheWorkspace.g[ 132 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1008 ]), &(nmheWorkspace.Dy[ 504 ]), &(nmheWorkspace.g[ 134 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1024 ]), &(nmheWorkspace.Dy[ 512 ]), &(nmheWorkspace.g[ 136 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1040 ]), &(nmheWorkspace.Dy[ 520 ]), &(nmheWorkspace.g[ 138 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1056 ]), &(nmheWorkspace.Dy[ 528 ]), &(nmheWorkspace.g[ 140 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1072 ]), &(nmheWorkspace.Dy[ 536 ]), &(nmheWorkspace.g[ 142 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1088 ]), &(nmheWorkspace.Dy[ 544 ]), &(nmheWorkspace.g[ 144 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1104 ]), &(nmheWorkspace.Dy[ 552 ]), &(nmheWorkspace.g[ 146 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1120 ]), &(nmheWorkspace.Dy[ 560 ]), &(nmheWorkspace.g[ 148 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1136 ]), &(nmheWorkspace.Dy[ 568 ]), &(nmheWorkspace.g[ 150 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1152 ]), &(nmheWorkspace.Dy[ 576 ]), &(nmheWorkspace.g[ 152 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1168 ]), &(nmheWorkspace.Dy[ 584 ]), &(nmheWorkspace.g[ 154 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1184 ]), &(nmheWorkspace.Dy[ 592 ]), &(nmheWorkspace.g[ 156 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1200 ]), &(nmheWorkspace.Dy[ 600 ]), &(nmheWorkspace.g[ 158 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1216 ]), &(nmheWorkspace.Dy[ 608 ]), &(nmheWorkspace.g[ 160 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1232 ]), &(nmheWorkspace.Dy[ 616 ]), &(nmheWorkspace.g[ 162 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1248 ]), &(nmheWorkspace.Dy[ 624 ]), &(nmheWorkspace.g[ 164 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1264 ]), &(nmheWorkspace.Dy[ 632 ]), &(nmheWorkspace.g[ 166 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1280 ]), &(nmheWorkspace.Dy[ 640 ]), &(nmheWorkspace.g[ 168 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1296 ]), &(nmheWorkspace.Dy[ 648 ]), &(nmheWorkspace.g[ 170 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1312 ]), &(nmheWorkspace.Dy[ 656 ]), &(nmheWorkspace.g[ 172 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1328 ]), &(nmheWorkspace.Dy[ 664 ]), &(nmheWorkspace.g[ 174 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1344 ]), &(nmheWorkspace.Dy[ 672 ]), &(nmheWorkspace.g[ 176 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1360 ]), &(nmheWorkspace.Dy[ 680 ]), &(nmheWorkspace.g[ 178 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1376 ]), &(nmheWorkspace.Dy[ 688 ]), &(nmheWorkspace.g[ 180 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1392 ]), &(nmheWorkspace.Dy[ 696 ]), &(nmheWorkspace.g[ 182 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1408 ]), &(nmheWorkspace.Dy[ 704 ]), &(nmheWorkspace.g[ 184 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1424 ]), &(nmheWorkspace.Dy[ 712 ]), &(nmheWorkspace.g[ 186 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1440 ]), &(nmheWorkspace.Dy[ 720 ]), &(nmheWorkspace.g[ 188 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1456 ]), &(nmheWorkspace.Dy[ 728 ]), &(nmheWorkspace.g[ 190 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1472 ]), &(nmheWorkspace.Dy[ 736 ]), &(nmheWorkspace.g[ 192 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1488 ]), &(nmheWorkspace.Dy[ 744 ]), &(nmheWorkspace.g[ 194 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1504 ]), &(nmheWorkspace.Dy[ 752 ]), &(nmheWorkspace.g[ 196 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1520 ]), &(nmheWorkspace.Dy[ 760 ]), &(nmheWorkspace.g[ 198 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1536 ]), &(nmheWorkspace.Dy[ 768 ]), &(nmheWorkspace.g[ 200 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1552 ]), &(nmheWorkspace.Dy[ 776 ]), &(nmheWorkspace.g[ 202 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1568 ]), &(nmheWorkspace.Dy[ 784 ]), &(nmheWorkspace.g[ 204 ]) );
nmhe_multRDy( &(nmheWorkspace.R2[ 1584 ]), &(nmheWorkspace.Dy[ 792 ]), &(nmheWorkspace.g[ 206 ]) );

nmhe_multQDy( nmheWorkspace.Q2, nmheWorkspace.Dy, nmheWorkspace.QDy );
nmhe_multQDy( &(nmheWorkspace.Q2[ 64 ]), &(nmheWorkspace.Dy[ 8 ]), &(nmheWorkspace.QDy[ 8 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 128 ]), &(nmheWorkspace.Dy[ 16 ]), &(nmheWorkspace.QDy[ 16 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 192 ]), &(nmheWorkspace.Dy[ 24 ]), &(nmheWorkspace.QDy[ 24 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 256 ]), &(nmheWorkspace.Dy[ 32 ]), &(nmheWorkspace.QDy[ 32 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 320 ]), &(nmheWorkspace.Dy[ 40 ]), &(nmheWorkspace.QDy[ 40 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 384 ]), &(nmheWorkspace.Dy[ 48 ]), &(nmheWorkspace.QDy[ 48 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 448 ]), &(nmheWorkspace.Dy[ 56 ]), &(nmheWorkspace.QDy[ 56 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 512 ]), &(nmheWorkspace.Dy[ 64 ]), &(nmheWorkspace.QDy[ 64 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 576 ]), &(nmheWorkspace.Dy[ 72 ]), &(nmheWorkspace.QDy[ 72 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 640 ]), &(nmheWorkspace.Dy[ 80 ]), &(nmheWorkspace.QDy[ 80 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 704 ]), &(nmheWorkspace.Dy[ 88 ]), &(nmheWorkspace.QDy[ 88 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 768 ]), &(nmheWorkspace.Dy[ 96 ]), &(nmheWorkspace.QDy[ 96 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 832 ]), &(nmheWorkspace.Dy[ 104 ]), &(nmheWorkspace.QDy[ 104 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 896 ]), &(nmheWorkspace.Dy[ 112 ]), &(nmheWorkspace.QDy[ 112 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 960 ]), &(nmheWorkspace.Dy[ 120 ]), &(nmheWorkspace.QDy[ 120 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1024 ]), &(nmheWorkspace.Dy[ 128 ]), &(nmheWorkspace.QDy[ 128 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1088 ]), &(nmheWorkspace.Dy[ 136 ]), &(nmheWorkspace.QDy[ 136 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1152 ]), &(nmheWorkspace.Dy[ 144 ]), &(nmheWorkspace.QDy[ 144 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1216 ]), &(nmheWorkspace.Dy[ 152 ]), &(nmheWorkspace.QDy[ 152 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1280 ]), &(nmheWorkspace.Dy[ 160 ]), &(nmheWorkspace.QDy[ 160 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1344 ]), &(nmheWorkspace.Dy[ 168 ]), &(nmheWorkspace.QDy[ 168 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1408 ]), &(nmheWorkspace.Dy[ 176 ]), &(nmheWorkspace.QDy[ 176 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1472 ]), &(nmheWorkspace.Dy[ 184 ]), &(nmheWorkspace.QDy[ 184 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1536 ]), &(nmheWorkspace.Dy[ 192 ]), &(nmheWorkspace.QDy[ 192 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1600 ]), &(nmheWorkspace.Dy[ 200 ]), &(nmheWorkspace.QDy[ 200 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1664 ]), &(nmheWorkspace.Dy[ 208 ]), &(nmheWorkspace.QDy[ 208 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1728 ]), &(nmheWorkspace.Dy[ 216 ]), &(nmheWorkspace.QDy[ 216 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1792 ]), &(nmheWorkspace.Dy[ 224 ]), &(nmheWorkspace.QDy[ 224 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1856 ]), &(nmheWorkspace.Dy[ 232 ]), &(nmheWorkspace.QDy[ 232 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1920 ]), &(nmheWorkspace.Dy[ 240 ]), &(nmheWorkspace.QDy[ 240 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 1984 ]), &(nmheWorkspace.Dy[ 248 ]), &(nmheWorkspace.QDy[ 248 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2048 ]), &(nmheWorkspace.Dy[ 256 ]), &(nmheWorkspace.QDy[ 256 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2112 ]), &(nmheWorkspace.Dy[ 264 ]), &(nmheWorkspace.QDy[ 264 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2176 ]), &(nmheWorkspace.Dy[ 272 ]), &(nmheWorkspace.QDy[ 272 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2240 ]), &(nmheWorkspace.Dy[ 280 ]), &(nmheWorkspace.QDy[ 280 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2304 ]), &(nmheWorkspace.Dy[ 288 ]), &(nmheWorkspace.QDy[ 288 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2368 ]), &(nmheWorkspace.Dy[ 296 ]), &(nmheWorkspace.QDy[ 296 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2432 ]), &(nmheWorkspace.Dy[ 304 ]), &(nmheWorkspace.QDy[ 304 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2496 ]), &(nmheWorkspace.Dy[ 312 ]), &(nmheWorkspace.QDy[ 312 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2560 ]), &(nmheWorkspace.Dy[ 320 ]), &(nmheWorkspace.QDy[ 320 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2624 ]), &(nmheWorkspace.Dy[ 328 ]), &(nmheWorkspace.QDy[ 328 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2688 ]), &(nmheWorkspace.Dy[ 336 ]), &(nmheWorkspace.QDy[ 336 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2752 ]), &(nmheWorkspace.Dy[ 344 ]), &(nmheWorkspace.QDy[ 344 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2816 ]), &(nmheWorkspace.Dy[ 352 ]), &(nmheWorkspace.QDy[ 352 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2880 ]), &(nmheWorkspace.Dy[ 360 ]), &(nmheWorkspace.QDy[ 360 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 2944 ]), &(nmheWorkspace.Dy[ 368 ]), &(nmheWorkspace.QDy[ 368 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3008 ]), &(nmheWorkspace.Dy[ 376 ]), &(nmheWorkspace.QDy[ 376 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3072 ]), &(nmheWorkspace.Dy[ 384 ]), &(nmheWorkspace.QDy[ 384 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3136 ]), &(nmheWorkspace.Dy[ 392 ]), &(nmheWorkspace.QDy[ 392 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3200 ]), &(nmheWorkspace.Dy[ 400 ]), &(nmheWorkspace.QDy[ 400 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3264 ]), &(nmheWorkspace.Dy[ 408 ]), &(nmheWorkspace.QDy[ 408 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3328 ]), &(nmheWorkspace.Dy[ 416 ]), &(nmheWorkspace.QDy[ 416 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3392 ]), &(nmheWorkspace.Dy[ 424 ]), &(nmheWorkspace.QDy[ 424 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3456 ]), &(nmheWorkspace.Dy[ 432 ]), &(nmheWorkspace.QDy[ 432 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3520 ]), &(nmheWorkspace.Dy[ 440 ]), &(nmheWorkspace.QDy[ 440 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3584 ]), &(nmheWorkspace.Dy[ 448 ]), &(nmheWorkspace.QDy[ 448 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3648 ]), &(nmheWorkspace.Dy[ 456 ]), &(nmheWorkspace.QDy[ 456 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3712 ]), &(nmheWorkspace.Dy[ 464 ]), &(nmheWorkspace.QDy[ 464 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3776 ]), &(nmheWorkspace.Dy[ 472 ]), &(nmheWorkspace.QDy[ 472 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3840 ]), &(nmheWorkspace.Dy[ 480 ]), &(nmheWorkspace.QDy[ 480 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3904 ]), &(nmheWorkspace.Dy[ 488 ]), &(nmheWorkspace.QDy[ 488 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 3968 ]), &(nmheWorkspace.Dy[ 496 ]), &(nmheWorkspace.QDy[ 496 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4032 ]), &(nmheWorkspace.Dy[ 504 ]), &(nmheWorkspace.QDy[ 504 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4096 ]), &(nmheWorkspace.Dy[ 512 ]), &(nmheWorkspace.QDy[ 512 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4160 ]), &(nmheWorkspace.Dy[ 520 ]), &(nmheWorkspace.QDy[ 520 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4224 ]), &(nmheWorkspace.Dy[ 528 ]), &(nmheWorkspace.QDy[ 528 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4288 ]), &(nmheWorkspace.Dy[ 536 ]), &(nmheWorkspace.QDy[ 536 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4352 ]), &(nmheWorkspace.Dy[ 544 ]), &(nmheWorkspace.QDy[ 544 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4416 ]), &(nmheWorkspace.Dy[ 552 ]), &(nmheWorkspace.QDy[ 552 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4480 ]), &(nmheWorkspace.Dy[ 560 ]), &(nmheWorkspace.QDy[ 560 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4544 ]), &(nmheWorkspace.Dy[ 568 ]), &(nmheWorkspace.QDy[ 568 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4608 ]), &(nmheWorkspace.Dy[ 576 ]), &(nmheWorkspace.QDy[ 576 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4672 ]), &(nmheWorkspace.Dy[ 584 ]), &(nmheWorkspace.QDy[ 584 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4736 ]), &(nmheWorkspace.Dy[ 592 ]), &(nmheWorkspace.QDy[ 592 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4800 ]), &(nmheWorkspace.Dy[ 600 ]), &(nmheWorkspace.QDy[ 600 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4864 ]), &(nmheWorkspace.Dy[ 608 ]), &(nmheWorkspace.QDy[ 608 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4928 ]), &(nmheWorkspace.Dy[ 616 ]), &(nmheWorkspace.QDy[ 616 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 4992 ]), &(nmheWorkspace.Dy[ 624 ]), &(nmheWorkspace.QDy[ 624 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5056 ]), &(nmheWorkspace.Dy[ 632 ]), &(nmheWorkspace.QDy[ 632 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5120 ]), &(nmheWorkspace.Dy[ 640 ]), &(nmheWorkspace.QDy[ 640 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5184 ]), &(nmheWorkspace.Dy[ 648 ]), &(nmheWorkspace.QDy[ 648 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5248 ]), &(nmheWorkspace.Dy[ 656 ]), &(nmheWorkspace.QDy[ 656 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5312 ]), &(nmheWorkspace.Dy[ 664 ]), &(nmheWorkspace.QDy[ 664 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5376 ]), &(nmheWorkspace.Dy[ 672 ]), &(nmheWorkspace.QDy[ 672 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5440 ]), &(nmheWorkspace.Dy[ 680 ]), &(nmheWorkspace.QDy[ 680 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5504 ]), &(nmheWorkspace.Dy[ 688 ]), &(nmheWorkspace.QDy[ 688 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5568 ]), &(nmheWorkspace.Dy[ 696 ]), &(nmheWorkspace.QDy[ 696 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5632 ]), &(nmheWorkspace.Dy[ 704 ]), &(nmheWorkspace.QDy[ 704 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5696 ]), &(nmheWorkspace.Dy[ 712 ]), &(nmheWorkspace.QDy[ 712 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5760 ]), &(nmheWorkspace.Dy[ 720 ]), &(nmheWorkspace.QDy[ 720 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5824 ]), &(nmheWorkspace.Dy[ 728 ]), &(nmheWorkspace.QDy[ 728 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5888 ]), &(nmheWorkspace.Dy[ 736 ]), &(nmheWorkspace.QDy[ 736 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 5952 ]), &(nmheWorkspace.Dy[ 744 ]), &(nmheWorkspace.QDy[ 744 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 6016 ]), &(nmheWorkspace.Dy[ 752 ]), &(nmheWorkspace.QDy[ 752 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 6080 ]), &(nmheWorkspace.Dy[ 760 ]), &(nmheWorkspace.QDy[ 760 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 6144 ]), &(nmheWorkspace.Dy[ 768 ]), &(nmheWorkspace.QDy[ 768 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 6208 ]), &(nmheWorkspace.Dy[ 776 ]), &(nmheWorkspace.QDy[ 776 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 6272 ]), &(nmheWorkspace.Dy[ 784 ]), &(nmheWorkspace.QDy[ 784 ]) );
nmhe_multQDy( &(nmheWorkspace.Q2[ 6336 ]), &(nmheWorkspace.Dy[ 792 ]), &(nmheWorkspace.QDy[ 792 ]) );

nmheWorkspace.QDy[800] = + nmheWorkspace.QN2[0]*nmheWorkspace.DyN[0] + nmheWorkspace.QN2[1]*nmheWorkspace.DyN[1] + nmheWorkspace.QN2[2]*nmheWorkspace.DyN[2] + nmheWorkspace.QN2[3]*nmheWorkspace.DyN[3] + nmheWorkspace.QN2[4]*nmheWorkspace.DyN[4] + nmheWorkspace.QN2[5]*nmheWorkspace.DyN[5];
nmheWorkspace.QDy[801] = + nmheWorkspace.QN2[6]*nmheWorkspace.DyN[0] + nmheWorkspace.QN2[7]*nmheWorkspace.DyN[1] + nmheWorkspace.QN2[8]*nmheWorkspace.DyN[2] + nmheWorkspace.QN2[9]*nmheWorkspace.DyN[3] + nmheWorkspace.QN2[10]*nmheWorkspace.DyN[4] + nmheWorkspace.QN2[11]*nmheWorkspace.DyN[5];
nmheWorkspace.QDy[802] = + nmheWorkspace.QN2[12]*nmheWorkspace.DyN[0] + nmheWorkspace.QN2[13]*nmheWorkspace.DyN[1] + nmheWorkspace.QN2[14]*nmheWorkspace.DyN[2] + nmheWorkspace.QN2[15]*nmheWorkspace.DyN[3] + nmheWorkspace.QN2[16]*nmheWorkspace.DyN[4] + nmheWorkspace.QN2[17]*nmheWorkspace.DyN[5];
nmheWorkspace.QDy[803] = + nmheWorkspace.QN2[18]*nmheWorkspace.DyN[0] + nmheWorkspace.QN2[19]*nmheWorkspace.DyN[1] + nmheWorkspace.QN2[20]*nmheWorkspace.DyN[2] + nmheWorkspace.QN2[21]*nmheWorkspace.DyN[3] + nmheWorkspace.QN2[22]*nmheWorkspace.DyN[4] + nmheWorkspace.QN2[23]*nmheWorkspace.DyN[5];
nmheWorkspace.QDy[804] = + nmheWorkspace.QN2[24]*nmheWorkspace.DyN[0] + nmheWorkspace.QN2[25]*nmheWorkspace.DyN[1] + nmheWorkspace.QN2[26]*nmheWorkspace.DyN[2] + nmheWorkspace.QN2[27]*nmheWorkspace.DyN[3] + nmheWorkspace.QN2[28]*nmheWorkspace.DyN[4] + nmheWorkspace.QN2[29]*nmheWorkspace.DyN[5];
nmheWorkspace.QDy[805] = + nmheWorkspace.QN2[30]*nmheWorkspace.DyN[0] + nmheWorkspace.QN2[31]*nmheWorkspace.DyN[1] + nmheWorkspace.QN2[32]*nmheWorkspace.DyN[2] + nmheWorkspace.QN2[33]*nmheWorkspace.DyN[3] + nmheWorkspace.QN2[34]*nmheWorkspace.DyN[4] + nmheWorkspace.QN2[35]*nmheWorkspace.DyN[5];
nmheWorkspace.QDy[806] = + nmheWorkspace.QN2[36]*nmheWorkspace.DyN[0] + nmheWorkspace.QN2[37]*nmheWorkspace.DyN[1] + nmheWorkspace.QN2[38]*nmheWorkspace.DyN[2] + nmheWorkspace.QN2[39]*nmheWorkspace.DyN[3] + nmheWorkspace.QN2[40]*nmheWorkspace.DyN[4] + nmheWorkspace.QN2[41]*nmheWorkspace.DyN[5];
nmheWorkspace.QDy[807] = + nmheWorkspace.QN2[42]*nmheWorkspace.DyN[0] + nmheWorkspace.QN2[43]*nmheWorkspace.DyN[1] + nmheWorkspace.QN2[44]*nmheWorkspace.DyN[2] + nmheWorkspace.QN2[45]*nmheWorkspace.DyN[3] + nmheWorkspace.QN2[46]*nmheWorkspace.DyN[4] + nmheWorkspace.QN2[47]*nmheWorkspace.DyN[5];

for (lRun2 = 0; lRun2 < 800; ++lRun2)
nmheWorkspace.QDy[lRun2 + 8] += nmheWorkspace.Qd[lRun2];


for (lRun2 = 0; lRun2 < 8; ++lRun2)
{
for (lRun4 = 0; lRun4 < 1; ++lRun4)
{
real_t t = 0.0;
for (lRun5 = 0; lRun5 < 800; ++lRun5)
{
t += + nmheWorkspace.evGx[(lRun5 * 8) + (lRun2)]*nmheWorkspace.QDy[(lRun5 + 8) + (lRun4)];
}
nmheWorkspace.g[(lRun2) + (lRun4)] = t;
}
}

nmheWorkspace.g[0] += nmheWorkspace.QDy[0];
nmheWorkspace.g[1] += nmheWorkspace.QDy[1];
nmheWorkspace.g[2] += nmheWorkspace.QDy[2];
nmheWorkspace.g[3] += nmheWorkspace.QDy[3];
nmheWorkspace.g[4] += nmheWorkspace.QDy[4];
nmheWorkspace.g[5] += nmheWorkspace.QDy[5];
nmheWorkspace.g[6] += nmheWorkspace.QDy[6];
nmheWorkspace.g[7] += nmheWorkspace.QDy[7];
nmheWorkspace.DxAC[0] = nmheVariables.x[0] - nmheVariables.xAC[0];
nmheWorkspace.DxAC[1] = nmheVariables.x[1] - nmheVariables.xAC[1];
nmheWorkspace.DxAC[2] = nmheVariables.x[2] - nmheVariables.xAC[2];
nmheWorkspace.DxAC[3] = nmheVariables.x[3] - nmheVariables.xAC[3];
nmheWorkspace.DxAC[4] = nmheVariables.x[4] - nmheVariables.xAC[4];
nmheWorkspace.DxAC[5] = nmheVariables.x[5] - nmheVariables.xAC[5];
nmheWorkspace.DxAC[6] = nmheVariables.x[6] - nmheVariables.xAC[6];
nmheWorkspace.DxAC[7] = nmheVariables.x[7] - nmheVariables.xAC[7];
nmheWorkspace.g[0] += + nmheVariables.SAC[0]*nmheWorkspace.DxAC[0] + nmheVariables.SAC[1]*nmheWorkspace.DxAC[1] + nmheVariables.SAC[2]*nmheWorkspace.DxAC[2] + nmheVariables.SAC[3]*nmheWorkspace.DxAC[3] + nmheVariables.SAC[4]*nmheWorkspace.DxAC[4] + nmheVariables.SAC[5]*nmheWorkspace.DxAC[5] + nmheVariables.SAC[6]*nmheWorkspace.DxAC[6] + nmheVariables.SAC[7]*nmheWorkspace.DxAC[7];
nmheWorkspace.g[1] += + nmheVariables.SAC[8]*nmheWorkspace.DxAC[0] + nmheVariables.SAC[9]*nmheWorkspace.DxAC[1] + nmheVariables.SAC[10]*nmheWorkspace.DxAC[2] + nmheVariables.SAC[11]*nmheWorkspace.DxAC[3] + nmheVariables.SAC[12]*nmheWorkspace.DxAC[4] + nmheVariables.SAC[13]*nmheWorkspace.DxAC[5] + nmheVariables.SAC[14]*nmheWorkspace.DxAC[6] + nmheVariables.SAC[15]*nmheWorkspace.DxAC[7];
nmheWorkspace.g[2] += + nmheVariables.SAC[16]*nmheWorkspace.DxAC[0] + nmheVariables.SAC[17]*nmheWorkspace.DxAC[1] + nmheVariables.SAC[18]*nmheWorkspace.DxAC[2] + nmheVariables.SAC[19]*nmheWorkspace.DxAC[3] + nmheVariables.SAC[20]*nmheWorkspace.DxAC[4] + nmheVariables.SAC[21]*nmheWorkspace.DxAC[5] + nmheVariables.SAC[22]*nmheWorkspace.DxAC[6] + nmheVariables.SAC[23]*nmheWorkspace.DxAC[7];
nmheWorkspace.g[3] += + nmheVariables.SAC[24]*nmheWorkspace.DxAC[0] + nmheVariables.SAC[25]*nmheWorkspace.DxAC[1] + nmheVariables.SAC[26]*nmheWorkspace.DxAC[2] + nmheVariables.SAC[27]*nmheWorkspace.DxAC[3] + nmheVariables.SAC[28]*nmheWorkspace.DxAC[4] + nmheVariables.SAC[29]*nmheWorkspace.DxAC[5] + nmheVariables.SAC[30]*nmheWorkspace.DxAC[6] + nmheVariables.SAC[31]*nmheWorkspace.DxAC[7];
nmheWorkspace.g[4] += + nmheVariables.SAC[32]*nmheWorkspace.DxAC[0] + nmheVariables.SAC[33]*nmheWorkspace.DxAC[1] + nmheVariables.SAC[34]*nmheWorkspace.DxAC[2] + nmheVariables.SAC[35]*nmheWorkspace.DxAC[3] + nmheVariables.SAC[36]*nmheWorkspace.DxAC[4] + nmheVariables.SAC[37]*nmheWorkspace.DxAC[5] + nmheVariables.SAC[38]*nmheWorkspace.DxAC[6] + nmheVariables.SAC[39]*nmheWorkspace.DxAC[7];
nmheWorkspace.g[5] += + nmheVariables.SAC[40]*nmheWorkspace.DxAC[0] + nmheVariables.SAC[41]*nmheWorkspace.DxAC[1] + nmheVariables.SAC[42]*nmheWorkspace.DxAC[2] + nmheVariables.SAC[43]*nmheWorkspace.DxAC[3] + nmheVariables.SAC[44]*nmheWorkspace.DxAC[4] + nmheVariables.SAC[45]*nmheWorkspace.DxAC[5] + nmheVariables.SAC[46]*nmheWorkspace.DxAC[6] + nmheVariables.SAC[47]*nmheWorkspace.DxAC[7];
nmheWorkspace.g[6] += + nmheVariables.SAC[48]*nmheWorkspace.DxAC[0] + nmheVariables.SAC[49]*nmheWorkspace.DxAC[1] + nmheVariables.SAC[50]*nmheWorkspace.DxAC[2] + nmheVariables.SAC[51]*nmheWorkspace.DxAC[3] + nmheVariables.SAC[52]*nmheWorkspace.DxAC[4] + nmheVariables.SAC[53]*nmheWorkspace.DxAC[5] + nmheVariables.SAC[54]*nmheWorkspace.DxAC[6] + nmheVariables.SAC[55]*nmheWorkspace.DxAC[7];
nmheWorkspace.g[7] += + nmheVariables.SAC[56]*nmheWorkspace.DxAC[0] + nmheVariables.SAC[57]*nmheWorkspace.DxAC[1] + nmheVariables.SAC[58]*nmheWorkspace.DxAC[2] + nmheVariables.SAC[59]*nmheWorkspace.DxAC[3] + nmheVariables.SAC[60]*nmheWorkspace.DxAC[4] + nmheVariables.SAC[61]*nmheWorkspace.DxAC[5] + nmheVariables.SAC[62]*nmheWorkspace.DxAC[6] + nmheVariables.SAC[63]*nmheWorkspace.DxAC[7];

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 100; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmhe_multEQDy( &(nmheWorkspace.E[ lRun3 * 16 ]), &(nmheWorkspace.QDy[ lRun2 * 8 + 8 ]), &(nmheWorkspace.g[ lRun1 * 2 + 8 ]) );
}
}

tmp = nmheVariables.x[14] + nmheWorkspace.d[6];
nmheWorkspace.lbA[0] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[0] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[15] + nmheWorkspace.d[7];
nmheWorkspace.lbA[1] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[1] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[22] + nmheWorkspace.d[14];
nmheWorkspace.lbA[2] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[2] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[23] + nmheWorkspace.d[15];
nmheWorkspace.lbA[3] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[3] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[30] + nmheWorkspace.d[22];
nmheWorkspace.lbA[4] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[4] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[31] + nmheWorkspace.d[23];
nmheWorkspace.lbA[5] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[5] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[38] + nmheWorkspace.d[30];
nmheWorkspace.lbA[6] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[6] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[39] + nmheWorkspace.d[31];
nmheWorkspace.lbA[7] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[7] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[46] + nmheWorkspace.d[38];
nmheWorkspace.lbA[8] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[8] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[47] + nmheWorkspace.d[39];
nmheWorkspace.lbA[9] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[9] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[54] + nmheWorkspace.d[46];
nmheWorkspace.lbA[10] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[10] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[55] + nmheWorkspace.d[47];
nmheWorkspace.lbA[11] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[11] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[62] + nmheWorkspace.d[54];
nmheWorkspace.lbA[12] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[12] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[63] + nmheWorkspace.d[55];
nmheWorkspace.lbA[13] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[13] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[70] + nmheWorkspace.d[62];
nmheWorkspace.lbA[14] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[14] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[71] + nmheWorkspace.d[63];
nmheWorkspace.lbA[15] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[15] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[78] + nmheWorkspace.d[70];
nmheWorkspace.lbA[16] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[16] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[79] + nmheWorkspace.d[71];
nmheWorkspace.lbA[17] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[17] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[86] + nmheWorkspace.d[78];
nmheWorkspace.lbA[18] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[18] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[87] + nmheWorkspace.d[79];
nmheWorkspace.lbA[19] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[19] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[94] + nmheWorkspace.d[86];
nmheWorkspace.lbA[20] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[20] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[95] + nmheWorkspace.d[87];
nmheWorkspace.lbA[21] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[21] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[102] + nmheWorkspace.d[94];
nmheWorkspace.lbA[22] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[22] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[103] + nmheWorkspace.d[95];
nmheWorkspace.lbA[23] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[23] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[110] + nmheWorkspace.d[102];
nmheWorkspace.lbA[24] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[24] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[111] + nmheWorkspace.d[103];
nmheWorkspace.lbA[25] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[25] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[118] + nmheWorkspace.d[110];
nmheWorkspace.lbA[26] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[26] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[119] + nmheWorkspace.d[111];
nmheWorkspace.lbA[27] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[27] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[126] + nmheWorkspace.d[118];
nmheWorkspace.lbA[28] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[28] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[127] + nmheWorkspace.d[119];
nmheWorkspace.lbA[29] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[29] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[134] + nmheWorkspace.d[126];
nmheWorkspace.lbA[30] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[30] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[135] + nmheWorkspace.d[127];
nmheWorkspace.lbA[31] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[31] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[142] + nmheWorkspace.d[134];
nmheWorkspace.lbA[32] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[32] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[143] + nmheWorkspace.d[135];
nmheWorkspace.lbA[33] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[33] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[150] + nmheWorkspace.d[142];
nmheWorkspace.lbA[34] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[34] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[151] + nmheWorkspace.d[143];
nmheWorkspace.lbA[35] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[35] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[158] + nmheWorkspace.d[150];
nmheWorkspace.lbA[36] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[36] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[159] + nmheWorkspace.d[151];
nmheWorkspace.lbA[37] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[37] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[166] + nmheWorkspace.d[158];
nmheWorkspace.lbA[38] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[38] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[167] + nmheWorkspace.d[159];
nmheWorkspace.lbA[39] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[39] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[174] + nmheWorkspace.d[166];
nmheWorkspace.lbA[40] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[40] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[175] + nmheWorkspace.d[167];
nmheWorkspace.lbA[41] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[41] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[182] + nmheWorkspace.d[174];
nmheWorkspace.lbA[42] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[42] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[183] + nmheWorkspace.d[175];
nmheWorkspace.lbA[43] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[43] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[190] + nmheWorkspace.d[182];
nmheWorkspace.lbA[44] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[44] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[191] + nmheWorkspace.d[183];
nmheWorkspace.lbA[45] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[45] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[198] + nmheWorkspace.d[190];
nmheWorkspace.lbA[46] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[46] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[199] + nmheWorkspace.d[191];
nmheWorkspace.lbA[47] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[47] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[206] + nmheWorkspace.d[198];
nmheWorkspace.lbA[48] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[48] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[207] + nmheWorkspace.d[199];
nmheWorkspace.lbA[49] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[49] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[214] + nmheWorkspace.d[206];
nmheWorkspace.lbA[50] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[50] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[215] + nmheWorkspace.d[207];
nmheWorkspace.lbA[51] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[51] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[222] + nmheWorkspace.d[214];
nmheWorkspace.lbA[52] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[52] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[223] + nmheWorkspace.d[215];
nmheWorkspace.lbA[53] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[53] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[230] + nmheWorkspace.d[222];
nmheWorkspace.lbA[54] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[54] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[231] + nmheWorkspace.d[223];
nmheWorkspace.lbA[55] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[55] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[238] + nmheWorkspace.d[230];
nmheWorkspace.lbA[56] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[56] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[239] + nmheWorkspace.d[231];
nmheWorkspace.lbA[57] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[57] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[246] + nmheWorkspace.d[238];
nmheWorkspace.lbA[58] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[58] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[247] + nmheWorkspace.d[239];
nmheWorkspace.lbA[59] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[59] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[254] + nmheWorkspace.d[246];
nmheWorkspace.lbA[60] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[60] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[255] + nmheWorkspace.d[247];
nmheWorkspace.lbA[61] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[61] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[262] + nmheWorkspace.d[254];
nmheWorkspace.lbA[62] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[62] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[263] + nmheWorkspace.d[255];
nmheWorkspace.lbA[63] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[63] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[270] + nmheWorkspace.d[262];
nmheWorkspace.lbA[64] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[64] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[271] + nmheWorkspace.d[263];
nmheWorkspace.lbA[65] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[65] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[278] + nmheWorkspace.d[270];
nmheWorkspace.lbA[66] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[66] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[279] + nmheWorkspace.d[271];
nmheWorkspace.lbA[67] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[67] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[286] + nmheWorkspace.d[278];
nmheWorkspace.lbA[68] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[68] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[287] + nmheWorkspace.d[279];
nmheWorkspace.lbA[69] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[69] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[294] + nmheWorkspace.d[286];
nmheWorkspace.lbA[70] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[70] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[295] + nmheWorkspace.d[287];
nmheWorkspace.lbA[71] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[71] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[302] + nmheWorkspace.d[294];
nmheWorkspace.lbA[72] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[72] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[303] + nmheWorkspace.d[295];
nmheWorkspace.lbA[73] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[73] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[310] + nmheWorkspace.d[302];
nmheWorkspace.lbA[74] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[74] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[311] + nmheWorkspace.d[303];
nmheWorkspace.lbA[75] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[75] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[318] + nmheWorkspace.d[310];
nmheWorkspace.lbA[76] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[76] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[319] + nmheWorkspace.d[311];
nmheWorkspace.lbA[77] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[77] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[326] + nmheWorkspace.d[318];
nmheWorkspace.lbA[78] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[78] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[327] + nmheWorkspace.d[319];
nmheWorkspace.lbA[79] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[79] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[334] + nmheWorkspace.d[326];
nmheWorkspace.lbA[80] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[80] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[335] + nmheWorkspace.d[327];
nmheWorkspace.lbA[81] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[81] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[342] + nmheWorkspace.d[334];
nmheWorkspace.lbA[82] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[82] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[343] + nmheWorkspace.d[335];
nmheWorkspace.lbA[83] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[83] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[350] + nmheWorkspace.d[342];
nmheWorkspace.lbA[84] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[84] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[351] + nmheWorkspace.d[343];
nmheWorkspace.lbA[85] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[85] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[358] + nmheWorkspace.d[350];
nmheWorkspace.lbA[86] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[86] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[359] + nmheWorkspace.d[351];
nmheWorkspace.lbA[87] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[87] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[366] + nmheWorkspace.d[358];
nmheWorkspace.lbA[88] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[88] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[367] + nmheWorkspace.d[359];
nmheWorkspace.lbA[89] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[89] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[374] + nmheWorkspace.d[366];
nmheWorkspace.lbA[90] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[90] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[375] + nmheWorkspace.d[367];
nmheWorkspace.lbA[91] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[91] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[382] + nmheWorkspace.d[374];
nmheWorkspace.lbA[92] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[92] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[383] + nmheWorkspace.d[375];
nmheWorkspace.lbA[93] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[93] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[390] + nmheWorkspace.d[382];
nmheWorkspace.lbA[94] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[94] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[391] + nmheWorkspace.d[383];
nmheWorkspace.lbA[95] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[95] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[398] + nmheWorkspace.d[390];
nmheWorkspace.lbA[96] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[96] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[399] + nmheWorkspace.d[391];
nmheWorkspace.lbA[97] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[97] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[406] + nmheWorkspace.d[398];
nmheWorkspace.lbA[98] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[98] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[407] + nmheWorkspace.d[399];
nmheWorkspace.lbA[99] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[99] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[414] + nmheWorkspace.d[406];
nmheWorkspace.lbA[100] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[100] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[415] + nmheWorkspace.d[407];
nmheWorkspace.lbA[101] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[101] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[422] + nmheWorkspace.d[414];
nmheWorkspace.lbA[102] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[102] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[423] + nmheWorkspace.d[415];
nmheWorkspace.lbA[103] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[103] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[430] + nmheWorkspace.d[422];
nmheWorkspace.lbA[104] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[104] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[431] + nmheWorkspace.d[423];
nmheWorkspace.lbA[105] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[105] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[438] + nmheWorkspace.d[430];
nmheWorkspace.lbA[106] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[106] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[439] + nmheWorkspace.d[431];
nmheWorkspace.lbA[107] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[107] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[446] + nmheWorkspace.d[438];
nmheWorkspace.lbA[108] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[108] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[447] + nmheWorkspace.d[439];
nmheWorkspace.lbA[109] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[109] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[454] + nmheWorkspace.d[446];
nmheWorkspace.lbA[110] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[110] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[455] + nmheWorkspace.d[447];
nmheWorkspace.lbA[111] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[111] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[462] + nmheWorkspace.d[454];
nmheWorkspace.lbA[112] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[112] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[463] + nmheWorkspace.d[455];
nmheWorkspace.lbA[113] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[113] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[470] + nmheWorkspace.d[462];
nmheWorkspace.lbA[114] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[114] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[471] + nmheWorkspace.d[463];
nmheWorkspace.lbA[115] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[115] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[478] + nmheWorkspace.d[470];
nmheWorkspace.lbA[116] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[116] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[479] + nmheWorkspace.d[471];
nmheWorkspace.lbA[117] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[117] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[486] + nmheWorkspace.d[478];
nmheWorkspace.lbA[118] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[118] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[487] + nmheWorkspace.d[479];
nmheWorkspace.lbA[119] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[119] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[494] + nmheWorkspace.d[486];
nmheWorkspace.lbA[120] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[120] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[495] + nmheWorkspace.d[487];
nmheWorkspace.lbA[121] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[121] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[502] + nmheWorkspace.d[494];
nmheWorkspace.lbA[122] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[122] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[503] + nmheWorkspace.d[495];
nmheWorkspace.lbA[123] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[123] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[510] + nmheWorkspace.d[502];
nmheWorkspace.lbA[124] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[124] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[511] + nmheWorkspace.d[503];
nmheWorkspace.lbA[125] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[125] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[518] + nmheWorkspace.d[510];
nmheWorkspace.lbA[126] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[126] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[519] + nmheWorkspace.d[511];
nmheWorkspace.lbA[127] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[127] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[526] + nmheWorkspace.d[518];
nmheWorkspace.lbA[128] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[128] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[527] + nmheWorkspace.d[519];
nmheWorkspace.lbA[129] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[129] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[534] + nmheWorkspace.d[526];
nmheWorkspace.lbA[130] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[130] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[535] + nmheWorkspace.d[527];
nmheWorkspace.lbA[131] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[131] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[542] + nmheWorkspace.d[534];
nmheWorkspace.lbA[132] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[132] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[543] + nmheWorkspace.d[535];
nmheWorkspace.lbA[133] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[133] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[550] + nmheWorkspace.d[542];
nmheWorkspace.lbA[134] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[134] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[551] + nmheWorkspace.d[543];
nmheWorkspace.lbA[135] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[135] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[558] + nmheWorkspace.d[550];
nmheWorkspace.lbA[136] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[136] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[559] + nmheWorkspace.d[551];
nmheWorkspace.lbA[137] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[137] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[566] + nmheWorkspace.d[558];
nmheWorkspace.lbA[138] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[138] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[567] + nmheWorkspace.d[559];
nmheWorkspace.lbA[139] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[139] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[574] + nmheWorkspace.d[566];
nmheWorkspace.lbA[140] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[140] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[575] + nmheWorkspace.d[567];
nmheWorkspace.lbA[141] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[141] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[582] + nmheWorkspace.d[574];
nmheWorkspace.lbA[142] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[142] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[583] + nmheWorkspace.d[575];
nmheWorkspace.lbA[143] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[143] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[590] + nmheWorkspace.d[582];
nmheWorkspace.lbA[144] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[144] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[591] + nmheWorkspace.d[583];
nmheWorkspace.lbA[145] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[145] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[598] + nmheWorkspace.d[590];
nmheWorkspace.lbA[146] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[146] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[599] + nmheWorkspace.d[591];
nmheWorkspace.lbA[147] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[147] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[606] + nmheWorkspace.d[598];
nmheWorkspace.lbA[148] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[148] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[607] + nmheWorkspace.d[599];
nmheWorkspace.lbA[149] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[149] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[614] + nmheWorkspace.d[606];
nmheWorkspace.lbA[150] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[150] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[615] + nmheWorkspace.d[607];
nmheWorkspace.lbA[151] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[151] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[622] + nmheWorkspace.d[614];
nmheWorkspace.lbA[152] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[152] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[623] + nmheWorkspace.d[615];
nmheWorkspace.lbA[153] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[153] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[630] + nmheWorkspace.d[622];
nmheWorkspace.lbA[154] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[154] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[631] + nmheWorkspace.d[623];
nmheWorkspace.lbA[155] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[155] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[638] + nmheWorkspace.d[630];
nmheWorkspace.lbA[156] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[156] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[639] + nmheWorkspace.d[631];
nmheWorkspace.lbA[157] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[157] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[646] + nmheWorkspace.d[638];
nmheWorkspace.lbA[158] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[158] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[647] + nmheWorkspace.d[639];
nmheWorkspace.lbA[159] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[159] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[654] + nmheWorkspace.d[646];
nmheWorkspace.lbA[160] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[160] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[655] + nmheWorkspace.d[647];
nmheWorkspace.lbA[161] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[161] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[662] + nmheWorkspace.d[654];
nmheWorkspace.lbA[162] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[162] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[663] + nmheWorkspace.d[655];
nmheWorkspace.lbA[163] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[163] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[670] + nmheWorkspace.d[662];
nmheWorkspace.lbA[164] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[164] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[671] + nmheWorkspace.d[663];
nmheWorkspace.lbA[165] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[165] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[678] + nmheWorkspace.d[670];
nmheWorkspace.lbA[166] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[166] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[679] + nmheWorkspace.d[671];
nmheWorkspace.lbA[167] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[167] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[686] + nmheWorkspace.d[678];
nmheWorkspace.lbA[168] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[168] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[687] + nmheWorkspace.d[679];
nmheWorkspace.lbA[169] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[169] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[694] + nmheWorkspace.d[686];
nmheWorkspace.lbA[170] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[170] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[695] + nmheWorkspace.d[687];
nmheWorkspace.lbA[171] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[171] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[702] + nmheWorkspace.d[694];
nmheWorkspace.lbA[172] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[172] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[703] + nmheWorkspace.d[695];
nmheWorkspace.lbA[173] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[173] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[710] + nmheWorkspace.d[702];
nmheWorkspace.lbA[174] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[174] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[711] + nmheWorkspace.d[703];
nmheWorkspace.lbA[175] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[175] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[718] + nmheWorkspace.d[710];
nmheWorkspace.lbA[176] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[176] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[719] + nmheWorkspace.d[711];
nmheWorkspace.lbA[177] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[177] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[726] + nmheWorkspace.d[718];
nmheWorkspace.lbA[178] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[178] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[727] + nmheWorkspace.d[719];
nmheWorkspace.lbA[179] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[179] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[734] + nmheWorkspace.d[726];
nmheWorkspace.lbA[180] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[180] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[735] + nmheWorkspace.d[727];
nmheWorkspace.lbA[181] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[181] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[742] + nmheWorkspace.d[734];
nmheWorkspace.lbA[182] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[182] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[743] + nmheWorkspace.d[735];
nmheWorkspace.lbA[183] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[183] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[750] + nmheWorkspace.d[742];
nmheWorkspace.lbA[184] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[184] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[751] + nmheWorkspace.d[743];
nmheWorkspace.lbA[185] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[185] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[758] + nmheWorkspace.d[750];
nmheWorkspace.lbA[186] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[186] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[759] + nmheWorkspace.d[751];
nmheWorkspace.lbA[187] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[187] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[766] + nmheWorkspace.d[758];
nmheWorkspace.lbA[188] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[188] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[767] + nmheWorkspace.d[759];
nmheWorkspace.lbA[189] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[189] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[774] + nmheWorkspace.d[766];
nmheWorkspace.lbA[190] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[190] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[775] + nmheWorkspace.d[767];
nmheWorkspace.lbA[191] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[191] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[782] + nmheWorkspace.d[774];
nmheWorkspace.lbA[192] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[192] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[783] + nmheWorkspace.d[775];
nmheWorkspace.lbA[193] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[193] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[790] + nmheWorkspace.d[782];
nmheWorkspace.lbA[194] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[194] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[791] + nmheWorkspace.d[783];
nmheWorkspace.lbA[195] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[195] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[798] + nmheWorkspace.d[790];
nmheWorkspace.lbA[196] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[196] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[799] + nmheWorkspace.d[791];
nmheWorkspace.lbA[197] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[197] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[806] + nmheWorkspace.d[798];
nmheWorkspace.lbA[198] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[198] = (real_t)1.0000000000000000e+00 - tmp;
tmp = nmheVariables.x[807] + nmheWorkspace.d[799];
nmheWorkspace.lbA[199] = (real_t)1.0000000000000001e-01 - tmp;
nmheWorkspace.ubA[199] = (real_t)1.0000000000000000e+00 - tmp;

}

void nmhe_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
nmheVariables.x[0] += nmheWorkspace.x[0];
nmheVariables.x[1] += nmheWorkspace.x[1];
nmheVariables.x[2] += nmheWorkspace.x[2];
nmheVariables.x[3] += nmheWorkspace.x[3];
nmheVariables.x[4] += nmheWorkspace.x[4];
nmheVariables.x[5] += nmheWorkspace.x[5];
nmheVariables.x[6] += nmheWorkspace.x[6];
nmheVariables.x[7] += nmheWorkspace.x[7];

for (lRun1 = 0; lRun1 < 200; ++lRun1)
nmheVariables.u[lRun1] += nmheWorkspace.x[lRun1 + 8];


for (lRun1 = 0; lRun1 < 800; ++lRun1)
{
for (lRun2 = 0; lRun2 < 1; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 8; ++lRun3)
{
t += + nmheWorkspace.evGx[(lRun1 * 8) + (lRun3)]*nmheWorkspace.x[(lRun3) + (lRun2)];
}
nmheVariables.x[(lRun1 + 8) + (lRun2)] += t + nmheWorkspace.d[(lRun1) + (lRun2)];
}
}

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmhe_multEDu( &(nmheWorkspace.E[ lRun3 * 16 ]), &(nmheWorkspace.x[ lRun2 * 2 + 8 ]), &(nmheVariables.x[ lRun1 * 8 + 8 ]) );
}
}
}

int nmhe_preparationStep(  )
{
int ret;

ret = nmhe_modelSimulation();
nmhe_evaluateObjective(  );
nmhe_condensePrep(  );
return ret;
}

int nmhe_feedbackStep(  )
{
int tmp;

nmhe_condenseFdb(  );

tmp = nmhe_solve( );

nmhe_expand(  );
return tmp;
}

int nmhe_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&nmheWorkspace, 0, sizeof( nmheWorkspace ));
nmheWorkspace.acHx[0] = 1.0000000000000000e+00;
nmheWorkspace.acHx[1] = 0.0000000000000000e+00;
nmheWorkspace.acHx[2] = 0.0000000000000000e+00;
nmheWorkspace.acHx[3] = 0.0000000000000000e+00;
nmheWorkspace.acHx[4] = 0.0000000000000000e+00;
nmheWorkspace.acHx[5] = 0.0000000000000000e+00;
nmheWorkspace.acHx[6] = 0.0000000000000000e+00;
nmheWorkspace.acHx[7] = 0.0000000000000000e+00;
nmheWorkspace.acHx[8] = 0.0000000000000000e+00;
nmheWorkspace.acHx[9] = 1.0000000000000000e+00;
nmheWorkspace.acHx[10] = 0.0000000000000000e+00;
nmheWorkspace.acHx[11] = 0.0000000000000000e+00;
nmheWorkspace.acHx[12] = 0.0000000000000000e+00;
nmheWorkspace.acHx[13] = 0.0000000000000000e+00;
nmheWorkspace.acHx[14] = 0.0000000000000000e+00;
nmheWorkspace.acHx[15] = 0.0000000000000000e+00;
nmheWorkspace.acHx[16] = 0.0000000000000000e+00;
nmheWorkspace.acHx[17] = 0.0000000000000000e+00;
nmheWorkspace.acHx[18] = 1.0000000000000000e+00;
nmheWorkspace.acHx[19] = 0.0000000000000000e+00;
nmheWorkspace.acHx[20] = 0.0000000000000000e+00;
nmheWorkspace.acHx[21] = 0.0000000000000000e+00;
nmheWorkspace.acHx[22] = 0.0000000000000000e+00;
nmheWorkspace.acHx[23] = 0.0000000000000000e+00;
nmheWorkspace.acHx[24] = 0.0000000000000000e+00;
nmheWorkspace.acHx[25] = 0.0000000000000000e+00;
nmheWorkspace.acHx[26] = 0.0000000000000000e+00;
nmheWorkspace.acHx[27] = 1.0000000000000000e+00;
nmheWorkspace.acHx[28] = 0.0000000000000000e+00;
nmheWorkspace.acHx[29] = 0.0000000000000000e+00;
nmheWorkspace.acHx[30] = 0.0000000000000000e+00;
nmheWorkspace.acHx[31] = 0.0000000000000000e+00;
nmheWorkspace.acHx[32] = 0.0000000000000000e+00;
nmheWorkspace.acHx[33] = 0.0000000000000000e+00;
nmheWorkspace.acHx[34] = 0.0000000000000000e+00;
nmheWorkspace.acHx[35] = 0.0000000000000000e+00;
nmheWorkspace.acHx[36] = 1.0000000000000000e+00;
nmheWorkspace.acHx[37] = 0.0000000000000000e+00;
nmheWorkspace.acHx[38] = 0.0000000000000000e+00;
nmheWorkspace.acHx[39] = 0.0000000000000000e+00;
nmheWorkspace.acHx[40] = 0.0000000000000000e+00;
nmheWorkspace.acHx[41] = 0.0000000000000000e+00;
nmheWorkspace.acHx[42] = 0.0000000000000000e+00;
nmheWorkspace.acHx[43] = 0.0000000000000000e+00;
nmheWorkspace.acHx[44] = 0.0000000000000000e+00;
nmheWorkspace.acHx[45] = 1.0000000000000000e+00;
nmheWorkspace.acHx[46] = 0.0000000000000000e+00;
nmheWorkspace.acHx[47] = 0.0000000000000000e+00;
nmheWorkspace.acHx[48] = 0.0000000000000000e+00;
nmheWorkspace.acHx[49] = 0.0000000000000000e+00;
nmheWorkspace.acHx[50] = 0.0000000000000000e+00;
nmheWorkspace.acHx[51] = 0.0000000000000000e+00;
nmheWorkspace.acHx[52] = 0.0000000000000000e+00;
nmheWorkspace.acHx[53] = 0.0000000000000000e+00;
nmheWorkspace.acHx[54] = 0.0000000000000000e+00;
nmheWorkspace.acHx[55] = 0.0000000000000000e+00;
nmheWorkspace.acHx[56] = 0.0000000000000000e+00;
nmheWorkspace.acHx[57] = 0.0000000000000000e+00;
nmheWorkspace.acHx[58] = 0.0000000000000000e+00;
nmheWorkspace.acHx[59] = 0.0000000000000000e+00;
nmheWorkspace.acHx[60] = 0.0000000000000000e+00;
nmheWorkspace.acHx[61] = 0.0000000000000000e+00;
nmheWorkspace.acHx[62] = 0.0000000000000000e+00;
nmheWorkspace.acHx[63] = 0.0000000000000000e+00;
nmheWorkspace.acHu[0] = 0.0000000000000000e+00;
nmheWorkspace.acHu[1] = 0.0000000000000000e+00;
nmheWorkspace.acHu[2] = 0.0000000000000000e+00;
nmheWorkspace.acHu[3] = 0.0000000000000000e+00;
nmheWorkspace.acHu[4] = 0.0000000000000000e+00;
nmheWorkspace.acHu[5] = 0.0000000000000000e+00;
nmheWorkspace.acHu[6] = 0.0000000000000000e+00;
nmheWorkspace.acHu[7] = 0.0000000000000000e+00;
nmheWorkspace.acHu[8] = 0.0000000000000000e+00;
nmheWorkspace.acHu[9] = 0.0000000000000000e+00;
nmheWorkspace.acHu[10] = 0.0000000000000000e+00;
nmheWorkspace.acHu[11] = 0.0000000000000000e+00;
nmheWorkspace.acHu[12] = 1.0000000000000000e+00;
nmheWorkspace.acHu[13] = 0.0000000000000000e+00;
nmheWorkspace.acHu[14] = 0.0000000000000000e+00;
nmheWorkspace.acHu[15] = 1.0000000000000000e+00;
return ret;
}

void nmhe_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 100; ++index)
{
nmheWorkspace.state[0] = nmheVariables.x[index * 8];
nmheWorkspace.state[1] = nmheVariables.x[index * 8 + 1];
nmheWorkspace.state[2] = nmheVariables.x[index * 8 + 2];
nmheWorkspace.state[3] = nmheVariables.x[index * 8 + 3];
nmheWorkspace.state[4] = nmheVariables.x[index * 8 + 4];
nmheWorkspace.state[5] = nmheVariables.x[index * 8 + 5];
nmheWorkspace.state[6] = nmheVariables.x[index * 8 + 6];
nmheWorkspace.state[7] = nmheVariables.x[index * 8 + 7];
nmheWorkspace.state[88] = nmheVariables.u[index * 2];
nmheWorkspace.state[89] = nmheVariables.u[index * 2 + 1];

nmhe_integrate(nmheWorkspace.state, index == 0);

nmheVariables.x[index * 8 + 8] = nmheWorkspace.state[0];
nmheVariables.x[index * 8 + 9] = nmheWorkspace.state[1];
nmheVariables.x[index * 8 + 10] = nmheWorkspace.state[2];
nmheVariables.x[index * 8 + 11] = nmheWorkspace.state[3];
nmheVariables.x[index * 8 + 12] = nmheWorkspace.state[4];
nmheVariables.x[index * 8 + 13] = nmheWorkspace.state[5];
nmheVariables.x[index * 8 + 14] = nmheWorkspace.state[6];
nmheVariables.x[index * 8 + 15] = nmheWorkspace.state[7];
}
}

void nmhe_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 100; ++index)
{
nmheVariables.x[index * 8] = nmheVariables.x[index * 8 + 8];
nmheVariables.x[index * 8 + 1] = nmheVariables.x[index * 8 + 9];
nmheVariables.x[index * 8 + 2] = nmheVariables.x[index * 8 + 10];
nmheVariables.x[index * 8 + 3] = nmheVariables.x[index * 8 + 11];
nmheVariables.x[index * 8 + 4] = nmheVariables.x[index * 8 + 12];
nmheVariables.x[index * 8 + 5] = nmheVariables.x[index * 8 + 13];
nmheVariables.x[index * 8 + 6] = nmheVariables.x[index * 8 + 14];
nmheVariables.x[index * 8 + 7] = nmheVariables.x[index * 8 + 15];
}

if (strategy == 1 && xEnd != 0)
{
nmheVariables.x[800] = xEnd[0];
nmheVariables.x[801] = xEnd[1];
nmheVariables.x[802] = xEnd[2];
nmheVariables.x[803] = xEnd[3];
nmheVariables.x[804] = xEnd[4];
nmheVariables.x[805] = xEnd[5];
nmheVariables.x[806] = xEnd[6];
nmheVariables.x[807] = xEnd[7];
}
else if (strategy == 2) 
{
nmheWorkspace.state[0] = nmheVariables.x[800];
nmheWorkspace.state[1] = nmheVariables.x[801];
nmheWorkspace.state[2] = nmheVariables.x[802];
nmheWorkspace.state[3] = nmheVariables.x[803];
nmheWorkspace.state[4] = nmheVariables.x[804];
nmheWorkspace.state[5] = nmheVariables.x[805];
nmheWorkspace.state[6] = nmheVariables.x[806];
nmheWorkspace.state[7] = nmheVariables.x[807];
if (uEnd != 0)
{
nmheWorkspace.state[88] = uEnd[0];
nmheWorkspace.state[89] = uEnd[1];
}
else
{
nmheWorkspace.state[88] = nmheVariables.u[198];
nmheWorkspace.state[89] = nmheVariables.u[199];
}

nmhe_integrate(nmheWorkspace.state, 1);

nmheVariables.x[800] = nmheWorkspace.state[0];
nmheVariables.x[801] = nmheWorkspace.state[1];
nmheVariables.x[802] = nmheWorkspace.state[2];
nmheVariables.x[803] = nmheWorkspace.state[3];
nmheVariables.x[804] = nmheWorkspace.state[4];
nmheVariables.x[805] = nmheWorkspace.state[5];
nmheVariables.x[806] = nmheWorkspace.state[6];
nmheVariables.x[807] = nmheWorkspace.state[7];
}
}

void nmhe_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 99; ++index)
{
nmheVariables.u[index * 2] = nmheVariables.u[index * 2 + 2];
nmheVariables.u[index * 2 + 1] = nmheVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
nmheVariables.u[198] = uEnd[0];
nmheVariables.u[199] = uEnd[1];
}
}

real_t nmhe_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmheWorkspace.g[0]*nmheWorkspace.x[0] + nmheWorkspace.g[1]*nmheWorkspace.x[1] + nmheWorkspace.g[2]*nmheWorkspace.x[2] + nmheWorkspace.g[3]*nmheWorkspace.x[3] + nmheWorkspace.g[4]*nmheWorkspace.x[4] + nmheWorkspace.g[5]*nmheWorkspace.x[5] + nmheWorkspace.g[6]*nmheWorkspace.x[6] + nmheWorkspace.g[7]*nmheWorkspace.x[7] + nmheWorkspace.g[8]*nmheWorkspace.x[8] + nmheWorkspace.g[9]*nmheWorkspace.x[9] + nmheWorkspace.g[10]*nmheWorkspace.x[10] + nmheWorkspace.g[11]*nmheWorkspace.x[11] + nmheWorkspace.g[12]*nmheWorkspace.x[12] + nmheWorkspace.g[13]*nmheWorkspace.x[13] + nmheWorkspace.g[14]*nmheWorkspace.x[14] + nmheWorkspace.g[15]*nmheWorkspace.x[15] + nmheWorkspace.g[16]*nmheWorkspace.x[16] + nmheWorkspace.g[17]*nmheWorkspace.x[17] + nmheWorkspace.g[18]*nmheWorkspace.x[18] + nmheWorkspace.g[19]*nmheWorkspace.x[19] + nmheWorkspace.g[20]*nmheWorkspace.x[20] + nmheWorkspace.g[21]*nmheWorkspace.x[21] + nmheWorkspace.g[22]*nmheWorkspace.x[22] + nmheWorkspace.g[23]*nmheWorkspace.x[23] + nmheWorkspace.g[24]*nmheWorkspace.x[24] + nmheWorkspace.g[25]*nmheWorkspace.x[25] + nmheWorkspace.g[26]*nmheWorkspace.x[26] + nmheWorkspace.g[27]*nmheWorkspace.x[27] + nmheWorkspace.g[28]*nmheWorkspace.x[28] + nmheWorkspace.g[29]*nmheWorkspace.x[29] + nmheWorkspace.g[30]*nmheWorkspace.x[30] + nmheWorkspace.g[31]*nmheWorkspace.x[31] + nmheWorkspace.g[32]*nmheWorkspace.x[32] + nmheWorkspace.g[33]*nmheWorkspace.x[33] + nmheWorkspace.g[34]*nmheWorkspace.x[34] + nmheWorkspace.g[35]*nmheWorkspace.x[35] + nmheWorkspace.g[36]*nmheWorkspace.x[36] + nmheWorkspace.g[37]*nmheWorkspace.x[37] + nmheWorkspace.g[38]*nmheWorkspace.x[38] + nmheWorkspace.g[39]*nmheWorkspace.x[39] + nmheWorkspace.g[40]*nmheWorkspace.x[40] + nmheWorkspace.g[41]*nmheWorkspace.x[41] + nmheWorkspace.g[42]*nmheWorkspace.x[42] + nmheWorkspace.g[43]*nmheWorkspace.x[43] + nmheWorkspace.g[44]*nmheWorkspace.x[44] + nmheWorkspace.g[45]*nmheWorkspace.x[45] + nmheWorkspace.g[46]*nmheWorkspace.x[46] + nmheWorkspace.g[47]*nmheWorkspace.x[47] + nmheWorkspace.g[48]*nmheWorkspace.x[48] + nmheWorkspace.g[49]*nmheWorkspace.x[49] + nmheWorkspace.g[50]*nmheWorkspace.x[50] + nmheWorkspace.g[51]*nmheWorkspace.x[51] + nmheWorkspace.g[52]*nmheWorkspace.x[52] + nmheWorkspace.g[53]*nmheWorkspace.x[53] + nmheWorkspace.g[54]*nmheWorkspace.x[54] + nmheWorkspace.g[55]*nmheWorkspace.x[55] + nmheWorkspace.g[56]*nmheWorkspace.x[56] + nmheWorkspace.g[57]*nmheWorkspace.x[57] + nmheWorkspace.g[58]*nmheWorkspace.x[58] + nmheWorkspace.g[59]*nmheWorkspace.x[59] + nmheWorkspace.g[60]*nmheWorkspace.x[60] + nmheWorkspace.g[61]*nmheWorkspace.x[61] + nmheWorkspace.g[62]*nmheWorkspace.x[62] + nmheWorkspace.g[63]*nmheWorkspace.x[63] + nmheWorkspace.g[64]*nmheWorkspace.x[64] + nmheWorkspace.g[65]*nmheWorkspace.x[65] + nmheWorkspace.g[66]*nmheWorkspace.x[66] + nmheWorkspace.g[67]*nmheWorkspace.x[67] + nmheWorkspace.g[68]*nmheWorkspace.x[68] + nmheWorkspace.g[69]*nmheWorkspace.x[69] + nmheWorkspace.g[70]*nmheWorkspace.x[70] + nmheWorkspace.g[71]*nmheWorkspace.x[71] + nmheWorkspace.g[72]*nmheWorkspace.x[72] + nmheWorkspace.g[73]*nmheWorkspace.x[73] + nmheWorkspace.g[74]*nmheWorkspace.x[74] + nmheWorkspace.g[75]*nmheWorkspace.x[75] + nmheWorkspace.g[76]*nmheWorkspace.x[76] + nmheWorkspace.g[77]*nmheWorkspace.x[77] + nmheWorkspace.g[78]*nmheWorkspace.x[78] + nmheWorkspace.g[79]*nmheWorkspace.x[79] + nmheWorkspace.g[80]*nmheWorkspace.x[80] + nmheWorkspace.g[81]*nmheWorkspace.x[81] + nmheWorkspace.g[82]*nmheWorkspace.x[82] + nmheWorkspace.g[83]*nmheWorkspace.x[83] + nmheWorkspace.g[84]*nmheWorkspace.x[84] + nmheWorkspace.g[85]*nmheWorkspace.x[85] + nmheWorkspace.g[86]*nmheWorkspace.x[86] + nmheWorkspace.g[87]*nmheWorkspace.x[87] + nmheWorkspace.g[88]*nmheWorkspace.x[88] + nmheWorkspace.g[89]*nmheWorkspace.x[89] + nmheWorkspace.g[90]*nmheWorkspace.x[90] + nmheWorkspace.g[91]*nmheWorkspace.x[91] + nmheWorkspace.g[92]*nmheWorkspace.x[92] + nmheWorkspace.g[93]*nmheWorkspace.x[93] + nmheWorkspace.g[94]*nmheWorkspace.x[94] + nmheWorkspace.g[95]*nmheWorkspace.x[95] + nmheWorkspace.g[96]*nmheWorkspace.x[96] + nmheWorkspace.g[97]*nmheWorkspace.x[97] + nmheWorkspace.g[98]*nmheWorkspace.x[98] + nmheWorkspace.g[99]*nmheWorkspace.x[99] + nmheWorkspace.g[100]*nmheWorkspace.x[100] + nmheWorkspace.g[101]*nmheWorkspace.x[101] + nmheWorkspace.g[102]*nmheWorkspace.x[102] + nmheWorkspace.g[103]*nmheWorkspace.x[103] + nmheWorkspace.g[104]*nmheWorkspace.x[104] + nmheWorkspace.g[105]*nmheWorkspace.x[105] + nmheWorkspace.g[106]*nmheWorkspace.x[106] + nmheWorkspace.g[107]*nmheWorkspace.x[107] + nmheWorkspace.g[108]*nmheWorkspace.x[108] + nmheWorkspace.g[109]*nmheWorkspace.x[109] + nmheWorkspace.g[110]*nmheWorkspace.x[110] + nmheWorkspace.g[111]*nmheWorkspace.x[111] + nmheWorkspace.g[112]*nmheWorkspace.x[112] + nmheWorkspace.g[113]*nmheWorkspace.x[113] + nmheWorkspace.g[114]*nmheWorkspace.x[114] + nmheWorkspace.g[115]*nmheWorkspace.x[115] + nmheWorkspace.g[116]*nmheWorkspace.x[116] + nmheWorkspace.g[117]*nmheWorkspace.x[117] + nmheWorkspace.g[118]*nmheWorkspace.x[118] + nmheWorkspace.g[119]*nmheWorkspace.x[119] + nmheWorkspace.g[120]*nmheWorkspace.x[120] + nmheWorkspace.g[121]*nmheWorkspace.x[121] + nmheWorkspace.g[122]*nmheWorkspace.x[122] + nmheWorkspace.g[123]*nmheWorkspace.x[123] + nmheWorkspace.g[124]*nmheWorkspace.x[124] + nmheWorkspace.g[125]*nmheWorkspace.x[125] + nmheWorkspace.g[126]*nmheWorkspace.x[126] + nmheWorkspace.g[127]*nmheWorkspace.x[127] + nmheWorkspace.g[128]*nmheWorkspace.x[128] + nmheWorkspace.g[129]*nmheWorkspace.x[129] + nmheWorkspace.g[130]*nmheWorkspace.x[130] + nmheWorkspace.g[131]*nmheWorkspace.x[131] + nmheWorkspace.g[132]*nmheWorkspace.x[132] + nmheWorkspace.g[133]*nmheWorkspace.x[133] + nmheWorkspace.g[134]*nmheWorkspace.x[134] + nmheWorkspace.g[135]*nmheWorkspace.x[135] + nmheWorkspace.g[136]*nmheWorkspace.x[136] + nmheWorkspace.g[137]*nmheWorkspace.x[137] + nmheWorkspace.g[138]*nmheWorkspace.x[138] + nmheWorkspace.g[139]*nmheWorkspace.x[139] + nmheWorkspace.g[140]*nmheWorkspace.x[140] + nmheWorkspace.g[141]*nmheWorkspace.x[141] + nmheWorkspace.g[142]*nmheWorkspace.x[142] + nmheWorkspace.g[143]*nmheWorkspace.x[143] + nmheWorkspace.g[144]*nmheWorkspace.x[144] + nmheWorkspace.g[145]*nmheWorkspace.x[145] + nmheWorkspace.g[146]*nmheWorkspace.x[146] + nmheWorkspace.g[147]*nmheWorkspace.x[147] + nmheWorkspace.g[148]*nmheWorkspace.x[148] + nmheWorkspace.g[149]*nmheWorkspace.x[149] + nmheWorkspace.g[150]*nmheWorkspace.x[150] + nmheWorkspace.g[151]*nmheWorkspace.x[151] + nmheWorkspace.g[152]*nmheWorkspace.x[152] + nmheWorkspace.g[153]*nmheWorkspace.x[153] + nmheWorkspace.g[154]*nmheWorkspace.x[154] + nmheWorkspace.g[155]*nmheWorkspace.x[155] + nmheWorkspace.g[156]*nmheWorkspace.x[156] + nmheWorkspace.g[157]*nmheWorkspace.x[157] + nmheWorkspace.g[158]*nmheWorkspace.x[158] + nmheWorkspace.g[159]*nmheWorkspace.x[159] + nmheWorkspace.g[160]*nmheWorkspace.x[160] + nmheWorkspace.g[161]*nmheWorkspace.x[161] + nmheWorkspace.g[162]*nmheWorkspace.x[162] + nmheWorkspace.g[163]*nmheWorkspace.x[163] + nmheWorkspace.g[164]*nmheWorkspace.x[164] + nmheWorkspace.g[165]*nmheWorkspace.x[165] + nmheWorkspace.g[166]*nmheWorkspace.x[166] + nmheWorkspace.g[167]*nmheWorkspace.x[167] + nmheWorkspace.g[168]*nmheWorkspace.x[168] + nmheWorkspace.g[169]*nmheWorkspace.x[169] + nmheWorkspace.g[170]*nmheWorkspace.x[170] + nmheWorkspace.g[171]*nmheWorkspace.x[171] + nmheWorkspace.g[172]*nmheWorkspace.x[172] + nmheWorkspace.g[173]*nmheWorkspace.x[173] + nmheWorkspace.g[174]*nmheWorkspace.x[174] + nmheWorkspace.g[175]*nmheWorkspace.x[175] + nmheWorkspace.g[176]*nmheWorkspace.x[176] + nmheWorkspace.g[177]*nmheWorkspace.x[177] + nmheWorkspace.g[178]*nmheWorkspace.x[178] + nmheWorkspace.g[179]*nmheWorkspace.x[179] + nmheWorkspace.g[180]*nmheWorkspace.x[180] + nmheWorkspace.g[181]*nmheWorkspace.x[181] + nmheWorkspace.g[182]*nmheWorkspace.x[182] + nmheWorkspace.g[183]*nmheWorkspace.x[183] + nmheWorkspace.g[184]*nmheWorkspace.x[184] + nmheWorkspace.g[185]*nmheWorkspace.x[185] + nmheWorkspace.g[186]*nmheWorkspace.x[186] + nmheWorkspace.g[187]*nmheWorkspace.x[187] + nmheWorkspace.g[188]*nmheWorkspace.x[188] + nmheWorkspace.g[189]*nmheWorkspace.x[189] + nmheWorkspace.g[190]*nmheWorkspace.x[190] + nmheWorkspace.g[191]*nmheWorkspace.x[191] + nmheWorkspace.g[192]*nmheWorkspace.x[192] + nmheWorkspace.g[193]*nmheWorkspace.x[193] + nmheWorkspace.g[194]*nmheWorkspace.x[194] + nmheWorkspace.g[195]*nmheWorkspace.x[195] + nmheWorkspace.g[196]*nmheWorkspace.x[196] + nmheWorkspace.g[197]*nmheWorkspace.x[197] + nmheWorkspace.g[198]*nmheWorkspace.x[198] + nmheWorkspace.g[199]*nmheWorkspace.x[199] + nmheWorkspace.g[200]*nmheWorkspace.x[200] + nmheWorkspace.g[201]*nmheWorkspace.x[201] + nmheWorkspace.g[202]*nmheWorkspace.x[202] + nmheWorkspace.g[203]*nmheWorkspace.x[203] + nmheWorkspace.g[204]*nmheWorkspace.x[204] + nmheWorkspace.g[205]*nmheWorkspace.x[205] + nmheWorkspace.g[206]*nmheWorkspace.x[206] + nmheWorkspace.g[207]*nmheWorkspace.x[207];
kkt = fabs( kkt );
for (index = 0; index < 208; ++index)
{
prd = nmheWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmheWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmheWorkspace.ub[index] * prd);
}
for (index = 0; index < 200; ++index)
{
prd = nmheWorkspace.y[index + 208];
if (prd > 1e-12)
kkt += fabs(nmheWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmheWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t nmhe_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 8 */
real_t tmpDy[ 8 ];

/** Row vector of size: 6 */
real_t tmpDyN[ 6 ];

/** Row vector of size: 8 */
real_t tmpDx[ 8 ];

for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
nmheWorkspace.objValueIn[0] = nmheVariables.x[lRun1 * 8];
nmheWorkspace.objValueIn[1] = nmheVariables.x[lRun1 * 8 + 1];
nmheWorkspace.objValueIn[2] = nmheVariables.x[lRun1 * 8 + 2];
nmheWorkspace.objValueIn[3] = nmheVariables.x[lRun1 * 8 + 3];
nmheWorkspace.objValueIn[4] = nmheVariables.x[lRun1 * 8 + 4];
nmheWorkspace.objValueIn[5] = nmheVariables.x[lRun1 * 8 + 5];
nmheWorkspace.objValueIn[6] = nmheVariables.x[lRun1 * 8 + 6];
nmheWorkspace.objValueIn[7] = nmheVariables.x[lRun1 * 8 + 7];
nmheWorkspace.objValueIn[8] = nmheVariables.u[lRun1 * 2];
nmheWorkspace.objValueIn[9] = nmheVariables.u[lRun1 * 2 + 1];

nmhe_evaluateLSQ( nmheWorkspace.objValueIn, nmheWorkspace.objValueOut );
nmheWorkspace.Dy[lRun1 * 8] = nmheWorkspace.objValueOut[0] - nmheVariables.y[lRun1 * 8];
nmheWorkspace.Dy[lRun1 * 8 + 1] = nmheWorkspace.objValueOut[1] - nmheVariables.y[lRun1 * 8 + 1];
nmheWorkspace.Dy[lRun1 * 8 + 2] = nmheWorkspace.objValueOut[2] - nmheVariables.y[lRun1 * 8 + 2];
nmheWorkspace.Dy[lRun1 * 8 + 3] = nmheWorkspace.objValueOut[3] - nmheVariables.y[lRun1 * 8 + 3];
nmheWorkspace.Dy[lRun1 * 8 + 4] = nmheWorkspace.objValueOut[4] - nmheVariables.y[lRun1 * 8 + 4];
nmheWorkspace.Dy[lRun1 * 8 + 5] = nmheWorkspace.objValueOut[5] - nmheVariables.y[lRun1 * 8 + 5];
nmheWorkspace.Dy[lRun1 * 8 + 6] = nmheWorkspace.objValueOut[6] - nmheVariables.y[lRun1 * 8 + 6];
nmheWorkspace.Dy[lRun1 * 8 + 7] = nmheWorkspace.objValueOut[7] - nmheVariables.y[lRun1 * 8 + 7];
}
nmheWorkspace.objValueIn[0] = nmheVariables.x[800];
nmheWorkspace.objValueIn[1] = nmheVariables.x[801];
nmheWorkspace.objValueIn[2] = nmheVariables.x[802];
nmheWorkspace.objValueIn[3] = nmheVariables.x[803];
nmheWorkspace.objValueIn[4] = nmheVariables.x[804];
nmheWorkspace.objValueIn[5] = nmheVariables.x[805];
nmheWorkspace.objValueIn[6] = nmheVariables.x[806];
nmheWorkspace.objValueIn[7] = nmheVariables.x[807];
nmhe_evaluateLSQEndTerm( nmheWorkspace.objValueIn, nmheWorkspace.objValueOut );
nmheWorkspace.DyN[0] = nmheWorkspace.objValueOut[0] - nmheVariables.yN[0];
nmheWorkspace.DyN[1] = nmheWorkspace.objValueOut[1] - nmheVariables.yN[1];
nmheWorkspace.DyN[2] = nmheWorkspace.objValueOut[2] - nmheVariables.yN[2];
nmheWorkspace.DyN[3] = nmheWorkspace.objValueOut[3] - nmheVariables.yN[3];
nmheWorkspace.DyN[4] = nmheWorkspace.objValueOut[4] - nmheVariables.yN[4];
nmheWorkspace.DyN[5] = nmheWorkspace.objValueOut[5] - nmheVariables.yN[5];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 100; ++lRun1)
{
tmpDy[0] = + nmheWorkspace.Dy[lRun1 * 8]*nmheVariables.W[0];
tmpDy[1] = + nmheWorkspace.Dy[lRun1 * 8 + 1]*nmheVariables.W[9];
tmpDy[2] = + nmheWorkspace.Dy[lRun1 * 8 + 2]*nmheVariables.W[18];
tmpDy[3] = + nmheWorkspace.Dy[lRun1 * 8 + 3]*nmheVariables.W[27];
tmpDy[4] = + nmheWorkspace.Dy[lRun1 * 8 + 4]*nmheVariables.W[36];
tmpDy[5] = + nmheWorkspace.Dy[lRun1 * 8 + 5]*nmheVariables.W[45];
tmpDy[6] = + nmheWorkspace.Dy[lRun1 * 8 + 6]*nmheVariables.W[54];
tmpDy[7] = + nmheWorkspace.Dy[lRun1 * 8 + 7]*nmheVariables.W[63];
objVal += + nmheWorkspace.Dy[lRun1 * 8]*tmpDy[0] + nmheWorkspace.Dy[lRun1 * 8 + 1]*tmpDy[1] + nmheWorkspace.Dy[lRun1 * 8 + 2]*tmpDy[2] + nmheWorkspace.Dy[lRun1 * 8 + 3]*tmpDy[3] + nmheWorkspace.Dy[lRun1 * 8 + 4]*tmpDy[4] + nmheWorkspace.Dy[lRun1 * 8 + 5]*tmpDy[5] + nmheWorkspace.Dy[lRun1 * 8 + 6]*tmpDy[6] + nmheWorkspace.Dy[lRun1 * 8 + 7]*tmpDy[7];
}

tmpDyN[0] = + nmheWorkspace.DyN[0]*nmheVariables.WN[0];
tmpDyN[1] = + nmheWorkspace.DyN[1]*nmheVariables.WN[7];
tmpDyN[2] = + nmheWorkspace.DyN[2]*nmheVariables.WN[14];
tmpDyN[3] = + nmheWorkspace.DyN[3]*nmheVariables.WN[21];
tmpDyN[4] = + nmheWorkspace.DyN[4]*nmheVariables.WN[28];
tmpDyN[5] = + nmheWorkspace.DyN[5]*nmheVariables.WN[35];
objVal += + nmheWorkspace.DyN[0]*tmpDyN[0] + nmheWorkspace.DyN[1]*tmpDyN[1] + nmheWorkspace.DyN[2]*tmpDyN[2] + nmheWorkspace.DyN[3]*tmpDyN[3] + nmheWorkspace.DyN[4]*tmpDyN[4] + nmheWorkspace.DyN[5]*tmpDyN[5];
tmpDx[0] = + nmheWorkspace.DxAC[0]*nmheVariables.SAC[0] + nmheWorkspace.DxAC[1]*nmheVariables.SAC[8] + nmheWorkspace.DxAC[2]*nmheVariables.SAC[16] + nmheWorkspace.DxAC[3]*nmheVariables.SAC[24] + nmheWorkspace.DxAC[4]*nmheVariables.SAC[32] + nmheWorkspace.DxAC[5]*nmheVariables.SAC[40] + nmheWorkspace.DxAC[6]*nmheVariables.SAC[48] + nmheWorkspace.DxAC[7]*nmheVariables.SAC[56];
tmpDx[1] = + nmheWorkspace.DxAC[0]*nmheVariables.SAC[1] + nmheWorkspace.DxAC[1]*nmheVariables.SAC[9] + nmheWorkspace.DxAC[2]*nmheVariables.SAC[17] + nmheWorkspace.DxAC[3]*nmheVariables.SAC[25] + nmheWorkspace.DxAC[4]*nmheVariables.SAC[33] + nmheWorkspace.DxAC[5]*nmheVariables.SAC[41] + nmheWorkspace.DxAC[6]*nmheVariables.SAC[49] + nmheWorkspace.DxAC[7]*nmheVariables.SAC[57];
tmpDx[2] = + nmheWorkspace.DxAC[0]*nmheVariables.SAC[2] + nmheWorkspace.DxAC[1]*nmheVariables.SAC[10] + nmheWorkspace.DxAC[2]*nmheVariables.SAC[18] + nmheWorkspace.DxAC[3]*nmheVariables.SAC[26] + nmheWorkspace.DxAC[4]*nmheVariables.SAC[34] + nmheWorkspace.DxAC[5]*nmheVariables.SAC[42] + nmheWorkspace.DxAC[6]*nmheVariables.SAC[50] + nmheWorkspace.DxAC[7]*nmheVariables.SAC[58];
tmpDx[3] = + nmheWorkspace.DxAC[0]*nmheVariables.SAC[3] + nmheWorkspace.DxAC[1]*nmheVariables.SAC[11] + nmheWorkspace.DxAC[2]*nmheVariables.SAC[19] + nmheWorkspace.DxAC[3]*nmheVariables.SAC[27] + nmheWorkspace.DxAC[4]*nmheVariables.SAC[35] + nmheWorkspace.DxAC[5]*nmheVariables.SAC[43] + nmheWorkspace.DxAC[6]*nmheVariables.SAC[51] + nmheWorkspace.DxAC[7]*nmheVariables.SAC[59];
tmpDx[4] = + nmheWorkspace.DxAC[0]*nmheVariables.SAC[4] + nmheWorkspace.DxAC[1]*nmheVariables.SAC[12] + nmheWorkspace.DxAC[2]*nmheVariables.SAC[20] + nmheWorkspace.DxAC[3]*nmheVariables.SAC[28] + nmheWorkspace.DxAC[4]*nmheVariables.SAC[36] + nmheWorkspace.DxAC[5]*nmheVariables.SAC[44] + nmheWorkspace.DxAC[6]*nmheVariables.SAC[52] + nmheWorkspace.DxAC[7]*nmheVariables.SAC[60];
tmpDx[5] = + nmheWorkspace.DxAC[0]*nmheVariables.SAC[5] + nmheWorkspace.DxAC[1]*nmheVariables.SAC[13] + nmheWorkspace.DxAC[2]*nmheVariables.SAC[21] + nmheWorkspace.DxAC[3]*nmheVariables.SAC[29] + nmheWorkspace.DxAC[4]*nmheVariables.SAC[37] + nmheWorkspace.DxAC[5]*nmheVariables.SAC[45] + nmheWorkspace.DxAC[6]*nmheVariables.SAC[53] + nmheWorkspace.DxAC[7]*nmheVariables.SAC[61];
tmpDx[6] = + nmheWorkspace.DxAC[0]*nmheVariables.SAC[6] + nmheWorkspace.DxAC[1]*nmheVariables.SAC[14] + nmheWorkspace.DxAC[2]*nmheVariables.SAC[22] + nmheWorkspace.DxAC[3]*nmheVariables.SAC[30] + nmheWorkspace.DxAC[4]*nmheVariables.SAC[38] + nmheWorkspace.DxAC[5]*nmheVariables.SAC[46] + nmheWorkspace.DxAC[6]*nmheVariables.SAC[54] + nmheWorkspace.DxAC[7]*nmheVariables.SAC[62];
tmpDx[7] = + nmheWorkspace.DxAC[0]*nmheVariables.SAC[7] + nmheWorkspace.DxAC[1]*nmheVariables.SAC[15] + nmheWorkspace.DxAC[2]*nmheVariables.SAC[23] + nmheWorkspace.DxAC[3]*nmheVariables.SAC[31] + nmheWorkspace.DxAC[4]*nmheVariables.SAC[39] + nmheWorkspace.DxAC[5]*nmheVariables.SAC[47] + nmheWorkspace.DxAC[6]*nmheVariables.SAC[55] + nmheWorkspace.DxAC[7]*nmheVariables.SAC[63];
objVal += + tmpDx[0]*nmheWorkspace.DxAC[0] + tmpDx[1]*nmheWorkspace.DxAC[1] + tmpDx[2]*nmheWorkspace.DxAC[2] + tmpDx[3]*nmheWorkspace.DxAC[3] + tmpDx[4]*nmheWorkspace.DxAC[4] + tmpDx[5]*nmheWorkspace.DxAC[5] + tmpDx[6]*nmheWorkspace.DxAC[6] + tmpDx[7]*nmheWorkspace.DxAC[7];

objVal *= 0.5;
return objVal;
}

void nmhe_solve_actriangular( real_t* const A, real_t* const b )
{

b[17] = b[17]/A[323];
b[16] -= + A[305]*b[17];
b[16] = b[16]/A[304];
b[15] -= + A[287]*b[17];
b[15] -= + A[286]*b[16];
b[15] = b[15]/A[285];
b[14] -= + A[269]*b[17];
b[14] -= + A[268]*b[16];
b[14] -= + A[267]*b[15];
b[14] = b[14]/A[266];
b[13] -= + A[251]*b[17];
b[13] -= + A[250]*b[16];
b[13] -= + A[249]*b[15];
b[13] -= + A[248]*b[14];
b[13] = b[13]/A[247];
b[12] -= + A[233]*b[17];
b[12] -= + A[232]*b[16];
b[12] -= + A[231]*b[15];
b[12] -= + A[230]*b[14];
b[12] -= + A[229]*b[13];
b[12] = b[12]/A[228];
b[11] -= + A[215]*b[17];
b[11] -= + A[214]*b[16];
b[11] -= + A[213]*b[15];
b[11] -= + A[212]*b[14];
b[11] -= + A[211]*b[13];
b[11] -= + A[210]*b[12];
b[11] = b[11]/A[209];
b[10] -= + A[197]*b[17];
b[10] -= + A[196]*b[16];
b[10] -= + A[195]*b[15];
b[10] -= + A[194]*b[14];
b[10] -= + A[193]*b[13];
b[10] -= + A[192]*b[12];
b[10] -= + A[191]*b[11];
b[10] = b[10]/A[190];
}

real_t nmhe_solve_acsystem( real_t* const A, real_t* const b, real_t* const rk_temp )
{
real_t det;

int i;
int j;
int k;

det = 1.0000000000000000e+00;
for( i=0; i < 18; i++ ) {
	for( j=i; j < 24; j++ ) {
		rk_temp[j] = A[j*18+i];
	}
	rk_temp[24] = rk_temp[i]*rk_temp[i];
	for( j=i+1; j < 24; j++ ) {
		rk_temp[24] += rk_temp[j]*rk_temp[j];
	}
	rk_temp[24] = sqrt(rk_temp[24]);
	rk_temp[i] += (rk_temp[i] < 0 ? -1 : 1)*rk_temp[24];
	rk_temp[24] = rk_temp[i]*rk_temp[i];
	for( j=i+1; j < 24; j++ ) {
		rk_temp[24] += rk_temp[j]*rk_temp[j];
	}
	rk_temp[24] = sqrt(rk_temp[24]);
	for( j=i; j < 24; j++ ) {
		rk_temp[j] = rk_temp[j]/rk_temp[24];
	}
	rk_temp[24] = rk_temp[i]*A[i*18+i];
	for( j=i+1; j < 24; j++ ) {
		rk_temp[24] += rk_temp[j]*A[j*18+i];
	}
	rk_temp[24] *= 2;
	A[i*18+i] -= rk_temp[i]*rk_temp[24];
	det *= 	A[i * 18 + i];
	for( j=i+1; j < 18; j++ ) {
		rk_temp[24] = rk_temp[i]*A[i*18+j];
		for( k=i+1; k < 24; k++ ) {
			rk_temp[24] += rk_temp[k]*A[k*18+j];
		}
		rk_temp[24] *= 2;
		for( k=i; k < 24; k++ ) {
			A[k*18+j] -= rk_temp[k]*rk_temp[24];
		}
	}
	rk_temp[24] = rk_temp[i]*b[i];
	for( k=i+1; k < 24; k++ ) {
		rk_temp[24] += rk_temp[k]*b[k];
	}
	rk_temp[24] *= 2;
	for( k=i; k < 24; k++ ) {
		b[k] -= rk_temp[k]*rk_temp[24];
	}
}

nmhe_solve_actriangular( A, b );
return det;
}



int nmhe_cholObjS( real_t* const A )
{
int ret;

register unsigned i, j, k;
real_t inv;
for (i = 0; i < 8; ++i)
{
A[i * 8 + i] = A[i * 8 + i] < 1e-8 ? 1e-8 : sqrt(A[i * 8 + i]);
inv = 1 / A[i * 8 + i];
for (j = i + 1; j < 8; ++j)
A[j * 8 + i] = A[j * 8 + i] * inv;
for (j = i + 1; j < 8; ++j)
for (k = j; k < 8; ++k)
A[k * 8 + j] = A[k * 8 + j] - A[k * 8 + i] * A[j * 8 + i];
}
for (i = 0; i < 8; ++i)
for (j = i + 1; j < 8; ++j)
A[i * 8 + j] = 0.0;
ret = 0;
return ret;
}

int nmhe_cholSAC( real_t* const A )
{
int ret;

register unsigned i, j, k;
real_t inv;
for (i = 0; i < 8; ++i)
{
A[i * 8 + i] = A[i * 8 + i] < 1e-8 ? 1e-8 : sqrt(A[i * 8 + i]);
inv = 1 / A[i * 8 + i];
for (j = i + 1; j < 8; ++j)
A[j * 8 + i] = A[j * 8 + i] * inv;
for (j = i + 1; j < 8; ++j)
for (k = j; k < 8; ++k)
A[k * 8 + j] = A[k * 8 + j] - A[k * 8 + i] * A[j * 8 + i];
}
for (i = 0; i < 8; ++i)
for (j = i + 1; j < 8; ++j)
A[i * 8 + j] = 0.0;
ret = 0;
return ret;
}

int nmhe_updateArrivalCost( int reset )
{
int ret;

ret = 0;

if ( reset )
{
nmheWorkspace.acXx[0] = nmheVariables.SAC[0];
nmheWorkspace.acXx[1] = nmheVariables.SAC[1];
nmheWorkspace.acXx[2] = nmheVariables.SAC[2];
nmheWorkspace.acXx[3] = nmheVariables.SAC[3];
nmheWorkspace.acXx[4] = nmheVariables.SAC[4];
nmheWorkspace.acXx[5] = nmheVariables.SAC[5];
nmheWorkspace.acXx[6] = nmheVariables.SAC[6];
nmheWorkspace.acXx[7] = nmheVariables.SAC[7];
nmheWorkspace.acXx[8] = nmheVariables.SAC[8];
nmheWorkspace.acXx[9] = nmheVariables.SAC[9];
nmheWorkspace.acXx[10] = nmheVariables.SAC[10];
nmheWorkspace.acXx[11] = nmheVariables.SAC[11];
nmheWorkspace.acXx[12] = nmheVariables.SAC[12];
nmheWorkspace.acXx[13] = nmheVariables.SAC[13];
nmheWorkspace.acXx[14] = nmheVariables.SAC[14];
nmheWorkspace.acXx[15] = nmheVariables.SAC[15];
nmheWorkspace.acXx[16] = nmheVariables.SAC[16];
nmheWorkspace.acXx[17] = nmheVariables.SAC[17];
nmheWorkspace.acXx[18] = nmheVariables.SAC[18];
nmheWorkspace.acXx[19] = nmheVariables.SAC[19];
nmheWorkspace.acXx[20] = nmheVariables.SAC[20];
nmheWorkspace.acXx[21] = nmheVariables.SAC[21];
nmheWorkspace.acXx[22] = nmheVariables.SAC[22];
nmheWorkspace.acXx[23] = nmheVariables.SAC[23];
nmheWorkspace.acXx[24] = nmheVariables.SAC[24];
nmheWorkspace.acXx[25] = nmheVariables.SAC[25];
nmheWorkspace.acXx[26] = nmheVariables.SAC[26];
nmheWorkspace.acXx[27] = nmheVariables.SAC[27];
nmheWorkspace.acXx[28] = nmheVariables.SAC[28];
nmheWorkspace.acXx[29] = nmheVariables.SAC[29];
nmheWorkspace.acXx[30] = nmheVariables.SAC[30];
nmheWorkspace.acXx[31] = nmheVariables.SAC[31];
nmheWorkspace.acXx[32] = nmheVariables.SAC[32];
nmheWorkspace.acXx[33] = nmheVariables.SAC[33];
nmheWorkspace.acXx[34] = nmheVariables.SAC[34];
nmheWorkspace.acXx[35] = nmheVariables.SAC[35];
nmheWorkspace.acXx[36] = nmheVariables.SAC[36];
nmheWorkspace.acXx[37] = nmheVariables.SAC[37];
nmheWorkspace.acXx[38] = nmheVariables.SAC[38];
nmheWorkspace.acXx[39] = nmheVariables.SAC[39];
nmheWorkspace.acXx[40] = nmheVariables.SAC[40];
nmheWorkspace.acXx[41] = nmheVariables.SAC[41];
nmheWorkspace.acXx[42] = nmheVariables.SAC[42];
nmheWorkspace.acXx[43] = nmheVariables.SAC[43];
nmheWorkspace.acXx[44] = nmheVariables.SAC[44];
nmheWorkspace.acXx[45] = nmheVariables.SAC[45];
nmheWorkspace.acXx[46] = nmheVariables.SAC[46];
nmheWorkspace.acXx[47] = nmheVariables.SAC[47];
nmheWorkspace.acXx[48] = nmheVariables.SAC[48];
nmheWorkspace.acXx[49] = nmheVariables.SAC[49];
nmheWorkspace.acXx[50] = nmheVariables.SAC[50];
nmheWorkspace.acXx[51] = nmheVariables.SAC[51];
nmheWorkspace.acXx[52] = nmheVariables.SAC[52];
nmheWorkspace.acXx[53] = nmheVariables.SAC[53];
nmheWorkspace.acXx[54] = nmheVariables.SAC[54];
nmheWorkspace.acXx[55] = nmheVariables.SAC[55];
nmheWorkspace.acXx[56] = nmheVariables.SAC[56];
nmheWorkspace.acXx[57] = nmheVariables.SAC[57];
nmheWorkspace.acXx[58] = nmheVariables.SAC[58];
nmheWorkspace.acXx[59] = nmheVariables.SAC[59];
nmheWorkspace.acXx[60] = nmheVariables.SAC[60];
nmheWorkspace.acXx[61] = nmheVariables.SAC[61];
nmheWorkspace.acXx[62] = nmheVariables.SAC[62];
nmheWorkspace.acXx[63] = nmheVariables.SAC[63];
nmhe_cholSAC( nmheWorkspace.acXx );
nmheWorkspace.acP[0] = nmheWorkspace.acXx[0];
nmheWorkspace.acP[1] = nmheWorkspace.acXx[8];
nmheWorkspace.acP[2] = nmheWorkspace.acXx[16];
nmheWorkspace.acP[3] = nmheWorkspace.acXx[24];
nmheWorkspace.acP[4] = nmheWorkspace.acXx[32];
nmheWorkspace.acP[5] = nmheWorkspace.acXx[40];
nmheWorkspace.acP[6] = nmheWorkspace.acXx[48];
nmheWorkspace.acP[7] = nmheWorkspace.acXx[56];
nmheWorkspace.acP[8] = nmheWorkspace.acXx[1];
nmheWorkspace.acP[9] = nmheWorkspace.acXx[9];
nmheWorkspace.acP[10] = nmheWorkspace.acXx[17];
nmheWorkspace.acP[11] = nmheWorkspace.acXx[25];
nmheWorkspace.acP[12] = nmheWorkspace.acXx[33];
nmheWorkspace.acP[13] = nmheWorkspace.acXx[41];
nmheWorkspace.acP[14] = nmheWorkspace.acXx[49];
nmheWorkspace.acP[15] = nmheWorkspace.acXx[57];
nmheWorkspace.acP[16] = nmheWorkspace.acXx[2];
nmheWorkspace.acP[17] = nmheWorkspace.acXx[10];
nmheWorkspace.acP[18] = nmheWorkspace.acXx[18];
nmheWorkspace.acP[19] = nmheWorkspace.acXx[26];
nmheWorkspace.acP[20] = nmheWorkspace.acXx[34];
nmheWorkspace.acP[21] = nmheWorkspace.acXx[42];
nmheWorkspace.acP[22] = nmheWorkspace.acXx[50];
nmheWorkspace.acP[23] = nmheWorkspace.acXx[58];
nmheWorkspace.acP[24] = nmheWorkspace.acXx[3];
nmheWorkspace.acP[25] = nmheWorkspace.acXx[11];
nmheWorkspace.acP[26] = nmheWorkspace.acXx[19];
nmheWorkspace.acP[27] = nmheWorkspace.acXx[27];
nmheWorkspace.acP[28] = nmheWorkspace.acXx[35];
nmheWorkspace.acP[29] = nmheWorkspace.acXx[43];
nmheWorkspace.acP[30] = nmheWorkspace.acXx[51];
nmheWorkspace.acP[31] = nmheWorkspace.acXx[59];
nmheWorkspace.acP[32] = nmheWorkspace.acXx[4];
nmheWorkspace.acP[33] = nmheWorkspace.acXx[12];
nmheWorkspace.acP[34] = nmheWorkspace.acXx[20];
nmheWorkspace.acP[35] = nmheWorkspace.acXx[28];
nmheWorkspace.acP[36] = nmheWorkspace.acXx[36];
nmheWorkspace.acP[37] = nmheWorkspace.acXx[44];
nmheWorkspace.acP[38] = nmheWorkspace.acXx[52];
nmheWorkspace.acP[39] = nmheWorkspace.acXx[60];
nmheWorkspace.acP[40] = nmheWorkspace.acXx[5];
nmheWorkspace.acP[41] = nmheWorkspace.acXx[13];
nmheWorkspace.acP[42] = nmheWorkspace.acXx[21];
nmheWorkspace.acP[43] = nmheWorkspace.acXx[29];
nmheWorkspace.acP[44] = nmheWorkspace.acXx[37];
nmheWorkspace.acP[45] = nmheWorkspace.acXx[45];
nmheWorkspace.acP[46] = nmheWorkspace.acXx[53];
nmheWorkspace.acP[47] = nmheWorkspace.acXx[61];
nmheWorkspace.acP[48] = nmheWorkspace.acXx[6];
nmheWorkspace.acP[49] = nmheWorkspace.acXx[14];
nmheWorkspace.acP[50] = nmheWorkspace.acXx[22];
nmheWorkspace.acP[51] = nmheWorkspace.acXx[30];
nmheWorkspace.acP[52] = nmheWorkspace.acXx[38];
nmheWorkspace.acP[53] = nmheWorkspace.acXx[46];
nmheWorkspace.acP[54] = nmheWorkspace.acXx[54];
nmheWorkspace.acP[55] = nmheWorkspace.acXx[62];
nmheWorkspace.acP[56] = nmheWorkspace.acXx[7];
nmheWorkspace.acP[57] = nmheWorkspace.acXx[15];
nmheWorkspace.acP[58] = nmheWorkspace.acXx[23];
nmheWorkspace.acP[59] = nmheWorkspace.acXx[31];
nmheWorkspace.acP[60] = nmheWorkspace.acXx[39];
nmheWorkspace.acP[61] = nmheWorkspace.acXx[47];
nmheWorkspace.acP[62] = nmheWorkspace.acXx[55];
nmheWorkspace.acP[63] = nmheWorkspace.acXx[63];
return 0;
}

nmheWorkspace.state[0] = nmheVariables.x[0];
nmheWorkspace.state[1] = nmheVariables.x[1];
nmheWorkspace.state[2] = nmheVariables.x[2];
nmheWorkspace.state[3] = nmheVariables.x[3];
nmheWorkspace.state[4] = nmheVariables.x[4];
nmheWorkspace.state[5] = nmheVariables.x[5];
nmheWorkspace.state[6] = nmheVariables.x[6];
nmheWorkspace.state[7] = nmheVariables.x[7];
nmheWorkspace.state[88] = nmheVariables.u[0];
nmheWorkspace.state[89] = nmheVariables.u[1];
nmhe_integrate(nmheWorkspace.state, 1);

nmheWorkspace.objValueIn[0] = nmheVariables.x[0];
nmheWorkspace.objValueIn[1] = nmheVariables.x[1];
nmheWorkspace.objValueIn[2] = nmheVariables.x[2];
nmheWorkspace.objValueIn[3] = nmheVariables.x[3];
nmheWorkspace.objValueIn[4] = nmheVariables.x[4];
nmheWorkspace.objValueIn[5] = nmheVariables.x[5];
nmheWorkspace.objValueIn[6] = nmheVariables.x[6];
nmheWorkspace.objValueIn[7] = nmheVariables.x[7];
nmheWorkspace.objValueIn[8] = nmheVariables.u[0];
nmheWorkspace.objValueIn[9] = nmheVariables.u[1];
nmhe_evaluateLSQ( nmheWorkspace.objValueIn, nmheWorkspace.objValueOut );

nmheWorkspace.acVL[0] = nmheVariables.W[0];
nmheWorkspace.acVL[1] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[2] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[3] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[4] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[5] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[6] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[7] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[8] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[9] = nmheVariables.W[9];
nmheWorkspace.acVL[10] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[11] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[12] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[13] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[14] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[15] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[16] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[17] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[18] = nmheVariables.W[18];
nmheWorkspace.acVL[19] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[20] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[21] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[22] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[23] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[24] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[25] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[26] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[27] = nmheVariables.W[27];
nmheWorkspace.acVL[28] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[29] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[30] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[31] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[32] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[33] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[34] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[35] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[36] = nmheVariables.W[36];
nmheWorkspace.acVL[37] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[38] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[39] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[40] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[41] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[42] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[43] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[44] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[45] = nmheVariables.W[45];
nmheWorkspace.acVL[46] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[47] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[48] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[49] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[50] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[51] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[52] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[53] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[54] = nmheVariables.W[54];
nmheWorkspace.acVL[55] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[56] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[57] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[58] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[59] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[60] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[61] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[62] = (real_t)0.0000000000000000e+00;
nmheWorkspace.acVL[63] = nmheVariables.W[63];
nmhe_cholObjS( nmheWorkspace.acVL );
{ int lCopy; for (lCopy = 0; lCopy < 432; lCopy++) nmheWorkspace.acA[ lCopy ] = 0; }
{ int lCopy; for (lCopy = 0; lCopy < 24; lCopy++) nmheWorkspace.acb[ lCopy ] = 0; }

nmheWorkspace.acXx[0] = nmheWorkspace.state[8];
nmheWorkspace.acXx[1] = nmheWorkspace.state[9];
nmheWorkspace.acXx[2] = nmheWorkspace.state[10];
nmheWorkspace.acXx[3] = nmheWorkspace.state[11];
nmheWorkspace.acXx[4] = nmheWorkspace.state[12];
nmheWorkspace.acXx[5] = nmheWorkspace.state[13];
nmheWorkspace.acXx[6] = nmheWorkspace.state[14];
nmheWorkspace.acXx[7] = nmheWorkspace.state[15];
nmheWorkspace.acXx[8] = nmheWorkspace.state[16];
nmheWorkspace.acXx[9] = nmheWorkspace.state[17];
nmheWorkspace.acXx[10] = nmheWorkspace.state[18];
nmheWorkspace.acXx[11] = nmheWorkspace.state[19];
nmheWorkspace.acXx[12] = nmheWorkspace.state[20];
nmheWorkspace.acXx[13] = nmheWorkspace.state[21];
nmheWorkspace.acXx[14] = nmheWorkspace.state[22];
nmheWorkspace.acXx[15] = nmheWorkspace.state[23];
nmheWorkspace.acXx[16] = nmheWorkspace.state[24];
nmheWorkspace.acXx[17] = nmheWorkspace.state[25];
nmheWorkspace.acXx[18] = nmheWorkspace.state[26];
nmheWorkspace.acXx[19] = nmheWorkspace.state[27];
nmheWorkspace.acXx[20] = nmheWorkspace.state[28];
nmheWorkspace.acXx[21] = nmheWorkspace.state[29];
nmheWorkspace.acXx[22] = nmheWorkspace.state[30];
nmheWorkspace.acXx[23] = nmheWorkspace.state[31];
nmheWorkspace.acXx[24] = nmheWorkspace.state[32];
nmheWorkspace.acXx[25] = nmheWorkspace.state[33];
nmheWorkspace.acXx[26] = nmheWorkspace.state[34];
nmheWorkspace.acXx[27] = nmheWorkspace.state[35];
nmheWorkspace.acXx[28] = nmheWorkspace.state[36];
nmheWorkspace.acXx[29] = nmheWorkspace.state[37];
nmheWorkspace.acXx[30] = nmheWorkspace.state[38];
nmheWorkspace.acXx[31] = nmheWorkspace.state[39];
nmheWorkspace.acXx[32] = nmheWorkspace.state[40];
nmheWorkspace.acXx[33] = nmheWorkspace.state[41];
nmheWorkspace.acXx[34] = nmheWorkspace.state[42];
nmheWorkspace.acXx[35] = nmheWorkspace.state[43];
nmheWorkspace.acXx[36] = nmheWorkspace.state[44];
nmheWorkspace.acXx[37] = nmheWorkspace.state[45];
nmheWorkspace.acXx[38] = nmheWorkspace.state[46];
nmheWorkspace.acXx[39] = nmheWorkspace.state[47];
nmheWorkspace.acXx[40] = nmheWorkspace.state[48];
nmheWorkspace.acXx[41] = nmheWorkspace.state[49];
nmheWorkspace.acXx[42] = nmheWorkspace.state[50];
nmheWorkspace.acXx[43] = nmheWorkspace.state[51];
nmheWorkspace.acXx[44] = nmheWorkspace.state[52];
nmheWorkspace.acXx[45] = nmheWorkspace.state[53];
nmheWorkspace.acXx[46] = nmheWorkspace.state[54];
nmheWorkspace.acXx[47] = nmheWorkspace.state[55];
nmheWorkspace.acXx[48] = nmheWorkspace.state[56];
nmheWorkspace.acXx[49] = nmheWorkspace.state[57];
nmheWorkspace.acXx[50] = nmheWorkspace.state[58];
nmheWorkspace.acXx[51] = nmheWorkspace.state[59];
nmheWorkspace.acXx[52] = nmheWorkspace.state[60];
nmheWorkspace.acXx[53] = nmheWorkspace.state[61];
nmheWorkspace.acXx[54] = nmheWorkspace.state[62];
nmheWorkspace.acXx[55] = nmheWorkspace.state[63];
nmheWorkspace.acXx[56] = nmheWorkspace.state[64];
nmheWorkspace.acXx[57] = nmheWorkspace.state[65];
nmheWorkspace.acXx[58] = nmheWorkspace.state[66];
nmheWorkspace.acXx[59] = nmheWorkspace.state[67];
nmheWorkspace.acXx[60] = nmheWorkspace.state[68];
nmheWorkspace.acXx[61] = nmheWorkspace.state[69];
nmheWorkspace.acXx[62] = nmheWorkspace.state[70];
nmheWorkspace.acXx[63] = nmheWorkspace.state[71];
nmheWorkspace.acXu[0] = nmheWorkspace.state[72];
nmheWorkspace.acXu[1] = nmheWorkspace.state[73];
nmheWorkspace.acXu[2] = nmheWorkspace.state[74];
nmheWorkspace.acXu[3] = nmheWorkspace.state[75];
nmheWorkspace.acXu[4] = nmheWorkspace.state[76];
nmheWorkspace.acXu[5] = nmheWorkspace.state[77];
nmheWorkspace.acXu[6] = nmheWorkspace.state[78];
nmheWorkspace.acXu[7] = nmheWorkspace.state[79];
nmheWorkspace.acXu[8] = nmheWorkspace.state[80];
nmheWorkspace.acXu[9] = nmheWorkspace.state[81];
nmheWorkspace.acXu[10] = nmheWorkspace.state[82];
nmheWorkspace.acXu[11] = nmheWorkspace.state[83];
nmheWorkspace.acXu[12] = nmheWorkspace.state[84];
nmheWorkspace.acXu[13] = nmheWorkspace.state[85];
nmheWorkspace.acXu[14] = nmheWorkspace.state[86];
nmheWorkspace.acXu[15] = nmheWorkspace.state[87];
nmheWorkspace.acA[0] = nmheWorkspace.acP[0];
nmheWorkspace.acA[1] = nmheWorkspace.acP[1];
nmheWorkspace.acA[2] = nmheWorkspace.acP[2];
nmheWorkspace.acA[3] = nmheWorkspace.acP[3];
nmheWorkspace.acA[4] = nmheWorkspace.acP[4];
nmheWorkspace.acA[5] = nmheWorkspace.acP[5];
nmheWorkspace.acA[6] = nmheWorkspace.acP[6];
nmheWorkspace.acA[7] = nmheWorkspace.acP[7];
nmheWorkspace.acA[18] = nmheWorkspace.acP[8];
nmheWorkspace.acA[19] = nmheWorkspace.acP[9];
nmheWorkspace.acA[20] = nmheWorkspace.acP[10];
nmheWorkspace.acA[21] = nmheWorkspace.acP[11];
nmheWorkspace.acA[22] = nmheWorkspace.acP[12];
nmheWorkspace.acA[23] = nmheWorkspace.acP[13];
nmheWorkspace.acA[24] = nmheWorkspace.acP[14];
nmheWorkspace.acA[25] = nmheWorkspace.acP[15];
nmheWorkspace.acA[36] = nmheWorkspace.acP[16];
nmheWorkspace.acA[37] = nmheWorkspace.acP[17];
nmheWorkspace.acA[38] = nmheWorkspace.acP[18];
nmheWorkspace.acA[39] = nmheWorkspace.acP[19];
nmheWorkspace.acA[40] = nmheWorkspace.acP[20];
nmheWorkspace.acA[41] = nmheWorkspace.acP[21];
nmheWorkspace.acA[42] = nmheWorkspace.acP[22];
nmheWorkspace.acA[43] = nmheWorkspace.acP[23];
nmheWorkspace.acA[54] = nmheWorkspace.acP[24];
nmheWorkspace.acA[55] = nmheWorkspace.acP[25];
nmheWorkspace.acA[56] = nmheWorkspace.acP[26];
nmheWorkspace.acA[57] = nmheWorkspace.acP[27];
nmheWorkspace.acA[58] = nmheWorkspace.acP[28];
nmheWorkspace.acA[59] = nmheWorkspace.acP[29];
nmheWorkspace.acA[60] = nmheWorkspace.acP[30];
nmheWorkspace.acA[61] = nmheWorkspace.acP[31];
nmheWorkspace.acA[72] = nmheWorkspace.acP[32];
nmheWorkspace.acA[73] = nmheWorkspace.acP[33];
nmheWorkspace.acA[74] = nmheWorkspace.acP[34];
nmheWorkspace.acA[75] = nmheWorkspace.acP[35];
nmheWorkspace.acA[76] = nmheWorkspace.acP[36];
nmheWorkspace.acA[77] = nmheWorkspace.acP[37];
nmheWorkspace.acA[78] = nmheWorkspace.acP[38];
nmheWorkspace.acA[79] = nmheWorkspace.acP[39];
nmheWorkspace.acA[90] = nmheWorkspace.acP[40];
nmheWorkspace.acA[91] = nmheWorkspace.acP[41];
nmheWorkspace.acA[92] = nmheWorkspace.acP[42];
nmheWorkspace.acA[93] = nmheWorkspace.acP[43];
nmheWorkspace.acA[94] = nmheWorkspace.acP[44];
nmheWorkspace.acA[95] = nmheWorkspace.acP[45];
nmheWorkspace.acA[96] = nmheWorkspace.acP[46];
nmheWorkspace.acA[97] = nmheWorkspace.acP[47];
nmheWorkspace.acA[108] = nmheWorkspace.acP[48];
nmheWorkspace.acA[109] = nmheWorkspace.acP[49];
nmheWorkspace.acA[110] = nmheWorkspace.acP[50];
nmheWorkspace.acA[111] = nmheWorkspace.acP[51];
nmheWorkspace.acA[112] = nmheWorkspace.acP[52];
nmheWorkspace.acA[113] = nmheWorkspace.acP[53];
nmheWorkspace.acA[114] = nmheWorkspace.acP[54];
nmheWorkspace.acA[115] = nmheWorkspace.acP[55];
nmheWorkspace.acA[126] = nmheWorkspace.acP[56];
nmheWorkspace.acA[127] = nmheWorkspace.acP[57];
nmheWorkspace.acA[128] = nmheWorkspace.acP[58];
nmheWorkspace.acA[129] = nmheWorkspace.acP[59];
nmheWorkspace.acA[130] = nmheWorkspace.acP[60];
nmheWorkspace.acA[131] = nmheWorkspace.acP[61];
nmheWorkspace.acA[132] = nmheWorkspace.acP[62];
nmheWorkspace.acA[133] = nmheWorkspace.acP[63];
nmheWorkspace.acA[144] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHx[0] + nmheWorkspace.acVL[8]*nmheWorkspace.acHx[8] + nmheWorkspace.acVL[16]*nmheWorkspace.acHx[16] + nmheWorkspace.acVL[24]*nmheWorkspace.acHx[24] + nmheWorkspace.acVL[32]*nmheWorkspace.acHx[32] + nmheWorkspace.acVL[40]*nmheWorkspace.acHx[40] + nmheWorkspace.acVL[48]*nmheWorkspace.acHx[48] + nmheWorkspace.acVL[56]*nmheWorkspace.acHx[56];
nmheWorkspace.acA[145] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHx[1] + nmheWorkspace.acVL[8]*nmheWorkspace.acHx[9] + nmheWorkspace.acVL[16]*nmheWorkspace.acHx[17] + nmheWorkspace.acVL[24]*nmheWorkspace.acHx[25] + nmheWorkspace.acVL[32]*nmheWorkspace.acHx[33] + nmheWorkspace.acVL[40]*nmheWorkspace.acHx[41] + nmheWorkspace.acVL[48]*nmheWorkspace.acHx[49] + nmheWorkspace.acVL[56]*nmheWorkspace.acHx[57];
nmheWorkspace.acA[146] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHx[2] + nmheWorkspace.acVL[8]*nmheWorkspace.acHx[10] + nmheWorkspace.acVL[16]*nmheWorkspace.acHx[18] + nmheWorkspace.acVL[24]*nmheWorkspace.acHx[26] + nmheWorkspace.acVL[32]*nmheWorkspace.acHx[34] + nmheWorkspace.acVL[40]*nmheWorkspace.acHx[42] + nmheWorkspace.acVL[48]*nmheWorkspace.acHx[50] + nmheWorkspace.acVL[56]*nmheWorkspace.acHx[58];
nmheWorkspace.acA[147] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHx[3] + nmheWorkspace.acVL[8]*nmheWorkspace.acHx[11] + nmheWorkspace.acVL[16]*nmheWorkspace.acHx[19] + nmheWorkspace.acVL[24]*nmheWorkspace.acHx[27] + nmheWorkspace.acVL[32]*nmheWorkspace.acHx[35] + nmheWorkspace.acVL[40]*nmheWorkspace.acHx[43] + nmheWorkspace.acVL[48]*nmheWorkspace.acHx[51] + nmheWorkspace.acVL[56]*nmheWorkspace.acHx[59];
nmheWorkspace.acA[148] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHx[4] + nmheWorkspace.acVL[8]*nmheWorkspace.acHx[12] + nmheWorkspace.acVL[16]*nmheWorkspace.acHx[20] + nmheWorkspace.acVL[24]*nmheWorkspace.acHx[28] + nmheWorkspace.acVL[32]*nmheWorkspace.acHx[36] + nmheWorkspace.acVL[40]*nmheWorkspace.acHx[44] + nmheWorkspace.acVL[48]*nmheWorkspace.acHx[52] + nmheWorkspace.acVL[56]*nmheWorkspace.acHx[60];
nmheWorkspace.acA[149] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHx[5] + nmheWorkspace.acVL[8]*nmheWorkspace.acHx[13] + nmheWorkspace.acVL[16]*nmheWorkspace.acHx[21] + nmheWorkspace.acVL[24]*nmheWorkspace.acHx[29] + nmheWorkspace.acVL[32]*nmheWorkspace.acHx[37] + nmheWorkspace.acVL[40]*nmheWorkspace.acHx[45] + nmheWorkspace.acVL[48]*nmheWorkspace.acHx[53] + nmheWorkspace.acVL[56]*nmheWorkspace.acHx[61];
nmheWorkspace.acA[150] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHx[6] + nmheWorkspace.acVL[8]*nmheWorkspace.acHx[14] + nmheWorkspace.acVL[16]*nmheWorkspace.acHx[22] + nmheWorkspace.acVL[24]*nmheWorkspace.acHx[30] + nmheWorkspace.acVL[32]*nmheWorkspace.acHx[38] + nmheWorkspace.acVL[40]*nmheWorkspace.acHx[46] + nmheWorkspace.acVL[48]*nmheWorkspace.acHx[54] + nmheWorkspace.acVL[56]*nmheWorkspace.acHx[62];
nmheWorkspace.acA[151] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHx[7] + nmheWorkspace.acVL[8]*nmheWorkspace.acHx[15] + nmheWorkspace.acVL[16]*nmheWorkspace.acHx[23] + nmheWorkspace.acVL[24]*nmheWorkspace.acHx[31] + nmheWorkspace.acVL[32]*nmheWorkspace.acHx[39] + nmheWorkspace.acVL[40]*nmheWorkspace.acHx[47] + nmheWorkspace.acVL[48]*nmheWorkspace.acHx[55] + nmheWorkspace.acVL[56]*nmheWorkspace.acHx[63];
nmheWorkspace.acA[162] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHx[0] + nmheWorkspace.acVL[9]*nmheWorkspace.acHx[8] + nmheWorkspace.acVL[17]*nmheWorkspace.acHx[16] + nmheWorkspace.acVL[25]*nmheWorkspace.acHx[24] + nmheWorkspace.acVL[33]*nmheWorkspace.acHx[32] + nmheWorkspace.acVL[41]*nmheWorkspace.acHx[40] + nmheWorkspace.acVL[49]*nmheWorkspace.acHx[48] + nmheWorkspace.acVL[57]*nmheWorkspace.acHx[56];
nmheWorkspace.acA[163] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHx[1] + nmheWorkspace.acVL[9]*nmheWorkspace.acHx[9] + nmheWorkspace.acVL[17]*nmheWorkspace.acHx[17] + nmheWorkspace.acVL[25]*nmheWorkspace.acHx[25] + nmheWorkspace.acVL[33]*nmheWorkspace.acHx[33] + nmheWorkspace.acVL[41]*nmheWorkspace.acHx[41] + nmheWorkspace.acVL[49]*nmheWorkspace.acHx[49] + nmheWorkspace.acVL[57]*nmheWorkspace.acHx[57];
nmheWorkspace.acA[164] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHx[2] + nmheWorkspace.acVL[9]*nmheWorkspace.acHx[10] + nmheWorkspace.acVL[17]*nmheWorkspace.acHx[18] + nmheWorkspace.acVL[25]*nmheWorkspace.acHx[26] + nmheWorkspace.acVL[33]*nmheWorkspace.acHx[34] + nmheWorkspace.acVL[41]*nmheWorkspace.acHx[42] + nmheWorkspace.acVL[49]*nmheWorkspace.acHx[50] + nmheWorkspace.acVL[57]*nmheWorkspace.acHx[58];
nmheWorkspace.acA[165] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHx[3] + nmheWorkspace.acVL[9]*nmheWorkspace.acHx[11] + nmheWorkspace.acVL[17]*nmheWorkspace.acHx[19] + nmheWorkspace.acVL[25]*nmheWorkspace.acHx[27] + nmheWorkspace.acVL[33]*nmheWorkspace.acHx[35] + nmheWorkspace.acVL[41]*nmheWorkspace.acHx[43] + nmheWorkspace.acVL[49]*nmheWorkspace.acHx[51] + nmheWorkspace.acVL[57]*nmheWorkspace.acHx[59];
nmheWorkspace.acA[166] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHx[4] + nmheWorkspace.acVL[9]*nmheWorkspace.acHx[12] + nmheWorkspace.acVL[17]*nmheWorkspace.acHx[20] + nmheWorkspace.acVL[25]*nmheWorkspace.acHx[28] + nmheWorkspace.acVL[33]*nmheWorkspace.acHx[36] + nmheWorkspace.acVL[41]*nmheWorkspace.acHx[44] + nmheWorkspace.acVL[49]*nmheWorkspace.acHx[52] + nmheWorkspace.acVL[57]*nmheWorkspace.acHx[60];
nmheWorkspace.acA[167] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHx[5] + nmheWorkspace.acVL[9]*nmheWorkspace.acHx[13] + nmheWorkspace.acVL[17]*nmheWorkspace.acHx[21] + nmheWorkspace.acVL[25]*nmheWorkspace.acHx[29] + nmheWorkspace.acVL[33]*nmheWorkspace.acHx[37] + nmheWorkspace.acVL[41]*nmheWorkspace.acHx[45] + nmheWorkspace.acVL[49]*nmheWorkspace.acHx[53] + nmheWorkspace.acVL[57]*nmheWorkspace.acHx[61];
nmheWorkspace.acA[168] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHx[6] + nmheWorkspace.acVL[9]*nmheWorkspace.acHx[14] + nmheWorkspace.acVL[17]*nmheWorkspace.acHx[22] + nmheWorkspace.acVL[25]*nmheWorkspace.acHx[30] + nmheWorkspace.acVL[33]*nmheWorkspace.acHx[38] + nmheWorkspace.acVL[41]*nmheWorkspace.acHx[46] + nmheWorkspace.acVL[49]*nmheWorkspace.acHx[54] + nmheWorkspace.acVL[57]*nmheWorkspace.acHx[62];
nmheWorkspace.acA[169] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHx[7] + nmheWorkspace.acVL[9]*nmheWorkspace.acHx[15] + nmheWorkspace.acVL[17]*nmheWorkspace.acHx[23] + nmheWorkspace.acVL[25]*nmheWorkspace.acHx[31] + nmheWorkspace.acVL[33]*nmheWorkspace.acHx[39] + nmheWorkspace.acVL[41]*nmheWorkspace.acHx[47] + nmheWorkspace.acVL[49]*nmheWorkspace.acHx[55] + nmheWorkspace.acVL[57]*nmheWorkspace.acHx[63];
nmheWorkspace.acA[180] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHx[0] + nmheWorkspace.acVL[10]*nmheWorkspace.acHx[8] + nmheWorkspace.acVL[18]*nmheWorkspace.acHx[16] + nmheWorkspace.acVL[26]*nmheWorkspace.acHx[24] + nmheWorkspace.acVL[34]*nmheWorkspace.acHx[32] + nmheWorkspace.acVL[42]*nmheWorkspace.acHx[40] + nmheWorkspace.acVL[50]*nmheWorkspace.acHx[48] + nmheWorkspace.acVL[58]*nmheWorkspace.acHx[56];
nmheWorkspace.acA[181] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHx[1] + nmheWorkspace.acVL[10]*nmheWorkspace.acHx[9] + nmheWorkspace.acVL[18]*nmheWorkspace.acHx[17] + nmheWorkspace.acVL[26]*nmheWorkspace.acHx[25] + nmheWorkspace.acVL[34]*nmheWorkspace.acHx[33] + nmheWorkspace.acVL[42]*nmheWorkspace.acHx[41] + nmheWorkspace.acVL[50]*nmheWorkspace.acHx[49] + nmheWorkspace.acVL[58]*nmheWorkspace.acHx[57];
nmheWorkspace.acA[182] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHx[2] + nmheWorkspace.acVL[10]*nmheWorkspace.acHx[10] + nmheWorkspace.acVL[18]*nmheWorkspace.acHx[18] + nmheWorkspace.acVL[26]*nmheWorkspace.acHx[26] + nmheWorkspace.acVL[34]*nmheWorkspace.acHx[34] + nmheWorkspace.acVL[42]*nmheWorkspace.acHx[42] + nmheWorkspace.acVL[50]*nmheWorkspace.acHx[50] + nmheWorkspace.acVL[58]*nmheWorkspace.acHx[58];
nmheWorkspace.acA[183] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHx[3] + nmheWorkspace.acVL[10]*nmheWorkspace.acHx[11] + nmheWorkspace.acVL[18]*nmheWorkspace.acHx[19] + nmheWorkspace.acVL[26]*nmheWorkspace.acHx[27] + nmheWorkspace.acVL[34]*nmheWorkspace.acHx[35] + nmheWorkspace.acVL[42]*nmheWorkspace.acHx[43] + nmheWorkspace.acVL[50]*nmheWorkspace.acHx[51] + nmheWorkspace.acVL[58]*nmheWorkspace.acHx[59];
nmheWorkspace.acA[184] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHx[4] + nmheWorkspace.acVL[10]*nmheWorkspace.acHx[12] + nmheWorkspace.acVL[18]*nmheWorkspace.acHx[20] + nmheWorkspace.acVL[26]*nmheWorkspace.acHx[28] + nmheWorkspace.acVL[34]*nmheWorkspace.acHx[36] + nmheWorkspace.acVL[42]*nmheWorkspace.acHx[44] + nmheWorkspace.acVL[50]*nmheWorkspace.acHx[52] + nmheWorkspace.acVL[58]*nmheWorkspace.acHx[60];
nmheWorkspace.acA[185] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHx[5] + nmheWorkspace.acVL[10]*nmheWorkspace.acHx[13] + nmheWorkspace.acVL[18]*nmheWorkspace.acHx[21] + nmheWorkspace.acVL[26]*nmheWorkspace.acHx[29] + nmheWorkspace.acVL[34]*nmheWorkspace.acHx[37] + nmheWorkspace.acVL[42]*nmheWorkspace.acHx[45] + nmheWorkspace.acVL[50]*nmheWorkspace.acHx[53] + nmheWorkspace.acVL[58]*nmheWorkspace.acHx[61];
nmheWorkspace.acA[186] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHx[6] + nmheWorkspace.acVL[10]*nmheWorkspace.acHx[14] + nmheWorkspace.acVL[18]*nmheWorkspace.acHx[22] + nmheWorkspace.acVL[26]*nmheWorkspace.acHx[30] + nmheWorkspace.acVL[34]*nmheWorkspace.acHx[38] + nmheWorkspace.acVL[42]*nmheWorkspace.acHx[46] + nmheWorkspace.acVL[50]*nmheWorkspace.acHx[54] + nmheWorkspace.acVL[58]*nmheWorkspace.acHx[62];
nmheWorkspace.acA[187] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHx[7] + nmheWorkspace.acVL[10]*nmheWorkspace.acHx[15] + nmheWorkspace.acVL[18]*nmheWorkspace.acHx[23] + nmheWorkspace.acVL[26]*nmheWorkspace.acHx[31] + nmheWorkspace.acVL[34]*nmheWorkspace.acHx[39] + nmheWorkspace.acVL[42]*nmheWorkspace.acHx[47] + nmheWorkspace.acVL[50]*nmheWorkspace.acHx[55] + nmheWorkspace.acVL[58]*nmheWorkspace.acHx[63];
nmheWorkspace.acA[198] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHx[0] + nmheWorkspace.acVL[11]*nmheWorkspace.acHx[8] + nmheWorkspace.acVL[19]*nmheWorkspace.acHx[16] + nmheWorkspace.acVL[27]*nmheWorkspace.acHx[24] + nmheWorkspace.acVL[35]*nmheWorkspace.acHx[32] + nmheWorkspace.acVL[43]*nmheWorkspace.acHx[40] + nmheWorkspace.acVL[51]*nmheWorkspace.acHx[48] + nmheWorkspace.acVL[59]*nmheWorkspace.acHx[56];
nmheWorkspace.acA[199] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHx[1] + nmheWorkspace.acVL[11]*nmheWorkspace.acHx[9] + nmheWorkspace.acVL[19]*nmheWorkspace.acHx[17] + nmheWorkspace.acVL[27]*nmheWorkspace.acHx[25] + nmheWorkspace.acVL[35]*nmheWorkspace.acHx[33] + nmheWorkspace.acVL[43]*nmheWorkspace.acHx[41] + nmheWorkspace.acVL[51]*nmheWorkspace.acHx[49] + nmheWorkspace.acVL[59]*nmheWorkspace.acHx[57];
nmheWorkspace.acA[200] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHx[2] + nmheWorkspace.acVL[11]*nmheWorkspace.acHx[10] + nmheWorkspace.acVL[19]*nmheWorkspace.acHx[18] + nmheWorkspace.acVL[27]*nmheWorkspace.acHx[26] + nmheWorkspace.acVL[35]*nmheWorkspace.acHx[34] + nmheWorkspace.acVL[43]*nmheWorkspace.acHx[42] + nmheWorkspace.acVL[51]*nmheWorkspace.acHx[50] + nmheWorkspace.acVL[59]*nmheWorkspace.acHx[58];
nmheWorkspace.acA[201] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHx[3] + nmheWorkspace.acVL[11]*nmheWorkspace.acHx[11] + nmheWorkspace.acVL[19]*nmheWorkspace.acHx[19] + nmheWorkspace.acVL[27]*nmheWorkspace.acHx[27] + nmheWorkspace.acVL[35]*nmheWorkspace.acHx[35] + nmheWorkspace.acVL[43]*nmheWorkspace.acHx[43] + nmheWorkspace.acVL[51]*nmheWorkspace.acHx[51] + nmheWorkspace.acVL[59]*nmheWorkspace.acHx[59];
nmheWorkspace.acA[202] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHx[4] + nmheWorkspace.acVL[11]*nmheWorkspace.acHx[12] + nmheWorkspace.acVL[19]*nmheWorkspace.acHx[20] + nmheWorkspace.acVL[27]*nmheWorkspace.acHx[28] + nmheWorkspace.acVL[35]*nmheWorkspace.acHx[36] + nmheWorkspace.acVL[43]*nmheWorkspace.acHx[44] + nmheWorkspace.acVL[51]*nmheWorkspace.acHx[52] + nmheWorkspace.acVL[59]*nmheWorkspace.acHx[60];
nmheWorkspace.acA[203] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHx[5] + nmheWorkspace.acVL[11]*nmheWorkspace.acHx[13] + nmheWorkspace.acVL[19]*nmheWorkspace.acHx[21] + nmheWorkspace.acVL[27]*nmheWorkspace.acHx[29] + nmheWorkspace.acVL[35]*nmheWorkspace.acHx[37] + nmheWorkspace.acVL[43]*nmheWorkspace.acHx[45] + nmheWorkspace.acVL[51]*nmheWorkspace.acHx[53] + nmheWorkspace.acVL[59]*nmheWorkspace.acHx[61];
nmheWorkspace.acA[204] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHx[6] + nmheWorkspace.acVL[11]*nmheWorkspace.acHx[14] + nmheWorkspace.acVL[19]*nmheWorkspace.acHx[22] + nmheWorkspace.acVL[27]*nmheWorkspace.acHx[30] + nmheWorkspace.acVL[35]*nmheWorkspace.acHx[38] + nmheWorkspace.acVL[43]*nmheWorkspace.acHx[46] + nmheWorkspace.acVL[51]*nmheWorkspace.acHx[54] + nmheWorkspace.acVL[59]*nmheWorkspace.acHx[62];
nmheWorkspace.acA[205] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHx[7] + nmheWorkspace.acVL[11]*nmheWorkspace.acHx[15] + nmheWorkspace.acVL[19]*nmheWorkspace.acHx[23] + nmheWorkspace.acVL[27]*nmheWorkspace.acHx[31] + nmheWorkspace.acVL[35]*nmheWorkspace.acHx[39] + nmheWorkspace.acVL[43]*nmheWorkspace.acHx[47] + nmheWorkspace.acVL[51]*nmheWorkspace.acHx[55] + nmheWorkspace.acVL[59]*nmheWorkspace.acHx[63];
nmheWorkspace.acA[216] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHx[0] + nmheWorkspace.acVL[12]*nmheWorkspace.acHx[8] + nmheWorkspace.acVL[20]*nmheWorkspace.acHx[16] + nmheWorkspace.acVL[28]*nmheWorkspace.acHx[24] + nmheWorkspace.acVL[36]*nmheWorkspace.acHx[32] + nmheWorkspace.acVL[44]*nmheWorkspace.acHx[40] + nmheWorkspace.acVL[52]*nmheWorkspace.acHx[48] + nmheWorkspace.acVL[60]*nmheWorkspace.acHx[56];
nmheWorkspace.acA[217] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHx[1] + nmheWorkspace.acVL[12]*nmheWorkspace.acHx[9] + nmheWorkspace.acVL[20]*nmheWorkspace.acHx[17] + nmheWorkspace.acVL[28]*nmheWorkspace.acHx[25] + nmheWorkspace.acVL[36]*nmheWorkspace.acHx[33] + nmheWorkspace.acVL[44]*nmheWorkspace.acHx[41] + nmheWorkspace.acVL[52]*nmheWorkspace.acHx[49] + nmheWorkspace.acVL[60]*nmheWorkspace.acHx[57];
nmheWorkspace.acA[218] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHx[2] + nmheWorkspace.acVL[12]*nmheWorkspace.acHx[10] + nmheWorkspace.acVL[20]*nmheWorkspace.acHx[18] + nmheWorkspace.acVL[28]*nmheWorkspace.acHx[26] + nmheWorkspace.acVL[36]*nmheWorkspace.acHx[34] + nmheWorkspace.acVL[44]*nmheWorkspace.acHx[42] + nmheWorkspace.acVL[52]*nmheWorkspace.acHx[50] + nmheWorkspace.acVL[60]*nmheWorkspace.acHx[58];
nmheWorkspace.acA[219] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHx[3] + nmheWorkspace.acVL[12]*nmheWorkspace.acHx[11] + nmheWorkspace.acVL[20]*nmheWorkspace.acHx[19] + nmheWorkspace.acVL[28]*nmheWorkspace.acHx[27] + nmheWorkspace.acVL[36]*nmheWorkspace.acHx[35] + nmheWorkspace.acVL[44]*nmheWorkspace.acHx[43] + nmheWorkspace.acVL[52]*nmheWorkspace.acHx[51] + nmheWorkspace.acVL[60]*nmheWorkspace.acHx[59];
nmheWorkspace.acA[220] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHx[4] + nmheWorkspace.acVL[12]*nmheWorkspace.acHx[12] + nmheWorkspace.acVL[20]*nmheWorkspace.acHx[20] + nmheWorkspace.acVL[28]*nmheWorkspace.acHx[28] + nmheWorkspace.acVL[36]*nmheWorkspace.acHx[36] + nmheWorkspace.acVL[44]*nmheWorkspace.acHx[44] + nmheWorkspace.acVL[52]*nmheWorkspace.acHx[52] + nmheWorkspace.acVL[60]*nmheWorkspace.acHx[60];
nmheWorkspace.acA[221] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHx[5] + nmheWorkspace.acVL[12]*nmheWorkspace.acHx[13] + nmheWorkspace.acVL[20]*nmheWorkspace.acHx[21] + nmheWorkspace.acVL[28]*nmheWorkspace.acHx[29] + nmheWorkspace.acVL[36]*nmheWorkspace.acHx[37] + nmheWorkspace.acVL[44]*nmheWorkspace.acHx[45] + nmheWorkspace.acVL[52]*nmheWorkspace.acHx[53] + nmheWorkspace.acVL[60]*nmheWorkspace.acHx[61];
nmheWorkspace.acA[222] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHx[6] + nmheWorkspace.acVL[12]*nmheWorkspace.acHx[14] + nmheWorkspace.acVL[20]*nmheWorkspace.acHx[22] + nmheWorkspace.acVL[28]*nmheWorkspace.acHx[30] + nmheWorkspace.acVL[36]*nmheWorkspace.acHx[38] + nmheWorkspace.acVL[44]*nmheWorkspace.acHx[46] + nmheWorkspace.acVL[52]*nmheWorkspace.acHx[54] + nmheWorkspace.acVL[60]*nmheWorkspace.acHx[62];
nmheWorkspace.acA[223] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHx[7] + nmheWorkspace.acVL[12]*nmheWorkspace.acHx[15] + nmheWorkspace.acVL[20]*nmheWorkspace.acHx[23] + nmheWorkspace.acVL[28]*nmheWorkspace.acHx[31] + nmheWorkspace.acVL[36]*nmheWorkspace.acHx[39] + nmheWorkspace.acVL[44]*nmheWorkspace.acHx[47] + nmheWorkspace.acVL[52]*nmheWorkspace.acHx[55] + nmheWorkspace.acVL[60]*nmheWorkspace.acHx[63];
nmheWorkspace.acA[234] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHx[0] + nmheWorkspace.acVL[13]*nmheWorkspace.acHx[8] + nmheWorkspace.acVL[21]*nmheWorkspace.acHx[16] + nmheWorkspace.acVL[29]*nmheWorkspace.acHx[24] + nmheWorkspace.acVL[37]*nmheWorkspace.acHx[32] + nmheWorkspace.acVL[45]*nmheWorkspace.acHx[40] + nmheWorkspace.acVL[53]*nmheWorkspace.acHx[48] + nmheWorkspace.acVL[61]*nmheWorkspace.acHx[56];
nmheWorkspace.acA[235] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHx[1] + nmheWorkspace.acVL[13]*nmheWorkspace.acHx[9] + nmheWorkspace.acVL[21]*nmheWorkspace.acHx[17] + nmheWorkspace.acVL[29]*nmheWorkspace.acHx[25] + nmheWorkspace.acVL[37]*nmheWorkspace.acHx[33] + nmheWorkspace.acVL[45]*nmheWorkspace.acHx[41] + nmheWorkspace.acVL[53]*nmheWorkspace.acHx[49] + nmheWorkspace.acVL[61]*nmheWorkspace.acHx[57];
nmheWorkspace.acA[236] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHx[2] + nmheWorkspace.acVL[13]*nmheWorkspace.acHx[10] + nmheWorkspace.acVL[21]*nmheWorkspace.acHx[18] + nmheWorkspace.acVL[29]*nmheWorkspace.acHx[26] + nmheWorkspace.acVL[37]*nmheWorkspace.acHx[34] + nmheWorkspace.acVL[45]*nmheWorkspace.acHx[42] + nmheWorkspace.acVL[53]*nmheWorkspace.acHx[50] + nmheWorkspace.acVL[61]*nmheWorkspace.acHx[58];
nmheWorkspace.acA[237] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHx[3] + nmheWorkspace.acVL[13]*nmheWorkspace.acHx[11] + nmheWorkspace.acVL[21]*nmheWorkspace.acHx[19] + nmheWorkspace.acVL[29]*nmheWorkspace.acHx[27] + nmheWorkspace.acVL[37]*nmheWorkspace.acHx[35] + nmheWorkspace.acVL[45]*nmheWorkspace.acHx[43] + nmheWorkspace.acVL[53]*nmheWorkspace.acHx[51] + nmheWorkspace.acVL[61]*nmheWorkspace.acHx[59];
nmheWorkspace.acA[238] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHx[4] + nmheWorkspace.acVL[13]*nmheWorkspace.acHx[12] + nmheWorkspace.acVL[21]*nmheWorkspace.acHx[20] + nmheWorkspace.acVL[29]*nmheWorkspace.acHx[28] + nmheWorkspace.acVL[37]*nmheWorkspace.acHx[36] + nmheWorkspace.acVL[45]*nmheWorkspace.acHx[44] + nmheWorkspace.acVL[53]*nmheWorkspace.acHx[52] + nmheWorkspace.acVL[61]*nmheWorkspace.acHx[60];
nmheWorkspace.acA[239] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHx[5] + nmheWorkspace.acVL[13]*nmheWorkspace.acHx[13] + nmheWorkspace.acVL[21]*nmheWorkspace.acHx[21] + nmheWorkspace.acVL[29]*nmheWorkspace.acHx[29] + nmheWorkspace.acVL[37]*nmheWorkspace.acHx[37] + nmheWorkspace.acVL[45]*nmheWorkspace.acHx[45] + nmheWorkspace.acVL[53]*nmheWorkspace.acHx[53] + nmheWorkspace.acVL[61]*nmheWorkspace.acHx[61];
nmheWorkspace.acA[240] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHx[6] + nmheWorkspace.acVL[13]*nmheWorkspace.acHx[14] + nmheWorkspace.acVL[21]*nmheWorkspace.acHx[22] + nmheWorkspace.acVL[29]*nmheWorkspace.acHx[30] + nmheWorkspace.acVL[37]*nmheWorkspace.acHx[38] + nmheWorkspace.acVL[45]*nmheWorkspace.acHx[46] + nmheWorkspace.acVL[53]*nmheWorkspace.acHx[54] + nmheWorkspace.acVL[61]*nmheWorkspace.acHx[62];
nmheWorkspace.acA[241] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHx[7] + nmheWorkspace.acVL[13]*nmheWorkspace.acHx[15] + nmheWorkspace.acVL[21]*nmheWorkspace.acHx[23] + nmheWorkspace.acVL[29]*nmheWorkspace.acHx[31] + nmheWorkspace.acVL[37]*nmheWorkspace.acHx[39] + nmheWorkspace.acVL[45]*nmheWorkspace.acHx[47] + nmheWorkspace.acVL[53]*nmheWorkspace.acHx[55] + nmheWorkspace.acVL[61]*nmheWorkspace.acHx[63];
nmheWorkspace.acA[252] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHx[0] + nmheWorkspace.acVL[14]*nmheWorkspace.acHx[8] + nmheWorkspace.acVL[22]*nmheWorkspace.acHx[16] + nmheWorkspace.acVL[30]*nmheWorkspace.acHx[24] + nmheWorkspace.acVL[38]*nmheWorkspace.acHx[32] + nmheWorkspace.acVL[46]*nmheWorkspace.acHx[40] + nmheWorkspace.acVL[54]*nmheWorkspace.acHx[48] + nmheWorkspace.acVL[62]*nmheWorkspace.acHx[56];
nmheWorkspace.acA[253] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHx[1] + nmheWorkspace.acVL[14]*nmheWorkspace.acHx[9] + nmheWorkspace.acVL[22]*nmheWorkspace.acHx[17] + nmheWorkspace.acVL[30]*nmheWorkspace.acHx[25] + nmheWorkspace.acVL[38]*nmheWorkspace.acHx[33] + nmheWorkspace.acVL[46]*nmheWorkspace.acHx[41] + nmheWorkspace.acVL[54]*nmheWorkspace.acHx[49] + nmheWorkspace.acVL[62]*nmheWorkspace.acHx[57];
nmheWorkspace.acA[254] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHx[2] + nmheWorkspace.acVL[14]*nmheWorkspace.acHx[10] + nmheWorkspace.acVL[22]*nmheWorkspace.acHx[18] + nmheWorkspace.acVL[30]*nmheWorkspace.acHx[26] + nmheWorkspace.acVL[38]*nmheWorkspace.acHx[34] + nmheWorkspace.acVL[46]*nmheWorkspace.acHx[42] + nmheWorkspace.acVL[54]*nmheWorkspace.acHx[50] + nmheWorkspace.acVL[62]*nmheWorkspace.acHx[58];
nmheWorkspace.acA[255] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHx[3] + nmheWorkspace.acVL[14]*nmheWorkspace.acHx[11] + nmheWorkspace.acVL[22]*nmheWorkspace.acHx[19] + nmheWorkspace.acVL[30]*nmheWorkspace.acHx[27] + nmheWorkspace.acVL[38]*nmheWorkspace.acHx[35] + nmheWorkspace.acVL[46]*nmheWorkspace.acHx[43] + nmheWorkspace.acVL[54]*nmheWorkspace.acHx[51] + nmheWorkspace.acVL[62]*nmheWorkspace.acHx[59];
nmheWorkspace.acA[256] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHx[4] + nmheWorkspace.acVL[14]*nmheWorkspace.acHx[12] + nmheWorkspace.acVL[22]*nmheWorkspace.acHx[20] + nmheWorkspace.acVL[30]*nmheWorkspace.acHx[28] + nmheWorkspace.acVL[38]*nmheWorkspace.acHx[36] + nmheWorkspace.acVL[46]*nmheWorkspace.acHx[44] + nmheWorkspace.acVL[54]*nmheWorkspace.acHx[52] + nmheWorkspace.acVL[62]*nmheWorkspace.acHx[60];
nmheWorkspace.acA[257] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHx[5] + nmheWorkspace.acVL[14]*nmheWorkspace.acHx[13] + nmheWorkspace.acVL[22]*nmheWorkspace.acHx[21] + nmheWorkspace.acVL[30]*nmheWorkspace.acHx[29] + nmheWorkspace.acVL[38]*nmheWorkspace.acHx[37] + nmheWorkspace.acVL[46]*nmheWorkspace.acHx[45] + nmheWorkspace.acVL[54]*nmheWorkspace.acHx[53] + nmheWorkspace.acVL[62]*nmheWorkspace.acHx[61];
nmheWorkspace.acA[258] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHx[6] + nmheWorkspace.acVL[14]*nmheWorkspace.acHx[14] + nmheWorkspace.acVL[22]*nmheWorkspace.acHx[22] + nmheWorkspace.acVL[30]*nmheWorkspace.acHx[30] + nmheWorkspace.acVL[38]*nmheWorkspace.acHx[38] + nmheWorkspace.acVL[46]*nmheWorkspace.acHx[46] + nmheWorkspace.acVL[54]*nmheWorkspace.acHx[54] + nmheWorkspace.acVL[62]*nmheWorkspace.acHx[62];
nmheWorkspace.acA[259] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHx[7] + nmheWorkspace.acVL[14]*nmheWorkspace.acHx[15] + nmheWorkspace.acVL[22]*nmheWorkspace.acHx[23] + nmheWorkspace.acVL[30]*nmheWorkspace.acHx[31] + nmheWorkspace.acVL[38]*nmheWorkspace.acHx[39] + nmheWorkspace.acVL[46]*nmheWorkspace.acHx[47] + nmheWorkspace.acVL[54]*nmheWorkspace.acHx[55] + nmheWorkspace.acVL[62]*nmheWorkspace.acHx[63];
nmheWorkspace.acA[270] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHx[0] + nmheWorkspace.acVL[15]*nmheWorkspace.acHx[8] + nmheWorkspace.acVL[23]*nmheWorkspace.acHx[16] + nmheWorkspace.acVL[31]*nmheWorkspace.acHx[24] + nmheWorkspace.acVL[39]*nmheWorkspace.acHx[32] + nmheWorkspace.acVL[47]*nmheWorkspace.acHx[40] + nmheWorkspace.acVL[55]*nmheWorkspace.acHx[48] + nmheWorkspace.acVL[63]*nmheWorkspace.acHx[56];
nmheWorkspace.acA[271] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHx[1] + nmheWorkspace.acVL[15]*nmheWorkspace.acHx[9] + nmheWorkspace.acVL[23]*nmheWorkspace.acHx[17] + nmheWorkspace.acVL[31]*nmheWorkspace.acHx[25] + nmheWorkspace.acVL[39]*nmheWorkspace.acHx[33] + nmheWorkspace.acVL[47]*nmheWorkspace.acHx[41] + nmheWorkspace.acVL[55]*nmheWorkspace.acHx[49] + nmheWorkspace.acVL[63]*nmheWorkspace.acHx[57];
nmheWorkspace.acA[272] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHx[2] + nmheWorkspace.acVL[15]*nmheWorkspace.acHx[10] + nmheWorkspace.acVL[23]*nmheWorkspace.acHx[18] + nmheWorkspace.acVL[31]*nmheWorkspace.acHx[26] + nmheWorkspace.acVL[39]*nmheWorkspace.acHx[34] + nmheWorkspace.acVL[47]*nmheWorkspace.acHx[42] + nmheWorkspace.acVL[55]*nmheWorkspace.acHx[50] + nmheWorkspace.acVL[63]*nmheWorkspace.acHx[58];
nmheWorkspace.acA[273] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHx[3] + nmheWorkspace.acVL[15]*nmheWorkspace.acHx[11] + nmheWorkspace.acVL[23]*nmheWorkspace.acHx[19] + nmheWorkspace.acVL[31]*nmheWorkspace.acHx[27] + nmheWorkspace.acVL[39]*nmheWorkspace.acHx[35] + nmheWorkspace.acVL[47]*nmheWorkspace.acHx[43] + nmheWorkspace.acVL[55]*nmheWorkspace.acHx[51] + nmheWorkspace.acVL[63]*nmheWorkspace.acHx[59];
nmheWorkspace.acA[274] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHx[4] + nmheWorkspace.acVL[15]*nmheWorkspace.acHx[12] + nmheWorkspace.acVL[23]*nmheWorkspace.acHx[20] + nmheWorkspace.acVL[31]*nmheWorkspace.acHx[28] + nmheWorkspace.acVL[39]*nmheWorkspace.acHx[36] + nmheWorkspace.acVL[47]*nmheWorkspace.acHx[44] + nmheWorkspace.acVL[55]*nmheWorkspace.acHx[52] + nmheWorkspace.acVL[63]*nmheWorkspace.acHx[60];
nmheWorkspace.acA[275] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHx[5] + nmheWorkspace.acVL[15]*nmheWorkspace.acHx[13] + nmheWorkspace.acVL[23]*nmheWorkspace.acHx[21] + nmheWorkspace.acVL[31]*nmheWorkspace.acHx[29] + nmheWorkspace.acVL[39]*nmheWorkspace.acHx[37] + nmheWorkspace.acVL[47]*nmheWorkspace.acHx[45] + nmheWorkspace.acVL[55]*nmheWorkspace.acHx[53] + nmheWorkspace.acVL[63]*nmheWorkspace.acHx[61];
nmheWorkspace.acA[276] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHx[6] + nmheWorkspace.acVL[15]*nmheWorkspace.acHx[14] + nmheWorkspace.acVL[23]*nmheWorkspace.acHx[22] + nmheWorkspace.acVL[31]*nmheWorkspace.acHx[30] + nmheWorkspace.acVL[39]*nmheWorkspace.acHx[38] + nmheWorkspace.acVL[47]*nmheWorkspace.acHx[46] + nmheWorkspace.acVL[55]*nmheWorkspace.acHx[54] + nmheWorkspace.acVL[63]*nmheWorkspace.acHx[62];
nmheWorkspace.acA[277] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHx[7] + nmheWorkspace.acVL[15]*nmheWorkspace.acHx[15] + nmheWorkspace.acVL[23]*nmheWorkspace.acHx[23] + nmheWorkspace.acVL[31]*nmheWorkspace.acHx[31] + nmheWorkspace.acVL[39]*nmheWorkspace.acHx[39] + nmheWorkspace.acVL[47]*nmheWorkspace.acHx[47] + nmheWorkspace.acVL[55]*nmheWorkspace.acHx[55] + nmheWorkspace.acVL[63]*nmheWorkspace.acHx[63];
nmheWorkspace.acA[152] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHu[0] + nmheWorkspace.acVL[8]*nmheWorkspace.acHu[2] + nmheWorkspace.acVL[16]*nmheWorkspace.acHu[4] + nmheWorkspace.acVL[24]*nmheWorkspace.acHu[6] + nmheWorkspace.acVL[32]*nmheWorkspace.acHu[8] + nmheWorkspace.acVL[40]*nmheWorkspace.acHu[10] + nmheWorkspace.acVL[48]*nmheWorkspace.acHu[12] + nmheWorkspace.acVL[56]*nmheWorkspace.acHu[14];
nmheWorkspace.acA[153] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHu[1] + nmheWorkspace.acVL[8]*nmheWorkspace.acHu[3] + nmheWorkspace.acVL[16]*nmheWorkspace.acHu[5] + nmheWorkspace.acVL[24]*nmheWorkspace.acHu[7] + nmheWorkspace.acVL[32]*nmheWorkspace.acHu[9] + nmheWorkspace.acVL[40]*nmheWorkspace.acHu[11] + nmheWorkspace.acVL[48]*nmheWorkspace.acHu[13] + nmheWorkspace.acVL[56]*nmheWorkspace.acHu[15];
nmheWorkspace.acA[170] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHu[0] + nmheWorkspace.acVL[9]*nmheWorkspace.acHu[2] + nmheWorkspace.acVL[17]*nmheWorkspace.acHu[4] + nmheWorkspace.acVL[25]*nmheWorkspace.acHu[6] + nmheWorkspace.acVL[33]*nmheWorkspace.acHu[8] + nmheWorkspace.acVL[41]*nmheWorkspace.acHu[10] + nmheWorkspace.acVL[49]*nmheWorkspace.acHu[12] + nmheWorkspace.acVL[57]*nmheWorkspace.acHu[14];
nmheWorkspace.acA[171] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHu[1] + nmheWorkspace.acVL[9]*nmheWorkspace.acHu[3] + nmheWorkspace.acVL[17]*nmheWorkspace.acHu[5] + nmheWorkspace.acVL[25]*nmheWorkspace.acHu[7] + nmheWorkspace.acVL[33]*nmheWorkspace.acHu[9] + nmheWorkspace.acVL[41]*nmheWorkspace.acHu[11] + nmheWorkspace.acVL[49]*nmheWorkspace.acHu[13] + nmheWorkspace.acVL[57]*nmheWorkspace.acHu[15];
nmheWorkspace.acA[188] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHu[0] + nmheWorkspace.acVL[10]*nmheWorkspace.acHu[2] + nmheWorkspace.acVL[18]*nmheWorkspace.acHu[4] + nmheWorkspace.acVL[26]*nmheWorkspace.acHu[6] + nmheWorkspace.acVL[34]*nmheWorkspace.acHu[8] + nmheWorkspace.acVL[42]*nmheWorkspace.acHu[10] + nmheWorkspace.acVL[50]*nmheWorkspace.acHu[12] + nmheWorkspace.acVL[58]*nmheWorkspace.acHu[14];
nmheWorkspace.acA[189] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHu[1] + nmheWorkspace.acVL[10]*nmheWorkspace.acHu[3] + nmheWorkspace.acVL[18]*nmheWorkspace.acHu[5] + nmheWorkspace.acVL[26]*nmheWorkspace.acHu[7] + nmheWorkspace.acVL[34]*nmheWorkspace.acHu[9] + nmheWorkspace.acVL[42]*nmheWorkspace.acHu[11] + nmheWorkspace.acVL[50]*nmheWorkspace.acHu[13] + nmheWorkspace.acVL[58]*nmheWorkspace.acHu[15];
nmheWorkspace.acA[206] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHu[0] + nmheWorkspace.acVL[11]*nmheWorkspace.acHu[2] + nmheWorkspace.acVL[19]*nmheWorkspace.acHu[4] + nmheWorkspace.acVL[27]*nmheWorkspace.acHu[6] + nmheWorkspace.acVL[35]*nmheWorkspace.acHu[8] + nmheWorkspace.acVL[43]*nmheWorkspace.acHu[10] + nmheWorkspace.acVL[51]*nmheWorkspace.acHu[12] + nmheWorkspace.acVL[59]*nmheWorkspace.acHu[14];
nmheWorkspace.acA[207] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHu[1] + nmheWorkspace.acVL[11]*nmheWorkspace.acHu[3] + nmheWorkspace.acVL[19]*nmheWorkspace.acHu[5] + nmheWorkspace.acVL[27]*nmheWorkspace.acHu[7] + nmheWorkspace.acVL[35]*nmheWorkspace.acHu[9] + nmheWorkspace.acVL[43]*nmheWorkspace.acHu[11] + nmheWorkspace.acVL[51]*nmheWorkspace.acHu[13] + nmheWorkspace.acVL[59]*nmheWorkspace.acHu[15];
nmheWorkspace.acA[224] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHu[0] + nmheWorkspace.acVL[12]*nmheWorkspace.acHu[2] + nmheWorkspace.acVL[20]*nmheWorkspace.acHu[4] + nmheWorkspace.acVL[28]*nmheWorkspace.acHu[6] + nmheWorkspace.acVL[36]*nmheWorkspace.acHu[8] + nmheWorkspace.acVL[44]*nmheWorkspace.acHu[10] + nmheWorkspace.acVL[52]*nmheWorkspace.acHu[12] + nmheWorkspace.acVL[60]*nmheWorkspace.acHu[14];
nmheWorkspace.acA[225] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHu[1] + nmheWorkspace.acVL[12]*nmheWorkspace.acHu[3] + nmheWorkspace.acVL[20]*nmheWorkspace.acHu[5] + nmheWorkspace.acVL[28]*nmheWorkspace.acHu[7] + nmheWorkspace.acVL[36]*nmheWorkspace.acHu[9] + nmheWorkspace.acVL[44]*nmheWorkspace.acHu[11] + nmheWorkspace.acVL[52]*nmheWorkspace.acHu[13] + nmheWorkspace.acVL[60]*nmheWorkspace.acHu[15];
nmheWorkspace.acA[242] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHu[0] + nmheWorkspace.acVL[13]*nmheWorkspace.acHu[2] + nmheWorkspace.acVL[21]*nmheWorkspace.acHu[4] + nmheWorkspace.acVL[29]*nmheWorkspace.acHu[6] + nmheWorkspace.acVL[37]*nmheWorkspace.acHu[8] + nmheWorkspace.acVL[45]*nmheWorkspace.acHu[10] + nmheWorkspace.acVL[53]*nmheWorkspace.acHu[12] + nmheWorkspace.acVL[61]*nmheWorkspace.acHu[14];
nmheWorkspace.acA[243] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHu[1] + nmheWorkspace.acVL[13]*nmheWorkspace.acHu[3] + nmheWorkspace.acVL[21]*nmheWorkspace.acHu[5] + nmheWorkspace.acVL[29]*nmheWorkspace.acHu[7] + nmheWorkspace.acVL[37]*nmheWorkspace.acHu[9] + nmheWorkspace.acVL[45]*nmheWorkspace.acHu[11] + nmheWorkspace.acVL[53]*nmheWorkspace.acHu[13] + nmheWorkspace.acVL[61]*nmheWorkspace.acHu[15];
nmheWorkspace.acA[260] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHu[0] + nmheWorkspace.acVL[14]*nmheWorkspace.acHu[2] + nmheWorkspace.acVL[22]*nmheWorkspace.acHu[4] + nmheWorkspace.acVL[30]*nmheWorkspace.acHu[6] + nmheWorkspace.acVL[38]*nmheWorkspace.acHu[8] + nmheWorkspace.acVL[46]*nmheWorkspace.acHu[10] + nmheWorkspace.acVL[54]*nmheWorkspace.acHu[12] + nmheWorkspace.acVL[62]*nmheWorkspace.acHu[14];
nmheWorkspace.acA[261] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHu[1] + nmheWorkspace.acVL[14]*nmheWorkspace.acHu[3] + nmheWorkspace.acVL[22]*nmheWorkspace.acHu[5] + nmheWorkspace.acVL[30]*nmheWorkspace.acHu[7] + nmheWorkspace.acVL[38]*nmheWorkspace.acHu[9] + nmheWorkspace.acVL[46]*nmheWorkspace.acHu[11] + nmheWorkspace.acVL[54]*nmheWorkspace.acHu[13] + nmheWorkspace.acVL[62]*nmheWorkspace.acHu[15];
nmheWorkspace.acA[278] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHu[0] + nmheWorkspace.acVL[15]*nmheWorkspace.acHu[2] + nmheWorkspace.acVL[23]*nmheWorkspace.acHu[4] + nmheWorkspace.acVL[31]*nmheWorkspace.acHu[6] + nmheWorkspace.acVL[39]*nmheWorkspace.acHu[8] + nmheWorkspace.acVL[47]*nmheWorkspace.acHu[10] + nmheWorkspace.acVL[55]*nmheWorkspace.acHu[12] + nmheWorkspace.acVL[63]*nmheWorkspace.acHu[14];
nmheWorkspace.acA[279] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHu[1] + nmheWorkspace.acVL[15]*nmheWorkspace.acHu[3] + nmheWorkspace.acVL[23]*nmheWorkspace.acHu[5] + nmheWorkspace.acVL[31]*nmheWorkspace.acHu[7] + nmheWorkspace.acVL[39]*nmheWorkspace.acHu[9] + nmheWorkspace.acVL[47]*nmheWorkspace.acHu[11] + nmheWorkspace.acVL[55]*nmheWorkspace.acHu[13] + nmheWorkspace.acVL[63]*nmheWorkspace.acHu[15];
nmheWorkspace.acA[288] -= + nmheVariables.WL[0]*nmheWorkspace.acXx[0] + nmheVariables.WL[8]*nmheWorkspace.acXx[8] + nmheVariables.WL[16]*nmheWorkspace.acXx[16] + nmheVariables.WL[24]*nmheWorkspace.acXx[24] + nmheVariables.WL[32]*nmheWorkspace.acXx[32] + nmheVariables.WL[40]*nmheWorkspace.acXx[40] + nmheVariables.WL[48]*nmheWorkspace.acXx[48] + nmheVariables.WL[56]*nmheWorkspace.acXx[56];
nmheWorkspace.acA[289] -= + nmheVariables.WL[0]*nmheWorkspace.acXx[1] + nmheVariables.WL[8]*nmheWorkspace.acXx[9] + nmheVariables.WL[16]*nmheWorkspace.acXx[17] + nmheVariables.WL[24]*nmheWorkspace.acXx[25] + nmheVariables.WL[32]*nmheWorkspace.acXx[33] + nmheVariables.WL[40]*nmheWorkspace.acXx[41] + nmheVariables.WL[48]*nmheWorkspace.acXx[49] + nmheVariables.WL[56]*nmheWorkspace.acXx[57];
nmheWorkspace.acA[290] -= + nmheVariables.WL[0]*nmheWorkspace.acXx[2] + nmheVariables.WL[8]*nmheWorkspace.acXx[10] + nmheVariables.WL[16]*nmheWorkspace.acXx[18] + nmheVariables.WL[24]*nmheWorkspace.acXx[26] + nmheVariables.WL[32]*nmheWorkspace.acXx[34] + nmheVariables.WL[40]*nmheWorkspace.acXx[42] + nmheVariables.WL[48]*nmheWorkspace.acXx[50] + nmheVariables.WL[56]*nmheWorkspace.acXx[58];
nmheWorkspace.acA[291] -= + nmheVariables.WL[0]*nmheWorkspace.acXx[3] + nmheVariables.WL[8]*nmheWorkspace.acXx[11] + nmheVariables.WL[16]*nmheWorkspace.acXx[19] + nmheVariables.WL[24]*nmheWorkspace.acXx[27] + nmheVariables.WL[32]*nmheWorkspace.acXx[35] + nmheVariables.WL[40]*nmheWorkspace.acXx[43] + nmheVariables.WL[48]*nmheWorkspace.acXx[51] + nmheVariables.WL[56]*nmheWorkspace.acXx[59];
nmheWorkspace.acA[292] -= + nmheVariables.WL[0]*nmheWorkspace.acXx[4] + nmheVariables.WL[8]*nmheWorkspace.acXx[12] + nmheVariables.WL[16]*nmheWorkspace.acXx[20] + nmheVariables.WL[24]*nmheWorkspace.acXx[28] + nmheVariables.WL[32]*nmheWorkspace.acXx[36] + nmheVariables.WL[40]*nmheWorkspace.acXx[44] + nmheVariables.WL[48]*nmheWorkspace.acXx[52] + nmheVariables.WL[56]*nmheWorkspace.acXx[60];
nmheWorkspace.acA[293] -= + nmheVariables.WL[0]*nmheWorkspace.acXx[5] + nmheVariables.WL[8]*nmheWorkspace.acXx[13] + nmheVariables.WL[16]*nmheWorkspace.acXx[21] + nmheVariables.WL[24]*nmheWorkspace.acXx[29] + nmheVariables.WL[32]*nmheWorkspace.acXx[37] + nmheVariables.WL[40]*nmheWorkspace.acXx[45] + nmheVariables.WL[48]*nmheWorkspace.acXx[53] + nmheVariables.WL[56]*nmheWorkspace.acXx[61];
nmheWorkspace.acA[294] -= + nmheVariables.WL[0]*nmheWorkspace.acXx[6] + nmheVariables.WL[8]*nmheWorkspace.acXx[14] + nmheVariables.WL[16]*nmheWorkspace.acXx[22] + nmheVariables.WL[24]*nmheWorkspace.acXx[30] + nmheVariables.WL[32]*nmheWorkspace.acXx[38] + nmheVariables.WL[40]*nmheWorkspace.acXx[46] + nmheVariables.WL[48]*nmheWorkspace.acXx[54] + nmheVariables.WL[56]*nmheWorkspace.acXx[62];
nmheWorkspace.acA[295] -= + nmheVariables.WL[0]*nmheWorkspace.acXx[7] + nmheVariables.WL[8]*nmheWorkspace.acXx[15] + nmheVariables.WL[16]*nmheWorkspace.acXx[23] + nmheVariables.WL[24]*nmheWorkspace.acXx[31] + nmheVariables.WL[32]*nmheWorkspace.acXx[39] + nmheVariables.WL[40]*nmheWorkspace.acXx[47] + nmheVariables.WL[48]*nmheWorkspace.acXx[55] + nmheVariables.WL[56]*nmheWorkspace.acXx[63];
nmheWorkspace.acA[306] -= + nmheVariables.WL[1]*nmheWorkspace.acXx[0] + nmheVariables.WL[9]*nmheWorkspace.acXx[8] + nmheVariables.WL[17]*nmheWorkspace.acXx[16] + nmheVariables.WL[25]*nmheWorkspace.acXx[24] + nmheVariables.WL[33]*nmheWorkspace.acXx[32] + nmheVariables.WL[41]*nmheWorkspace.acXx[40] + nmheVariables.WL[49]*nmheWorkspace.acXx[48] + nmheVariables.WL[57]*nmheWorkspace.acXx[56];
nmheWorkspace.acA[307] -= + nmheVariables.WL[1]*nmheWorkspace.acXx[1] + nmheVariables.WL[9]*nmheWorkspace.acXx[9] + nmheVariables.WL[17]*nmheWorkspace.acXx[17] + nmheVariables.WL[25]*nmheWorkspace.acXx[25] + nmheVariables.WL[33]*nmheWorkspace.acXx[33] + nmheVariables.WL[41]*nmheWorkspace.acXx[41] + nmheVariables.WL[49]*nmheWorkspace.acXx[49] + nmheVariables.WL[57]*nmheWorkspace.acXx[57];
nmheWorkspace.acA[308] -= + nmheVariables.WL[1]*nmheWorkspace.acXx[2] + nmheVariables.WL[9]*nmheWorkspace.acXx[10] + nmheVariables.WL[17]*nmheWorkspace.acXx[18] + nmheVariables.WL[25]*nmheWorkspace.acXx[26] + nmheVariables.WL[33]*nmheWorkspace.acXx[34] + nmheVariables.WL[41]*nmheWorkspace.acXx[42] + nmheVariables.WL[49]*nmheWorkspace.acXx[50] + nmheVariables.WL[57]*nmheWorkspace.acXx[58];
nmheWorkspace.acA[309] -= + nmheVariables.WL[1]*nmheWorkspace.acXx[3] + nmheVariables.WL[9]*nmheWorkspace.acXx[11] + nmheVariables.WL[17]*nmheWorkspace.acXx[19] + nmheVariables.WL[25]*nmheWorkspace.acXx[27] + nmheVariables.WL[33]*nmheWorkspace.acXx[35] + nmheVariables.WL[41]*nmheWorkspace.acXx[43] + nmheVariables.WL[49]*nmheWorkspace.acXx[51] + nmheVariables.WL[57]*nmheWorkspace.acXx[59];
nmheWorkspace.acA[310] -= + nmheVariables.WL[1]*nmheWorkspace.acXx[4] + nmheVariables.WL[9]*nmheWorkspace.acXx[12] + nmheVariables.WL[17]*nmheWorkspace.acXx[20] + nmheVariables.WL[25]*nmheWorkspace.acXx[28] + nmheVariables.WL[33]*nmheWorkspace.acXx[36] + nmheVariables.WL[41]*nmheWorkspace.acXx[44] + nmheVariables.WL[49]*nmheWorkspace.acXx[52] + nmheVariables.WL[57]*nmheWorkspace.acXx[60];
nmheWorkspace.acA[311] -= + nmheVariables.WL[1]*nmheWorkspace.acXx[5] + nmheVariables.WL[9]*nmheWorkspace.acXx[13] + nmheVariables.WL[17]*nmheWorkspace.acXx[21] + nmheVariables.WL[25]*nmheWorkspace.acXx[29] + nmheVariables.WL[33]*nmheWorkspace.acXx[37] + nmheVariables.WL[41]*nmheWorkspace.acXx[45] + nmheVariables.WL[49]*nmheWorkspace.acXx[53] + nmheVariables.WL[57]*nmheWorkspace.acXx[61];
nmheWorkspace.acA[312] -= + nmheVariables.WL[1]*nmheWorkspace.acXx[6] + nmheVariables.WL[9]*nmheWorkspace.acXx[14] + nmheVariables.WL[17]*nmheWorkspace.acXx[22] + nmheVariables.WL[25]*nmheWorkspace.acXx[30] + nmheVariables.WL[33]*nmheWorkspace.acXx[38] + nmheVariables.WL[41]*nmheWorkspace.acXx[46] + nmheVariables.WL[49]*nmheWorkspace.acXx[54] + nmheVariables.WL[57]*nmheWorkspace.acXx[62];
nmheWorkspace.acA[313] -= + nmheVariables.WL[1]*nmheWorkspace.acXx[7] + nmheVariables.WL[9]*nmheWorkspace.acXx[15] + nmheVariables.WL[17]*nmheWorkspace.acXx[23] + nmheVariables.WL[25]*nmheWorkspace.acXx[31] + nmheVariables.WL[33]*nmheWorkspace.acXx[39] + nmheVariables.WL[41]*nmheWorkspace.acXx[47] + nmheVariables.WL[49]*nmheWorkspace.acXx[55] + nmheVariables.WL[57]*nmheWorkspace.acXx[63];
nmheWorkspace.acA[324] -= + nmheVariables.WL[2]*nmheWorkspace.acXx[0] + nmheVariables.WL[10]*nmheWorkspace.acXx[8] + nmheVariables.WL[18]*nmheWorkspace.acXx[16] + nmheVariables.WL[26]*nmheWorkspace.acXx[24] + nmheVariables.WL[34]*nmheWorkspace.acXx[32] + nmheVariables.WL[42]*nmheWorkspace.acXx[40] + nmheVariables.WL[50]*nmheWorkspace.acXx[48] + nmheVariables.WL[58]*nmheWorkspace.acXx[56];
nmheWorkspace.acA[325] -= + nmheVariables.WL[2]*nmheWorkspace.acXx[1] + nmheVariables.WL[10]*nmheWorkspace.acXx[9] + nmheVariables.WL[18]*nmheWorkspace.acXx[17] + nmheVariables.WL[26]*nmheWorkspace.acXx[25] + nmheVariables.WL[34]*nmheWorkspace.acXx[33] + nmheVariables.WL[42]*nmheWorkspace.acXx[41] + nmheVariables.WL[50]*nmheWorkspace.acXx[49] + nmheVariables.WL[58]*nmheWorkspace.acXx[57];
nmheWorkspace.acA[326] -= + nmheVariables.WL[2]*nmheWorkspace.acXx[2] + nmheVariables.WL[10]*nmheWorkspace.acXx[10] + nmheVariables.WL[18]*nmheWorkspace.acXx[18] + nmheVariables.WL[26]*nmheWorkspace.acXx[26] + nmheVariables.WL[34]*nmheWorkspace.acXx[34] + nmheVariables.WL[42]*nmheWorkspace.acXx[42] + nmheVariables.WL[50]*nmheWorkspace.acXx[50] + nmheVariables.WL[58]*nmheWorkspace.acXx[58];
nmheWorkspace.acA[327] -= + nmheVariables.WL[2]*nmheWorkspace.acXx[3] + nmheVariables.WL[10]*nmheWorkspace.acXx[11] + nmheVariables.WL[18]*nmheWorkspace.acXx[19] + nmheVariables.WL[26]*nmheWorkspace.acXx[27] + nmheVariables.WL[34]*nmheWorkspace.acXx[35] + nmheVariables.WL[42]*nmheWorkspace.acXx[43] + nmheVariables.WL[50]*nmheWorkspace.acXx[51] + nmheVariables.WL[58]*nmheWorkspace.acXx[59];
nmheWorkspace.acA[328] -= + nmheVariables.WL[2]*nmheWorkspace.acXx[4] + nmheVariables.WL[10]*nmheWorkspace.acXx[12] + nmheVariables.WL[18]*nmheWorkspace.acXx[20] + nmheVariables.WL[26]*nmheWorkspace.acXx[28] + nmheVariables.WL[34]*nmheWorkspace.acXx[36] + nmheVariables.WL[42]*nmheWorkspace.acXx[44] + nmheVariables.WL[50]*nmheWorkspace.acXx[52] + nmheVariables.WL[58]*nmheWorkspace.acXx[60];
nmheWorkspace.acA[329] -= + nmheVariables.WL[2]*nmheWorkspace.acXx[5] + nmheVariables.WL[10]*nmheWorkspace.acXx[13] + nmheVariables.WL[18]*nmheWorkspace.acXx[21] + nmheVariables.WL[26]*nmheWorkspace.acXx[29] + nmheVariables.WL[34]*nmheWorkspace.acXx[37] + nmheVariables.WL[42]*nmheWorkspace.acXx[45] + nmheVariables.WL[50]*nmheWorkspace.acXx[53] + nmheVariables.WL[58]*nmheWorkspace.acXx[61];
nmheWorkspace.acA[330] -= + nmheVariables.WL[2]*nmheWorkspace.acXx[6] + nmheVariables.WL[10]*nmheWorkspace.acXx[14] + nmheVariables.WL[18]*nmheWorkspace.acXx[22] + nmheVariables.WL[26]*nmheWorkspace.acXx[30] + nmheVariables.WL[34]*nmheWorkspace.acXx[38] + nmheVariables.WL[42]*nmheWorkspace.acXx[46] + nmheVariables.WL[50]*nmheWorkspace.acXx[54] + nmheVariables.WL[58]*nmheWorkspace.acXx[62];
nmheWorkspace.acA[331] -= + nmheVariables.WL[2]*nmheWorkspace.acXx[7] + nmheVariables.WL[10]*nmheWorkspace.acXx[15] + nmheVariables.WL[18]*nmheWorkspace.acXx[23] + nmheVariables.WL[26]*nmheWorkspace.acXx[31] + nmheVariables.WL[34]*nmheWorkspace.acXx[39] + nmheVariables.WL[42]*nmheWorkspace.acXx[47] + nmheVariables.WL[50]*nmheWorkspace.acXx[55] + nmheVariables.WL[58]*nmheWorkspace.acXx[63];
nmheWorkspace.acA[342] -= + nmheVariables.WL[3]*nmheWorkspace.acXx[0] + nmheVariables.WL[11]*nmheWorkspace.acXx[8] + nmheVariables.WL[19]*nmheWorkspace.acXx[16] + nmheVariables.WL[27]*nmheWorkspace.acXx[24] + nmheVariables.WL[35]*nmheWorkspace.acXx[32] + nmheVariables.WL[43]*nmheWorkspace.acXx[40] + nmheVariables.WL[51]*nmheWorkspace.acXx[48] + nmheVariables.WL[59]*nmheWorkspace.acXx[56];
nmheWorkspace.acA[343] -= + nmheVariables.WL[3]*nmheWorkspace.acXx[1] + nmheVariables.WL[11]*nmheWorkspace.acXx[9] + nmheVariables.WL[19]*nmheWorkspace.acXx[17] + nmheVariables.WL[27]*nmheWorkspace.acXx[25] + nmheVariables.WL[35]*nmheWorkspace.acXx[33] + nmheVariables.WL[43]*nmheWorkspace.acXx[41] + nmheVariables.WL[51]*nmheWorkspace.acXx[49] + nmheVariables.WL[59]*nmheWorkspace.acXx[57];
nmheWorkspace.acA[344] -= + nmheVariables.WL[3]*nmheWorkspace.acXx[2] + nmheVariables.WL[11]*nmheWorkspace.acXx[10] + nmheVariables.WL[19]*nmheWorkspace.acXx[18] + nmheVariables.WL[27]*nmheWorkspace.acXx[26] + nmheVariables.WL[35]*nmheWorkspace.acXx[34] + nmheVariables.WL[43]*nmheWorkspace.acXx[42] + nmheVariables.WL[51]*nmheWorkspace.acXx[50] + nmheVariables.WL[59]*nmheWorkspace.acXx[58];
nmheWorkspace.acA[345] -= + nmheVariables.WL[3]*nmheWorkspace.acXx[3] + nmheVariables.WL[11]*nmheWorkspace.acXx[11] + nmheVariables.WL[19]*nmheWorkspace.acXx[19] + nmheVariables.WL[27]*nmheWorkspace.acXx[27] + nmheVariables.WL[35]*nmheWorkspace.acXx[35] + nmheVariables.WL[43]*nmheWorkspace.acXx[43] + nmheVariables.WL[51]*nmheWorkspace.acXx[51] + nmheVariables.WL[59]*nmheWorkspace.acXx[59];
nmheWorkspace.acA[346] -= + nmheVariables.WL[3]*nmheWorkspace.acXx[4] + nmheVariables.WL[11]*nmheWorkspace.acXx[12] + nmheVariables.WL[19]*nmheWorkspace.acXx[20] + nmheVariables.WL[27]*nmheWorkspace.acXx[28] + nmheVariables.WL[35]*nmheWorkspace.acXx[36] + nmheVariables.WL[43]*nmheWorkspace.acXx[44] + nmheVariables.WL[51]*nmheWorkspace.acXx[52] + nmheVariables.WL[59]*nmheWorkspace.acXx[60];
nmheWorkspace.acA[347] -= + nmheVariables.WL[3]*nmheWorkspace.acXx[5] + nmheVariables.WL[11]*nmheWorkspace.acXx[13] + nmheVariables.WL[19]*nmheWorkspace.acXx[21] + nmheVariables.WL[27]*nmheWorkspace.acXx[29] + nmheVariables.WL[35]*nmheWorkspace.acXx[37] + nmheVariables.WL[43]*nmheWorkspace.acXx[45] + nmheVariables.WL[51]*nmheWorkspace.acXx[53] + nmheVariables.WL[59]*nmheWorkspace.acXx[61];
nmheWorkspace.acA[348] -= + nmheVariables.WL[3]*nmheWorkspace.acXx[6] + nmheVariables.WL[11]*nmheWorkspace.acXx[14] + nmheVariables.WL[19]*nmheWorkspace.acXx[22] + nmheVariables.WL[27]*nmheWorkspace.acXx[30] + nmheVariables.WL[35]*nmheWorkspace.acXx[38] + nmheVariables.WL[43]*nmheWorkspace.acXx[46] + nmheVariables.WL[51]*nmheWorkspace.acXx[54] + nmheVariables.WL[59]*nmheWorkspace.acXx[62];
nmheWorkspace.acA[349] -= + nmheVariables.WL[3]*nmheWorkspace.acXx[7] + nmheVariables.WL[11]*nmheWorkspace.acXx[15] + nmheVariables.WL[19]*nmheWorkspace.acXx[23] + nmheVariables.WL[27]*nmheWorkspace.acXx[31] + nmheVariables.WL[35]*nmheWorkspace.acXx[39] + nmheVariables.WL[43]*nmheWorkspace.acXx[47] + nmheVariables.WL[51]*nmheWorkspace.acXx[55] + nmheVariables.WL[59]*nmheWorkspace.acXx[63];
nmheWorkspace.acA[360] -= + nmheVariables.WL[4]*nmheWorkspace.acXx[0] + nmheVariables.WL[12]*nmheWorkspace.acXx[8] + nmheVariables.WL[20]*nmheWorkspace.acXx[16] + nmheVariables.WL[28]*nmheWorkspace.acXx[24] + nmheVariables.WL[36]*nmheWorkspace.acXx[32] + nmheVariables.WL[44]*nmheWorkspace.acXx[40] + nmheVariables.WL[52]*nmheWorkspace.acXx[48] + nmheVariables.WL[60]*nmheWorkspace.acXx[56];
nmheWorkspace.acA[361] -= + nmheVariables.WL[4]*nmheWorkspace.acXx[1] + nmheVariables.WL[12]*nmheWorkspace.acXx[9] + nmheVariables.WL[20]*nmheWorkspace.acXx[17] + nmheVariables.WL[28]*nmheWorkspace.acXx[25] + nmheVariables.WL[36]*nmheWorkspace.acXx[33] + nmheVariables.WL[44]*nmheWorkspace.acXx[41] + nmheVariables.WL[52]*nmheWorkspace.acXx[49] + nmheVariables.WL[60]*nmheWorkspace.acXx[57];
nmheWorkspace.acA[362] -= + nmheVariables.WL[4]*nmheWorkspace.acXx[2] + nmheVariables.WL[12]*nmheWorkspace.acXx[10] + nmheVariables.WL[20]*nmheWorkspace.acXx[18] + nmheVariables.WL[28]*nmheWorkspace.acXx[26] + nmheVariables.WL[36]*nmheWorkspace.acXx[34] + nmheVariables.WL[44]*nmheWorkspace.acXx[42] + nmheVariables.WL[52]*nmheWorkspace.acXx[50] + nmheVariables.WL[60]*nmheWorkspace.acXx[58];
nmheWorkspace.acA[363] -= + nmheVariables.WL[4]*nmheWorkspace.acXx[3] + nmheVariables.WL[12]*nmheWorkspace.acXx[11] + nmheVariables.WL[20]*nmheWorkspace.acXx[19] + nmheVariables.WL[28]*nmheWorkspace.acXx[27] + nmheVariables.WL[36]*nmheWorkspace.acXx[35] + nmheVariables.WL[44]*nmheWorkspace.acXx[43] + nmheVariables.WL[52]*nmheWorkspace.acXx[51] + nmheVariables.WL[60]*nmheWorkspace.acXx[59];
nmheWorkspace.acA[364] -= + nmheVariables.WL[4]*nmheWorkspace.acXx[4] + nmheVariables.WL[12]*nmheWorkspace.acXx[12] + nmheVariables.WL[20]*nmheWorkspace.acXx[20] + nmheVariables.WL[28]*nmheWorkspace.acXx[28] + nmheVariables.WL[36]*nmheWorkspace.acXx[36] + nmheVariables.WL[44]*nmheWorkspace.acXx[44] + nmheVariables.WL[52]*nmheWorkspace.acXx[52] + nmheVariables.WL[60]*nmheWorkspace.acXx[60];
nmheWorkspace.acA[365] -= + nmheVariables.WL[4]*nmheWorkspace.acXx[5] + nmheVariables.WL[12]*nmheWorkspace.acXx[13] + nmheVariables.WL[20]*nmheWorkspace.acXx[21] + nmheVariables.WL[28]*nmheWorkspace.acXx[29] + nmheVariables.WL[36]*nmheWorkspace.acXx[37] + nmheVariables.WL[44]*nmheWorkspace.acXx[45] + nmheVariables.WL[52]*nmheWorkspace.acXx[53] + nmheVariables.WL[60]*nmheWorkspace.acXx[61];
nmheWorkspace.acA[366] -= + nmheVariables.WL[4]*nmheWorkspace.acXx[6] + nmheVariables.WL[12]*nmheWorkspace.acXx[14] + nmheVariables.WL[20]*nmheWorkspace.acXx[22] + nmheVariables.WL[28]*nmheWorkspace.acXx[30] + nmheVariables.WL[36]*nmheWorkspace.acXx[38] + nmheVariables.WL[44]*nmheWorkspace.acXx[46] + nmheVariables.WL[52]*nmheWorkspace.acXx[54] + nmheVariables.WL[60]*nmheWorkspace.acXx[62];
nmheWorkspace.acA[367] -= + nmheVariables.WL[4]*nmheWorkspace.acXx[7] + nmheVariables.WL[12]*nmheWorkspace.acXx[15] + nmheVariables.WL[20]*nmheWorkspace.acXx[23] + nmheVariables.WL[28]*nmheWorkspace.acXx[31] + nmheVariables.WL[36]*nmheWorkspace.acXx[39] + nmheVariables.WL[44]*nmheWorkspace.acXx[47] + nmheVariables.WL[52]*nmheWorkspace.acXx[55] + nmheVariables.WL[60]*nmheWorkspace.acXx[63];
nmheWorkspace.acA[378] -= + nmheVariables.WL[5]*nmheWorkspace.acXx[0] + nmheVariables.WL[13]*nmheWorkspace.acXx[8] + nmheVariables.WL[21]*nmheWorkspace.acXx[16] + nmheVariables.WL[29]*nmheWorkspace.acXx[24] + nmheVariables.WL[37]*nmheWorkspace.acXx[32] + nmheVariables.WL[45]*nmheWorkspace.acXx[40] + nmheVariables.WL[53]*nmheWorkspace.acXx[48] + nmheVariables.WL[61]*nmheWorkspace.acXx[56];
nmheWorkspace.acA[379] -= + nmheVariables.WL[5]*nmheWorkspace.acXx[1] + nmheVariables.WL[13]*nmheWorkspace.acXx[9] + nmheVariables.WL[21]*nmheWorkspace.acXx[17] + nmheVariables.WL[29]*nmheWorkspace.acXx[25] + nmheVariables.WL[37]*nmheWorkspace.acXx[33] + nmheVariables.WL[45]*nmheWorkspace.acXx[41] + nmheVariables.WL[53]*nmheWorkspace.acXx[49] + nmheVariables.WL[61]*nmheWorkspace.acXx[57];
nmheWorkspace.acA[380] -= + nmheVariables.WL[5]*nmheWorkspace.acXx[2] + nmheVariables.WL[13]*nmheWorkspace.acXx[10] + nmheVariables.WL[21]*nmheWorkspace.acXx[18] + nmheVariables.WL[29]*nmheWorkspace.acXx[26] + nmheVariables.WL[37]*nmheWorkspace.acXx[34] + nmheVariables.WL[45]*nmheWorkspace.acXx[42] + nmheVariables.WL[53]*nmheWorkspace.acXx[50] + nmheVariables.WL[61]*nmheWorkspace.acXx[58];
nmheWorkspace.acA[381] -= + nmheVariables.WL[5]*nmheWorkspace.acXx[3] + nmheVariables.WL[13]*nmheWorkspace.acXx[11] + nmheVariables.WL[21]*nmheWorkspace.acXx[19] + nmheVariables.WL[29]*nmheWorkspace.acXx[27] + nmheVariables.WL[37]*nmheWorkspace.acXx[35] + nmheVariables.WL[45]*nmheWorkspace.acXx[43] + nmheVariables.WL[53]*nmheWorkspace.acXx[51] + nmheVariables.WL[61]*nmheWorkspace.acXx[59];
nmheWorkspace.acA[382] -= + nmheVariables.WL[5]*nmheWorkspace.acXx[4] + nmheVariables.WL[13]*nmheWorkspace.acXx[12] + nmheVariables.WL[21]*nmheWorkspace.acXx[20] + nmheVariables.WL[29]*nmheWorkspace.acXx[28] + nmheVariables.WL[37]*nmheWorkspace.acXx[36] + nmheVariables.WL[45]*nmheWorkspace.acXx[44] + nmheVariables.WL[53]*nmheWorkspace.acXx[52] + nmheVariables.WL[61]*nmheWorkspace.acXx[60];
nmheWorkspace.acA[383] -= + nmheVariables.WL[5]*nmheWorkspace.acXx[5] + nmheVariables.WL[13]*nmheWorkspace.acXx[13] + nmheVariables.WL[21]*nmheWorkspace.acXx[21] + nmheVariables.WL[29]*nmheWorkspace.acXx[29] + nmheVariables.WL[37]*nmheWorkspace.acXx[37] + nmheVariables.WL[45]*nmheWorkspace.acXx[45] + nmheVariables.WL[53]*nmheWorkspace.acXx[53] + nmheVariables.WL[61]*nmheWorkspace.acXx[61];
nmheWorkspace.acA[384] -= + nmheVariables.WL[5]*nmheWorkspace.acXx[6] + nmheVariables.WL[13]*nmheWorkspace.acXx[14] + nmheVariables.WL[21]*nmheWorkspace.acXx[22] + nmheVariables.WL[29]*nmheWorkspace.acXx[30] + nmheVariables.WL[37]*nmheWorkspace.acXx[38] + nmheVariables.WL[45]*nmheWorkspace.acXx[46] + nmheVariables.WL[53]*nmheWorkspace.acXx[54] + nmheVariables.WL[61]*nmheWorkspace.acXx[62];
nmheWorkspace.acA[385] -= + nmheVariables.WL[5]*nmheWorkspace.acXx[7] + nmheVariables.WL[13]*nmheWorkspace.acXx[15] + nmheVariables.WL[21]*nmheWorkspace.acXx[23] + nmheVariables.WL[29]*nmheWorkspace.acXx[31] + nmheVariables.WL[37]*nmheWorkspace.acXx[39] + nmheVariables.WL[45]*nmheWorkspace.acXx[47] + nmheVariables.WL[53]*nmheWorkspace.acXx[55] + nmheVariables.WL[61]*nmheWorkspace.acXx[63];
nmheWorkspace.acA[396] -= + nmheVariables.WL[6]*nmheWorkspace.acXx[0] + nmheVariables.WL[14]*nmheWorkspace.acXx[8] + nmheVariables.WL[22]*nmheWorkspace.acXx[16] + nmheVariables.WL[30]*nmheWorkspace.acXx[24] + nmheVariables.WL[38]*nmheWorkspace.acXx[32] + nmheVariables.WL[46]*nmheWorkspace.acXx[40] + nmheVariables.WL[54]*nmheWorkspace.acXx[48] + nmheVariables.WL[62]*nmheWorkspace.acXx[56];
nmheWorkspace.acA[397] -= + nmheVariables.WL[6]*nmheWorkspace.acXx[1] + nmheVariables.WL[14]*nmheWorkspace.acXx[9] + nmheVariables.WL[22]*nmheWorkspace.acXx[17] + nmheVariables.WL[30]*nmheWorkspace.acXx[25] + nmheVariables.WL[38]*nmheWorkspace.acXx[33] + nmheVariables.WL[46]*nmheWorkspace.acXx[41] + nmheVariables.WL[54]*nmheWorkspace.acXx[49] + nmheVariables.WL[62]*nmheWorkspace.acXx[57];
nmheWorkspace.acA[398] -= + nmheVariables.WL[6]*nmheWorkspace.acXx[2] + nmheVariables.WL[14]*nmheWorkspace.acXx[10] + nmheVariables.WL[22]*nmheWorkspace.acXx[18] + nmheVariables.WL[30]*nmheWorkspace.acXx[26] + nmheVariables.WL[38]*nmheWorkspace.acXx[34] + nmheVariables.WL[46]*nmheWorkspace.acXx[42] + nmheVariables.WL[54]*nmheWorkspace.acXx[50] + nmheVariables.WL[62]*nmheWorkspace.acXx[58];
nmheWorkspace.acA[399] -= + nmheVariables.WL[6]*nmheWorkspace.acXx[3] + nmheVariables.WL[14]*nmheWorkspace.acXx[11] + nmheVariables.WL[22]*nmheWorkspace.acXx[19] + nmheVariables.WL[30]*nmheWorkspace.acXx[27] + nmheVariables.WL[38]*nmheWorkspace.acXx[35] + nmheVariables.WL[46]*nmheWorkspace.acXx[43] + nmheVariables.WL[54]*nmheWorkspace.acXx[51] + nmheVariables.WL[62]*nmheWorkspace.acXx[59];
nmheWorkspace.acA[400] -= + nmheVariables.WL[6]*nmheWorkspace.acXx[4] + nmheVariables.WL[14]*nmheWorkspace.acXx[12] + nmheVariables.WL[22]*nmheWorkspace.acXx[20] + nmheVariables.WL[30]*nmheWorkspace.acXx[28] + nmheVariables.WL[38]*nmheWorkspace.acXx[36] + nmheVariables.WL[46]*nmheWorkspace.acXx[44] + nmheVariables.WL[54]*nmheWorkspace.acXx[52] + nmheVariables.WL[62]*nmheWorkspace.acXx[60];
nmheWorkspace.acA[401] -= + nmheVariables.WL[6]*nmheWorkspace.acXx[5] + nmheVariables.WL[14]*nmheWorkspace.acXx[13] + nmheVariables.WL[22]*nmheWorkspace.acXx[21] + nmheVariables.WL[30]*nmheWorkspace.acXx[29] + nmheVariables.WL[38]*nmheWorkspace.acXx[37] + nmheVariables.WL[46]*nmheWorkspace.acXx[45] + nmheVariables.WL[54]*nmheWorkspace.acXx[53] + nmheVariables.WL[62]*nmheWorkspace.acXx[61];
nmheWorkspace.acA[402] -= + nmheVariables.WL[6]*nmheWorkspace.acXx[6] + nmheVariables.WL[14]*nmheWorkspace.acXx[14] + nmheVariables.WL[22]*nmheWorkspace.acXx[22] + nmheVariables.WL[30]*nmheWorkspace.acXx[30] + nmheVariables.WL[38]*nmheWorkspace.acXx[38] + nmheVariables.WL[46]*nmheWorkspace.acXx[46] + nmheVariables.WL[54]*nmheWorkspace.acXx[54] + nmheVariables.WL[62]*nmheWorkspace.acXx[62];
nmheWorkspace.acA[403] -= + nmheVariables.WL[6]*nmheWorkspace.acXx[7] + nmheVariables.WL[14]*nmheWorkspace.acXx[15] + nmheVariables.WL[22]*nmheWorkspace.acXx[23] + nmheVariables.WL[30]*nmheWorkspace.acXx[31] + nmheVariables.WL[38]*nmheWorkspace.acXx[39] + nmheVariables.WL[46]*nmheWorkspace.acXx[47] + nmheVariables.WL[54]*nmheWorkspace.acXx[55] + nmheVariables.WL[62]*nmheWorkspace.acXx[63];
nmheWorkspace.acA[414] -= + nmheVariables.WL[7]*nmheWorkspace.acXx[0] + nmheVariables.WL[15]*nmheWorkspace.acXx[8] + nmheVariables.WL[23]*nmheWorkspace.acXx[16] + nmheVariables.WL[31]*nmheWorkspace.acXx[24] + nmheVariables.WL[39]*nmheWorkspace.acXx[32] + nmheVariables.WL[47]*nmheWorkspace.acXx[40] + nmheVariables.WL[55]*nmheWorkspace.acXx[48] + nmheVariables.WL[63]*nmheWorkspace.acXx[56];
nmheWorkspace.acA[415] -= + nmheVariables.WL[7]*nmheWorkspace.acXx[1] + nmheVariables.WL[15]*nmheWorkspace.acXx[9] + nmheVariables.WL[23]*nmheWorkspace.acXx[17] + nmheVariables.WL[31]*nmheWorkspace.acXx[25] + nmheVariables.WL[39]*nmheWorkspace.acXx[33] + nmheVariables.WL[47]*nmheWorkspace.acXx[41] + nmheVariables.WL[55]*nmheWorkspace.acXx[49] + nmheVariables.WL[63]*nmheWorkspace.acXx[57];
nmheWorkspace.acA[416] -= + nmheVariables.WL[7]*nmheWorkspace.acXx[2] + nmheVariables.WL[15]*nmheWorkspace.acXx[10] + nmheVariables.WL[23]*nmheWorkspace.acXx[18] + nmheVariables.WL[31]*nmheWorkspace.acXx[26] + nmheVariables.WL[39]*nmheWorkspace.acXx[34] + nmheVariables.WL[47]*nmheWorkspace.acXx[42] + nmheVariables.WL[55]*nmheWorkspace.acXx[50] + nmheVariables.WL[63]*nmheWorkspace.acXx[58];
nmheWorkspace.acA[417] -= + nmheVariables.WL[7]*nmheWorkspace.acXx[3] + nmheVariables.WL[15]*nmheWorkspace.acXx[11] + nmheVariables.WL[23]*nmheWorkspace.acXx[19] + nmheVariables.WL[31]*nmheWorkspace.acXx[27] + nmheVariables.WL[39]*nmheWorkspace.acXx[35] + nmheVariables.WL[47]*nmheWorkspace.acXx[43] + nmheVariables.WL[55]*nmheWorkspace.acXx[51] + nmheVariables.WL[63]*nmheWorkspace.acXx[59];
nmheWorkspace.acA[418] -= + nmheVariables.WL[7]*nmheWorkspace.acXx[4] + nmheVariables.WL[15]*nmheWorkspace.acXx[12] + nmheVariables.WL[23]*nmheWorkspace.acXx[20] + nmheVariables.WL[31]*nmheWorkspace.acXx[28] + nmheVariables.WL[39]*nmheWorkspace.acXx[36] + nmheVariables.WL[47]*nmheWorkspace.acXx[44] + nmheVariables.WL[55]*nmheWorkspace.acXx[52] + nmheVariables.WL[63]*nmheWorkspace.acXx[60];
nmheWorkspace.acA[419] -= + nmheVariables.WL[7]*nmheWorkspace.acXx[5] + nmheVariables.WL[15]*nmheWorkspace.acXx[13] + nmheVariables.WL[23]*nmheWorkspace.acXx[21] + nmheVariables.WL[31]*nmheWorkspace.acXx[29] + nmheVariables.WL[39]*nmheWorkspace.acXx[37] + nmheVariables.WL[47]*nmheWorkspace.acXx[45] + nmheVariables.WL[55]*nmheWorkspace.acXx[53] + nmheVariables.WL[63]*nmheWorkspace.acXx[61];
nmheWorkspace.acA[420] -= + nmheVariables.WL[7]*nmheWorkspace.acXx[6] + nmheVariables.WL[15]*nmheWorkspace.acXx[14] + nmheVariables.WL[23]*nmheWorkspace.acXx[22] + nmheVariables.WL[31]*nmheWorkspace.acXx[30] + nmheVariables.WL[39]*nmheWorkspace.acXx[38] + nmheVariables.WL[47]*nmheWorkspace.acXx[46] + nmheVariables.WL[55]*nmheWorkspace.acXx[54] + nmheVariables.WL[63]*nmheWorkspace.acXx[62];
nmheWorkspace.acA[421] -= + nmheVariables.WL[7]*nmheWorkspace.acXx[7] + nmheVariables.WL[15]*nmheWorkspace.acXx[15] + nmheVariables.WL[23]*nmheWorkspace.acXx[23] + nmheVariables.WL[31]*nmheWorkspace.acXx[31] + nmheVariables.WL[39]*nmheWorkspace.acXx[39] + nmheVariables.WL[47]*nmheWorkspace.acXx[47] + nmheVariables.WL[55]*nmheWorkspace.acXx[55] + nmheVariables.WL[63]*nmheWorkspace.acXx[63];
nmheWorkspace.acA[296] -= + nmheVariables.WL[0]*nmheWorkspace.acXu[0] + nmheVariables.WL[8]*nmheWorkspace.acXu[2] + nmheVariables.WL[16]*nmheWorkspace.acXu[4] + nmheVariables.WL[24]*nmheWorkspace.acXu[6] + nmheVariables.WL[32]*nmheWorkspace.acXu[8] + nmheVariables.WL[40]*nmheWorkspace.acXu[10] + nmheVariables.WL[48]*nmheWorkspace.acXu[12] + nmheVariables.WL[56]*nmheWorkspace.acXu[14];
nmheWorkspace.acA[297] -= + nmheVariables.WL[0]*nmheWorkspace.acXu[1] + nmheVariables.WL[8]*nmheWorkspace.acXu[3] + nmheVariables.WL[16]*nmheWorkspace.acXu[5] + nmheVariables.WL[24]*nmheWorkspace.acXu[7] + nmheVariables.WL[32]*nmheWorkspace.acXu[9] + nmheVariables.WL[40]*nmheWorkspace.acXu[11] + nmheVariables.WL[48]*nmheWorkspace.acXu[13] + nmheVariables.WL[56]*nmheWorkspace.acXu[15];
nmheWorkspace.acA[314] -= + nmheVariables.WL[1]*nmheWorkspace.acXu[0] + nmheVariables.WL[9]*nmheWorkspace.acXu[2] + nmheVariables.WL[17]*nmheWorkspace.acXu[4] + nmheVariables.WL[25]*nmheWorkspace.acXu[6] + nmheVariables.WL[33]*nmheWorkspace.acXu[8] + nmheVariables.WL[41]*nmheWorkspace.acXu[10] + nmheVariables.WL[49]*nmheWorkspace.acXu[12] + nmheVariables.WL[57]*nmheWorkspace.acXu[14];
nmheWorkspace.acA[315] -= + nmheVariables.WL[1]*nmheWorkspace.acXu[1] + nmheVariables.WL[9]*nmheWorkspace.acXu[3] + nmheVariables.WL[17]*nmheWorkspace.acXu[5] + nmheVariables.WL[25]*nmheWorkspace.acXu[7] + nmheVariables.WL[33]*nmheWorkspace.acXu[9] + nmheVariables.WL[41]*nmheWorkspace.acXu[11] + nmheVariables.WL[49]*nmheWorkspace.acXu[13] + nmheVariables.WL[57]*nmheWorkspace.acXu[15];
nmheWorkspace.acA[332] -= + nmheVariables.WL[2]*nmheWorkspace.acXu[0] + nmheVariables.WL[10]*nmheWorkspace.acXu[2] + nmheVariables.WL[18]*nmheWorkspace.acXu[4] + nmheVariables.WL[26]*nmheWorkspace.acXu[6] + nmheVariables.WL[34]*nmheWorkspace.acXu[8] + nmheVariables.WL[42]*nmheWorkspace.acXu[10] + nmheVariables.WL[50]*nmheWorkspace.acXu[12] + nmheVariables.WL[58]*nmheWorkspace.acXu[14];
nmheWorkspace.acA[333] -= + nmheVariables.WL[2]*nmheWorkspace.acXu[1] + nmheVariables.WL[10]*nmheWorkspace.acXu[3] + nmheVariables.WL[18]*nmheWorkspace.acXu[5] + nmheVariables.WL[26]*nmheWorkspace.acXu[7] + nmheVariables.WL[34]*nmheWorkspace.acXu[9] + nmheVariables.WL[42]*nmheWorkspace.acXu[11] + nmheVariables.WL[50]*nmheWorkspace.acXu[13] + nmheVariables.WL[58]*nmheWorkspace.acXu[15];
nmheWorkspace.acA[350] -= + nmheVariables.WL[3]*nmheWorkspace.acXu[0] + nmheVariables.WL[11]*nmheWorkspace.acXu[2] + nmheVariables.WL[19]*nmheWorkspace.acXu[4] + nmheVariables.WL[27]*nmheWorkspace.acXu[6] + nmheVariables.WL[35]*nmheWorkspace.acXu[8] + nmheVariables.WL[43]*nmheWorkspace.acXu[10] + nmheVariables.WL[51]*nmheWorkspace.acXu[12] + nmheVariables.WL[59]*nmheWorkspace.acXu[14];
nmheWorkspace.acA[351] -= + nmheVariables.WL[3]*nmheWorkspace.acXu[1] + nmheVariables.WL[11]*nmheWorkspace.acXu[3] + nmheVariables.WL[19]*nmheWorkspace.acXu[5] + nmheVariables.WL[27]*nmheWorkspace.acXu[7] + nmheVariables.WL[35]*nmheWorkspace.acXu[9] + nmheVariables.WL[43]*nmheWorkspace.acXu[11] + nmheVariables.WL[51]*nmheWorkspace.acXu[13] + nmheVariables.WL[59]*nmheWorkspace.acXu[15];
nmheWorkspace.acA[368] -= + nmheVariables.WL[4]*nmheWorkspace.acXu[0] + nmheVariables.WL[12]*nmheWorkspace.acXu[2] + nmheVariables.WL[20]*nmheWorkspace.acXu[4] + nmheVariables.WL[28]*nmheWorkspace.acXu[6] + nmheVariables.WL[36]*nmheWorkspace.acXu[8] + nmheVariables.WL[44]*nmheWorkspace.acXu[10] + nmheVariables.WL[52]*nmheWorkspace.acXu[12] + nmheVariables.WL[60]*nmheWorkspace.acXu[14];
nmheWorkspace.acA[369] -= + nmheVariables.WL[4]*nmheWorkspace.acXu[1] + nmheVariables.WL[12]*nmheWorkspace.acXu[3] + nmheVariables.WL[20]*nmheWorkspace.acXu[5] + nmheVariables.WL[28]*nmheWorkspace.acXu[7] + nmheVariables.WL[36]*nmheWorkspace.acXu[9] + nmheVariables.WL[44]*nmheWorkspace.acXu[11] + nmheVariables.WL[52]*nmheWorkspace.acXu[13] + nmheVariables.WL[60]*nmheWorkspace.acXu[15];
nmheWorkspace.acA[386] -= + nmheVariables.WL[5]*nmheWorkspace.acXu[0] + nmheVariables.WL[13]*nmheWorkspace.acXu[2] + nmheVariables.WL[21]*nmheWorkspace.acXu[4] + nmheVariables.WL[29]*nmheWorkspace.acXu[6] + nmheVariables.WL[37]*nmheWorkspace.acXu[8] + nmheVariables.WL[45]*nmheWorkspace.acXu[10] + nmheVariables.WL[53]*nmheWorkspace.acXu[12] + nmheVariables.WL[61]*nmheWorkspace.acXu[14];
nmheWorkspace.acA[387] -= + nmheVariables.WL[5]*nmheWorkspace.acXu[1] + nmheVariables.WL[13]*nmheWorkspace.acXu[3] + nmheVariables.WL[21]*nmheWorkspace.acXu[5] + nmheVariables.WL[29]*nmheWorkspace.acXu[7] + nmheVariables.WL[37]*nmheWorkspace.acXu[9] + nmheVariables.WL[45]*nmheWorkspace.acXu[11] + nmheVariables.WL[53]*nmheWorkspace.acXu[13] + nmheVariables.WL[61]*nmheWorkspace.acXu[15];
nmheWorkspace.acA[404] -= + nmheVariables.WL[6]*nmheWorkspace.acXu[0] + nmheVariables.WL[14]*nmheWorkspace.acXu[2] + nmheVariables.WL[22]*nmheWorkspace.acXu[4] + nmheVariables.WL[30]*nmheWorkspace.acXu[6] + nmheVariables.WL[38]*nmheWorkspace.acXu[8] + nmheVariables.WL[46]*nmheWorkspace.acXu[10] + nmheVariables.WL[54]*nmheWorkspace.acXu[12] + nmheVariables.WL[62]*nmheWorkspace.acXu[14];
nmheWorkspace.acA[405] -= + nmheVariables.WL[6]*nmheWorkspace.acXu[1] + nmheVariables.WL[14]*nmheWorkspace.acXu[3] + nmheVariables.WL[22]*nmheWorkspace.acXu[5] + nmheVariables.WL[30]*nmheWorkspace.acXu[7] + nmheVariables.WL[38]*nmheWorkspace.acXu[9] + nmheVariables.WL[46]*nmheWorkspace.acXu[11] + nmheVariables.WL[54]*nmheWorkspace.acXu[13] + nmheVariables.WL[62]*nmheWorkspace.acXu[15];
nmheWorkspace.acA[422] -= + nmheVariables.WL[7]*nmheWorkspace.acXu[0] + nmheVariables.WL[15]*nmheWorkspace.acXu[2] + nmheVariables.WL[23]*nmheWorkspace.acXu[4] + nmheVariables.WL[31]*nmheWorkspace.acXu[6] + nmheVariables.WL[39]*nmheWorkspace.acXu[8] + nmheVariables.WL[47]*nmheWorkspace.acXu[10] + nmheVariables.WL[55]*nmheWorkspace.acXu[12] + nmheVariables.WL[63]*nmheWorkspace.acXu[14];
nmheWorkspace.acA[423] -= + nmheVariables.WL[7]*nmheWorkspace.acXu[1] + nmheVariables.WL[15]*nmheWorkspace.acXu[3] + nmheVariables.WL[23]*nmheWorkspace.acXu[5] + nmheVariables.WL[31]*nmheWorkspace.acXu[7] + nmheVariables.WL[39]*nmheWorkspace.acXu[9] + nmheVariables.WL[47]*nmheWorkspace.acXu[11] + nmheVariables.WL[55]*nmheWorkspace.acXu[13] + nmheVariables.WL[63]*nmheWorkspace.acXu[15];
nmheWorkspace.acA[298] = nmheVariables.WL[0];
nmheWorkspace.acA[299] = nmheVariables.WL[8];
nmheWorkspace.acA[300] = nmheVariables.WL[16];
nmheWorkspace.acA[301] = nmheVariables.WL[24];
nmheWorkspace.acA[302] = nmheVariables.WL[32];
nmheWorkspace.acA[303] = nmheVariables.WL[40];
nmheWorkspace.acA[304] = nmheVariables.WL[48];
nmheWorkspace.acA[305] = nmheVariables.WL[56];
nmheWorkspace.acA[316] = nmheVariables.WL[1];
nmheWorkspace.acA[317] = nmheVariables.WL[9];
nmheWorkspace.acA[318] = nmheVariables.WL[17];
nmheWorkspace.acA[319] = nmheVariables.WL[25];
nmheWorkspace.acA[320] = nmheVariables.WL[33];
nmheWorkspace.acA[321] = nmheVariables.WL[41];
nmheWorkspace.acA[322] = nmheVariables.WL[49];
nmheWorkspace.acA[323] = nmheVariables.WL[57];
nmheWorkspace.acA[334] = nmheVariables.WL[2];
nmheWorkspace.acA[335] = nmheVariables.WL[10];
nmheWorkspace.acA[336] = nmheVariables.WL[18];
nmheWorkspace.acA[337] = nmheVariables.WL[26];
nmheWorkspace.acA[338] = nmheVariables.WL[34];
nmheWorkspace.acA[339] = nmheVariables.WL[42];
nmheWorkspace.acA[340] = nmheVariables.WL[50];
nmheWorkspace.acA[341] = nmheVariables.WL[58];
nmheWorkspace.acA[352] = nmheVariables.WL[3];
nmheWorkspace.acA[353] = nmheVariables.WL[11];
nmheWorkspace.acA[354] = nmheVariables.WL[19];
nmheWorkspace.acA[355] = nmheVariables.WL[27];
nmheWorkspace.acA[356] = nmheVariables.WL[35];
nmheWorkspace.acA[357] = nmheVariables.WL[43];
nmheWorkspace.acA[358] = nmheVariables.WL[51];
nmheWorkspace.acA[359] = nmheVariables.WL[59];
nmheWorkspace.acA[370] = nmheVariables.WL[4];
nmheWorkspace.acA[371] = nmheVariables.WL[12];
nmheWorkspace.acA[372] = nmheVariables.WL[20];
nmheWorkspace.acA[373] = nmheVariables.WL[28];
nmheWorkspace.acA[374] = nmheVariables.WL[36];
nmheWorkspace.acA[375] = nmheVariables.WL[44];
nmheWorkspace.acA[376] = nmheVariables.WL[52];
nmheWorkspace.acA[377] = nmheVariables.WL[60];
nmheWorkspace.acA[388] = nmheVariables.WL[5];
nmheWorkspace.acA[389] = nmheVariables.WL[13];
nmheWorkspace.acA[390] = nmheVariables.WL[21];
nmheWorkspace.acA[391] = nmheVariables.WL[29];
nmheWorkspace.acA[392] = nmheVariables.WL[37];
nmheWorkspace.acA[393] = nmheVariables.WL[45];
nmheWorkspace.acA[394] = nmheVariables.WL[53];
nmheWorkspace.acA[395] = nmheVariables.WL[61];
nmheWorkspace.acA[406] = nmheVariables.WL[6];
nmheWorkspace.acA[407] = nmheVariables.WL[14];
nmheWorkspace.acA[408] = nmheVariables.WL[22];
nmheWorkspace.acA[409] = nmheVariables.WL[30];
nmheWorkspace.acA[410] = nmheVariables.WL[38];
nmheWorkspace.acA[411] = nmheVariables.WL[46];
nmheWorkspace.acA[412] = nmheVariables.WL[54];
nmheWorkspace.acA[413] = nmheVariables.WL[62];
nmheWorkspace.acA[424] = nmheVariables.WL[7];
nmheWorkspace.acA[425] = nmheVariables.WL[15];
nmheWorkspace.acA[426] = nmheVariables.WL[23];
nmheWorkspace.acA[427] = nmheVariables.WL[31];
nmheWorkspace.acA[428] = nmheVariables.WL[39];
nmheWorkspace.acA[429] = nmheVariables.WL[47];
nmheWorkspace.acA[430] = nmheVariables.WL[55];
nmheWorkspace.acA[431] = nmheVariables.WL[63];
nmheWorkspace.acXTilde[0] = nmheWorkspace.state[0];
nmheWorkspace.acXTilde[1] = nmheWorkspace.state[1];
nmheWorkspace.acXTilde[2] = nmheWorkspace.state[2];
nmheWorkspace.acXTilde[3] = nmheWorkspace.state[3];
nmheWorkspace.acXTilde[4] = nmheWorkspace.state[4];
nmheWorkspace.acXTilde[5] = nmheWorkspace.state[5];
nmheWorkspace.acXTilde[6] = nmheWorkspace.state[6];
nmheWorkspace.acXTilde[7] = nmheWorkspace.state[7];
nmheWorkspace.acXTilde[0] -= + nmheWorkspace.acXx[0]*nmheVariables.x[0] + nmheWorkspace.acXx[1]*nmheVariables.x[1] + nmheWorkspace.acXx[2]*nmheVariables.x[2] + nmheWorkspace.acXx[3]*nmheVariables.x[3] + nmheWorkspace.acXx[4]*nmheVariables.x[4] + nmheWorkspace.acXx[5]*nmheVariables.x[5] + nmheWorkspace.acXx[6]*nmheVariables.x[6] + nmheWorkspace.acXx[7]*nmheVariables.x[7];
nmheWorkspace.acXTilde[1] -= + nmheWorkspace.acXx[8]*nmheVariables.x[0] + nmheWorkspace.acXx[9]*nmheVariables.x[1] + nmheWorkspace.acXx[10]*nmheVariables.x[2] + nmheWorkspace.acXx[11]*nmheVariables.x[3] + nmheWorkspace.acXx[12]*nmheVariables.x[4] + nmheWorkspace.acXx[13]*nmheVariables.x[5] + nmheWorkspace.acXx[14]*nmheVariables.x[6] + nmheWorkspace.acXx[15]*nmheVariables.x[7];
nmheWorkspace.acXTilde[2] -= + nmheWorkspace.acXx[16]*nmheVariables.x[0] + nmheWorkspace.acXx[17]*nmheVariables.x[1] + nmheWorkspace.acXx[18]*nmheVariables.x[2] + nmheWorkspace.acXx[19]*nmheVariables.x[3] + nmheWorkspace.acXx[20]*nmheVariables.x[4] + nmheWorkspace.acXx[21]*nmheVariables.x[5] + nmheWorkspace.acXx[22]*nmheVariables.x[6] + nmheWorkspace.acXx[23]*nmheVariables.x[7];
nmheWorkspace.acXTilde[3] -= + nmheWorkspace.acXx[24]*nmheVariables.x[0] + nmheWorkspace.acXx[25]*nmheVariables.x[1] + nmheWorkspace.acXx[26]*nmheVariables.x[2] + nmheWorkspace.acXx[27]*nmheVariables.x[3] + nmheWorkspace.acXx[28]*nmheVariables.x[4] + nmheWorkspace.acXx[29]*nmheVariables.x[5] + nmheWorkspace.acXx[30]*nmheVariables.x[6] + nmheWorkspace.acXx[31]*nmheVariables.x[7];
nmheWorkspace.acXTilde[4] -= + nmheWorkspace.acXx[32]*nmheVariables.x[0] + nmheWorkspace.acXx[33]*nmheVariables.x[1] + nmheWorkspace.acXx[34]*nmheVariables.x[2] + nmheWorkspace.acXx[35]*nmheVariables.x[3] + nmheWorkspace.acXx[36]*nmheVariables.x[4] + nmheWorkspace.acXx[37]*nmheVariables.x[5] + nmheWorkspace.acXx[38]*nmheVariables.x[6] + nmheWorkspace.acXx[39]*nmheVariables.x[7];
nmheWorkspace.acXTilde[5] -= + nmheWorkspace.acXx[40]*nmheVariables.x[0] + nmheWorkspace.acXx[41]*nmheVariables.x[1] + nmheWorkspace.acXx[42]*nmheVariables.x[2] + nmheWorkspace.acXx[43]*nmheVariables.x[3] + nmheWorkspace.acXx[44]*nmheVariables.x[4] + nmheWorkspace.acXx[45]*nmheVariables.x[5] + nmheWorkspace.acXx[46]*nmheVariables.x[6] + nmheWorkspace.acXx[47]*nmheVariables.x[7];
nmheWorkspace.acXTilde[6] -= + nmheWorkspace.acXx[48]*nmheVariables.x[0] + nmheWorkspace.acXx[49]*nmheVariables.x[1] + nmheWorkspace.acXx[50]*nmheVariables.x[2] + nmheWorkspace.acXx[51]*nmheVariables.x[3] + nmheWorkspace.acXx[52]*nmheVariables.x[4] + nmheWorkspace.acXx[53]*nmheVariables.x[5] + nmheWorkspace.acXx[54]*nmheVariables.x[6] + nmheWorkspace.acXx[55]*nmheVariables.x[7];
nmheWorkspace.acXTilde[7] -= + nmheWorkspace.acXx[56]*nmheVariables.x[0] + nmheWorkspace.acXx[57]*nmheVariables.x[1] + nmheWorkspace.acXx[58]*nmheVariables.x[2] + nmheWorkspace.acXx[59]*nmheVariables.x[3] + nmheWorkspace.acXx[60]*nmheVariables.x[4] + nmheWorkspace.acXx[61]*nmheVariables.x[5] + nmheWorkspace.acXx[62]*nmheVariables.x[6] + nmheWorkspace.acXx[63]*nmheVariables.x[7];
nmheWorkspace.acXTilde[0] -= + nmheWorkspace.acXu[0]*nmheVariables.u[0] + nmheWorkspace.acXu[1]*nmheVariables.u[1];
nmheWorkspace.acXTilde[1] -= + nmheWorkspace.acXu[2]*nmheVariables.u[0] + nmheWorkspace.acXu[3]*nmheVariables.u[1];
nmheWorkspace.acXTilde[2] -= + nmheWorkspace.acXu[4]*nmheVariables.u[0] + nmheWorkspace.acXu[5]*nmheVariables.u[1];
nmheWorkspace.acXTilde[3] -= + nmheWorkspace.acXu[6]*nmheVariables.u[0] + nmheWorkspace.acXu[7]*nmheVariables.u[1];
nmheWorkspace.acXTilde[4] -= + nmheWorkspace.acXu[8]*nmheVariables.u[0] + nmheWorkspace.acXu[9]*nmheVariables.u[1];
nmheWorkspace.acXTilde[5] -= + nmheWorkspace.acXu[10]*nmheVariables.u[0] + nmheWorkspace.acXu[11]*nmheVariables.u[1];
nmheWorkspace.acXTilde[6] -= + nmheWorkspace.acXu[12]*nmheVariables.u[0] + nmheWorkspace.acXu[13]*nmheVariables.u[1];
nmheWorkspace.acXTilde[7] -= + nmheWorkspace.acXu[14]*nmheVariables.u[0] + nmheWorkspace.acXu[15]*nmheVariables.u[1];
nmheWorkspace.acHTilde[0] = nmheVariables.y[0];
nmheWorkspace.acHTilde[1] = nmheVariables.y[1];
nmheWorkspace.acHTilde[2] = nmheVariables.y[2];
nmheWorkspace.acHTilde[3] = nmheVariables.y[3];
nmheWorkspace.acHTilde[4] = nmheVariables.y[4];
nmheWorkspace.acHTilde[5] = nmheVariables.y[5];
nmheWorkspace.acHTilde[6] = nmheVariables.y[6];
nmheWorkspace.acHTilde[7] = nmheVariables.y[7];
nmheWorkspace.acHTilde[0] -= nmheWorkspace.objValueOut[0];
nmheWorkspace.acHTilde[1] -= nmheWorkspace.objValueOut[1];
nmheWorkspace.acHTilde[2] -= nmheWorkspace.objValueOut[2];
nmheWorkspace.acHTilde[3] -= nmheWorkspace.objValueOut[3];
nmheWorkspace.acHTilde[4] -= nmheWorkspace.objValueOut[4];
nmheWorkspace.acHTilde[5] -= nmheWorkspace.objValueOut[5];
nmheWorkspace.acHTilde[6] -= nmheWorkspace.objValueOut[6];
nmheWorkspace.acHTilde[7] -= nmheWorkspace.objValueOut[7];
nmheWorkspace.acHTilde[0] += + nmheWorkspace.acHx[0]*nmheVariables.x[0] + nmheWorkspace.acHx[1]*nmheVariables.x[1] + nmheWorkspace.acHx[2]*nmheVariables.x[2] + nmheWorkspace.acHx[3]*nmheVariables.x[3] + nmheWorkspace.acHx[4]*nmheVariables.x[4] + nmheWorkspace.acHx[5]*nmheVariables.x[5] + nmheWorkspace.acHx[6]*nmheVariables.x[6] + nmheWorkspace.acHx[7]*nmheVariables.x[7];
nmheWorkspace.acHTilde[1] += + nmheWorkspace.acHx[8]*nmheVariables.x[0] + nmheWorkspace.acHx[9]*nmheVariables.x[1] + nmheWorkspace.acHx[10]*nmheVariables.x[2] + nmheWorkspace.acHx[11]*nmheVariables.x[3] + nmheWorkspace.acHx[12]*nmheVariables.x[4] + nmheWorkspace.acHx[13]*nmheVariables.x[5] + nmheWorkspace.acHx[14]*nmheVariables.x[6] + nmheWorkspace.acHx[15]*nmheVariables.x[7];
nmheWorkspace.acHTilde[2] += + nmheWorkspace.acHx[16]*nmheVariables.x[0] + nmheWorkspace.acHx[17]*nmheVariables.x[1] + nmheWorkspace.acHx[18]*nmheVariables.x[2] + nmheWorkspace.acHx[19]*nmheVariables.x[3] + nmheWorkspace.acHx[20]*nmheVariables.x[4] + nmheWorkspace.acHx[21]*nmheVariables.x[5] + nmheWorkspace.acHx[22]*nmheVariables.x[6] + nmheWorkspace.acHx[23]*nmheVariables.x[7];
nmheWorkspace.acHTilde[3] += + nmheWorkspace.acHx[24]*nmheVariables.x[0] + nmheWorkspace.acHx[25]*nmheVariables.x[1] + nmheWorkspace.acHx[26]*nmheVariables.x[2] + nmheWorkspace.acHx[27]*nmheVariables.x[3] + nmheWorkspace.acHx[28]*nmheVariables.x[4] + nmheWorkspace.acHx[29]*nmheVariables.x[5] + nmheWorkspace.acHx[30]*nmheVariables.x[6] + nmheWorkspace.acHx[31]*nmheVariables.x[7];
nmheWorkspace.acHTilde[4] += + nmheWorkspace.acHx[32]*nmheVariables.x[0] + nmheWorkspace.acHx[33]*nmheVariables.x[1] + nmheWorkspace.acHx[34]*nmheVariables.x[2] + nmheWorkspace.acHx[35]*nmheVariables.x[3] + nmheWorkspace.acHx[36]*nmheVariables.x[4] + nmheWorkspace.acHx[37]*nmheVariables.x[5] + nmheWorkspace.acHx[38]*nmheVariables.x[6] + nmheWorkspace.acHx[39]*nmheVariables.x[7];
nmheWorkspace.acHTilde[5] += + nmheWorkspace.acHx[40]*nmheVariables.x[0] + nmheWorkspace.acHx[41]*nmheVariables.x[1] + nmheWorkspace.acHx[42]*nmheVariables.x[2] + nmheWorkspace.acHx[43]*nmheVariables.x[3] + nmheWorkspace.acHx[44]*nmheVariables.x[4] + nmheWorkspace.acHx[45]*nmheVariables.x[5] + nmheWorkspace.acHx[46]*nmheVariables.x[6] + nmheWorkspace.acHx[47]*nmheVariables.x[7];
nmheWorkspace.acHTilde[6] += + nmheWorkspace.acHx[48]*nmheVariables.x[0] + nmheWorkspace.acHx[49]*nmheVariables.x[1] + nmheWorkspace.acHx[50]*nmheVariables.x[2] + nmheWorkspace.acHx[51]*nmheVariables.x[3] + nmheWorkspace.acHx[52]*nmheVariables.x[4] + nmheWorkspace.acHx[53]*nmheVariables.x[5] + nmheWorkspace.acHx[54]*nmheVariables.x[6] + nmheWorkspace.acHx[55]*nmheVariables.x[7];
nmheWorkspace.acHTilde[7] += + nmheWorkspace.acHx[56]*nmheVariables.x[0] + nmheWorkspace.acHx[57]*nmheVariables.x[1] + nmheWorkspace.acHx[58]*nmheVariables.x[2] + nmheWorkspace.acHx[59]*nmheVariables.x[3] + nmheWorkspace.acHx[60]*nmheVariables.x[4] + nmheWorkspace.acHx[61]*nmheVariables.x[5] + nmheWorkspace.acHx[62]*nmheVariables.x[6] + nmheWorkspace.acHx[63]*nmheVariables.x[7];
nmheWorkspace.acHTilde[0] += + nmheWorkspace.acHu[0]*nmheVariables.u[0] + nmheWorkspace.acHu[1]*nmheVariables.u[1];
nmheWorkspace.acHTilde[1] += + nmheWorkspace.acHu[2]*nmheVariables.u[0] + nmheWorkspace.acHu[3]*nmheVariables.u[1];
nmheWorkspace.acHTilde[2] += + nmheWorkspace.acHu[4]*nmheVariables.u[0] + nmheWorkspace.acHu[5]*nmheVariables.u[1];
nmheWorkspace.acHTilde[3] += + nmheWorkspace.acHu[6]*nmheVariables.u[0] + nmheWorkspace.acHu[7]*nmheVariables.u[1];
nmheWorkspace.acHTilde[4] += + nmheWorkspace.acHu[8]*nmheVariables.u[0] + nmheWorkspace.acHu[9]*nmheVariables.u[1];
nmheWorkspace.acHTilde[5] += + nmheWorkspace.acHu[10]*nmheVariables.u[0] + nmheWorkspace.acHu[11]*nmheVariables.u[1];
nmheWorkspace.acHTilde[6] += + nmheWorkspace.acHu[12]*nmheVariables.u[0] + nmheWorkspace.acHu[13]*nmheVariables.u[1];
nmheWorkspace.acHTilde[7] += + nmheWorkspace.acHu[14]*nmheVariables.u[0] + nmheWorkspace.acHu[15]*nmheVariables.u[1];
nmheWorkspace.acb[0] = + nmheWorkspace.acP[0]*nmheVariables.xAC[0] + nmheWorkspace.acP[1]*nmheVariables.xAC[1] + nmheWorkspace.acP[2]*nmheVariables.xAC[2] + nmheWorkspace.acP[3]*nmheVariables.xAC[3] + nmheWorkspace.acP[4]*nmheVariables.xAC[4] + nmheWorkspace.acP[5]*nmheVariables.xAC[5] + nmheWorkspace.acP[6]*nmheVariables.xAC[6] + nmheWorkspace.acP[7]*nmheVariables.xAC[7];
nmheWorkspace.acb[1] = + nmheWorkspace.acP[8]*nmheVariables.xAC[0] + nmheWorkspace.acP[9]*nmheVariables.xAC[1] + nmheWorkspace.acP[10]*nmheVariables.xAC[2] + nmheWorkspace.acP[11]*nmheVariables.xAC[3] + nmheWorkspace.acP[12]*nmheVariables.xAC[4] + nmheWorkspace.acP[13]*nmheVariables.xAC[5] + nmheWorkspace.acP[14]*nmheVariables.xAC[6] + nmheWorkspace.acP[15]*nmheVariables.xAC[7];
nmheWorkspace.acb[2] = + nmheWorkspace.acP[16]*nmheVariables.xAC[0] + nmheWorkspace.acP[17]*nmheVariables.xAC[1] + nmheWorkspace.acP[18]*nmheVariables.xAC[2] + nmheWorkspace.acP[19]*nmheVariables.xAC[3] + nmheWorkspace.acP[20]*nmheVariables.xAC[4] + nmheWorkspace.acP[21]*nmheVariables.xAC[5] + nmheWorkspace.acP[22]*nmheVariables.xAC[6] + nmheWorkspace.acP[23]*nmheVariables.xAC[7];
nmheWorkspace.acb[3] = + nmheWorkspace.acP[24]*nmheVariables.xAC[0] + nmheWorkspace.acP[25]*nmheVariables.xAC[1] + nmheWorkspace.acP[26]*nmheVariables.xAC[2] + nmheWorkspace.acP[27]*nmheVariables.xAC[3] + nmheWorkspace.acP[28]*nmheVariables.xAC[4] + nmheWorkspace.acP[29]*nmheVariables.xAC[5] + nmheWorkspace.acP[30]*nmheVariables.xAC[6] + nmheWorkspace.acP[31]*nmheVariables.xAC[7];
nmheWorkspace.acb[4] = + nmheWorkspace.acP[32]*nmheVariables.xAC[0] + nmheWorkspace.acP[33]*nmheVariables.xAC[1] + nmheWorkspace.acP[34]*nmheVariables.xAC[2] + nmheWorkspace.acP[35]*nmheVariables.xAC[3] + nmheWorkspace.acP[36]*nmheVariables.xAC[4] + nmheWorkspace.acP[37]*nmheVariables.xAC[5] + nmheWorkspace.acP[38]*nmheVariables.xAC[6] + nmheWorkspace.acP[39]*nmheVariables.xAC[7];
nmheWorkspace.acb[5] = + nmheWorkspace.acP[40]*nmheVariables.xAC[0] + nmheWorkspace.acP[41]*nmheVariables.xAC[1] + nmheWorkspace.acP[42]*nmheVariables.xAC[2] + nmheWorkspace.acP[43]*nmheVariables.xAC[3] + nmheWorkspace.acP[44]*nmheVariables.xAC[4] + nmheWorkspace.acP[45]*nmheVariables.xAC[5] + nmheWorkspace.acP[46]*nmheVariables.xAC[6] + nmheWorkspace.acP[47]*nmheVariables.xAC[7];
nmheWorkspace.acb[6] = + nmheWorkspace.acP[48]*nmheVariables.xAC[0] + nmheWorkspace.acP[49]*nmheVariables.xAC[1] + nmheWorkspace.acP[50]*nmheVariables.xAC[2] + nmheWorkspace.acP[51]*nmheVariables.xAC[3] + nmheWorkspace.acP[52]*nmheVariables.xAC[4] + nmheWorkspace.acP[53]*nmheVariables.xAC[5] + nmheWorkspace.acP[54]*nmheVariables.xAC[6] + nmheWorkspace.acP[55]*nmheVariables.xAC[7];
nmheWorkspace.acb[7] = + nmheWorkspace.acP[56]*nmheVariables.xAC[0] + nmheWorkspace.acP[57]*nmheVariables.xAC[1] + nmheWorkspace.acP[58]*nmheVariables.xAC[2] + nmheWorkspace.acP[59]*nmheVariables.xAC[3] + nmheWorkspace.acP[60]*nmheVariables.xAC[4] + nmheWorkspace.acP[61]*nmheVariables.xAC[5] + nmheWorkspace.acP[62]*nmheVariables.xAC[6] + nmheWorkspace.acP[63]*nmheVariables.xAC[7];
nmheWorkspace.acb[8] -= + nmheWorkspace.acVL[0]*nmheWorkspace.acHTilde[0] + nmheWorkspace.acVL[8]*nmheWorkspace.acHTilde[1] + nmheWorkspace.acVL[16]*nmheWorkspace.acHTilde[2] + nmheWorkspace.acVL[24]*nmheWorkspace.acHTilde[3] + nmheWorkspace.acVL[32]*nmheWorkspace.acHTilde[4] + nmheWorkspace.acVL[40]*nmheWorkspace.acHTilde[5] + nmheWorkspace.acVL[48]*nmheWorkspace.acHTilde[6] + nmheWorkspace.acVL[56]*nmheWorkspace.acHTilde[7];
nmheWorkspace.acb[9] -= + nmheWorkspace.acVL[1]*nmheWorkspace.acHTilde[0] + nmheWorkspace.acVL[9]*nmheWorkspace.acHTilde[1] + nmheWorkspace.acVL[17]*nmheWorkspace.acHTilde[2] + nmheWorkspace.acVL[25]*nmheWorkspace.acHTilde[3] + nmheWorkspace.acVL[33]*nmheWorkspace.acHTilde[4] + nmheWorkspace.acVL[41]*nmheWorkspace.acHTilde[5] + nmheWorkspace.acVL[49]*nmheWorkspace.acHTilde[6] + nmheWorkspace.acVL[57]*nmheWorkspace.acHTilde[7];
nmheWorkspace.acb[10] -= + nmheWorkspace.acVL[2]*nmheWorkspace.acHTilde[0] + nmheWorkspace.acVL[10]*nmheWorkspace.acHTilde[1] + nmheWorkspace.acVL[18]*nmheWorkspace.acHTilde[2] + nmheWorkspace.acVL[26]*nmheWorkspace.acHTilde[3] + nmheWorkspace.acVL[34]*nmheWorkspace.acHTilde[4] + nmheWorkspace.acVL[42]*nmheWorkspace.acHTilde[5] + nmheWorkspace.acVL[50]*nmheWorkspace.acHTilde[6] + nmheWorkspace.acVL[58]*nmheWorkspace.acHTilde[7];
nmheWorkspace.acb[11] -= + nmheWorkspace.acVL[3]*nmheWorkspace.acHTilde[0] + nmheWorkspace.acVL[11]*nmheWorkspace.acHTilde[1] + nmheWorkspace.acVL[19]*nmheWorkspace.acHTilde[2] + nmheWorkspace.acVL[27]*nmheWorkspace.acHTilde[3] + nmheWorkspace.acVL[35]*nmheWorkspace.acHTilde[4] + nmheWorkspace.acVL[43]*nmheWorkspace.acHTilde[5] + nmheWorkspace.acVL[51]*nmheWorkspace.acHTilde[6] + nmheWorkspace.acVL[59]*nmheWorkspace.acHTilde[7];
nmheWorkspace.acb[12] -= + nmheWorkspace.acVL[4]*nmheWorkspace.acHTilde[0] + nmheWorkspace.acVL[12]*nmheWorkspace.acHTilde[1] + nmheWorkspace.acVL[20]*nmheWorkspace.acHTilde[2] + nmheWorkspace.acVL[28]*nmheWorkspace.acHTilde[3] + nmheWorkspace.acVL[36]*nmheWorkspace.acHTilde[4] + nmheWorkspace.acVL[44]*nmheWorkspace.acHTilde[5] + nmheWorkspace.acVL[52]*nmheWorkspace.acHTilde[6] + nmheWorkspace.acVL[60]*nmheWorkspace.acHTilde[7];
nmheWorkspace.acb[13] -= + nmheWorkspace.acVL[5]*nmheWorkspace.acHTilde[0] + nmheWorkspace.acVL[13]*nmheWorkspace.acHTilde[1] + nmheWorkspace.acVL[21]*nmheWorkspace.acHTilde[2] + nmheWorkspace.acVL[29]*nmheWorkspace.acHTilde[3] + nmheWorkspace.acVL[37]*nmheWorkspace.acHTilde[4] + nmheWorkspace.acVL[45]*nmheWorkspace.acHTilde[5] + nmheWorkspace.acVL[53]*nmheWorkspace.acHTilde[6] + nmheWorkspace.acVL[61]*nmheWorkspace.acHTilde[7];
nmheWorkspace.acb[14] -= + nmheWorkspace.acVL[6]*nmheWorkspace.acHTilde[0] + nmheWorkspace.acVL[14]*nmheWorkspace.acHTilde[1] + nmheWorkspace.acVL[22]*nmheWorkspace.acHTilde[2] + nmheWorkspace.acVL[30]*nmheWorkspace.acHTilde[3] + nmheWorkspace.acVL[38]*nmheWorkspace.acHTilde[4] + nmheWorkspace.acVL[46]*nmheWorkspace.acHTilde[5] + nmheWorkspace.acVL[54]*nmheWorkspace.acHTilde[6] + nmheWorkspace.acVL[62]*nmheWorkspace.acHTilde[7];
nmheWorkspace.acb[15] -= + nmheWorkspace.acVL[7]*nmheWorkspace.acHTilde[0] + nmheWorkspace.acVL[15]*nmheWorkspace.acHTilde[1] + nmheWorkspace.acVL[23]*nmheWorkspace.acHTilde[2] + nmheWorkspace.acVL[31]*nmheWorkspace.acHTilde[3] + nmheWorkspace.acVL[39]*nmheWorkspace.acHTilde[4] + nmheWorkspace.acVL[47]*nmheWorkspace.acHTilde[5] + nmheWorkspace.acVL[55]*nmheWorkspace.acHTilde[6] + nmheWorkspace.acVL[63]*nmheWorkspace.acHTilde[7];
nmheWorkspace.acb[16] = + nmheVariables.WL[0]*nmheWorkspace.acXTilde[0] + nmheVariables.WL[8]*nmheWorkspace.acXTilde[1] + nmheVariables.WL[16]*nmheWorkspace.acXTilde[2] + nmheVariables.WL[24]*nmheWorkspace.acXTilde[3] + nmheVariables.WL[32]*nmheWorkspace.acXTilde[4] + nmheVariables.WL[40]*nmheWorkspace.acXTilde[5] + nmheVariables.WL[48]*nmheWorkspace.acXTilde[6] + nmheVariables.WL[56]*nmheWorkspace.acXTilde[7];
nmheWorkspace.acb[17] = + nmheVariables.WL[1]*nmheWorkspace.acXTilde[0] + nmheVariables.WL[9]*nmheWorkspace.acXTilde[1] + nmheVariables.WL[17]*nmheWorkspace.acXTilde[2] + nmheVariables.WL[25]*nmheWorkspace.acXTilde[3] + nmheVariables.WL[33]*nmheWorkspace.acXTilde[4] + nmheVariables.WL[41]*nmheWorkspace.acXTilde[5] + nmheVariables.WL[49]*nmheWorkspace.acXTilde[6] + nmheVariables.WL[57]*nmheWorkspace.acXTilde[7];
nmheWorkspace.acb[18] = + nmheVariables.WL[2]*nmheWorkspace.acXTilde[0] + nmheVariables.WL[10]*nmheWorkspace.acXTilde[1] + nmheVariables.WL[18]*nmheWorkspace.acXTilde[2] + nmheVariables.WL[26]*nmheWorkspace.acXTilde[3] + nmheVariables.WL[34]*nmheWorkspace.acXTilde[4] + nmheVariables.WL[42]*nmheWorkspace.acXTilde[5] + nmheVariables.WL[50]*nmheWorkspace.acXTilde[6] + nmheVariables.WL[58]*nmheWorkspace.acXTilde[7];
nmheWorkspace.acb[19] = + nmheVariables.WL[3]*nmheWorkspace.acXTilde[0] + nmheVariables.WL[11]*nmheWorkspace.acXTilde[1] + nmheVariables.WL[19]*nmheWorkspace.acXTilde[2] + nmheVariables.WL[27]*nmheWorkspace.acXTilde[3] + nmheVariables.WL[35]*nmheWorkspace.acXTilde[4] + nmheVariables.WL[43]*nmheWorkspace.acXTilde[5] + nmheVariables.WL[51]*nmheWorkspace.acXTilde[6] + nmheVariables.WL[59]*nmheWorkspace.acXTilde[7];
nmheWorkspace.acb[20] = + nmheVariables.WL[4]*nmheWorkspace.acXTilde[0] + nmheVariables.WL[12]*nmheWorkspace.acXTilde[1] + nmheVariables.WL[20]*nmheWorkspace.acXTilde[2] + nmheVariables.WL[28]*nmheWorkspace.acXTilde[3] + nmheVariables.WL[36]*nmheWorkspace.acXTilde[4] + nmheVariables.WL[44]*nmheWorkspace.acXTilde[5] + nmheVariables.WL[52]*nmheWorkspace.acXTilde[6] + nmheVariables.WL[60]*nmheWorkspace.acXTilde[7];
nmheWorkspace.acb[21] = + nmheVariables.WL[5]*nmheWorkspace.acXTilde[0] + nmheVariables.WL[13]*nmheWorkspace.acXTilde[1] + nmheVariables.WL[21]*nmheWorkspace.acXTilde[2] + nmheVariables.WL[29]*nmheWorkspace.acXTilde[3] + nmheVariables.WL[37]*nmheWorkspace.acXTilde[4] + nmheVariables.WL[45]*nmheWorkspace.acXTilde[5] + nmheVariables.WL[53]*nmheWorkspace.acXTilde[6] + nmheVariables.WL[61]*nmheWorkspace.acXTilde[7];
nmheWorkspace.acb[22] = + nmheVariables.WL[6]*nmheWorkspace.acXTilde[0] + nmheVariables.WL[14]*nmheWorkspace.acXTilde[1] + nmheVariables.WL[22]*nmheWorkspace.acXTilde[2] + nmheVariables.WL[30]*nmheWorkspace.acXTilde[3] + nmheVariables.WL[38]*nmheWorkspace.acXTilde[4] + nmheVariables.WL[46]*nmheWorkspace.acXTilde[5] + nmheVariables.WL[54]*nmheWorkspace.acXTilde[6] + nmheVariables.WL[62]*nmheWorkspace.acXTilde[7];
nmheWorkspace.acb[23] = + nmheVariables.WL[7]*nmheWorkspace.acXTilde[0] + nmheVariables.WL[15]*nmheWorkspace.acXTilde[1] + nmheVariables.WL[23]*nmheWorkspace.acXTilde[2] + nmheVariables.WL[31]*nmheWorkspace.acXTilde[3] + nmheVariables.WL[39]*nmheWorkspace.acXTilde[4] + nmheVariables.WL[47]*nmheWorkspace.acXTilde[5] + nmheVariables.WL[55]*nmheWorkspace.acXTilde[6] + nmheVariables.WL[63]*nmheWorkspace.acXTilde[7];
nmhe_solve_acsystem( nmheWorkspace.acA, nmheWorkspace.acb, nmheWorkspace.rk_actemp );
nmheVariables.xAC[0] = nmheWorkspace.acb[10];
nmheVariables.xAC[1] = nmheWorkspace.acb[11];
nmheVariables.xAC[2] = nmheWorkspace.acb[12];
nmheVariables.xAC[3] = nmheWorkspace.acb[13];
nmheVariables.xAC[4] = nmheWorkspace.acb[14];
nmheVariables.xAC[5] = nmheWorkspace.acb[15];
nmheVariables.xAC[6] = nmheWorkspace.acb[16];
nmheVariables.xAC[7] = nmheWorkspace.acb[17];
nmheWorkspace.acP[0] = nmheWorkspace.acA[190];
nmheWorkspace.acP[1] = nmheWorkspace.acA[191];
nmheWorkspace.acP[2] = nmheWorkspace.acA[192];
nmheWorkspace.acP[3] = nmheWorkspace.acA[193];
nmheWorkspace.acP[4] = nmheWorkspace.acA[194];
nmheWorkspace.acP[5] = nmheWorkspace.acA[195];
nmheWorkspace.acP[6] = nmheWorkspace.acA[196];
nmheWorkspace.acP[7] = nmheWorkspace.acA[197];
nmheWorkspace.acP[9] = nmheWorkspace.acA[209];
nmheWorkspace.acP[10] = nmheWorkspace.acA[210];
nmheWorkspace.acP[11] = nmheWorkspace.acA[211];
nmheWorkspace.acP[12] = nmheWorkspace.acA[212];
nmheWorkspace.acP[13] = nmheWorkspace.acA[213];
nmheWorkspace.acP[14] = nmheWorkspace.acA[214];
nmheWorkspace.acP[15] = nmheWorkspace.acA[215];
nmheWorkspace.acP[18] = nmheWorkspace.acA[228];
nmheWorkspace.acP[19] = nmheWorkspace.acA[229];
nmheWorkspace.acP[20] = nmheWorkspace.acA[230];
nmheWorkspace.acP[21] = nmheWorkspace.acA[231];
nmheWorkspace.acP[22] = nmheWorkspace.acA[232];
nmheWorkspace.acP[23] = nmheWorkspace.acA[233];
nmheWorkspace.acP[27] = nmheWorkspace.acA[247];
nmheWorkspace.acP[28] = nmheWorkspace.acA[248];
nmheWorkspace.acP[29] = nmheWorkspace.acA[249];
nmheWorkspace.acP[30] = nmheWorkspace.acA[250];
nmheWorkspace.acP[31] = nmheWorkspace.acA[251];
nmheWorkspace.acP[36] = nmheWorkspace.acA[266];
nmheWorkspace.acP[37] = nmheWorkspace.acA[267];
nmheWorkspace.acP[38] = nmheWorkspace.acA[268];
nmheWorkspace.acP[39] = nmheWorkspace.acA[269];
nmheWorkspace.acP[45] = nmheWorkspace.acA[285];
nmheWorkspace.acP[46] = nmheWorkspace.acA[286];
nmheWorkspace.acP[47] = nmheWorkspace.acA[287];
nmheWorkspace.acP[54] = nmheWorkspace.acA[304];
nmheWorkspace.acP[55] = nmheWorkspace.acA[305];
nmheWorkspace.acP[63] = nmheWorkspace.acA[323];
nmheVariables.SAC[0] = + nmheWorkspace.acP[0]*nmheWorkspace.acP[0] + nmheWorkspace.acP[8]*nmheWorkspace.acP[8] + nmheWorkspace.acP[16]*nmheWorkspace.acP[16] + nmheWorkspace.acP[24]*nmheWorkspace.acP[24] + nmheWorkspace.acP[32]*nmheWorkspace.acP[32] + nmheWorkspace.acP[40]*nmheWorkspace.acP[40] + nmheWorkspace.acP[48]*nmheWorkspace.acP[48] + nmheWorkspace.acP[56]*nmheWorkspace.acP[56];
nmheVariables.SAC[1] = + nmheWorkspace.acP[0]*nmheWorkspace.acP[1] + nmheWorkspace.acP[8]*nmheWorkspace.acP[9] + nmheWorkspace.acP[16]*nmheWorkspace.acP[17] + nmheWorkspace.acP[24]*nmheWorkspace.acP[25] + nmheWorkspace.acP[32]*nmheWorkspace.acP[33] + nmheWorkspace.acP[40]*nmheWorkspace.acP[41] + nmheWorkspace.acP[48]*nmheWorkspace.acP[49] + nmheWorkspace.acP[56]*nmheWorkspace.acP[57];
nmheVariables.SAC[2] = + nmheWorkspace.acP[0]*nmheWorkspace.acP[2] + nmheWorkspace.acP[8]*nmheWorkspace.acP[10] + nmheWorkspace.acP[16]*nmheWorkspace.acP[18] + nmheWorkspace.acP[24]*nmheWorkspace.acP[26] + nmheWorkspace.acP[32]*nmheWorkspace.acP[34] + nmheWorkspace.acP[40]*nmheWorkspace.acP[42] + nmheWorkspace.acP[48]*nmheWorkspace.acP[50] + nmheWorkspace.acP[56]*nmheWorkspace.acP[58];
nmheVariables.SAC[3] = + nmheWorkspace.acP[0]*nmheWorkspace.acP[3] + nmheWorkspace.acP[8]*nmheWorkspace.acP[11] + nmheWorkspace.acP[16]*nmheWorkspace.acP[19] + nmheWorkspace.acP[24]*nmheWorkspace.acP[27] + nmheWorkspace.acP[32]*nmheWorkspace.acP[35] + nmheWorkspace.acP[40]*nmheWorkspace.acP[43] + nmheWorkspace.acP[48]*nmheWorkspace.acP[51] + nmheWorkspace.acP[56]*nmheWorkspace.acP[59];
nmheVariables.SAC[4] = + nmheWorkspace.acP[0]*nmheWorkspace.acP[4] + nmheWorkspace.acP[8]*nmheWorkspace.acP[12] + nmheWorkspace.acP[16]*nmheWorkspace.acP[20] + nmheWorkspace.acP[24]*nmheWorkspace.acP[28] + nmheWorkspace.acP[32]*nmheWorkspace.acP[36] + nmheWorkspace.acP[40]*nmheWorkspace.acP[44] + nmheWorkspace.acP[48]*nmheWorkspace.acP[52] + nmheWorkspace.acP[56]*nmheWorkspace.acP[60];
nmheVariables.SAC[5] = + nmheWorkspace.acP[0]*nmheWorkspace.acP[5] + nmheWorkspace.acP[8]*nmheWorkspace.acP[13] + nmheWorkspace.acP[16]*nmheWorkspace.acP[21] + nmheWorkspace.acP[24]*nmheWorkspace.acP[29] + nmheWorkspace.acP[32]*nmheWorkspace.acP[37] + nmheWorkspace.acP[40]*nmheWorkspace.acP[45] + nmheWorkspace.acP[48]*nmheWorkspace.acP[53] + nmheWorkspace.acP[56]*nmheWorkspace.acP[61];
nmheVariables.SAC[6] = + nmheWorkspace.acP[0]*nmheWorkspace.acP[6] + nmheWorkspace.acP[8]*nmheWorkspace.acP[14] + nmheWorkspace.acP[16]*nmheWorkspace.acP[22] + nmheWorkspace.acP[24]*nmheWorkspace.acP[30] + nmheWorkspace.acP[32]*nmheWorkspace.acP[38] + nmheWorkspace.acP[40]*nmheWorkspace.acP[46] + nmheWorkspace.acP[48]*nmheWorkspace.acP[54] + nmheWorkspace.acP[56]*nmheWorkspace.acP[62];
nmheVariables.SAC[7] = + nmheWorkspace.acP[0]*nmheWorkspace.acP[7] + nmheWorkspace.acP[8]*nmheWorkspace.acP[15] + nmheWorkspace.acP[16]*nmheWorkspace.acP[23] + nmheWorkspace.acP[24]*nmheWorkspace.acP[31] + nmheWorkspace.acP[32]*nmheWorkspace.acP[39] + nmheWorkspace.acP[40]*nmheWorkspace.acP[47] + nmheWorkspace.acP[48]*nmheWorkspace.acP[55] + nmheWorkspace.acP[56]*nmheWorkspace.acP[63];
nmheVariables.SAC[8] = + nmheWorkspace.acP[1]*nmheWorkspace.acP[0] + nmheWorkspace.acP[9]*nmheWorkspace.acP[8] + nmheWorkspace.acP[17]*nmheWorkspace.acP[16] + nmheWorkspace.acP[25]*nmheWorkspace.acP[24] + nmheWorkspace.acP[33]*nmheWorkspace.acP[32] + nmheWorkspace.acP[41]*nmheWorkspace.acP[40] + nmheWorkspace.acP[49]*nmheWorkspace.acP[48] + nmheWorkspace.acP[57]*nmheWorkspace.acP[56];
nmheVariables.SAC[9] = + nmheWorkspace.acP[1]*nmheWorkspace.acP[1] + nmheWorkspace.acP[9]*nmheWorkspace.acP[9] + nmheWorkspace.acP[17]*nmheWorkspace.acP[17] + nmheWorkspace.acP[25]*nmheWorkspace.acP[25] + nmheWorkspace.acP[33]*nmheWorkspace.acP[33] + nmheWorkspace.acP[41]*nmheWorkspace.acP[41] + nmheWorkspace.acP[49]*nmheWorkspace.acP[49] + nmheWorkspace.acP[57]*nmheWorkspace.acP[57];
nmheVariables.SAC[10] = + nmheWorkspace.acP[1]*nmheWorkspace.acP[2] + nmheWorkspace.acP[9]*nmheWorkspace.acP[10] + nmheWorkspace.acP[17]*nmheWorkspace.acP[18] + nmheWorkspace.acP[25]*nmheWorkspace.acP[26] + nmheWorkspace.acP[33]*nmheWorkspace.acP[34] + nmheWorkspace.acP[41]*nmheWorkspace.acP[42] + nmheWorkspace.acP[49]*nmheWorkspace.acP[50] + nmheWorkspace.acP[57]*nmheWorkspace.acP[58];
nmheVariables.SAC[11] = + nmheWorkspace.acP[1]*nmheWorkspace.acP[3] + nmheWorkspace.acP[9]*nmheWorkspace.acP[11] + nmheWorkspace.acP[17]*nmheWorkspace.acP[19] + nmheWorkspace.acP[25]*nmheWorkspace.acP[27] + nmheWorkspace.acP[33]*nmheWorkspace.acP[35] + nmheWorkspace.acP[41]*nmheWorkspace.acP[43] + nmheWorkspace.acP[49]*nmheWorkspace.acP[51] + nmheWorkspace.acP[57]*nmheWorkspace.acP[59];
nmheVariables.SAC[12] = + nmheWorkspace.acP[1]*nmheWorkspace.acP[4] + nmheWorkspace.acP[9]*nmheWorkspace.acP[12] + nmheWorkspace.acP[17]*nmheWorkspace.acP[20] + nmheWorkspace.acP[25]*nmheWorkspace.acP[28] + nmheWorkspace.acP[33]*nmheWorkspace.acP[36] + nmheWorkspace.acP[41]*nmheWorkspace.acP[44] + nmheWorkspace.acP[49]*nmheWorkspace.acP[52] + nmheWorkspace.acP[57]*nmheWorkspace.acP[60];
nmheVariables.SAC[13] = + nmheWorkspace.acP[1]*nmheWorkspace.acP[5] + nmheWorkspace.acP[9]*nmheWorkspace.acP[13] + nmheWorkspace.acP[17]*nmheWorkspace.acP[21] + nmheWorkspace.acP[25]*nmheWorkspace.acP[29] + nmheWorkspace.acP[33]*nmheWorkspace.acP[37] + nmheWorkspace.acP[41]*nmheWorkspace.acP[45] + nmheWorkspace.acP[49]*nmheWorkspace.acP[53] + nmheWorkspace.acP[57]*nmheWorkspace.acP[61];
nmheVariables.SAC[14] = + nmheWorkspace.acP[1]*nmheWorkspace.acP[6] + nmheWorkspace.acP[9]*nmheWorkspace.acP[14] + nmheWorkspace.acP[17]*nmheWorkspace.acP[22] + nmheWorkspace.acP[25]*nmheWorkspace.acP[30] + nmheWorkspace.acP[33]*nmheWorkspace.acP[38] + nmheWorkspace.acP[41]*nmheWorkspace.acP[46] + nmheWorkspace.acP[49]*nmheWorkspace.acP[54] + nmheWorkspace.acP[57]*nmheWorkspace.acP[62];
nmheVariables.SAC[15] = + nmheWorkspace.acP[1]*nmheWorkspace.acP[7] + nmheWorkspace.acP[9]*nmheWorkspace.acP[15] + nmheWorkspace.acP[17]*nmheWorkspace.acP[23] + nmheWorkspace.acP[25]*nmheWorkspace.acP[31] + nmheWorkspace.acP[33]*nmheWorkspace.acP[39] + nmheWorkspace.acP[41]*nmheWorkspace.acP[47] + nmheWorkspace.acP[49]*nmheWorkspace.acP[55] + nmheWorkspace.acP[57]*nmheWorkspace.acP[63];
nmheVariables.SAC[16] = + nmheWorkspace.acP[2]*nmheWorkspace.acP[0] + nmheWorkspace.acP[10]*nmheWorkspace.acP[8] + nmheWorkspace.acP[18]*nmheWorkspace.acP[16] + nmheWorkspace.acP[26]*nmheWorkspace.acP[24] + nmheWorkspace.acP[34]*nmheWorkspace.acP[32] + nmheWorkspace.acP[42]*nmheWorkspace.acP[40] + nmheWorkspace.acP[50]*nmheWorkspace.acP[48] + nmheWorkspace.acP[58]*nmheWorkspace.acP[56];
nmheVariables.SAC[17] = + nmheWorkspace.acP[2]*nmheWorkspace.acP[1] + nmheWorkspace.acP[10]*nmheWorkspace.acP[9] + nmheWorkspace.acP[18]*nmheWorkspace.acP[17] + nmheWorkspace.acP[26]*nmheWorkspace.acP[25] + nmheWorkspace.acP[34]*nmheWorkspace.acP[33] + nmheWorkspace.acP[42]*nmheWorkspace.acP[41] + nmheWorkspace.acP[50]*nmheWorkspace.acP[49] + nmheWorkspace.acP[58]*nmheWorkspace.acP[57];
nmheVariables.SAC[18] = + nmheWorkspace.acP[2]*nmheWorkspace.acP[2] + nmheWorkspace.acP[10]*nmheWorkspace.acP[10] + nmheWorkspace.acP[18]*nmheWorkspace.acP[18] + nmheWorkspace.acP[26]*nmheWorkspace.acP[26] + nmheWorkspace.acP[34]*nmheWorkspace.acP[34] + nmheWorkspace.acP[42]*nmheWorkspace.acP[42] + nmheWorkspace.acP[50]*nmheWorkspace.acP[50] + nmheWorkspace.acP[58]*nmheWorkspace.acP[58];
nmheVariables.SAC[19] = + nmheWorkspace.acP[2]*nmheWorkspace.acP[3] + nmheWorkspace.acP[10]*nmheWorkspace.acP[11] + nmheWorkspace.acP[18]*nmheWorkspace.acP[19] + nmheWorkspace.acP[26]*nmheWorkspace.acP[27] + nmheWorkspace.acP[34]*nmheWorkspace.acP[35] + nmheWorkspace.acP[42]*nmheWorkspace.acP[43] + nmheWorkspace.acP[50]*nmheWorkspace.acP[51] + nmheWorkspace.acP[58]*nmheWorkspace.acP[59];
nmheVariables.SAC[20] = + nmheWorkspace.acP[2]*nmheWorkspace.acP[4] + nmheWorkspace.acP[10]*nmheWorkspace.acP[12] + nmheWorkspace.acP[18]*nmheWorkspace.acP[20] + nmheWorkspace.acP[26]*nmheWorkspace.acP[28] + nmheWorkspace.acP[34]*nmheWorkspace.acP[36] + nmheWorkspace.acP[42]*nmheWorkspace.acP[44] + nmheWorkspace.acP[50]*nmheWorkspace.acP[52] + nmheWorkspace.acP[58]*nmheWorkspace.acP[60];
nmheVariables.SAC[21] = + nmheWorkspace.acP[2]*nmheWorkspace.acP[5] + nmheWorkspace.acP[10]*nmheWorkspace.acP[13] + nmheWorkspace.acP[18]*nmheWorkspace.acP[21] + nmheWorkspace.acP[26]*nmheWorkspace.acP[29] + nmheWorkspace.acP[34]*nmheWorkspace.acP[37] + nmheWorkspace.acP[42]*nmheWorkspace.acP[45] + nmheWorkspace.acP[50]*nmheWorkspace.acP[53] + nmheWorkspace.acP[58]*nmheWorkspace.acP[61];
nmheVariables.SAC[22] = + nmheWorkspace.acP[2]*nmheWorkspace.acP[6] + nmheWorkspace.acP[10]*nmheWorkspace.acP[14] + nmheWorkspace.acP[18]*nmheWorkspace.acP[22] + nmheWorkspace.acP[26]*nmheWorkspace.acP[30] + nmheWorkspace.acP[34]*nmheWorkspace.acP[38] + nmheWorkspace.acP[42]*nmheWorkspace.acP[46] + nmheWorkspace.acP[50]*nmheWorkspace.acP[54] + nmheWorkspace.acP[58]*nmheWorkspace.acP[62];
nmheVariables.SAC[23] = + nmheWorkspace.acP[2]*nmheWorkspace.acP[7] + nmheWorkspace.acP[10]*nmheWorkspace.acP[15] + nmheWorkspace.acP[18]*nmheWorkspace.acP[23] + nmheWorkspace.acP[26]*nmheWorkspace.acP[31] + nmheWorkspace.acP[34]*nmheWorkspace.acP[39] + nmheWorkspace.acP[42]*nmheWorkspace.acP[47] + nmheWorkspace.acP[50]*nmheWorkspace.acP[55] + nmheWorkspace.acP[58]*nmheWorkspace.acP[63];
nmheVariables.SAC[24] = + nmheWorkspace.acP[3]*nmheWorkspace.acP[0] + nmheWorkspace.acP[11]*nmheWorkspace.acP[8] + nmheWorkspace.acP[19]*nmheWorkspace.acP[16] + nmheWorkspace.acP[27]*nmheWorkspace.acP[24] + nmheWorkspace.acP[35]*nmheWorkspace.acP[32] + nmheWorkspace.acP[43]*nmheWorkspace.acP[40] + nmheWorkspace.acP[51]*nmheWorkspace.acP[48] + nmheWorkspace.acP[59]*nmheWorkspace.acP[56];
nmheVariables.SAC[25] = + nmheWorkspace.acP[3]*nmheWorkspace.acP[1] + nmheWorkspace.acP[11]*nmheWorkspace.acP[9] + nmheWorkspace.acP[19]*nmheWorkspace.acP[17] + nmheWorkspace.acP[27]*nmheWorkspace.acP[25] + nmheWorkspace.acP[35]*nmheWorkspace.acP[33] + nmheWorkspace.acP[43]*nmheWorkspace.acP[41] + nmheWorkspace.acP[51]*nmheWorkspace.acP[49] + nmheWorkspace.acP[59]*nmheWorkspace.acP[57];
nmheVariables.SAC[26] = + nmheWorkspace.acP[3]*nmheWorkspace.acP[2] + nmheWorkspace.acP[11]*nmheWorkspace.acP[10] + nmheWorkspace.acP[19]*nmheWorkspace.acP[18] + nmheWorkspace.acP[27]*nmheWorkspace.acP[26] + nmheWorkspace.acP[35]*nmheWorkspace.acP[34] + nmheWorkspace.acP[43]*nmheWorkspace.acP[42] + nmheWorkspace.acP[51]*nmheWorkspace.acP[50] + nmheWorkspace.acP[59]*nmheWorkspace.acP[58];
nmheVariables.SAC[27] = + nmheWorkspace.acP[3]*nmheWorkspace.acP[3] + nmheWorkspace.acP[11]*nmheWorkspace.acP[11] + nmheWorkspace.acP[19]*nmheWorkspace.acP[19] + nmheWorkspace.acP[27]*nmheWorkspace.acP[27] + nmheWorkspace.acP[35]*nmheWorkspace.acP[35] + nmheWorkspace.acP[43]*nmheWorkspace.acP[43] + nmheWorkspace.acP[51]*nmheWorkspace.acP[51] + nmheWorkspace.acP[59]*nmheWorkspace.acP[59];
nmheVariables.SAC[28] = + nmheWorkspace.acP[3]*nmheWorkspace.acP[4] + nmheWorkspace.acP[11]*nmheWorkspace.acP[12] + nmheWorkspace.acP[19]*nmheWorkspace.acP[20] + nmheWorkspace.acP[27]*nmheWorkspace.acP[28] + nmheWorkspace.acP[35]*nmheWorkspace.acP[36] + nmheWorkspace.acP[43]*nmheWorkspace.acP[44] + nmheWorkspace.acP[51]*nmheWorkspace.acP[52] + nmheWorkspace.acP[59]*nmheWorkspace.acP[60];
nmheVariables.SAC[29] = + nmheWorkspace.acP[3]*nmheWorkspace.acP[5] + nmheWorkspace.acP[11]*nmheWorkspace.acP[13] + nmheWorkspace.acP[19]*nmheWorkspace.acP[21] + nmheWorkspace.acP[27]*nmheWorkspace.acP[29] + nmheWorkspace.acP[35]*nmheWorkspace.acP[37] + nmheWorkspace.acP[43]*nmheWorkspace.acP[45] + nmheWorkspace.acP[51]*nmheWorkspace.acP[53] + nmheWorkspace.acP[59]*nmheWorkspace.acP[61];
nmheVariables.SAC[30] = + nmheWorkspace.acP[3]*nmheWorkspace.acP[6] + nmheWorkspace.acP[11]*nmheWorkspace.acP[14] + nmheWorkspace.acP[19]*nmheWorkspace.acP[22] + nmheWorkspace.acP[27]*nmheWorkspace.acP[30] + nmheWorkspace.acP[35]*nmheWorkspace.acP[38] + nmheWorkspace.acP[43]*nmheWorkspace.acP[46] + nmheWorkspace.acP[51]*nmheWorkspace.acP[54] + nmheWorkspace.acP[59]*nmheWorkspace.acP[62];
nmheVariables.SAC[31] = + nmheWorkspace.acP[3]*nmheWorkspace.acP[7] + nmheWorkspace.acP[11]*nmheWorkspace.acP[15] + nmheWorkspace.acP[19]*nmheWorkspace.acP[23] + nmheWorkspace.acP[27]*nmheWorkspace.acP[31] + nmheWorkspace.acP[35]*nmheWorkspace.acP[39] + nmheWorkspace.acP[43]*nmheWorkspace.acP[47] + nmheWorkspace.acP[51]*nmheWorkspace.acP[55] + nmheWorkspace.acP[59]*nmheWorkspace.acP[63];
nmheVariables.SAC[32] = + nmheWorkspace.acP[4]*nmheWorkspace.acP[0] + nmheWorkspace.acP[12]*nmheWorkspace.acP[8] + nmheWorkspace.acP[20]*nmheWorkspace.acP[16] + nmheWorkspace.acP[28]*nmheWorkspace.acP[24] + nmheWorkspace.acP[36]*nmheWorkspace.acP[32] + nmheWorkspace.acP[44]*nmheWorkspace.acP[40] + nmheWorkspace.acP[52]*nmheWorkspace.acP[48] + nmheWorkspace.acP[60]*nmheWorkspace.acP[56];
nmheVariables.SAC[33] = + nmheWorkspace.acP[4]*nmheWorkspace.acP[1] + nmheWorkspace.acP[12]*nmheWorkspace.acP[9] + nmheWorkspace.acP[20]*nmheWorkspace.acP[17] + nmheWorkspace.acP[28]*nmheWorkspace.acP[25] + nmheWorkspace.acP[36]*nmheWorkspace.acP[33] + nmheWorkspace.acP[44]*nmheWorkspace.acP[41] + nmheWorkspace.acP[52]*nmheWorkspace.acP[49] + nmheWorkspace.acP[60]*nmheWorkspace.acP[57];
nmheVariables.SAC[34] = + nmheWorkspace.acP[4]*nmheWorkspace.acP[2] + nmheWorkspace.acP[12]*nmheWorkspace.acP[10] + nmheWorkspace.acP[20]*nmheWorkspace.acP[18] + nmheWorkspace.acP[28]*nmheWorkspace.acP[26] + nmheWorkspace.acP[36]*nmheWorkspace.acP[34] + nmheWorkspace.acP[44]*nmheWorkspace.acP[42] + nmheWorkspace.acP[52]*nmheWorkspace.acP[50] + nmheWorkspace.acP[60]*nmheWorkspace.acP[58];
nmheVariables.SAC[35] = + nmheWorkspace.acP[4]*nmheWorkspace.acP[3] + nmheWorkspace.acP[12]*nmheWorkspace.acP[11] + nmheWorkspace.acP[20]*nmheWorkspace.acP[19] + nmheWorkspace.acP[28]*nmheWorkspace.acP[27] + nmheWorkspace.acP[36]*nmheWorkspace.acP[35] + nmheWorkspace.acP[44]*nmheWorkspace.acP[43] + nmheWorkspace.acP[52]*nmheWorkspace.acP[51] + nmheWorkspace.acP[60]*nmheWorkspace.acP[59];
nmheVariables.SAC[36] = + nmheWorkspace.acP[4]*nmheWorkspace.acP[4] + nmheWorkspace.acP[12]*nmheWorkspace.acP[12] + nmheWorkspace.acP[20]*nmheWorkspace.acP[20] + nmheWorkspace.acP[28]*nmheWorkspace.acP[28] + nmheWorkspace.acP[36]*nmheWorkspace.acP[36] + nmheWorkspace.acP[44]*nmheWorkspace.acP[44] + nmheWorkspace.acP[52]*nmheWorkspace.acP[52] + nmheWorkspace.acP[60]*nmheWorkspace.acP[60];
nmheVariables.SAC[37] = + nmheWorkspace.acP[4]*nmheWorkspace.acP[5] + nmheWorkspace.acP[12]*nmheWorkspace.acP[13] + nmheWorkspace.acP[20]*nmheWorkspace.acP[21] + nmheWorkspace.acP[28]*nmheWorkspace.acP[29] + nmheWorkspace.acP[36]*nmheWorkspace.acP[37] + nmheWorkspace.acP[44]*nmheWorkspace.acP[45] + nmheWorkspace.acP[52]*nmheWorkspace.acP[53] + nmheWorkspace.acP[60]*nmheWorkspace.acP[61];
nmheVariables.SAC[38] = + nmheWorkspace.acP[4]*nmheWorkspace.acP[6] + nmheWorkspace.acP[12]*nmheWorkspace.acP[14] + nmheWorkspace.acP[20]*nmheWorkspace.acP[22] + nmheWorkspace.acP[28]*nmheWorkspace.acP[30] + nmheWorkspace.acP[36]*nmheWorkspace.acP[38] + nmheWorkspace.acP[44]*nmheWorkspace.acP[46] + nmheWorkspace.acP[52]*nmheWorkspace.acP[54] + nmheWorkspace.acP[60]*nmheWorkspace.acP[62];
nmheVariables.SAC[39] = + nmheWorkspace.acP[4]*nmheWorkspace.acP[7] + nmheWorkspace.acP[12]*nmheWorkspace.acP[15] + nmheWorkspace.acP[20]*nmheWorkspace.acP[23] + nmheWorkspace.acP[28]*nmheWorkspace.acP[31] + nmheWorkspace.acP[36]*nmheWorkspace.acP[39] + nmheWorkspace.acP[44]*nmheWorkspace.acP[47] + nmheWorkspace.acP[52]*nmheWorkspace.acP[55] + nmheWorkspace.acP[60]*nmheWorkspace.acP[63];
nmheVariables.SAC[40] = + nmheWorkspace.acP[5]*nmheWorkspace.acP[0] + nmheWorkspace.acP[13]*nmheWorkspace.acP[8] + nmheWorkspace.acP[21]*nmheWorkspace.acP[16] + nmheWorkspace.acP[29]*nmheWorkspace.acP[24] + nmheWorkspace.acP[37]*nmheWorkspace.acP[32] + nmheWorkspace.acP[45]*nmheWorkspace.acP[40] + nmheWorkspace.acP[53]*nmheWorkspace.acP[48] + nmheWorkspace.acP[61]*nmheWorkspace.acP[56];
nmheVariables.SAC[41] = + nmheWorkspace.acP[5]*nmheWorkspace.acP[1] + nmheWorkspace.acP[13]*nmheWorkspace.acP[9] + nmheWorkspace.acP[21]*nmheWorkspace.acP[17] + nmheWorkspace.acP[29]*nmheWorkspace.acP[25] + nmheWorkspace.acP[37]*nmheWorkspace.acP[33] + nmheWorkspace.acP[45]*nmheWorkspace.acP[41] + nmheWorkspace.acP[53]*nmheWorkspace.acP[49] + nmheWorkspace.acP[61]*nmheWorkspace.acP[57];
nmheVariables.SAC[42] = + nmheWorkspace.acP[5]*nmheWorkspace.acP[2] + nmheWorkspace.acP[13]*nmheWorkspace.acP[10] + nmheWorkspace.acP[21]*nmheWorkspace.acP[18] + nmheWorkspace.acP[29]*nmheWorkspace.acP[26] + nmheWorkspace.acP[37]*nmheWorkspace.acP[34] + nmheWorkspace.acP[45]*nmheWorkspace.acP[42] + nmheWorkspace.acP[53]*nmheWorkspace.acP[50] + nmheWorkspace.acP[61]*nmheWorkspace.acP[58];
nmheVariables.SAC[43] = + nmheWorkspace.acP[5]*nmheWorkspace.acP[3] + nmheWorkspace.acP[13]*nmheWorkspace.acP[11] + nmheWorkspace.acP[21]*nmheWorkspace.acP[19] + nmheWorkspace.acP[29]*nmheWorkspace.acP[27] + nmheWorkspace.acP[37]*nmheWorkspace.acP[35] + nmheWorkspace.acP[45]*nmheWorkspace.acP[43] + nmheWorkspace.acP[53]*nmheWorkspace.acP[51] + nmheWorkspace.acP[61]*nmheWorkspace.acP[59];
nmheVariables.SAC[44] = + nmheWorkspace.acP[5]*nmheWorkspace.acP[4] + nmheWorkspace.acP[13]*nmheWorkspace.acP[12] + nmheWorkspace.acP[21]*nmheWorkspace.acP[20] + nmheWorkspace.acP[29]*nmheWorkspace.acP[28] + nmheWorkspace.acP[37]*nmheWorkspace.acP[36] + nmheWorkspace.acP[45]*nmheWorkspace.acP[44] + nmheWorkspace.acP[53]*nmheWorkspace.acP[52] + nmheWorkspace.acP[61]*nmheWorkspace.acP[60];
nmheVariables.SAC[45] = + nmheWorkspace.acP[5]*nmheWorkspace.acP[5] + nmheWorkspace.acP[13]*nmheWorkspace.acP[13] + nmheWorkspace.acP[21]*nmheWorkspace.acP[21] + nmheWorkspace.acP[29]*nmheWorkspace.acP[29] + nmheWorkspace.acP[37]*nmheWorkspace.acP[37] + nmheWorkspace.acP[45]*nmheWorkspace.acP[45] + nmheWorkspace.acP[53]*nmheWorkspace.acP[53] + nmheWorkspace.acP[61]*nmheWorkspace.acP[61];
nmheVariables.SAC[46] = + nmheWorkspace.acP[5]*nmheWorkspace.acP[6] + nmheWorkspace.acP[13]*nmheWorkspace.acP[14] + nmheWorkspace.acP[21]*nmheWorkspace.acP[22] + nmheWorkspace.acP[29]*nmheWorkspace.acP[30] + nmheWorkspace.acP[37]*nmheWorkspace.acP[38] + nmheWorkspace.acP[45]*nmheWorkspace.acP[46] + nmheWorkspace.acP[53]*nmheWorkspace.acP[54] + nmheWorkspace.acP[61]*nmheWorkspace.acP[62];
nmheVariables.SAC[47] = + nmheWorkspace.acP[5]*nmheWorkspace.acP[7] + nmheWorkspace.acP[13]*nmheWorkspace.acP[15] + nmheWorkspace.acP[21]*nmheWorkspace.acP[23] + nmheWorkspace.acP[29]*nmheWorkspace.acP[31] + nmheWorkspace.acP[37]*nmheWorkspace.acP[39] + nmheWorkspace.acP[45]*nmheWorkspace.acP[47] + nmheWorkspace.acP[53]*nmheWorkspace.acP[55] + nmheWorkspace.acP[61]*nmheWorkspace.acP[63];
nmheVariables.SAC[48] = + nmheWorkspace.acP[6]*nmheWorkspace.acP[0] + nmheWorkspace.acP[14]*nmheWorkspace.acP[8] + nmheWorkspace.acP[22]*nmheWorkspace.acP[16] + nmheWorkspace.acP[30]*nmheWorkspace.acP[24] + nmheWorkspace.acP[38]*nmheWorkspace.acP[32] + nmheWorkspace.acP[46]*nmheWorkspace.acP[40] + nmheWorkspace.acP[54]*nmheWorkspace.acP[48] + nmheWorkspace.acP[62]*nmheWorkspace.acP[56];
nmheVariables.SAC[49] = + nmheWorkspace.acP[6]*nmheWorkspace.acP[1] + nmheWorkspace.acP[14]*nmheWorkspace.acP[9] + nmheWorkspace.acP[22]*nmheWorkspace.acP[17] + nmheWorkspace.acP[30]*nmheWorkspace.acP[25] + nmheWorkspace.acP[38]*nmheWorkspace.acP[33] + nmheWorkspace.acP[46]*nmheWorkspace.acP[41] + nmheWorkspace.acP[54]*nmheWorkspace.acP[49] + nmheWorkspace.acP[62]*nmheWorkspace.acP[57];
nmheVariables.SAC[50] = + nmheWorkspace.acP[6]*nmheWorkspace.acP[2] + nmheWorkspace.acP[14]*nmheWorkspace.acP[10] + nmheWorkspace.acP[22]*nmheWorkspace.acP[18] + nmheWorkspace.acP[30]*nmheWorkspace.acP[26] + nmheWorkspace.acP[38]*nmheWorkspace.acP[34] + nmheWorkspace.acP[46]*nmheWorkspace.acP[42] + nmheWorkspace.acP[54]*nmheWorkspace.acP[50] + nmheWorkspace.acP[62]*nmheWorkspace.acP[58];
nmheVariables.SAC[51] = + nmheWorkspace.acP[6]*nmheWorkspace.acP[3] + nmheWorkspace.acP[14]*nmheWorkspace.acP[11] + nmheWorkspace.acP[22]*nmheWorkspace.acP[19] + nmheWorkspace.acP[30]*nmheWorkspace.acP[27] + nmheWorkspace.acP[38]*nmheWorkspace.acP[35] + nmheWorkspace.acP[46]*nmheWorkspace.acP[43] + nmheWorkspace.acP[54]*nmheWorkspace.acP[51] + nmheWorkspace.acP[62]*nmheWorkspace.acP[59];
nmheVariables.SAC[52] = + nmheWorkspace.acP[6]*nmheWorkspace.acP[4] + nmheWorkspace.acP[14]*nmheWorkspace.acP[12] + nmheWorkspace.acP[22]*nmheWorkspace.acP[20] + nmheWorkspace.acP[30]*nmheWorkspace.acP[28] + nmheWorkspace.acP[38]*nmheWorkspace.acP[36] + nmheWorkspace.acP[46]*nmheWorkspace.acP[44] + nmheWorkspace.acP[54]*nmheWorkspace.acP[52] + nmheWorkspace.acP[62]*nmheWorkspace.acP[60];
nmheVariables.SAC[53] = + nmheWorkspace.acP[6]*nmheWorkspace.acP[5] + nmheWorkspace.acP[14]*nmheWorkspace.acP[13] + nmheWorkspace.acP[22]*nmheWorkspace.acP[21] + nmheWorkspace.acP[30]*nmheWorkspace.acP[29] + nmheWorkspace.acP[38]*nmheWorkspace.acP[37] + nmheWorkspace.acP[46]*nmheWorkspace.acP[45] + nmheWorkspace.acP[54]*nmheWorkspace.acP[53] + nmheWorkspace.acP[62]*nmheWorkspace.acP[61];
nmheVariables.SAC[54] = + nmheWorkspace.acP[6]*nmheWorkspace.acP[6] + nmheWorkspace.acP[14]*nmheWorkspace.acP[14] + nmheWorkspace.acP[22]*nmheWorkspace.acP[22] + nmheWorkspace.acP[30]*nmheWorkspace.acP[30] + nmheWorkspace.acP[38]*nmheWorkspace.acP[38] + nmheWorkspace.acP[46]*nmheWorkspace.acP[46] + nmheWorkspace.acP[54]*nmheWorkspace.acP[54] + nmheWorkspace.acP[62]*nmheWorkspace.acP[62];
nmheVariables.SAC[55] = + nmheWorkspace.acP[6]*nmheWorkspace.acP[7] + nmheWorkspace.acP[14]*nmheWorkspace.acP[15] + nmheWorkspace.acP[22]*nmheWorkspace.acP[23] + nmheWorkspace.acP[30]*nmheWorkspace.acP[31] + nmheWorkspace.acP[38]*nmheWorkspace.acP[39] + nmheWorkspace.acP[46]*nmheWorkspace.acP[47] + nmheWorkspace.acP[54]*nmheWorkspace.acP[55] + nmheWorkspace.acP[62]*nmheWorkspace.acP[63];
nmheVariables.SAC[56] = + nmheWorkspace.acP[7]*nmheWorkspace.acP[0] + nmheWorkspace.acP[15]*nmheWorkspace.acP[8] + nmheWorkspace.acP[23]*nmheWorkspace.acP[16] + nmheWorkspace.acP[31]*nmheWorkspace.acP[24] + nmheWorkspace.acP[39]*nmheWorkspace.acP[32] + nmheWorkspace.acP[47]*nmheWorkspace.acP[40] + nmheWorkspace.acP[55]*nmheWorkspace.acP[48] + nmheWorkspace.acP[63]*nmheWorkspace.acP[56];
nmheVariables.SAC[57] = + nmheWorkspace.acP[7]*nmheWorkspace.acP[1] + nmheWorkspace.acP[15]*nmheWorkspace.acP[9] + nmheWorkspace.acP[23]*nmheWorkspace.acP[17] + nmheWorkspace.acP[31]*nmheWorkspace.acP[25] + nmheWorkspace.acP[39]*nmheWorkspace.acP[33] + nmheWorkspace.acP[47]*nmheWorkspace.acP[41] + nmheWorkspace.acP[55]*nmheWorkspace.acP[49] + nmheWorkspace.acP[63]*nmheWorkspace.acP[57];
nmheVariables.SAC[58] = + nmheWorkspace.acP[7]*nmheWorkspace.acP[2] + nmheWorkspace.acP[15]*nmheWorkspace.acP[10] + nmheWorkspace.acP[23]*nmheWorkspace.acP[18] + nmheWorkspace.acP[31]*nmheWorkspace.acP[26] + nmheWorkspace.acP[39]*nmheWorkspace.acP[34] + nmheWorkspace.acP[47]*nmheWorkspace.acP[42] + nmheWorkspace.acP[55]*nmheWorkspace.acP[50] + nmheWorkspace.acP[63]*nmheWorkspace.acP[58];
nmheVariables.SAC[59] = + nmheWorkspace.acP[7]*nmheWorkspace.acP[3] + nmheWorkspace.acP[15]*nmheWorkspace.acP[11] + nmheWorkspace.acP[23]*nmheWorkspace.acP[19] + nmheWorkspace.acP[31]*nmheWorkspace.acP[27] + nmheWorkspace.acP[39]*nmheWorkspace.acP[35] + nmheWorkspace.acP[47]*nmheWorkspace.acP[43] + nmheWorkspace.acP[55]*nmheWorkspace.acP[51] + nmheWorkspace.acP[63]*nmheWorkspace.acP[59];
nmheVariables.SAC[60] = + nmheWorkspace.acP[7]*nmheWorkspace.acP[4] + nmheWorkspace.acP[15]*nmheWorkspace.acP[12] + nmheWorkspace.acP[23]*nmheWorkspace.acP[20] + nmheWorkspace.acP[31]*nmheWorkspace.acP[28] + nmheWorkspace.acP[39]*nmheWorkspace.acP[36] + nmheWorkspace.acP[47]*nmheWorkspace.acP[44] + nmheWorkspace.acP[55]*nmheWorkspace.acP[52] + nmheWorkspace.acP[63]*nmheWorkspace.acP[60];
nmheVariables.SAC[61] = + nmheWorkspace.acP[7]*nmheWorkspace.acP[5] + nmheWorkspace.acP[15]*nmheWorkspace.acP[13] + nmheWorkspace.acP[23]*nmheWorkspace.acP[21] + nmheWorkspace.acP[31]*nmheWorkspace.acP[29] + nmheWorkspace.acP[39]*nmheWorkspace.acP[37] + nmheWorkspace.acP[47]*nmheWorkspace.acP[45] + nmheWorkspace.acP[55]*nmheWorkspace.acP[53] + nmheWorkspace.acP[63]*nmheWorkspace.acP[61];
nmheVariables.SAC[62] = + nmheWorkspace.acP[7]*nmheWorkspace.acP[6] + nmheWorkspace.acP[15]*nmheWorkspace.acP[14] + nmheWorkspace.acP[23]*nmheWorkspace.acP[22] + nmheWorkspace.acP[31]*nmheWorkspace.acP[30] + nmheWorkspace.acP[39]*nmheWorkspace.acP[38] + nmheWorkspace.acP[47]*nmheWorkspace.acP[46] + nmheWorkspace.acP[55]*nmheWorkspace.acP[54] + nmheWorkspace.acP[63]*nmheWorkspace.acP[62];
nmheVariables.SAC[63] = + nmheWorkspace.acP[7]*nmheWorkspace.acP[7] + nmheWorkspace.acP[15]*nmheWorkspace.acP[15] + nmheWorkspace.acP[23]*nmheWorkspace.acP[23] + nmheWorkspace.acP[31]*nmheWorkspace.acP[31] + nmheWorkspace.acP[39]*nmheWorkspace.acP[39] + nmheWorkspace.acP[47]*nmheWorkspace.acP[47] + nmheWorkspace.acP[55]*nmheWorkspace.acP[55] + nmheWorkspace.acP[63]*nmheWorkspace.acP[63];
return ret;
}

