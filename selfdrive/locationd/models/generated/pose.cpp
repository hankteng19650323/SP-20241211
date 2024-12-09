#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6017775153784914369) {
   out_6017775153784914369[0] = delta_x[0] + nom_x[0];
   out_6017775153784914369[1] = delta_x[1] + nom_x[1];
   out_6017775153784914369[2] = delta_x[2] + nom_x[2];
   out_6017775153784914369[3] = delta_x[3] + nom_x[3];
   out_6017775153784914369[4] = delta_x[4] + nom_x[4];
   out_6017775153784914369[5] = delta_x[5] + nom_x[5];
   out_6017775153784914369[6] = delta_x[6] + nom_x[6];
   out_6017775153784914369[7] = delta_x[7] + nom_x[7];
   out_6017775153784914369[8] = delta_x[8] + nom_x[8];
   out_6017775153784914369[9] = delta_x[9] + nom_x[9];
   out_6017775153784914369[10] = delta_x[10] + nom_x[10];
   out_6017775153784914369[11] = delta_x[11] + nom_x[11];
   out_6017775153784914369[12] = delta_x[12] + nom_x[12];
   out_6017775153784914369[13] = delta_x[13] + nom_x[13];
   out_6017775153784914369[14] = delta_x[14] + nom_x[14];
   out_6017775153784914369[15] = delta_x[15] + nom_x[15];
   out_6017775153784914369[16] = delta_x[16] + nom_x[16];
   out_6017775153784914369[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3276244028421344337) {
   out_3276244028421344337[0] = -nom_x[0] + true_x[0];
   out_3276244028421344337[1] = -nom_x[1] + true_x[1];
   out_3276244028421344337[2] = -nom_x[2] + true_x[2];
   out_3276244028421344337[3] = -nom_x[3] + true_x[3];
   out_3276244028421344337[4] = -nom_x[4] + true_x[4];
   out_3276244028421344337[5] = -nom_x[5] + true_x[5];
   out_3276244028421344337[6] = -nom_x[6] + true_x[6];
   out_3276244028421344337[7] = -nom_x[7] + true_x[7];
   out_3276244028421344337[8] = -nom_x[8] + true_x[8];
   out_3276244028421344337[9] = -nom_x[9] + true_x[9];
   out_3276244028421344337[10] = -nom_x[10] + true_x[10];
   out_3276244028421344337[11] = -nom_x[11] + true_x[11];
   out_3276244028421344337[12] = -nom_x[12] + true_x[12];
   out_3276244028421344337[13] = -nom_x[13] + true_x[13];
   out_3276244028421344337[14] = -nom_x[14] + true_x[14];
   out_3276244028421344337[15] = -nom_x[15] + true_x[15];
   out_3276244028421344337[16] = -nom_x[16] + true_x[16];
   out_3276244028421344337[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3027202661191403252) {
   out_3027202661191403252[0] = 1.0;
   out_3027202661191403252[1] = 0;
   out_3027202661191403252[2] = 0;
   out_3027202661191403252[3] = 0;
   out_3027202661191403252[4] = 0;
   out_3027202661191403252[5] = 0;
   out_3027202661191403252[6] = 0;
   out_3027202661191403252[7] = 0;
   out_3027202661191403252[8] = 0;
   out_3027202661191403252[9] = 0;
   out_3027202661191403252[10] = 0;
   out_3027202661191403252[11] = 0;
   out_3027202661191403252[12] = 0;
   out_3027202661191403252[13] = 0;
   out_3027202661191403252[14] = 0;
   out_3027202661191403252[15] = 0;
   out_3027202661191403252[16] = 0;
   out_3027202661191403252[17] = 0;
   out_3027202661191403252[18] = 0;
   out_3027202661191403252[19] = 1.0;
   out_3027202661191403252[20] = 0;
   out_3027202661191403252[21] = 0;
   out_3027202661191403252[22] = 0;
   out_3027202661191403252[23] = 0;
   out_3027202661191403252[24] = 0;
   out_3027202661191403252[25] = 0;
   out_3027202661191403252[26] = 0;
   out_3027202661191403252[27] = 0;
   out_3027202661191403252[28] = 0;
   out_3027202661191403252[29] = 0;
   out_3027202661191403252[30] = 0;
   out_3027202661191403252[31] = 0;
   out_3027202661191403252[32] = 0;
   out_3027202661191403252[33] = 0;
   out_3027202661191403252[34] = 0;
   out_3027202661191403252[35] = 0;
   out_3027202661191403252[36] = 0;
   out_3027202661191403252[37] = 0;
   out_3027202661191403252[38] = 1.0;
   out_3027202661191403252[39] = 0;
   out_3027202661191403252[40] = 0;
   out_3027202661191403252[41] = 0;
   out_3027202661191403252[42] = 0;
   out_3027202661191403252[43] = 0;
   out_3027202661191403252[44] = 0;
   out_3027202661191403252[45] = 0;
   out_3027202661191403252[46] = 0;
   out_3027202661191403252[47] = 0;
   out_3027202661191403252[48] = 0;
   out_3027202661191403252[49] = 0;
   out_3027202661191403252[50] = 0;
   out_3027202661191403252[51] = 0;
   out_3027202661191403252[52] = 0;
   out_3027202661191403252[53] = 0;
   out_3027202661191403252[54] = 0;
   out_3027202661191403252[55] = 0;
   out_3027202661191403252[56] = 0;
   out_3027202661191403252[57] = 1.0;
   out_3027202661191403252[58] = 0;
   out_3027202661191403252[59] = 0;
   out_3027202661191403252[60] = 0;
   out_3027202661191403252[61] = 0;
   out_3027202661191403252[62] = 0;
   out_3027202661191403252[63] = 0;
   out_3027202661191403252[64] = 0;
   out_3027202661191403252[65] = 0;
   out_3027202661191403252[66] = 0;
   out_3027202661191403252[67] = 0;
   out_3027202661191403252[68] = 0;
   out_3027202661191403252[69] = 0;
   out_3027202661191403252[70] = 0;
   out_3027202661191403252[71] = 0;
   out_3027202661191403252[72] = 0;
   out_3027202661191403252[73] = 0;
   out_3027202661191403252[74] = 0;
   out_3027202661191403252[75] = 0;
   out_3027202661191403252[76] = 1.0;
   out_3027202661191403252[77] = 0;
   out_3027202661191403252[78] = 0;
   out_3027202661191403252[79] = 0;
   out_3027202661191403252[80] = 0;
   out_3027202661191403252[81] = 0;
   out_3027202661191403252[82] = 0;
   out_3027202661191403252[83] = 0;
   out_3027202661191403252[84] = 0;
   out_3027202661191403252[85] = 0;
   out_3027202661191403252[86] = 0;
   out_3027202661191403252[87] = 0;
   out_3027202661191403252[88] = 0;
   out_3027202661191403252[89] = 0;
   out_3027202661191403252[90] = 0;
   out_3027202661191403252[91] = 0;
   out_3027202661191403252[92] = 0;
   out_3027202661191403252[93] = 0;
   out_3027202661191403252[94] = 0;
   out_3027202661191403252[95] = 1.0;
   out_3027202661191403252[96] = 0;
   out_3027202661191403252[97] = 0;
   out_3027202661191403252[98] = 0;
   out_3027202661191403252[99] = 0;
   out_3027202661191403252[100] = 0;
   out_3027202661191403252[101] = 0;
   out_3027202661191403252[102] = 0;
   out_3027202661191403252[103] = 0;
   out_3027202661191403252[104] = 0;
   out_3027202661191403252[105] = 0;
   out_3027202661191403252[106] = 0;
   out_3027202661191403252[107] = 0;
   out_3027202661191403252[108] = 0;
   out_3027202661191403252[109] = 0;
   out_3027202661191403252[110] = 0;
   out_3027202661191403252[111] = 0;
   out_3027202661191403252[112] = 0;
   out_3027202661191403252[113] = 0;
   out_3027202661191403252[114] = 1.0;
   out_3027202661191403252[115] = 0;
   out_3027202661191403252[116] = 0;
   out_3027202661191403252[117] = 0;
   out_3027202661191403252[118] = 0;
   out_3027202661191403252[119] = 0;
   out_3027202661191403252[120] = 0;
   out_3027202661191403252[121] = 0;
   out_3027202661191403252[122] = 0;
   out_3027202661191403252[123] = 0;
   out_3027202661191403252[124] = 0;
   out_3027202661191403252[125] = 0;
   out_3027202661191403252[126] = 0;
   out_3027202661191403252[127] = 0;
   out_3027202661191403252[128] = 0;
   out_3027202661191403252[129] = 0;
   out_3027202661191403252[130] = 0;
   out_3027202661191403252[131] = 0;
   out_3027202661191403252[132] = 0;
   out_3027202661191403252[133] = 1.0;
   out_3027202661191403252[134] = 0;
   out_3027202661191403252[135] = 0;
   out_3027202661191403252[136] = 0;
   out_3027202661191403252[137] = 0;
   out_3027202661191403252[138] = 0;
   out_3027202661191403252[139] = 0;
   out_3027202661191403252[140] = 0;
   out_3027202661191403252[141] = 0;
   out_3027202661191403252[142] = 0;
   out_3027202661191403252[143] = 0;
   out_3027202661191403252[144] = 0;
   out_3027202661191403252[145] = 0;
   out_3027202661191403252[146] = 0;
   out_3027202661191403252[147] = 0;
   out_3027202661191403252[148] = 0;
   out_3027202661191403252[149] = 0;
   out_3027202661191403252[150] = 0;
   out_3027202661191403252[151] = 0;
   out_3027202661191403252[152] = 1.0;
   out_3027202661191403252[153] = 0;
   out_3027202661191403252[154] = 0;
   out_3027202661191403252[155] = 0;
   out_3027202661191403252[156] = 0;
   out_3027202661191403252[157] = 0;
   out_3027202661191403252[158] = 0;
   out_3027202661191403252[159] = 0;
   out_3027202661191403252[160] = 0;
   out_3027202661191403252[161] = 0;
   out_3027202661191403252[162] = 0;
   out_3027202661191403252[163] = 0;
   out_3027202661191403252[164] = 0;
   out_3027202661191403252[165] = 0;
   out_3027202661191403252[166] = 0;
   out_3027202661191403252[167] = 0;
   out_3027202661191403252[168] = 0;
   out_3027202661191403252[169] = 0;
   out_3027202661191403252[170] = 0;
   out_3027202661191403252[171] = 1.0;
   out_3027202661191403252[172] = 0;
   out_3027202661191403252[173] = 0;
   out_3027202661191403252[174] = 0;
   out_3027202661191403252[175] = 0;
   out_3027202661191403252[176] = 0;
   out_3027202661191403252[177] = 0;
   out_3027202661191403252[178] = 0;
   out_3027202661191403252[179] = 0;
   out_3027202661191403252[180] = 0;
   out_3027202661191403252[181] = 0;
   out_3027202661191403252[182] = 0;
   out_3027202661191403252[183] = 0;
   out_3027202661191403252[184] = 0;
   out_3027202661191403252[185] = 0;
   out_3027202661191403252[186] = 0;
   out_3027202661191403252[187] = 0;
   out_3027202661191403252[188] = 0;
   out_3027202661191403252[189] = 0;
   out_3027202661191403252[190] = 1.0;
   out_3027202661191403252[191] = 0;
   out_3027202661191403252[192] = 0;
   out_3027202661191403252[193] = 0;
   out_3027202661191403252[194] = 0;
   out_3027202661191403252[195] = 0;
   out_3027202661191403252[196] = 0;
   out_3027202661191403252[197] = 0;
   out_3027202661191403252[198] = 0;
   out_3027202661191403252[199] = 0;
   out_3027202661191403252[200] = 0;
   out_3027202661191403252[201] = 0;
   out_3027202661191403252[202] = 0;
   out_3027202661191403252[203] = 0;
   out_3027202661191403252[204] = 0;
   out_3027202661191403252[205] = 0;
   out_3027202661191403252[206] = 0;
   out_3027202661191403252[207] = 0;
   out_3027202661191403252[208] = 0;
   out_3027202661191403252[209] = 1.0;
   out_3027202661191403252[210] = 0;
   out_3027202661191403252[211] = 0;
   out_3027202661191403252[212] = 0;
   out_3027202661191403252[213] = 0;
   out_3027202661191403252[214] = 0;
   out_3027202661191403252[215] = 0;
   out_3027202661191403252[216] = 0;
   out_3027202661191403252[217] = 0;
   out_3027202661191403252[218] = 0;
   out_3027202661191403252[219] = 0;
   out_3027202661191403252[220] = 0;
   out_3027202661191403252[221] = 0;
   out_3027202661191403252[222] = 0;
   out_3027202661191403252[223] = 0;
   out_3027202661191403252[224] = 0;
   out_3027202661191403252[225] = 0;
   out_3027202661191403252[226] = 0;
   out_3027202661191403252[227] = 0;
   out_3027202661191403252[228] = 1.0;
   out_3027202661191403252[229] = 0;
   out_3027202661191403252[230] = 0;
   out_3027202661191403252[231] = 0;
   out_3027202661191403252[232] = 0;
   out_3027202661191403252[233] = 0;
   out_3027202661191403252[234] = 0;
   out_3027202661191403252[235] = 0;
   out_3027202661191403252[236] = 0;
   out_3027202661191403252[237] = 0;
   out_3027202661191403252[238] = 0;
   out_3027202661191403252[239] = 0;
   out_3027202661191403252[240] = 0;
   out_3027202661191403252[241] = 0;
   out_3027202661191403252[242] = 0;
   out_3027202661191403252[243] = 0;
   out_3027202661191403252[244] = 0;
   out_3027202661191403252[245] = 0;
   out_3027202661191403252[246] = 0;
   out_3027202661191403252[247] = 1.0;
   out_3027202661191403252[248] = 0;
   out_3027202661191403252[249] = 0;
   out_3027202661191403252[250] = 0;
   out_3027202661191403252[251] = 0;
   out_3027202661191403252[252] = 0;
   out_3027202661191403252[253] = 0;
   out_3027202661191403252[254] = 0;
   out_3027202661191403252[255] = 0;
   out_3027202661191403252[256] = 0;
   out_3027202661191403252[257] = 0;
   out_3027202661191403252[258] = 0;
   out_3027202661191403252[259] = 0;
   out_3027202661191403252[260] = 0;
   out_3027202661191403252[261] = 0;
   out_3027202661191403252[262] = 0;
   out_3027202661191403252[263] = 0;
   out_3027202661191403252[264] = 0;
   out_3027202661191403252[265] = 0;
   out_3027202661191403252[266] = 1.0;
   out_3027202661191403252[267] = 0;
   out_3027202661191403252[268] = 0;
   out_3027202661191403252[269] = 0;
   out_3027202661191403252[270] = 0;
   out_3027202661191403252[271] = 0;
   out_3027202661191403252[272] = 0;
   out_3027202661191403252[273] = 0;
   out_3027202661191403252[274] = 0;
   out_3027202661191403252[275] = 0;
   out_3027202661191403252[276] = 0;
   out_3027202661191403252[277] = 0;
   out_3027202661191403252[278] = 0;
   out_3027202661191403252[279] = 0;
   out_3027202661191403252[280] = 0;
   out_3027202661191403252[281] = 0;
   out_3027202661191403252[282] = 0;
   out_3027202661191403252[283] = 0;
   out_3027202661191403252[284] = 0;
   out_3027202661191403252[285] = 1.0;
   out_3027202661191403252[286] = 0;
   out_3027202661191403252[287] = 0;
   out_3027202661191403252[288] = 0;
   out_3027202661191403252[289] = 0;
   out_3027202661191403252[290] = 0;
   out_3027202661191403252[291] = 0;
   out_3027202661191403252[292] = 0;
   out_3027202661191403252[293] = 0;
   out_3027202661191403252[294] = 0;
   out_3027202661191403252[295] = 0;
   out_3027202661191403252[296] = 0;
   out_3027202661191403252[297] = 0;
   out_3027202661191403252[298] = 0;
   out_3027202661191403252[299] = 0;
   out_3027202661191403252[300] = 0;
   out_3027202661191403252[301] = 0;
   out_3027202661191403252[302] = 0;
   out_3027202661191403252[303] = 0;
   out_3027202661191403252[304] = 1.0;
   out_3027202661191403252[305] = 0;
   out_3027202661191403252[306] = 0;
   out_3027202661191403252[307] = 0;
   out_3027202661191403252[308] = 0;
   out_3027202661191403252[309] = 0;
   out_3027202661191403252[310] = 0;
   out_3027202661191403252[311] = 0;
   out_3027202661191403252[312] = 0;
   out_3027202661191403252[313] = 0;
   out_3027202661191403252[314] = 0;
   out_3027202661191403252[315] = 0;
   out_3027202661191403252[316] = 0;
   out_3027202661191403252[317] = 0;
   out_3027202661191403252[318] = 0;
   out_3027202661191403252[319] = 0;
   out_3027202661191403252[320] = 0;
   out_3027202661191403252[321] = 0;
   out_3027202661191403252[322] = 0;
   out_3027202661191403252[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_1694909997772681373) {
   out_1694909997772681373[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_1694909997772681373[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_1694909997772681373[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_1694909997772681373[3] = dt*state[12] + state[3];
   out_1694909997772681373[4] = dt*state[13] + state[4];
   out_1694909997772681373[5] = dt*state[14] + state[5];
   out_1694909997772681373[6] = state[6];
   out_1694909997772681373[7] = state[7];
   out_1694909997772681373[8] = state[8];
   out_1694909997772681373[9] = state[9];
   out_1694909997772681373[10] = state[10];
   out_1694909997772681373[11] = state[11];
   out_1694909997772681373[12] = state[12];
   out_1694909997772681373[13] = state[13];
   out_1694909997772681373[14] = state[14];
   out_1694909997772681373[15] = state[15];
   out_1694909997772681373[16] = state[16];
   out_1694909997772681373[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7329719651390048604) {
   out_7329719651390048604[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7329719651390048604[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7329719651390048604[2] = 0;
   out_7329719651390048604[3] = 0;
   out_7329719651390048604[4] = 0;
   out_7329719651390048604[5] = 0;
   out_7329719651390048604[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7329719651390048604[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7329719651390048604[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7329719651390048604[9] = 0;
   out_7329719651390048604[10] = 0;
   out_7329719651390048604[11] = 0;
   out_7329719651390048604[12] = 0;
   out_7329719651390048604[13] = 0;
   out_7329719651390048604[14] = 0;
   out_7329719651390048604[15] = 0;
   out_7329719651390048604[16] = 0;
   out_7329719651390048604[17] = 0;
   out_7329719651390048604[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7329719651390048604[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7329719651390048604[20] = 0;
   out_7329719651390048604[21] = 0;
   out_7329719651390048604[22] = 0;
   out_7329719651390048604[23] = 0;
   out_7329719651390048604[24] = 0;
   out_7329719651390048604[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7329719651390048604[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7329719651390048604[27] = 0;
   out_7329719651390048604[28] = 0;
   out_7329719651390048604[29] = 0;
   out_7329719651390048604[30] = 0;
   out_7329719651390048604[31] = 0;
   out_7329719651390048604[32] = 0;
   out_7329719651390048604[33] = 0;
   out_7329719651390048604[34] = 0;
   out_7329719651390048604[35] = 0;
   out_7329719651390048604[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7329719651390048604[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7329719651390048604[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7329719651390048604[39] = 0;
   out_7329719651390048604[40] = 0;
   out_7329719651390048604[41] = 0;
   out_7329719651390048604[42] = 0;
   out_7329719651390048604[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7329719651390048604[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7329719651390048604[45] = 0;
   out_7329719651390048604[46] = 0;
   out_7329719651390048604[47] = 0;
   out_7329719651390048604[48] = 0;
   out_7329719651390048604[49] = 0;
   out_7329719651390048604[50] = 0;
   out_7329719651390048604[51] = 0;
   out_7329719651390048604[52] = 0;
   out_7329719651390048604[53] = 0;
   out_7329719651390048604[54] = 0;
   out_7329719651390048604[55] = 0;
   out_7329719651390048604[56] = 0;
   out_7329719651390048604[57] = 1;
   out_7329719651390048604[58] = 0;
   out_7329719651390048604[59] = 0;
   out_7329719651390048604[60] = 0;
   out_7329719651390048604[61] = 0;
   out_7329719651390048604[62] = 0;
   out_7329719651390048604[63] = 0;
   out_7329719651390048604[64] = 0;
   out_7329719651390048604[65] = 0;
   out_7329719651390048604[66] = dt;
   out_7329719651390048604[67] = 0;
   out_7329719651390048604[68] = 0;
   out_7329719651390048604[69] = 0;
   out_7329719651390048604[70] = 0;
   out_7329719651390048604[71] = 0;
   out_7329719651390048604[72] = 0;
   out_7329719651390048604[73] = 0;
   out_7329719651390048604[74] = 0;
   out_7329719651390048604[75] = 0;
   out_7329719651390048604[76] = 1;
   out_7329719651390048604[77] = 0;
   out_7329719651390048604[78] = 0;
   out_7329719651390048604[79] = 0;
   out_7329719651390048604[80] = 0;
   out_7329719651390048604[81] = 0;
   out_7329719651390048604[82] = 0;
   out_7329719651390048604[83] = 0;
   out_7329719651390048604[84] = 0;
   out_7329719651390048604[85] = dt;
   out_7329719651390048604[86] = 0;
   out_7329719651390048604[87] = 0;
   out_7329719651390048604[88] = 0;
   out_7329719651390048604[89] = 0;
   out_7329719651390048604[90] = 0;
   out_7329719651390048604[91] = 0;
   out_7329719651390048604[92] = 0;
   out_7329719651390048604[93] = 0;
   out_7329719651390048604[94] = 0;
   out_7329719651390048604[95] = 1;
   out_7329719651390048604[96] = 0;
   out_7329719651390048604[97] = 0;
   out_7329719651390048604[98] = 0;
   out_7329719651390048604[99] = 0;
   out_7329719651390048604[100] = 0;
   out_7329719651390048604[101] = 0;
   out_7329719651390048604[102] = 0;
   out_7329719651390048604[103] = 0;
   out_7329719651390048604[104] = dt;
   out_7329719651390048604[105] = 0;
   out_7329719651390048604[106] = 0;
   out_7329719651390048604[107] = 0;
   out_7329719651390048604[108] = 0;
   out_7329719651390048604[109] = 0;
   out_7329719651390048604[110] = 0;
   out_7329719651390048604[111] = 0;
   out_7329719651390048604[112] = 0;
   out_7329719651390048604[113] = 0;
   out_7329719651390048604[114] = 1;
   out_7329719651390048604[115] = 0;
   out_7329719651390048604[116] = 0;
   out_7329719651390048604[117] = 0;
   out_7329719651390048604[118] = 0;
   out_7329719651390048604[119] = 0;
   out_7329719651390048604[120] = 0;
   out_7329719651390048604[121] = 0;
   out_7329719651390048604[122] = 0;
   out_7329719651390048604[123] = 0;
   out_7329719651390048604[124] = 0;
   out_7329719651390048604[125] = 0;
   out_7329719651390048604[126] = 0;
   out_7329719651390048604[127] = 0;
   out_7329719651390048604[128] = 0;
   out_7329719651390048604[129] = 0;
   out_7329719651390048604[130] = 0;
   out_7329719651390048604[131] = 0;
   out_7329719651390048604[132] = 0;
   out_7329719651390048604[133] = 1;
   out_7329719651390048604[134] = 0;
   out_7329719651390048604[135] = 0;
   out_7329719651390048604[136] = 0;
   out_7329719651390048604[137] = 0;
   out_7329719651390048604[138] = 0;
   out_7329719651390048604[139] = 0;
   out_7329719651390048604[140] = 0;
   out_7329719651390048604[141] = 0;
   out_7329719651390048604[142] = 0;
   out_7329719651390048604[143] = 0;
   out_7329719651390048604[144] = 0;
   out_7329719651390048604[145] = 0;
   out_7329719651390048604[146] = 0;
   out_7329719651390048604[147] = 0;
   out_7329719651390048604[148] = 0;
   out_7329719651390048604[149] = 0;
   out_7329719651390048604[150] = 0;
   out_7329719651390048604[151] = 0;
   out_7329719651390048604[152] = 1;
   out_7329719651390048604[153] = 0;
   out_7329719651390048604[154] = 0;
   out_7329719651390048604[155] = 0;
   out_7329719651390048604[156] = 0;
   out_7329719651390048604[157] = 0;
   out_7329719651390048604[158] = 0;
   out_7329719651390048604[159] = 0;
   out_7329719651390048604[160] = 0;
   out_7329719651390048604[161] = 0;
   out_7329719651390048604[162] = 0;
   out_7329719651390048604[163] = 0;
   out_7329719651390048604[164] = 0;
   out_7329719651390048604[165] = 0;
   out_7329719651390048604[166] = 0;
   out_7329719651390048604[167] = 0;
   out_7329719651390048604[168] = 0;
   out_7329719651390048604[169] = 0;
   out_7329719651390048604[170] = 0;
   out_7329719651390048604[171] = 1;
   out_7329719651390048604[172] = 0;
   out_7329719651390048604[173] = 0;
   out_7329719651390048604[174] = 0;
   out_7329719651390048604[175] = 0;
   out_7329719651390048604[176] = 0;
   out_7329719651390048604[177] = 0;
   out_7329719651390048604[178] = 0;
   out_7329719651390048604[179] = 0;
   out_7329719651390048604[180] = 0;
   out_7329719651390048604[181] = 0;
   out_7329719651390048604[182] = 0;
   out_7329719651390048604[183] = 0;
   out_7329719651390048604[184] = 0;
   out_7329719651390048604[185] = 0;
   out_7329719651390048604[186] = 0;
   out_7329719651390048604[187] = 0;
   out_7329719651390048604[188] = 0;
   out_7329719651390048604[189] = 0;
   out_7329719651390048604[190] = 1;
   out_7329719651390048604[191] = 0;
   out_7329719651390048604[192] = 0;
   out_7329719651390048604[193] = 0;
   out_7329719651390048604[194] = 0;
   out_7329719651390048604[195] = 0;
   out_7329719651390048604[196] = 0;
   out_7329719651390048604[197] = 0;
   out_7329719651390048604[198] = 0;
   out_7329719651390048604[199] = 0;
   out_7329719651390048604[200] = 0;
   out_7329719651390048604[201] = 0;
   out_7329719651390048604[202] = 0;
   out_7329719651390048604[203] = 0;
   out_7329719651390048604[204] = 0;
   out_7329719651390048604[205] = 0;
   out_7329719651390048604[206] = 0;
   out_7329719651390048604[207] = 0;
   out_7329719651390048604[208] = 0;
   out_7329719651390048604[209] = 1;
   out_7329719651390048604[210] = 0;
   out_7329719651390048604[211] = 0;
   out_7329719651390048604[212] = 0;
   out_7329719651390048604[213] = 0;
   out_7329719651390048604[214] = 0;
   out_7329719651390048604[215] = 0;
   out_7329719651390048604[216] = 0;
   out_7329719651390048604[217] = 0;
   out_7329719651390048604[218] = 0;
   out_7329719651390048604[219] = 0;
   out_7329719651390048604[220] = 0;
   out_7329719651390048604[221] = 0;
   out_7329719651390048604[222] = 0;
   out_7329719651390048604[223] = 0;
   out_7329719651390048604[224] = 0;
   out_7329719651390048604[225] = 0;
   out_7329719651390048604[226] = 0;
   out_7329719651390048604[227] = 0;
   out_7329719651390048604[228] = 1;
   out_7329719651390048604[229] = 0;
   out_7329719651390048604[230] = 0;
   out_7329719651390048604[231] = 0;
   out_7329719651390048604[232] = 0;
   out_7329719651390048604[233] = 0;
   out_7329719651390048604[234] = 0;
   out_7329719651390048604[235] = 0;
   out_7329719651390048604[236] = 0;
   out_7329719651390048604[237] = 0;
   out_7329719651390048604[238] = 0;
   out_7329719651390048604[239] = 0;
   out_7329719651390048604[240] = 0;
   out_7329719651390048604[241] = 0;
   out_7329719651390048604[242] = 0;
   out_7329719651390048604[243] = 0;
   out_7329719651390048604[244] = 0;
   out_7329719651390048604[245] = 0;
   out_7329719651390048604[246] = 0;
   out_7329719651390048604[247] = 1;
   out_7329719651390048604[248] = 0;
   out_7329719651390048604[249] = 0;
   out_7329719651390048604[250] = 0;
   out_7329719651390048604[251] = 0;
   out_7329719651390048604[252] = 0;
   out_7329719651390048604[253] = 0;
   out_7329719651390048604[254] = 0;
   out_7329719651390048604[255] = 0;
   out_7329719651390048604[256] = 0;
   out_7329719651390048604[257] = 0;
   out_7329719651390048604[258] = 0;
   out_7329719651390048604[259] = 0;
   out_7329719651390048604[260] = 0;
   out_7329719651390048604[261] = 0;
   out_7329719651390048604[262] = 0;
   out_7329719651390048604[263] = 0;
   out_7329719651390048604[264] = 0;
   out_7329719651390048604[265] = 0;
   out_7329719651390048604[266] = 1;
   out_7329719651390048604[267] = 0;
   out_7329719651390048604[268] = 0;
   out_7329719651390048604[269] = 0;
   out_7329719651390048604[270] = 0;
   out_7329719651390048604[271] = 0;
   out_7329719651390048604[272] = 0;
   out_7329719651390048604[273] = 0;
   out_7329719651390048604[274] = 0;
   out_7329719651390048604[275] = 0;
   out_7329719651390048604[276] = 0;
   out_7329719651390048604[277] = 0;
   out_7329719651390048604[278] = 0;
   out_7329719651390048604[279] = 0;
   out_7329719651390048604[280] = 0;
   out_7329719651390048604[281] = 0;
   out_7329719651390048604[282] = 0;
   out_7329719651390048604[283] = 0;
   out_7329719651390048604[284] = 0;
   out_7329719651390048604[285] = 1;
   out_7329719651390048604[286] = 0;
   out_7329719651390048604[287] = 0;
   out_7329719651390048604[288] = 0;
   out_7329719651390048604[289] = 0;
   out_7329719651390048604[290] = 0;
   out_7329719651390048604[291] = 0;
   out_7329719651390048604[292] = 0;
   out_7329719651390048604[293] = 0;
   out_7329719651390048604[294] = 0;
   out_7329719651390048604[295] = 0;
   out_7329719651390048604[296] = 0;
   out_7329719651390048604[297] = 0;
   out_7329719651390048604[298] = 0;
   out_7329719651390048604[299] = 0;
   out_7329719651390048604[300] = 0;
   out_7329719651390048604[301] = 0;
   out_7329719651390048604[302] = 0;
   out_7329719651390048604[303] = 0;
   out_7329719651390048604[304] = 1;
   out_7329719651390048604[305] = 0;
   out_7329719651390048604[306] = 0;
   out_7329719651390048604[307] = 0;
   out_7329719651390048604[308] = 0;
   out_7329719651390048604[309] = 0;
   out_7329719651390048604[310] = 0;
   out_7329719651390048604[311] = 0;
   out_7329719651390048604[312] = 0;
   out_7329719651390048604[313] = 0;
   out_7329719651390048604[314] = 0;
   out_7329719651390048604[315] = 0;
   out_7329719651390048604[316] = 0;
   out_7329719651390048604[317] = 0;
   out_7329719651390048604[318] = 0;
   out_7329719651390048604[319] = 0;
   out_7329719651390048604[320] = 0;
   out_7329719651390048604[321] = 0;
   out_7329719651390048604[322] = 0;
   out_7329719651390048604[323] = 1;
}
void h_4(double *state, double *unused, double *out_3452650106359951024) {
   out_3452650106359951024[0] = state[6] + state[9];
   out_3452650106359951024[1] = state[7] + state[10];
   out_3452650106359951024[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_1923156489118298873) {
   out_1923156489118298873[0] = 0;
   out_1923156489118298873[1] = 0;
   out_1923156489118298873[2] = 0;
   out_1923156489118298873[3] = 0;
   out_1923156489118298873[4] = 0;
   out_1923156489118298873[5] = 0;
   out_1923156489118298873[6] = 1;
   out_1923156489118298873[7] = 0;
   out_1923156489118298873[8] = 0;
   out_1923156489118298873[9] = 1;
   out_1923156489118298873[10] = 0;
   out_1923156489118298873[11] = 0;
   out_1923156489118298873[12] = 0;
   out_1923156489118298873[13] = 0;
   out_1923156489118298873[14] = 0;
   out_1923156489118298873[15] = 0;
   out_1923156489118298873[16] = 0;
   out_1923156489118298873[17] = 0;
   out_1923156489118298873[18] = 0;
   out_1923156489118298873[19] = 0;
   out_1923156489118298873[20] = 0;
   out_1923156489118298873[21] = 0;
   out_1923156489118298873[22] = 0;
   out_1923156489118298873[23] = 0;
   out_1923156489118298873[24] = 0;
   out_1923156489118298873[25] = 1;
   out_1923156489118298873[26] = 0;
   out_1923156489118298873[27] = 0;
   out_1923156489118298873[28] = 1;
   out_1923156489118298873[29] = 0;
   out_1923156489118298873[30] = 0;
   out_1923156489118298873[31] = 0;
   out_1923156489118298873[32] = 0;
   out_1923156489118298873[33] = 0;
   out_1923156489118298873[34] = 0;
   out_1923156489118298873[35] = 0;
   out_1923156489118298873[36] = 0;
   out_1923156489118298873[37] = 0;
   out_1923156489118298873[38] = 0;
   out_1923156489118298873[39] = 0;
   out_1923156489118298873[40] = 0;
   out_1923156489118298873[41] = 0;
   out_1923156489118298873[42] = 0;
   out_1923156489118298873[43] = 0;
   out_1923156489118298873[44] = 1;
   out_1923156489118298873[45] = 0;
   out_1923156489118298873[46] = 0;
   out_1923156489118298873[47] = 1;
   out_1923156489118298873[48] = 0;
   out_1923156489118298873[49] = 0;
   out_1923156489118298873[50] = 0;
   out_1923156489118298873[51] = 0;
   out_1923156489118298873[52] = 0;
   out_1923156489118298873[53] = 0;
}
void h_10(double *state, double *unused, double *out_2947394259008857773) {
   out_2947394259008857773[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2947394259008857773[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2947394259008857773[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8876134415720308413) {
   out_8876134415720308413[0] = 0;
   out_8876134415720308413[1] = 9.8100000000000005*cos(state[1]);
   out_8876134415720308413[2] = 0;
   out_8876134415720308413[3] = 0;
   out_8876134415720308413[4] = -state[8];
   out_8876134415720308413[5] = state[7];
   out_8876134415720308413[6] = 0;
   out_8876134415720308413[7] = state[5];
   out_8876134415720308413[8] = -state[4];
   out_8876134415720308413[9] = 0;
   out_8876134415720308413[10] = 0;
   out_8876134415720308413[11] = 0;
   out_8876134415720308413[12] = 1;
   out_8876134415720308413[13] = 0;
   out_8876134415720308413[14] = 0;
   out_8876134415720308413[15] = 1;
   out_8876134415720308413[16] = 0;
   out_8876134415720308413[17] = 0;
   out_8876134415720308413[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8876134415720308413[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8876134415720308413[20] = 0;
   out_8876134415720308413[21] = state[8];
   out_8876134415720308413[22] = 0;
   out_8876134415720308413[23] = -state[6];
   out_8876134415720308413[24] = -state[5];
   out_8876134415720308413[25] = 0;
   out_8876134415720308413[26] = state[3];
   out_8876134415720308413[27] = 0;
   out_8876134415720308413[28] = 0;
   out_8876134415720308413[29] = 0;
   out_8876134415720308413[30] = 0;
   out_8876134415720308413[31] = 1;
   out_8876134415720308413[32] = 0;
   out_8876134415720308413[33] = 0;
   out_8876134415720308413[34] = 1;
   out_8876134415720308413[35] = 0;
   out_8876134415720308413[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8876134415720308413[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8876134415720308413[38] = 0;
   out_8876134415720308413[39] = -state[7];
   out_8876134415720308413[40] = state[6];
   out_8876134415720308413[41] = 0;
   out_8876134415720308413[42] = state[4];
   out_8876134415720308413[43] = -state[3];
   out_8876134415720308413[44] = 0;
   out_8876134415720308413[45] = 0;
   out_8876134415720308413[46] = 0;
   out_8876134415720308413[47] = 0;
   out_8876134415720308413[48] = 0;
   out_8876134415720308413[49] = 0;
   out_8876134415720308413[50] = 1;
   out_8876134415720308413[51] = 0;
   out_8876134415720308413[52] = 0;
   out_8876134415720308413[53] = 1;
}
void h_13(double *state, double *unused, double *out_1604531653721123784) {
   out_1604531653721123784[0] = state[3];
   out_1604531653721123784[1] = state[4];
   out_1604531653721123784[2] = state[5];
}
void H_13(double *state, double *unused, double *out_1289117336214033928) {
   out_1289117336214033928[0] = 0;
   out_1289117336214033928[1] = 0;
   out_1289117336214033928[2] = 0;
   out_1289117336214033928[3] = 1;
   out_1289117336214033928[4] = 0;
   out_1289117336214033928[5] = 0;
   out_1289117336214033928[6] = 0;
   out_1289117336214033928[7] = 0;
   out_1289117336214033928[8] = 0;
   out_1289117336214033928[9] = 0;
   out_1289117336214033928[10] = 0;
   out_1289117336214033928[11] = 0;
   out_1289117336214033928[12] = 0;
   out_1289117336214033928[13] = 0;
   out_1289117336214033928[14] = 0;
   out_1289117336214033928[15] = 0;
   out_1289117336214033928[16] = 0;
   out_1289117336214033928[17] = 0;
   out_1289117336214033928[18] = 0;
   out_1289117336214033928[19] = 0;
   out_1289117336214033928[20] = 0;
   out_1289117336214033928[21] = 0;
   out_1289117336214033928[22] = 1;
   out_1289117336214033928[23] = 0;
   out_1289117336214033928[24] = 0;
   out_1289117336214033928[25] = 0;
   out_1289117336214033928[26] = 0;
   out_1289117336214033928[27] = 0;
   out_1289117336214033928[28] = 0;
   out_1289117336214033928[29] = 0;
   out_1289117336214033928[30] = 0;
   out_1289117336214033928[31] = 0;
   out_1289117336214033928[32] = 0;
   out_1289117336214033928[33] = 0;
   out_1289117336214033928[34] = 0;
   out_1289117336214033928[35] = 0;
   out_1289117336214033928[36] = 0;
   out_1289117336214033928[37] = 0;
   out_1289117336214033928[38] = 0;
   out_1289117336214033928[39] = 0;
   out_1289117336214033928[40] = 0;
   out_1289117336214033928[41] = 1;
   out_1289117336214033928[42] = 0;
   out_1289117336214033928[43] = 0;
   out_1289117336214033928[44] = 0;
   out_1289117336214033928[45] = 0;
   out_1289117336214033928[46] = 0;
   out_1289117336214033928[47] = 0;
   out_1289117336214033928[48] = 0;
   out_1289117336214033928[49] = 0;
   out_1289117336214033928[50] = 0;
   out_1289117336214033928[51] = 0;
   out_1289117336214033928[52] = 0;
   out_1289117336214033928[53] = 0;
}
void h_14(double *state, double *unused, double *out_2048301774952519882) {
   out_2048301774952519882[0] = state[6];
   out_2048301774952519882[1] = state[7];
   out_2048301774952519882[2] = state[8];
}
void H_14(double *state, double *unused, double *out_9042441769311512319) {
   out_9042441769311512319[0] = 0;
   out_9042441769311512319[1] = 0;
   out_9042441769311512319[2] = 0;
   out_9042441769311512319[3] = 0;
   out_9042441769311512319[4] = 0;
   out_9042441769311512319[5] = 0;
   out_9042441769311512319[6] = 1;
   out_9042441769311512319[7] = 0;
   out_9042441769311512319[8] = 0;
   out_9042441769311512319[9] = 0;
   out_9042441769311512319[10] = 0;
   out_9042441769311512319[11] = 0;
   out_9042441769311512319[12] = 0;
   out_9042441769311512319[13] = 0;
   out_9042441769311512319[14] = 0;
   out_9042441769311512319[15] = 0;
   out_9042441769311512319[16] = 0;
   out_9042441769311512319[17] = 0;
   out_9042441769311512319[18] = 0;
   out_9042441769311512319[19] = 0;
   out_9042441769311512319[20] = 0;
   out_9042441769311512319[21] = 0;
   out_9042441769311512319[22] = 0;
   out_9042441769311512319[23] = 0;
   out_9042441769311512319[24] = 0;
   out_9042441769311512319[25] = 1;
   out_9042441769311512319[26] = 0;
   out_9042441769311512319[27] = 0;
   out_9042441769311512319[28] = 0;
   out_9042441769311512319[29] = 0;
   out_9042441769311512319[30] = 0;
   out_9042441769311512319[31] = 0;
   out_9042441769311512319[32] = 0;
   out_9042441769311512319[33] = 0;
   out_9042441769311512319[34] = 0;
   out_9042441769311512319[35] = 0;
   out_9042441769311512319[36] = 0;
   out_9042441769311512319[37] = 0;
   out_9042441769311512319[38] = 0;
   out_9042441769311512319[39] = 0;
   out_9042441769311512319[40] = 0;
   out_9042441769311512319[41] = 0;
   out_9042441769311512319[42] = 0;
   out_9042441769311512319[43] = 0;
   out_9042441769311512319[44] = 1;
   out_9042441769311512319[45] = 0;
   out_9042441769311512319[46] = 0;
   out_9042441769311512319[47] = 0;
   out_9042441769311512319[48] = 0;
   out_9042441769311512319[49] = 0;
   out_9042441769311512319[50] = 0;
   out_9042441769311512319[51] = 0;
   out_9042441769311512319[52] = 0;
   out_9042441769311512319[53] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_6017775153784914369) {
  err_fun(nom_x, delta_x, out_6017775153784914369);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3276244028421344337) {
  inv_err_fun(nom_x, true_x, out_3276244028421344337);
}
void pose_H_mod_fun(double *state, double *out_3027202661191403252) {
  H_mod_fun(state, out_3027202661191403252);
}
void pose_f_fun(double *state, double dt, double *out_1694909997772681373) {
  f_fun(state,  dt, out_1694909997772681373);
}
void pose_F_fun(double *state, double dt, double *out_7329719651390048604) {
  F_fun(state,  dt, out_7329719651390048604);
}
void pose_h_4(double *state, double *unused, double *out_3452650106359951024) {
  h_4(state, unused, out_3452650106359951024);
}
void pose_H_4(double *state, double *unused, double *out_1923156489118298873) {
  H_4(state, unused, out_1923156489118298873);
}
void pose_h_10(double *state, double *unused, double *out_2947394259008857773) {
  h_10(state, unused, out_2947394259008857773);
}
void pose_H_10(double *state, double *unused, double *out_8876134415720308413) {
  H_10(state, unused, out_8876134415720308413);
}
void pose_h_13(double *state, double *unused, double *out_1604531653721123784) {
  h_13(state, unused, out_1604531653721123784);
}
void pose_H_13(double *state, double *unused, double *out_1289117336214033928) {
  H_13(state, unused, out_1289117336214033928);
}
void pose_h_14(double *state, double *unused, double *out_2048301774952519882) {
  h_14(state, unused, out_2048301774952519882);
}
void pose_H_14(double *state, double *unused, double *out_9042441769311512319) {
  H_14(state, unused, out_9042441769311512319);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
