#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4081214693445522564) {
   out_4081214693445522564[0] = delta_x[0] + nom_x[0];
   out_4081214693445522564[1] = delta_x[1] + nom_x[1];
   out_4081214693445522564[2] = delta_x[2] + nom_x[2];
   out_4081214693445522564[3] = delta_x[3] + nom_x[3];
   out_4081214693445522564[4] = delta_x[4] + nom_x[4];
   out_4081214693445522564[5] = delta_x[5] + nom_x[5];
   out_4081214693445522564[6] = delta_x[6] + nom_x[6];
   out_4081214693445522564[7] = delta_x[7] + nom_x[7];
   out_4081214693445522564[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7803987367555923231) {
   out_7803987367555923231[0] = -nom_x[0] + true_x[0];
   out_7803987367555923231[1] = -nom_x[1] + true_x[1];
   out_7803987367555923231[2] = -nom_x[2] + true_x[2];
   out_7803987367555923231[3] = -nom_x[3] + true_x[3];
   out_7803987367555923231[4] = -nom_x[4] + true_x[4];
   out_7803987367555923231[5] = -nom_x[5] + true_x[5];
   out_7803987367555923231[6] = -nom_x[6] + true_x[6];
   out_7803987367555923231[7] = -nom_x[7] + true_x[7];
   out_7803987367555923231[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8573730827112540197) {
   out_8573730827112540197[0] = 1.0;
   out_8573730827112540197[1] = 0;
   out_8573730827112540197[2] = 0;
   out_8573730827112540197[3] = 0;
   out_8573730827112540197[4] = 0;
   out_8573730827112540197[5] = 0;
   out_8573730827112540197[6] = 0;
   out_8573730827112540197[7] = 0;
   out_8573730827112540197[8] = 0;
   out_8573730827112540197[9] = 0;
   out_8573730827112540197[10] = 1.0;
   out_8573730827112540197[11] = 0;
   out_8573730827112540197[12] = 0;
   out_8573730827112540197[13] = 0;
   out_8573730827112540197[14] = 0;
   out_8573730827112540197[15] = 0;
   out_8573730827112540197[16] = 0;
   out_8573730827112540197[17] = 0;
   out_8573730827112540197[18] = 0;
   out_8573730827112540197[19] = 0;
   out_8573730827112540197[20] = 1.0;
   out_8573730827112540197[21] = 0;
   out_8573730827112540197[22] = 0;
   out_8573730827112540197[23] = 0;
   out_8573730827112540197[24] = 0;
   out_8573730827112540197[25] = 0;
   out_8573730827112540197[26] = 0;
   out_8573730827112540197[27] = 0;
   out_8573730827112540197[28] = 0;
   out_8573730827112540197[29] = 0;
   out_8573730827112540197[30] = 1.0;
   out_8573730827112540197[31] = 0;
   out_8573730827112540197[32] = 0;
   out_8573730827112540197[33] = 0;
   out_8573730827112540197[34] = 0;
   out_8573730827112540197[35] = 0;
   out_8573730827112540197[36] = 0;
   out_8573730827112540197[37] = 0;
   out_8573730827112540197[38] = 0;
   out_8573730827112540197[39] = 0;
   out_8573730827112540197[40] = 1.0;
   out_8573730827112540197[41] = 0;
   out_8573730827112540197[42] = 0;
   out_8573730827112540197[43] = 0;
   out_8573730827112540197[44] = 0;
   out_8573730827112540197[45] = 0;
   out_8573730827112540197[46] = 0;
   out_8573730827112540197[47] = 0;
   out_8573730827112540197[48] = 0;
   out_8573730827112540197[49] = 0;
   out_8573730827112540197[50] = 1.0;
   out_8573730827112540197[51] = 0;
   out_8573730827112540197[52] = 0;
   out_8573730827112540197[53] = 0;
   out_8573730827112540197[54] = 0;
   out_8573730827112540197[55] = 0;
   out_8573730827112540197[56] = 0;
   out_8573730827112540197[57] = 0;
   out_8573730827112540197[58] = 0;
   out_8573730827112540197[59] = 0;
   out_8573730827112540197[60] = 1.0;
   out_8573730827112540197[61] = 0;
   out_8573730827112540197[62] = 0;
   out_8573730827112540197[63] = 0;
   out_8573730827112540197[64] = 0;
   out_8573730827112540197[65] = 0;
   out_8573730827112540197[66] = 0;
   out_8573730827112540197[67] = 0;
   out_8573730827112540197[68] = 0;
   out_8573730827112540197[69] = 0;
   out_8573730827112540197[70] = 1.0;
   out_8573730827112540197[71] = 0;
   out_8573730827112540197[72] = 0;
   out_8573730827112540197[73] = 0;
   out_8573730827112540197[74] = 0;
   out_8573730827112540197[75] = 0;
   out_8573730827112540197[76] = 0;
   out_8573730827112540197[77] = 0;
   out_8573730827112540197[78] = 0;
   out_8573730827112540197[79] = 0;
   out_8573730827112540197[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3756193624839508385) {
   out_3756193624839508385[0] = state[0];
   out_3756193624839508385[1] = state[1];
   out_3756193624839508385[2] = state[2];
   out_3756193624839508385[3] = state[3];
   out_3756193624839508385[4] = state[4];
   out_3756193624839508385[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3756193624839508385[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3756193624839508385[7] = state[7];
   out_3756193624839508385[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8428133878130417062) {
   out_8428133878130417062[0] = 1;
   out_8428133878130417062[1] = 0;
   out_8428133878130417062[2] = 0;
   out_8428133878130417062[3] = 0;
   out_8428133878130417062[4] = 0;
   out_8428133878130417062[5] = 0;
   out_8428133878130417062[6] = 0;
   out_8428133878130417062[7] = 0;
   out_8428133878130417062[8] = 0;
   out_8428133878130417062[9] = 0;
   out_8428133878130417062[10] = 1;
   out_8428133878130417062[11] = 0;
   out_8428133878130417062[12] = 0;
   out_8428133878130417062[13] = 0;
   out_8428133878130417062[14] = 0;
   out_8428133878130417062[15] = 0;
   out_8428133878130417062[16] = 0;
   out_8428133878130417062[17] = 0;
   out_8428133878130417062[18] = 0;
   out_8428133878130417062[19] = 0;
   out_8428133878130417062[20] = 1;
   out_8428133878130417062[21] = 0;
   out_8428133878130417062[22] = 0;
   out_8428133878130417062[23] = 0;
   out_8428133878130417062[24] = 0;
   out_8428133878130417062[25] = 0;
   out_8428133878130417062[26] = 0;
   out_8428133878130417062[27] = 0;
   out_8428133878130417062[28] = 0;
   out_8428133878130417062[29] = 0;
   out_8428133878130417062[30] = 1;
   out_8428133878130417062[31] = 0;
   out_8428133878130417062[32] = 0;
   out_8428133878130417062[33] = 0;
   out_8428133878130417062[34] = 0;
   out_8428133878130417062[35] = 0;
   out_8428133878130417062[36] = 0;
   out_8428133878130417062[37] = 0;
   out_8428133878130417062[38] = 0;
   out_8428133878130417062[39] = 0;
   out_8428133878130417062[40] = 1;
   out_8428133878130417062[41] = 0;
   out_8428133878130417062[42] = 0;
   out_8428133878130417062[43] = 0;
   out_8428133878130417062[44] = 0;
   out_8428133878130417062[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8428133878130417062[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8428133878130417062[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8428133878130417062[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8428133878130417062[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8428133878130417062[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8428133878130417062[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8428133878130417062[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8428133878130417062[53] = -9.8000000000000007*dt;
   out_8428133878130417062[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8428133878130417062[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8428133878130417062[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8428133878130417062[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8428133878130417062[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8428133878130417062[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8428133878130417062[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8428133878130417062[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8428133878130417062[62] = 0;
   out_8428133878130417062[63] = 0;
   out_8428133878130417062[64] = 0;
   out_8428133878130417062[65] = 0;
   out_8428133878130417062[66] = 0;
   out_8428133878130417062[67] = 0;
   out_8428133878130417062[68] = 0;
   out_8428133878130417062[69] = 0;
   out_8428133878130417062[70] = 1;
   out_8428133878130417062[71] = 0;
   out_8428133878130417062[72] = 0;
   out_8428133878130417062[73] = 0;
   out_8428133878130417062[74] = 0;
   out_8428133878130417062[75] = 0;
   out_8428133878130417062[76] = 0;
   out_8428133878130417062[77] = 0;
   out_8428133878130417062[78] = 0;
   out_8428133878130417062[79] = 0;
   out_8428133878130417062[80] = 1;
}
void h_25(double *state, double *unused, double *out_6852182668869851547) {
   out_6852182668869851547[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8524457231801112947) {
   out_8524457231801112947[0] = 0;
   out_8524457231801112947[1] = 0;
   out_8524457231801112947[2] = 0;
   out_8524457231801112947[3] = 0;
   out_8524457231801112947[4] = 0;
   out_8524457231801112947[5] = 0;
   out_8524457231801112947[6] = 1;
   out_8524457231801112947[7] = 0;
   out_8524457231801112947[8] = 0;
}
void h_24(double *state, double *unused, double *out_5832356844480796856) {
   out_5832356844480796856[0] = state[4];
   out_5832356844480796856[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3777947904834603913) {
   out_3777947904834603913[0] = 0;
   out_3777947904834603913[1] = 0;
   out_3777947904834603913[2] = 0;
   out_3777947904834603913[3] = 0;
   out_3777947904834603913[4] = 1;
   out_3777947904834603913[5] = 0;
   out_3777947904834603913[6] = 0;
   out_3777947904834603913[7] = 0;
   out_3777947904834603913[8] = 0;
   out_3777947904834603913[9] = 0;
   out_3777947904834603913[10] = 0;
   out_3777947904834603913[11] = 0;
   out_3777947904834603913[12] = 0;
   out_3777947904834603913[13] = 0;
   out_3777947904834603913[14] = 1;
   out_3777947904834603913[15] = 0;
   out_3777947904834603913[16] = 0;
   out_3777947904834603913[17] = 0;
}
void h_30(double *state, double *unused, double *out_608376095379036778) {
   out_608376095379036778[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3005596500416821914) {
   out_3005596500416821914[0] = 0;
   out_3005596500416821914[1] = 0;
   out_3005596500416821914[2] = 0;
   out_3005596500416821914[3] = 0;
   out_3005596500416821914[4] = 1;
   out_3005596500416821914[5] = 0;
   out_3005596500416821914[6] = 0;
   out_3005596500416821914[7] = 0;
   out_3005596500416821914[8] = 0;
}
void h_26(double *state, double *unused, double *out_5834283311214361725) {
   out_5834283311214361725[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6617760872147638068) {
   out_6617760872147638068[0] = 0;
   out_6617760872147638068[1] = 0;
   out_6617760872147638068[2] = 0;
   out_6617760872147638068[3] = 0;
   out_6617760872147638068[4] = 0;
   out_6617760872147638068[5] = 0;
   out_6617760872147638068[6] = 0;
   out_6617760872147638068[7] = 1;
   out_6617760872147638068[8] = 0;
}
void h_27(double *state, double *unused, double *out_596357946910804293) {
   out_596357946910804293[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5180359812217246825) {
   out_5180359812217246825[0] = 0;
   out_5180359812217246825[1] = 0;
   out_5180359812217246825[2] = 0;
   out_5180359812217246825[3] = 1;
   out_5180359812217246825[4] = 0;
   out_5180359812217246825[5] = 0;
   out_5180359812217246825[6] = 0;
   out_5180359812217246825[7] = 0;
   out_5180359812217246825[8] = 0;
}
void h_29(double *state, double *unused, double *out_8043519529807737591) {
   out_8043519529807737591[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6893722539086797858) {
   out_6893722539086797858[0] = 0;
   out_6893722539086797858[1] = 1;
   out_6893722539086797858[2] = 0;
   out_6893722539086797858[3] = 0;
   out_6893722539086797858[4] = 0;
   out_6893722539086797858[5] = 0;
   out_6893722539086797858[6] = 0;
   out_6893722539086797858[7] = 0;
   out_6893722539086797858[8] = 0;
}
void h_28(double *state, double *unused, double *out_5598031214571649938) {
   out_5598031214571649938[0] = state[0];
}
void H_28(double *state, double *unused, double *out_6470622517553223184) {
   out_6470622517553223184[0] = 1;
   out_6470622517553223184[1] = 0;
   out_6470622517553223184[2] = 0;
   out_6470622517553223184[3] = 0;
   out_6470622517553223184[4] = 0;
   out_6470622517553223184[5] = 0;
   out_6470622517553223184[6] = 0;
   out_6470622517553223184[7] = 0;
   out_6470622517553223184[8] = 0;
}
void h_31(double *state, double *unused, double *out_7009369337220980692) {
   out_7009369337220980692[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2845611591396621416) {
   out_2845611591396621416[0] = 0;
   out_2845611591396621416[1] = 0;
   out_2845611591396621416[2] = 0;
   out_2845611591396621416[3] = 0;
   out_2845611591396621416[4] = 0;
   out_2845611591396621416[5] = 0;
   out_2845611591396621416[6] = 0;
   out_2845611591396621416[7] = 0;
   out_2845611591396621416[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4081214693445522564) {
  err_fun(nom_x, delta_x, out_4081214693445522564);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7803987367555923231) {
  inv_err_fun(nom_x, true_x, out_7803987367555923231);
}
void car_H_mod_fun(double *state, double *out_8573730827112540197) {
  H_mod_fun(state, out_8573730827112540197);
}
void car_f_fun(double *state, double dt, double *out_3756193624839508385) {
  f_fun(state,  dt, out_3756193624839508385);
}
void car_F_fun(double *state, double dt, double *out_8428133878130417062) {
  F_fun(state,  dt, out_8428133878130417062);
}
void car_h_25(double *state, double *unused, double *out_6852182668869851547) {
  h_25(state, unused, out_6852182668869851547);
}
void car_H_25(double *state, double *unused, double *out_8524457231801112947) {
  H_25(state, unused, out_8524457231801112947);
}
void car_h_24(double *state, double *unused, double *out_5832356844480796856) {
  h_24(state, unused, out_5832356844480796856);
}
void car_H_24(double *state, double *unused, double *out_3777947904834603913) {
  H_24(state, unused, out_3777947904834603913);
}
void car_h_30(double *state, double *unused, double *out_608376095379036778) {
  h_30(state, unused, out_608376095379036778);
}
void car_H_30(double *state, double *unused, double *out_3005596500416821914) {
  H_30(state, unused, out_3005596500416821914);
}
void car_h_26(double *state, double *unused, double *out_5834283311214361725) {
  h_26(state, unused, out_5834283311214361725);
}
void car_H_26(double *state, double *unused, double *out_6617760872147638068) {
  H_26(state, unused, out_6617760872147638068);
}
void car_h_27(double *state, double *unused, double *out_596357946910804293) {
  h_27(state, unused, out_596357946910804293);
}
void car_H_27(double *state, double *unused, double *out_5180359812217246825) {
  H_27(state, unused, out_5180359812217246825);
}
void car_h_29(double *state, double *unused, double *out_8043519529807737591) {
  h_29(state, unused, out_8043519529807737591);
}
void car_H_29(double *state, double *unused, double *out_6893722539086797858) {
  H_29(state, unused, out_6893722539086797858);
}
void car_h_28(double *state, double *unused, double *out_5598031214571649938) {
  h_28(state, unused, out_5598031214571649938);
}
void car_H_28(double *state, double *unused, double *out_6470622517553223184) {
  H_28(state, unused, out_6470622517553223184);
}
void car_h_31(double *state, double *unused, double *out_7009369337220980692) {
  h_31(state, unused, out_7009369337220980692);
}
void car_H_31(double *state, double *unused, double *out_2845611591396621416) {
  H_31(state, unused, out_2845611591396621416);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
