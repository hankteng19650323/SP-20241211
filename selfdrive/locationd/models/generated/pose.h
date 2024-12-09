#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6017775153784914369);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_3276244028421344337);
void pose_H_mod_fun(double *state, double *out_3027202661191403252);
void pose_f_fun(double *state, double dt, double *out_1694909997772681373);
void pose_F_fun(double *state, double dt, double *out_7329719651390048604);
void pose_h_4(double *state, double *unused, double *out_3452650106359951024);
void pose_H_4(double *state, double *unused, double *out_1923156489118298873);
void pose_h_10(double *state, double *unused, double *out_2947394259008857773);
void pose_H_10(double *state, double *unused, double *out_8876134415720308413);
void pose_h_13(double *state, double *unused, double *out_1604531653721123784);
void pose_H_13(double *state, double *unused, double *out_1289117336214033928);
void pose_h_14(double *state, double *unused, double *out_2048301774952519882);
void pose_H_14(double *state, double *unused, double *out_9042441769311512319);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}