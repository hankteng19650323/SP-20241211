#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4081214693445522564);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7803987367555923231);
void car_H_mod_fun(double *state, double *out_8573730827112540197);
void car_f_fun(double *state, double dt, double *out_3756193624839508385);
void car_F_fun(double *state, double dt, double *out_8428133878130417062);
void car_h_25(double *state, double *unused, double *out_6852182668869851547);
void car_H_25(double *state, double *unused, double *out_8524457231801112947);
void car_h_24(double *state, double *unused, double *out_5832356844480796856);
void car_H_24(double *state, double *unused, double *out_3777947904834603913);
void car_h_30(double *state, double *unused, double *out_608376095379036778);
void car_H_30(double *state, double *unused, double *out_3005596500416821914);
void car_h_26(double *state, double *unused, double *out_5834283311214361725);
void car_H_26(double *state, double *unused, double *out_6617760872147638068);
void car_h_27(double *state, double *unused, double *out_596357946910804293);
void car_H_27(double *state, double *unused, double *out_5180359812217246825);
void car_h_29(double *state, double *unused, double *out_8043519529807737591);
void car_H_29(double *state, double *unused, double *out_6893722539086797858);
void car_h_28(double *state, double *unused, double *out_5598031214571649938);
void car_H_28(double *state, double *unused, double *out_6470622517553223184);
void car_h_31(double *state, double *unused, double *out_7009369337220980692);
void car_H_31(double *state, double *unused, double *out_2845611591396621416);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}