#ifndef IDENTIFICATION_HPP
#define IDENTIFICATION_HPP

// グローバル関数
void get_theta(float *theta_l, float *i_m);
void get_phi(float *theta_k, float *phi_k);
void get_Y(float *i_k, float *Y_k);
int solve_gauss(float *A_g, float *b_g, float *x_g);

#endif