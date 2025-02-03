//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: QSFIntegralController.cpp
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 2025-02-02 04:22:22:687
//

// Include Files
#include "QSFIntegralController.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <emmintrin.h>

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions
//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else {
    double d;
    double d1;
    d = std::abs(u0);
    d1 = std::abs(u1);
    if (std::isinf(u1)) {
      if (d == 1.0) {
        y = 1.0;
      } else if (d > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d1 == 0.0) {
      y = 1.0;
    } else if (d1 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = std::pow(u0, u1);
    }
  }
  return y;
}

//
// Arguments    : const double z[15]
//                const double k[13]
//                const double param[4]
//                const double ref_1[5]
//                const double ref_2[5]
//                const double ref_3[5]
//                double F[3]
//                double xi_dot[3]
// Return Type  : void
//
void QSFIntegralController(const double z[15], const double k[13],
                           const double param[4], const double ref_1[5],
                           const double ref_2[5], const double ref_3[5],
                           double F[3], double xi_dot[3])
{
  double c_z[9];
  double b_z[5];
  double b_k;
  double b_z_tmp;
  double c_k;
  double c_z_tmp;
  double d_z_tmp;
  double e3_dt;
  double e_z_tmp;
  double nu__3;
  double nu__3_dt;
  double omega_idx_1;
  double z_idx_1_tmp;
  double z_tmp;
  //     %% Input Interface
  //  p = [p__1;p__2;p__3];
  //  q = [q__1;q__2;q__3];
  //  v = [v__1;v__2;v__3];
  //     %% The output
  //     %% Aux input
  //  nu__3 = -k3*[y3;y3_dt];
  //  nu__3_dt = -k3*[y3_dt;nu__3];
  //  along z-axis
  e3_dt = z[8] - ref_3[1];
  //  set nu_3 = y3_ddt
  z_idx_1_tmp = z[2] - ref_3[0];
  nu__3 = ((-k[10] * z[14] + -k[11] * z_idx_1_tmp) + -k[12] * e3_dt) + ref_3[2];
  nu__3_dt =
      ((-k[10] * z_idx_1_tmp + -k[11] * e3_dt) + -k[12] * (nu__3 - ref_3[2])) +
      ref_3[3];
  //  k31=k3(1);
  //  k32=k3(2);
  //  Further derivatives for output
  //  along x-axis
  //  nu__1 = - k1*[y1; y1_dt; y1_ddt; y1_dddt];
  //  along y-axis
  //  nu__2 = - k2*[y2; y2_dt; y2_ddt; y2_dddt];
  //     %% Transformed input, u
  // u__2 = -L * m__q * (((2 * omega__2 * omega__3 * g - 2 * nu__3 * omega__2 *
  // omega__3 + nu__2) * q__3 ^ 3) + ((((2 * omega__3 * k32 - 3 * omega__1 *
  // omega__2) * nu__3 + 3 * omega__1 * omega__2 * g + 2 * v__3 * omega__3 *
  // k31) * q__1 - ((k32 ^ 2 + omega__2 ^ 2 - omega__3 ^ 2 - k31) * nu__3 + v__3
  // * k31 * k32 - omega__2 ^ 2 * g + omega__3 ^ 2 * g) * q__2) * q__3 ^ 2) +
  // ((((2 * omega__1 * k32 - 3 * omega__2 * omega__3) * nu__3 + 2 * v__3 *
  // omega__1 * k31 + 3 * omega__2 * omega__3 * g) * q__1 ^ 2) + 0.3e1 * q__2 *
  // (((omega__1 * omega__3) + 0.2e1 / 0.3e1 * omega__2 * k32) * nu__3 -
  // (omega__1 * omega__3 * g) + 0.2e1 / 0.3e1 * v__3 * omega__2 * k31) * q__1 +
  // ((-2 * omega__1 * k32 + omega__2 * omega__3) * nu__3) - (2 * v__3 *
  // omega__1 * k31) - (omega__2 * omega__3 * g)) * q__3 + (2 * (-2 * omega__1 *
  // omega__2 * q__1 ^ 3 + q__2 * (omega__1 ^ 2 - omega__2 ^ 2) * q__1 ^ 2 + 2 *
  // omega__1 * omega__2 * q__1 - omega__1 ^ 2 * q__2) * (-g + nu__3))) / q__3 /
  // (-g + nu__3); u__3 = L * m__q * ((((2 * omega__2 * k32 + omega__1 *
  // omega__3) * nu__3 - omega__1 * omega__3 * g + 2 * v__3 * omega__2 * k31 +
  // nu__1) * q__3 ^ 3) + ((((-k32 ^ 2 + omega__1 ^ 2 - 2 * omega__2 ^ 2 +
  // omega__3 ^ 2 + k31) * nu__3 - omega__1 ^ 2 * g + 2 * omega__2 ^ 2 * g -
  // omega__3 ^ 2 * g - v__3 * k31 * k32) * q__1 + q__2 * ((-2 * omega__3 * k32
  // + omega__1 * omega__2) * nu__3 - omega__1 * omega__2 * g - 2 * v__3 *
  // omega__3 * k31)) * q__3 ^ 2) + ((((2 * omega__2 * k32 + 3 * omega__1 *
  // omega__3) * nu__3 - 3 * omega__1 * omega__3 * g + 2 * v__3 * omega__2 *
  // k31) * q__1 ^ 2) - 0.2e1 * (((omega__1 * k32) - 0.3e1 / 0.2e1 * omega__2 *
  // omega__3) * nu__3 + (v__3 * omega__1 * k31) + 0.3e1 / 0.2e1 * omega__2 *
  // omega__3 * g) * q__2 * q__1 - (2 * omega__1 * omega__3 * (-g + nu__3))) *
  // q__3 + (2 * ((omega__1 ^ 2 - omega__2 ^ 2) * q__1 ^ 2 + 2 * omega__1 *
  // omega__2 * q__1 * q__2 - omega__1 ^ 2) * (-g + nu__3) * q__1)) / q__3 / (-g
  // + nu__3);
  b_z[0] = z[13];
  z_tmp = z[1] - ref_2[0];
  b_z[1] = z_tmp;
  b_z[2] = z[7] - ref_2[1];
  omega_idx_1 = -param[3] + nu__3;
  b_z[3] = z[4] * omega_idx_1 / z[5] - ref_2[2];
  b_z_tmp = param[3] - nu__3;
  c_z_tmp = z[5] * z[5];
  e3_dt = z[3] * z[10] - z[4] * z[9];
  b_z[4] = ((z[9] * b_z_tmp * c_z_tmp +
             (z[4] * nu__3_dt - z[3] * z[11] * b_z_tmp) * z[5]) -
            z[4] * e3_dt * b_z_tmp) /
               c_z_tmp -
           ref_2[3];
  b_k = 0.0;
  for (int i{0}; i < 5; i++) {
    b_k += -k[i + 5] * b_z[i];
  }
  b_z[0] = z[12];
  d_z_tmp = z[0] - ref_1[0];
  b_z[1] = d_z_tmp;
  b_z[2] = z[6] - ref_1[1];
  b_z[3] = z[3] * omega_idx_1 / z[5] - ref_1[2];
  e_z_tmp = z[4] * z[11] * b_z_tmp;
  b_z[4] = ((-z[10] * b_z_tmp * c_z_tmp + (z[3] * nu__3_dt + e_z_tmp) * z[5]) -
            z[3] * e3_dt * b_z_tmp) /
               c_z_tmp -
           ref_1[3];
  c_k = 0.0;
  for (int i{0}; i < 5; i++) {
    c_k += -k[i] * b_z[i];
  }
  __m128d r;
  __m128d r1;
  double b_c_tmp;
  double c_c_tmp;
  double c_tmp;
  double f_z_tmp;
  double omega_idx_0;
  double omega_idx_1_tmp;
  double z_tmp_tmp;
  //     %% Input transfomration
  //  Qinv = [q__1 * (m__q + m__p) - (q__2 * q__1 / q__3), (q__1 ^ 2 - 1) /
  //  q__3; q__2 * (m__q + m__p) (-q__2 ^ 2 + 1) / q__3 q__2 * q__1 / q__3; q__3
  //  * (m__q + m__p) -q__2 q__1;]; actual force, F
  c_tmp = z[9] * z[9];
  b_c_tmp = z[10] * z[10];
  c_c_tmp = z[11] * z[11];
  e3_dt = param[0] + param[1];
  c_z[0] = z[3] * e3_dt;
  z_tmp_tmp = z[3] * z[4];
  nu__3 = z_tmp_tmp / z[5];
  c_z[3] = -nu__3;
  f_z_tmp = z[3] * z[3];
  c_z[6] = (f_z_tmp - 1.0) / z[5];
  c_z[1] = z[4] * e3_dt;
  c_z[4] = (-(z[4] * z[4]) + 1.0) / z[5];
  c_z[7] = nu__3;
  c_z[2] = z[5] * e3_dt;
  c_z[5] = -z[4];
  c_z[8] = z[3];
  omega_idx_0 = omega_idx_1 / z[5] +
                param[1] * param[2] / e3_dt * ((c_tmp + b_c_tmp) + c_c_tmp);
  e3_dt = z[11] * b_z_tmp;
  omega_idx_1_tmp = rt_powd_snf(z[5], 3.0);
  omega_idx_1 =
      -2.0 * param[1] *
      ((((-z[10] * z[11] * b_z_tmp - (b_k + ref_2[4]) / 2.0) * omega_idx_1_tmp +
         ((-z[4] * b_z_tmp * b_c_tmp - 3.0 * z[9] * z[3] * b_z_tmp * z[10]) +
          (2.0 * z[3] * nu__3_dt + e_z_tmp) * z[11]) *
             c_z_tmp / 2.0) +
        (((-1.5 * z[11] * b_z_tmp * f_z_tmp + z_tmp_tmp * nu__3_dt) +
          e3_dt / 2.0) *
             z[10] +
         1.5 * z[9] *
             ((0.66666666666666663 * f_z_tmp * nu__3_dt + e_z_tmp * z[3]) -
              0.66666666666666663 * nu__3_dt)) *
            z[5]) +
       b_z_tmp * ((-b_c_tmp * f_z_tmp * z[4] +
                   2.0 * z[9] * (-rt_powd_snf(z[3], 3.0) + z[3]) * z[10]) +
                  c_tmp * z[4] * (z[3] - 1.0) * (z[3] + 1.0))) *
      param[2] / z[5] / b_z_tmp;
  nu__3 = nu__3_dt * z[10];
  e3_dt *= z[9];
  e3_dt =
      2.0 * param[1] *
      (((((e3_dt / 2.0 + nu__3) - (c_k + ref_1[4]) / 2.0) * omega_idx_1_tmp +
         (((c_tmp - 2.0 * b_c_tmp) + c_c_tmp) * b_z_tmp * z[3] +
          (z[10] * b_z_tmp * z[9] - 2.0 * z[11] * nu__3_dt) * z[4]) *
             c_z_tmp / 2.0) +
        (((1.5 * z[11] * b_z_tmp * z[9] + nu__3) * f_z_tmp -
          (z[9] * nu__3_dt - 1.5 * z[10] * z[11] * b_z_tmp) * z[4] * z[3]) -
         e3_dt) *
            z[5]) +
       (((c_tmp - b_c_tmp) * f_z_tmp + 2.0 * z[9] * z[10] * z[3] * z[4]) -
        c_tmp) *
           b_z_tmp * z[3]) *
      param[2] / z[5] / b_z_tmp;
  r = _mm_loadu_pd(&c_z[0]);
  r = _mm_mul_pd(r, _mm_set1_pd(omega_idx_0));
  r1 = _mm_loadu_pd(&c_z[3]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(omega_idx_1));
  r = _mm_add_pd(r, r1);
  r1 = _mm_loadu_pd(&c_z[6]);
  r1 = _mm_mul_pd(r1, _mm_set1_pd(e3_dt));
  r = _mm_add_pd(r, r1);
  _mm_storeu_pd(&F[0], r);
  F[2] = (c_z[2] * omega_idx_0 + c_z[5] * omega_idx_1) + c_z[8] * e3_dt;
  xi_dot[0] = d_z_tmp;
  xi_dot[1] = z_tmp;
  xi_dot[2] = z_idx_1_tmp;
}

//
// File trailer for QSFIntegralController.cpp
//
// [EOF]
//
