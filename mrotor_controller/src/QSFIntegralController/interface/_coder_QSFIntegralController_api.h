//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_QSFIntegralController_api.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 2025-01-31 03:51:16:595
//

#ifndef _CODER_QSFINTEGRALCONTROLLER_API_H
#define _CODER_QSFINTEGRALCONTROLLER_API_H

// Include Files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void QSFIntegralController(real_T z[15], real_T k[13], real_T param[4],
                           real_T ref_1[5], real_T ref_2[5], real_T ref_3[5],
                           real_T F[3]);

void QSFIntegralController_api(const mxArray *const prhs[6],
                               const mxArray **plhs);

void QSFIntegralController_atexit();

void QSFIntegralController_initialize();

void QSFIntegralController_terminate();

void QSFIntegralController_xil_shutdown();

void QSFIntegralController_xil_terminate();

#endif
//
// File trailer for _coder_QSFIntegralController_api.h
//
// [EOF]
//
