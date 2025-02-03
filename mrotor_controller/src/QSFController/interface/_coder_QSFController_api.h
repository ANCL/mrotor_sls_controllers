//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: _coder_QSFController_api.h
//
// MATLAB Coder version            : 24.1
// C/C++ source code generated on  : 2025-01-30 08:35:22:766
//

#ifndef _CODER_QSFCONTROLLER_API_H
#define _CODER_QSFCONTROLLER_API_H

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
void QSFController(real_T z[12], real_T k[10], real_T param[4], real_T ref_1[5],
                   real_T ref_2[5], real_T ref_3[5], real_T F[3]);

void QSFController_api(const mxArray *const prhs[6], const mxArray **plhs);

void QSFController_atexit();

void QSFController_initialize();

void QSFController_terminate();

void QSFController_xil_shutdown();

void QSFController_xil_terminate();

#endif
//
// File trailer for _coder_QSFController_api.h
//
// [EOF]
//
