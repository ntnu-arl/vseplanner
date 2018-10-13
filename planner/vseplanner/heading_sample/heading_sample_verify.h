//
// File: heading_sample_verify.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//
#ifndef HEADING_SAMPLE_VERIFY_H
#define HEADING_SAMPLE_VERIFY_H

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "heading_sample_types.h"

// Function Declarations
extern boolean_T heading_sample_verify(double dphi_max, double v_max, double
  t_allow, double dphi_lim_min, double dphi_lim_max, double phi_0, double phi_n,
  double dphi_n_0, double n, const double d_data[], const emxArray_real_T *dphi);

#endif

//
// File trailer for heading_sample_verify.h
//
// [EOF]
//
