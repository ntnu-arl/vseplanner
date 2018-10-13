//
// File: heading_sample.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//
#ifndef HEADING_SAMPLE_H
#define HEADING_SAMPLE_H

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "heading_sample_types.h"

// Function Declarations
extern void heading_sample(double dphi_max, double v_max, double t_allow, double
  dphi_lim_min, double dphi_lim_max, double phi_0, double phi_n, double n,
  double m, const double d_data[], const int d_size[1], double max_iter, double *
  res, emxArray_real_T *dphi);

#endif

//
// File trailer for heading_sample.h
//
// [EOF]
//
