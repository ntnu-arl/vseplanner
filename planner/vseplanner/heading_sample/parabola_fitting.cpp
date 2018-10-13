//
// File: parabola_fitting.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "parabola_fitting.h"

// Function Definitions

//
// mode:
//  1: 3 points at dphi = {-pi, 0, pi}
//  2: 3 points --> dphi
//  NOTE:
//  d: at max the length of each vertex is 2m
//  And: d * dphi_max / v_max < pi
// Arguments    : double v_max
//                double dphi_max
//                double d
//                double *a
//                double *b
//                double *err
// Return Type  : void
//
void parabola_fitting(double v_max, double dphi_max, double d, double *a, double
                      *b, double *err)
{
  double xt;
  *err = 1.0;
  if ((d > 2.0) || (d * dphi_max / v_max > 3.1415926535897931)) {
    *err = 2.0;
  }

  xt = 0.5 * (3.1415926535897931 + dphi_max / v_max * d);
  *b = d / v_max;
  *a = (0.5 * (3.1415926535897931 / dphi_max + d / v_max) - *b) / (xt * xt);
}

//
// File trailer for parabola_fitting.cpp
//
// [EOF]
//
