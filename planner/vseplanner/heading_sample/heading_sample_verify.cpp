//
// File: heading_sample_verify.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "heading_sample_verify.h"

// Function Definitions

//
// dphi_max: max rotation velocity
//  v_max: max linear velocity
//  t_allow: time budget
//  dphi_lim_min, dphi_lim_max: limit for dphi, make its less agreesive in
//  rotating. If not important, just set to -pi and pi
//  phi_0, phi_n: start and end configurations
//  dphi_n_0: max differential-accumulative angles from phi_0 to phi_n
//  n: number of links
//  m: number of samples
// Arguments    : double dphi_max
//                double v_max
//                double t_allow
//                double dphi_lim_min
//                double dphi_lim_max
//                double phi_0
//                double phi_n
//                double dphi_n_0
//                double n
//                const double d_data[]
//                const emxArray_real_T *dphi
// Return Type  : boolean_T
//
boolean_T heading_sample_verify(double dphi_max, double v_max, double t_allow,
  double dphi_lim_min, double dphi_lim_max, double phi_0, double phi_n, double
  dphi_n_0, double n, const double d_data[], const emxArray_real_T *dphi)
{
  boolean_T res;
  double gamma0;
  double t;
  int j;
  int exitg1;
  int ixstart;
  double y;
  double varargin_1_idx_1;
  int ix;
  boolean_T exitg2;

  //  This verification step use actual nonlinear-discrete function
  res = false;
  gamma0 = phi_n - phi_0;

  //  The smallest plane
  if (gamma0 > 3.1415926535897931) {
    gamma0 -= 6.2831853071795862;
  } else {
    if (gamma0 < -3.1415926535897931) {
      gamma0 += 6.2831853071795862;
    }
  }

  t = 0.0;
  j = 0;
  do {
    exitg1 = 0;
    if (j <= (int)n - 1) {
      y = d_data[j] / v_max;
      varargin_1_idx_1 = fabs(dphi->data[dphi->size[0] * j]) / dphi_max;
      ixstart = 1;
      if (rtIsNaN(y)) {
        ix = 2;
        exitg2 = false;
        while ((!exitg2) && (ix < 3)) {
          ixstart = 2;
          if (!rtIsNaN(varargin_1_idx_1)) {
            y = varargin_1_idx_1;
            exitg2 = true;
          } else {
            ix = 3;
          }
        }
      }

      if ((ixstart < 2) && (varargin_1_idx_1 > y)) {
        y = varargin_1_idx_1;
      }

      t += y;

      //  This check is for practical purpose, we don't want the robot
      //  rotates too fast
      //  But this will constraint the sample space a lot.
      if ((dphi->data[dphi->size[0] * j] > dphi_lim_max) || (dphi->data
           [dphi->size[0] * j] < dphi_lim_min)) {
        exitg1 = 1;
      } else {
        j++;
      }
    } else {
      ixstart = dphi->size[1];
      j = dphi->size[1];
      if (j == 0) {
        y = 0.0;
      } else {
        j = dphi->size[1];
        if (j == 0) {
          y = 0.0;
        } else {
          y = dphi->data[0];
          for (j = 2; j <= ixstart; j++) {
            y += dphi->data[dphi->size[0] * (j - 1)];
          }
        }
      }

      if ((!(fabs(y - (gamma0 + dphi_n_0)) > 0.01)) && (!(t - t_allow > 0.1 *
            t_allow))) {
        res = true;
      } else {
        //  due to numerical issue, dtmp ~ 0
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return res;
}

//
// File trailer for heading_sample_verify.cpp
//
// [EOF]
//
