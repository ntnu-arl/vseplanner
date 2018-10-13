//
// File: norm.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "norm.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *x
// Return Type  : double
//
double norm(const emxArray_real_T *x)
{
  double y;
  double scale;
  int k;
  if (x->size[0] == 0) {
    y = 0.0;
  } else {
    y = 0.0;
    if (x->size[0] == 1) {
      y = 1.0;
    } else {
      scale = 3.3121686421112381E-170;
      for (k = 1; k <= x->size[0]; k++) {
        if (1.0 > scale) {
          y = 1.0 + y * scale * scale;
          scale = 1.0;
        } else {
          y++;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

//
// File trailer for norm.cpp
//
// [EOF]
//
