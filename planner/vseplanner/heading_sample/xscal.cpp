//
// File: xscal.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "xscal.h"

// Function Definitions

//
// Arguments    : int n
//                double a
//                emxArray_real_T *x
//                int ix0
// Return Type  : void
//
void xscal(int n, double a, emxArray_real_T *x, int ix0)
{
  int i10;
  int k;
  i10 = (ix0 + n) - 1;
  for (k = ix0; k <= i10; k++) {
    x->data[k - 1] *= a;
  }
}

//
// File trailer for xscal.cpp
//
// [EOF]
//
