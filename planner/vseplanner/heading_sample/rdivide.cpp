//
// File: rdivide.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "rdivide.h"
#include "heading_sample_emxutil.h"

// Function Definitions

//
// Arguments    : double x
//                const emxArray_real_T *y
//                emxArray_real_T *z
// Return Type  : void
//
void rdivide(double x, const emxArray_real_T *y, emxArray_real_T *z)
{
  int i4;
  int loop_ub;
  i4 = z->size[0];
  z->size[0] = y->size[0];
  emxEnsureCapacity_real_T1(z, i4);
  loop_ub = y->size[0];
  for (i4 = 0; i4 < loop_ub; i4++) {
    z->data[i4] = x / y->data[i4];
  }
}

//
// File trailer for rdivide.cpp
//
// [EOF]
//
