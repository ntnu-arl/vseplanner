//
// File: diag.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "diag.h"
#include "heading_sample_emxutil.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *v
//                emxArray_real_T *d
// Return Type  : void
//
void diag(const emxArray_real_T *v, emxArray_real_T *d)
{
  int unnamed_idx_0;
  int unnamed_idx_1;
  int i5;
  unnamed_idx_0 = v->size[0];
  unnamed_idx_1 = v->size[0];
  i5 = d->size[0] * d->size[1];
  d->size[0] = unnamed_idx_0;
  d->size[1] = unnamed_idx_1;
  emxEnsureCapacity_real_T(d, i5);
  unnamed_idx_0 *= unnamed_idx_1;
  for (i5 = 0; i5 < unnamed_idx_0; i5++) {
    d->data[i5] = 0.0;
  }

  for (unnamed_idx_0 = 0; unnamed_idx_0 + 1 <= v->size[0]; unnamed_idx_0++) {
    d->data[unnamed_idx_0 + d->size[0] * unnamed_idx_0] = v->data[unnamed_idx_0];
  }
}

//
// File trailer for diag.cpp
//
// [EOF]
//
