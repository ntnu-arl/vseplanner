//
// File: svd.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "svd.h"
#include "heading_sample_emxutil.h"
#include "svd1.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
//                emxArray_real_T *S
//                double *V
// Return Type  : void
//
void svd(const emxArray_real_T *A, emxArray_real_T *U, emxArray_real_T *S,
         double *V)
{
  boolean_T p;
  int k;
  emxArray_real_T *r0;
  double s_data[1];
  int s_size[1];
  double V1;
  int i6;
  emxArray_real_T *U1;
  p = true;
  for (k = 0; k + 1 <= A->size[0]; k++) {
    if (p && ((!rtIsInf(A->data[k])) && (!rtIsNaN(A->data[k])))) {
      p = true;
    } else {
      p = false;
    }
  }

  if (p) {
    b_svd(A, U, s_data, s_size, &V1);
  } else {
    emxInit_real_T(&r0, 1);
    k = A->size[0];
    i6 = r0->size[0];
    r0->size[0] = k;
    emxEnsureCapacity_real_T1(r0, i6);
    for (i6 = 0; i6 < k; i6++) {
      r0->data[i6] = 0.0;
    }

    emxInit_real_T1(&U1, 2);
    b_svd(r0, U1, s_data, s_size, &V1);
    i6 = U->size[0] * U->size[1];
    U->size[0] = U1->size[0];
    U->size[1] = U1->size[1];
    emxEnsureCapacity_real_T(U, i6);
    k = U1->size[0] * U1->size[1];
    emxFree_real_T(&r0);
    emxFree_real_T(&U1);
    for (i6 = 0; i6 < k; i6++) {
      U->data[i6] = rtNaN;
    }

    k = s_size[0];
    for (i6 = 0; i6 < k; i6++) {
      s_data[i6] = rtNaN;
    }

    V1 = rtNaN;
  }

  i6 = S->size[0];
  S->size[0] = U->size[1];
  emxEnsureCapacity_real_T1(S, i6);
  k = U->size[1];
  for (i6 = 0; i6 < k; i6++) {
    S->data[i6] = 0.0;
  }

  k = 0;
  while (k <= s_size[0] - 1) {
    S->data[0] = s_data[0];
    k = 1;
  }

  *V = V1;
}

//
// File trailer for svd.cpp
//
// [EOF]
//
