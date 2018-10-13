//
// File: inv.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "inv.h"
#include "heading_sample_emxutil.h"

// Function Declarations
static void invNxN(const emxArray_real_T *x, emxArray_real_T *y);

// Function Definitions

//
// Arguments    : const emxArray_real_T *x
//                emxArray_real_T *y
// Return Type  : void
//
static void invNxN(const emxArray_real_T *x, emxArray_real_T *y)
{
  int n;
  int i8;
  int yk;
  emxArray_real_T *b_x;
  int b_n;
  emxArray_int32_T *ipiv;
  int k;
  int u1;
  emxArray_int32_T *p;
  int j;
  int mmj;
  int c;
  int ix;
  double smax;
  int jy;
  double s;
  int ijA;
  n = x->size[0];
  i8 = y->size[0] * y->size[1];
  y->size[0] = x->size[0];
  y->size[1] = x->size[1];
  emxEnsureCapacity_real_T(y, i8);
  yk = x->size[0] * x->size[1];
  for (i8 = 0; i8 < yk; i8++) {
    y->data[i8] = 0.0;
  }

  emxInit_real_T1(&b_x, 2);
  i8 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = x->size[0];
  b_x->size[1] = x->size[1];
  emxEnsureCapacity_real_T(b_x, i8);
  yk = x->size[0] * x->size[1];
  for (i8 = 0; i8 < yk; i8++) {
    b_x->data[i8] = x->data[i8];
  }

  yk = x->size[0];
  if (yk < 1) {
    b_n = 0;
  } else {
    b_n = yk;
  }

  emxInit_int32_T(&ipiv, 2);
  i8 = ipiv->size[0] * ipiv->size[1];
  ipiv->size[0] = 1;
  ipiv->size[1] = b_n;
  emxEnsureCapacity_int32_T(ipiv, i8);
  if (b_n > 0) {
    ipiv->data[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      ipiv->data[k - 1] = yk;
    }
  }

  if (x->size[0] < 1) {
    b_n = 0;
  } else {
    yk = x->size[0] - 1;
    u1 = x->size[0];
    if (yk < u1) {
      u1 = yk;
    }

    for (j = 0; j + 1 <= u1; j++) {
      mmj = n - j;
      c = j * (n + 1);
      if (mmj < 1) {
        yk = -1;
      } else {
        yk = 0;
        if (mmj > 1) {
          ix = c;
          smax = fabs(b_x->data[c]);
          for (k = 2; k <= mmj; k++) {
            ix++;
            s = fabs(b_x->data[ix]);
            if (s > smax) {
              yk = k - 1;
              smax = s;
            }
          }
        }
      }

      if (b_x->data[c + yk] != 0.0) {
        if (yk != 0) {
          ipiv->data[j] = (j + yk) + 1;
          ix = j;
          yk += j;
          for (k = 1; k <= n; k++) {
            smax = b_x->data[ix];
            b_x->data[ix] = b_x->data[yk];
            b_x->data[yk] = smax;
            ix += n;
            yk += n;
          }
        }

        i8 = c + mmj;
        for (jy = c + 1; jy + 1 <= i8; jy++) {
          b_x->data[jy] /= b_x->data[c];
        }
      }

      yk = n - j;
      b_n = (c + n) + 1;
      jy = c + n;
      for (k = 1; k < yk; k++) {
        smax = b_x->data[jy];
        if (b_x->data[jy] != 0.0) {
          ix = c + 1;
          i8 = mmj + b_n;
          for (ijA = b_n; ijA + 1 < i8; ijA++) {
            b_x->data[ijA] += b_x->data[ix] * -smax;
            ix++;
          }
        }

        jy += n;
        b_n += n;
      }
    }

    b_n = x->size[0];
  }

  emxInit_int32_T(&p, 2);
  i8 = p->size[0] * p->size[1];
  p->size[0] = 1;
  p->size[1] = b_n;
  emxEnsureCapacity_int32_T(p, i8);
  if (b_n > 0) {
    p->data[0] = 1;
    yk = 1;
    for (k = 2; k <= b_n; k++) {
      yk++;
      p->data[k - 1] = yk;
    }
  }

  for (k = 0; k < ipiv->size[1]; k++) {
    if (ipiv->data[k] > 1 + k) {
      yk = p->data[ipiv->data[k] - 1];
      p->data[ipiv->data[k] - 1] = p->data[k];
      p->data[k] = yk;
    }
  }

  emxFree_int32_T(&ipiv);
  for (k = 0; k + 1 <= n; k++) {
    c = p->data[k] - 1;
    y->data[k + y->size[0] * (p->data[k] - 1)] = 1.0;
    for (j = k; j + 1 <= n; j++) {
      if (y->data[j + y->size[0] * c] != 0.0) {
        for (jy = j + 1; jy + 1 <= n; jy++) {
          y->data[jy + y->size[0] * c] -= y->data[j + y->size[0] * c] *
            b_x->data[jy + b_x->size[0] * j];
        }
      }
    }
  }

  emxFree_int32_T(&p);
  if ((x->size[0] == 0) || ((y->size[0] == 0) || (y->size[1] == 0))) {
  } else {
    for (j = 1; j <= n; j++) {
      yk = n * (j - 1) - 1;
      for (k = n; k > 0; k--) {
        b_n = n * (k - 1) - 1;
        if (y->data[k + yk] != 0.0) {
          y->data[k + yk] /= b_x->data[k + b_n];
          for (jy = 1; jy < k; jy++) {
            y->data[jy + yk] -= y->data[k + yk] * b_x->data[jy + b_n];
          }
        }
      }
    }
  }

  emxFree_real_T(&b_x);
}

//
// Arguments    : emxArray_real_T *x
// Return Type  : void
//
void inv(emxArray_real_T *x)
{
  emxArray_real_T *b_x;
  int i11;
  int loop_ub;
  if (!((x->size[0] == 0) || (x->size[1] == 0))) {
    emxInit_real_T1(&b_x, 2);
    i11 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = x->size[0];
    b_x->size[1] = x->size[1];
    emxEnsureCapacity_real_T(b_x, i11);
    loop_ub = x->size[0] * x->size[1];
    for (i11 = 0; i11 < loop_ub; i11++) {
      b_x->data[i11] = x->data[i11];
    }

    invNxN(b_x, x);
    emxFree_real_T(&b_x);
  }
}

//
// File trailer for inv.cpp
//
// [EOF]
//
