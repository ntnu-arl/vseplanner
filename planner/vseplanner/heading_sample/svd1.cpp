//
// File: svd1.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "svd1.h"
#include "xaxpy.h"
#include "xdotc.h"
#include "xnrm2.h"
#include "xscal.h"
#include "heading_sample_emxutil.h"
#include "xrotg.h"
#include "sqrt.h"

// Function Definitions

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
//                double s_data[]
//                int s_size[1]
//                double *V
// Return Type  : void
//
void b_svd(const emxArray_real_T *A, emxArray_real_T *U, double s_data[], int
           s_size[1], double *V)
{
  emxArray_real_T *b_A;
  int i7;
  int qjj;
  int n;
  int minnp;
  double s_data_idx_0;
  int unnamed_idx_1;
  int nct;
  double rt;
  double c_A;
  emxInit_real_T(&b_A, 1);
  i7 = b_A->size[0];
  b_A->size[0] = A->size[0];
  emxEnsureCapacity_real_T1(b_A, i7);
  qjj = A->size[0];
  for (i7 = 0; i7 < qjj; i7++) {
    b_A->data[i7] = A->data[i7];
  }

  n = A->size[0];
  minnp = !(A->size[0] < 1);
  s_data_idx_0 = 0.0;
  qjj = A->size[0];
  unnamed_idx_1 = A->size[0];
  i7 = U->size[0] * U->size[1];
  U->size[0] = qjj;
  U->size[1] = unnamed_idx_1;
  emxEnsureCapacity_real_T(U, i7);
  qjj *= unnamed_idx_1;
  for (i7 = 0; i7 < qjj; i7++) {
    U->data[i7] = 0.0;
  }

  if (A->size[0] != 0) {
    if (A->size[0] > 1) {
      nct = A->size[0] - 1;
    } else {
      nct = 0;
    }

    if (!(nct < 1)) {
      nct = 1;
    }

    i7 = (nct > 0);
    qjj = 1;
    while (qjj <= i7) {
      if (1 <= nct) {
        rt = xnrm2(n, b_A, 1);
        if (rt > 0.0) {
          if (b_A->data[0] < 0.0) {
            rt = -rt;
          }

          if (fabs(rt) >= 1.0020841800044864E-292) {
            c_A = 1.0 / rt;
            for (qjj = 0; qjj + 1 <= n; qjj++) {
              b_A->data[qjj] *= c_A;
            }
          } else {
            for (qjj = 0; qjj + 1 <= n; qjj++) {
              b_A->data[qjj] /= rt;
            }
          }

          b_A->data[0]++;
          s_data_idx_0 = -rt;
        } else {
          s_data_idx_0 = 0.0;
        }

        for (qjj = 0; qjj + 1 <= n; qjj++) {
          U->data[qjj] = b_A->data[qjj];
        }
      }

      qjj = 2;
    }

    if (nct < 1) {
      s_data_idx_0 = b_A->data[0];
    }

    if (nct + 1 <= A->size[0]) {
      for (unnamed_idx_1 = nct; unnamed_idx_1 + 1 <= n; unnamed_idx_1++) {
        for (qjj = 1; qjj <= n; qjj++) {
          U->data[(qjj + U->size[0] * unnamed_idx_1) - 1] = 0.0;
        }

        U->data[unnamed_idx_1 + U->size[0] * unnamed_idx_1] = 1.0;
      }
    }

    while (nct > 0) {
      if (s_data_idx_0 != 0.0) {
        for (unnamed_idx_1 = 2; unnamed_idx_1 <= n; unnamed_idx_1++) {
          qjj = 1 + n * (unnamed_idx_1 - 1);
          rt = -(xdotc(n, U, 1, U, qjj) / U->data[0]);
          xaxpy(n, rt, 1, U, qjj);
        }

        for (qjj = 0; qjj + 1 <= n; qjj++) {
          U->data[qjj] = -U->data[qjj];
        }

        U->data[0]++;
      } else {
        for (qjj = 1; qjj <= n; qjj++) {
          U->data[qjj - 1] = 0.0;
        }

        U->data[0] = 1.0;
      }

      nct = 0;
    }

    if (s_data_idx_0 != 0.0) {
      rt = fabs(s_data_idx_0);
      c_A = s_data_idx_0;
      s_data_idx_0 = rt;
      if (1 <= n) {
        xscal(n, c_A / rt, U, 1);
      }
    }
  }

  emxFree_real_T(&b_A);
  s_size[0] = minnp;
  qjj = 1;
  while (qjj <= minnp) {
    s_data[0] = s_data_idx_0;
    qjj = 2;
  }

  *V = 1.0;
}

//
// Arguments    : const emxArray_real_T *A
//                emxArray_real_T *U
// Return Type  : void
//
void c_svd(const emxArray_real_T *A, emxArray_real_T *U)
{
  emxArray_real_T *b_A;
  int m;
  int ns;
  int n;
  int p;
  int qs;
  int minnp;
  emxArray_real_T *s;
  emxArray_real_T *e;
  emxArray_real_T *work;
  int nrt;
  int nct;
  int q;
  int iter;
  int mm;
  boolean_T apply_transform;
  double ztest0;
  double ztest;
  double snorm;
  boolean_T exitg1;
  double f;
  double varargin_1[5];
  double mtmp;
  double sqds;
  double b;
  emxInit_real_T1(&b_A, 2);
  m = b_A->size[0] * b_A->size[1];
  b_A->size[0] = A->size[0];
  b_A->size[1] = A->size[1];
  emxEnsureCapacity_real_T(b_A, m);
  ns = A->size[0] * A->size[1];
  for (m = 0; m < ns; m++) {
    b_A->data[m] = A->data[m];
  }

  n = A->size[0];
  p = A->size[1];
  qs = A->size[0] + 1;
  ns = A->size[1];
  if (qs < ns) {
    ns = qs;
  }

  qs = A->size[0];
  minnp = A->size[1];
  if (qs < minnp) {
    minnp = qs;
  }

  emxInit_real_T(&s, 1);
  m = s->size[0];
  s->size[0] = ns;
  emxEnsureCapacity_real_T1(s, m);
  for (m = 0; m < ns; m++) {
    s->data[m] = 0.0;
  }

  emxInit_real_T(&e, 1);
  ns = A->size[1];
  m = e->size[0];
  e->size[0] = ns;
  emxEnsureCapacity_real_T1(e, m);
  for (m = 0; m < ns; m++) {
    e->data[m] = 0.0;
  }

  emxInit_real_T(&work, 1);
  ns = A->size[0];
  m = work->size[0];
  work->size[0] = ns;
  emxEnsureCapacity_real_T1(work, m);
  for (m = 0; m < ns; m++) {
    work->data[m] = 0.0;
  }

  if (!((A->size[0] == 0) || (A->size[1] == 0))) {
    if (A->size[1] > 2) {
      ns = A->size[1] - 2;
    } else {
      ns = 0;
    }

    nrt = A->size[0];
    if (ns < nrt) {
      nrt = ns;
    }

    if (A->size[0] > 1) {
      ns = A->size[0] - 1;
    } else {
      ns = 0;
    }

    nct = A->size[1];
    if (ns < nct) {
      nct = ns;
    }

    if (nct > nrt) {
      m = nct;
    } else {
      m = nrt;
    }

    for (q = 0; q + 1 <= m; q++) {
      iter = q + n * q;
      mm = n - q;
      apply_transform = false;
      if (q + 1 <= nct) {
        ztest0 = b_xnrm2(mm, b_A, iter + 1);
        if (ztest0 > 0.0) {
          apply_transform = true;
          if (b_A->data[iter] < 0.0) {
            ztest0 = -ztest0;
          }

          s->data[q] = ztest0;
          if (fabs(s->data[q]) >= 1.0020841800044864E-292) {
            xscal(mm, 1.0 / s->data[q], b_A, iter + 1);
          } else {
            ns = iter + mm;
            for (qs = iter; qs + 1 <= ns; qs++) {
              b_A->data[qs] /= s->data[q];
            }
          }

          b_A->data[iter]++;
          s->data[q] = -s->data[q];
        } else {
          s->data[q] = 0.0;
        }
      }

      for (ns = q + 1; ns + 1 <= p; ns++) {
        qs = q + n * ns;
        if (apply_transform) {
          ztest0 = -(xdotc(mm, b_A, iter + 1, b_A, qs + 1) / b_A->data[q +
                     b_A->size[0] * q]);
          xaxpy(mm, ztest0, iter + 1, b_A, qs + 1);
        }

        e->data[ns] = b_A->data[qs];
      }

      if (q + 1 <= nrt) {
        ns = p - q;
        ztest0 = xnrm2(ns - 1, e, q + 2);
        if (ztest0 == 0.0) {
          e->data[q] = 0.0;
        } else {
          if (e->data[q + 1] < 0.0) {
            ztest0 = -ztest0;
          }

          e->data[q] = ztest0;
          ztest0 = e->data[q];
          if (fabs(e->data[q]) >= 1.0020841800044864E-292) {
            ztest0 = 1.0 / e->data[q];
            ns += q;
            for (qs = q + 1; qs + 1 <= ns; qs++) {
              e->data[qs] *= ztest0;
            }
          } else {
            ns += q;
            for (qs = q + 1; qs + 1 <= ns; qs++) {
              e->data[qs] /= ztest0;
            }
          }

          e->data[q + 1]++;
          e->data[q] = -e->data[q];
          if (q + 2 <= n) {
            for (ns = q + 1; ns + 1 <= n; ns++) {
              work->data[ns] = 0.0;
            }

            for (ns = q + 1; ns + 1 <= p; ns++) {
              b_xaxpy(mm - 1, e->data[ns], b_A, (q + n * ns) + 2, work, q + 2);
            }

            for (ns = q + 1; ns + 1 <= p; ns++) {
              c_xaxpy(mm - 1, -e->data[ns] / e->data[q + 1], work, q + 2, b_A,
                      (q + n * ns) + 2);
            }
          }
        }
      }
    }

    qs = A->size[1];
    m = A->size[0] + 1;
    if (qs < m) {
      m = qs;
    }

    if (nct < A->size[1]) {
      s->data[nct] = b_A->data[nct + b_A->size[0] * nct];
    }

    if (A->size[0] < m) {
      s->data[m - 1] = 0.0;
    }

    if (nrt + 1 < m) {
      e->data[nrt] = b_A->data[nrt + b_A->size[0] * (m - 1)];
    }

    e->data[m - 1] = 0.0;
    for (q = 0; q + 1 <= m; q++) {
      if (s->data[q] != 0.0) {
        ztest = fabs(s->data[q]);
        ztest0 = s->data[q] / ztest;
        s->data[q] = ztest;
        if (q + 1 < m) {
          e->data[q] /= ztest0;
        }
      }

      if ((q + 1 < m) && (e->data[q] != 0.0)) {
        ztest = fabs(e->data[q]);
        ztest0 = e->data[q];
        e->data[q] = ztest;
        s->data[q + 1] *= ztest / ztest0;
      }
    }

    mm = m;
    iter = 0;
    snorm = 0.0;
    for (ns = 0; ns + 1 <= m; ns++) {
      ztest0 = fabs(s->data[ns]);
      ztest = fabs(e->data[ns]);
      if ((ztest0 > ztest) || rtIsNaN(ztest)) {
      } else {
        ztest0 = ztest;
      }

      if (!((snorm > ztest0) || rtIsNaN(ztest0))) {
        snorm = ztest0;
      }
    }

    while ((m > 0) && (!(iter >= 75))) {
      q = m - 1;
      exitg1 = false;
      while (!(exitg1 || (q == 0))) {
        ztest0 = fabs(e->data[q - 1]);
        if ((ztest0 <= 2.2204460492503131E-16 * (fabs(s->data[q - 1]) + fabs
              (s->data[q]))) || (ztest0 <= 1.0020841800044864E-292) || ((iter >
              20) && (ztest0 <= 2.2204460492503131E-16 * snorm))) {
          e->data[q - 1] = 0.0;
          exitg1 = true;
        } else {
          q--;
        }
      }

      if (q == m - 1) {
        ns = 4;
      } else {
        qs = m;
        ns = m;
        exitg1 = false;
        while ((!exitg1) && (ns >= q)) {
          qs = ns;
          if (ns == q) {
            exitg1 = true;
          } else {
            ztest0 = 0.0;
            if (ns < m) {
              ztest0 = fabs(e->data[ns - 1]);
            }

            if (ns > q + 1) {
              ztest0 += fabs(e->data[ns - 2]);
            }

            ztest = fabs(s->data[ns - 1]);
            if ((ztest <= 2.2204460492503131E-16 * ztest0) || (ztest <=
                 1.0020841800044864E-292)) {
              s->data[ns - 1] = 0.0;
              exitg1 = true;
            } else {
              ns--;
            }
          }
        }

        if (qs == q) {
          ns = 3;
        } else if (qs == m) {
          ns = 1;
        } else {
          ns = 2;
          q = qs;
        }
      }

      switch (ns) {
       case 1:
        f = e->data[m - 2];
        e->data[m - 2] = 0.0;
        for (qs = m - 3; qs + 2 >= q + 1; qs--) {
          xrotg(&s->data[qs + 1], &f, &ztest0, &ztest);
          if (qs + 2 > q + 1) {
            f = -ztest * e->data[qs];
            e->data[qs] *= ztest0;
          }
        }
        break;

       case 2:
        f = e->data[q - 1];
        e->data[q - 1] = 0.0;
        while (q + 1 <= m) {
          xrotg(&s->data[q], &f, &ztest0, &ztest);
          f = -ztest * e->data[q];
          e->data[q] *= ztest0;
          q++;
        }
        break;

       case 3:
        varargin_1[0] = fabs(s->data[m - 1]);
        varargin_1[1] = fabs(s->data[m - 2]);
        varargin_1[2] = fabs(e->data[m - 2]);
        varargin_1[3] = fabs(s->data[q]);
        varargin_1[4] = fabs(e->data[q]);
        ns = 1;
        mtmp = varargin_1[0];
        if (rtIsNaN(varargin_1[0])) {
          qs = 2;
          exitg1 = false;
          while ((!exitg1) && (qs < 6)) {
            ns = qs;
            if (!rtIsNaN(varargin_1[qs - 1])) {
              mtmp = varargin_1[qs - 1];
              exitg1 = true;
            } else {
              qs++;
            }
          }
        }

        if (ns < 5) {
          while (ns + 1 < 6) {
            if (varargin_1[ns] > mtmp) {
              mtmp = varargin_1[ns];
            }

            ns++;
          }
        }

        f = s->data[m - 1] / mtmp;
        ztest0 = s->data[m - 2] / mtmp;
        ztest = e->data[m - 2] / mtmp;
        sqds = s->data[q] / mtmp;
        b = ((ztest0 + f) * (ztest0 - f) + ztest * ztest) / 2.0;
        ztest0 = f * ztest;
        ztest0 *= ztest0;
        if ((b != 0.0) || (ztest0 != 0.0)) {
          ztest = b * b + ztest0;
          b_sqrt(&ztest);
          if (b < 0.0) {
            ztest = -ztest;
          }

          ztest = ztest0 / (b + ztest);
        } else {
          ztest = 0.0;
        }

        f = (sqds + f) * (sqds - f) + ztest;
        b = sqds * (e->data[q] / mtmp);
        for (qs = q + 1; qs < m; qs++) {
          xrotg(&f, &b, &ztest0, &ztest);
          if (qs > q + 1) {
            e->data[qs - 2] = f;
          }

          f = ztest0 * s->data[qs - 1] + ztest * e->data[qs - 1];
          e->data[qs - 1] = ztest0 * e->data[qs - 1] - ztest * s->data[qs - 1];
          b = ztest * s->data[qs];
          s->data[qs] *= ztest0;
          s->data[qs - 1] = f;
          xrotg(&s->data[qs - 1], &b, &ztest0, &ztest);
          f = ztest0 * e->data[qs - 1] + ztest * s->data[qs];
          s->data[qs] = -ztest * e->data[qs - 1] + ztest0 * s->data[qs];
          b = ztest * e->data[qs];
          e->data[qs] *= ztest0;
        }

        e->data[m - 2] = f;
        iter++;
        break;

       default:
        if (s->data[q] < 0.0) {
          s->data[q] = -s->data[q];
        }

        ns = q + 1;
        while ((q + 1 < mm) && (s->data[q] < s->data[ns])) {
          ztest = s->data[q];
          s->data[q] = s->data[ns];
          s->data[ns] = ztest;
          q = ns;
          ns++;
        }

        iter = 0;
        m--;
        break;
      }
    }
  }

  emxFree_real_T(&work);
  emxFree_real_T(&e);
  emxFree_real_T(&b_A);
  m = U->size[0];
  U->size[0] = minnp;
  emxEnsureCapacity_real_T1(U, m);
  for (qs = 0; qs + 1 <= minnp; qs++) {
    U->data[qs] = s->data[qs];
  }

  emxFree_real_T(&s);
}

//
// File trailer for svd1.cpp
//
// [EOF]
//
