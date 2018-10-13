//
// File: heading_sample.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "heading_sample_emxutil.h"
#include "parabola_fitting.h"
#include "xscal.h"
#include "rand.h"
#include "heading_sample_verify.h"
#include "inv.h"
#include "svd1.h"
#include "svd.h"
#include "norm.h"
#include "diag.h"
#include "rdivide.h"
#include "sum.h"

// Function Definitions

//
// dphi_max: max rotation velocity
//  v_max: max linear velocity
//  t_allow: time budget
//  dphi_lim_min, dphi_lim_max: limit for dphi, make its less agreesive in
//  rotating. If not important, just set to -pi and pi
//  phi_0, phi_n: start and end configurations
//  n: number of links
//  m: number of samples
//  d: vector of distances of each links
//  max_iter: max iteration allowed to search for valid sample points
// Arguments    : double dphi_max
//                double v_max
//                double t_allow
//                double dphi_lim_min
//                double dphi_lim_max
//                double phi_0
//                double phi_n
//                double n
//                double m
//                const double d_data[]
//                const int d_size[1]
//                double max_iter
//                double *res
//                emxArray_real_T *dphi
// Return Type  : void
//
void heading_sample(double dphi_max, double v_max, double t_allow, double
                    dphi_lim_min, double dphi_lim_max, double phi_0, double
                    phi_n, double n, double m, const double d_data[], const int
                    [1], double max_iter, double *res, emxArray_real_T *dphi)
{
  int b_res;
  int i0;
  int ia;
  emxArray_real_T *a;
  emxArray_real_T *b;
  int br;
  emxArray_real_T *Q;
  emxArray_real_T *c;
  emxArray_real_T *S;
  emxArray_real_T *q_;
  emxArray_real_T *Q_;
  emxArray_real_T *M_bar;
  emxArray_real_T *w_;
  emxArray_real_T *W__;
  emxArray_real_T *wE_T;
  emxArray_real_T *dphi_unit_circle;
  emxArray_real_T *rand_angle;
  emxArray_real_T *b_b;
  cell_wrap_0 reshapes[2];
  emxArray_real_T *C;
  emxArray_real_T *b_q_;
  int exitg1;
  double b_sum;
  double cos_cummulative;
  double b_c;
  double gamma0;
  unsigned int ind;
  int loop;
  boolean_T exitg2;
  double b_gamma;
  static const signed char iv0[5] = { 0, -1, 1, 2, 2 };

  int nmj;
  int k;
  int i1;
  unsigned int unnamed_idx_0;
  unsigned int unnamed_idx_1;
  int b_m;
  int i2;
  int cr;
  int iy;
  int ar;
  int ix;
  int i3;
  int b_n;
  boolean_T empty_non_axis_sizes;
  int info;
  boolean_T exitg3;
  int exponent;

  //  Returns:
  //  res: 1: OK; ~= 1: Failed;
  //  Check the code below
  //  Sample m sets for n nodes
  b_res = 0;
  i0 = dphi->size[0] * dphi->size[1];
  dphi->size[0] = (int)m;
  dphi->size[1] = (int)n;
  emxEnsureCapacity_real_T(dphi, i0);
  ia = (int)m * (int)n;
  for (i0 = 0; i0 < ia; i0++) {
    dphi->data[i0] = 0.0;
  }

  emxInit_real_T(&a, 1);

  //  mark a hyperplane that sampled dphi
  //  Build the ellipsoid E first: sum(a_i * dphi_i^2) < t_total - sum(b_i)
  //  For each vertex, check the condition and approximate as a parabola
  i0 = a->size[0];
  a->size[0] = (int)n;
  emxEnsureCapacity_real_T1(a, i0);
  ia = (int)n;
  for (i0 = 0; i0 < ia; i0++) {
    a->data[i0] = 0.0;
  }

  emxInit_real_T(&b, 1);
  i0 = b->size[0];
  b->size[0] = (int)n;
  emxEnsureCapacity_real_T1(b, i0);
  ia = (int)n;
  for (i0 = 0; i0 < ia; i0++) {
    b->data[i0] = 0.0;
  }

  br = 0;
  emxInit_real_T1(&Q, 2);
  emxInit_real_T(&c, 1);
  emxInit_real_T1(&S, 2);
  emxInit_real_T(&q_, 1);
  emxInit_real_T1(&Q_, 2);
  emxInit_real_T1(&M_bar, 2);
  emxInit_real_T(&w_, 1);
  emxInit_real_T1(&W__, 2);
  emxInit_real_T1(&wE_T, 2);
  emxInit_real_T1(&dphi_unit_circle, 2);
  emxInit_real_T1(&rand_angle, 2);
  emxInit_real_T1(&b_b, 2);
  emxInitMatrix_cell_wrap_0(reshapes);
  emxInit_real_T1(&C, 2);
  emxInit_real_T(&b_q_, 1);
  do {
    exitg1 = 0;
    if (br <= (int)n - 1) {
      parabola_fitting(v_max, dphi_max, d_data[br], &b_sum, &cos_cummulative,
                       &b_c);
      if (b_c == 1.0) {
        a->data[br] = b_sum;
        b->data[br] = cos_cummulative;
        br++;
      } else {
        exitg1 = 1;
      }
    } else {
      b_sum = sum(b);

      //  Here we should always have t_res > 0 since in the first step we already  
      //  constraint the path within the allowed time.
      //  However, due to the parabola approximation, we might have the t_res
      //  smaller than 0. Can apply a trick here, set t_res = epsilon
      b_sum = t_allow - b_sum;
      if (b_sum <= 0.0) {
        //  another check for sure
        //      res = RES_TIM_NOT_ALLOW;
        b_sum = 0.1;

        //      return;
      }

      rdivide(b_sum, a, b);
      diag(b, Q);

      //  Build the hyperplane H: c^T x = gamma
      gamma0 = phi_n - phi_0;

      //  The smallest plane
      if (gamma0 > 3.1415926535897931) {
        gamma0 -= 6.2831853071795862;
      } else {
        if (gamma0 < -3.1415926535897931) {
          gamma0 += 6.2831853071795862;
        }
      }

      //  Which plane we should use, it may have several planes with different of 2*pi 
      ind = 0U;

      //  k = [0];
      loop = 0;
      exitg2 = false;
      while ((!exitg2) && (loop < 5)) {
        i0 = a->size[0];
        a->size[0] = (int)n;
        emxEnsureCapacity_real_T1(a, i0);
        ia = (int)n;
        for (i0 = 0; i0 < ia; i0++) {
          a->data[i0] = 1.0;
        }

        b_sum = norm(a);
        i0 = c->size[0];
        c->size[0] = (int)n;
        emxEnsureCapacity_real_T1(c, i0);
        ia = (int)n;
        for (i0 = 0; i0 < ia; i0++) {
          c->data[i0] = 1.0 / b_sum;
        }

        b_gamma = (gamma0 + (double)iv0[loop] * 2.0 * 3.1415926535897931) /
          b_sum;

        //     %% Transform H to i-coordinate ([1,0...0]) v_i = S * v_h
        svd(c, W__, b, &b_sum);
        i0 = a->size[0];
        a->size[0] = 1 + (int)(n - 1.0);
        emxEnsureCapacity_real_T1(a, i0);
        a->data[0] = 1.0;
        ia = (int)(n - 1.0);
        for (i0 = 0; i0 < ia; i0++) {
          a->data[i0 + 1] = 0.0;
        }

        svd(a, M_bar, b, &cos_cummulative);
        i0 = M_bar->size[0] * M_bar->size[1];
        emxEnsureCapacity_real_T(M_bar, i0);
        br = M_bar->size[0];
        nmj = M_bar->size[1];
        ia = br * nmj;
        for (i0 = 0; i0 < ia; i0++) {
          M_bar->data[i0] = M_bar->data[i0] * cos_cummulative * b_sum;
        }

        i0 = b_b->size[0] * b_b->size[1];
        b_b->size[0] = W__->size[1];
        b_b->size[1] = W__->size[0];
        emxEnsureCapacity_real_T(b_b, i0);
        ia = W__->size[0];
        for (i0 = 0; i0 < ia; i0++) {
          br = W__->size[1];
          for (i1 = 0; i1 < br; i1++) {
            b_b->data[i1 + b_b->size[0] * i0] = W__->data[i0 + W__->size[0] * i1];
          }
        }

        if ((M_bar->size[1] == 1) || (b_b->size[0] == 1)) {
          i0 = S->size[0] * S->size[1];
          S->size[0] = M_bar->size[0];
          S->size[1] = b_b->size[1];
          emxEnsureCapacity_real_T(S, i0);
          ia = M_bar->size[0];
          for (i0 = 0; i0 < ia; i0++) {
            br = b_b->size[1];
            for (i1 = 0; i1 < br; i1++) {
              S->data[i0 + S->size[0] * i1] = 0.0;
              nmj = M_bar->size[1];
              for (i2 = 0; i2 < nmj; i2++) {
                S->data[i0 + S->size[0] * i1] += M_bar->data[i0 + M_bar->size[0]
                  * i2] * b_b->data[i2 + b_b->size[0] * i1];
              }
            }
          }
        } else {
          k = M_bar->size[1];
          unnamed_idx_0 = (unsigned int)M_bar->size[0];
          unnamed_idx_1 = (unsigned int)b_b->size[1];
          i0 = S->size[0] * S->size[1];
          S->size[0] = (int)unnamed_idx_0;
          S->size[1] = (int)unnamed_idx_1;
          emxEnsureCapacity_real_T(S, i0);
          b_m = M_bar->size[0];
          i0 = S->size[0] * S->size[1];
          emxEnsureCapacity_real_T(S, i0);
          ia = S->size[1];
          for (i0 = 0; i0 < ia; i0++) {
            br = S->size[0];
            for (i1 = 0; i1 < br; i1++) {
              S->data[i1 + S->size[0] * i0] = 0.0;
            }
          }

          if ((M_bar->size[0] == 0) || (b_b->size[1] == 0)) {
          } else {
            nmj = M_bar->size[0] * (b_b->size[1] - 1);
            cr = 0;
            while ((b_m > 0) && (cr <= nmj)) {
              i0 = cr + b_m;
              for (iy = cr; iy + 1 <= i0; iy++) {
                S->data[iy] = 0.0;
              }

              cr += b_m;
            }

            br = 0;
            cr = 0;
            while ((b_m > 0) && (cr <= nmj)) {
              ar = -1;
              i0 = br + k;
              for (ix = br; ix + 1 <= i0; ix++) {
                if (b_b->data[ix] != 0.0) {
                  ia = ar;
                  i1 = cr + b_m;
                  for (iy = cr; iy + 1 <= i1; iy++) {
                    ia++;
                    S->data[iy] += b_b->data[ix] * M_bar->data[ia];
                  }
                }

                ar += b_m;
              }

              br += k;
              cr += b_m;
            }
          }
        }

        //  transform E to i-coordinate
        if ((S->size[1] == 1) || ((int)n == 1)) {
          i0 = q_->size[0];
          q_->size[0] = S->size[0];
          emxEnsureCapacity_real_T1(q_, i0);
          ia = S->size[0];
          for (i0 = 0; i0 < ia; i0++) {
            q_->data[i0] = 0.0;
            br = S->size[1];
            for (i1 = 0; i1 < br; i1++) {
              q_->data[i0] += S->data[i0 + S->size[0] * i1] * 0.0;
            }
          }
        } else {
          unnamed_idx_0 = (unsigned int)S->size[0];
          i0 = q_->size[0];
          q_->size[0] = (int)unnamed_idx_0;
          emxEnsureCapacity_real_T1(q_, i0);
          b_m = S->size[0];
          br = q_->size[0];
          i0 = q_->size[0];
          q_->size[0] = br;
          emxEnsureCapacity_real_T1(q_, i0);
          for (i0 = 0; i0 < br; i0++) {
            q_->data[i0] = 0.0;
          }

          if (S->size[0] != 0) {
            cr = 0;
            while ((b_m > 0) && (cr <= 0)) {
              for (iy = 1; iy <= b_m; iy++) {
                q_->data[iy - 1] = 0.0;
              }

              cr = b_m;
            }
          }
        }

        i0 = W__->size[0] * W__->size[1];
        W__->size[0] = S->size[0];
        W__->size[1] = S->size[1];
        emxEnsureCapacity_real_T(W__, i0);
        ia = S->size[0] * S->size[1];
        for (i0 = 0; i0 < ia; i0++) {
          W__->data[i0] = b_gamma * S->data[i0];
        }

        if ((W__->size[1] == 1) || (c->size[0] == 1)) {
          i0 = b->size[0];
          b->size[0] = W__->size[0];
          emxEnsureCapacity_real_T1(b, i0);
          ia = W__->size[0];
          for (i0 = 0; i0 < ia; i0++) {
            b->data[i0] = 0.0;
            br = W__->size[1];
            for (i1 = 0; i1 < br; i1++) {
              b->data[i0] += W__->data[i0 + W__->size[0] * i1] * c->data[i1];
            }
          }
        } else {
          k = W__->size[1];
          unnamed_idx_0 = (unsigned int)W__->size[0];
          i0 = b->size[0];
          b->size[0] = (int)unnamed_idx_0;
          emxEnsureCapacity_real_T1(b, i0);
          b_m = W__->size[0];
          nmj = b->size[0];
          i0 = b->size[0];
          b->size[0] = nmj;
          emxEnsureCapacity_real_T1(b, i0);
          for (i0 = 0; i0 < nmj; i0++) {
            b->data[i0] = 0.0;
          }

          if (W__->size[0] != 0) {
            cr = 0;
            while ((b_m > 0) && (cr <= 0)) {
              for (iy = 1; iy <= b_m; iy++) {
                b->data[iy - 1] = 0.0;
              }

              cr = b_m;
            }

            br = 0;
            cr = 0;
            while ((b_m > 0) && (cr <= 0)) {
              ar = -1;
              i0 = br + k;
              for (ix = br; ix + 1 <= i0; ix++) {
                if (c->data[ix] != 0.0) {
                  ia = ar;
                  for (iy = 0; iy + 1 <= b_m; iy++) {
                    ia++;
                    b->data[iy] += c->data[ix] * W__->data[ia];
                  }
                }

                ar += b_m;
              }

              br += k;
              cr = b_m;
            }
          }
        }

        i0 = q_->size[0];
        emxEnsureCapacity_real_T1(q_, i0);
        ia = q_->size[0];
        for (i0 = 0; i0 < ia; i0++) {
          q_->data[i0] -= b->data[i0];
        }

        if ((S->size[1] == 1) || (Q->size[0] == 1)) {
          i0 = W__->size[0] * W__->size[1];
          W__->size[0] = S->size[0];
          W__->size[1] = Q->size[1];
          emxEnsureCapacity_real_T(W__, i0);
          ia = S->size[0];
          for (i0 = 0; i0 < ia; i0++) {
            br = Q->size[1];
            for (i1 = 0; i1 < br; i1++) {
              W__->data[i0 + W__->size[0] * i1] = 0.0;
              nmj = S->size[1];
              for (i2 = 0; i2 < nmj; i2++) {
                W__->data[i0 + W__->size[0] * i1] += S->data[i0 + S->size[0] *
                  i2] * Q->data[i2 + Q->size[0] * i1];
              }
            }
          }
        } else {
          k = S->size[1];
          unnamed_idx_0 = (unsigned int)S->size[0];
          unnamed_idx_1 = (unsigned int)Q->size[1];
          i0 = W__->size[0] * W__->size[1];
          W__->size[0] = (int)unnamed_idx_0;
          W__->size[1] = (int)unnamed_idx_1;
          emxEnsureCapacity_real_T(W__, i0);
          b_m = S->size[0];
          i0 = W__->size[0] * W__->size[1];
          emxEnsureCapacity_real_T(W__, i0);
          ia = W__->size[1];
          for (i0 = 0; i0 < ia; i0++) {
            br = W__->size[0];
            for (i1 = 0; i1 < br; i1++) {
              W__->data[i1 + W__->size[0] * i0] = 0.0;
            }
          }

          if ((S->size[0] == 0) || (Q->size[1] == 0)) {
          } else {
            nmj = S->size[0] * (Q->size[1] - 1);
            cr = 0;
            while ((b_m > 0) && (cr <= nmj)) {
              i0 = cr + b_m;
              for (iy = cr; iy + 1 <= i0; iy++) {
                W__->data[iy] = 0.0;
              }

              cr += b_m;
            }

            br = 0;
            cr = 0;
            while ((b_m > 0) && (cr <= nmj)) {
              ar = -1;
              i0 = br + k;
              for (ix = br; ix + 1 <= i0; ix++) {
                if (Q->data[ix] != 0.0) {
                  ia = ar;
                  i1 = cr + b_m;
                  for (iy = cr; iy + 1 <= i1; iy++) {
                    ia++;
                    W__->data[iy] += Q->data[ix] * S->data[ia];
                  }
                }

                ar += b_m;
              }

              br += k;
              cr += b_m;
            }
          }
        }

        i0 = b_b->size[0] * b_b->size[1];
        b_b->size[0] = S->size[1];
        b_b->size[1] = S->size[0];
        emxEnsureCapacity_real_T(b_b, i0);
        ia = S->size[0];
        for (i0 = 0; i0 < ia; i0++) {
          br = S->size[1];
          for (i1 = 0; i1 < br; i1++) {
            b_b->data[i1 + b_b->size[0] * i0] = S->data[i0 + S->size[0] * i1];
          }
        }

        if ((W__->size[1] == 1) || (b_b->size[0] == 1)) {
          i0 = Q_->size[0] * Q_->size[1];
          Q_->size[0] = W__->size[0];
          Q_->size[1] = b_b->size[1];
          emxEnsureCapacity_real_T(Q_, i0);
          ia = W__->size[0];
          for (i0 = 0; i0 < ia; i0++) {
            br = b_b->size[1];
            for (i1 = 0; i1 < br; i1++) {
              Q_->data[i0 + Q_->size[0] * i1] = 0.0;
              nmj = W__->size[1];
              for (i2 = 0; i2 < nmj; i2++) {
                Q_->data[i0 + Q_->size[0] * i1] += W__->data[i0 + W__->size[0] *
                  i2] * b_b->data[i2 + b_b->size[0] * i1];
              }
            }
          }
        } else {
          k = W__->size[1];
          unnamed_idx_0 = (unsigned int)W__->size[0];
          unnamed_idx_1 = (unsigned int)b_b->size[1];
          i0 = Q_->size[0] * Q_->size[1];
          Q_->size[0] = (int)unnamed_idx_0;
          Q_->size[1] = (int)unnamed_idx_1;
          emxEnsureCapacity_real_T(Q_, i0);
          b_m = W__->size[0];
          i0 = Q_->size[0] * Q_->size[1];
          emxEnsureCapacity_real_T(Q_, i0);
          ia = Q_->size[1];
          for (i0 = 0; i0 < ia; i0++) {
            br = Q_->size[0];
            for (i1 = 0; i1 < br; i1++) {
              Q_->data[i1 + Q_->size[0] * i0] = 0.0;
            }
          }

          if ((W__->size[0] == 0) || (b_b->size[1] == 0)) {
          } else {
            nmj = W__->size[0] * (b_b->size[1] - 1);
            cr = 0;
            while ((b_m > 0) && (cr <= nmj)) {
              i0 = cr + b_m;
              for (iy = cr; iy + 1 <= i0; iy++) {
                Q_->data[iy] = 0.0;
              }

              cr += b_m;
            }

            br = 0;
            cr = 0;
            while ((b_m > 0) && (cr <= nmj)) {
              ar = -1;
              i0 = br + k;
              for (ix = br; ix + 1 <= i0; ix++) {
                if (b_b->data[ix] != 0.0) {
                  ia = ar;
                  i1 = cr + b_m;
                  for (iy = cr; iy + 1 <= i1; iy++) {
                    ia++;
                    Q_->data[iy] += b_b->data[ix] * W__->data[ia];
                  }
                }

                ar += b_m;
              }

              br += k;
              cr += b_m;
            }
          }
        }

        inv(Q_);
        if (2.0 > n) {
          i0 = 0;
          i1 = 0;
          i2 = 0;
          i3 = 0;
        } else {
          i0 = 1;
          i1 = (int)n;
          i2 = 1;
          i3 = (int)n;
        }

        ix = M_bar->size[0] * M_bar->size[1];
        M_bar->size[0] = i1 - i0;
        M_bar->size[1] = i3 - i2;
        emxEnsureCapacity_real_T(M_bar, ix);
        ia = i3 - i2;
        for (i3 = 0; i3 < ia; i3++) {
          br = i1 - i0;
          for (ix = 0; ix < br; ix++) {
            M_bar->data[ix + M_bar->size[0] * i3] = Q_->data[(i0 + ix) +
              Q_->size[0] * (i2 + i3)];
          }
        }

        if (2.0 > n) {
          i0 = 0;
          i1 = 1;
        } else {
          i0 = 1;
          i1 = (int)n + 1;
        }

        i2 = b->size[0];
        b->size[0] = (i1 - i0) - 1;
        emxEnsureCapacity_real_T1(b, i2);
        ia = i1 - i0;
        for (i2 = 0; i2 <= ia - 2; i2++) {
          b->data[i2] = Q_->data[i0 + i2];
        }

        //  this is E in i-coordinate
        i2 = W__->size[0] * W__->size[1];
        W__->size[0] = M_bar->size[0];
        W__->size[1] = M_bar->size[1];
        emxEnsureCapacity_real_T(W__, i2);
        ia = M_bar->size[0] * M_bar->size[1];
        for (i2 = 0; i2 < ia; i2++) {
          W__->data[i2] = M_bar->data[i2];
        }

        inv(W__);
        if ((W__->size[1] == 1) || ((i1 - i0) - 1 == 1)) {
          i2 = a->size[0];
          a->size[0] = W__->size[0];
          emxEnsureCapacity_real_T1(a, i2);
          ia = W__->size[0];
          for (i2 = 0; i2 < ia; i2++) {
            a->data[i2] = 0.0;
            br = W__->size[1];
            for (i3 = 0; i3 < br; i3++) {
              a->data[i2] += W__->data[i2 + W__->size[0] * i3] * b->data[i3];
            }
          }
        } else {
          k = W__->size[1];
          unnamed_idx_0 = (unsigned int)W__->size[0];
          i2 = a->size[0];
          a->size[0] = (int)unnamed_idx_0;
          emxEnsureCapacity_real_T1(a, i2);
          b_m = W__->size[0];
          nmj = a->size[0];
          i2 = a->size[0];
          a->size[0] = nmj;
          emxEnsureCapacity_real_T1(a, i2);
          for (i2 = 0; i2 < nmj; i2++) {
            a->data[i2] = 0.0;
          }

          if (W__->size[0] != 0) {
            cr = 0;
            while ((b_m > 0) && (cr <= 0)) {
              for (iy = 1; iy <= b_m; iy++) {
                a->data[iy - 1] = 0.0;
              }

              cr = b_m;
            }

            br = 0;
            cr = 0;
            while ((b_m > 0) && (cr <= 0)) {
              ar = -1;
              i2 = br + k;
              for (ix = br; ix + 1 <= i2; ix++) {
                if (Q_->data[i0 + ix] != 0.0) {
                  ia = ar;
                  for (iy = 0; iy + 1 <= b_m; iy++) {
                    ia++;
                    a->data[iy] += Q_->data[i0 + ix] * W__->data[ia];
                  }
                }

                ar += b_m;
              }

              br += k;
              cr = b_m;
            }
          }
        }

        b_sum = q_->data[0];
        i2 = b_q_->size[0];
        b_q_->size[0] = 1 + a->size[0];
        emxEnsureCapacity_real_T1(b_q_, i2);
        b_q_->data[0] = -b_sum;
        ia = a->size[0];
        for (i2 = 0; i2 < ia; i2++) {
          b_q_->data[i2 + 1] = b_sum * a->data[i2];
        }

        i2 = w_->size[0];
        w_->size[0] = q_->size[0];
        emxEnsureCapacity_real_T1(w_, i2);
        ia = q_->size[0];
        for (i2 = 0; i2 < ia; i2++) {
          w_->data[i2] = q_->data[i2] + b_q_->data[i2];
        }

        i2 = dphi_unit_circle->size[0] * dphi_unit_circle->size[1];
        dphi_unit_circle->size[0] = 1;
        dphi_unit_circle->size[1] = b->size[0];
        emxEnsureCapacity_real_T(dphi_unit_circle, i2);
        ia = b->size[0];
        for (i2 = 0; i2 < ia; i2++) {
          dphi_unit_circle->data[dphi_unit_circle->size[0] * i2] = b->data[i2];
        }

        i2 = b_b->size[0] * b_b->size[1];
        b_b->size[0] = M_bar->size[0];
        b_b->size[1] = M_bar->size[1];
        emxEnsureCapacity_real_T(b_b, i2);
        ia = M_bar->size[0] * M_bar->size[1];
        for (i2 = 0; i2 < ia; i2++) {
          b_b->data[i2] = M_bar->data[i2];
        }

        inv(b_b);
        if ((dphi_unit_circle->size[1] == 1) || (b_b->size[0] == 1)) {
          i2 = rand_angle->size[0] * rand_angle->size[1];
          rand_angle->size[0] = 1;
          rand_angle->size[1] = b_b->size[1];
          emxEnsureCapacity_real_T(rand_angle, i2);
          ia = b_b->size[1];
          for (i2 = 0; i2 < ia; i2++) {
            rand_angle->data[rand_angle->size[0] * i2] = 0.0;
            br = dphi_unit_circle->size[1];
            for (i3 = 0; i3 < br; i3++) {
              rand_angle->data[rand_angle->size[0] * i2] +=
                dphi_unit_circle->data[dphi_unit_circle->size[0] * i3] *
                b_b->data[i3 + b_b->size[0] * i2];
            }
          }
        } else {
          k = dphi_unit_circle->size[1];
          unnamed_idx_1 = (unsigned int)b_b->size[1];
          i2 = rand_angle->size[0] * rand_angle->size[1];
          rand_angle->size[1] = (int)unnamed_idx_1;
          emxEnsureCapacity_real_T(rand_angle, i2);
          b_n = b_b->size[1] - 1;
          i2 = rand_angle->size[0] * rand_angle->size[1];
          rand_angle->size[0] = 1;
          emxEnsureCapacity_real_T(rand_angle, i2);
          ia = rand_angle->size[1];
          for (i2 = 0; i2 < ia; i2++) {
            rand_angle->data[rand_angle->size[0] * i2] = 0.0;
          }

          if (b_b->size[1] != 0) {
            for (cr = 1; cr - 1 <= b_n; cr++) {
              for (iy = cr; iy <= cr; iy++) {
                rand_angle->data[iy - 1] = 0.0;
              }
            }

            br = 0;
            for (cr = 0; cr <= b_n; cr++) {
              ar = -1;
              i2 = br + k;
              for (ix = br; ix + 1 <= i2; ix++) {
                if (b_b->data[ix] != 0.0) {
                  ia = ar;
                  for (iy = cr; iy + 1 <= cr + 1; iy++) {
                    ia++;
                    rand_angle->data[iy] += b_b->data[ix] *
                      dphi_unit_circle->data[ia];
                  }
                }

                ar++;
              }

              br += k;
            }
          }
        }

        if ((rand_angle->size[1] == 1) || ((i1 - i0) - 1 == 1)) {
          b_sum = 0.0;
          for (i0 = 0; i0 < rand_angle->size[1]; i0++) {
            b_sum += rand_angle->data[rand_angle->size[0] * i0] * b->data[i0];
          }
        } else {
          b_sum = 0.0;
          for (i0 = 0; i0 < rand_angle->size[1]; i0++) {
            b_sum += rand_angle->data[rand_angle->size[0] * i0] * b->data[i0];
          }
        }

        inv(M_bar);
        if (!((int)n - 1 == 0)) {
          nmj = (int)n - 1;
        } else if (!((M_bar->size[0] == 0) || (M_bar->size[1] == 0))) {
          nmj = M_bar->size[0];
        } else {
          nmj = (int)n - 1;
          if (!(nmj > 0)) {
            nmj = 0;
          }

          if (M_bar->size[0] > nmj) {
            nmj = M_bar->size[0];
          }
        }

        empty_non_axis_sizes = (nmj == 0);
        if (empty_non_axis_sizes || (!((int)n - 1 == 0))) {
          cr = 1;
        } else {
          cr = 0;
        }

        i0 = reshapes[0].f1->size[0] * reshapes[0].f1->size[1];
        reshapes[0].f1->size[0] = nmj;
        reshapes[0].f1->size[1] = cr;
        emxEnsureCapacity_real_T(reshapes[0].f1, i0);
        ia = nmj * cr;
        for (i0 = 0; i0 < ia; i0++) {
          reshapes[0].f1->data[i0] = 0.0;
        }

        if (empty_non_axis_sizes || (!((M_bar->size[0] == 0) || (M_bar->size[1] ==
               0)))) {
          cr = M_bar->size[1];
        } else {
          cr = 0;
        }

        i0 = W__->size[0] * W__->size[1];
        W__->size[0] = reshapes[0].f1->size[0];
        W__->size[1] = reshapes[0].f1->size[1] + cr;
        emxEnsureCapacity_real_T(W__, i0);
        ia = reshapes[0].f1->size[1];
        for (i0 = 0; i0 < ia; i0++) {
          br = reshapes[0].f1->size[0];
          for (i1 = 0; i1 < br; i1++) {
            W__->data[i1 + W__->size[0] * i0] = reshapes[0].f1->data[i1 +
              reshapes[0].f1->size[0] * i0];
          }
        }

        for (i0 = 0; i0 < cr; i0++) {
          for (i1 = 0; i1 < nmj; i1++) {
            W__->data[i1 + W__->size[0] * (i0 + reshapes[0].f1->size[1])] =
              M_bar->data[i1 + nmj * i0];
          }
        }

        i0 = rand_angle->size[0] * rand_angle->size[1];
        rand_angle->size[0] = 1;
        rand_angle->size[1] = (int)n;
        emxEnsureCapacity_real_T(rand_angle, i0);
        rand_angle->data[0] = 0.0;
        ia = (int)n - 1;
        for (i0 = 0; i0 < ia; i0++) {
          rand_angle->data[rand_angle->size[0] * (i0 + 1)] = 0.0;
        }

        if (!((W__->size[0] == 0) || (W__->size[1] == 0))) {
          cr = W__->size[0];
        } else {
          cr = 0;
        }

        b_sum = 1.0 - q_->data[0] * q_->data[0] * (Q_->data[0] - b_sum);
        br = rand_angle->size[1];
        nmj = rand_angle->size[1];
        i0 = M_bar->size[0] * M_bar->size[1];
        M_bar->size[0] = 1 + cr;
        M_bar->size[1] = br;
        emxEnsureCapacity_real_T(M_bar, i0);
        for (i0 = 0; i0 < br; i0++) {
          for (i1 = 0; i1 < 1; i1++) {
            M_bar->data[M_bar->size[0] * i0] = b_sum * rand_angle->data[i0];
          }
        }

        for (i0 = 0; i0 < nmj; i0++) {
          for (i1 = 0; i1 < cr; i1++) {
            M_bar->data[(i1 + M_bar->size[0] * i0) + 1] = b_sum * W__->data[i1 +
              cr * i0];
          }
        }

        //  Convert back to E coordinate (if we want)
        //  w = S' * w_ + gamma/c_norm * c;
        //  W = S' * W_ * S;
        //     %% Sample here
        //  Since the intersection is (n-1) dimensions; remove the first element 
        if (2.0 > n) {
          i0 = 0;
          i1 = 0;
          i2 = 1;
          i3 = 1;
          ix = 1;
          iy = 1;
        } else {
          i0 = 1;
          i1 = (int)n;
          i2 = 2;
          i3 = (int)n + 1;
          ix = 2;
          iy = (int)n + 1;
        }

        cr = W__->size[0] * W__->size[1];
        W__->size[0] = i3 - i2;
        W__->size[1] = iy - ix;
        emxEnsureCapacity_real_T(W__, cr);
        ia = iy - ix;
        for (cr = 0; cr < ia; cr++) {
          br = i3 - i2;
          for (nmj = 0; nmj < br; nmj++) {
            W__->data[nmj + W__->size[0] * cr] = M_bar->data[((i2 + nmj) +
              M_bar->size[0] * ((ix + cr) - 1)) - 1];
          }
        }

        ar = 0;
        if (!((i3 - i2 == 0) || (iy - ix == 0))) {
          br = (i3 - i2) * (iy - ix);
          empty_non_axis_sizes = true;
          for (k = 1; k <= br; k++) {
            if (empty_non_axis_sizes) {
              b_sum = M_bar->data[((i2 + (k - 1) % (i3 - i2)) + M_bar->size[0] *
                                   ((ix + (k - 1) / (i3 - i2)) - 1)) - 1];
              if ((!rtIsInf(b_sum)) && (!rtIsNaN(b_sum))) {
                empty_non_axis_sizes = true;
              } else {
                empty_non_axis_sizes = false;
              }
            } else {
              empty_non_axis_sizes = false;
            }
          }

          if (empty_non_axis_sizes) {
            c_svd(W__, b);
          } else {
            unnamed_idx_0 = (unsigned int)(i3 - i2);
            unnamed_idx_1 = (unsigned int)(iy - ix);
            cr = M_bar->size[0] * M_bar->size[1];
            M_bar->size[0] = (int)unnamed_idx_0;
            M_bar->size[1] = (int)unnamed_idx_1;
            emxEnsureCapacity_real_T(M_bar, cr);
            ia = (int)unnamed_idx_0 * (int)unnamed_idx_1;
            for (cr = 0; cr < ia; cr++) {
              M_bar->data[cr] = 0.0;
            }

            c_svd(M_bar, b);
            nmj = b->size[0];
            cr = b->size[0];
            b->size[0] = nmj;
            emxEnsureCapacity_real_T1(b, cr);
            for (cr = 0; cr < nmj; cr++) {
              b->data[cr] = rtNaN;
            }
          }

          if ((i3 - i2 == 0) || (iy - ix == 0)) {
            br = 0;
          } else {
            nmj = i3 - i2;
            br = iy - ix;
            if (nmj > br) {
              br = nmj;
            }
          }

          b_sum = fabs(b->data[0]);
          if ((!rtIsInf(b_sum)) && (!rtIsNaN(b_sum))) {
            if (b_sum <= 2.2250738585072014E-308) {
              b_sum = 4.94065645841247E-324;
            } else {
              frexp(b_sum, &exponent);
              b_sum = ldexp(1.0, exponent - 53);
            }
          } else {
            b_sum = rtNaN;
          }

          b_sum *= (double)br;
          k = 1;
          while ((k <= b->size[0]) && (b->data[k - 1] > b_sum)) {
            ar++;
            k++;
          }
        }

        if (ar != (int)n - 1) {
          //  failed to sample: I will check this later, but simplify for now
          b_res = 3;
          loop++;
        } else {
          //  Using Cholesky to decompose the matrix W^-1 = L * L^T
          inv(W__);
          b_n = W__->size[1];
          info = 0;
          if (W__->size[1] != 0) {
            br = W__->size[0];
            info = 0;
            if (W__->size[0] != 0) {
              b_m = 0;
              exitg3 = false;
              while ((!exitg3) && (b_m + 1 <= br)) {
                ar = b_m + b_m * b_n;
                b_sum = 0.0;
                if (!(b_m < 1)) {
                  ix = b_m;
                  iy = b_m;
                  for (k = 1; k <= b_m; k++) {
                    b_sum += W__->data[ix] * W__->data[iy];
                    ix += b_n;
                    iy += b_n;
                  }
                }

                b_sum = W__->data[ar] - b_sum;
                if (b_sum > 0.0) {
                  b_sum = sqrt(b_sum);
                  W__->data[ar] = b_sum;
                  if (b_m + 1 < br) {
                    nmj = (br - b_m) - 1;
                    if ((nmj == 0) || (b_m == 0)) {
                    } else {
                      ix = b_m;
                      i2 = (b_m + b_n * (b_m - 1)) + 2;
                      for (cr = b_m + 2; cr <= i2; cr += b_n) {
                        b_c = -W__->data[ix];
                        iy = ar + 1;
                        i3 = (cr + nmj) - 1;
                        for (ia = cr; ia <= i3; ia++) {
                          W__->data[iy] += W__->data[ia - 1] * b_c;
                          iy++;
                        }

                        ix += b_n;
                      }
                    }

                    xscal(nmj, 1.0 / b_sum, W__, ar + 2);
                  }

                  b_m++;
                } else {
                  W__->data[ar] = b_sum;
                  info = b_m + 1;
                  exitg3 = true;
                }
              }
            }

            if (info == 0) {
              nmj = b_n;
            } else {
              nmj = info - 1;
            }

            for (b_m = 1; b_m + 1 <= nmj; b_m++) {
              for (br = 1; br <= b_m; br++) {
                W__->data[(br + W__->size[0] * b_m) - 1] = 0.0;
              }
            }

            if (1 > nmj) {
              ia = 0;
              br = 0;
            } else {
              ia = nmj;
              br = nmj;
            }

            i2 = M_bar->size[0] * M_bar->size[1];
            M_bar->size[0] = ia;
            M_bar->size[1] = br;
            emxEnsureCapacity_real_T(M_bar, i2);
            for (i2 = 0; i2 < br; i2++) {
              for (i3 = 0; i3 < ia; i3++) {
                M_bar->data[i3 + M_bar->size[0] * i2] = W__->data[i3 + W__->
                  size[0] * i2];
              }
            }

            i2 = W__->size[0] * W__->size[1];
            W__->size[0] = M_bar->size[0];
            W__->size[1] = M_bar->size[1];
            emxEnsureCapacity_real_T(W__, i2);
            ia = M_bar->size[1];
            for (i2 = 0; i2 < ia; i2++) {
              br = M_bar->size[0];
              for (i3 = 0; i3 < br; i3++) {
                W__->data[i3 + W__->size[0] * i2] = M_bar->data[i3 + M_bar->
                  size[0] * i2];
              }
            }
          }

          // follow L*L^T order
          if (info != 0) {
            b_res = 4;
            loop++;
          } else {
            inv(W__);
            i2 = wE_T->size[0] * wE_T->size[1];
            wE_T->size[0] = 1;
            wE_T->size[1] = i1 - i0;
            emxEnsureCapacity_real_T(wE_T, i2);
            ia = i1 - i0;
            for (i1 = 0; i1 < ia; i1++) {
              wE_T->data[wE_T->size[0] * i1] = w_->data[i0 + i1];
            }

            b_sum = 1.0;
            exitg3 = false;
            while ((!exitg3) && (b_sum < max_iter)) {
              //  First sample in the unit circle m points
              //  we need (n-1) angles for a circle n-dimension
              i0 = dphi_unit_circle->size[0] * dphi_unit_circle->size[1];
              dphi_unit_circle->size[0] = 1;
              dphi_unit_circle->size[1] = (int)n - 1;
              emxEnsureCapacity_real_T(dphi_unit_circle, i0);
              ia = (int)n - 1;
              for (i0 = 0; i0 < ia; i0++) {
                dphi_unit_circle->data[i0] = 0.0;
              }

              b_rand(n - 2.0, rand_angle);
              i0 = rand_angle->size[0] * rand_angle->size[1];
              rand_angle->size[0] = 1;
              emxEnsureCapacity_real_T(rand_angle, i0);
              br = rand_angle->size[0];
              nmj = rand_angle->size[1];
              ia = br * nmj;
              for (i0 = 0; i0 < ia; i0++) {
                rand_angle->data[i0] -= 0.5;
              }

              i0 = rand_angle->size[0] * rand_angle->size[1];
              rand_angle->size[0] = 1;
              emxEnsureCapacity_real_T(rand_angle, i0);
              br = rand_angle->size[0];
              nmj = rand_angle->size[1];
              ia = br * nmj;
              for (i0 = 0; i0 < ia; i0++) {
                rand_angle->data[i0] = rand_angle->data[i0] * 2.0 *
                  3.1415926535897931;
              }

              //  [-pi,pi]
              cos_cummulative = 1.0;
              for (br = 0; br <= (int)n - 3; br++) {
                dphi_unit_circle->data[dphi_unit_circle->size[0] * br] = c_rand()
                  * sin(rand_angle->data[rand_angle->size[0] * br]) *
                  cos_cummulative;
                cos_cummulative *= cos(rand_angle->data[rand_angle->size[0] * br]);
              }

              dphi_unit_circle->data[dphi_unit_circle->size[0] * ((int)n - 2)] =
                cos_cummulative;

              //  scale w.r.t the limit and size of final ellipsoid
              //          for i=1:(n-1)
              //              dphi_unit_circle(1,i) = dphi_unit_circle(1,i) * abs(dphi_lim_max) / det(inv_L); 
              //          end
              //  Transform to the ellipsoid
              if ((dphi_unit_circle->size[1] == 1) || (W__->size[0] == 1)) {
                i0 = C->size[0] * C->size[1];
                C->size[0] = 1;
                C->size[1] = W__->size[1];
                emxEnsureCapacity_real_T(C, i0);
                ia = W__->size[1];
                for (i0 = 0; i0 < ia; i0++) {
                  C->data[C->size[0] * i0] = 0.0;
                  br = dphi_unit_circle->size[1];
                  for (i1 = 0; i1 < br; i1++) {
                    C->data[C->size[0] * i0] += dphi_unit_circle->
                      data[dphi_unit_circle->size[0] * i1] * W__->data[i1 +
                      W__->size[0] * i0];
                  }
                }
              } else {
                k = dphi_unit_circle->size[1];
                unnamed_idx_1 = (unsigned int)W__->size[1];
                i0 = C->size[0] * C->size[1];
                C->size[1] = (int)unnamed_idx_1;
                emxEnsureCapacity_real_T(C, i0);
                b_n = W__->size[1] - 1;
                i0 = C->size[0] * C->size[1];
                C->size[0] = 1;
                emxEnsureCapacity_real_T(C, i0);
                ia = C->size[1];
                for (i0 = 0; i0 < ia; i0++) {
                  C->data[C->size[0] * i0] = 0.0;
                }

                if (W__->size[1] != 0) {
                  for (cr = 1; cr - 1 <= b_n; cr++) {
                    for (iy = cr; iy <= cr; iy++) {
                      C->data[iy - 1] = 0.0;
                    }
                  }

                  br = 0;
                  for (cr = 0; cr <= b_n; cr++) {
                    ar = -1;
                    i0 = br + k;
                    for (ix = br; ix + 1 <= i0; ix++) {
                      if (W__->data[ix] != 0.0) {
                        ia = ar;
                        for (iy = cr; iy + 1 <= cr + 1; iy++) {
                          ia++;
                          C->data[iy] += W__->data[ix] * dphi_unit_circle->
                            data[ia];
                        }
                      }

                      ar++;
                    }

                    br += k;
                  }
                }
              }

              i0 = dphi_unit_circle->size[0] * dphi_unit_circle->size[1];
              dphi_unit_circle->size[0] = 1;
              dphi_unit_circle->size[1] = 1 + C->size[1];
              emxEnsureCapacity_real_T(dphi_unit_circle, i0);
              dphi_unit_circle->data[0] = 0.0;
              ia = C->size[1];
              for (i0 = 0; i0 < ia; i0++) {
                dphi_unit_circle->data[dphi_unit_circle->size[0] * (i0 + 1)] =
                  C->data[C->size[0] * i0] + wE_T->data[wE_T->size[0] * i0];
              }

              if ((dphi_unit_circle->size[1] == 1) || (S->size[0] == 1)) {
                i0 = C->size[0] * C->size[1];
                C->size[0] = 1;
                C->size[1] = S->size[1];
                emxEnsureCapacity_real_T(C, i0);
                ia = S->size[1];
                for (i0 = 0; i0 < ia; i0++) {
                  C->data[C->size[0] * i0] = 0.0;
                  br = dphi_unit_circle->size[1];
                  for (i1 = 0; i1 < br; i1++) {
                    C->data[C->size[0] * i0] += dphi_unit_circle->
                      data[dphi_unit_circle->size[0] * i1] * S->data[i1 +
                      S->size[0] * i0];
                  }
                }
              } else {
                k = dphi_unit_circle->size[1];
                unnamed_idx_1 = (unsigned int)S->size[1];
                i0 = C->size[0] * C->size[1];
                C->size[1] = (int)unnamed_idx_1;
                emxEnsureCapacity_real_T(C, i0);
                b_n = S->size[1] - 1;
                i0 = C->size[0] * C->size[1];
                C->size[0] = 1;
                emxEnsureCapacity_real_T(C, i0);
                ia = C->size[1];
                for (i0 = 0; i0 < ia; i0++) {
                  C->data[C->size[0] * i0] = 0.0;
                }

                if (S->size[1] != 0) {
                  for (cr = 1; cr - 1 <= b_n; cr++) {
                    for (iy = cr; iy <= cr; iy++) {
                      C->data[iy - 1] = 0.0;
                    }
                  }

                  br = 0;
                  for (cr = 0; cr <= b_n; cr++) {
                    ar = -1;
                    i0 = br + k;
                    for (ix = br; ix + 1 <= i0; ix++) {
                      if (S->data[ix] != 0.0) {
                        ia = ar;
                        for (iy = cr; iy + 1 <= cr + 1; iy++) {
                          ia++;
                          C->data[iy] += S->data[ix] * dphi_unit_circle->data[ia];
                        }
                      }

                      ar++;
                    }

                    br += k;
                  }
                }
              }

              i0 = rand_angle->size[0] * rand_angle->size[1];
              rand_angle->size[0] = 1;
              rand_angle->size[1] = C->size[1];
              emxEnsureCapacity_real_T(rand_angle, i0);
              ia = C->size[1];
              for (i0 = 0; i0 < ia; i0++) {
                rand_angle->data[rand_angle->size[0] * i0] = C->data[C->size[0] *
                  i0] + b_gamma * c->data[i0];
              }

              //  Verify again with actual function
              empty_non_axis_sizes = heading_sample_verify(dphi_max, v_max,
                t_allow, dphi_lim_min, dphi_lim_max, phi_0, phi_n, (double)
                iv0[loop] * 2.0 * 3.1415926535897931, n, d_data, rand_angle);
              if (empty_non_axis_sizes) {
                ind++;
                ia = rand_angle->size[1];
                for (i0 = 0; i0 < ia; i0++) {
                  dphi->data[((int)ind + dphi->size[0] * i0) - 1] =
                    rand_angle->data[rand_angle->size[0] * i0];
                }

                if (ind == m) {
                  exitg3 = true;
                } else {
                  b_sum++;
                }
              } else {
                b_sum++;
              }
            }

            if (ind == m) {
              exitg2 = true;
            } else {
              loop++;
            }
          }
        }
      }

      //
      if (ind == m) {
        b_res = 1;
      } else {
        if (((int)ind > 0) && (ind < m)) {
          b_res = 5;
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  emxFree_real_T(&b_q_);
  emxFree_real_T(&C);
  emxFreeMatrix_cell_wrap_0(reshapes);
  emxFree_real_T(&b_b);
  emxFree_real_T(&rand_angle);
  emxFree_real_T(&dphi_unit_circle);
  emxFree_real_T(&wE_T);
  emxFree_real_T(&W__);
  emxFree_real_T(&w_);
  emxFree_real_T(&M_bar);
  emxFree_real_T(&Q_);
  emxFree_real_T(&q_);
  emxFree_real_T(&S);
  emxFree_real_T(&c);
  emxFree_real_T(&Q);
  emxFree_real_T(&b);
  emxFree_real_T(&a);
  *res = b_res;
}

//
// File trailer for heading_sample.cpp
//
// [EOF]
//
