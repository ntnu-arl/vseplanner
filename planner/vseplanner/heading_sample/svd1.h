//
// File: svd1.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//
#ifndef SVD1_H
#define SVD1_H

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "heading_sample_types.h"

// Function Declarations
extern void b_svd(const emxArray_real_T *A, emxArray_real_T *U, double s_data[],
                  int s_size[1], double *V);
extern void c_svd(const emxArray_real_T *A, emxArray_real_T *U);

#endif

//
// File trailer for svd1.h
//
// [EOF]
//
