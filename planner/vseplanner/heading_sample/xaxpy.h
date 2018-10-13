//
// File: xaxpy.h
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//
#ifndef XAXPY_H
#define XAXPY_H

// Include Files
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "heading_sample_types.h"

// Function Declarations
extern void b_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
                    emxArray_real_T *y, int iy0);
extern void c_xaxpy(int n, double a, const emxArray_real_T *x, int ix0,
                    emxArray_real_T *y, int iy0);
extern void xaxpy(int n, double a, int ix0, emxArray_real_T *y, int iy0);

#endif

//
// File trailer for xaxpy.h
//
// [EOF]
//
