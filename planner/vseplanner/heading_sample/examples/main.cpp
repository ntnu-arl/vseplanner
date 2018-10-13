//
// File: main.cpp
//
// MATLAB Coder version            : 3.4
// C/C++ source code generated on  : 12-Oct-2018 11:37:27
//

//***********************************************************************
// This automatically generated example C main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************
// Include Files
#include "rt_nonfinite.h"
#include "heading_sample.h"
#include "main.h"
#include "heading_sample_terminate.h"
#include "heading_sample_emxAPI.h"
#include "heading_sample_initialize.h"

// Function Declarations
static void argInit_d50x1_real_T(double result_data[], int result_size[1]);
static double argInit_real_T();
static void main_heading_sample();

// Function Definitions

//
// Arguments    : double result_data[]
//                int result_size[1]
// Return Type  : void
//
static void argInit_d50x1_real_T(double result_data[], int result_size[1])
{
  int idx0;

  // Set the size of the array.
  // Change this size to the value that the application requires.
  result_size[0] = 2;

  // Loop over the array to initialize each element.
  for (idx0 = 0; idx0 < 2; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result_data[idx0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_heading_sample()
{
  emxArray_real_T *dphi;
  double dphi_max;
  double v_max;
  double t_allow;
  double dphi_lim_min;
  double dphi_lim_max;
  double phi_0;
  double phi_n;
  double n;
  double m;
  double d_data[50];
  int d_size[1];
  double res;
  emxInitArray_real_T(&dphi, 2);

  // Initialize function 'heading_sample' input arguments.
  dphi_max = argInit_real_T();
  v_max = argInit_real_T();
  t_allow = argInit_real_T();
  dphi_lim_min = argInit_real_T();
  dphi_lim_max = argInit_real_T();
  phi_0 = argInit_real_T();
  phi_n = argInit_real_T();
  n = argInit_real_T();
  m = argInit_real_T();

  // Initialize function input argument 'd'.
  argInit_d50x1_real_T(d_data, d_size);

  // Call the entry-point 'heading_sample'.
  heading_sample(dphi_max, v_max, t_allow, dphi_lim_min, dphi_lim_max, phi_0,
                 phi_n, n, m, d_data, d_size, argInit_real_T(), &res, dphi);
  emxDestroyArray_real_T(dphi);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // Initialize the application.
  // You do not need to do this more than one time.
  heading_sample_initialize();

  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_heading_sample();

  // Terminate the application.
  // You do not need to do this more than one time.
  heading_sample_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//
