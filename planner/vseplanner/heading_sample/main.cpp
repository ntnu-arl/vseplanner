//
// File: main.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 13-May-2018 17:47:51
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
#include <stdio.h>
#include <iostream>
#include <ctime>

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
  double d_data[3] = {1.32785, 0.482216, 0.226923};
  int d_size[1] = {3};
  double res;
  emxInitArray_real_T(&dphi, 2);
  
  
  ////tung
  
	dphi_max =0.314160; 
	v_max =0.200000; 
	t_allow =17.986503; 
	dphi_lim_min =-1.570796; 
	dphi_lim_max =1.570796; 
	phi_0 =1.406940; 
	phi_n =-1.852485; 
	n = 7.000000; 
	m = 5.000000; 


	dphi_max = 0.314160;
	v_max = 0.200000;
	t_allow = 25.000000;
	dphi_lim_min = -1.570796*2;
	dphi_lim_max = 1.570796*2;
	phi_0 = 1.458305;
	phi_n = -1.428062;
	n = 3;
	m = 500;
	int max_iter = 2000;



  // Initialize function input argument 'd'.
  //argInit_d50x1_real_T(d_data, d_size);

	clock_t t1, t2;
    t1 = clock();

  // Call the entry-point 'heading_sample'.
	heading_sample(dphi_max, v_max, t_allow, dphi_lim_min, dphi_lim_max, phi_0,
                 phi_n, n, m, d_data, d_size, max_iter, &res, dphi);

	std::cout << "Result: " << res << "\n" << std::endl;
	if (res == 1)
		for (int i=1; i < (dphi->size[0] * dphi->size[1]); i++){
			std::cout << dphi->data[i] << ",";
		}
	
	t2 = clock();
	float diff = ((float)t2-(float)t1);
	printf("\nTime to sample: %f (sec) ", diff / CLOCKS_PER_SEC);

	
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
