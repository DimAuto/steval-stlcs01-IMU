/*
 * ellipsoid_fit.h
 *
 *  Created on: Jul 21, 2023
 *      Author: dkalaitzakis
 */

#ifndef SRC_ELLIPSOID_FIT_H_
#define SRC_ELLIPSOID_FIT_H_

#include "lsm6_gyro.h"
#include "Fusion/Fusion.h"


int magneto_calculate(float *buffer, uint32_t samples, FusionVector *hardiron, FusionMatrix *softiron);

//                    Required Externally Defined Routines
static int Lower_Triangular_Solve(double *L, double B[], double x[], int n);
static int Lower_Triangular_Inverse(double *L, int n);
static int Upper_Triangular_Solve(double *U, double B[], double x[], int n);


void Interchange_Rows(double *A, int row1, int row2, int ncols);
static void Interchange_Columns(double *A, int col1, int col2, int nrows, int ncols);
void Identity_Matrix(double *A, int n);
void Copy_Vector(double *d, double *s, int n);
static void Hessenberg_Elementary_Transform(double* H, double *S,
		int perm[], int n);
//                        Internally Defined Routines
static void   One_Real_Eigenvalue(double Hrow[], double eigen_real[],
		double eigen_imag[], int row, double shift);
static void   Two_Eigenvalues(double *H, double *S, double eigen_real[],
		double eigen_imag[], int n, int k, double t);
static void   Update_Row(double *Hrow, double cos, double sin, int n, int k);
static void   Update_Column(double* H, double cos, double sin, int n, int k);
static void   Update_Transformation(double *S, double cos, double sin,
		int n, int k);
static void   Double_QR_Iteration(double *H, double *S, int row, int min_row,
		int n, double* shift, int iteration);
static void   Product_and_Sum_of_Shifts(double *H, int n, int max_row,
		double* shift, double *trace, double *det, int iteration);
static int    Two_Consecutive_Small_Subdiagonal(double* H, int min_row,
		int max_row, int n, double trace, double det);
static void   Double_QR_Step(double *H, int min_row, int max_row, int min_col,
		double trace, double det, double *S, int n);
static void   Complex_Division(double x, double y, double u, double v,
		double* a, double* b);
static void   BackSubstitution(double *H, double eigen_real[],
		double eigen_imag[], int n);
static void   BackSubstitute_Real_Vector(double *H, double eigen_real[],
		double eigen_imag[], int row,  double zero_tolerance, int n);
static void   BackSubstitute_Complex_Vector(double *H, double eigen_real[],
		double eigen_imag[], int row,  double zero_tolerance, int n);
static void   Calculate_Eigenvectors(double *H, double *S, double eigen_real[],
		double eigen_imag[], int n);

#endif /* SRC_ELLIPSOID_FIT_H_ */
