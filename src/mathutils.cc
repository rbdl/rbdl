#include <mathutils.h>

#include "stacktrace.h"
#include <cmath>
#include <limits>

#include <iostream>
#include <assert.h>

#include "Logging.h"

using namespace SpatialAlgebra;

Vector3d Vector3dZero (0., 0., 0.);
Matrix3d Matrix3dIdentity (
		1., 0., 0.,
		0., 1., 0.,
		0., 0., 1
		);
Matrix3d Matrix3dZero (
		0., 0., 0.,
		0., 0., 0.,
		0., 0., 0.
		);
SpatialMatrix SpatialMatrixIdentity (
		1., 0., 0., 0., 0., 0.,
		0., 1., 0., 0., 0., 0.,
		0., 0., 1., 0., 0., 0.,
		0., 0., 0., 1., 0., 0.,
		0., 0., 0., 0., 1., 0.,
		0., 0., 0., 0., 0., 1.
		);

void VectorCrossVector (Vector3d &result, const Vector3d &vec_a, const Vector3d &vec_b) {
	result[0] = vec_a[1]*vec_b[2] - vec_a[2]*vec_b[1];
	result[1] = vec_a[2]*vec_b[0] - vec_a[0]*vec_b[2];
	result[2] = vec_a[0]*vec_b[1] - vec_a[1]*vec_b[0];
}

void VectorPrint (const char* msg, const Vector3d &vector) {
	std::cout << msg;
	int i;
	for (i = 0; i < 3; i++)
		std::cout << vector[i] << "\t";
	std::cout << std::endl;
}

void MatrixPrint (const char* msg, const Matrix3d &matrix) {
	std::cout << msg;
	int i, j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			std::cout << matrix(i,j) << "\t";
		}
		std::cout << std::endl;
	}
}

inline void MatrixSetIdentity (Matrix3d &result) {
	result(0,0) = 1.;
	result(0,1) = 0.;
	result(0,2) = 0.;

	result(1,0) = 0.;
	result(1,1) = 1.;
	result(1,2) = 0.;
	
	result(2,0) = 0.;
	result(2,1) = 0.;
	result(2,2) = 1.;
}

inline void MatrixSetZero (Matrix3d &result) {
	result(0,0) = 0.;
	result(0,1) = 0.;
	result(0,2) = 0.;

	result(1,0) = 0.;
	result(1,1) = 0.;
	result(1,2) = 0.;
	
	result(2,0) = 0.;
	result(2,1) = 0.;
	result(2,2) = 0.;
}

inline void MatrixCopyTranspose (Matrix3d &result, const Matrix3d &src) {
	int i,j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			result(i,j) = src(i,j);
		}
	}
}

inline void MatrixCopy (const Matrix3d &result, Matrix3d &src) {
	src = result;
}

Matrix3d VectorCrossMatrix (const Vector3d &vector) {
	return Matrix3d (
			         0, -vector[2],  vector[1],
			 vector[2],          0, -vector[0],
			-vector[1],  vector[0], 0);
}

bool LinSolveGaussElim (cmlMatrix A, cmlVector b, cmlVector &x) {
	x.zero();

	// We can only solve quadratic systems
	assert (A.rows() == A.cols());

	unsigned int n = A.rows();
	
	int i,j;
	for (j = 0; j < n; j++) {
		for (i = j + 1; i < n; i++) {
			if (fabs(A(j,j)) <= std::numeric_limits<double>::epsilon()) {
				std::cout << LogOutput.str() << std::endl;
			}
			assert (fabs(A(j,j)) > std::numeric_limits<double>::epsilon());
			double d = A(i,j)/A(j,j);

			b[i] -= b[j] * d;

			int k;
			for (k = j; k < n; k++) {
				A(i,k) -= A(j,k) * d;
			}
		}
	}

	for (i = n - 1; i >= 0; i--) {
		for (j = i + 1; j < n; j++) {
			x[i] += A(i,j) * x[j];
		}
		x[i] = (b[i] - x[i]) / A(i,i);
	}

	return true;
}

bool LinSolveGaussElimPivot (cmlMatrix A, cmlVector b, cmlVector &x) {
	x.zero();

	// We can only solve quadratic systems
	assert (A.rows() == A.cols());

	unsigned int n = A.rows();
	unsigned int pi;

	// the pivots
	size_t *pivot = new size_t[n];

	// temporary result vector which contains the pivoted result
	cmlVector px(x);
	
	int i,j,k;

	for (i = 0; i < n; i++)
		pivot[i] = i;

	for (j = 0; j < n; j++) {
		pi = j;
		double pv = fabs (A(j,pivot[j]));

		// LOG << "j = " << j << " pv = " << pv << std::endl;
		// find the pivot
		for (k = j; k < n; k++) {
			double pt = fabs (A(j,pivot[k]));
			if (pt > pv) {
				pv = pt;
				pi = k;
				unsigned int p_swap = pivot[j];
				pivot[j] = pivot[pi];
				pivot[pi] = p_swap;
			//	LOG << "swap " << j << " with " << pi << std::endl;
			//	LOG << "j = " << j << " pv = " << pv << std::endl;
			}
		}

		for (i = j + 1; i < n; i++) {
			if (fabs(A(j,pivot[j])) <= std::numeric_limits<double>::epsilon()) {
				LOG << "Pivoting failed for matrix A = " << std::endl;
				LOG << "A = " << std::endl << A << std::endl;
//				LOG << "b = " << b << std::endl;
				std::cout << LogOutput.str() << std::endl;
			}
			assert (fabs(A(j,pivot[j])) > std::numeric_limits<double>::epsilon());
			double d = A(i,pivot[j])/A(j,pivot[j]);

			b[i] -= b[j] * d;

			int k;
			for (k = j; k < n; k++) {
				A(i,pivot[k]) -= A(j,pivot[k]) * d;
			}
		}
	}

	for (i = n - 1; i >= 0; i--) {
		for (j = i + 1; j < n; j++) {
			px[i] += A(i,pivot[j]) * px[j];
		}
		px[i] = (b[i] - px[i]) / A(i,pivot[i]);
	}

	// Unswapping
	for (i = 0; i < n; i++) {
		x[pivot[i]] = px[i];
	}

	/*
	LOG << "A = " << std::endl << A << std::endl;
	LOG << "b = " << b << std::endl;
	LOG << "x = " << x << std::endl;
	LOG << "pivot = " << pivot[0] << " " << pivot[1] << " " << pivot[2] << std::endl;
	std::cout << LogOutput.str() << std::endl;
	*/

	delete[] pivot;

	return true;
}

void SpatialMatrixSetSubmatrix(SpatialMatrix &dest, unsigned int row, unsigned int col, const Matrix3d &matrix) {
	assert (row < 2 && col < 2);
	
	dest(row*3,col*3) = matrix(0,0);
	dest(row*3,col*3 + 1) = matrix(0,1);
	dest(row*3,col*3 + 2) = matrix(0,2);

	dest(row*3 + 1,col*3) = matrix(1,0);
	dest(row*3 + 1,col*3 + 1) = matrix(1,1);
	dest(row*3 + 1,col*3 + 2) = matrix(1,2);

	dest(row*3 + 2,col*3) = matrix(2,0);
	dest(row*3 + 2,col*3 + 1) = matrix(2,1);
	dest(row*3 + 2,col*3 + 2) = matrix(2,2);
}

bool SpatialMatrixCompareEpsilon (const SpatialMatrix &matrix_a, const SpatialMatrix &matrix_b, double epsilon) {
	assert (epsilon >= 0.);
	unsigned int i, j;

	for (i = 0; i < 6; i++) {
		for (j = 0; j < 6; j++) {
			if (fabs(matrix_a(i,j) - matrix_b(i,j)) >= epsilon) {
				std::cerr << "Expected:" 
					<< std::endl << matrix_a << std::endl
					<< "but was" << std::endl 
					<< matrix_b << std::endl;
				return false;
			}
		}
	}

	return true;
}

bool SpatialVectorCompareEpsilon (const SpatialVector &vector_a, const SpatialVector &vector_b, double epsilon) {
	assert (epsilon >= 0.);
	unsigned int i, j;

	for (i = 0; i < 6; i++) {
		if (fabs(vector_a[i] - vector_b[i]) >= epsilon) {
			std::cerr << "Expected:" 
				<< std::endl << vector_a << std::endl
				<< "but was" << std::endl 
				<< vector_b << std::endl;
			return false;
		}
	}

	return true;
}

SpatialMatrix Xtrans (const Vector3d &r) {
	return SpatialMatrix (
			   1.,    0.,    0.,  0.,  0.,  0.,
			   0.,    1.,    0.,  0.,  0.,  0.,
			   0.,    0.,    1.,  0.,  0.,  0.,
			   0.,  r[2], -r[1],  1.,  0.,  0.,
			-r[2],    0.,  r[0],  0.,  1.,  0.,
			 r[1], -r[0],    0.,  0.,  0.,  1.
			);
}

SpatialMatrix Xrotx (const double &xrot) {
	double s, c;
	s = sin (xrot);
	c = cos (xrot);

	return SpatialMatrix(
			   1.,    0.,    0.,  0.,  0.,  0.,
			   0.,     c,     s,  0.,  0.,  0.,
			   0.,    -s,     c,  0.,  0.,  0.,
			   0.,    0.,    0.,  1.,  0.,  0.,
			   0.,    0.,    0.,  0.,   c,   s,
			   0.,    0.,    0.,  0.,  -s,   c
			);
}

SpatialMatrix Xroty (const double &yrot) {
	double s, c;
	s = sin (yrot);
	c = cos (yrot);

	return SpatialMatrix(
			    c,    0.,    -s,  0.,  0.,  0.,
			   0.,    1.,    0.,  0.,  0.,  0.,
			    s,    0.,     c,  0.,  0.,  0.,
			   0.,    0.,    0.,   c,  0.,  -s,
			   0.,    0.,    0.,  0.,  1.,  0.,
			   0.,    0.,    0.,   s,  0.,   c
			);
}

SpatialMatrix Xrotz (const double &zrot) {
	double s, c;
	s = sin (zrot);
	c = cos (zrot);

	return SpatialMatrix(
			    c,     s,    0.,  0.,  0.,  0.,
			   -s,     c,    0.,  0.,  0.,  0.,
			   0.,    0.,    1.,  0.,  0.,  0.,
			   0.,    0.,    0.,   c,   s,  0.,
			   0.,    0.,    0.,  -s,   c,  0.,
			   0.,    0.,    0.,  0.,  0.,  1.
			);
}

SpatialMatrix XtransRotZYXEuler (const Vector3d &displacement, const Vector3d &zyx_euler) {
	return Xtrans(displacement) * Xrotz(zyx_euler[0]) * Xroty(zyx_euler[1]) * Xrotx(zyx_euler[2]);
}
