#include <mathutils.h>

#include <math.h>

#include <iostream>
#include <assert.h>

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

