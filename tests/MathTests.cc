#include <UnitTest++.h>

#include "Logging.h"
#include "mathutils.h"
#include "mathwrapper.h"
#include <iostream>

const double TEST_PREC = 1.0e-14;

using namespace std;

struct MathFixture {
};

TEST (GaussElimPivot) {
	ClearLogOutput();

	MatrixNd A;
	A.resize(3,3);
	VectorNd b(3);
	VectorNd x(3);

	A(0,0) = 0; A(0,1) = 2; A(0,2) = 1;
	A(1,0) = 1; A(1,1) = 1; A(1,2) = 5;
	A(2,0) = 0; A(2,1) = 0; A(2,2) = 1;

	b[0] = 1;
	b[1] = 2;
	b[2] = 3;

	VectorNd test_result (3);

	test_result[0] = -12;
	test_result[1] = -1;
	test_result[2] = 3;

	LinSolveGaussElimPivot (A, b, x);

	CHECK_ARRAY_CLOSE (test_result.data(), x.data(), 3, TEST_PREC);

	A(0,0) = 0; A(0,1) = -2; A(0,2) = 1;
	A(1,0) = 1; A(1,1) =  1; A(1,2) = 5;
	A(2,0) = 0; A(2,1) =  0; A(2,2) = 1;

	LinSolveGaussElimPivot (A, b, x);
	test_result[0] = -14;
	test_result[1] = 1;
	test_result[2] = 3;

	CHECK_ARRAY_CLOSE (test_result.data(), x.data(), 3, TEST_PREC);
}

TEST (Dynamic_1D_initialize_value) {
	VectorNd myvector_10 = VectorNd::Constant ((size_t) 10, 12.);

	double *test_values = new double[10];
	for (unsigned int i = 0; i < 10; i++)
		test_values[i] = 12.;

	CHECK_ARRAY_EQUAL (test_values, myvector_10.data(), 10);
	delete[] test_values;
}

TEST (Dynamic_2D_initialize_value) {
	MatrixNd mymatrix_10x10 = MatrixNd::Constant (10, 10, 12.);

	double *test_values = new double[10 * 10];
	for (unsigned int i = 0; i < 10; i++)
		for (unsigned int j = 0; j < 10; j++)
			test_values[i*10 + j] = 12.;

	CHECK_ARRAY_EQUAL (test_values, mymatrix_10x10.data(), 10*10);
	delete[] test_values;
}
