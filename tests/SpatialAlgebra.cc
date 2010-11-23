#include <UnitTest++.h>

#include <iostream>

#include "spatialalgebra.h"

using namespace std;
using namespace SpatialAlgebra;

/// \brief Checks the multiplication of a SpatialMatrix with a SpatialVector
TEST(TestSpatialMatrixTimesSpatialVector) {
	SpatialMatrix s_matrix (
			1., 0., 0., 0., 0., 7.,
			0., 2., 0., 0., 8., 0.,
			0., 0., 3., 9., 0., 0.,
			0., 0., 6., 4., 0., 0.,
			0., 5., 0., 0., 5., 0.,
			4., 0., 0., 0., 0., 6.
			);
	SpatialVector s_vector (
			1., 2., 3., 4., 5., 6.
			);

	SpatialVector result;
	result = s_matrix * s_vector;

	SpatialVector test_result (
			43., 44., 45., 34., 35., 40.
			);
	CHECK_EQUAL (test_result, result);
}

/// \brief Checks the multiplication of a scalar with a SpatialVector
TEST(TestScalarTimesSpatialVector) {
	SpatialVector s_vector (
			1., 2., 3., 4., 5., 6.
			);

	SpatialVector result;
	result = 3. * s_vector;
	
	SpatialVector test_result(3., 6., 9., 12., 15., 18.);

	CHECK_EQUAL (test_result, result);
}

/// \brief Checks the multiplication of a scalar with a SpatialMatrix
TEST(TestScalarTimesSpatialMatrix) {
	SpatialMatrix s_matrix (
			1., 0., 0., 0., 0., 7.,
			0., 2., 0., 0., 8., 0.,
			0., 0., 3., 9., 0., 0.,
			0., 0., 6., 4., 0., 0.,
			0., 5., 0., 0., 5., 0.,
			4., 0., 0., 0., 0., 6.
			);
	
	SpatialMatrix result;
	result = 3. * s_matrix;
	
	SpatialMatrix test_result(
			3., 0., 0., 0., 0., 21.,
			0., 6., 0., 0., 24., 0.,
			0., 0., 9., 27., 0., 0.,
			0., 0., 18., 12., 0., 0.,
			0., 15., 0., 0., 15., 0.,
			12., 0., 0., 0., 0., 18.
			);

	CHECK_EQUAL (test_result, result);
}

/// \brief Checks the multiplication of a scalar with a SpatialMatrix
TEST(TestSpatialMatrixTimesSpatialMatrix) {
	SpatialMatrix s_matrix (
			1., 0., 0., 0., 0., 7.,
			0., 2., 0., 0., 8., 0.,
			0., 0., 3., 9., 0., 0.,
			0., 0., 6., 4., 0., 0.,
			0., 5., 0., 0., 5., 0.,
			4., 0., 0., 0., 0., 6.
			);
	
	SpatialMatrix result;
	result = s_matrix * s_matrix;
	
	SpatialMatrix test_result(
			29., 0., 0., 0., 0., 49.,
			0., 44., 0., 0., 56., 0.,
			0., 0., 63., 63., 0., 0.,
			0., 0., 42., 70., 0., 0.,
			0., 35., 0., 0., 65., 0.,
			28., 0., 0., 0., 0., 64.
			);

	CHECK_EQUAL (test_result, result);
}

TEST(TestSpatialMatrixInverse) {
	SpatialMatrix s_matrix (
			0, 1, 2, 0, 1, 2,
			3, 4, 5, 3, 4, 5,
			6, 7, 8, 6, 7, 8,
			0, 1, 2, 0, 1, 2,
			3, 4, 5, 3, 4, 5,
			6, 7, 8, 6, 7, 8
			);

	SpatialMatrix test_inv (
			0, 3, 6, 0, 3, 6,
			1, 4, 7, 1, 4, 7,
			2, 5, 8, 2, 5, 8,
			0, 3, 6, 0, 3, 6,
			1, 4, 7, 1, 4, 7,
			2, 5, 8, 2, 5, 8
			);
			
	CHECK_EQUAL (test_inv, s_matrix.inverse());
}

TEST(TestSpatialVectorCross) {
	SpatialVector s_vec (1., 2., 3., 4., 5., 6.);
	
	SpatialMatrix test_cross (
			 0., -3.,  2.,  0.,  0.,  0.,
			 3.,  0., -1.,  0.,  0.,  0.,
			-2.,  1.,  0.,  0.,  0.,  0.,
			 0., -6.,  5.,  0., -3.,  2.,
			 6.,  0., -4.,  3.,  0., -1.,
			-5.,  4.,  0., -2.,  1.,  0.
			);

	SpatialMatrix s_vec_cross (s_vec.crossm());
	CHECK_EQUAL (test_cross, s_vec_cross);

	SpatialMatrix s_vec_crossf (s_vec.crossf());
	SpatialMatrix test_crossf = -1. * s_vec.crossm().transpose();

	CHECK_EQUAL (test_crossf, s_vec_crossf);
}

TEST(TestSpatialLinSolve) {
	SpatialVector b (1, 2, 0, 1, 1, 1);
	SpatialMatrix A (
			1., 2., 3., 0., 0., 0.,
			3., 4., 5., 0., 0., 0.,
			6., 7., 7., 0., 0., 0.,
			0., 0., 0., 1., 0., 0.,
			0., 0., 0., 0., 1., 0.,
			0., 0., 0., 0., 0., 1.
			);

	SpatialVector x = SpatialLinSolve (A, b);
	SpatialVector x_test (3.5, -6.5, 3.5, 1, 1, 1);

	CHECK_EQUAL (x_test, x);
}
