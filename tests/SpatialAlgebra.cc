#include <UnitTest++.h>

#include <iostream>

#include "spatialalgebra.h"

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


