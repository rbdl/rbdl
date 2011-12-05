#include <UnitTest++.h>

#include "SimpleMath.h"

#include <iostream>

using namespace std;
using namespace SimpleMath;

typedef SimpleMath::Dynamic::Matrix<double> MatrixXd;
typedef SimpleMath::Dynamic::Matrix<double> VectorXd;

TEST (SimpleTestMixed) {
	Fixed::Matrix<double, 1, 3> row_vector;
	Dynamic::Matrix<double> col_vector (3,1);

	row_vector[0] = 1.;
	row_vector[1] = 2.;
	row_vector[2] = 3.;

	col_vector[0] = 4.;
	col_vector[1] = 5.;
	col_vector[2] = 6.;

	Dynamic::Matrix<double> scalar_result = row_vector * col_vector;

	CHECK_EQUAL (1, scalar_result.size());
	CHECK_EQUAL (1, scalar_result.rows());
	CHECK_EQUAL (1, scalar_result.cols());
	CHECK_EQUAL (32., scalar_result[0]);

	Dynamic::Matrix<double> outer_result = col_vector * row_vector;

	CHECK_EQUAL (9, outer_result.size());
	CHECK_EQUAL (3, outer_result.rows());
	CHECK_EQUAL (3, outer_result.cols());

	Fixed::Matrix<double, 3, 3> outer_test_result;
	outer_test_result(0,0) =  4;
	outer_test_result(0,1) =  8;
	outer_test_result(0,2) = 12;

	outer_test_result(1,0) =  5;
	outer_test_result(1,1) = 10;
	outer_test_result(1,2) = 15;

	outer_test_result(2,0) =  6;
	outer_test_result(2,1) = 12;
	outer_test_result(2,2) = 18;

	CHECK_EQUAL (outer_test_result, outer_result);
}
