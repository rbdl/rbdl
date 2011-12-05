#include <UnitTest++.h>
#include "SimpleMathDynamic.h"
#include <iostream>

using namespace std;
using namespace SimpleMath;

typedef SimpleMath::Dynamic::Matrix<double> MatrixXd;
typedef SimpleMath::Dynamic::Matrix<double> VectorXd;

TEST (SimpleTestDynamic) {
	MatrixXd mymatrix (4,4);
	MatrixXd myvector (4);

	mymatrix.identity();
	myvector.random();

	MatrixXd result = MatrixXd::Zero (3);
  result = MatrixXd::Zero (3,3);
	result.resize(4);
	result = mymatrix * myvector;

	CHECK_EQUAL (4, result.size());
	CHECK_EQUAL (4, result.rows());
	CHECK_EQUAL (1, result.cols());
	CHECK_ARRAY_EQUAL (myvector.data(), result, myvector.size());
}
