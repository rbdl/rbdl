#include <UnitTest++.h>
#include "SimpleMathFixed.h"
#include <iostream>

using namespace std;
using namespace SimpleMath;

typedef SimpleMath::Fixed::Matrix<double, 4, 4> Matrix44d;
typedef SimpleMath::Fixed::Matrix<double, 3, 3> Matrix33d;
typedef SimpleMath::Fixed::Matrix<double, 4, 1> Vector4d;
typedef SimpleMath::Fixed::Matrix<double, 3, 1> Vector3d;

TEST (SimpleTestFixed) {
	Matrix44d mymatrix;
	Vector4d myvector;

	mymatrix.identity();
	myvector.random();

	Vector4d result = mymatrix * myvector;

	CHECK_ARRAY_EQUAL (myvector.data(), result, myvector.size());
}

TEST (FixedTestBlockFullMatrix) {
	Matrix44d mymatrix;
	Vector4d myvector;

	mymatrix.identity();
	myvector.random();

	Matrix44d othermatrix = Matrix44d::Zero (4,4);

	mymatrix.block<4,4>(0,0) = othermatrix;

	CHECK_EQUAL (mymatrix, othermatrix);
}

TEST (FixedTestBlockPartialToBorderMatrix) {
	Matrix44d mymatrix;
	Vector4d myvector;

	mymatrix.setZero();
	mymatrix(3,3) = 1.;
	mymatrix.identity();
	myvector.random();

	Matrix33d othermatrix = Matrix33d::Identity (3,3);

	mymatrix.block<3,3>(0,0) = othermatrix;

	CHECK_EQUAL (mymatrix, Matrix44d::Identity (4,4));
}
