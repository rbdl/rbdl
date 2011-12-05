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


