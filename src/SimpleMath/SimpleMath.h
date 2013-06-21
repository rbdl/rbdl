#ifndef _SIMPLEMATH_H
#define _SIMPLEMATH_H

#include "SimpleMathFixed.h"
#include "SimpleMathDynamic.h"
#include "SimpleMathMixed.h"
#include "SimpleMathQR.h"
#include "SimpleMathCommaInitializer.h"

typedef SimpleMath::Fixed::Matrix<int, 3, 1> Vector3i;

typedef SimpleMath::Fixed::Matrix<double, 3, 1> Vector3d;
typedef SimpleMath::Fixed::Matrix<double, 3, 3> Matrix33d;

typedef SimpleMath::Fixed::Matrix<double, 4, 1> Vector4d;

typedef SimpleMath::Fixed::Matrix<float, 3, 1> Vector3f;
typedef SimpleMath::Fixed::Matrix<float, 4, 1> Vector4f;
typedef SimpleMath::Fixed::Matrix<float, 3, 3> Matrix33f;
typedef SimpleMath::Fixed::Matrix<float, 4, 4> Matrix44f;


#endif /* _SIMPLEMATH_H */
