/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MATHWRAPPER_H
#define _MATHWRAPPER_H

#include <rbdlconfig.h>

#ifdef RBDL_USE_SIMPLE_MATH
  #include "SimpleMath/SimpleMath.h"
	#include <vector>

	typedef SimpleMath::Fixed::Matrix<double, 3,1> Vector3_t;
	typedef SimpleMath::Fixed::Matrix<double, 3,3> Matrix3_t;

	typedef SimpleMath::Fixed::Matrix<double, 6,1> SpatialVector_t;
	typedef SimpleMath::Fixed::Matrix<double, 6,6> SpatialMatrix_t;

	typedef SimpleMath::Dynamic::Matrix<double> MatrixNd_t;
	typedef SimpleMath::Dynamic::Matrix<double> VectorNd_t;

#else
	#define EIGEN_DEFAULT_TO_ROW_MAJOR
	#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

	#include "Eigen/Dense"
	#include "Eigen/StdVector"

	typedef Eigen::Matrix< double, 3, 1> Vector3d;
	typedef Eigen::Matrix< double, 3, 3> Matrix3d;

	typedef Eigen::VectorXd VectorNd;
	typedef Eigen::MatrixXd MatrixNd;

	namespace SpatialAlgebra {
		typedef Eigen::Matrix< double, 6, 1> SpatialVector;
		typedef Eigen::Matrix< double, 6, 6> SpatialMatrix;
	}

	inline EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SpatialAlgebra::SpatialVector)
	inline EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SpatialAlgebra::SpatialMatrix)
#endif

namespace RigidBodyDynamics {
namespace Math {
	typedef Vector3_t Vector3d;
	typedef Matrix3_t Matrix3d;
	typedef SpatialVector_t SpatialVector;
	typedef SpatialMatrix_t SpatialMatrix;
	typedef VectorNd_t VectorNd;
	typedef MatrixNd_t MatrixNd;
} /* Math */
} /* RigidBodyDynamics */

#include "SpatialAlgebraOperators.h"

#endif /* _MATHWRAPPER_H */
