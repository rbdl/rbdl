/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MATHWRAPPER_H
#define _MATHWRAPPER_H

#include <rbdl_config.h>

#ifdef RBDL_USE_SIMPLE_MATH
  #include "SimpleMath/SimpleMath.h"
	#include <vector>

	typedef SimpleMath::Fixed::Matrix<double, 3,1> Vector3_t;
	typedef SimpleMath::Fixed::Matrix<double, 3,3> Matrix3_t;

	typedef SimpleMath::Fixed::Matrix<double, 6,1> SpatialVector_t;
	typedef SimpleMath::Fixed::Matrix<double, 6,6> SpatialMatrix_t;

	typedef SimpleMath::Dynamic::Matrix<double> MatrixN_t;
	typedef SimpleMath::Dynamic::Matrix<double> VectorN_t;

#else
	#define EIGEN_DEFAULT_TO_ROW_MAJOR
	#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

	#include "Eigen/Dense"
	#include "Eigen/StdVector"

	typedef Eigen::Matrix< double, 3, 1> Vector3_t;
	typedef Eigen::Matrix< double, 3, 3> Matrix3_t;

	typedef Eigen::VectorXd VectorN_t;
	typedef Eigen::MatrixXd MatrixN_t;

	typedef Eigen::Matrix< double, 6, 1> SpatialVector_t;
	typedef Eigen::Matrix< double, 6, 6> SpatialMatrix_t;
#endif

namespace RigidBodyDynamics {

/** \brief Math types such as vectors and matrices and utility functions. */
namespace Math {
	typedef Vector3_t Vector3d;
	typedef Matrix3_t Matrix3d;
	typedef SpatialVector_t SpatialVector;
	typedef SpatialMatrix_t SpatialMatrix;
	typedef VectorN_t VectorNd;
	typedef MatrixN_t MatrixNd;
} /* Math */

} /* RigidBodyDynamics */

#include "SpatialAlgebraOperators.h"

// If we use Eigen3 we have to create specializations of the STL
// std::vector such that the alignment is done properly.
#ifndef RBDL_USE_SIMPLE_MATH
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialVector)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialMatrix)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialTransform)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Math::SpatialRigidBodyInertia)
#endif

#endif /* _MATHWRAPPER_H */
