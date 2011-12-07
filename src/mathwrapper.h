/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MATHWRAPPER_H
#define _MATHWRAPPER_H

#include "rbdlconfig.h"

#ifdef RBDL_USE_SIMPLE_MATH
  #include "SimpleMath.h"
	#include <vector>

	typedef SimpleMath::Fixed::Matrix<double, 3,1> Vector3d;
	typedef SimpleMath::Fixed::Matrix<double, 3,3> Matrix3d;

	namespace SpatialAlgebra {
		typedef SimpleMath::Fixed::Matrix<double, 6,1> SpatialVector;
		typedef SimpleMath::Fixed::Matrix<double, 6,6> SpatialMatrix;
	}

	typedef SimpleMath::Dynamic::Matrix<double> MatrixNd;
	typedef SimpleMath::Dynamic::Matrix<double> VectorNd;

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

#include "SpatialAlgebraOperators.h"

#endif /* _MATHWRAPPER_H */
