#ifndef _MATHWRAPPER_H
#define _MATHWRAPPER_H

#define EIGEN_DEFAULT_TO_ROW_MAJOR
#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"

#include "Eigen/Dense"

typedef Eigen::Matrix< double, 3, 1> Vector3d;
typedef Eigen::Matrix< double, 3, 3> Matrix3d;

typedef Eigen::VectorXd VectorNd;
typedef Eigen::MatrixXd MatrixNd;

namespace SpatialAlgebra {
	typedef Eigen::Matrix< double, 6, 1> SpatialVector;
	typedef Eigen::Matrix< double, 6, 6> SpatialMatrix;
}

#include <SpatialAlgebraOperators.h>

#endif /* _MATHWRAPPER_H */
