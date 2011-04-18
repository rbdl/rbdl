#ifndef _MATHWRAPPER_H
#define _MATHWRAPPER_H

#define USE_EIGEN

#ifndef USE_EIGEN

	#include "cml/cml_config.h"
	#include "cml/cml.h"

	typedef cml::vector<double, cml::fixed<3> > Vector3d;
	typedef cml::matrix<double, cml::fixed<3,3> > Matrix3d;

	typedef cml::vector<double, cml::dynamic<> > VectorNd;
	typedef cml::matrix<double, cml::dynamic<> > MatrixNd;

#else

	#define EIGEN_MATRIX_PLUGIN "MatrixAddons.h"
	#define EIGEN_DEFAULT_TO_ROW_MAJOR

	#include "Eigen/Dense"

	typedef Eigen::Matrix< double, 3, 1> Vector3d;
	typedef Eigen::Matrix< double, 3, 3> Matrix3d;

	typedef Eigen::Matrix< double, 6, 1> Vector6d;
	typedef Eigen::Matrix< double, 6, 6> Matrix6d;

	typedef Eigen::VectorXd VectorNd;
	typedef Eigen::MatrixXd MatrixNd;

#endif

#include "SpatialAlgebra.h"

#endif /* _MATHWRAPPER_H */
