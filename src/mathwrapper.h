#ifndef _MATHWRAPPER_H
#define _MATHWRAPPER_H

#ifndef USE_EIGEN

	#include "cml/cml_config.h"
	#include "cml/cml.h"

	typedef cml::vector<double, cml::fixed<3> > Vector3d;
	typedef cml::matrix<double, cml::fixed<3,3> > Matrix3d;

	typedef cml::vector<double, cml::dynamic<> > VectorNd;
	typedef cml::matrix<double, cml::dynamic<> > MatrixNd;

#else

	#include "Eigen/Dense"
	typedef Eigen::Vector3d Vector3d;
	typedef Eigen::Matrix3d Matrix3d;

	typedef Eigen::VectorXd VectorNd;
	typedef Eigen::MatrixXd MatrixNd;

#endif


#include "SpatialAlgebra.h"

#endif /* _MATHWRAPPER_H */
