#ifndef _CMLWRAPPER_H
#define _CMLWRAPPER_H

#include "cml/cml_config.h"
#include "cml/cml.h"

typedef cml::vector<double, cml::fixed<3> > Vector3d;
typedef cml::matrix<double, cml::fixed<3,3> > Matrix3d;

typedef cml::vector<double, cml::dynamic<> > VectorNd;
typedef cml::matrix<double, cml::dynamic<> > MatrixNd;

#include "SpatialAlgebra.h"

#endif /* _CMLWRAPPER_H */
