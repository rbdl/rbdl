#ifndef _CMLWRAPPER_H
#define _CMLWRAPPER_H

#include "cml/cml_config.h"
#include "cml/cml.h"

typedef cml::vector<double, cml::dynamic<> > cmlVector;
typedef cml::matrix<double, cml::dynamic<> > cmlMatrix;

typedef cml::vector<double, cml::fixed<3> > Vector3d;
typedef cml::matrix<double, cml::fixed<3,3> > Matrix3d;

#include "spatialalgebra.h"

/*
typedef SpatialAlgebra::SpatialVector SpatialVector;
typedef SpatialAlgebra::SpatialMatrix SpatialMatrix;
*/

//typedef cml::vector<double, cml::fixed<6> > SpatialVector;
//typedef cml::matrix<double, cml::fixed<6,6> > SpatialMatrix;

#endif /* _CMLWRAPPER_H */
