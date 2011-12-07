/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _SPATIALALGEBRAOPERATORS_H
#define _SPATIALALGEBRAOPERATORS_H

#include <iostream>

namespace SpatialAlgebra {

/** \brief Contains operators such as crossf(), crossm(), etc.
 */
namespace Operators {
	inline SpatialMatrix crossm (const SpatialVector &v) {
		return SpatialMatrix (
		        0,  -v[2],  v[1],         0,          0,         0,
		 v[2],          0, -v[0],         0,          0,         0, 
		-v[1],   v[0],         0,         0,          0,         0,
		        0,  -v[5],  v[4],         0,  -v[2],  v[1],
		 v[5],          0, -v[3],  v[2],          0, -v[0],
		-v[4],   v[3],         0, -v[1],   v[0],         0
			);
	}

	inline SpatialVector crossm (const SpatialVector &v1, const SpatialVector &v2) {
		return SpatialVector (
				-v1[2] * v2[1] + v1[1] * v2[2],
				 v1[2] * v2[0] - v1[0] * v2[2],
				-v1[1] * v2[0] + v1[0] * v2[1],
				-v1[5] * v2[1] + v1[4] * v2[2] - v1[2] * v2[4] + v1[1] * v2[5],
				 v1[5] * v2[0] - v1[3] * v2[2] + v1[2] * v2[3] - v1[0] * v2[5],
				-v1[4] * v2[0] + v1[3] * v2[1] - v1[1] * v2[3] + v1[0] * v2[4]
				);
	}

	inline SpatialMatrix crossf (const SpatialVector &v) {
		return SpatialMatrix (
		        0,  -v[2],  v[1],         0,  -v[5],  v[4],
		 v[2],          0, -v[0],  v[5],          0, -v[3],
		-v[1],   v[0],         0, -v[4],   v[3],         0,
		        0,          0,         0,         0,  -v[2],  v[1],
		        0,          0,         0,  v[2],          0, -v[0],
		        0,          0,         0, -v[1],   v[0],         0
			);
	}

	inline SpatialVector crossf (const SpatialVector &v1, const SpatialVector &v2) {
		return SpatialVector (
				 -v1[2] * v2[1] + v1[1] * v2[2] - v1[5] * v2[4] + v1[4] * v2[5],
				  v1[2] * v2[0] - v1[0] * v2[2] + v1[5] * v2[3] - v1[3] * v2[5],
				 -v1[1] * v2[0] + v1[0] * v2[1] - v1[4] * v2[3] + v1[3] * v2[4],
				- v1[2] * v2[4] + v1[1] * v2[5],
				+ v1[2] * v2[3] - v1[0] * v2[5],
				- v1[1] * v2[3] + v1[0] * v2[4]
				);
	}

	inline SpatialMatrix spatial_adjoint(const SpatialMatrix &m) {
		SpatialMatrix res (m);
		res.block<3,3>(3,0) = m.block<3,3>(0,3);
		res.block<3,3>(0,3) = m.block<3,3>(3,0);
		return res;
	}

	inline SpatialMatrix spatial_inverse(const SpatialMatrix &m) {
		SpatialMatrix res(m);
		res.block<3,3>(0,0) = m.block<3,3>(0,0).transpose();
		res.block<3,3>(3,0) = m.block<3,3>(3,0).transpose();
		res.block<3,3>(0,3) = m.block<3,3>(0,3).transpose();
		res.block<3,3>(3,3) = m.block<3,3>(3,3).transpose();
		return res;
	}

	inline SpatialVector SpatialLinSolve (const SpatialMatrix &A, const SpatialVector &b) {
#ifdef RBDL_USE_SIMPLE_MATH
		std::cerr << "Cannot solve linear systems with slow math library! Use eigen instead" << std::endl;
		return b;
//		exit (-1);
#else
		return A.partialPivLu().solve(b);
#endif
	}

	inline Matrix3d get_rotation (const SpatialMatrix &m) {
		return m.block<3,3>(0,0);
	}
	inline Vector3d get_translation (const SpatialMatrix &m) {
		return Vector3d (-m(4,2), m(3,2), -m(3,1));
	}
}

}

/* _SPATIALALGEBRAOPERATORS_H*/
#endif
