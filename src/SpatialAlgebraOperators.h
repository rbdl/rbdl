/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _SPATIALALGEBRAOPERATORS_H
#define _SPATIALALGEBRAOPERATORS_H

#include <iostream>
#include <cmath>


namespace RigidBodyDynamics {

namespace Math {
/** \brief Spatial algebra matrices, vectors, and operators. */

/** \brief Compact representation of spatial transformations.
 *
 * Instead of using a verbose 6x6 matrix, this structure only stores a 3x3
 * matrix and a 3-d vector to store spatial transformations. It also
 * encapsulates efficient operations such as concatenations and
 * transformation of spatial vectors.
 */
struct SpatialTransform {
	SpatialTransform() :
		E (Matrix3d::Identity(3,3)),
		r (Vector3d::Zero(3,1))
	{}
	SpatialTransform (Matrix3d rotation, Vector3d translation) :
		E (rotation),
		r (translation)
	{}

	/** Same as X * v.
	 *
	 * \returns (E * w, -rxw + Ev)
	 */
	SpatialVector apply (const SpatialVector &v_sp) {
		Vector3d v_rxw (
				v_sp[3] - r[1]*v_sp[2] + r[2]*v_sp[1],
				v_sp[4] - r[2]*v_sp[0] + r[0]*v_sp[2],
				v_sp[5] - r[0]*v_sp[1] + r[1]*v_sp[0]
				);
		return SpatialVector (
				E(0,0) * v_sp[0] + E(0,1) * v_sp[1] + E(0,2) * v_sp[2],
				E(1,0) * v_sp[0] + E(1,1) * v_sp[1] + E(1,2) * v_sp[2],
				E(2,0) * v_sp[0] + E(2,1) * v_sp[1] + E(2,2) * v_sp[2],
				E(0,0) * v_rxw[0] + E(0,1) * v_rxw[1] + E(0,2) * v_rxw[2],
				E(1,0) * v_rxw[0] + E(1,1) * v_rxw[1] + E(1,2) * v_rxw[2],
				E(2,0) * v_rxw[0] + E(2,1) * v_rxw[1] + E(2,2) * v_rxw[2]
				);
	}

	SpatialMatrix toMatrix () const {
		Matrix3d _Erx =
			E * Matrix3d (
					0., -r[2], r[1],
					r[2], 0., -r[0],
					-r[1], r[0], 0.
					);
		SpatialMatrix result;
		result.block<3,3>(0,0) = E;
		result.block<3,3>(0,3) = Matrix3d::Zero(3,3);
		result.block<3,3>(3,0) = -_Erx;
		result.block<3,3>(3,3) = E;

		return result;
	}

	SpatialMatrix toMatrixAdjoint () const {
		Matrix3d _Erx =
			E * Matrix3d (
					0., -r[2], r[1],
					r[2], 0., -r[0],
					-r[1], r[0], 0.
					);
		SpatialMatrix result;
		result.block<3,3>(0,0) = E;
		result.block<3,3>(0,3) = -_Erx;
		result.block<3,3>(3,0) = Matrix3d::Zero(3,3);
		result.block<3,3>(3,3) = E;

		return result;
	}

	SpatialMatrix toMatrixTranspose () const {
		Matrix3d _Erx =
			E * Matrix3d (
					0., -r[2], r[1],
					r[2], 0., -r[0],
					-r[1], r[0], 0.
					);
		SpatialMatrix result;
		result.block<3,3>(0,0) = E.transpose();
		result.block<3,3>(0,3) = -_Erx.transpose();
		result.block<3,3>(3,0) = Matrix3d::Zero(3,3);
		result.block<3,3>(3,3) = E.transpose();

		return result;
	}

	SpatialTransform operator* (const SpatialTransform &XT) const {
		return SpatialTransform (E * XT.E, XT.r + XT.E.transpose() * r);
	}

	void operator*= (const SpatialTransform &XT) {
		r = XT.r + XT.E.transpose() * r;
		E *= XT.E;
	}

	Matrix3d E;
	Vector3d r;
};

inline std::ostream& operator<<(std::ostream& output, const SpatialTransform &X) {
	output << X.toMatrix();
	return output;
}


inline SpatialTransform Xrotx (const double &xrot) {
	double s, c;
	s = sin (xrot);
	c = cos (xrot);
	return SpatialTransform (
			Matrix3d (
				1., 0., 0.,
				0., c, s,
				0., -s, c
				),
			Vector3d (0., 0., 0.)
			);
}

inline SpatialTransform Xroty (const double &yrot) {
	double s, c;
	s = sin (yrot);
	c = cos (yrot);
	return SpatialTransform (
			Matrix3d (
				c, 0., -s,
				0., 1., 0.,
				s, 0., c
				),
			Vector3d (0., 0., 0.)
			);
}

inline SpatialTransform Xrotz (const double &zrot) {
	double s, c;
	s = sin (zrot);
	c = cos (zrot);
	return SpatialTransform (
			Matrix3d (
				c, s, 0.,
				-s, c, 0.,
				0., 0., 1.
				),
			Vector3d (0., 0., 0.)
			);
}

inline SpatialTransform Xtrans (const Vector3d &r) {
	return SpatialTransform (
			Matrix3d::Identity(3,3),
			r
			);
}

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

#ifndef RBDL_USE_SIMPLE_MATH
inline EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(SpatialAlgebra::SpatialTransform)
#endif

} /* Math */

} /* RigidBodyDynamics */

/* _SPATIALALGEBRAOPERATORS_H*/
#endif
