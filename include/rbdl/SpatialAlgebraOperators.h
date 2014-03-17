/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _SPATIALALGEBRAOPERATORS_H
#define _SPATIALALGEBRAOPERATORS_H

#include <iostream>
#include <cmath>

namespace RigidBodyDynamics {

namespace Math {

inline Matrix3d VectorCrossMatrix (const Vector3d &vector) {
	return Matrix3d (
			0., -vector[2], vector[1],
			vector[2], 0., -vector[0],
			-vector[1], vector[0], 0.
			);
}

/** \brief Spatial algebra matrices, vectors, and operators. */

struct RBDL_DLLAPI SpatialRigidBodyInertia {
	SpatialRigidBodyInertia() :
		m (0.),
		h (Vector3d::Zero(3,1)),
		I (Matrix3d::Zero(3,3))
	{}
	SpatialRigidBodyInertia (
			double mass, const Vector3d &com_mass, const Matrix3d &inertia) : 
		m (mass), h (com_mass), I (inertia)
	{ }

	SpatialVector operator* (const SpatialVector &mv) {
		Vector3d mv_upper (mv[0], mv[1], mv[2]);
		Vector3d mv_lower (mv[3], mv[4], mv[5]);

		Vector3d res_upper = I * Vector3d (mv[0], mv[1], mv[2]) + h.cross(mv_lower);
		Vector3d res_lower = m * mv_lower - h.cross (mv_upper);
			
		return SpatialVector (
				res_upper[0], res_upper[1], res_upper[2],
				res_lower[0], res_lower[1], res_lower[2]
				);
	}

	SpatialRigidBodyInertia operator+ (const SpatialRigidBodyInertia &rbi) {
		return SpatialRigidBodyInertia (
				m + rbi.m,
				h + rbi.h,
				I + rbi.I
				);
	}

	void createFromMatrix (const SpatialMatrix &Ic) {
		m = Ic(3,3);
		h.set (-Ic(1,5), Ic(0,5), -Ic(0,4));
		I = Ic.block<3,3>(0,0);
	}

	SpatialMatrix toMatrix() const {
		SpatialMatrix result;
		result.block<3,3>(0,0) = I;
		result.block<3,3>(0,3) = VectorCrossMatrix(h);
		result.block<3,3>(3,0) = - VectorCrossMatrix(h);
		result.block<3,3>(3,3) = Matrix3d::Identity(3,3) * m;

		return result;
	}

	/// Mass
	double m;
	/// Coordinates of the center of mass
	Vector3d h;
	/// Inertia expressed at the origin
	Matrix3d I;
};

/** \brief Compact representation of spatial transformations.
 *
 * Instead of using a verbose 6x6 matrix, this structure only stores a 3x3
 * matrix and a 3-d vector to store spatial transformations. It also
 * encapsulates efficient operations such as concatenations and
 * transformation of spatial vectors.
 */
struct RBDL_DLLAPI SpatialTransform {
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
	 * \returns (E * w, - E * rxw + E * v)
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

	/** Same as X^T * f.
	 *
	 * \returns (E^T * n + rx * E^T * f, E^T * f)
	 */
	SpatialVector applyTranspose (const SpatialVector &f_sp) {
		Vector3d E_T_f (
				E(0,0) * f_sp[3] + E(1,0) * f_sp[4] + E(2,0) * f_sp[5],
				E(0,1) * f_sp[3] + E(1,1) * f_sp[4] + E(2,1) * f_sp[5],
				E(0,2) * f_sp[3] + E(1,2) * f_sp[4] + E(2,2) * f_sp[5]
				);

		return SpatialVector (
				E(0,0) * f_sp[0] + E(1,0) * f_sp[1] + E(2,0) * f_sp[2] - r[2] * E_T_f[1] + r[1] * E_T_f[2],
				E(0,1) * f_sp[0] + E(1,1) * f_sp[1] + E(2,1) * f_sp[2] + r[2] * E_T_f[0] - r[0] * E_T_f[2],
				E(0,2) * f_sp[0] + E(1,2) * f_sp[1] + E(2,2) * f_sp[2] - r[1] * E_T_f[0] + r[0] * E_T_f[1],
				E_T_f [0],
				E_T_f [1],
				E_T_f [2]
				);
	}

	/** Same as X^* I X^{-1}
	 */
	SpatialRigidBodyInertia apply (const SpatialRigidBodyInertia &rbi) {
		return SpatialRigidBodyInertia (
				rbi.m,
				E * (rbi.h - rbi.m * r),
				E * 
					( 
						rbi.I + VectorCrossMatrix (r) * VectorCrossMatrix (rbi.h)
						+ (VectorCrossMatrix(rbi.h - rbi.m * r) * VectorCrossMatrix (r))
					)
				* E.transpose()
			);
	}

	/** Same as X^T I X
	 */
	SpatialRigidBodyInertia applyTranspose (const SpatialRigidBodyInertia &rbi) {
		Vector3d E_T_mr = E.transpose() * rbi.h + rbi.m * r;
		return SpatialRigidBodyInertia (
				rbi.m,
				E_T_mr,
				E.transpose() * rbi.I * E 
					- VectorCrossMatrix(r) * VectorCrossMatrix (E.transpose() * rbi.h)  
					- VectorCrossMatrix (E_T_mr) * VectorCrossMatrix (r));
	}

	SpatialVector applyAdjoint (const SpatialVector &f_sp) {
		Vector3d En_rxf = E * (Vector3d (f_sp[0], f_sp[1], f_sp[2]) - r.cross(Vector3d (f_sp[3], f_sp[4], f_sp[5])));
//		Vector3d En_rxf = E * (Vector3d (f_sp[0], f_sp[1], f_sp[2]) - r.cross(Eigen::Map<Vector3d> (&(f_sp[3]))));

		return SpatialVector (
				En_rxf[0],
				En_rxf[1],
				En_rxf[2],
				E(0,0) * f_sp[3] + E(0,1) * f_sp[4] + E(0,2) * f_sp[5],
				E(1,0) * f_sp[3] + E(1,1) * f_sp[4] + E(1,2) * f_sp[5],
				E(2,0) * f_sp[3] + E(2,1) * f_sp[4] + E(2,2) * f_sp[5]
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

	SpatialTransform inverse() const {
		return SpatialTransform (
				E.transpose(),
				- E * r
				);
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

inline std::ostream& operator<<(std::ostream& output, const SpatialRigidBodyInertia &rbi) {
	output << "rbi.m = " << rbi.m << std::endl;
	output << "rbi.h = " << rbi.h.transpose();
	output << "rbi.I = " << std::endl << rbi.I << std::endl;
	return output;
}

inline std::ostream& operator<<(std::ostream& output, const SpatialTransform &X) {
	output << "X.E = " << std::endl << X.E << std::endl;
	output << "X.r = " << X.r.transpose();
	return output;
}

inline SpatialTransform Xrot (double angle_rad, const Vector3d &axis) {
	double s, c;
	s = sin(angle_rad);
	c = cos(angle_rad);

	return SpatialTransform (
			Matrix3d (
				axis[0] * axis[0] * (1.0f - c) + c,
				axis[1] * axis[0] * (1.0f - c) + axis[2] * s,
				axis[0] * axis[2] * (1.0f - c) - axis[1] * s,

				axis[0] * axis[1] * (1.0f - c) - axis[2] * s,
				axis[1] * axis[1] * (1.0f - c) + c,
				axis[1] * axis[2] * (1.0f - c) + axis[0] * s,

				axis[0] * axis[2] * (1.0f - c) + axis[1] * s,
				axis[1] * axis[2] * (1.0f - c) - axis[0] * s,
				axis[2] * axis[2] * (1.0f - c) + c

				),
			Vector3d (0., 0., 0.)
			);
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

inline Matrix3d get_rotation (const SpatialMatrix &m) {
	return m.block<3,3>(0,0);
}

inline Vector3d get_translation (const SpatialMatrix &m) {
	return Vector3d (-m(4,2), m(3,2), -m(3,1));
}

} /* Math */

} /* RigidBodyDynamics */

/* _SPATIALALGEBRAOPERATORS_H*/
#endif
