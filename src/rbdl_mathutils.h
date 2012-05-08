/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MATHUTILS_H
#define _MATHUTILS_H

#include <assert.h>
#include <cmath>

#include "rbdl_math.h"

namespace RigidBodyDynamics {

namespace Math {

/** \brief Available solver methods for the linear systems.
 *
 * Please note that these methods are only available when Eigen3 is used.
 * When the math library SimpleMath is used it will always use a slow
 * column pivoting gauss elimination.
 */
enum LinearSolver {
	LinearSolverUnknown = 0,
	LinearSolverPartialPivLU,
	LinearSolverColPivHouseholderQR,
	LinearSolverLast,
};

extern Vector3d Vector3dZero;
extern Matrix3d Matrix3dIdentity;
extern Matrix3d Matrix3dZero;

extern SpatialVector SpatialVectorZero;
extern SpatialMatrix SpatialMatrixIdentity;
extern SpatialMatrix SpatialMatrixZero;

/// \brief Solves a linear system using gaussian elimination with pivoting
bool LinSolveGaussElimPivot (MatrixNd A, VectorNd b, VectorNd &x);

/// \brief Creates the skew symmetric matrix of the cross product of a given 3D vector
inline Matrix3d VectorCrossMatrix (const Vector3d &vector) {
	return Matrix3d (
			0., -vector[2], vector[1],
			vector[2], 0., -vector[0],
			-vector[1], vector[0], 0.
			);
}

// \todo write test 
void SpatialMatrixSetSubmatrix(SpatialMatrix &dest, unsigned int row, unsigned int col, const Matrix3d &matrix);

bool SpatialMatrixCompareEpsilon (const SpatialMatrix &matrix_a,
		const SpatialMatrix &matrix_b, double epsilon);
bool SpatialVectorCompareEpsilon (const SpatialVector &vector_a,
		const SpatialVector &vector_b, double epsilon);

/** \brief Translates the inertia matrix to a new center. */
Matrix3d parallel_axis (const Matrix3d &inertia, double mass, const Vector3d &com);

/** \brief Creates a transformation of a linear displacement
 *
 * This can be used to specify the translation to the joint center when
 * adding a body to a model. See also section 2.8 in RBDA.
 *
 * \note The transformation returned is for motions. For a transformation for forces
 * \note one has to conjugate the matrix.
 *
 * \param displacement The displacement as a 3D vector
 */
SpatialMatrix Xtrans_mat (const Vector3d &displacement);

/** \brief Creates a rotational transformation around the Z-axis
 *
 * Creates a rotation around the current Z-axis by the given angle
 * (specified in radians).
 *
 * \param zrot Rotation angle in radians.
 */
SpatialMatrix Xrotz_mat (const double &zrot);

/** \brief Creates a rotational transformation around the Y-axis
 *
 * Creates a rotation around the current Y-axis by the given angle
 * (specified in radians).
 *
 * \param yrot Rotation angle in radians.
 */
SpatialMatrix Xroty_mat (const double &yrot);

/** \brief Creates a rotational transformation around the X-axis
 *
 * Creates a rotation around the current X-axis by the given angle
 * (specified in radians).
 *
 * \param xrot Rotation angle in radians.
 */
SpatialMatrix Xrotx_mat (const double &xrot);

/** \brief Creates a spatial transformation for given parameters 
 *
 * Creates a transformation to a coordinate system that is first rotated
 * and then translated.
 *
 * \param displacement The displacement to the new origin
 * \param zyx_euler The orientation of the new coordinate system, specifyed
 * by ZYX-Euler angles.
 */
SpatialMatrix XtransRotZYXEuler (const Vector3d &displacement, const Vector3d &zyx_euler);

} /* Math */

} /* RigidBodyDynamics */
#endif /* _MATHUTILS_H */
