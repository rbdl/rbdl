#ifndef _MATHUTILS_H
#define _MATHUTILS_H

#include <assert.h>
#include <cmath>

#include "cmlwrapper.h"
#include "spatialalgebra.h"

extern Vector3d Vector3dZero;
extern Matrix3d Matrix3dIdentity;
extern Matrix3d Matrix3dZero;

extern SpatialAlgebra::SpatialMatrix SpatialMatrixIdentity;

void VectorCrossVector (Vector3d &result, const Vector3d &vec_a, const Vector3d &vec_b);
void VectorPrint (const char* msg, const Vector3d &vector);

void MatrixPrint (const char* msg, const Matrix3d &matrix);
void MatrixSetIdentity (Matrix3d &result);
void MatrixSetZero (Matrix3d &result);
void MatrixCopyTranspose (Matrix3d &result, const Matrix3d &src);
void MatrixCopy (Matrix3d &result, const Matrix3d &src);

/** \brief Solves a linear system using gaussian elimination
 */
bool LinSolveGaussElim (cmlMatrix A, cmlVector b, cmlVector &x);

/** \brief Creates the matrix to the cross product of a given 3D vector
 *  
 * \todo write test 
 */
Matrix3d VectorCrossMatrix (const Vector3d &vector);
// \todo write test 
void SpatialMatrixSetSubmatrix(SpatialAlgebra::SpatialMatrix &dest, unsigned int row, unsigned int col, const Matrix3d &matrix);

bool SpatialMatrixCompareEpsilon (const SpatialAlgebra::SpatialMatrix &matrix_a,
		const SpatialAlgebra::SpatialMatrix &matrix_b, double epsilon);
bool SpatialVectorCompareEpsilon (const SpatialAlgebra::SpatialVector &vector_a,
		const SpatialAlgebra::SpatialVector &vector_b, double epsilon);

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
SpatialAlgebra::SpatialMatrix Xtrans (const Vector3d &displacement);

/** \brief Creates a rotational transformation around the Z-axis
 *
 * Creates a rotation around the current Z-axis by the given angle
 * (specified in radians).
 *
 * \param zrot Rotation angle in radians.
 */
SpatialAlgebra::SpatialMatrix Xrotz (const double &zrot);

/** \brief Creates a rotational transformation around the Y-axis
 *
 * Creates a rotation around the current Y-axis by the given angle
 * (specified in radians).
 *
 * \param yrot Rotation angle in radians.
 */
SpatialAlgebra::SpatialMatrix Xroty (const double &yrot);

/** \brief Creates a rotational transformation around the X-axis
 *
 * Creates a rotation around the current X-axis by the given angle
 * (specified in radians).
 *
 * \param xrot Rotation angle in radians.
 */
SpatialAlgebra::SpatialMatrix Xrotx (const double &xrot);

/** \brief Creates a spatial transformation for given parameters 
 *
 * Creates a transformation to a coordinate system that is first rotated
 * and then translated.
 *
 * \param displacement The displacement to the new origin
 * \param zyx_euler The orientation of the new coordinate system, specifyed
 * by ZYX-Euler angles.
 */
SpatialAlgebra::SpatialMatrix XtransRotZYXEuler (const Vector3d &displacement, const Vector3d &zyx_euler);

inline std::ostream& operator<<(std::ostream& output, const std::vector<double> &val) {
	int i;
	for (i = 0; i < val.size(); i++)
		output << val.at(i) << " ";

	output << std::endl;

	return output;
}

#endif /* _MATHUTILS_H */
