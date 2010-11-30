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

// \todo write test 
Matrix3d VectorCrossMatrix (const Vector3d &vector);
// \todo write test 
void SpatialMatrixSetSubmatrix(SpatialAlgebra::SpatialMatrix &dest, unsigned int row, unsigned int col, const Matrix3d &matrix);

bool SpatialMatrixCompareEpsilon (const SpatialAlgebra::SpatialMatrix &matrix_a,
		const SpatialAlgebra::SpatialMatrix &matrix_b, double epsilon);
bool SpatialVectorCompareEpsilon (const SpatialAlgebra::SpatialVector &vector_a,
		const SpatialAlgebra::SpatialVector &vector_b, double epsilon);

SpatialAlgebra::SpatialMatrix Xtrans (const Vector3d &displacement);
SpatialAlgebra::SpatialMatrix Xrotz (const double &zrot);
SpatialAlgebra::SpatialMatrix Xroty (const double &yrot);
SpatialAlgebra::SpatialMatrix Xrotx (const double &xrot);

SpatialAlgebra::SpatialMatrix XtransRotZYXEuler (const Vector3d &displacement, const Vector3d &zyx_euler);

inline std::ostream& operator<<(std::ostream& output, const std::vector<double> &val) {
	int i;
	for (i = 0; i < val.size(); i++)
		output << val.at(i) << " ";

	output << std::endl;

	return output;
}


#endif /* _MATHUTILS_H */
