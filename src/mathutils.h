#ifndef _MATHUTILS_H
#define _MATHUTILS_H

#include <assert.h>
#include <cmath>

#include "cmlwrapper.h"

extern Vector3d Vector3dZero;
extern Matrix3d Matrix3dIdentity;
extern Matrix3d Matrix3dZero;

extern SpatialMatrix SpatialMatrixIdentity;

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
void SpatialMatrixSetSubmatrix(SpatialMatrix &dest, unsigned int row, unsigned int col, const Matrix3d &matrix);

bool SpatialMatrixCompareEpsilon (const SpatialMatrix &matrix_a, const SpatialMatrix &matrix_b, double epsilon);

#endif /* _MATHUTILS_H */
