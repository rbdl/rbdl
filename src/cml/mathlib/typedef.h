/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef typedef_h
#define typedef_h

#include <cml/vector.h>
#include <cml/matrix.h>
#include <cml/quaternion.h>
#include <cml/constants.h>
#include <cml/mathlib/epsilon.h>

namespace cml {

/* fixed-size vectors */
typedef vector< int,    fixed<2> > vector2i;
typedef vector< float,  fixed<2> > vector2f;
typedef vector< double, fixed<2> > vector2d;

typedef vector< int,    fixed<3> > vector3i;
typedef vector< float,  fixed<3> > vector3f;
typedef vector< double, fixed<3> > vector3d;

typedef vector< int,    fixed<4> > vector4i;
typedef vector< float,  fixed<4> > vector4f;
typedef vector< double, fixed<4> > vector4d;

/* fixed-size matrices */

typedef matrix< int,    fixed<2,2> > matrix22i;
typedef matrix< float,  fixed<2,2> > matrix22f;
typedef matrix< double, fixed<2,2> > matrix22d;

typedef matrix< int,    fixed<2,2>, row_basis, row_major > matrix22i_r;
typedef matrix< int,    fixed<2,2>, col_basis, col_major > matrix22i_c;
typedef matrix< float,  fixed<2,2>, row_basis, row_major > matrix22f_r;
typedef matrix< float,  fixed<2,2>, col_basis, col_major > matrix22f_c;
typedef matrix< double, fixed<2,2>, row_basis, row_major > matrix22d_r;
typedef matrix< double, fixed<2,2>, col_basis, col_major > matrix22d_c;


typedef matrix< int,    fixed<3,3> > matrix33i;
typedef matrix< float,  fixed<3,3> > matrix33f;
typedef matrix< double, fixed<3,3> > matrix33d;

typedef matrix< int,    fixed<3,3>, row_basis, row_major > matrix33i_r;
typedef matrix< int,    fixed<3,3>, col_basis, col_major > matrix33i_c;
typedef matrix< float,  fixed<3,3>, row_basis, row_major > matrix33f_r;
typedef matrix< float,  fixed<3,3>, col_basis, col_major > matrix33f_c;
typedef matrix< double, fixed<3,3>, row_basis, row_major > matrix33d_r;
typedef matrix< double, fixed<3,3>, col_basis, col_major > matrix33d_c;


typedef matrix< int,    fixed<4,4> > matrix44i;
typedef matrix< float,  fixed<4,4> > matrix44f;
typedef matrix< double, fixed<4,4> > matrix44d;

typedef matrix< int,    fixed<4,4>, row_basis, row_major > matrix44i_r;
typedef matrix< int,    fixed<4,4>, col_basis, col_major > matrix44i_c;
typedef matrix< float,  fixed<4,4>, row_basis, row_major > matrix44f_r;
typedef matrix< float,  fixed<4,4>, col_basis, col_major > matrix44f_c;
typedef matrix< double, fixed<4,4>, row_basis, row_major > matrix44d_r;
typedef matrix< double, fixed<4,4>, col_basis, col_major > matrix44d_c;


typedef matrix< int,    fixed<3,2>, row_basis, row_major > matrix32i_r;
typedef matrix< float,  fixed<3,2>, row_basis, row_major > matrix32f_r;
typedef matrix< double, fixed<3,2>, row_basis, row_major > matrix32d_r;

typedef matrix< int,    fixed<2,3>, col_basis, col_major > matrix23i_c;
typedef matrix< float,  fixed<2,3>, col_basis, col_major > matrix23f_c;
typedef matrix< double, fixed<2,3>, col_basis, col_major > matrix23d_c;

typedef matrix< int,    fixed<4,3>, row_basis, row_major > matrix43i_r;
typedef matrix< float,  fixed<4,3>, row_basis, row_major > matrix43f_r;
typedef matrix< double, fixed<4,3>, row_basis, row_major > matrix43d_r;

typedef matrix< int,    fixed<3,4>, col_basis, col_major > matrix34i_c;
typedef matrix< float,  fixed<3,4>, col_basis, col_major > matrix34f_c;
typedef matrix< double, fixed<3,4>, col_basis, col_major > matrix34d_c;


/* quaternions */
typedef quaternion<float, fixed<>,vector_first,negative_cross>
    quaternionf_n;
typedef quaternion<float, fixed<>,vector_first,positive_cross>
    quaternionf_p;
typedef quaternion<double,fixed<>,vector_first,negative_cross>
    quaterniond_n;
typedef quaternion<double,fixed<>,vector_first,positive_cross>
    quaterniond_p;
typedef quaternion<float> quaternionf;
typedef quaternion<double> quaterniond;


/* dynamically resizable vectors */
typedef vector< int,    dynamic<> > vectori;
typedef vector< float,  dynamic<> > vectorf;
typedef vector< double, dynamic<> > vectord;


/* dynamically resizable matrices */
typedef matrix< int,    dynamic<> > matrixi;
typedef matrix< float,  dynamic<> > matrixf;
typedef matrix< double, dynamic<> > matrixd;

typedef matrix< int,    dynamic<>, row_basis, row_major > matrixi_r;
typedef matrix< int,    dynamic<>, col_basis, col_major > matrixi_c;
typedef matrix< float,  dynamic<>, row_basis, row_major > matrixf_r;
typedef matrix< float,  dynamic<>, col_basis, col_major > matrixf_c;
typedef matrix< double, dynamic<>, row_basis, row_major > matrixd_r;
typedef matrix< double, dynamic<>, col_basis, col_major > matrixd_c;


/* constants */
typedef constants<float>  constantsf;
typedef constants<double> constantsd;

/* epsilon/tolerance values (placeholder) */
typedef epsilon<float>  epsilonf;
typedef epsilon<double> epsilond;

} // namespace cml

#endif
