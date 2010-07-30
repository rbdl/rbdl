/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_ortho_h
#define matrix_ortho_h

#include <cml/mathlib/vector_ortho.h>

/* Functions for orthogonalizing a matrix.
 *
 * matrix_orthogonalize_3x3() and _2x2() operate on the upper-left-hand part
 * of any matrix of suitable size; this is to allow orthonormalization of the
 * rotation part of an affine transform matrix.
 *
 * Note: These functions pass off to the orthonormalization functions in
 * vector_ortho.h, so see that file for details on the optional parameters.
 *
 * @todo: General NxN matrix orthogonalization.
 */

namespace cml {

/** Orthogonalize the upper-left 3x3 portion of a matrix */
template < typename E, class A, class B, class L > void
matrix_orthogonalize_3x3(matrix<E,A,B,L>& m, size_t stable_axis = 2,
    size_t num_iter = 0, E s = E(1))
{
    typedef vector< E, fixed<3> > vector_type;

    vector_type x, y, z;
    matrix_get_basis_vectors(m,x,y,z);
    orthonormalize(x,y,z,stable_axis,num_iter,s);
    matrix_set_basis_vectors(m,x,y,z);
}

/** Orthogonalize the upper-left 2x2 portion of a matrix */
template < typename E, class A, class B, class L > void
matrix_orthogonalize_2x2(matrix<E,A,B,L>& m, size_t stable_axis = 0,
    size_t num_iter = 0, E s = E(1))
{
    typedef vector< E, fixed<2> > vector_type;

    vector_type x, y;
    matrix_get_basis_vectors_2D(m,x,y);
    orthonormalize(x,y,stable_axis,num_iter,s);
    matrix_set_basis_vectors_2D(m,x,y);
}

} // namespace cml

#endif
