/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_misc_h
#define matrix_misc_h

#include <cml/mathlib/checking.h>

/* Miscellaneous matrix functions. */

namespace cml {

/** Set a (possibly non-square) matrix to represent an identity transform */
template < typename E, class A, class B, class L > void
identity_transform(matrix<E,A,B,L>& m)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    for (size_t i = 0; i < m.rows(); ++i) {
        for (size_t j = 0; j < m.cols(); ++j) {
            m(i,j) = value_type((i == j) ? 1 : 0);
        }
    }
}

/** Trace of a square matrix */
template < class MatT > typename MatT::value_type
trace(const MatT& m)
{
    typedef typename MatT::value_type value_type;
    
    /* Checking */
    detail::CheckMatSquare(m);

    value_type t = value_type(0);
    for (size_t i = 0; i < m.rows(); ++i) {
        t += m(i,i);
    }
    return t;
}

/** Trace of the upper-left 3x3 part of a matrix */
template < class MatT > typename MatT::value_type
trace_3x3(const MatT& m)
{
    /* Checking */
    detail::CheckMatMin3x3(m);
    
    return m(0,0) + m(1,1) + m(2,2);
}

/** Trace of the upper-left 2x2 part of a matrix */
template < class MatT > typename MatT::value_type
trace_2x2(const MatT& m)
{
    /* Checking */
    detail::CheckMatMin2x2(m);
    
    return m(0,0) + m(1,1);
}

/** 3D skew-symmetric matrix */
template < typename E, class A, class B, class L, class VecT > void
matrix_skew_symmetric(matrix<E,A,B,L>& m, const VecT& v)
{
    /* Checking */
    detail::CheckMatMin3x3(m);
    detail::CheckVec3(v);

    m.zero();

    m.set_basis_element(1,2, v[0]);
    m.set_basis_element(2,1,-v[0]);
    m.set_basis_element(2,0, v[1]);
    m.set_basis_element(0,2,-v[1]);
    m.set_basis_element(0,1, v[2]);
    m.set_basis_element(1,0,-v[2]);
}

/** 2D skew-symmetric matrix */
template < typename E, class A, class B, class L > void
matrix_skew_symmetric_2D(matrix<E,A,B,L>& m, E s)
{
    /* Checking */
    detail::CheckMatMin2x2(m);

    m.zero();

    m.set_basis_element(0,1, s);
    m.set_basis_element(1,0,-s);
}

/* @todo: Clean this up, and implement SRT as well */

/** Invert a matrix consisting of a 3D rotation and translation */
template < typename E, class A, class B, class L > void
matrix_invert_RT_only(matrix<E,A,B,L>& m)
{
    typedef vector< E, fixed<3> > vector_type;

    vector_type x, y, z;
    matrix_get_basis_vectors(m,x,y,z);
    matrix_set_transposed_basis_vectors(m,x,y,z);
    
    vector_type p = matrix_get_translation(m);
    matrix_set_translation(m,-dot(p,x),-dot(p,y),-dot(p,z));
}

/** Invert a matrix consisting of a 2D rotation and ranslation */
template < typename E, class A, class B, class L > void
matrix_invert_RT_only_2D(matrix<E,A,B,L>& m)
{
    typedef vector< E, fixed<2> > vector_type;

    vector_type x, y;
    matrix_get_basis_vectors_2D(m,x,y);
    matrix_set_transposed_basis_vectors_2D(m,x,y);
    
    vector_type p = matrix_get_translation_2D(m);
    matrix_set_translation_2D(m,-dot(p,x),-dot(p,y));
}

} // namespace cml

#endif
