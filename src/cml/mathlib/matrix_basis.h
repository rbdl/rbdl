/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_basis_h
#define matrix_basis_h

#include <cml/mathlib/checking.h>

/* This file contains functions for setting and retrieving the basis vectors
 * or transposed basis vectors of a matrix representing a 3D or 2D transform,
 * either by index (0,1,2) or name (x,y,z).
 *
 * In addition to being a convenience for the user, the functions are also
 * in support of other matrix functions which are best implemented in vector
 * form (such as orthogonalization and construction of orthonormal bases).
 *
 * Note that matrix expression arguments are allowed to have dimensions larger
 * than the minimum requirement. For example, matrix_get_basis_vector() can be
 * called on any NxM matrix with N,M >= 3.
 *
 * As with other matrix functions, the following template argument notation is
 * used for conciseness:
 *
 * E = vector or matrix element type
 * A = vector or matrix array storage type
 * B = matrix basis orientation type
 * L = matrix layout type
 */

namespace cml {

//////////////////////////////////////////////////////////////////////////////
// Functions for setting the basis vectors of a 3D or 2D transform matrix
//////////////////////////////////////////////////////////////////////////////

/** Set the i'th basis vector of a 3D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_basis_vector(matrix<E,A,B,L>& m, size_t i, const VecT& v)
{
    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckVec3(v);
    detail::CheckIndex3(i);

    m.set_basis_element(i,0,v[0]);
    m.set_basis_element(i,1,v[1]);
    m.set_basis_element(i,2,v[2]);
}

/** Set the i'th transposed basis vector of a 3D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_transposed_basis_vector(matrix<E,A,B,L>& m,size_t i,const VecT& v)
{
    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckVec3(v);
    detail::CheckIndex3(i);

    m.set_basis_element(0,i,v[0]);
    m.set_basis_element(1,i,v[1]);
    m.set_basis_element(2,i,v[2]);
}

/** Set the i'th basis vector of a 2D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_basis_vector_2D(matrix<E,A,B,L>& m, size_t i, const VecT& v)
{
    /* Checking */
    detail::CheckMatLinear2D(m);
    detail::CheckVec2(v);
    detail::CheckIndex2(i);

    m.set_basis_element(i,0,v[0]);
    m.set_basis_element(i,1,v[1]);
}

/** Set the i'th transposed basis vector of a 2D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_transposed_basis_vector_2D(
    matrix<E,A,B,L>& m, size_t i, const VecT& v)
{
    /* Checking */
    detail::CheckMatLinear2D(m);
    detail::CheckVec2(v);
    detail::CheckIndex2(i);

    m.set_basis_element(0,i,v[0]);
    m.set_basis_element(1,i,v[1]);
}

/** Set the x basis vector of a 3D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_x_basis_vector(matrix<E,A,B,L>& m, const VecT& x) {
    matrix_set_basis_vector(m,0,x);
}

/** Set the y basis vector of a 3D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_y_basis_vector(matrix<E,A,B,L>& m, const VecT& y) {
    matrix_set_basis_vector(m,1,y);
}

/** Set the z basis vector of a 3D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_z_basis_vector(matrix<E,A,B,L>& m, const VecT& z) {
    matrix_set_basis_vector(m,2,z);
}

/** Set the transposed x basis vector of a 3D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_transposed_x_basis_vector(matrix<E,A,B,L>& m, const VecT& x) {
    matrix_set_transposed_basis_vector(m,0,x);
}

/** Set the transposed y basis vector of a 3D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_transposed_y_basis_vector(matrix<E,A,B,L>& m, const VecT& y) {
    matrix_set_transposed_basis_vector(m,1,y);
}

/** Set the transposed z basis vector of a 3D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_transposed_z_basis_vector(matrix<E,A,B,L>& m, const VecT& z) {
    matrix_set_transposed_basis_vector(m,2,z);
}

/** Set the x basis vector of a 2D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_x_basis_vector_2D(matrix<E,A,B,L>& m, const VecT& x) {
    matrix_set_basis_vector_2D(m,0,x);
}

/** Set the y basis vector of a 2D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_y_basis_vector_2D(matrix<E,A,B,L>& m, const VecT& y) {
    matrix_set_basis_vector_2D(m,1,y);
}

/** Set the transposed x basis vector of a 2D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_transposed_x_basis_vector_2D(matrix<E,A,B,L>& m,const VecT& x) {
    matrix_set_transposed_basis_vector_2D(m,0,x);
}

/** Set the transposed y basis vector of a 2D transform */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_transposed_y_basis_vector_2D(matrix<E,A,B,L>& m,const VecT& y) {
    matrix_set_transposed_basis_vector_2D(m,1,y);
}

/** Set the basis vectors of a 3D transform */
template < typename E, class A, class B, class L,
    class VecT_1, class VecT_2, class VecT_3 > void
matrix_set_basis_vectors(
    matrix<E,A,B,L>& m, const VecT_1& x, const VecT_2& y, const VecT_3& z)
{
    matrix_set_x_basis_vector(m,x);
    matrix_set_y_basis_vector(m,y);
    matrix_set_z_basis_vector(m,z);
}

/** Set the transposed basis vectors of a 3D transform */
template < typename E, class A, class B, class L,
    class VecT_1, class VecT_2, class VecT_3 > void
matrix_set_transposed_basis_vectors(
    matrix<E,A,B,L>& m, const VecT_1& x, const VecT_2& y, const VecT_3& z)
{
    matrix_set_transposed_x_basis_vector(m,x);
    matrix_set_transposed_y_basis_vector(m,y);
    matrix_set_transposed_z_basis_vector(m,z);
}

/** Set the basis vectors of a 2D transform */
template < typename E,class A,class B,class L,class VecT_1,class VecT_2 > void
matrix_set_basis_vectors_2D(
    matrix<E,A,B,L>& m, const VecT_1& x, const VecT_2& y)
{
    matrix_set_x_basis_vector_2D(m,x);
    matrix_set_y_basis_vector_2D(m,y);
}

/** Set the transposed basis vectors of a 2D transform */
template < typename E,class A,class B,class L,class VecT_1,class VecT_2 > void
matrix_set_transposed_basis_vectors_2D(
    matrix<E,A,B,L>& m, const VecT_1& x, const VecT_2& y)
{
    matrix_set_transposed_x_basis_vector_2D(m,x);
    matrix_set_transposed_y_basis_vector_2D(m,y);
}

//////////////////////////////////////////////////////////////////////////////
// Functions for getting the basis vectors of a 3D or 2D transform matrix
//////////////////////////////////////////////////////////////////////////////

#define TEMP_VEC3 vector< typename MatT::value_type, fixed<3> >
#define TEMP_VEC2 vector< typename MatT::value_type, fixed<2> >

/** Get the i'th basis vector of a 3D transform */
template < class MatT > TEMP_VEC3
matrix_get_basis_vector(const MatT& m, size_t i)
{
    typedef TEMP_VEC3 vector_type;

    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckIndex3(i);

    return vector_type(
        m.basis_element(i,0), m.basis_element(i,1), m.basis_element(i,2));
}

/** Get the i'th transposed basis vector of a 3D transform */
template < class MatT > TEMP_VEC3
matrix_get_transposed_basis_vector(const MatT& m, size_t i)
{
    typedef typename MatT::value_type value_type;
    typedef vector< value_type, fixed<3> > vector_type;
    
    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckIndex3(i);

    return vector_type(
        m.basis_element(0,i), m.basis_element(1,i), m.basis_element(2,i));
}

/** Get the i'th basis vector of a 2D transform */
template < class MatT > TEMP_VEC2
matrix_get_basis_vector_2D(const MatT& m, size_t i)
{
    typedef TEMP_VEC2 vector_type;

    /* Checking */
    detail::CheckMatLinear2D(m);
    detail::CheckIndex2(i);

    return vector_type(m.basis_element(i,0), m.basis_element(i,1));
}

/** Get the i'th transposed basis vector of a 2D transform */
template < class MatT > TEMP_VEC2
matrix_get_transposed_basis_vector_2D(const MatT& m, size_t i)
{
    typedef TEMP_VEC2 vector_type;

    /* Checking */
    detail::CheckMatLinear2D(m);
    detail::CheckIndex2(i);

    return vector_type(m.basis_element(0,i), m.basis_element(1,i));
}

/** Get the x basis vector of a 3D transform */
template < class MatT > TEMP_VEC3
matrix_get_x_basis_vector(const MatT& m) {
    return matrix_get_basis_vector(m,0);
}

/** Get the y basis vector of a 3D transform */
template < class MatT > TEMP_VEC3
matrix_get_y_basis_vector(const MatT& m) {
    return matrix_get_basis_vector(m,1);
}

/** Get the z basis vector of a 3D transform */
template < class MatT > TEMP_VEC3
matrix_get_z_basis_vector(const MatT& m) {
    return matrix_get_basis_vector(m,2);
}

/** Get the transposed x basis vector of a 3D transform */
template < class MatT > TEMP_VEC3
matrix_get_transposed_x_basis_vector(const MatT& m) {
    return matrix_get_transposed_basis_vector(m,0);
}

/** Get the transposed y basis vector of a 3D transform */
template < class MatT > TEMP_VEC3
matrix_get_transposed_y_basis_vector(const MatT& m) {
    return matrix_get_transposed_basis_vector(m,1);
}

/** Get the transposed z basis vector of a 3D transform */
template < class MatT > TEMP_VEC3
matrix_get_transposed_z_basis_vector(const MatT& m) {
    return matrix_get_transposed_basis_vector(m,2);
}

/** Get the x basis vector of a 2D transform */
template < class MatT > TEMP_VEC2
matrix_get_x_basis_vector_2D(const MatT& m) {
    return matrix_get_basis_vector_2D(m,0);
}

/** Get the y basis vector of a 2D transform */
template < class MatT > TEMP_VEC2
matrix_get_y_basis_vector_2D(const MatT& m) {
    return matrix_get_basis_vector_2D(m,1);
}

/** Get the transposed x basis vector of a 2D transform */
template < class MatT > TEMP_VEC2
matrix_get_transposed_x_basis_vector_2D(const MatT& m) {
    return matrix_get_transposed_basis_vector_2D(m,0);
}

/** Get the transposed y basis vector of a 2D transform */
template < class MatT > TEMP_VEC2
matrix_get_transposed_y_basis_vector_2D(const MatT& m) {
    return matrix_get_transposed_basis_vector_2D(m,1);
}

/** Get the basis vectors of a 3D transform */
template < class MatT, class E, class A > void
matrix_get_basis_vectors(
    const MatT& m, vector<E,A>& x, vector<E,A>& y, vector<E,A>& z)
{
    x = matrix_get_x_basis_vector(m);
    y = matrix_get_y_basis_vector(m);
    z = matrix_get_z_basis_vector(m);
}

/** Get the transposed basis vectors of a 3D transform */
template < class MatT, typename E, class A > void
matrix_get_transposed_basis_vectors(
    const MatT& m, vector<E,A>& x, vector<E,A>& y, vector<E,A>& z)
{
    x = matrix_get_transposed_x_basis_vector(m);
    y = matrix_get_transposed_y_basis_vector(m);
    z = matrix_get_transposed_z_basis_vector(m);
}

/** Get the basis vectors of a 2D transform */
template < class MatT, typename E, class A > void
matrix_get_basis_vectors_2D(const MatT& m,vector<E,A>& x,vector<E,A>& y)
{
    x = matrix_get_x_basis_vector_2D(m);
    y = matrix_get_y_basis_vector_2D(m);
}

/** Get the transposed basis vectors of a 2D transform */
template < class MatT, typename E, class A > void
matrix_get_transposed_basis_vectors_2D(
    const MatT& m, vector<E,A>& x, vector<E,A>& y)
{
    x = matrix_get_transposed_x_basis_vector_2D(m);
    y = matrix_get_transposed_y_basis_vector_2D(m);
}

#undef TEMP_VEC3
#undef TEMP_VEC2

} // namespace cml

#endif
