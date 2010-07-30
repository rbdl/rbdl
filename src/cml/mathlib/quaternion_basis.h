/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef quaternion_basis_h
#define quaternion_basis_h

#include <cml/mathlib/checking.h>

/* Functions for getting the basis vectors of a quaternion rotation. */

namespace cml {

/** Get the i'th basis vector of a quaternion rotation */
template < class QuatT > vector< typename QuatT::value_type, fixed<3> >
quaternion_get_basis_vector(const QuatT& q, size_t i)
{
    typedef QuatT quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef typename quaternion_type::order_type order_type;
    typedef vector< value_type, fixed<3> > vector_type;

    /* Checking */
    detail::CheckQuat(q);
    detail::CheckIndex3(i);

    size_t j, k;
    cyclic_permutation(i, i, j, k);
    
    /* @todo: Clean this up. */
    const size_t W = order_type::W;
    const size_t I = order_type::X + i;
    const size_t J = order_type::X + j;
    const size_t K = order_type::X + k;
    
    value_type j2 = q[J] + q[J];
    value_type k2 = q[K] + q[K];
    
    /* @todo: use set_permuted() for the following when available. */

    vector_type result;
    result[i] = value_type(1) - q[J] * j2 - q[K] * k2;
    result[j] = q[I] * j2 + q[W] * k2;
    result[k] = q[I] * k2 - q[W] * j2;
    return result;
}

/** Get the x basis vector of a quaternion rotation */
template < class QuatT > vector< typename QuatT::value_type, fixed<3> >
quaternion_get_x_basis_vector(const QuatT& q) {
    return quaternion_get_basis_vector(q,0);
}

/** Get the y basis vector of a quaternion rotation */
template < class QuatT > vector< typename QuatT::value_type, fixed<3> >
quaternion_get_y_basis_vector(const QuatT& q) {
    return quaternion_get_basis_vector(q,1);
}

/** Get the z basis vector of a quaternion rotation */
template < class QuatT > vector< typename QuatT::value_type, fixed<3> >
quaternion_get_z_basis_vector(const QuatT& q) {
    return quaternion_get_basis_vector(q,2);
}

/** Get the basis vectors of a quaternion rotation */
template < class QuatT, typename E, class A > void
quaternion_get_basis_vectors(
    const QuatT& q,
    vector<E,A>& x,
    vector<E,A>& y,
    vector<E,A>& z)
{
    x = quaternion_get_x_basis_vector(q);
    y = quaternion_get_y_basis_vector(q);
    z = quaternion_get_z_basis_vector(q);
}

} // namespace cml

#endif
