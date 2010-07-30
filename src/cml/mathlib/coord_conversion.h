/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef coord_conversion_h
#define coord_conversion_h

#include <cml/mathlib/checking.h>
#include <cml/mathlib/epsilon.h>
#include <cml/mathlib/helper.h>

/* Functions for converting between Cartesian, polar, cylindrical and
 * spherical coordinates.
 *
 * The 3D conversion functions take an integer axis index argument. For
 * cylindrical coordinates this determines the axis of the cylinder, and for
 * spherical it determines which cardinal axis is normal to the azimuth plane.
 *
 * For spherical coordinates the option of whether to treat phi as latitude
 * or colatitude is also available. The 'type' argument takes either of the
 * enumerants cml::latitude and cml::colatitude to reflect this.
 */

namespace cml {

//////////////////////////////////////////////////////////////////////////////
// Conversion to Cartesian coordinates
//////////////////////////////////////////////////////////////////////////////

/* Convert cylindrical coordinates to Cartesian coordinates in R3 */
template < typename E, class A > void
cylindrical_to_cartesian(
    E radius, E theta, E height, size_t axis, vector<E,A>& v)
{
    typedef vector<E,A> vector_type;
    typedef typename vector_type::value_type value_type;

    /* Checking */
    detail::CheckVec3(v);
    detail::CheckIndex3(axis);

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);
    
    v[i] = height;
    v[j] = std::cos(theta) * radius;
    v[k] = std::sin(theta) * radius;
}

/* Convert spherical coordinates to Cartesian coordinates in R3 */
template < typename E, class A > void
spherical_to_cartesian(E radius, E theta, E phi, size_t axis,
    SphericalType type, vector<E,A>& v)
{
    typedef vector<E,A> vector_type;
    typedef typename vector_type::value_type value_type;

    /* Checking */
    detail::CheckVec3(v);
    detail::CheckIndex3(axis);
    
    if (type == latitude) {
        phi = constants<value_type>::pi_over_2() - phi;
    }

    value_type sin_phi = std::sin(phi);
    value_type cos_phi = std::cos(phi);
    value_type sin_phi_r = sin_phi * radius;

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);

    v[i] = cos_phi * radius;
    v[j] = sin_phi_r * std::cos(theta);
    v[k] = sin_phi_r * std::sin(theta);
}

/* Convert polar coordinates to Cartesian coordinates in R2 */
template < typename E, class A > void
polar_to_cartesian(E radius, E theta, vector<E,A>& v)
{
    /* Checking handled by set() */
    v.set(std::cos(theta) * double(radius), std::sin(theta) * double(radius));
}

//////////////////////////////////////////////////////////////////////////////
// Conversion from Cartesian coordinates
//////////////////////////////////////////////////////////////////////////////

/* Convert Cartesian coordinates to cylindrical coordinates in R3  */
template < class VecT, typename Real > void
cartesian_to_cylindrical(const VecT& v, Real& radius, Real& theta,
    Real& height, size_t axis, Real tolerance = epsilon<Real>::placeholder())
{
    typedef Real value_type;

    /* Checking */
    detail::CheckVec3(v);
    detail::CheckIndex3(axis);

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);
    
    radius = length(v[j],v[k]);
    theta = radius < tolerance ? value_type(0) : std::atan2(v[k],v[j]);
    height = v[i];
}

/* Convert Cartesian coordinates to spherical coordinates in R3 */
template < class VecT, typename Real > void
cartesian_to_spherical(const VecT& v, Real& radius, Real& theta, Real& phi,
    size_t axis, SphericalType type,
    Real tolerance = epsilon<Real>::placeholder())
{
    typedef Real value_type;

    /* Checking */
    detail::CheckVec3(v);
    detail::CheckIndex3(axis);

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);

    value_type len = length(v[j],v[k]);
    theta = len < tolerance ? value_type(0) : std::atan2(v[k],v[j]);
    radius = length(v[i], len);
    if (radius < tolerance) {
        phi = value_type(0);
    } else {
        phi = std::atan2(len,v[i]);
        //phi = type.convert(phi);
        if (type == latitude) {
            phi = constants<value_type>::pi_over_2() - phi;
        }
    }
}

/* Convert Cartesian coordinates to polar coordinates in R2 */
template < class VecT, typename Real > void
cartesian_to_polar(const VecT& v, Real& radius, Real& theta,
    Real tolerance = epsilon<Real>::placeholder())
{
    typedef Real value_type;

    /* Checking */
    detail::CheckVec2(v);

    radius = v.length();
    theta = radius < tolerance ? value_type(0) : std::atan2(v[1],v[0]);
}

} // namespace cml

#endif
