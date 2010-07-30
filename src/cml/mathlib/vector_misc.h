/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef vector_misc_h
#define vector_misc_h

#include <cml/mathlib/coord_conversion.h>

/* Miscellaneous vector functions. */

namespace cml {

/* Function to project a vector v onto a hyperplane specified by a unit-length
 * normal n.
 *
 * @todo: Clean up promotion code.
 */

template < class VecT_1, class VecT_2 >
typename detail::CrossPromote<VecT_1,VecT_2>::promoted_vector
project_to_hplane(const VecT_1& v, const VecT_2& n)
{
    typedef typename detail::CrossPromote<VecT_1,VecT_2>::promoted_vector
        result_type;
        
    result_type result;
    et::detail::Resize(result, v.size());

    result = v - dot(v,n) * n;
    return result;
}

/* Return a vector perpendicular (CCW) to a 2D vector. */
template < class VecT > vector< typename VecT::value_type, fixed<2> >
perp(const VecT& v)
{
    typedef vector< typename VecT::value_type, fixed<2> > temporary_type;
    
    /* Checking */
    detail::CheckVec2(v);
    
    return temporary_type(-v[1],v[0]);
}

/* @todo: unit_cross() and cross_cardinal() should probably go in
 * vector_products.h, but I'm trying to avoid modifying the existing codebase
 * for now.
 */

/** Return normalized cross product of two vectors */
template< class LeftT, class RightT >
typename detail::CrossPromote<LeftT,RightT>::promoted_vector
unit_cross(const LeftT& left, const RightT& right) {
    /* @todo: This will probably break with dynamic<> vectors */
    return normalize(cross(left,right));
}

/** Return the cross product of v and the i'th cardinal basis vector */
template < class VecT > vector< typename VecT::value_type, fixed<3> >
cross_cardinal(const VecT& v, size_t i)
{
    typedef vector< typename VecT::value_type, fixed<3> > vector_type;
    typedef typename vector_type::value_type value_type;

    /* Checking */
    detail::CheckVec3(v);
    detail::CheckIndex3(i);

    size_t j, k;
    cyclic_permutation(i, i, j, k);
    vector_type result;
    result[i] = value_type(0);
    result[j] = v[k];
    result[k] = -v[j];
    return result;
}

/** Return the cross product of the i'th cardinal basis vector and v */
template < class VecT > vector< typename VecT::value_type, fixed<3> >
cross_cardinal(size_t i, const VecT& v)
{
    typedef vector< typename VecT::value_type, fixed<3> > vector_type;
    typedef typename vector_type::value_type value_type;

    /* Checking */
    detail::CheckVec3(v);
    detail::CheckIndex3(i);

    size_t j, k;
    cyclic_permutation(i, i, j, k);
    vector_type result;
    result[i] = value_type(0);
    result[j] = -v[k];
    result[k] = v[j];
    return result;
}

/** Rotate a 3D vector v about a unit-length vector n. */
template< class VecT_1, class VecT_2, typename Real >
vector<
    typename et::ScalarPromote<
        typename VecT_1::value_type,
        typename VecT_2::value_type
    >::type,
    fixed<3>
>
rotate_vector(const VecT_1& v, const VecT_2& n, Real angle)
{
    typedef vector<
        typename et::ScalarPromote<
            typename VecT_1::value_type,
            typename VecT_2::value_type
        >::type,
        fixed<3>
    > result_type;
    
    /* Checking */
    detail::CheckVec3(v);
    detail::CheckVec3(n);
    
    result_type parallel = dot(v,n)*n;
    return (
        std::cos(angle)*(v-parallel) + std::sin(angle)*cross(n,v) + parallel
    );
}

/** Rotate a 2D vector v about a unit-length vector n. */
template< class VecT, typename Real >
vector< typename VecT::value_type, fixed<2> >
rotate_vector_2D(const VecT& v, Real angle)
{
    typedef vector< typename VecT::value_type, fixed<2> > result_type;
    typedef typename result_type::value_type value_type;
    
    /* Checking */
    detail::CheckVec2(v);
    
    value_type s = std::sin(angle);
    value_type c = std::cos(angle);
    
    return result_type(c * v[0] - s * v[1], s * v[0] + c * v[1]);
}

/** Random unit 3D or 2D vector
 *
 * @todo: This is just placeholder code for what will be a more thorough
 * 'random unit' implementation:
 *
 * - All dimensions will be handled uniformly if practical, perhaps through
 *   a normal distrubution PRNG.
 *
 * - Failing that (or perhaps even in this case), dimensions 2 and 3 will be
 *   dispatched to special-case code, most likely implementing the algorithms
 *   below.
 *
 * - Like the utility random functions, the option of using one's own PRGN
 *   will be made available.
 *
 * @todo: Once N-d random vectors are supported, add a 'random unit
 * quaternion' function that wraps a call to random_unit() with a 4D vector as
 * the argument.
 */
template < typename E, class A > void
random_unit(vector<E,A>& v)
{
    typedef vector<E,A> vector_type;
    typedef typename vector_type::value_type value_type;
    
    switch (v.size()) {
        case 3:
        {
            vector< E, fixed<3> > temp;
            spherical_to_cartesian(
                value_type(1),
                value_type(random_unit() * constants<value_type>::two_pi()),
                acos_safe(random_real(value_type(-1),value_type(1))),
                2,
                colatitude,
                temp
            );
            v[0] = temp[0];
            v[1] = temp[1];
            v[2] = temp[2];
            break;
        }
        case 2:
        {
            vector< E, fixed<2> > temp;
            polar_to_cartesian(
                value_type(1),
                value_type(random_unit() * constants<value_type>::two_pi()),
                temp
            );
            v[0] = temp[0];
            v[1] = temp[1];
            break;
        }
        default:
            throw std::invalid_argument(
                "random_unit() for N-d vectors not implemented yet");
            break;
    }
}

/* Random vector within a given angle of a unit-length axis, i.e. in a cone
 * (3D) or wedge (2D).
 *
 * The same notes the appear above apply here too, more or less. One
 * difference is that this is really only useful in 2D and 3D (presumably), so
 * we'll probably just do a compile- or run-time dispatch as appropriate.
 *
 * Also, there may be a better algorithm for generating a random unit vector
 * in a cone; need to look into that.
 *
 * All of this 'temp' stuff is because there's no compile-time dispatch for
 * 3D and 2D vectors, but that'll be fixed soon.
 */

template < typename E, class A, class VecT > void
random_unit(vector<E,A>& v, const VecT& axis, E theta)
{
    typedef vector<E,A> vector_type;
    typedef typename vector_type::value_type value_type;
    
    switch (v.size()) {
        case 3:
        {
            vector< E, fixed<3> > temp, n, temp_axis;
            temp_axis[0] = axis[0];
            temp_axis[1] = axis[1];
            temp_axis[2] = axis[2];

            /* @todo: Function for finding 'any perpendicular vector'? */
            n = axis_3D(cml::index_of_min_abs(axis[0],axis[1],axis[2]));
            n = cross(n,temp_axis);
            
            /* Rotate v 'away from' the axis by a random angle in the range
             * [-theta,theta]
             */
            temp = rotate_vector(temp_axis,n,random_real(-theta,theta));
             
            /* Rotate v about the axis by a random angle in the range [-pi,pi]
             */
            temp = rotate_vector(
                temp,
                temp_axis,
                random_real(
                    -constants<value_type>::pi(),
                     constants<value_type>::pi()
                )
            );

            v[0] = temp[0];
            v[1] = temp[1];
            v[2] = temp[2];
            break;
        }
        case 2:
        {
            vector< E, fixed<2> > temp, temp_axis;
            temp_axis[0] = axis[0];
            temp_axis[1] = axis[1];
            temp = rotate_vector_2D(temp_axis, random_real(-theta,theta));
            v[0] = temp[0];
            v[1] = temp[1];
            break;
        }
        default:
            throw std::invalid_argument(
                "random_unit(v,axis,theta) only implemented for 2D and 3D");
            break;
    }
}

/* NEW: Manhattan distance */

template< class VecT_1, class VecT_2 >
typename detail::DotPromote< VecT_1, VecT_2 >::promoted_scalar
manhattan_distance(const VecT_1& v1, const VecT_2& v2) {
    /* Check that a promotion exists */
    typedef typename et::VectorPromote<
        VecT_1,VecT_2>::temporary_type promoted_vector;
        
    typedef typename detail::DotPromote< VecT_1, VecT_2 >::promoted_scalar scalar_type;
    
    scalar_type sum = scalar_type(0);
    for (size_t i = 0; i < v1.size(); ++i) {
        sum += std::fabs(v2[i]-v1[i]);
    }
    return sum;
}

} // namespace cml

#endif
