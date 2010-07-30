/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef vector_angle_h
#define vector_angle_h

#include <cml/mathlib/checking.h>

/* Functions for finding the signed and unsigned angles between vectors in
 * 3D and 2D.
 *
 * Note that the input vectors for these functions are not required to be
 * unit length.
 *
 * @todo: Clean up promotions, conversions, and return types.
 */

namespace cml {

/** Signed angle between two 3D vectors. */
template< class VecT_1, class VecT_2, class VecT_3 >
typename detail::DotPromote<
    typename detail::CrossPromote<VecT_1,VecT_2>::promoted_vector, VecT_3
>::promoted_scalar
signed_angle(const VecT_1& v1, const VecT_2& v2, const VecT_3& reference)
{
    typedef typename detail::CrossPromote<VecT_1,VecT_2>::promoted_vector
        vector_type;
    typedef typename detail::DotPromote<vector_type,VecT_3>::promoted_scalar
        value_type;

    vector_type c = cross(v1,v2);
    value_type angle = std::atan2(double(length(c)),double(dot(v1,v2)));
    return dot(c,reference) < value_type(0) ? -angle : angle;
}

/** Unsigned angle between two 3D vectors. */
template< class VecT_1, class VecT_2 >
typename detail::DotPromote< VecT_1, VecT_2 >::promoted_scalar
unsigned_angle(const VecT_1& v1, const VecT_2& v2) {
    return std::atan2(double(length(cross(v1,v2))),double(dot(v1,v2)));
}

/** Signed angle between two 2D vectors. */
template< class VecT_1, class VecT_2 >
typename detail::DotPromote< VecT_1, VecT_2 >::promoted_scalar
signed_angle_2D(const VecT_1& v1, const VecT_2& v2) {
    return std::atan2(double(perp_dot(v1,v2)),double(dot(v1,v2)));
}

/** Unsigned angle between two 2D vectors. */
template< class VecT_1, class VecT_2 >
typename detail::DotPromote< VecT_1, VecT_2 >::promoted_scalar
unsigned_angle_2D(const VecT_1& v1, const VecT_2& v2) {
    return std::fabs(signed_angle_2D(v1,v2));
}

} // namespace cml

#endif
