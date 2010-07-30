/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef quaternion_dot_h
#define quaternion_dot_h

#include <cml/vector/vector_products.h>
#include <cml/quaternion/quaternion_expr.h>

namespace cml {
namespace detail {

template<class LeftT, class RightT> inline
typename detail::DotPromote<LeftT,RightT>::promoted_scalar
quaternion_dot(const LeftT& p, const RightT& q)
{
    return p[0]*q[0] + p[1]*q[1] + p[2]*q[2] + p[3]*q[3];
}

} // namespace detail

template<typename E1, class AT1, typename E2, class AT2, class OT, class CT>
inline typename detail::DotPromote<
    quaternion<E1,AT1,OT,CT>, quaternion<E2,AT2,OT,CT>
>::promoted_scalar
dot(const quaternion<E1,AT1,OT,CT>& p,
    const quaternion<E2,AT2,OT,CT>& q)
{
    return detail::quaternion_dot(p,q);
}

template<typename E, class AT, class OT, class CT, class XprT>
inline typename detail::DotPromote<
    quaternion<E,AT,OT,CT>, et::QuaternionXpr<XprT>
>::promoted_scalar
dot(const quaternion<E,AT,OT,CT>& p, QUATXPR_ARG_TYPE q)
{
    return detail::quaternion_dot(p,q);
}

template<class XprT, typename E, class AT, class OT, class CT>
inline typename detail::DotPromote<
    et::QuaternionXpr<XprT>, quaternion<E,AT,OT,CT>
>::promoted_scalar
dot(QUATXPR_ARG_TYPE p, const quaternion<E,AT,OT,CT>& q)
{
    return detail::quaternion_dot(p,q);
}

template<class XprT1, class XprT2> inline
typename detail::DotPromote<
    et::QuaternionXpr<XprT1>, et::QuaternionXpr<XprT2>
>::promoted_scalar
dot(QUATXPR_ARG_TYPE_N(1) p, QUATXPR_ARG_TYPE_N(2) q)
{
    return detail::quaternion_dot(p,q);
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
