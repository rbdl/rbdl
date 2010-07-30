/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Multiplication of two quaternions, p*q.
 *
 * This uses the expression tree, since the result is closed-form and can be
 * computed by index.
 */

#ifndef quaternion_mul_h
#define quaternion_mul_h

#include <cml/mathlib/checking.h>
#include <cml/quaternion/quaternion_promotions.h>

namespace cml {
namespace detail {

template < class CrossType, class Real > struct SumOp;

template < class Real > struct SumOp< positive_cross, Real > {
    Real operator()(Real a, Real b) const {
        return a + b;
    }
};

template < class Real > struct SumOp< negative_cross, Real > {
    Real operator()(Real a, Real b) const {
        return a - b;
    }
};

template < class Quat1_T, class Quat2_T >
typename et::QuaternionPromote<
    typename Quat1_T::temporary_type, typename Quat2_T::temporary_type
>::temporary_type
QuaternionMult(const Quat1_T& q1, const Quat2_T& q2)
{
    detail::CheckQuat(q1);
    detail::CheckQuat(q2);
    
    typedef typename et::QuaternionPromote<
        typename Quat1_T::temporary_type, typename Quat2_T::temporary_type
    >::temporary_type temporary_type;

    typedef typename temporary_type::value_type value_type;
    typedef typename temporary_type::order_type order_type;
    typedef typename temporary_type::cross_type cross_type;

    typedef detail::SumOp<cross_type, value_type> sum_op;

    enum {
        W = order_type::W,
        X = order_type::X,
        Y = order_type::Y,
        Z = order_type::Z
    };
    
    temporary_type result;

    /* s1*s2-dot(v1,v2): */
    result[W] =
        q1[W]*q2[W] - q1[X]*q2[X] - q1[Y]*q2[Y] - q1[Z]*q2[Z];

    /* (s1*v2 + s2*v1 + v1^v2) i: */
    result[X] =
        sum_op()(q1[W]*q2[X] + q2[W]*q1[X], q1[Y]*q2[Z] - q1[Z]*q2[Y]);

    /* (s1*v2 + s2*v1 + v1^v2) j: */
    result[Y] =
        sum_op()(q1[W]*q2[Y] + q2[W]*q1[Y], q1[Z]*q2[X] - q1[X]*q2[Z]);

    /* (s1*v2 + s2*v1 + v1^v2) k: */
    result[Z] =
        sum_op()(q1[W]*q2[Z] + q2[W]*q1[Z], q1[X]*q2[Y] - q1[Y]*q2[X]);

    return result;
}

} // namespace detail

/** Declare mul taking two quaternion operands. */
template<typename E1, class AT1, typename E2, class AT2, class OT, class CT>
inline typename et::QuaternionPromote<
    typename quaternion<E1,AT1,OT,CT>::temporary_type,
    typename quaternion<E2,AT2,OT,CT>::temporary_type
>::temporary_type operator*(
    const quaternion<E1,AT1,OT,CT>& left,
    const quaternion<E2,AT2,OT,CT>& right)
{
    return detail::QuaternionMult(left, right);
}

/** Declare mul taking a quaternion and a et::QuaternionXpr. */
template<typename E, class AT, class OT, class CT, class XprT>
inline typename et::QuaternionPromote<
    typename quaternion<E,AT,OT,CT>::temporary_type,
    typename XprT::temporary_type
>::temporary_type operator*(
    const quaternion<E,AT,OT,CT>& left,
    QUATXPR_ARG_TYPE right)
{
    return detail::QuaternionMult(left, right);
}

/** Declare mul taking an et::QuaternionXpr and a quaternion. */
template<class XprT, typename E, class AT, class OT, class CT>
inline typename et::QuaternionPromote<
    typename XprT::temporary_type,
    typename quaternion<E,AT,OT,CT>::temporary_type
>::temporary_type operator*(
    QUATXPR_ARG_TYPE left,
    const quaternion<E,AT,OT,CT>& right)
{
    return detail::QuaternionMult(left, right);
}

/** Declare mul taking two et::QuaternionXpr operands. */
template<class XprT1, class XprT2>
inline typename et::QuaternionPromote<
    typename XprT1::temporary_type, typename XprT2::temporary_type
>::temporary_type operator*(
    QUATXPR_ARG_TYPE_N(1) left,
    QUATXPR_ARG_TYPE_N(2) right)
{
    return detail::QuaternionMult(left, right);
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
