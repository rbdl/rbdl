/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef quaternion_comparison_h
#define quaternion_comparison_h

#include <cml/core/cml_assert.h>
#include <cml/et/scalar_ops.h>

/* This is used below to create a more meaningful compile-time error when
 * quaternion_comparison is not provided with quaternion or QuaternionExpr arguments:
 */
struct quaternion_comparison_expects_quaternion_args_error;

#define CML_QUAT_QUAT_ORDER(_order_, _op_, _OpT_)                       \
template<                                                               \
    typename E1, class AT1, typename E2, class AT2, class O, class C >  \
inline bool                                                             \
_op_ (                                                                  \
        const quaternion<E1,AT1,O,C>& left,                             \
        const quaternion<E2,AT2,O,C>& right)                            \
{                                                                       \
    return detail::quaternion_##_order_ (left, right, _OpT_ <E1,E2>()); \
}

#define CML_QUAT_QUATXPR_ORDER(_order_, _op_, _OpT_)                    \
template<typename E, class AT, class O, class C, class XprT>            \
inline bool                                                             \
_op_ (                                                                  \
        const quaternion<E,AT,O,C>& left,                               \
        QUATXPR_ARG_TYPE right)                                         \
{                                                                       \
    return detail::quaternion_##_order_ (left, right,                   \
            _OpT_ <E, typename XprT::value_type>());                    \
}

#define CML_QUATXPR_QUAT_ORDER(_order_, _op_, _OpT_)                    \
template<class XprT, typename E, class AT, class O, class C >           \
inline bool                                                             \
_op_ (                                                                  \
        QUATXPR_ARG_TYPE left,                                          \
        const quaternion<E,AT,O,C>& right)                              \
{                                                                       \
    return detail::quaternion_##_order_ (left, right,                   \
            _OpT_ <typename XprT::value_type, E>());                    \
}

#define CML_QUATXPR_QUATXPR_ORDER(_order_, _op_, _OpT_)                 \
template<class XprT1, class XprT2>                                      \
inline bool                                                             \
_op_ (                                                                  \
        QUATXPR_ARG_TYPE_N(1) left,                                     \
        QUATXPR_ARG_TYPE_N(2) right)                                    \
{                                                                       \
    return detail::quaternion_##_order_ (left, right,                   \
            _OpT_ <                                                     \
                typename XprT1::value_type,                             \
                typename XprT2::value_type>());                         \
}


namespace cml {
namespace detail {

/** Quaternion strict weak ordering relationship.
 *
 * OpT must implement a strict weak order on the quaternion element type.
 * operator< and operator> on integer and floating-point types are
 * examples.
 */
template<typename LeftT, typename RightT, typename OpT>
inline bool
quaternion_weak_order(const LeftT& left, const RightT& right, OpT)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;

    /* quaternion_comparison() requires quaternion expressions: */
    CML_STATIC_REQUIRE_M(
            (et::QuaternionExpressions<LeftT,RightT>::is_true),
            quaternion_comparison_expects_quaternion_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas:
     */

    typedef typename et::QuaternionPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::type result_type;

    for(ssize_t i = 0; i < result_type::array_size; ++ i) {

        if(OpT().apply(
                    left_traits().get(left,i),
                    right_traits().get(right,i)
                    ))
        {
            /* If weak order (a < b) is satisfied, return true: */
            return true;
        } else if(OpT().apply(
                    right_traits().get(right,i),
                    left_traits().get(left,i)
                    ))
        {
            /* If !(b < a), then return false: */
            return false;
        } else {

            /* Have !(a < b) && !(b < a) <=> (a >= b && b >= a) <=> (a == b).
             * so need to test next element:
             */
            continue;
        }
    }
    /* XXX Can this be unrolled in any reasonable way? */

    /* If we get here, then left == right: */
    return false;
}

/** Quaternion total order relationship.
 *
 * OpT must implement a total order on the quaternion element type.  operator<=
 * and operator>= on integer and floating-point types are examples.
 */
template<typename LeftT, typename RightT, typename OpT>
inline bool
quaternion_total_order(const LeftT& left, const RightT& right, OpT)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;

    /* quaternion_comparison() requires quaternion expressions: */
    CML_STATIC_REQUIRE_M(
            (et::QuaternionExpressions<LeftT,RightT>::is_true),
            quaternion_comparison_expects_quaternion_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas:
     */

    typedef typename et::QuaternionPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::type result_type;

    for(ssize_t i = 0; i < result_type::array_size; ++ i) {

        /* Test total order: */
        if(OpT().apply(
                    left_traits().get(left,i),
                    right_traits().get(right,i)
		    ))
        {
            /* Automatically true if weak order (a <= b) && !(b <= a) <=>
             * (a <= b) && (b > a) <=> (a < b) is satisfied:
             */
            if(!OpT().apply(
                        right_traits().get(right,i),
                        left_traits().get(left,i)
                        ))
                return true;

            /* Otherwise, have equality (a <= b) && (b <= a), so continue
             * to next element:
             */
            else
                continue;

        } else {
            
            /* Total order isn't satisfied (a > b), so return false: */
            return false;
        }
    }
    /* XXX Can this be unrolled in any reasonable way? */

    /* Total (==) or weak (<) order was satisfied, so return true: */
    return true;
}

}

/* XXX There is a better way to handle these with operator traits... */

CML_QUAT_QUAT_ORDER(       total_order, operator==, et::OpEqual)
CML_QUATXPR_QUAT_ORDER(    total_order, operator==, et::OpEqual)
CML_QUAT_QUATXPR_ORDER(    total_order, operator==, et::OpEqual)
CML_QUATXPR_QUATXPR_ORDER( total_order, operator==, et::OpEqual)

CML_QUAT_QUAT_ORDER(       weak_order, operator!=, et::OpNotEqual)
CML_QUATXPR_QUAT_ORDER(    weak_order, operator!=, et::OpNotEqual)
CML_QUAT_QUATXPR_ORDER(    weak_order, operator!=, et::OpNotEqual)
CML_QUATXPR_QUATXPR_ORDER( weak_order, operator!=, et::OpNotEqual)

CML_QUAT_QUAT_ORDER(       weak_order, operator<, et::OpLess)
CML_QUATXPR_QUAT_ORDER(    weak_order, operator<, et::OpLess)
CML_QUAT_QUATXPR_ORDER(    weak_order, operator<, et::OpLess)
CML_QUATXPR_QUATXPR_ORDER( weak_order, operator<, et::OpLess)

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
