/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef vector_comparison_h
#define vector_comparison_h

#include <cml/core/cml_assert.h>
#include <cml/et/size_checking.h>
#include <cml/et/scalar_ops.h>

/* This is used below to create a more meaningful compile-time error when
 * vector_comparison is not provided with vector or VectorExpr arguments:
 */
struct vector_comparison_expects_vector_args_error;

#define CML_VEC_VEC_ORDER(_order_, _op_, _OpT_)                         \
template<typename E1, class AT1, typename E2, class AT2>                \
inline bool                                                             \
_op_ (                                                                  \
        const vector<E1,AT1>& left,                                     \
        const vector<E2,AT2>& right)                                    \
{                                                                       \
    return detail::vector_##_order_ (left, right, _OpT_ <E1,E2>());     \
}

#define CML_VEC_VECXPR_ORDER(_order_, _op_, _OpT_)                      \
template<typename E, class AT, class XprT>                              \
inline bool                                                             \
_op_ (                                                                  \
        const vector<E,AT>& left,                                       \
        VECXPR_ARG_TYPE right)                                          \
{                                                                       \
    return detail::vector_##_order_ (left, right,                       \
            _OpT_ <E, typename XprT::value_type>());                    \
}

#define CML_VECXPR_VEC_ORDER(_order_, _op_, _OpT_)                      \
template<class XprT, typename E, class AT>                              \
inline bool                                                             \
_op_ (                                                                  \
        VECXPR_ARG_TYPE left,                                           \
        const vector<E,AT>& right)                                      \
{                                                                       \
    return detail::vector_##_order_ (left, right,                       \
            _OpT_ <typename XprT::value_type, E>());                    \
}

#define CML_VECXPR_VECXPR_ORDER(_order_, _op_, _OpT_)                   \
template<class XprT1, class XprT2>                                      \
inline bool                                                             \
_op_ (                                                                  \
        VECXPR_ARG_TYPE_N(1) left,                                      \
        VECXPR_ARG_TYPE_N(2) right)                                     \
{                                                                       \
    return detail::vector_##_order_ (left, right,                       \
            _OpT_ <                                                     \
                typename XprT1::value_type,                             \
                typename XprT2::value_type>());                         \
}


namespace cml {
namespace detail {

/** Vector strict weak ordering relationship.
 *
 * OpT must implement a strict weak order on the vector element type.
 * operator< and operator> on integer and floating-point types are
 * examples.
 */
template<typename LeftT, typename RightT, typename OpT>
inline bool
vector_weak_order(const LeftT& left, const RightT& right, OpT)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;

    /* vector_comparison() requires vector expressions: */
    CML_STATIC_REQUIRE_M(
            (et::VectorExpressions<LeftT,RightT>::is_true),
            vector_comparison_expects_vector_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas:
     */

    typedef typename et::VectorPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::type result_type;
    typedef typename result_type::size_tag size_tag;

    /* Verify expression size: */
    ssize_t N = (ssize_t) et::CheckedSize(left,right,size_tag());
    for(ssize_t i = 0; i < N; ++ i) {
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

/** Vector total order relationship.
 *
 * OpT must implement a total order on the vector element type.  operator<=
 * and operator>= on integer and floating-point types are examples.
 */
template<typename LeftT, typename RightT, typename OpT>
inline bool
vector_total_order(const LeftT& left, const RightT& right, OpT)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;

    /* vector_comparison() requires vector expressions: */
    CML_STATIC_REQUIRE_M(
            (et::VectorExpressions<LeftT,RightT>::is_true),
            vector_comparison_expects_vector_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas:
     */

    typedef typename et::VectorPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::type result_type;
    typedef typename result_type::size_tag size_tag;

    /* Verify expression size: */
    ssize_t N = (ssize_t) et::CheckedSize(left,right,size_tag());
    for(ssize_t i = 0; i < N; ++ i) {

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

CML_VEC_VEC_ORDER(       total_order, operator==, et::OpEqual)
CML_VECXPR_VEC_ORDER(    total_order, operator==, et::OpEqual)
CML_VEC_VECXPR_ORDER(    total_order, operator==, et::OpEqual)
CML_VECXPR_VECXPR_ORDER( total_order, operator==, et::OpEqual)

CML_VEC_VEC_ORDER(       weak_order, operator!=, et::OpNotEqual)
CML_VECXPR_VEC_ORDER(    weak_order, operator!=, et::OpNotEqual)
CML_VEC_VECXPR_ORDER(    weak_order, operator!=, et::OpNotEqual)
CML_VECXPR_VECXPR_ORDER( weak_order, operator!=, et::OpNotEqual)

CML_VEC_VEC_ORDER(       weak_order, operator<, et::OpLess)
CML_VECXPR_VEC_ORDER(    weak_order, operator<, et::OpLess)
CML_VEC_VECXPR_ORDER(    weak_order, operator<, et::OpLess)
CML_VECXPR_VECXPR_ORDER( weak_order, operator<, et::OpLess)

CML_VEC_VEC_ORDER(       weak_order, operator>, et::OpGreater)
CML_VECXPR_VEC_ORDER(    weak_order, operator>, et::OpGreater)
CML_VEC_VECXPR_ORDER(    weak_order, operator>, et::OpGreater)
CML_VECXPR_VECXPR_ORDER( weak_order, operator>, et::OpGreater)

CML_VEC_VEC_ORDER(       total_order, operator<=, et::OpLessEqual)
CML_VECXPR_VEC_ORDER(    total_order, operator<=, et::OpLessEqual)
CML_VEC_VECXPR_ORDER(    total_order, operator<=, et::OpLessEqual)
CML_VECXPR_VECXPR_ORDER( total_order, operator<=, et::OpLessEqual)

CML_VEC_VEC_ORDER(       total_order, operator>=, et::OpGreaterEqual)
CML_VECXPR_VEC_ORDER(    total_order, operator>=, et::OpGreaterEqual)
CML_VEC_VECXPR_ORDER(    total_order, operator>=, et::OpGreaterEqual)
CML_VECXPR_VECXPR_ORDER( total_order, operator>=, et::OpGreaterEqual)

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
