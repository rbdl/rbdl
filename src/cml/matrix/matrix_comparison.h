/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * @todo The matrix and matrix order operators could probably be combined
 * into a single templated implementation, since the only thing that is
 * different is the access method.
 */

#ifndef matrix_comparison_h
#define matrix_comparison_h

#include <cml/core/cml_assert.h>
#include <cml/et/size_checking.h>
#include <cml/et/scalar_ops.h>

/* This is used below to create a more meaningful compile-time error when
 * matrix_comparison is not provided with matrix or MatrixExpr arguments:
 */
struct matrix_comparison_expects_matrix_args_error;

#define CML_MAT_MAT_ORDER(_order_, _op_, _OpT_)                         \
template<typename E1, class AT1, typename L1,                           \
         typename E2, class AT2, typename L2, typename BO>              \
inline bool                                                             \
_op_ (                                                                  \
        const matrix<E1,AT1,L1,BO>& left,                               \
        const matrix<E2,AT2,L2,BO>& right)                              \
{                                                                       \
    return detail::matrix_##_order_ (left, right, _OpT_ <E1,E2>());     \
}

#define CML_MAT_MATXPR_ORDER(_order_, _op_, _OpT_)                      \
template<typename E, class AT, typename L, typename BO, class XprT>     \
inline bool                                                             \
_op_ (                                                                  \
        const matrix<E,AT,L,BO>& left,                                  \
        MATXPR_ARG_TYPE right)                                          \
{                                                                       \
    return detail::matrix_##_order_ (left, right,                       \
            _OpT_ <E, typename XprT::value_type>());                    \
}

#define CML_MATXPR_MAT_ORDER(_order_, _op_, _OpT_)                      \
template<class XprT, typename E, class AT, typename L, typename BO>     \
inline bool                                                             \
_op_ (                                                                  \
        MATXPR_ARG_TYPE left,                                           \
        const matrix<E,AT,L,BO>& right)                                 \
{                                                                       \
    return detail::matrix_##_order_ (left, right,                       \
            _OpT_ <typename XprT::value_type, E>());                    \
}

#define CML_MATXPR_MATXPR_ORDER(_order_, _op_, _OpT_)                   \
template<class XprT1, class XprT2>                                      \
inline bool                                                             \
_op_ (                                                                  \
        MATXPR_ARG_TYPE_N(1) left,                                      \
        MATXPR_ARG_TYPE_N(2) right)                                     \
{                                                                       \
    return detail::matrix_##_order_ (left, right,                       \
            _OpT_ <                                                     \
                typename XprT1::value_type,                             \
                typename XprT2::value_type>());                         \
}


namespace cml {
namespace detail {

/** Matrix strict weak ordering relationship.
 *
 * OpT must implement a strict weak order on the matrix element type.
 * operator< and operator> on integer and floating-point types are
 * examples.
 */
template<typename LeftT, typename RightT, typename OpT>
inline bool
matrix_weak_order(const LeftT& left, const RightT& right, OpT)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;

    /* matrix_comparison() requires matrix expressions: */
    CML_STATIC_REQUIRE_M(
            (et::MatrixExpressions<LeftT,RightT>::is_true),
            matrix_comparison_expects_matrix_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas:
     */

    typedef typename et::MatrixPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::type result_type;
    typedef typename result_type::size_tag size_tag;

    /* Verify expression size: */
    matrix_size N = et::CheckedSize(left,right,size_tag());
    for(ssize_t i = 0; i < N.first; ++ i) {
        for(ssize_t j = 0; j < N.second; ++ j) {
            if(OpT().apply(
                        left_traits().get(left,i,j),
                        right_traits().get(right,i,j)
                        ))
            {
                /* If weak order (a < b) is satisfied, return true: */
                return true;
            } else if(OpT().apply(
                        right_traits().get(right,i,j),
                        left_traits().get(left,i,j)
                        ))
            {
                /* If !(b < a), then return false: */
                return false;
            } else {

                /* Have !(a < b) && !(b < a) <=> (a >= b && b >= a)
                 * <=> (a == b).  so need to test next element:
                 */
                continue;
            }
        }
    }
    /* XXX Can this be unrolled in any reasonable way? */

    /* If we get here, then left == right: */
    return false;
}

/** Matrix total order relationship.
 *
 * OpT must implement a total order on the matrix element type.  operator<=
 * and operator>= on integer and floating-point types are examples.
 */
template<typename LeftT, typename RightT, typename OpT>
inline bool
matrix_total_order(const LeftT& left, const RightT& right, OpT)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;

    /* matrix_comparison() requires matrix expressions: */
    CML_STATIC_REQUIRE_M(
            (et::MatrixExpressions<LeftT,RightT>::is_true),
            matrix_comparison_expects_matrix_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas:
     */

    typedef typename et::MatrixPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::type result_type;
    typedef typename result_type::size_tag size_tag;

    /* Verify expression size: */
    matrix_size N = et::CheckedSize(left,right,size_tag());
    for(ssize_t i = 0; i < N.first; ++ i) {
        for(ssize_t j = 0; j < N.second; ++ j) {

            /* Test total order: */
            if(OpT().apply(
                        left_traits().get(left,i,j),
                        right_traits().get(right,i,j)
                        ))
            {
                /* Automatically true if weak order (a <= b) && !(b <= a)
                 * <=> (a <= b) && (b > a) <=> (a < b) is satisfied:
                 */
                if(!OpT().apply(
                            right_traits().get(right,i,j),
                            left_traits().get(left,i,j)
                            ))
                    return true;

                /* Otherwise, have equality (a <= b) && (b <= a), so
                 * continue to next element:
                 */
                else
                    continue;

            } else {

                /* Total order isn't satisfied (a > b), so return false: */
                return false;
            }
        }
    }
    /* XXX Can this be unrolled in any reasonable way? */

    /* Total (==) or weak (<) order was satisfied, so return true: */
    return true;
}

}

} // namespace cml

/* XXX There is a better way to handle these with operator traits... */

CML_MAT_VEC_ORDER(       total_order, operator==, et::OpEqual)
CML_MATXPR_MAT_ORDER(    total_order, operator==, et::OpEqual)
CML_MAT_MATXPR_ORDER(    total_order, operator==, et::OpEqual)
CML_MATXPR_VECXPR_ORDER( total_order, operator==, et::OpEqual)

CML_MAT_VEC_ORDER(       weak_order, operator!=, et::OpNotEqual)
CML_MATXPR_MAT_ORDER(    weak_order, operator!=, et::OpNotEqual)
CML_MAT_MATXPR_ORDER(    weak_order, operator!=, et::OpNotEqual)
CML_MATXPR_VECXPR_ORDER( weak_order, operator!=, et::OpNotEqual)

CML_MAT_VEC_ORDER(       weak_order, operator<, et::OpLess)
CML_MATXPR_MAT_ORDER(    weak_order, operator<, et::OpLess)
CML_MAT_MATXPR_ORDER(    weak_order, operator<, et::OpLess)
CML_MATXPR_VECXPR_ORDER( weak_order, operator<, et::OpLess)

CML_MAT_VEC_ORDER(       weak_order, operator>, et::OpGreater)
CML_MATXPR_MAT_ORDER(    weak_order, operator>, et::OpGreater)
CML_MAT_MATXPR_ORDER(    weak_order, operator>, et::OpGreater)
CML_MATXPR_VECXPR_ORDER( weak_order, operator>, et::OpGreater)

CML_MAT_VEC_ORDER(       total_order, operator<=, et::OpLessEqual)
CML_MATXPR_MAT_ORDER(    total_order, operator<=, et::OpLessEqual)
CML_MAT_MATXPR_ORDER(    total_order, operator<=, et::OpLessEqual)
CML_MATXPR_VECXPR_ORDER( total_order, operator<=, et::OpLessEqual)

CML_MAT_VEC_ORDER(       total_order, operator>=, et::OpGreaterEqual)
CML_MATXPR_MAT_ORDER(    total_order, operator>=, et::OpGreaterEqual)
CML_MAT_MATXPR_ORDER(    total_order, operator>=, et::OpGreaterEqual)
CML_MATXPR_VECXPR_ORDER( total_order, operator>=, et::OpGreaterEqual)

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
