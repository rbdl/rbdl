/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Defines vector dot and outer products.
 *
 * @todo Figure out if the source or destination size type should trigger
 * unrolling.  May need a per-compiler compile-time option for this.
 */

#ifndef vector_products_h
#define vector_products_h

#include <cml/core/cml_assert.h>
#include <cml/et/scalar_promotions.h>
#include <cml/et/size_checking.h>
#include <cml/vector/vector_unroller.h>
#include <cml/vector/vector_expr.h>
#include <cml/matrix/matrix_expr.h>

/* This is used below to create a more meaningful compile-time error when
 * dot() is not provided with vector or VectorExpr arguments:
 */
struct dot_expects_vector_args_error;

/* This is used below to create a more meaningful compile-time error when
 * perp_dot() is not provided with 2D vector or VectorExpr arguments:
 */
struct perp_dot_expects_vector_args_error;
struct perp_dot_expects_2D_vector_args_error;

/* This is used below to create a more meaningful compile-time error when
 * outer() is not provided with vector or VectorExpr arguments:
 */
struct outer_expects_vector_args_error;

/* This is used below to create a more meaningful compile-time error when
 * cross() is not provided with 3D vector or VectorExpr arguments:
 */
struct cross_expects_vector_args_error;
struct cross_expects_3D_vector_args_error;


namespace cml {
namespace detail {

template<typename LeftT, typename RightT>
struct DotPromote
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::value_type left_value;
    typedef typename right_traits::value_type right_value;

    /* Deduce the promoted scalar type: */
    typedef et::OpMul<left_value, right_value> op_mul;
    typedef typename et::OpAdd<
        typename op_mul::value_type,
                 typename op_mul::value_type> op_add;
    typedef typename op_add::value_type promoted_scalar;
};

template<typename LeftT, typename RightT>
struct CrossPromote
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_type left_type;
    typedef typename right_traits::result_type right_type;

    /* Deduce the matrix result type: */
    typedef typename et::VectorPromote<
        left_type,right_type>::temporary_type promoted_vector;
};

template<typename LeftT, typename RightT>
struct OuterPromote
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_type left_type;
    typedef typename right_traits::result_type right_type;

    /* Deduce the matrix result type: */
    typedef typename et::MatrixPromote<
        left_type,right_type>::temporary_type promoted_matrix;
};

/** Construct a dot unroller for fixed-size arrays.
 *
 * @note This should only be called for vectors.
 *
 * @sa cml::dot
 */
template<typename LeftT, typename RightT>
inline typename DotPromote<LeftT,RightT>::promoted_scalar
UnrollDot(const LeftT& left, const RightT& right, fixed_size_tag)
{
    /* Shorthand: */
    typedef DotPromote<LeftT,RightT> dot_helper;

    /* Compile-type vector size check: */
    typedef typename et::GetCheckedSize<LeftT,RightT,fixed_size_tag>
        ::check_type check_sizes;

    /* Get the fixed array size using the helper: */
    enum { Len = check_sizes::array_size };

    /* Record the unroller type: */
    typedef typename dot_helper::op_mul op_mul;
    typedef typename dot_helper::op_add op_add;
    typedef typename et::detail::VectorAccumulateUnroller<
        op_add,op_mul,LeftT,RightT>::template
        Eval<0, Len-1, (Len <= CML_VECTOR_DOT_UNROLL_LIMIT)> Unroller;
    /* Note: Len is the array size, so Len-1 is the last element. */

    /* Now, call the unroller: */
    return Unroller()(left,right);
}

/** Use a loop to compute the dot product for dynamic arrays.
 *
 * @note This should only be called for vectors.
 *
 * @sa cml::dot
 */
template<typename LeftT, typename RightT>
inline typename DotPromote<LeftT,RightT>::promoted_scalar
UnrollDot(const LeftT& left, const RightT& right, dynamic_size_tag)
{
    /* Shorthand: */
    typedef DotPromote<LeftT,RightT> dot_helper;
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename dot_helper::op_mul op_mul;
    typedef typename dot_helper::op_add op_add;

    /* Record the return type: */
    typedef typename dot_helper::promoted_scalar sum_type;

    /* Verify expression sizes: */
    const size_t N = et::CheckedSize(left,right,dynamic_size_tag());

    /* Initialize the sum. Left and right must be vector expressions, so
     * it's okay to use array notation here:
     */
    sum_type sum(op_mul().apply(left[0],right[0]));
    for(size_t i = 1; i < N; ++i) {
        /* XXX This might not be optimized properly by some compilers.
         * but to do anything else requires changing the requirements
         * of a scalar operator, or requires defining a new class of scalar
         * <op>= operators.
         */
        sum = op_add().apply(sum, op_mul().apply(left[i], right[i]));
        /* Note: we don't need get(), since both arguments are required to
         * be vector expressions.
         */
    }
    return sum;
}

/** For cross(): compile-time check for a 3D vector. */
template<typename VecT> inline void
Require3D(const VecT&, fixed_size_tag) {
    CML_STATIC_REQUIRE_M(
            ((size_t)VecT::array_size == 3),
            cross_expects_3D_vector_args_error);
}

/** For cross(): run-time check for a 3D vector. */
template<typename VecT> inline void
Require3D(const VecT& v, dynamic_size_tag) {
    et::GetCheckedSize<VecT,VecT,dynamic_size_tag>()
        .equal_or_fail(v.size(),size_t(3));
}

/** For perp_dot(): compile-time check for a 2D vector. */
template<typename VecT> inline void
Require2D(const VecT& v, fixed_size_tag) {
    CML_STATIC_REQUIRE_M(
            ((size_t)VecT::array_size == 2),
            perp_dot_expects_2D_vector_args_error);
}

/** For perp_dot(): run-time check for a 2D vector. */
template<typename VecT> inline void
Require2D(const VecT& v, dynamic_size_tag) {
    et::GetCheckedSize<VecT,VecT,dynamic_size_tag>()
        .equal_or_fail(v.size(),size_t(2));
}

} // namespace detail


/** Vector dot (inner) product implementation.
 */
template<typename LeftT, typename RightT>
inline typename detail::DotPromote<LeftT,RightT>::promoted_scalar
dot(const LeftT& left, const RightT& right)
{
    /* Shorthand: */
    typedef detail::DotPromote<LeftT,RightT> dot_helper;
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_type left_type;
    typedef typename right_traits::result_type right_type;
    typedef typename left_traits::size_tag left_size;
    typedef typename right_traits::size_tag right_size;

    /* dot() requires vector expressions: */
    CML_STATIC_REQUIRE_M(
            (et::VectorExpressions<LeftT,RightT>::is_true),
            dot_expects_vector_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas:
     */

    /* Figure out the unroller to use (fixed or dynamic): */
    typedef typename et::VectorPromote<
        left_type, right_type>::temporary_type promoted_vector;
    typedef typename promoted_vector::size_tag size_tag;

    /* Call unroller: */
    return detail::UnrollDot(left,right,size_tag());
}

/** perp_dot()
 */
template<typename LeftT, typename RightT>
inline typename detail::DotPromote<LeftT,RightT>::promoted_scalar
perp_dot(const LeftT& left, const RightT& right)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_tag left_result;
    typedef typename right_traits::result_tag right_result;

    /* perp_dot() requires vector expressions: */
    CML_STATIC_REQUIRE_M(
        (same_type<left_result, et::vector_result_tag>::is_true
         && same_type<right_result, et::vector_result_tag>::is_true),
        perp_dot_expects_vector_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas.
     */

    /* Make sure arguments are 2D vectors: */
    detail::Require2D(left, typename left_traits::size_tag());
    detail::Require2D(right, typename right_traits::size_tag());

    /* Get result type: */
    typedef typename detail::DotPromote<
        LeftT,RightT>::promoted_scalar result_type;

    /* Compute and return: */
    return result_type(left[0]*right[1]-left[1]*right[0]);
}

template<typename LeftT, typename RightT>
inline typename detail::CrossPromote<LeftT,RightT>::promoted_vector
cross(const LeftT& left, const RightT& right)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_tag left_result;
    typedef typename right_traits::result_tag right_result;

    /* outer() requires vector expressions: */
    CML_STATIC_REQUIRE_M(
        (same_type<left_result, et::vector_result_tag>::is_true
         && same_type<right_result, et::vector_result_tag>::is_true),
        cross_expects_vector_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas.
     */

    /* Make sure arguments are 3D vectors: */
    detail::Require3D(left, typename left_traits::size_tag());
    detail::Require3D(right, typename right_traits::size_tag());

    /* Get result type: */
    typedef typename detail::CrossPromote<
        LeftT,RightT>::promoted_vector result_type;

    /* Now, compute and return the cross product: */
    result_type result(
            left[1]*right[2] - left[2]*right[1],
            left[2]*right[0] - left[0]*right[2],
            left[0]*right[1] - left[1]*right[0]
            );
    return result;
}

/** Return the triple product of three 3D vectors.
 *
 * No checking is done here, as dot() and cross() will catch any size or
 * type errors.
 */

template < class VecT_1, class VecT_2, class VecT_3 >
typename detail::DotPromote<
    VecT_1, typename detail::CrossPromote< VecT_2, VecT_3 >::promoted_vector
>::promoted_scalar
triple_product(const VecT_1& v1, const VecT_2& v2, const VecT_3& v3) {
    return dot(v1,cross(v2,v3));
}

template<typename LeftT, typename RightT>
inline typename detail::OuterPromote<LeftT,RightT>::promoted_matrix
outer(const LeftT& left, const RightT& right)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_tag left_result;
    typedef typename right_traits::result_tag right_result;

    /* outer() requires vector expressions: */
    CML_STATIC_REQUIRE_M(
        (same_type<left_result, et::vector_result_tag>::is_true
         && same_type<right_result, et::vector_result_tag>::is_true),
        dot_expects_vector_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas.
     */

    /* Create a matrix with the right size (resize() is a no-op for
     * fixed-size matrices):
     */
    typename detail::OuterPromote<LeftT,RightT>::promoted_matrix C;
    cml::et::detail::Resize(C, left.size(), right.size());

    /* Now, compute the outer product: */
    for(size_t i = 0; i < left.size(); ++i) {
        for(size_t j = 0; j < right.size(); ++j) {
            C(i,j) = left[i]*right[j];
            /* Note: both arguments are vectors, so array notation
             * is okay here.
             */
        }
    }

    return C;
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
