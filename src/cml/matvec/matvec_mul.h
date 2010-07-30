/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Multiply a matrix and a vector.
 *
 * @todo Implement smarter temporary generation.
 *
 * @todo Does it make sense to put mat-vec multiplication as a node into the
 * expression tree?
 *
 * @internal This does not need to return an expression type, since the
 * temporary generation for the matrix result is handled automatically by the
 * compiler.  i.e. when used in an expression, the result is automatically
 * included in the expression tree as a temporary by the compiler.
 */

#ifndef	matvec_mul_h
#define	matvec_mul_h

#include <cml/core/cml_meta.h>
#include <cml/vector/vector_expr.h>
#include <cml/matrix/matrix_expr.h>
#include <cml/matvec/matvec_promotions.h>

/* This is used below to create a more meaningful compile-time error when
 * mat-vec mul is not provided with the right arguments:
 */
struct mvmul_expects_one_matrix_and_one_vector_arg_error;
struct mvmul_expects_one_vector_and_one_matrix_arg_error;

namespace cml {
namespace detail {

/* For choosing the proper multiplication order: */
typedef true_type mul_Ax;
typedef false_type mul_xA;

/** Compute y = A*x. */
template<typename LeftT, typename RightT> inline
typename et::MatVecPromote<
    typename et::ExprTraits<LeftT>::result_type,
    typename et::ExprTraits<RightT>::result_type
>::temporary_type
mul(const LeftT& A, const RightT& x, mul_Ax)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_tag left_result;
    typedef typename right_traits::result_tag right_result;

    /* mul()[A*x] requires a matrix and a vector expression: */
    CML_STATIC_REQUIRE_M(
        (same_type<left_result, et::matrix_result_tag>::is_true
         && same_type<right_result, et::vector_result_tag>::is_true),
        mvmul_expects_one_matrix_and_one_vector_arg_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas.
     */

    /* Get result type: */
    typedef typename et::MatVecPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::temporary_type result_type;

    /* Record size type: */
    typedef typename result_type::size_tag size_tag;

    /* Check the size: */
    size_t N = et::CheckedSize(A, x, size_tag());

    /* Initialize the new vector: */
    result_type y; cml::et::detail::Resize(y, N);

    /* Compute y = A*x: */
    typedef typename result_type::value_type sum_type;
    for(size_t i = 0; i < N; ++i) {
        /* XXX This should be unrolled. */
        sum_type sum(A(i,0)*x[0]);
        for(size_t k = 1; k < x.size(); ++k) {
            sum += (A(i,k)*x[k]);
        }
        y[i] = sum;
    }

    return y;
}

/** Compute y = x*A. */
template<typename LeftT, typename RightT> inline
typename et::MatVecPromote<
    typename et::ExprTraits<LeftT>::result_type,
    typename et::ExprTraits<RightT>::result_type
>::temporary_type
mul(const LeftT& x, const RightT& A, mul_xA)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_tag left_result;
    typedef typename right_traits::result_tag right_result;

    /* mul()[x*A] requires a vector and a matrix expression: */
    CML_STATIC_REQUIRE_M(
        (same_type<left_result, et::vector_result_tag>::is_true
         && same_type<right_result, et::matrix_result_tag>::is_true),
        mvmul_expects_one_vector_and_one_matrix_arg_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas.
     */

    /* Get result type: */
    typedef typename et::MatVecPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::temporary_type result_type;

    /* Record size type: */
    typedef typename result_type::size_tag size_tag;

    /* Check the size: */
    size_t N = et::CheckedSize(x, A, size_tag());

    /* Initialize the new vector: */
    result_type y; cml::et::detail::Resize(y, N);

    /* Compute y = x*A: */
    typedef typename result_type::value_type sum_type;
    for(size_t i = 0; i < N; ++i) {
        /* XXX This should be unrolled. */
        sum_type sum(x[0]*A(0,i));
        for(size_t k = 1; k < x.size(); ++k) {
            sum += (x[k]*A(k,i));
        }
        y[i] = sum;
    }

    return y;
}

} // namespace detail


/** operator*() for a matrix and a vector. */
template<typename E1, class AT1, typename BO, class L,
         typename E2, class AT2>
inline typename et::MatVecPromote<
    matrix<E1,AT1,BO,L>, vector<E2,AT2>
>::temporary_type
operator*(const matrix<E1,AT1,BO,L>& left,
          const vector<E2,AT2>& right)
{
    return detail::mul(left,right,detail::mul_Ax());
}

/** operator*() for a matrix and a VectorXpr. */
template<typename E, class AT, class L, typename BO, typename XprT>
inline typename et::MatVecPromote<
    matrix<E,AT,BO,L>, typename XprT::result_type
>::temporary_type
operator*(const matrix<E,AT,BO,L>& left,
          const et::VectorXpr<XprT>& right)
{
    /* Generate a temporary, and compute the right-hand expression: */
    typename et::VectorXpr<XprT>::temporary_type right_tmp;
    cml::et::detail::Resize(right_tmp,right.size());
    right_tmp = right;

    return detail::mul(left,right_tmp,detail::mul_Ax());
}

/** operator*() for a MatrixXpr and a vector. */
template<typename XprT, typename E, class AT>
inline typename et::MatVecPromote<
    typename XprT::result_type, vector<E,AT>
>::temporary_type
operator*(const et::MatrixXpr<XprT>& left,
          const vector<E,AT>& right)
{
    /* Generate a temporary, and compute the left-hand expression: */
    typename et::MatrixXpr<XprT>::temporary_type left_tmp;
    cml::et::detail::Resize(left_tmp,left.rows(),left.cols());
    left_tmp = left;

    return detail::mul(left_tmp,right,detail::mul_Ax());
}

/** operator*() for a MatrixXpr and a VectorXpr. */
template<typename XprT1, typename XprT2>
inline typename et::MatVecPromote<
    typename XprT1::result_type, typename XprT2::result_type
>::temporary_type
operator*(const et::MatrixXpr<XprT1>& left,
          const et::VectorXpr<XprT2>& right)
{
    /* Generate a temporary, and compute the left-hand expression: */
    typename et::MatrixXpr<XprT1>::temporary_type left_tmp;
    cml::et::detail::Resize(left_tmp,left.rows(),left.cols());
    left_tmp = left;

    /* Generate a temporary, and compute the right-hand expression: */
    typename et::VectorXpr<XprT2>::temporary_type right_tmp;
    cml::et::detail::Resize(right_tmp,right.size());
    right_tmp = right;

    return detail::mul(left_tmp,right_tmp,detail::mul_Ax());
}

/** operator*() for a vector and a matrix. */
template<typename E1, class AT1, typename E2, class AT2, typename BO, class L>
inline typename et::MatVecPromote<
    vector<E1,AT1>, matrix<E2,AT2,BO,L>
>::temporary_type
operator*(const vector<E1,AT1>& left,
          const matrix<E2,AT2,BO,L>& right)
{
    return detail::mul(left,right,detail::mul_xA());
}

/** operator*() for a vector and a MatrixXpr. */
template<typename XprT, typename E, class AT>
inline typename et::MatVecPromote<
    typename XprT::result_type, vector<E,AT>
>::temporary_type
operator*(const vector<E,AT>& left,
          const et::MatrixXpr<XprT>& right)
{
    /* Generate a temporary, and compute the right-hand expression: */
    typename et::MatrixXpr<XprT>::temporary_type right_tmp;
    cml::et::detail::Resize(right_tmp,right.rows(),right.cols());
    right_tmp = right;

    return detail::mul(left,right_tmp,detail::mul_xA());
}

/** operator*() for a VectorXpr and a matrix. */
template<typename XprT, typename E, class AT, typename BO, class L>
inline typename et::MatVecPromote<
    typename XprT::result_type, matrix<E,AT,BO,L>
>::temporary_type
operator*(const et::VectorXpr<XprT>& left,
          const matrix<E,AT,BO,L>& right)
{
    /* Generate a temporary, and compute the left-hand expression: */
    typename et::VectorXpr<XprT>::temporary_type left_tmp;
    cml::et::detail::Resize(left_tmp,left.size());
    left_tmp = left;

    return detail::mul(left_tmp,right,detail::mul_xA());
}

/** operator*() for a VectorXpr and a MatrixXpr. */
template<typename XprT1, typename XprT2>
inline typename et::MatVecPromote<
    typename XprT1::result_type, typename XprT2::result_type
>::temporary_type
operator*(const et::VectorXpr<XprT1>& left,
          const et::MatrixXpr<XprT2>& right)
{
    /* Generate a temporary, and compute the left-hand expression: */
    typename et::VectorXpr<XprT1>::temporary_type left_tmp;
    cml::et::detail::Resize(left_tmp,left.size());
    left_tmp = left;

    /* Generate a temporary, and compute the right-hand expression: */
    typename et::MatrixXpr<XprT2>::temporary_type right_tmp;
    cml::et::detail::Resize(right_tmp,right.rows(),right.cols());
    right_tmp = right;

    return detail::mul(left_tmp,right_tmp,detail::mul_xA());
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
