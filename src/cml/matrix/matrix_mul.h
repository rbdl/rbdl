/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Multiply two matrices.
 *
 * @todo Does it make sense to put mat-mat multiplication as a node into the
 * expression tree?
 *
 * @internal This does not need to return an expression type, since the
 * temporary generation for the matrix result is handled automatically by the
 * compiler.  i.e. when used in an expression, the result is automatically
 * included in the expression tree as a temporary by the compiler.
 */

#ifndef	matrix_mul_h
#define	matrix_mul_h

#include <cml/et/size_checking.h>
#include <cml/matrix/matrix_expr.h>

/* This is used below to create a more meaningful compile-time error when
 * mul is not provided with matrix or MatrixExpr arguments:
 */
struct mul_expects_matrix_args_error;

/* This is used below to create a more meaningful compile-time error when
 * fixed-size arguments to mul() have the wrong size:
 */
struct mul_expressions_have_wrong_size_error;

namespace cml {
namespace detail {

/** Verify the sizes of the argument matrices for matrix multiplication.
 *
 * @returns a matrix_size containing the size of the resulting matrix.
 */
template<typename LeftT, typename RightT> inline matrix_size
MatMulCheckedSize(const LeftT&, const RightT&, fixed_size_tag)
{
    CML_STATIC_REQUIRE_M(
            ((size_t)LeftT::array_cols == (size_t)RightT::array_rows),
            mul_expressions_have_wrong_size_error);
    return matrix_size(LeftT::array_rows,RightT::array_cols);
}

/** Verify the sizes of the argument matrices for matrix multiplication.
 *
 * @returns a matrix_size containing the size of the resulting matrix.
 */
template<typename LeftT, typename RightT> inline matrix_size
MatMulCheckedSize(const LeftT& left, const RightT& right, dynamic_size_tag)
{
    matrix_size left_N = left.size(), right_N = right.size();
    et::GetCheckedSize<LeftT,RightT,dynamic_size_tag>()
        .equal_or_fail(left_N.second, right_N.first); /* cols,rows */
    return matrix_size(left_N.first, right_N.second); /* rows,cols */
}


/** Matrix multiplication.
 *
 * Computes C = A x B (O(N^3), non-blocked algorithm).
 */
template<class LeftT, class RightT>
inline typename et::MatrixPromote<
    typename et::ExprTraits<LeftT>::result_type,
    typename et::ExprTraits<RightT>::result_type
>::temporary_type
mul(const LeftT& left, const RightT& right)
{
    /* Shorthand: */
    typedef et::ExprTraits<LeftT> left_traits;
    typedef et::ExprTraits<RightT> right_traits;
    typedef typename left_traits::result_type left_result;
    typedef typename right_traits::result_type right_result;

    /* First, require matrix expressions: */
    CML_STATIC_REQUIRE_M(
            (et::MatrixExpressions<LeftT,RightT>::is_true),
            mul_expects_matrix_args_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas.
     */

    /* Deduce size type to ensure that a run-time check is performed if
     * necessary:
     */
    typedef typename et::MatrixPromote<
        typename left_traits::result_type,
        typename right_traits::result_type
    >::type result_type;
    typedef typename result_type::size_tag size_tag;

    /* Require that left has the same number of columns as right has rows.
     * This automatically checks fixed-size matrices at compile time, and
     * throws at run-time if the sizes don't match:
     */
    matrix_size N = detail::MatMulCheckedSize(left, right, size_tag());

    /* Create an array with the right size (resize() is a no-op for
     * fixed-size matrices):
     */
    result_type C;
    cml::et::detail::Resize(C, N);

    /* XXX Specialize this for fixed-size matrices: */
    typedef typename result_type::value_type value_type;
    for(size_t i = 0; i < left.rows(); ++i) {               /* rows */
        for(size_t j = 0; j < right.cols(); ++j) {          /* cols */
            value_type sum(left(i,0)*right(0,j));
            for(size_t k = 1; k < right.rows(); ++k) {
                sum += (left(i,k)*right(k,j));
            }
            C(i,j) = sum;
        }
    }

    return C;
}

} // namespace detail


/** operator*() for two matrices. */
template<typename E1, class AT1, typename L1,
         typename E2, class AT2, typename L2,
         typename BO>
inline typename et::MatrixPromote<
    matrix<E1,AT1,BO,L1>, matrix<E2,AT2,BO,L2>
>::temporary_type
operator*(const matrix<E1,AT1,BO,L1>& left,
          const matrix<E2,AT2,BO,L2>& right)
{
    return detail::mul(left,right);
}

/** operator*() for a matrix and a MatrixXpr. */
template<typename E, class AT, typename BO, typename L, typename XprT>
inline typename et::MatrixPromote<
    matrix<E,AT,BO,L>, typename XprT::result_type
>::temporary_type
operator*(const matrix<E,AT,BO,L>& left,
          const et::MatrixXpr<XprT>& right)
{
    /* Generate a temporary, and compute the right-hand expression: */
    typedef typename et::MatrixXpr<XprT>::temporary_type expr_tmp;
    expr_tmp tmp;
    cml::et::detail::Resize(tmp,right.rows(),right.cols());
    tmp = right;

    return detail::mul(left,tmp);
}

/** operator*() for a MatrixXpr and a matrix. */
template<typename XprT, typename E, class AT, typename BO, typename L>
inline typename et::MatrixPromote<
    typename XprT::result_type , matrix<E,AT,BO,L>
>::temporary_type
operator*(const et::MatrixXpr<XprT>& left,
          const matrix<E,AT,BO,L>& right)
{
    /* Generate a temporary, and compute the left-hand expression: */
    typedef typename et::MatrixXpr<XprT>::temporary_type expr_tmp;
    expr_tmp tmp;
    cml::et::detail::Resize(tmp,left.rows(),left.cols());
    tmp = left;

    return detail::mul(tmp,right);
}

/** operator*() for two MatrixXpr's. */
template<typename XprT1, typename XprT2>
inline typename et::MatrixPromote<
    typename XprT1::result_type, typename XprT2::result_type
>::temporary_type
operator*(const et::MatrixXpr<XprT1>& left,
          const et::MatrixXpr<XprT2>& right)
{
    /* Generate temporaries and compute expressions: */
    typedef typename et::MatrixXpr<XprT1>::temporary_type left_tmp;
    left_tmp ltmp;
    cml::et::detail::Resize(ltmp,left.rows(),left.cols());
    ltmp = left;

    typedef typename et::MatrixXpr<XprT2>::temporary_type right_tmp;
    right_tmp rtmp;
    cml::et::detail::Resize(rtmp,right.rows(),right.cols());
    rtmp = right;

    return detail::mul(ltmp,rtmp);
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
