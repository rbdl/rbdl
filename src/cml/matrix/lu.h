/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Implements LU decomposition for square matrix expressions.
 *
 * @todo The LU implementation does not check for a zero diagonal entry
 * (implying that the input has no LU factorization).
 *
 * @todo Should also have a pivoting implementation.
 *
 * @todo need to throw a numeric error if the determinant of the matrix
 * given to lu(), lu_solve(), or inverse() is 0.
 *
 * @internal The implementation is the same for fixed- and dynamic-size
 * matrices.  It can be sped up for small matrices later.
 */

#ifndef lu_h
#define lu_h

#include <cml/et/size_checking.h>
#include <cml/matrix/matrix_expr.h>
#include <cml/matvec/matvec_promotions.h>

/* This is used below to create a more meaningful compile-time error when
 * lu is not provided with a matrix or MatrixExpr argument:
 */
struct lu_expects_a_matrix_arg_error;

/* This is used below to create a more meaningful compile-time error when
 * lu_inplace is not provided with an assignable matrix argument:
 */
struct lu_inplace_expects_an_assignable_matrix_arg_error;

namespace cml {
namespace detail {

/* Compute the LU decomposition in-place: */
template<class MatT> inline
void lu_inplace(MatT& A)
{
    /* Shorthand: */
    typedef et::ExprTraits<MatT> arg_traits;
    typedef typename arg_traits::result_tag arg_result;
    typedef typename arg_traits::assignable_tag arg_assignment;
    typedef typename arg_traits::size_tag size_tag;
    typedef typename arg_traits::value_type value_type;

    /* lu_inplace() requires an assignable matrix expression: */
    CML_STATIC_REQUIRE_M(
        (same_type<arg_result, et::matrix_result_tag>::is_true
         && same_type<arg_assignment, et::assignable_tag>::is_true),
        lu_inplace_expects_an_assignable_matrix_arg_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas.
     */

    /* Verify that the matrix is square, and get the size: */
    ssize_t N = (ssize_t) cml::et::CheckedSquare(A, size_tag());


    for(ssize_t k = 0; k < N-1; ++k) {
        /* XXX Should check if A(k,k) = 0! */
        for(ssize_t i = k+1; i < N; ++i) {
            value_type n = (A(i,k) /= A(k,k));
            for(ssize_t j = k+1; j < N; ++ j) {
                A(i,j) -= n*A(k,j);
            }
        }
    }
}

/* Compute the LU decomposition, and return a copy of the result: */
template<class MatT>
inline typename MatT::temporary_type
lu_copy(const MatT& M)
{
    /* Shorthand: */
    typedef et::ExprTraits<MatT> arg_traits;
    typedef typename arg_traits::result_tag arg_result;

    /* lu_with_copy() requires a matrix expression: */
    CML_STATIC_REQUIRE_M(
        (same_type<arg_result, et::matrix_result_tag>::is_true),
        lu_expects_a_matrix_arg_error);
    /* Note: parens are required here so that the preprocessor ignores the
     * commas.
     */

    /* Use the in-place LU function, and return the result: */
    typename MatT::temporary_type A;
    cml::et::detail::Resize(A,M.rows(),M.cols());
    A = M;
    lu_inplace(A);
    return A;
}

} // namespace detail

/** LU factorization for a matrix. */
template<typename E, class AT, typename BO, class L>
inline typename matrix<E,AT,BO,L>::temporary_type
lu(const matrix<E,AT,BO,L>& m)
{
    return detail::lu_copy(m);
}

/** LU factorization for a matrix expression. */
template<typename XprT>
inline typename et::MatrixXpr<XprT>::temporary_type
lu(const et::MatrixXpr<XprT>& e)
{
    return detail::lu_copy(e);
}

/** Solve y = LUx for x.
 *
 * This solves Lb = y for b by forward substitution, then Ux = b for x by
 * backward substitution.
 */
template<typename MatT, typename VecT> inline
typename et::MatVecPromote<MatT,VecT>::temporary_type
lu_solve(const MatT& LU, const VecT& b)
{
    /* Shorthand. */
    typedef et::ExprTraits<MatT> lu_traits;
    typedef typename et::MatVecPromote<MatT,VecT>::temporary_type vector_type;
    typedef typename vector_type::value_type value_type;

    /* Verify that the matrix is square, and get the size: */
    ssize_t N = (ssize_t) cml::et::CheckedSquare(
            LU, typename lu_traits::size_tag());

    /* Verify that the matrix and vector have compatible sizes: */
    et::CheckedSize(LU, b, typename vector_type::size_tag());

    /* Solve Ly = b for y by forward substitution.  The entries below the
     * diagonal of LU correspond to L, understood to be below a diagonal of
     * 1's:
     */
    vector_type y; cml::et::detail::Resize(y,N);
    for(ssize_t i = 0; i < N; ++i) {
        y[i] = b[i];
        for(ssize_t j = 0; j < i; ++j) {
            y[i] -= LU(i,j)*y[j];
        }
    }

    /* Solve Ux = y for x by backward substitution.  The entries at and above
     * the diagonal of LU correspond to U:
     */
    vector_type x; cml::et::detail::Resize(x,N);
    for(ssize_t i = N-1; i >= 0; --i) {
        x[i] = y[i];
        for(ssize_t j = i+1; j < N; ++j) {
            x[i] -= LU(i,j)*x[j];
        }
        x[i] /= LU(i,i);
    }

    /* Return x: */
    return x;
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
