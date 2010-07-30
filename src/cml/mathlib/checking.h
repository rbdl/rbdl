/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef checking_h
#define checking_h

#include <cml/vector/vector_expr.h>
#include <cml/matrix/matrix_expr.h>
#include <cml/quaternion/quaternion_expr.h>

/* Run- and compile-time checking of argument types, values and sizes. */

struct function_expects_vector_arg_error;
struct function_expects_matrix_arg_error;
struct function_expects_quaternion_arg_error;

struct function_expects_2D_vector_arg_error;
struct function_expects_3D_vector_arg_error;
struct function_expects_4D_vector_arg_error;
struct function_expects_2D_or_3D_vector_arg_error;
struct function_expects_2x2_matrix_arg_error;
struct function_expects_3x3_matrix_arg_error;
struct function_expects_4x4_matrix_arg_error;
struct function_expects_square_matrix_arg_error;

struct matrix_arg_fails_minimum_size_requirement;

namespace cml {
namespace detail {

//////////////////////////////////////////////////////////////////////////////
// Vector argument checking
//////////////////////////////////////////////////////////////////////////////

/** Compile-time check for a vector argument */
template< class VecT > inline void
CheckVec(const VecT&)
{
    typedef et::ExprTraits<VecT> vector_traits;
    typedef typename vector_traits::result_tag result_type;
    
    CML_STATIC_REQUIRE_M(
        (same_type<result_type, et::vector_result_tag>::is_true),
        function_expects_vector_arg_error);
}

/** Compile-time check for a vector of size N */
template< class VecT, size_t N, class ErrorT > inline void
CheckVecN(const VecT& v, fixed_size_tag) {
    CheckVec(v);

    CML_STATIC_REQUIRE_M(((size_t)VecT::array_size == N), ErrorT);
}

/** Run-time check for a vector of size N */
template< class VecT, size_t N, class /*ErrorT*/ > inline void
CheckVecN(const VecT& v, dynamic_size_tag) {
    CheckVec(v);

    et::GetCheckedSize<VecT,VecT,dynamic_size_tag>()
        .equal_or_fail(v.size(),size_t(N));
}

/** Check for a vector of size N */
template< class VecT, size_t N, class ErrorT > inline void
CheckVecN(const VecT& v) {
    typedef et::ExprTraits<VecT> vector_traits;
    typedef typename vector_traits::size_tag size_tag;

    detail::CheckVecN<VecT,N,ErrorT>(v, size_tag());
}

/** Check for a vector of size 2 */
template< class VecT > inline void
CheckVec2(const VecT& v) {
    detail::CheckVecN<VecT,2,function_expects_2D_vector_arg_error>(v);
}

/** Check for a vector of size 3 */
template< class VecT > inline void
CheckVec3(const VecT& v) {
    detail::CheckVecN<VecT,3,function_expects_3D_vector_arg_error>(v);
}

/** Check for a vector of size 4 */
template< class VecT > inline void
CheckVec4(const VecT& v) {
    CheckVecN<VecT,4,function_expects_4D_vector_arg_error>(v);
}

/** Compile-time check for a vector of size 2 or 3 */
template< class VecT > inline void
CheckVec2Or3(const VecT& v, fixed_size_tag) {
    CheckVec(v);

    CML_STATIC_REQUIRE_M(
        (VecT::array_size == 2 || VecT::array_size == 3),
        function_expects_2D_or_3D_vector_arg_error);
}

/** Run-time check for a vector of size 2 or 3 */
template< class VecT > inline void
CheckVec2Or3(const VecT& v, dynamic_size_tag) {
    CheckVec(v);

    if (v.size() != 2 && v.size() != 3) {
        throw std::invalid_argument("2d or 3d vector arg expected");
    }
}

/** Check for a vector of size 2 or 3 */
template< class VecT > inline void
CheckVec2Or3(const VecT& v) {
    typedef et::ExprTraits<VecT> vector_traits;
    typedef typename vector_traits::size_tag size_tag;

    detail::CheckVec2Or3(v, size_tag());
}

//////////////////////////////////////////////////////////////////////////////
// Matrix argument checking
//////////////////////////////////////////////////////////////////////////////

/** Compile-time check for a matrix argument */
template< class MatT > inline void
CheckMat(const MatT&)
{
    typedef et::ExprTraits<MatT> matrix_traits;
    typedef typename matrix_traits::result_tag result_type;
    
    CML_STATIC_REQUIRE_M(
        (same_type<result_type, et::matrix_result_tag>::is_true),
        function_expects_matrix_arg_error);
}

/** Compile-time check for a matrix of size NxM */
template< class MatT, size_t N, size_t M, class ErrorT > inline void
CheckMatNxM(const MatT& m, fixed_size_tag) {
    CheckMat(m);

    CML_STATIC_REQUIRE_M(
        (MatT::array_rows == N && MatT::array_cols == M), ErrorT);
}

/** Run-time check for a matrix of size NxM */
template< class MatT, size_t N, size_t M, class /*ErrorT*/ > inline void
CheckMatNxM(const MatT& m, dynamic_size_tag) {
    CheckMat(m);

    et::GetCheckedSize<MatT,MatT,dynamic_size_tag>()
        .equal_or_fail(m.rows(),N);
    et::GetCheckedSize<MatT,MatT,dynamic_size_tag>()
        .equal_or_fail(m.cols(),M);
}

/** Check for a matrix of size NxM */
template< class MatT, size_t N, size_t M, class ErrorT > inline void
CheckMatNxM(const MatT& m) {
    typedef et::ExprTraits<MatT> matrix_traits;
    typedef typename matrix_traits::size_tag size_tag;

    CheckMatNxM<MatT,N,M,ErrorT>(m, size_tag());
}

/** Check for a square matrix of size NxN */
template< class MatT, size_t N, class ErrorT > inline void
CheckMatN(const MatT& m) {
    CheckMatNxM<MatT,N,N,ErrorT>(m);
}

/** Check for a square matrix of size 2x2 */
template< class MatT > inline void
CheckMat2x2(const MatT& m) {
    CheckMatN<MatT,2,function_expects_2x2_matrix_arg_error>(m);
}

/** Check for a square matrix of size 3x3 */
template< class MatT > inline void
CheckMat3x3(const MatT& m) {
    CheckMatN<MatT,3,function_expects_3x3_matrix_arg_error>(m);
}

/** Check for a square matrix of size 4x4 */
template< class MatT > inline void
CheckMat4x4(const MatT& m) {
    CheckMatN<MatT,4,function_expects_4x4_matrix_arg_error>(m);
}

/** Compile-time check for a matrix with minimum dimensions NxM */
template< class MatT, size_t N, size_t M, class ErrorT > inline void
CheckMatMinNxM(const MatT& m, fixed_size_tag) {
    CheckMat(m);

    CML_STATIC_REQUIRE_M(
        (MatT::array_rows >= N && MatT::array_cols >= M), ErrorT);
}

/** Run-time check for a matrix with minimum dimensions NxM */
template< class MatT, size_t N, size_t M, class /*ErrorT*/ > inline void
CheckMatMinNxM(const MatT& m, dynamic_size_tag) {
    CheckMat(m);

    if (m.rows() < N || m.cols() < M) {
        throw std::invalid_argument(
            "matrix does not meet minimum size requirement");
    }
}

/** Check for a matrix with minimum dimensions NxM */
template< class MatT, size_t N, size_t M, class ErrorT > inline void
CheckMatMinNxM(const MatT& m) {
    typedef et::ExprTraits<MatT> matrix_traits;
    typedef typename matrix_traits::size_tag size_tag;

    CheckMatMinNxM<MatT,N,M,ErrorT>(m, size_tag());
}

/** Check for a matrix with minimum dimensions NxN */
template< class MatT, size_t N, class ErrorT > inline void
CheckMatMinN(const MatT& m) {
    CheckMatMinNxM<MatT,N,N,ErrorT>(m);
}

/** Check for a matrix with minimum dimensions 2x2 */
template< class MatT > inline void
CheckMatMin2x2(const MatT& m) {
    CheckMatMinN<MatT,2,matrix_arg_fails_minimum_size_requirement>(m);
}

/** Check for a matrix with minimum dimensions 3x3 */
template< class MatT > inline void
CheckMatMin3x3(const MatT& m) {
    CheckMatMinN<MatT,3,matrix_arg_fails_minimum_size_requirement>(m);
}

/** Check for a matrix with minimum dimensions 4x4 */
template< class MatT > inline void
CheckMatMin4x4(const MatT& m) {
    CheckMatMinN<MatT,4,matrix_arg_fails_minimum_size_requirement>(m);
}

/** Check for a matrix that can represent a 3D linear transform */
template< class MatT > inline void
CheckMatLinear3D(const MatT& m) {
    CheckMatMin3x3(m);
}

/** Check for a matrix that can represent a 2D linear transform */
template< class MatT > inline void
CheckMatLinear2D(const MatT& m) {
    CheckMatMin2x2(m);
}

/** Check for a matrix that can represent a 3D row-basis affine transform */
template< class MatT > inline void
CheckMatAffine3D(const MatT& m, row_basis) {
    CheckMatMinNxM<MatT,4,3,matrix_arg_fails_minimum_size_requirement>(m);
}

/** Check for a matrix that can represent a 3D col-basis affine transform */
template< class MatT > inline void
CheckMatAffine3D(const MatT& m, col_basis) {
    CheckMatMinNxM<MatT,3,4,matrix_arg_fails_minimum_size_requirement>(m);
}

/** Check for a matrix that can represent a 2D row-basis affine transform */
template< class MatT > inline void
CheckMatAffine2D(const MatT& m, row_basis) {
    CheckMatMinNxM<MatT,3,2,matrix_arg_fails_minimum_size_requirement>(m);
}

/** Check for a matrix that can represent a 2D col-basis affine transform */
template< class MatT > inline void
CheckMatAffine2D(const MatT& m, col_basis) {
    CheckMatMinNxM<MatT,2,3,matrix_arg_fails_minimum_size_requirement>(m);
}

/** Check for a matrix that can represent a 3D affine transform */
template< class MatT > inline void
CheckMatAffine3D(const MatT& m) {
    CheckMatAffine3D(m, typename MatT::basis_orient());
}

/** Check for a matrix that can represent a 2D affine transform */
template< class MatT > inline void
CheckMatAffine2D(const MatT& m) {
    CheckMatAffine2D(m, typename MatT::basis_orient());
}

/** Check for a matrix that can represent a 3D homogenous transform */
template< class MatT > inline void
CheckMatHomogeneous3D(const MatT& m) {
    CheckMatMin4x4(m);
}

/** Compile-time check for a square matrix */
template< class MatT, class ErrorT> inline void
CheckMatSquare(const MatT& m, fixed_size_tag) {
    CheckMat(m);

    CML_STATIC_REQUIRE_M(
        (MatT::array_rows == MatT::array_cols), ErrorT);
}

/** Run-time check for a square matrix */
template< class MatT, class /*ErrorT*/ > inline void
CheckMatSquare(const MatT& m, dynamic_size_tag) {
    CheckMat(m);

    if (m.rows() != m.cols()) {
        throw std::invalid_argument(
            "function expects square matrix as argument");
    }
}

/** Check for a square matrix */
template< class MatT > inline void
CheckMatSquare(const MatT& m) {
    typedef et::ExprTraits<MatT> matrix_traits;
    typedef typename matrix_traits::size_tag size_tag;

    detail::CheckMatSquare<
        MatT,function_expects_square_matrix_arg_error>(m, size_tag());
}

//////////////////////////////////////////////////////////////////////////////
// Quaternion argument checking
//////////////////////////////////////////////////////////////////////////////

/** Compile-time check for a quaternion argument*/
template< class QuatT > inline void
CheckQuat(const QuatT& /*q*/)
{
    typedef et::ExprTraits<QuatT> quaternion_traits;
    typedef typename quaternion_traits::result_tag result_type;
    
    CML_STATIC_REQUIRE_M(
        (same_type<result_type, et::quaternion_result_tag>::is_true),
        function_expects_quaternion_arg_error);
}

//////////////////////////////////////////////////////////////////////////////
// Index argument checking
//////////////////////////////////////////////////////////////////////////////

/** Run-time check for a valid argument */
inline void CheckValidArg(bool valid)
{
    if (!valid) {
        throw std::invalid_argument("invalid function argument");
    }
}

/** Check for a valid integer index with value < N */
template < size_t N >
inline void CheckIndexN(size_t index) {
    CheckValidArg(index < N);
}

/** Check for a valid integer index with value < 2 */
inline void CheckIndex2(size_t index) {
    CheckIndexN<2>(index);
}

/** Check for a valid integer index with value < 3 */
inline void CheckIndex3(size_t index) {
    CheckIndexN<3>(index);
}

} // namespace detail
} // namespace cml

#endif
