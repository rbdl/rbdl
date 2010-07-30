/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * Define matrix and vector linear expression size-checking classes.
 */

#ifndef size_checking_h
#define size_checking_h

#include <stdexcept>
#include <cml/core/cml_meta.h>
#include <cml/core/cml_assert.h>
#include <cml/core/fwd.h>
#include <cml/et/traits.h>

#if defined(_MSC_VER) && _MSC_VER < 1400
#pragma warning(push)
#pragma warning(disable:4348)
// XXX This is a terrible hack for VC7.1, and should really be fixed by
// separating out the "impl" templates from GetCheckedSize.
#endif

/* This is used below to create a more meaningful compile-time error when
 * fixed-size vector arguments don't match at compile time:
 */
struct incompatible_expression_size_error;

/* This is used below to create a more meaningful compile-time error when a
 * function is not provided with a square matrix or MatrixExpr argument:
 */
struct square_matrix_arg_expected_error;

namespace cml {
namespace et {
namespace detail {

} // namespace detail

/* Forward declare for specialization below: */
template<typename LeftT, typename RightT, typename SizeT>
    struct GetCheckedSize;

/* Checking for fixed-size expression: */
template<typename LeftT, typename RightT>
struct GetCheckedSize<LeftT,RightT,fixed_size_tag>
{
    /* Record argument traits: */
    typedef ExprTraits<LeftT> left_traits;
    typedef ExprTraits<RightT> right_traits;

    /* Result types: */
    typedef typename left_traits::result_tag left_result;
    typedef typename right_traits::result_tag right_result;


    /* For specialization below: */
    template<typename LR, typename RR, class X = void> struct impl;

    /* Check for two matrices (linear operators only): */
    template<class X> struct impl<matrix_result_tag,matrix_result_tag,X> {
        typedef matrix_size size_type;
        CML_STATIC_REQUIRE_M(
                (size_t)LeftT::array_rows == (size_t)RightT::array_rows
                && (size_t)LeftT::array_cols == (size_t)RightT::array_cols,
                incompatible_expression_size_error);

        /* Record the array size as a constant: */
        enum {
            array_rows = LeftT::array_rows,
            array_cols = LeftT::array_cols
        };

        /* Return the matrix size: */
        size_type size() const { return size_type(array_rows,array_cols); }
    };

    /* Check for a matrix and a vector: */
    template<class X> struct impl<matrix_result_tag,vector_result_tag,X> {
        typedef size_t size_type;
        CML_STATIC_REQUIRE_M(
                (size_t)LeftT::array_cols == (size_t)RightT::array_size,
                incompatible_expression_size_error);

        /* Record the array size as a constant: */
        enum { array_size = LeftT::array_rows };

        /* Return the vector size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Check for a vector and a matrix: */
    template<class X> struct impl<vector_result_tag,matrix_result_tag,X> {
        typedef size_t size_type;
        CML_STATIC_REQUIRE_M(
                (size_t)LeftT::array_size == (size_t)RightT::array_rows,
                incompatible_expression_size_error);

        /* Record the array size as a constant: */
        enum { array_size = RightT::array_cols };

        /* Return the vector size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Check for a matrix and a scalar: */
    template<class X> struct impl<matrix_result_tag,scalar_result_tag,X> {
        typedef matrix_size size_type;

        /* Record the array size as a constant: */
        enum {
            array_rows = LeftT::array_rows,
            array_cols = LeftT::array_cols
        };

        /* Return the matrix size: */
        size_type size() const { return size_type(array_rows,array_cols); }
    };

    /* Check for a scalar and a matrix: */
    template<class X> struct impl<scalar_result_tag,matrix_result_tag,X> {
        typedef matrix_size size_type;

        /* Record the array size as a constant: */
        enum {
            array_rows = RightT::array_rows,
            array_cols = RightT::array_cols
        };

        /* Return the matrix size: */
        size_type size() const { return size_type(array_rows,array_cols); }
    };


    /* Check for two vectors: */
    template<class X> struct impl<vector_result_tag,vector_result_tag,X> {
        typedef size_t size_type;
        CML_STATIC_REQUIRE_M(
                (size_t)LeftT::array_size == (size_t)RightT::array_size,
                incompatible_expression_size_error);

        /* Record the array size as a constant: */
        enum { array_size = LeftT::array_size };

        /* Return the vector size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Check for a vector and a scalar: */
    template<class X> struct impl<vector_result_tag,scalar_result_tag,X> {
        typedef size_t size_type;

        /* Record the array size as a constant: */
        enum { array_size = LeftT::array_size };

        /* Return the vector size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Check for a scalar and a vector: */
    template<class X> struct impl<scalar_result_tag,vector_result_tag,X> {
        typedef size_t size_type;

        /* Record the array size as a constant: */
        enum { array_size = RightT::array_size };

        /* Return the vector size: */
        size_type size() const { return size_type(array_size); }
    };


    /* Check for two quaternions: */
    template<class X>
    struct impl<quaternion_result_tag,quaternion_result_tag,X> {
        typedef size_t size_type;

        /* Record the quaternion size as a constant: */
        enum { array_size = 4 };

        /* Return the quaternion size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Check for a quaternion and a vector: */
    template<class X> struct impl<quaternion_result_tag,vector_result_tag,X> {
        typedef size_t size_type;
        CML_STATIC_REQUIRE_M(
                RightT::array_size == 4,
                incompatible_expression_size_error);

        /* Record the quaternion size as a constant: */
        enum { array_size = 4 };

        /* Return the quaternion size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Check for a vector and a quaternion: */
    template<class X> struct impl<vector_result_tag,quaternion_result_tag,X> {
        typedef size_t size_type;
        CML_STATIC_REQUIRE_M(
                LeftT::array_size == 4,
                incompatible_expression_size_error);

        /* Record the quaternion size as a constant: */
        enum { array_size = 4 };

        /* Return the quaternion size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Check for a quaternion and a scalar: */
    template<class X> struct impl<quaternion_result_tag,scalar_result_tag,X> {
        typedef size_t size_type;

        /* Record the quaternion size as a constant: */
        enum { array_size = 4 };

        /* Return the quaternion size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Check for a scalar and a quaternion: */
    template<class X> struct impl<scalar_result_tag,quaternion_result_tag,X> {
        typedef size_t size_type;

        /* Record the array size as a constant: */
        enum { array_size = 4 };

        /* Return the quaternion size: */
        size_type size() const { return size_type(array_size); }
    };

    /* Record the type of the checker: */
    typedef impl<left_result,right_result> check_type;
    typedef typename check_type::size_type size_type;

    /* The implementation: */
    size_type operator()(const LeftT&, const RightT&) const {
        return check_type().size();
    }
};

/* Checking for resizeable expression: */
template<typename LeftT, typename RightT>
struct GetCheckedSize<LeftT,RightT,dynamic_size_tag>
{
    /* Type of the size checker (for calling equal_or_fail): */
    typedef GetCheckedSize<LeftT,RightT,dynamic_size_tag> self;

    /* Record argument traits: */
    typedef ExprTraits<LeftT> left_traits;
    typedef ExprTraits<RightT> right_traits;

    /* Result types: */
    typedef typename left_traits::result_tag left_result;
    typedef typename right_traits::result_tag right_result;


    /* For specialization below: */
    template<typename LR, typename RR, class X = void> struct impl;

    /* Return the size if the same, or fail if different: */
    template<typename V> V equal_or_fail(V left, V right) const {
        if(left != right)
            throw std::invalid_argument(
                    "expressions have incompatible sizes.");
        return left;
    }

    /* Check for two matrices (linear operators only): */
    template<class X> struct impl<matrix_result_tag,matrix_result_tag,X> {
        typedef matrix_size size_type;

        /* Return the matrix size, or fail if incompatible: */
        size_type size(const LeftT& left, const RightT& right) const {
#if defined(CML_CHECK_MATRIX_EXPR_SIZES)
            return self().equal_or_fail(left.size(), right.size());
#else
            return left.size();
#endif
        }
    };

    /* Check for a matrix and a vector: */
    template<class X> struct impl<matrix_result_tag,vector_result_tag,X> {
        typedef size_t size_type;

        /* Return the vector size: */
        size_type size(const LeftT& left, const RightT& right) const {
#if defined(CML_CHECK_MATVEC_EXPR_SIZES)
            self().equal_or_fail(left.cols(), right.size());
#endif
            return left.rows();
        }
    };

    /* Check for a vector and a matrix: */
    template<class X> struct impl<vector_result_tag,matrix_result_tag,X> {
        typedef size_t size_type;

        /* Return the vector size: */
        size_type size(const LeftT& left, const RightT& right) const {
#if defined(CML_CHECK_MATVEC_EXPR_SIZES)
            self().equal_or_fail(left.size(), right.rows());
#endif
            return right.cols(right);
        }
    };

    /* Check for a matrix and a scalar: */
    template<class X> struct impl<matrix_result_tag,scalar_result_tag,X> {
        typedef matrix_size size_type;

        /* Return the matrix size: */
        size_type size(const LeftT& left, const RightT&) const {
            return left.size();
        }
    };

    /* Check for a scalar and a matrix: */
    template<class X> struct impl<scalar_result_tag,matrix_result_tag,X> {
        typedef matrix_size size_type;

        /* Return the matrix size: */
        size_type size(const LeftT&, const RightT& right) const {
            return right.size();
        }
    };

    /* Check for two vectors: */
    template<class X> struct impl<vector_result_tag,vector_result_tag,X> {
        typedef size_t size_type;

        /* Return the vector size: */
        size_type size(const LeftT& left, const RightT& right) const {
#if defined(CML_CHECK_VECTOR_EXPR_SIZES)
            return self().equal_or_fail(left.size(), right.size());
#else
            return left.size();
#endif
        }
    };

    /* Check for a vector and a scalar: */
    template<class X> struct impl<vector_result_tag,scalar_result_tag,X> {
        typedef size_t size_type;

        /* Return the vector size: */
        size_type size(const LeftT& left, const RightT&) const {
            return left.size();
        }
    };

    /* Check for a scalar and a vector: */
    template<class X> struct impl<scalar_result_tag,vector_result_tag,X> {
        typedef size_t size_type;

        /* Return the vector size: */
        size_type size(const LeftT&, const RightT& right) const {
            return right.size();
        }
    };

    /* Record the type of the checker: */
    typedef impl<left_result,right_result> check_type;
    typedef typename check_type::size_type size_type;

    /* The implementation: */
    size_type operator()(const LeftT& left, const RightT& right) const {
        return check_type().size(left,right);
    }
};

/** Generator for GetCheckedSize. */
template<typename LeftT, typename RightT, typename SizeTag>
inline typename et::GetCheckedSize<LeftT,RightT,SizeTag>::size_type
CheckedSize(const LeftT& left, const RightT& right, SizeTag)
{
    return et::GetCheckedSize<LeftT,RightT,SizeTag>()(left,right);
}

/** Verify the sizes of the argument matrices for matrix multiplication.
 *
 * @returns a the size of the resulting matrix.
 */
template<typename MatT> inline size_t
CheckedSquare(const MatT&, fixed_size_tag)
{
    CML_STATIC_REQUIRE_M(
            ((size_t)MatT::array_rows == (size_t)MatT::array_cols),
            square_matrix_arg_expected_error);
    return (size_t)MatT::array_rows;
}

/** Verify the sizes of the argument matrices for matrix multiplication.
 *
 * @returns the size of the resulting matrix.
 */
template<typename MatT> inline size_t
CheckedSquare(const MatT& m, dynamic_size_tag)
{
    matrix_size N = m.size();
    et::GetCheckedSize<MatT,MatT,dynamic_size_tag>()
        .equal_or_fail(N.first, N.second);
    return N.first;
}

} // namespace et
} // namespace cml

#if defined(_MSC_VER) && _MSC_VER < 1400
#pragma warning(pop)
#endif

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
