/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * Defines promotions for matrices used in matrix/matrix or matrix/scalar
 * expressions.
 *
 * @sa UnaryMat4_TOp
 * @sa BinaryMat4_TOp
 */

#ifndef matrix_promotions_h
#define matrix_promotions_h

#include <cml/core/cml_meta.h>
#include <cml/et/scalar_promotions.h>
#include <cml/et/array_promotions.h>
#include <cml/fixed.h>
#include <cml/dynamic.h>

/* This is used below to create a more meaningful compile-time error when
 * either argument to OuterPromote has the wrong orientation.
 */
struct outer_promote_expects_properly_oriented_args_error;

namespace cml {
namespace et {

/** Promote two types to a matrixt type. */
template<typename LeftT, typename RightT> struct MatrixPromote
{
    /* Default matrix type promotion template. */
    template<typename M1, typename M2> struct MatrixPromoteHelper;

    /** Type promotion for two matrix types.
     *
     * @note This always uses the basis orientation of the left-hand matrix.
     * @bug This always uses the basis orientation of the left-hand matrix,
     *      which is not always correct.
     */
    template<typename E1, class AT1, typename L1, typename BO1,
             typename E2, class AT2, typename L2, typename BO2>
    struct MatrixPromoteHelper<
    cml::matrix<E1,AT1,BO1,L1>, cml::matrix<E2,AT2,BO2,L2>
    >
    {
        /* Promote the arrays: */
        typedef typename ArrayPromote<
            typename cml::matrix<E1,AT1,BO1,L1>::array_type,
        typename cml::matrix<E2,AT2,BO2,L2>::array_type
            >::type promoted_array;

        /* The deduced matrix result type: */
        typedef cml::matrix<
            typename promoted_array::value_type,
                     typename promoted_array::generator_type,
                     BO1,
                     typename promoted_array::layout
                         > type;

        /* The deduced temporary type: */
        typedef typename type::temporary_type temporary_type;
    };

    /** Type promotion for a matrix and a scalar. */
    template<typename E, class AT, typename BO, typename L, typename S>
    struct MatrixPromoteHelper<cml::matrix<E,AT,BO,L>, S>
    {
        /* The deduced matrix result type (the array type is the same): */
        typedef cml::matrix<typename ScalarPromote<E,S>::type, AT, BO, L> type;

        /* The deduced temporary type: */
        typedef typename type::temporary_type temporary_type;
    };

    /** Type promotion for a scalar and a matrix. */
    template<typename S, typename E, class AT, typename BO, typename L>
    struct MatrixPromoteHelper<S, cml::matrix<E,AT,BO,L> >
    {
        /* The deduced matrix result type (the array type is the same): */
        typedef cml::matrix<typename ScalarPromote<S,E>::type, AT, BO, L> type;

        /* The deduced temporary type: */
        typedef typename type::temporary_type temporary_type;
    };

    /** Type promotion for outer product. */
    template<typename E1, class AT1, typename E2, class AT2>
    struct MatrixPromoteHelper< cml::vector<E1,AT1>, cml::vector<E2,AT2> >
    {
        typedef cml::vector<E1,AT1> left_type;
        typedef cml::vector<E2,AT2> right_type;
        typedef CML_DEFAULT_BASIS_ORIENTATION basis_orient;

        /* Get matrix size: */
        enum {
            array_rows = left_type::array_size,
            array_cols = right_type::array_size
        };

        /* Deduce the corresponding matrix types for the vectors: */
        typedef CML_DEFAULT_ARRAY_LAYOUT layout;
        typedef typename select_if<
            array_rows == -1, dynamic<>, fixed<array_rows,1>
            >::result left_storage;
        typedef cml::matrix<E1,left_storage,basis_orient,layout> left_matrix;

        typedef typename select_if<
            array_cols == -1, dynamic<>, fixed<1,array_cols>
            >::result right_storage;
        typedef cml::matrix<E2,right_storage,basis_orient,layout> right_matrix;

        /* Finally, promote the matrix types to get the result: */
        typedef typename et::MatrixPromote<left_matrix,right_matrix>::type type;
        typedef typename type::temporary_type temporary_type;
    };

    /** Remove const and & from the to-be-promoted types. */
    typedef typename remove_const<
        typename remove_reference<LeftT>::type>::type LeftBaseT;
    typedef typename remove_const<
        typename remove_reference<RightT>::type>::type RightBaseT;

    typedef typename MatrixPromoteHelper<LeftBaseT,RightBaseT>::type type;
    typedef typename type::temporary_type temporary_type;
};

/**
 * NOTE: MatrixPromote* are somewhat ad hoc, and were added to
 * simplify the code for matrix slerp/squad/etc.
 */

/** Type promotion for two matrix types. */
template < class Mat1_T, class Mat2_T >
struct MatrixPromote2
{
    typedef typename MatrixPromote<
        typename Mat1_T::temporary_type, typename Mat2_T::temporary_type
        >::temporary_type temporary_type;
    typedef typename temporary_type::value_type value_type;
};

/** Type promotion for three matrix types. */
template < class Mat1_T, class Mat2_T, class Mat3_T >
struct MatrixPromote3
{
    typedef typename MatrixPromote<
        typename Mat1_T::temporary_type,
        typename MatrixPromote<
            typename Mat2_T::temporary_type,
            typename Mat3_T::temporary_type
        >::temporary_type
     >::temporary_type temporary_type;
    typedef typename temporary_type::value_type value_type;
};

/** Type promotion for four matrix types. */
template < class Mat1_T, class Mat2_T, class Mat3_T, class Mat4_T >
struct MatrixPromote4
{
    typedef typename MatrixPromote<
        typename Mat1_T::temporary_type,
        typename MatrixPromote<
            typename Mat2_T::temporary_type,
            typename MatrixPromote<
                typename Mat3_T::temporary_type,
                typename Mat4_T::temporary_type
            >::temporary_type
         >::temporary_type
     >::temporary_type temporary_type;
    typedef typename temporary_type::value_type value_type;
};

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
