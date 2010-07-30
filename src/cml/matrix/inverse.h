/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Compute the inverse of a matrix by LU factorization.
 */

#ifndef matrix_inverse_h
#define matrix_inverse_h

#include <vector>
#include <cml/matrix/lu.h>

namespace cml {
namespace detail {

/* Need to use a functional, since template functions cannot be
 * specialized. _tag is used to specialize based upon dimension:
 */
template<typename MatT, int _tag> struct inverse_f;

/* @todo: Reciprocal optimization for division by determinant.
 */

/* 2x2 inverse.  Despite being marked for fixed_size matrices, this can
 * be used for dynamic-sized ones also:
 */
template<typename MatT>
struct inverse_f<MatT,2>
{
    typename MatT::temporary_type operator()(const MatT& M) const
    {
        typedef typename MatT::temporary_type temporary_type;
        typedef typename temporary_type::value_type value_type;

        /* Matrix containing the inverse: */
        temporary_type Z;
        cml::et::detail::Resize(Z,2,2);

        /* Compute determinant and inverse: */
        value_type D = value_type(1) / (M(0,0)*M(1,1) - M(0,1)*M(1,0));
        Z(0,0) =   M(1,1)*D; Z(0,1) = - M(0,1)*D;
        Z(1,0) = - M(1,0)*D; Z(1,1) =   M(0,0)*D;

        return Z;
    }
};

/* 3x3 inverse.  Despite being marked for fixed_size matrices, this can
 * be used for dynamic-sized ones also:
 */
template<typename MatT>
struct inverse_f<MatT,3>
{
    /*     [00 01 02]
     * M = [10 11 12]
     *     [20 21 22]
     */
    typename MatT::temporary_type operator()(const MatT& M) const
    {
        /* Shorthand. */
        typedef typename MatT::value_type value_type;

        /* Compute cofactors for each entry: */
        value_type m_00 = M(1,1)*M(2,2) - M(1,2)*M(2,1);
        value_type m_01 = M(1,2)*M(2,0) - M(1,0)*M(2,2);
        value_type m_02 = M(1,0)*M(2,1) - M(1,1)*M(2,0);

        value_type m_10 = M(0,2)*M(2,1) - M(0,1)*M(2,2);
        value_type m_11 = M(0,0)*M(2,2) - M(0,2)*M(2,0);
        value_type m_12 = M(0,1)*M(2,0) - M(0,0)*M(2,1);

        value_type m_20 = M(0,1)*M(1,2) - M(0,2)*M(1,1);
        value_type m_21 = M(0,2)*M(1,0) - M(0,0)*M(1,2);
        value_type m_22 = M(0,0)*M(1,1) - M(0,1)*M(1,0);

        /* Compute determinant from the minors: */
        value_type D =
            value_type(1) / (M(0,0)*m_00 + M(0,1)*m_01 + M(0,2)*m_02);

        /* Matrix containing the inverse: */
        typename MatT::temporary_type Z;
        cml::et::detail::Resize(Z,3,3);

        /* Assign the inverse as (1/D) * (cofactor matrix)^T: */
        Z(0,0) = m_00*D;  Z(0,1) = m_10*D;  Z(0,2) = m_20*D;
        Z(1,0) = m_01*D;  Z(1,1) = m_11*D;  Z(1,2) = m_21*D;
        Z(2,0) = m_02*D;  Z(2,1) = m_12*D;  Z(2,2) = m_22*D;

        return Z;
    }
};

/* 4x4 inverse.  Despite being marked for fixed_size matrices, this can
 * be used for dynamic-sized ones also:
 */
template<typename MatT>
struct inverse_f<MatT,4>
{
    /*     [00 01 02 03]
     * M = [10 11 12 13]
     *     [20 21 22 23]
     *     [30 31 32 33]
     *
     *       |11 12 13|         |10 12 13|
     * C00 = |21 22 23|   C01 = |20 22 23|
     *       |31 32 33|         |30 32 33|
     *
     *       |10 11 13|         |10 11 12|
     * C02 = |20 21 23|   C03 = |20 21 22|
     *       |30 31 33|         |30 31 32|
     */
    typename MatT::temporary_type operator()(const MatT& M) const
    {
        /* Shorthand. */
        typedef typename MatT::value_type value_type;

        /* Common cofactors, rows 0,1: */
        value_type m_22_33_23_32 = M(2,2)*M(3,3) - M(2,3)*M(3,2);
        value_type m_23_30_20_33 = M(2,3)*M(3,0) - M(2,0)*M(3,3);
        value_type m_20_31_21_30 = M(2,0)*M(3,1) - M(2,1)*M(3,0);
        value_type m_21_32_22_31 = M(2,1)*M(3,2) - M(2,2)*M(3,1);
        value_type m_23_31_21_33 = M(2,3)*M(3,1) - M(2,1)*M(3,3);
        value_type m_20_32_22_30 = M(2,0)*M(3,2) - M(2,2)*M(3,0);

        /* Compute minors: */
        value_type d00
            = M(1,1)*m_22_33_23_32+M(1,2)*m_23_31_21_33+M(1,3)*m_21_32_22_31;

        value_type d01
            = M(1,0)*m_22_33_23_32+M(1,2)*m_23_30_20_33+M(1,3)*m_20_32_22_30;

        value_type d02
            = M(1,0)*-m_23_31_21_33+M(1,1)*m_23_30_20_33+M(1,3)*m_20_31_21_30;

        value_type d03
            = M(1,0)*m_21_32_22_31+M(1,1)*-m_20_32_22_30+M(1,2)*m_20_31_21_30;

        /* Compute minors: */
        value_type d10
            = M(0,1)*m_22_33_23_32+M(0,2)*m_23_31_21_33+M(0,3)*m_21_32_22_31;

        value_type d11
            = M(0,0)*m_22_33_23_32+M(0,2)*m_23_30_20_33+M(0,3)*m_20_32_22_30;

        value_type d12
            = M(0,0)*-m_23_31_21_33+M(0,1)*m_23_30_20_33+M(0,3)*m_20_31_21_30;

        value_type d13
            = M(0,0)*m_21_32_22_31+M(0,1)*-m_20_32_22_30+M(0,2)*m_20_31_21_30;

        /* Common cofactors, rows 2,3: */
        value_type m_02_13_03_12 = M(0,2)*M(1,3) - M(0,3)*M(1,2);
        value_type m_03_10_00_13 = M(0,3)*M(1,0) - M(0,0)*M(1,3);
        value_type m_00_11_01_10 = M(0,0)*M(1,1) - M(0,1)*M(1,0);
        value_type m_01_12_02_11 = M(0,1)*M(1,2) - M(0,2)*M(1,1);
        value_type m_03_11_01_13 = M(0,3)*M(1,1) - M(0,1)*M(1,3);
        value_type m_00_12_02_10 = M(0,0)*M(1,2) - M(0,2)*M(1,0);

        /* Compute minors (uses row 3 as the multipliers instead of row 0,
         * which uses the same signs as row 0):
         */
        value_type d20
            = M(3,1)*m_02_13_03_12+M(3,2)*m_03_11_01_13+M(3,3)*m_01_12_02_11;

        value_type d21
            = M(3,0)*m_02_13_03_12+M(3,2)*m_03_10_00_13+M(3,3)*m_00_12_02_10;

        value_type d22
            = M(3,0)*-m_03_11_01_13+M(3,1)*m_03_10_00_13+M(3,3)*m_00_11_01_10;

        value_type d23
            = M(3,0)*m_01_12_02_11+M(3,1)*-m_00_12_02_10+M(3,2)*m_00_11_01_10;

        /* Compute minors: */
        value_type d30
            = M(2,1)*m_02_13_03_12+M(2,2)*m_03_11_01_13+M(2,3)*m_01_12_02_11;

        value_type d31
            = M(2,0)*m_02_13_03_12+M(2,2)*m_03_10_00_13+M(2,3)*m_00_12_02_10;

        value_type d32
            = M(2,0)*-m_03_11_01_13+M(2,1)*m_03_10_00_13+M(2,3)*m_00_11_01_10;

        value_type d33
            = M(2,0)*m_01_12_02_11+M(2,1)*-m_00_12_02_10+M(2,2)*m_00_11_01_10;

        /* Finally, compute determinant from the minors, and assign the
         * inverse as (1/D) * (cofactor matrix)^T:
         */
        typename MatT::temporary_type Z;
        cml::et::detail::Resize(Z,4,4);

        value_type D = value_type(1) /
            (M(0,0)*d00 - M(0,1)*d01 + M(0,2)*d02 - M(0,3)*d03);
        Z(0,0) = +d00*D; Z(0,1) = -d10*D; Z(0,2) = +d20*D; Z(0,3) = -d30*D;
        Z(1,0) = -d01*D; Z(1,1) = +d11*D; Z(1,2) = -d21*D; Z(1,3) = +d31*D;
        Z(2,0) = +d02*D; Z(2,1) = -d12*D; Z(2,2) = +d22*D; Z(2,3) = -d32*D;
        Z(3,0) = -d03*D; Z(3,1) = +d13*D; Z(3,2) = -d23*D; Z(3,3) = +d33*D;

        return Z;
    }
};

/* If more extensive general linear algebra functionality is offered in
 * future versions it may be useful to make the elementary row and column
 * operations separate functions. For now they're simply performed in place,
 * but the commented-out lines of code show where the calls to these functions
 * should go if and when they become available.
 */
 
/* @todo: In-place version, and address memory allocation for pivot vector.
 */

/* General NxN inverse by Gauss-Jordan elimination with full pivoting:  */
template<typename MatT, int _tag>
struct inverse_f
{
    typename MatT::temporary_type operator()(const MatT& M) const
    {
        /* Shorthand. */
        typedef typename MatT::value_type value_type;
        
        /* Size of matrix */
        size_t N = M.rows();

        /* Matrix containing the inverse: */
        typename MatT::temporary_type Z;
        cml::et::detail::Resize(Z,N,N);
        Z = M;

        /* For tracking pivots */
        std::vector<size_t> row_index(N);
        std::vector<size_t> col_index(N);
        std::vector<size_t> pivoted(N,0);

        /* For each column */
        for (size_t i = 0; i < N; ++i) {
        
            /* Find the pivot */
            size_t row = 0, col = 0;
            value_type max = value_type(0);
            for (size_t j = 0; j < N; ++j) {
                if (!pivoted[j]) {
                    for (size_t k = 0; k < N; ++k) {
                        if (!pivoted[k]) {
                            value_type mag = std::fabs(Z(j,k));
                            if (mag > max) {
                                max = mag;
                                row = j;
                                col = k;
                            }
                        }
                    }
                }
            }

            /* TODO: Check max against epsilon here to catch singularity */

            row_index[i] = row;
            col_index[i] = col;

            /* Swap rows if necessary */
            if (row != col) {
                /*Z.row_op_swap(row,col);*/
                for (size_t j = 0; j < Z.cols(); ++j) {
                    std::swap(Z(row,j),Z(col,j));
                }
            }
            
            /* Process pivot row */
            pivoted[col] = true;
            value_type pivot = Z(col,col);
            Z(col,col) = value_type(1);
            /*Z.row_op_mult(col,value_type(1)/pivot);*/
            value_type k = value_type(1)/pivot;
            for (size_t j = 0; j < Z.cols(); ++j) {
                Z(col,j) *= k;
            }

            /* Process other rows */
            for (size_t j = 0; j < N; ++j) {
                if (j != col) {
                    value_type mult = -Z(j,col);
                    Z(j,col) = value_type(0);
                    /*Z.row_op_add_mult(col,j,mult);*/
                    for (size_t k = 0; k < Z.cols(); ++k) {
                        Z(j,k) += mult * Z(col,k);
                    }
                }
            }
        }

        /* Swap columns if necessary */
        for (int i = N-1; i >= 0; --i) {
            if (row_index[i] != col_index[i]) {
                /*Z.col_op_swap(row_index[i],col_index[i]);*/
                for (size_t j = 0; j < Z.rows(); ++j) {
                    std::swap(Z(j,row_index[i]),Z(j,col_index[i]));
                }
            }
        }

        /* Return result */
        return Z;
    }
};

/* Inversion by LU factorization is turned off for now due to lack of
 * pivoting in the implementation, but we may switch back to it at some future
 * time.
 */
 
#if 0

/* General NxN inverse by LU factorization:  */
template<typename MatT, int _tag>
struct inverse_f
{
    typename MatT::temporary_type operator()(const MatT& M) const
    {
        /* Shorthand. */
        typedef typename MatT::value_type value_type;

        /* Compute LU factorization: */
        size_t N = M.rows();
        typename MatT::temporary_type LU;
        cml::et::detail::Resize(LU,N,N);
        LU = lu(M);

        /* Matrix containing the inverse: */
        typename MatT::temporary_type Z;
        cml::et::detail::Resize(Z,N,N);

        typename MatT::col_vector_type v, x;
        cml::et::detail::Resize(v,N);
        cml::et::detail::Resize(x,N);
        for(size_t i = 0; i < N; ++i)
            v[i] = value_type(0);
        /* XXX Need a fill() function here. */

        /* Use lu_solve to solve M*x = v for x, where v = [0 ... 1 ... 0]^T: */
        for(size_t i = 0; i < N; ++i) {
            v[i] = 1.;
            x = lu_solve(LU,v);

            /* x is column i of the inverse of LU: */
            for(size_t k = 0; k < N; ++ k) {
                Z(k,i) = x[k];
            }
            v[i] = 0.;
        }

        return Z;
    }

};

#endif

/* Note: force_NxN is for checking general NxN inversion against the special-
 * case 2x2, 3x3 and 4x4 code. I'm leaving it in for now since we may need to
 * test the NxN code further if the implementation changes. At some future
 * time when the implementation is stable, everything related to force_NxN can
 * be taken out.
 */

/* Note: Commenting the force_NxN stuff out, but leaving the code here in
 * case we need to do more testing in the future.
 */

/* Generator for the inverse functional for fixed-size matrices: */
template<typename MatT> typename MatT::temporary_type
inverse(const MatT& M, fixed_size_tag/*, bool force_NxN*/)
{
    /* Require a square matrix: */
    cml::et::CheckedSquare(M, fixed_size_tag());
    
    /*
    if (force_NxN) {
        return inverse_f<MatT,0>()(M);
    } else {
    */
    return inverse_f<MatT,MatT::array_rows>()(M);
    /*
    }
    */
}

/* Generator for the inverse functional for dynamic-size matrices: */
template<typename MatT> typename MatT::temporary_type
inverse(const MatT& M, dynamic_size_tag/*, bool force_NxN*/)
{
    /* Require a square matrix: */
    cml::et::CheckedSquare(M, dynamic_size_tag());
    
    /*
    if (force_NxN) { 
        return inverse_f<MatT,0>()(M);
    } else {
    */
    /* Dispatch based upon the matrix dimension: */
    switch(M.rows()) {
        case 2:  return inverse_f<MatT,2>()(M);     //   2x2
        case 3:  return inverse_f<MatT,3>()(M);     //   3x3
        case 4:  return inverse_f<MatT,4>()(M);     //   4x4
        default: return inverse_f<MatT,0>()(M);     // > 4x4 (or 1x1)
    }
    /*
    }
    */
}

} // namespace detail

/** Inverse of a matrix. */
template<typename E, class AT, typename BO, typename L> inline
typename matrix<E,AT,BO,L>::temporary_type
inverse(const matrix<E,AT,BO,L>& M/*, bool force_NxN = false*/)
{
    typedef typename matrix<E,AT,BO,L>::size_tag size_tag;
    return detail::inverse(M,size_tag()/*,force_NxN*/);
}

/** Inverse of a matrix expression. */
template<typename XprT> inline
typename et::MatrixXpr<XprT>::temporary_type
inverse(const et::MatrixXpr<XprT>& e/*, bool force_NxN = false*/)
{
    typedef typename et::MatrixXpr<XprT>::size_tag size_tag;
    return detail::inverse(e,size_tag()/*,force_NxN*/);
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
