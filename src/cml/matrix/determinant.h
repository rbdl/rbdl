/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Compute the determinant of a square matrix using LU factorization.
 *
 *  @todo This should be specialized on the matrix size for small matrices.
 */

#ifndef determinant_h
#define determinant_h

#include <cml/matrix/lu.h>

namespace cml {
namespace detail {

/* Need to use a functional, since template functions cannot be
 * specialized.  N is used to differentiate dimension only, so this can be
 * used for any matrix size type:
 */
template<typename MatT, int N> struct determinant_f;

/* 2x2 determinant.  Despite being marked for fixed_size matrices, this can
 * be used for dynamic-sized ones also:
 */
template<typename MatT>
struct determinant_f<MatT,2>
{
    typename MatT::value_type operator()(const MatT& M) const
    {
        return M(0,0)*M(1,1) - M(1,0)*M(0,1);
    }

};

/* 3x3 determinant.  Despite being marked for fixed_size matrices, this can
 * be used for dynamic-sized ones also:
 */
template<typename MatT>
struct determinant_f<MatT,3>
{
    /*     [00 01 02]
     * M = [10 11 12]
     *     [20 21 22]
     */
    typename MatT::value_type operator()(const MatT& M) const
    {
        return M(0,0)*(M(1,1)*M(2,2) - M(1,2)*M(2,1))
             + M(0,1)*(M(1,2)*M(2,0) - M(1,0)*M(2,2))
             + M(0,2)*(M(1,0)*M(2,1) - M(1,1)*M(2,0));
    }

};

/* 4x4 determinant.  Despite being marked for fixed_size matrices, this can
 * be used for dynamic-sized ones also:
 */
template<typename MatT>
struct determinant_f<MatT,4>
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
     *
     * d00 =   11 * (22*33 - 23*32)  d01 =   10 * (22*33 - 23*32)
     *       + 12 * (23*31 - 21*33)        + 12 * (23*30 - 20*33)
     *       + 13 * (21*32 - 22*31)        + 13 * (20*32 - 22*30)
     *
     * d02 =   10 * (21*33 - 23*31)  d03 =   10 * (21*32 - 22*31)
     *       + 11 * (23*30 - 20*33)        + 11 * (22*30 - 20*32)
     *       + 13 * (20*31 - 21*30)        + 12 * (20*31 - 21*30)
     */
    typename MatT::value_type operator()(const MatT& M) const
    {
        /* Shorthand. */
        typedef typename MatT::value_type value_type;

        /* Common cofactors: */
        value_type m_22_33_23_32 = M(2,2)*M(3,3) - M(2,3)*M(3,2);
        value_type m_23_30_20_33 = M(2,3)*M(3,0) - M(2,0)*M(3,3);
        value_type m_20_31_21_30 = M(2,0)*M(3,1) - M(2,1)*M(3,0);
        value_type m_21_32_22_31 = M(2,1)*M(3,2) - M(2,2)*M(3,1);
        value_type m_23_31_21_33 = M(2,3)*M(3,1) - M(2,1)*M(3,3);
        value_type m_20_32_22_30 = M(2,0)*M(3,2) - M(2,2)*M(3,0);

        value_type d00 = M(0,0)*(
                M(1,1) * m_22_33_23_32
              + M(1,2) * m_23_31_21_33
              + M(1,3) * m_21_32_22_31);

        value_type d01 = M(0,1)*(
                M(1,0) * m_22_33_23_32
              + M(1,2) * m_23_30_20_33
              + M(1,3) * m_20_32_22_30);

        value_type d02 = M(0,2)*(
                M(1,0) * - m_23_31_21_33
              + M(1,1) * m_23_30_20_33
              + M(1,3) * m_20_31_21_30);

        value_type d03 = M(0,3)*(
                M(1,0) * m_21_32_22_31
              + M(1,1) * - m_20_32_22_30
              + M(1,2) * m_20_31_21_30);

        return d00 - d01 + d02 - d03;
    }

};

/* General NxN determinant by LU factorization:  */
template<typename MatT, int N>
struct determinant_f
{
    typename MatT::value_type operator()(const MatT& M) const
    {
        /* Compute the LU factorization: */
        typename MatT::temporary_type LU = lu(M);

        /* The product of the diagonal entries is the determinant: */
        typename MatT::value_type det = LU(0,0);
        for(size_t i = 1; i < LU.rows(); ++ i)
            det *= LU(i,i);
        return det;
    }

};

/* Generator for the determinant functional for fixed-size matrices: */
template<typename MatT> typename MatT::value_type
determinant(const MatT& M, fixed_size_tag)
{
    /* Require a square matrix: */
    cml::et::CheckedSquare(M, fixed_size_tag());
    return determinant_f<MatT,MatT::array_rows>()(M);
}

/* Generator for the determinant functional for dynamic-size matrices: */
template<typename MatT> typename MatT::value_type
determinant(const MatT& M, dynamic_size_tag)
{
    /* Require a square matrix: */
    cml::et::CheckedSquare(M, dynamic_size_tag());

    /* Dispatch based upon the matrix dimension: */
    switch(M.rows()) {
        case 2:  return determinant_f<MatT,2>()(M);
        case 3:  return determinant_f<MatT,3>()(M);
        case 4:  return determinant_f<MatT,4>()(M);
        default: return determinant_f<MatT,0>()(M);     // > 4x4.
    }
}

} // namespace detail

/** Determinant of a matrix. */
template<typename E, class AT, class BO, class L> inline E
determinant(const matrix<E,AT,BO,L>& M)
{
    typedef typename matrix<E,AT,BO,L>::size_tag size_tag;
    return detail::determinant(M,size_tag());
}

/** Determinant of a matrix expression. */
template<typename XprT> inline typename XprT::value_type
determinant(const et::MatrixXpr<XprT>& e)
{
    typedef typename et::MatrixXpr<XprT>::size_tag size_tag;
    return detail::determinant(e,size_tag());
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
