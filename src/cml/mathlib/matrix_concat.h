/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_concat_h
#define matrix_concat_h

#include <cml/matrix/matrix_expr.h>

/* This will all most likely be abstracted away in a future version of the
 * CML. For now, this file provides support for functions that need to
 * concatenate transformation matrices in a basis-independent manner.
 *
 * @todo: The 2x2 and 3x3 versions of these functions are currently in
 * matrix_rotation.h. They should be moved here.
 */

namespace cml {
namespace detail {

/** A fixed-size temporary 4x4 matrix */
#define MAT_TEMP_4X4 matrix<         \
    typename et::ScalarPromote<      \
        typename MatT_1::value_type, \
        typename MatT_2::value_type  \
    >::type,                         \
    fixed<4,4>,                      \
    typename MatT_1::basis_orient,   \
    typename MatT_1::layout          \
>

template < class MatT_1, class MatT_2 > MAT_TEMP_4X4
matrix_concat_transforms_4x4(const MatT_1& m1, const MatT_2& m2, row_basis) {
    return m1*m2;
}

/** Concatenate two 3D col-basis rotation matrices in the order m1->m2 */
template < class MatT_1, class MatT_2 > MAT_TEMP_4X4
matrix_concat_transforms_4x4(const MatT_1& m1, const MatT_2& m2, col_basis) {
    return m2*m1;
}

/** Concatenate two 3D rotation matrices in the order m1->m2 */
template < class MatT_1, class MatT_2 > MAT_TEMP_4X4
matrix_concat_transforms_4x4(const MatT_1& m1, const MatT_2& m2) {
    return matrix_concat_transforms_4x4(m1,m2,typename MatT_1::basis_orient());
}

#undef MAT_TEMP_4x4

} // namespace detail
} // namespace cml

#endif
