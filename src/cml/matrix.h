/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 *  The configurable matrix<> class.
 */

#ifndef cml_matrix_h
#define cml_matrix_h

#include <cml/core/common.h>

namespace cml {

/** A configurable matrix.
 *
 * This class encapsulates the notion of a matrix.  The ArrayType template
 * argument can be used to select the type of array to be used as internal
 * storage for a 2D array of type Element.
 *
 * @internal Unlike the previous version, this uses specializations to better
 * enable varied array and matrix types. For example, with the rebind method,
 * it's difficult to support external<> matrix types that should not be
 * assigned to.
 *
 * @internal All assignments to the matrix should go through UnrollAssignment,
 * which ensures that the source expression and the destination matrix have
 * the same size.  This is particularly important for dynamically-sized
 * matrices.
 */
template<typename Element, class ArrayType,
    typename BasisOrient = CML_DEFAULT_BASIS_ORIENTATION,
    typename Layout = CML_DEFAULT_ARRAY_LAYOUT> class matrix;

} // namespace cml

#include <cml/matrix/matrix_ops.h>
#include <cml/matrix/matrix_transpose.h>
#include <cml/matrix/matrix_rowcol.h>
#include <cml/matrix/matrix_mul.h>
#include <cml/matvec/matvec_mul.h>
#include <cml/matrix/matrix_functions.h>
#include <cml/matrix/matrix_comparison.h>
#include <cml/matrix/lu.h>
#include <cml/matrix/inverse.h>
#include <cml/matrix/determinant.h>
#include <cml/matrix/matrix_print.h>

#include <cml/matrix/fixed.h>
#include <cml/matrix/dynamic.h>
#include <cml/matrix/external.h>

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
