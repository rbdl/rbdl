/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 *  The configurable vector<> class.
 */

#ifndef cml_vector_h
#define cml_vector_h

#include <cml/core/common.h>

namespace cml {

/** A configurable vector type.
 *
 * This class encapsulates the notion of a vector.  The ArrayType template
 * argument can be used to select the type of array to be used as internal
 * storage for a list of type Element.  The vector orientation determines
 * how vectors are used arithmetically in expressions; i.e. a*b, when a is
 * a row vector and b is a column vector, is the dot (inner) product, while
 * a*b, when a is a column vector and b is a row vector, is the matrix
 * (outer) product of a and b.
 *
 * @internal Unlike the previous version, this uses specializations to
 * better enable varied array and vector types. For example, with the
 * rebind method, it's difficult to support external<> vector types that
 * should not be assigned to.
 *
 * @internal All assignments to the vector should go through UnrollAssignment,
 * which ensures that the source expression and the destination vector have
 * the same size.  This is particularly important for dynamically-sized
 * vectors.
 */
template<typename Element, class ArrayType> class vector;

} // namespace cml

#include <cml/core/common.h>
#include <cml/util.h>

#include <cml/vector/vector_ops.h>
#include <cml/vector/vector_products.h>
#include <cml/vector/vector_functions.h>
#include <cml/vector/vector_comparison.h>
#include <cml/vector/vector_print.h>

#include <cml/vector/fixed.h>
#include <cml/vector/dynamic.h>
#include <cml/vector/external.h>

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
