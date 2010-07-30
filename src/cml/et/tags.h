/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef et_tags_h
#define et_tags_h

namespace cml {
namespace et {

/** Tag an expression as returning a scalar. */
struct scalar_result_tag {};

/** Tag an expression as returning a vector. */
struct vector_result_tag {};

/** Tag an expression as returning a matrix. */
struct matrix_result_tag {};

/** Tag an expression as returning a quaternion. */
struct quaternion_result_tag {};

/** Marker for unary expression ops. */
struct unary_expression {};

/** Marker for biary expression ops. */
struct binary_expression {};

/** Marker for expression tree operator nodes. */
struct expr_node_tag {};

/** Marker for expression tree terminals (leaves). */
struct expr_leaf_tag {};

/** Marker for assignable types. */
struct assignable_tag {};

/** Marker for assignable types. */
struct not_assignable_tag {};

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
