/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Default values for certain parameters.
 */

#ifndef defaults_h
#define defaults_h

#if defined(_MSC_VER)

#if _MSC_VER >= 1400

/* Ignore "C4003: not enough actual parameters for macro": */
#pragma warning (disable: 4003)

/* This one is odd, but apparently harmless (but should be fixed!):
 * "C4348: redefinition of default parameter"
 */
#pragma warning (disable: 4348)

#endif

#endif

/* The default vector unroll limit: */
#if !defined(CML_VECTOR_UNROLL_LIMIT)
#define CML_VECTOR_UNROLL_LIMIT 8
#endif

/* Don't unroll matrix operations by default: */
#if !defined(CML_2D_UNROLLER) && !defined(CML_NO_2D_UNROLLER)
#define CML_NO_2D_UNROLLER
#endif

/* The default vector dot() unroll limit: */
#if !defined(CML_VECTOR_DOT_UNROLL_LIMIT)
#define CML_VECTOR_DOT_UNROLL_LIMIT CML_VECTOR_UNROLL_LIMIT
#endif

/* The default array layout is the C/C++ row-major array layout: */
#if !defined(CML_DEFAULT_ARRAY_LAYOUT)
#define CML_DEFAULT_ARRAY_LAYOUT cml::row_major
#endif

/* The default basis orientation: */
#if !defined(CML_DEFAULT_BASIS_ORIENTATION)
#define CML_DEFAULT_BASIS_ORIENTATION cml::col_basis
#endif

/* Always use the default layout in promotions, by default: */
#if !defined(CML_ALWAYS_PROMOTE_TO_DEFAULT_LAYOUT)
#define CML_ALWAYS_PROMOTE_TO_DEFAULT_LAYOUT
#endif

/* The default memory allocator is std::allocator<void>: */
#if !defined(CML_DEFAULT_ARRAY_ALLOC)
#include <memory>               // for std::allocator
#define CML_DEFAULT_ARRAY_ALLOC std::allocator<void>
#endif

/* By default, automatically resize dynamic vectors and matrices: */
#if !defined(CML_AUTOMATIC_VECTOR_RESIZE_ON_ASSIGNMENT)
#define CML_AUTOMATIC_VECTOR_RESIZE_ON_ASSIGNMENT
#endif

#if !defined(CML_AUTOMATIC_MATRIX_RESIZE_ON_ASSIGNMENT)
#define CML_AUTOMATIC_MATRIX_RESIZE_ON_ASSIGNMENT
#endif

/* By default, check vector and matrix sizes: */
#if !defined(CML_CHECK_VECTOR_EXPR_SIZES)
#define CML_CHECK_VECTOR_EXPR_SIZES
#endif

#if !defined(CML_CHECK_MATRIX_EXPR_SIZES)
#define CML_CHECK_MATRIX_EXPR_SIZES
#endif

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
