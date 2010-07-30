/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef traits_h
#define traits_h

#include <cml/et/tags.h>

/* XXX This is here temporarily, should be rolled into the traits classes
 * once it's clear how to best specify scalar args
 */
//#define SCALAR_ARG_TYPE const ScalarT&
//#define ELEMENT_ARG_TYPE const Element&
#define SCALAR_ARG_TYPE ScalarT
#define ELEMENT_ARG_TYPE Element

namespace cml {
namespace et {

/** The expression traits class.
 *
 * The traits class is used to provide uniform access to expression
 * objects, including scalars, when used in vector and matrix expressions.
 * One especially useful property for scalars is that scalars are
 * implicitly "promoted" to vectors or scalars as necessary via the
 * ExprTraits's get() method.  Without this functionality, a separate
 * expression tree node would be needed to hold a scalar, which would
 * adversely affect performance.
 *
 * @internal This is also currently used for determining traits of scalar
 * types from the scalar operators (+,-,etc.).  Really, a separate traits
 * class should probably be used for this (e.g. ScalarTraits).
 */
template<typename T> struct ExprTraits
#if defined(CML_NO_DEFAULT_EXPR_TRAITS)
/* For testing, don't default to scalar traits: */
#else
{
    /* Standard: */
    typedef T expr_type;
    typedef T value_type;
    typedef T& reference;
    typedef T const_reference;
    typedef scalar_result_tag result_tag;
    typedef fixed_memory_tag memory_tag;
    typedef unit_size_tag size_tag;
    typedef expr_type result_type;
    typedef expr_leaf_tag node_tag;

    /** Vector-like access, just returns the value. */
    value_type get(const_reference v, size_t) const { return v; }

    /** Matrix-like access, just returns the value. */
    value_type get(const_reference v, size_t, size_t) const { return v; }

    /** Size is always 1. */
    size_t size(const_reference) const { return 1; }

    /** Size is always 1. */
    size_t rows(double) const { return 1; }

    /** Size is always 1. */
    size_t cols(double) const { return 1; }
}
#endif
;

#if defined(CML_NO_DEFAULT_EXPR_TRAITS)
template<> struct ExprTraits<double>
{
    /* Standard: */
    typedef double expr_type;
    typedef double value_type;
    typedef double& reference;
    typedef double const_reference;
    typedef scalar_result_tag result_tag;
    typedef fixed_memory_tag memory_tag;
    typedef unit_size_tag size_tag;
    typedef double result_type;
    typedef expr_leaf_tag node_tag;

    /** Vector-like access, just returns the value. */
    value_type get(double v, size_t) const { return v; }

    /** Matrix-like access, just returns the value. */
    value_type get(double v, size_t, size_t) const { return v; }

    /** Size is always 1. */
    size_t size(double) const { return 1; }

    /** Size is always 1. */
    size_t rows(double) const { return 1; }

    /** Size is always 1. */
    size_t cols(double) const { return 1; }
};

template<> struct ExprTraits<float>
{
    /* Standard: */
    typedef float expr_type;
    typedef float value_type;
    typedef float& reference;
    typedef float const_reference;
    typedef scalar_result_tag result_tag;
    typedef fixed_memory_tag memory_tag;
    typedef unit_size_tag size_tag;
    typedef float result_type;
    typedef expr_leaf_tag node_tag;

    /** Vector-like access, just returns the value. */
    value_type get(float v, size_t) const { return v; }

    /** Matrix-like access, just returns the value. */
    value_type get(float v, size_t, size_t) const { return v; }

    /** Size is always 1. */
    size_t size(float) const { return 1; }

    /** Size is always 1. */
    size_t rows(float) const { return 1; }

    /** Size is always 1. */
    size_t cols(float) const { return 1; }
};
#endif

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
