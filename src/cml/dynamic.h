/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef dynamic_h
#define dynamic_h

#include <cml/defaults.h>

namespace cml {

/** This is a selector for dynamic 1D and 2D arrays.
 *
 * The dynamic<> struct has no implementation; it is used only to select a
 * 1D or 2D array type as the base class of a vector or matrix.
 *
 * @sa fixed
 * @sa external
 */
template<class Alloc = CML_DEFAULT_ARRAY_ALLOC> struct dynamic;

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
