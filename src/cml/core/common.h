/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef core_common_h
#define core_common_h

// XXX This isn't really the right place for this.
#if defined(_MSC_VER)
#include <cstdlib>
#ifndef _SSIZE_T_DEFINED
#ifdef  _WIN64
typedef __int64    ssize_t;
#else
typedef _W64 int   ssize_t;
#endif
#define _SSIZE_T_DEFINED
#endif
#endif

#include <cstddef>              // for size_t
#include <utility>              // for std::pair<>
#include <cml/defaults.h>

namespace cml {

/** 1D tag (to select array shape). */
struct oned_tag {};

/** 2D tag (to select array shape). */
struct twod_tag {};

/** Statically-allocated memory tag. */
struct fixed_memory_tag {};

/** Dynamically-allocated memory tag. */
struct dynamic_memory_tag {};

/** Externally-allocated memory tag. */
struct external_memory_tag {};

/** Statically-sized tag. */
struct fixed_size_tag {};

/** Runtime-sized tag. */
struct dynamic_size_tag {};

/** Resizable tag. */
struct resizable_tag {};

/** Not resizable tag. */
struct not_resizable_tag {};

/** Unit-sized tag. */
struct unit_size_tag {};

/** Row-major storage tag. */
struct row_major {};

/** Col-major storage tag. */
struct col_major {};

/** Row-vector matrix basis tag. */
struct row_basis {};

/** Column-vector matrix basis tag. */
struct col_basis {};

/* This is the pair returned from the matrix size() method, as well as from
 * the matrix expression size checking code:
 */
typedef std::pair<size_t,size_t> matrix_size;

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
