/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * Forward declarations, useful to avoid including lots of headers.
 *
 * @sa cml/et/array_promotions.h
 */

#ifndef core_fwd_h
#define core_fwd_h

namespace cml {

/* cml/core/fixed_1D.h */
template<typename E, int S> class fixed_1D;

/* cml/core/fixed_2D.h */
template<typename E, int R, int C, class L> class fixed_2D;

/* cml/core/dynamic_1D.h */
template<typename E, class A> class dynamic_1D;

/* cml/core/dynamic_2D.h */
template<typename E, class L, class A> class dynamic_2D;

/* cml/core/external_1D.h */
template<typename E, int S> class external_1D;

/* cml/core/external_2D.h */
template<typename E, int R, int C, class L> class external_2D;

/* cml/fixed.h */
template<int Dim1, int Dim2> struct fixed;

/* cml/dynamic.h */
template<class Alloc> struct dynamic;

/* cml/external.h */
template<int Dim1, int Dim2> struct external;

/* cml/vector.h */
template<typename E, class AT> class vector;

/* cml/matrix.h */
template<typename E, class AT, class BO, class L> class matrix;

/* cml/quaternion.h */
template<typename E, class AT, class OT, class CT> class quaternion;

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
