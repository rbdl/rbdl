/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef meta_if_h
#define meta_if_h

#include <cml/core/meta/common.h>

namespace cml {

/** Select argument type based upon truth value. */
template<bool yn, typename TrueT, typename FalseT> struct select_if;

/** Result is TrueT if true. */
template<typename TrueT, typename FalseT>
struct select_if<true,TrueT,FalseT> {
    typedef TrueT result;
    enum { is_true = true };
};

/** Result is FalseT if false. */
template<typename TrueT, typename FalseT>
struct select_if<false,TrueT,FalseT> {
    typedef FalseT result;
    enum { is_true = false };
};

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
