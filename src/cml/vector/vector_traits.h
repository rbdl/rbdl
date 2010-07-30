/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef vector_traits_h
#define vector_traits_h

#include <cml/et/traits.h>

namespace cml {
namespace et {

/** Expression traits for a vector<> type. */
template<typename E, class AT>
struct ExprTraits< cml::vector<E,AT> >
{
    typedef typename cml::vector<E,AT>::expr_type expr_type;
    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_reference reference;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::resizing_tag resizing_tag;
    typedef typename expr_type::assignable_tag assignable_tag;
    typedef expr_type result_type;
    typedef expr_leaf_tag node_tag;

    value_type get(const expr_type& v, size_t i) const { return v[i]; }
    size_t size(const expr_type& v) const { return v.size(); }
};

} // namespace et
} // namespace cml


#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
