/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * Defines promotions for vectors used in vector/vector or vector/scalar
 * expressions.
 *
 * @sa BinaryVectorOp
 */

#ifndef vector_promotions_h
#define vector_promotions_h

#include <cml/et/scalar_promotions.h>
#include <cml/et/array_promotions.h>

namespace cml {
namespace et {

/* Default vector type promotion template. */
template<class LeftT, class RightT> struct VectorPromote;

/** Type promotion for two vector types. */
template<typename E1, class AT1, typename E2, class AT2>
struct VectorPromote< cml::vector<E1,AT1>, cml::vector<E2,AT2> >
{
    typedef typename ArrayPromote<
        typename cml::vector<E1,AT1>::array_type,
        typename cml::vector<E2,AT2>::array_type
    >::type promoted_array;

    /* The deduced vector result type: */
    typedef cml::vector<
        typename promoted_array::value_type,
        typename promoted_array::generator_type
    > type;

    /* The deduced temporary type: */
    typedef typename type::temporary_type temporary_type;
};

/** Type promotion for a vector and a scalar. */
template<typename E, class AT, typename S>
struct VectorPromote<cml::vector<E,AT>, S>
{
    /* The deduced vector result type (the array type is the same): */
    typedef cml::vector<typename ScalarPromote<E,S>::type, AT> type;

    /* The deduced temporary type: */
    typedef typename type::temporary_type temporary_type;
};

/** Type promotion for a scalar and a vector. */
template<typename S, typename E, class AT>
struct VectorPromote<S, cml::vector<E,AT> >
{
    /* The deduced vector result type (the array type is the same): */
    typedef cml::vector<typename ScalarPromote<S,E>::type, AT> type;

    /* The deduced temporary type: */
    typedef typename type::temporary_type temporary_type;
};

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
