/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef quaternion_promotions_h
#define quaternion_promotions_h

#include <cml/et/scalar_promotions.h>
#include <cml/vector/vector_promotions.h>

namespace cml {
namespace et {

/* Default quaternion type promotion templates. */
template<class LeftT, class RightT> struct QuaternionPromote;

/** Type promotion for two quaternion types. */
template<typename E1, class AT1, typename E2, class AT2, class OT, class CT>
struct QuaternionPromote<
    cml::quaternion<E1,AT1,OT,CT>,
    cml::quaternion<E2,AT2,OT,CT>
>
{
    /* The deduced vector type: */
    typedef typename VectorPromote<
        typename cml::quaternion<E1,AT1,OT,CT>::vector_type,
        typename cml::quaternion<E2,AT2,OT,CT>::vector_type
        >::type promoted_vector;

    /* The deduced element and storage types: */
    typedef typename promoted_vector::value_type value_type;
    typedef typename promoted_vector::storage_type storage_type;

    /* The deduced quaternion result type: */
    typedef cml::quaternion<value_type,storage_type,OT,CT> type;

    /* The temporary type: */
    typedef typename type::temporary_type temporary_type;
};

/**
 * NOTE: QuaternionPromote* are somewhat ad hoc, and were added to
 * simplify the code for quaternion slerp/squad/etc.
 */

/** Type promotion for two quaternion types. */
template < class Quat1_T, class Quat2_T >
struct QuaternionPromote2
{
    typedef typename QuaternionPromote<
        typename Quat1_T::temporary_type, typename Quat2_T::temporary_type
    >::temporary_type temporary_type;
    typedef typename temporary_type::value_type value_type;
};

/** Type promotion for three quaternion types. */
template < class Quat1_T, class Quat2_T, class Quat3_T >
struct QuaternionPromote3
{
    typedef typename QuaternionPromote<
        typename Quat1_T::temporary_type,
        typename QuaternionPromote<
            typename Quat2_T::temporary_type, typename Quat3_T::temporary_type
        >::temporary_type
    >::temporary_type temporary_type;
    typedef typename temporary_type::value_type value_type;
};

/** Type promotion for four quaternion types. */
template < class Quat1_T, class Quat2_T, class Quat3_T, class Quat4_T >
struct QuaternionPromote4
{
    typedef typename QuaternionPromote<
        typename Quat1_T::temporary_type,
        typename QuaternionPromote<
            typename Quat2_T::temporary_type,
            typename QuaternionPromote<
                typename Quat3_T::temporary_type,
                typename Quat4_T::temporary_type
            >::temporary_type
        >::temporary_type
    >::temporary_type temporary_type;
    typedef typename temporary_type::value_type value_type;
};

/** Type promotion for a quaternion and a scalar. */
template<typename E, class AT, class OT, class CT, typename S>
struct QuaternionPromote<cml::quaternion<E,AT,OT,CT>, S>
{
    /* The deduced vector type: */
    typedef typename VectorPromote<
        typename quaternion<E,AT,OT,CT>::vector_type, S
        >::type promoted_vector;

    /* The deduced element and storage types: */
    typedef typename promoted_vector::value_type value_type;
    typedef typename promoted_vector::storage_type storage_type;

    /* The deduced quaternion result type: */
    typedef cml::quaternion<value_type,storage_type,OT,CT> type;

    /* The temporary type: */
    typedef typename type::temporary_type temporary_type;
};

/** Type promotion for a scalar and a quaternion. */
template<class S, typename E, class AT, class OT, class CT>
struct QuaternionPromote<S, cml::quaternion<E,AT,OT,CT> >
{
    /* The deduced vector type: */
    typedef typename VectorPromote<
        S, typename quaternion<E,AT,OT,CT>::vector_type
        >::type promoted_vector;

    /* The deduced element and storage types: */
    typedef typename promoted_vector::value_type value_type;
    typedef typename promoted_vector::storage_type storage_type;

    /* The deduced quaternion result type: */
    typedef cml::quaternion<value_type,storage_type,OT,CT> type;

    /* The temporary type: */
    typedef typename type::temporary_type temporary_type;
};

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
