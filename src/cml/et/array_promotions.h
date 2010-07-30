/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * Defines promotions between array types.
 *
 * @todo Can/should an expression with a fixed-size argument promote to a
 * fixed array instead of a dynamic array?
 */

#ifndef array_promotions_h
#define array_promotions_h

#include <cml/core/cml_meta.h>
#include <cml/et/scalar_promotions.h>

namespace cml {
namespace et {

#define VAL_MAX(_a_,_b_)        ( ((_a_)>(_b_))?(_a_):(_b_) )

namespace detail {

/* This is specialized for 1D and 2D promotions: */
template<class A1, class A2, typename DTag1, typename DTag2,
    typename PromotedSizeTag> struct promote;

/* Promote 1D fixed-size arrays to a 1D fixed-size array: */
template<class A1, class A2>
struct promote<A1,A2,oned_tag,oned_tag,fixed_size_tag>
{
    typedef typename A1::value_type left_scalar;
    typedef typename A2::value_type right_scalar;

    /* First, promote the scalar type: */
    typedef typename ScalarPromote<
        left_scalar,right_scalar>::type promoted_scalar;

    /* Next, deduce the array size: */
    enum { Size = VAL_MAX((size_t)A1::array_size, (size_t)A2::array_size) };

    /* Finally, generate the promoted array type: */
    typedef fixed_1D<promoted_scalar,Size> type;
};

/* Promote 1D dynamic arrays to a 1D dynamic array: */
template<class A1, class A2>
struct promote<A1,A2,oned_tag,oned_tag,dynamic_size_tag>
{
    typedef typename A1::value_type left_scalar;
    typedef typename A2::value_type right_scalar;

    /* First, promote the scalar type: */
    typedef typename ScalarPromote<
        left_scalar,right_scalar>::type promoted_scalar;

    /* Next, rebind to get the proper allocator: */
    typedef typename CML_DEFAULT_ARRAY_ALLOC
        ::rebind<promoted_scalar>::other allocator;

    /* Finally, generate the promoted array type: */
    typedef dynamic_1D<promoted_scalar,allocator> type;
};

/* Promote fixed 2D+1D array expressions to a fixed 1D array: */
template<class A1, class A2>
struct promote<A1,A2,twod_tag,oned_tag,fixed_size_tag>
{
    typedef typename A1::value_type left_scalar;
    typedef typename A2::value_type right_scalar;

    /* First, promote the scalar type: */
    typedef typename ScalarPromote<
        left_scalar,right_scalar>::type promoted_scalar;

    /* Next, deduce the array size: */
    enum { Size = (size_t)A1::array_rows };

    /* Finally, generate the promoted array type: */
    typedef fixed_1D<promoted_scalar,Size> type;
};

/* Promote fixed 1D+2D array expressions to a fixed 1D array: */
template<class A1, class A2>
struct promote<A1,A2,oned_tag,twod_tag,fixed_size_tag>
{
    typedef typename A1::value_type left_scalar;
    typedef typename A2::value_type right_scalar;

    /* First, promote the scalar type: */
    typedef typename ScalarPromote<
        left_scalar,right_scalar>::type promoted_scalar;

    /* Next, deduce the array size: */
    enum { Size = (size_t)A2::array_cols };

    /* Finally, generate the promoted array type: */
    typedef fixed_1D<promoted_scalar,Size> type;
};

/* Promote dynamic 2D+1D array expression to a 1D dynamic array: */
template<class A1, class A2>
struct promote<A1,A2,twod_tag,oned_tag,dynamic_size_tag>
{
    typedef typename A1::value_type left_scalar;
    typedef typename A2::value_type right_scalar;

    /* First, promote the scalar type: */
    typedef typename ScalarPromote<
        left_scalar,right_scalar>::type promoted_scalar;

    /* Next, rebind to get the proper allocator: */
    typedef typename CML_DEFAULT_ARRAY_ALLOC
        ::rebind<promoted_scalar>::other allocator;

    /* Finally, generate the promoted array type: */
    typedef dynamic_1D<promoted_scalar,allocator> type;
};

/* Promote dynamic 1D+2D array expression to a 1D dynamic array: */
template<class A1, class A2>
struct promote<A1,A2,oned_tag,twod_tag,dynamic_size_tag>
{
    typedef typename A1::value_type left_scalar;
    typedef typename A2::value_type right_scalar;

    /* First, promote the scalar type: */
    typedef typename ScalarPromote<
        left_scalar,right_scalar>::type promoted_scalar;

    /* Next, rebind to get the proper allocator: */
    typedef typename CML_DEFAULT_ARRAY_ALLOC
        ::rebind<promoted_scalar>::other allocator;

    /* Finally, generate the promoted array type: */
    typedef dynamic_1D<promoted_scalar,allocator> type;
};


/* This is a helper to deduce the result of a promoted 2D array: */
template<typename LeftL, typename RightL> struct deduce_layout {
#if defined(CML_ALWAYS_PROMOTE_TO_DEFAULT_LAYOUT)
    typedef CML_DEFAULT_ARRAY_LAYOUT promoted_layout;
#else
    typedef typename select_if<
        same_type<LeftL,RightL>::is_true, LeftL,
        CML_DEFAULT_ARRAY_LAYOUT>::result promoted_layout;
#endif
};

/* Promote 2D fixed-size arrays to a 2D fixed-size array.  The resulting
 * matrix has the same number of rows as A1, and the same number of
 * columns as A2.
 */
template<class A1, class A2>
struct promote<A1,A2,twod_tag,twod_tag,fixed_size_tag>
{
    typedef typename A1::value_type left_scalar;
    typedef typename A2::value_type right_scalar;

    /* First, promote the scalar type: */
    typedef typename ScalarPromote<
        left_scalar,right_scalar>::type promoted_scalar;

    /* Next, deduce the array size: */
    enum {
        Rows = (size_t)A1::array_rows,
        Cols = (size_t)A2::array_cols
    };

    /* Then deduce the array layout: */
    typedef typename A1::layout left_layout;
    typedef typename A2::layout right_layout;
    typedef typename deduce_layout<left_layout,right_layout>
        ::promoted_layout promoted_layout;

    /* Finally, generate the promoted array type: */
    typedef fixed_2D<promoted_scalar,Rows,Cols,promoted_layout> type;
};

/* Promote 2D dynamic arrays to a 2D dynamic array: */
template<class A1, class A2>
struct promote<A1,A2,twod_tag,twod_tag,dynamic_size_tag>
{
    typedef typename A1::value_type left_scalar;
    typedef typename A2::value_type right_scalar;

    /* First, promote the scalar type: */
    typedef typename ScalarPromote<
        left_scalar,right_scalar>::type promoted_scalar;

    /* Next, rebind to get the proper allocator: */
    typedef typename CML_DEFAULT_ARRAY_ALLOC
        ::rebind<promoted_scalar>::other allocator;

    /* Then deduce the array layout: */
    typedef typename A1::layout left_layout;
    typedef typename A2::layout right_layout;
    typedef typename deduce_layout<left_layout,right_layout>
        ::promoted_layout promoted_layout;

    /* Finally, generate the promoted array type: */
    typedef dynamic_2D<promoted_scalar,promoted_layout,allocator> type;
};

} // namespace detail

/** Class to promote array types.
 *
 * Both arguments must be array types.
 *
 * @sa fixed_1D
 * @sa fixed_2D
 * @sa dynamic_1D
 * @sa dynamic_2D
 */
template<class A1, class A2>
struct ArrayPromote
{
    /* Shorthand: */
    //typedef typename A1::value_type left_scalar;
    //typedef typename A2::value_type right_scalar;
    typedef typename A1::dimension_tag left_dtag;
    typedef typename A2::dimension_tag right_dtag;

    /* Deduce the proper type based upon the characteristics of AT1 and
     * AT2.  This is the table of type conversions:
     *
     *         AT1               AT2              Result
     *   memory   size     memory   size      memory   size
     *
     *   fixed    fixed    fixed    fixed     fixed    fixed
     *   fixed    fixed    dynamic  dynamic   dynamic  dynamic
     *   fixed    fixed    external fixed     fixed    fixed
     *   fixed    fixed    external dynamic   dynamic  dynamic
     *
     *   dynamic  dynamic  fixed    fixed     dynamic  dynamic
     *   dynamic  dynamic  dynamic  dynamic   dynamic  dynamic
     *   dynamic  dynamic  external fixed     dynamic  dynamic
     *   dynamic  dynamic  external dynamic   dynamic  dynamic
     *
     *   external fixed    external fixed     fixed    fixed
     *   external fixed    fixed    fixed     fixed    fixed
     *   external fixed    dynamic  dynamic   dynamic  dynamic
     *   external fixed    external dynamic   dynamic  dynamic
     *
     *   external dynamic  external fixed     dynamic  dynamic
     *   external dynamic  fixed    fixed     dynamic  dynamic
     *   external dynamic  dynamic  dynamic   dynamic  dynamic
     *   external dynamic  external dynamic   dynamic  dynamic
     *
     * Note that if one argument is a dynamically-sized array, the result
     * must be a dynamically allocated and sized array.  Likewise, if both
     * arguments have fixed size, the result can be a fixed-sized array.
     */

    /* Check if both arguments are fixed-size arrays.  If so, the promoted
     * array will be a fixed array, and if not, it will be a dynamic array:
     */
    typedef typename select_if<
        (same_type<typename A1::size_tag, fixed_size_tag>::is_true
         && same_type<typename A2::size_tag, fixed_size_tag>::is_true),
        fixed_size_tag,         /* True */
        dynamic_size_tag        /* False */
    >::result promoted_size_tag;

    /* Deduce the promoted type: */
    typedef typename detail::promote<
        A1, A2, left_dtag, right_dtag, promoted_size_tag>::type type;
};

/* Cleanup internal macros: */
#undef VAL_MAX

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
