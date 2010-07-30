/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef projection_h
#define projection_h

#include <cml/mathlib/matrix_concat.h>
#include <cml/mathlib/vector_transform.h>

/* Functions for projection and 'unprojection' of points in 3D. */

namespace cml {

namespace detail {

template < typename E > void
divide_by_w(vector< E,fixed<4> >& v) {
    v *= E(1) / v[3];
}

} // namespace detail

/* Project a point to screen space using the given model, view, projection,
 * and viewport matrices. The z value of the returned point is a depth value
 * in the range specified by the viewport matrix.
 */

template <class MatT_1, class MatT_2, class MatT_3, class MatT_4, class VecT>
vector< typename VecT::value_type, fixed<3> > project_point(
    const MatT_1& model,
    const MatT_2& view,
    const MatT_3& projection,
    const MatT_4& viewport,
    const VecT& p)
{
    return project_point(
        detail::matrix_concat_transforms_4x4(model,view),
        projection,
        viewport,
        p
    );
}

/* Project a point to screen space using the given modelview, projection, and
 * viewport matrices. The z value of the returned point is a depth value in
 * the range specified by the viewport matrix.
 */

template < class MatT_1, class MatT_2, class MatT_3, class VecT >
vector< typename VecT::value_type, fixed<3> > project_point(
    const MatT_1& modelview,
    const MatT_2& projection,
    const MatT_3& viewport,
    const VecT& p)
{
    typedef vector< typename VecT::value_type, fixed<3> > vector3_type;
    typedef vector< typename VecT::value_type, fixed<4> > vector4_type;
    typedef typename vector3_type::value_type value_type;

    detail::CheckVec3(p);

    vector4_type result = transform_vector_4D(
        detail::matrix_concat_transforms_4x4(
            modelview,
            detail::matrix_concat_transforms_4x4(
                projection,
                viewport
            )
        ),
        vector4_type(p[0],p[1],p[2],value_type(1))
    );
    detail::divide_by_w(result);
    return vector3_type(result[0],result[1],result[2]);
}

/* 'Unproject' a point from screen space using the given model, view,
 * projection, and viewport matrices. The z value of the input point is a
 * depth value in the range specified by the viewport matrix.
 */

template <class MatT_1, class MatT_2, class MatT_3, class MatT_4, class VecT>
vector< typename VecT::value_type, fixed<3> > unproject_point(
    const MatT_1& model,
    const MatT_2& view,
    const MatT_3& projection,
    const MatT_4& viewport,
    const VecT& p)
{
    return unproject_point(
        detail::matrix_concat_transforms_4x4(model,view),
        projection,
        viewport,
        p
    );
}

/* 'Unproject' a point from screen space using the given modelview,
 * projection, and viewport matrices. The z value of the input point is a
 * depth value in the range specified by the viewport matrix.
 */
 
template < class MatT_1, class MatT_2, class MatT_3, class VecT >
vector< typename VecT::value_type, fixed<3> > unproject_point(
    const MatT_1& modelview,
    const MatT_2& projection,
    const MatT_3& viewport,
    const VecT& p)
{
    typedef vector< typename VecT::value_type, fixed<3> > vector3_type;
    typedef vector< typename VecT::value_type, fixed<4> > vector4_type;
    typedef typename vector3_type::value_type value_type;

    detail::CheckVec3(p);

    vector4_type result = transform_vector_4D(
        inverse(
            detail::matrix_concat_transforms_4x4(
                modelview,
                detail::matrix_concat_transforms_4x4(
                    projection,
                    viewport
                )
            )
        ),
        vector4_type(p[0],p[1],p[2],value_type(1))
    );
    detail::divide_by_w(result);
    return vector3_type(result[0],result[1],result[2]);
}

} // namespace cml

#endif
