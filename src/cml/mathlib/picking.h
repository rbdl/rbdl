/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef picking_h
#define picking_h

#include <cml/mathlib/projection.h>

/* Functions for picking with rays, volumes, and drag-enclosed volumes. */

namespace cml {

/* Support function for extracting the near and far depth range values from
 * a viewport matrix.
  */
 
namespace detail {

// NOTE: Changed 'near' and 'far' to 'n' and 'f' to work around windows.h
// 'near' and 'far' macros.

template < class MatT, typename Real > void
depth_range_from_viewport_matrix(const MatT& viewport, Real& n, Real& f)
{
    detail::CheckMatHomogeneous3D(viewport);
    
    n = viewport.basis_element(3,2);
    f = viewport.basis_element(2,2) + n;
}

} // namespace detail

/* Make a pick ray given screen coordinates and view, projection, and viewport
 * matrices. The origin of the ray lies in the near plane of the frustum; the
 * direction vector extends to the far plane if 'normalize' is false, and is
 * made unit-length if 'normalize' is true (its default value).
 *
 * Note that the origin of the ray lies in the near plane rather than
 * coinciding with the position of the virtual camera, as the latter gives
 * incorrect results when the projection is orthographic.
 *
 * Note also that the screen y coordinate increases from bottom to top rather
 * than top to bottom. If mouse coordinates are returned in window space where
 * the y coordinate increases from top to bottom (as is often the case), the
 * y value should be recomputed as 'y = <window height> - y' before being
 * submitted to this function.
 */

template < class MatT_1, class MatT_2, class MatT_3, typename E, class A >
void make_pick_ray(
    E pick_x,
    E pick_y,
    const MatT_1& view,
    const MatT_2& projection,
    const MatT_3& viewport,
    vector<E,A>& origin,
    vector<E,A>& direction,
    bool normalize = true)
{
    typedef vector<E,A> vector_type;
    typedef typename vector_type::value_type value_type;

    // NOTE: Changed 'near' and 'far' to 'n' and 'f' to work around
    // windows.h 'near' and 'far' macros.
    value_type n, f;
    detail::depth_range_from_viewport_matrix(viewport, n, f);

    origin =
        unproject_point(
            view,projection,viewport,vector_type(pick_x,pick_y,n)
        );
    direction =
        unproject_point(
            view,projection,viewport,vector_type(pick_x,pick_y,f)
        ) - origin;
    if (normalize) {
        direction.normalize();
    }
}

/* Make a pick volume given the screen coordinates of the center of the
 * picking rect, the width and height of the picking rect, and view and
 * projection matrices.
 *
 * The volume is loaded into the 'planes' array. The planes are of the form
 * ax+by+cz+d = 0, and are in the order left, right, bottom, top, near, far.
 *
 * The z_clip argument should be either z_clip_neg_one or z_clip_zero, and
 * should correspond to the near z-clipping range of the projection matrix
 * argument.
 *
 * The 'normalize' argument indicates whether the output planes should be
 * normalized; its default value is 'true'.
 *
 * Note that the screen y coordinate increases from bottom to top rather
 * than top to bottom. If mouse coordinates are returned in window space where
 * the y coordinate increases from top to bottom (as is often the case), the
 * y value should be recomputed as 'y = <window height> - y' before being
 * submitted to this function.
 */

template < class MatT_1, class MatT_2, typename Real >
void make_pick_volume(
    Real pick_x,
    Real pick_y,
    Real pick_width,
    Real pick_height,
    Real viewport_x,
    Real viewport_y,
    Real viewport_width,
    Real viewport_height,
    const MatT_1& view,
    const MatT_2& projection,
    Real planes[6][4],
    ZClip z_clip,
    bool normalize = true)
{
    // FIXME: Should be promoted type...
    typedef matrix<
        Real, fixed<4,4>,
        typename MatT_1::basis_orient, typename MatT_1::layout >
    matrix_type;
    
    matrix_type pick;
    matrix_pick(
        pick, pick_x, pick_y, pick_width, pick_height,
        viewport_x, viewport_y, viewport_width, viewport_height
    );
    cml::extract_frustum_planes(
        view,detail::matrix_concat_transforms_4x4(projection,pick),
        planes,z_clip,normalize);
}

/* Make a pick volume given two opposite corners of a rectangle in screen
 * space, and view and projection matrices. The corners of the screen rect
 * need not be in any particular 'order' with regard to the values of the
 * coordinates.
 *
 * The volume is loaded into the 'planes' array. The planes are of the form
 * ax+by+cz+d = 0, and are in the order left, right, bottom, top, near, far.
 *
 * The z_clip argument should be either z_clip_neg_one or z_clip_zero, and
 * should correspond to the near z-clipping range of the projection matrix
 * argument.
 *
 * The 'normalize' argument indicates whether the output planes should be
 * normalized; its default value is 'true'.
 *
 * Note that the screen y coordinate increases from bottom to top rather
 * than top to bottom. If mouse coordinates are returned in window space where
 * the y coordinate increases from top to bottom (as is often the case), the
 * y value should be recomputed as 'y = <window height> - y' before being
 * submitted to this function.
 */

template < class MatT_1, class MatT_2, typename Real >
void make_pick_drag_volume(
    Real pick_x1,
    Real pick_y1,
    Real pick_x2,
    Real pick_y2,
    Real viewport_x,
    Real viewport_y,
    Real viewport_width,
    Real viewport_height,
    const MatT_1& view,
    const MatT_2& projection,
    Real planes[6][4],
    ZClip z_clip,
    bool normalize = true)
{
    typedef Real value_type;

    make_pick_volume(
        (pick_x1+pick_x2)*value_type(.5),
        (pick_y1+pick_y2)*value_type(.5),
        std::fabs(pick_x2-pick_x1),
        std::fabs(pick_y2-pick_y1),
        viewport_x, viewport_y, viewport_width, viewport_height,
        view, projection, planes, z_clip, normalize
    );
}

} // namespace cml

#endif
