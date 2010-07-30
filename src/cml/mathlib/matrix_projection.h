/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_projection_h
#define matrix_projection_h

#include <cml/mathlib/checking.h>
#include <cml/mathlib/helper.h>

/* Functions for building matrix transforms other than rotations
 * (matrix_rotation.h) and viewing projections (matrix_projection.h).
 *
 * @todo: Clean up comments and documentation throughout.
 */

// NOTE: Changed 'near' and 'far' to 'n' and 'f' throughout to work around
// windows.h 'near' and 'far' macros.

namespace cml {

//////////////////////////////////////////////////////////////////////////////
// 3D perspective projection from frustum
//////////////////////////////////////////////////////////////////////////////

/** Build a matrix representing a perspective projection, specified by frustum
 *  bounds in l,r,b,t,n,f form, and with the given handedness and z clipping
 *  range
 */
template < typename E, class A, class B, class L > void
matrix_perspective(matrix<E,A,B,L>& m, E left, E right, E bottom, E top,
    E n, E f, Handedness handedness,
    ZClip z_clip)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatHomogeneous3D(m);

    identity_transform(m);
    
    value_type inv_width = value_type(1) / (right - left);
    value_type inv_height = value_type(1) / (top - bottom);
    value_type inv_depth = value_type(1) / (f - n);
    value_type near2 = value_type(2) * n;
    value_type s = handedness == left_handed ? 1 : -1;

    if (z_clip == z_clip_neg_one) {
        m.set_basis_element(2,2,s * (f + n) * inv_depth);
        m.set_basis_element(3,2,value_type(-2) * f * n * inv_depth);
    } else { // z_clip == z_clip_zero
        m.set_basis_element(2,2,s * f * inv_depth);
        m.set_basis_element(3,2,-s * n * m.basis_element(2,2));
    }
    
    m.set_basis_element(0,0,near2 * inv_width               );
    m.set_basis_element(1,1,near2 * inv_height              );
    m.set_basis_element(2,0,-s * (right + left) * inv_width );
    m.set_basis_element(2,1,-s * (top + bottom) * inv_height);
    m.set_basis_element(2,3,s                               );
    m.set_basis_element(3,3,value_type(0)                   );
}

/** Build a matrix representing a perspective projection, specified by frustum
 *  bounds in w,h,n,f form, and with the given handedness and z clipping
 *  range
 */
template < typename E, class A, class B, class L > void
matrix_perspective(matrix<E,A,B,L>& m, E width, E height, E n, E f,
    Handedness handedness, ZClip z_clip)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;
    
    value_type half_width = width * value_type(.5);
    value_type half_height = height * value_type(.5);
    matrix_perspective(m, -half_width, half_width,
        -half_height, half_height, n, f, handedness, z_clip);
}

/** Build a left-handedness frustum perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_LH(matrix<E,A,B,L>& m, E left, E right, E bottom,
    E top, E n, E f, ZClip z_clip)
{
    matrix_perspective(m, left, right, bottom, top, n, f,
        left_handed, z_clip);
}

/** Build a right-handedness frustum perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_RH(matrix<E,A,B,L>& m, E left, E right, E bottom,
    E top, E n, E f, ZClip z_clip)
{
    matrix_perspective(m, left, right, bottom, top, n, f,
        right_handed, z_clip);
}

/** Build a left-handedness frustum perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_LH(matrix<E,A,B,L>& m, E width, E height, E n,
    E f, ZClip z_clip)
{
    matrix_perspective(m, width, height, n, f, left_handed, z_clip);
}

/** Build a right-handedness frustum perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_RH(matrix<E,A,B,L>& m, E width, E height, E n,
    E f, ZClip z_clip)
{
    matrix_perspective(m, width, height, n, f, right_handed, z_clip);
}

//////////////////////////////////////////////////////////////////////////////
// 3D perspective projection from horizontal field of view
//////////////////////////////////////////////////////////////////////////////

/** Build a perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_xfov(matrix<E,A,B,L>& m, E xfov, E aspect, E n,
    E f, Handedness handedness, ZClip z_clip)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;
    
    value_type width = value_type(2) * std::tan(xfov * value_type(.5)) * n;
    matrix_perspective(m, width, width / aspect, n, f,
        handedness, z_clip);
}

/** Build a left-handedness perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_xfov_LH(matrix<E,A,B,L>& m, E xfov, E aspect, E n,
    E f, ZClip z_clip)
{
    matrix_perspective_xfov(m,xfov,aspect,n,f,left_handed,z_clip);
}

/** Build a right-handedness perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_xfov_RH(matrix<E,A,B,L>& m, E xfov, E aspect, E n,
    E f, ZClip z_clip)
{
    matrix_perspective_xfov(m,xfov,aspect,n,f,right_handed,z_clip);
}

//////////////////////////////////////////////////////////////////////////////
// 3D perspective projection from vertical field of view
//////////////////////////////////////////////////////////////////////////////

/** Build a perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_yfov(matrix<E,A,B,L>& m, E yfov, E aspect, E n,
    E f, Handedness handedness, ZClip z_clip)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;
    
    value_type height = value_type(2) * std::tan(yfov * value_type(.5)) * n;
    matrix_perspective(m, height * aspect, height, n, f,
        handedness, z_clip);
}

/** Build a left-handedness perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_yfov_LH(matrix<E,A,B,L>& m, E yfov, E aspect, E n,
    E f, ZClip z_clip)
{
    matrix_perspective_yfov(m,yfov,aspect,n,f,left_handed,z_clip);
}

/** Build a right-handedness perspective matrix */
template < typename E, class A, class B, class L > void
matrix_perspective_yfov_RH(matrix<E,A,B,L>& m, E yfov, E aspect, E n,
    E f, ZClip z_clip)
{
    matrix_perspective_yfov(m,yfov,aspect,n,f,right_handed,z_clip);
}

//////////////////////////////////////////////////////////////////////////////
// 3D orthographic projection from frustum
//////////////////////////////////////////////////////////////////////////////

/** Build a matrix representing an orthographic projection, specified by
 *  frustum bounds in l,r,b,t,n,f form, and with the given handedness and z
 *  clipping range
 */

template < typename E, class A, class B, class L > void
matrix_orthographic(matrix<E,A,B,L>& m, E left, E right, E bottom, E top,
    E n, E f, Handedness handedness,
    ZClip z_clip)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatHomogeneous3D(m);

    identity_transform(m);
    
    value_type inv_width = value_type(1) / (right - left);
    value_type inv_height = value_type(1) / (top - bottom);
    value_type inv_depth = value_type(1) / (f - n);
    value_type s = handedness == left_handed ? 1 : -1;

    if (z_clip == z_clip_neg_one) {
        m.set_basis_element(2,2,s * value_type(2) * inv_depth);
        m.set_basis_element(3,2,-(f + n) * inv_depth);
    } else { // z_clip.z_clip() == 0
        m.set_basis_element(2,2,s * inv_depth);
        m.set_basis_element(3,2,-n * inv_depth);
    }
    
    m.set_basis_element(0,0,value_type(2) * inv_width   );
    m.set_basis_element(1,1,value_type(2) * inv_height  );
    m.set_basis_element(3,0,-(right + left) * inv_width );
    m.set_basis_element(3,1,-(top + bottom) * inv_height);
}

/** Build an orthographic projection matrix */
template < typename E, class A, class B, class L > void
matrix_orthographic(matrix<E,A,B,L>& m, E width, E height, E n, E f,
    Handedness handedness, ZClip z_clip)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;
    
    value_type half_width = width * value_type(.5);
    value_type half_height = height * value_type(.5);
    matrix_orthographic(m, -half_width, half_width,
        -half_height, half_height, n, f, handedness, z_clip);
}

/** Build a left-handedness orthographic projection matrix */
template < typename E, class A, class B, class L > void
matrix_orthographic_LH(matrix<E,A,B,L>& m, E left, E right, E bottom,
    E top, E n, E f, ZClip z_clip)
{
    matrix_orthographic(m, left, right, bottom, top, n, f,
        left_handed, z_clip);
}

/** Build a right-handedness orthographic projection matrix */
template < typename E, class A, class B, class L > void
matrix_orthographic_RH(matrix<E,A,B,L>& m, E left, E right, E bottom,
    E top, E n, E f, ZClip z_clip)
{
    matrix_orthographic(m, left, right, bottom, top, n, f,
        right_handed, z_clip);
}

/** Build a left-handedness orthographic projection matrix */
template < typename E, class A, class B, class L > void
matrix_orthographic_LH(matrix<E,A,B,L>& m, E width, E height, E n,
    E f, ZClip z_clip)
{
    matrix_orthographic(m, width, height, n, f, left_handed,
        z_clip);
}

/** Build a right-handedness orthographic projection matrix */
template < typename E, class A, class B, class L > void
matrix_orthographic_RH(matrix<E,A,B,L>& m, E width, E height, E n,
    E f, ZClip z_clip)
{
    matrix_orthographic(m, width, height, n, f, right_handed,
        z_clip);
}

//////////////////////////////////////////////////////////////////////////////
// 3D viewport
//////////////////////////////////////////////////////////////////////////////

/* Build a viewport matrix
 *
 * Note: A viewport matrix is in a sense the opposite of an orthographics
 * projection matrix, and can be build by constructing and inverting the
 * latter.
 *
 * @todo: Need to look into D3D viewport conventions and see if this needs to
 * be adapted accordingly.
 */

template < typename E, class A, class B, class L > void
matrix_viewport(matrix<E,A,B,L>& m, E left, E right, E bottom,
    E top, ZClip z_clip, E n = E(0), E f = E(1))
{
    matrix_orthographic_LH(m, left, right, bottom, top, n, f, z_clip);
    /* @todo: invert(m), when available */
    m = inverse(m);
}

//////////////////////////////////////////////////////////////////////////////
// 3D picking volume
//////////////////////////////////////////////////////////////////////////////

/* Build a pick volume matrix
 *
 * When post-concatenated with a projection matrix, the pick matrix modifies
 * the view volume to create a 'picking volume'. This volume corresponds to
 * a screen rectangle centered at (pick_x, pick_y) and with dimensions
 * pick_widthXpick_height.
 *
 * @todo: Representation of viewport between this function and
 * matrix_viewport() is inconsistent (position and dimensions vs. bounds).
 * Should this be addressed?
 */

template < typename E, class A, class B, class L > void
matrix_pick(
    matrix<E,A,B,L>& m, E pick_x, E pick_y, E pick_width, E pick_height,
    E viewport_x, E viewport_y, E viewport_width, E viewport_height)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatHomogeneous3D(m);

    identity_transform(m);
    
    value_type inv_width = value_type(1) / pick_width;
    value_type inv_height = value_type(1) / pick_height;
    
    m.set_basis_element(0,0,viewport_width*inv_width);
    m.set_basis_element(1,1,viewport_height*inv_height);
    m.set_basis_element(3,0,
        (viewport_width+value_type(2)*(viewport_x-pick_x))*inv_width);
    m.set_basis_element(3,1,
        (viewport_height+value_type(2)*(viewport_y-pick_y))*inv_height);
}

} // namespace cml

#endif
