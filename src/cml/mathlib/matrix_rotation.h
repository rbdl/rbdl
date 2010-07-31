/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_rotation_h
#define matrix_rotation_h

#include <cml/mathlib/matrix_misc.h>
#include <cml/mathlib/vector_ortho.h>

/* Functions related to matrix rotations in 3D and 2D. */

namespace cml {

//////////////////////////////////////////////////////////////////////////////
// 3D rotation about world axes
//////////////////////////////////////////////////////////////////////////////

/** Build a matrix representing a 3D rotation about the given world axis */
template < typename E, class A, class B, class L > void
matrix_rotation_world_axis( matrix<E,A,B,L>& m, size_t axis, E angle)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckIndex3(axis);

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);
    
    value_type s = value_type(std::sin(angle));
    value_type c = value_type(std::cos(angle));
    
    identity_transform(m);

    m.set_basis_element(j,j, c);
    m.set_basis_element(j,k, s);
    m.set_basis_element(k,j,-s);
    m.set_basis_element(k,k, c);
}

/** Build a matrix representing a 3D rotation about the world x axis */
template < typename E, class A, class B, class L > void
matrix_rotation_world_x(matrix<E,A,B,L>& m, E angle) {
    matrix_rotation_world_axis(m,0,angle);
}

/** Build a matrix representing a 3D rotation about the world y axis */
template < typename E, class A, class B, class L > void
matrix_rotation_world_y(matrix<E,A,B,L>& m, E angle) {
    matrix_rotation_world_axis(m,1,angle);
}

/** Build a matrix representing a 3D rotation about the world z axis */
template < typename E, class A, class B, class L > void
matrix_rotation_world_z(matrix<E,A,B,L>& m, E angle) {
    matrix_rotation_world_axis(m,2,angle);
}

//////////////////////////////////////////////////////////////////////////////
// 3D rotation from an axis-angle pair
//////////////////////////////////////////////////////////////////////////////

/** Build a rotation matrix from an axis-angle pair */
template < typename E, class A, class B, class L, class VecT > void
matrix_rotation_axis_angle(matrix<E,A,B,L>& m, const VecT& axis, E angle)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckVec3(axis);
    
    identity_transform(m);

    value_type s = std::sin(angle);
    value_type c = std::cos(angle);
    value_type omc = value_type(1) - c;

    value_type xomc = axis[0] * omc;
    value_type yomc = axis[1] * omc;
    value_type zomc = axis[2] * omc;
    
    value_type xxomc = axis[0] * xomc;
    value_type yyomc = axis[1] * yomc;
    value_type zzomc = axis[2] * zomc;
    value_type xyomc = axis[0] * yomc;
    value_type yzomc = axis[1] * zomc;
    value_type zxomc = axis[2] * xomc;

    value_type xs = axis[0] * s;
    value_type ys = axis[1] * s;
    value_type zs = axis[2] * s;

    m.set_basis_element(0,0, xxomc + c );
    m.set_basis_element(0,1, xyomc + zs);
    m.set_basis_element(0,2, zxomc - ys);
    m.set_basis_element(1,0, xyomc - zs);
    m.set_basis_element(1,1, yyomc + c );
    m.set_basis_element(1,2, yzomc + xs);
    m.set_basis_element(2,0, zxomc + ys);
    m.set_basis_element(2,1, yzomc - xs);
    m.set_basis_element(2,2, zzomc + c );
}

//////////////////////////////////////////////////////////////////////////////
// 3D rotation from a quaternion
//////////////////////////////////////////////////////////////////////////////

/** Build a rotation matrix from a quaternion */
template < typename E, class A, class B, class L, class QuatT > void
matrix_rotation_quaternion(matrix<E,A,B,L>& m, const QuatT& q)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef QuatT quaternion_type;
    typedef typename quaternion_type::order_type order_type;
    typedef typename matrix_type::value_type value_type;

    enum {
        W = order_type::W,
        X = order_type::X,
        Y = order_type::Y,
        Z = order_type::Z
    };
    
    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckQuat(q);

    identity_transform(m);
    
    value_type x2 = q[X] + q[X];
    value_type y2 = q[Y] + q[Y];
    value_type z2 = q[Z] + q[Z];    

    value_type xx2 = q[X] * x2;
    value_type yy2 = q[Y] * y2;
    value_type zz2 = q[Z] * z2;
    value_type xy2 = q[X] * y2;
    value_type yz2 = q[Y] * z2;
    value_type zx2 = q[Z] * x2;
    value_type xw2 = q[W] * x2;
    value_type yw2 = q[W] * y2;
    value_type zw2 = q[W] * z2;
    
    m.set_basis_element(0,0, value_type(1) - yy2 - zz2);
    m.set_basis_element(0,1,                 xy2 + zw2);
    m.set_basis_element(0,2,                 zx2 - yw2);
    m.set_basis_element(1,0,                 xy2 - zw2);
    m.set_basis_element(1,1, value_type(1) - zz2 - xx2);
    m.set_basis_element(1,2,                 yz2 + xw2);
    m.set_basis_element(2,0,                 zx2 + yw2);
    m.set_basis_element(2,1,                 yz2 - xw2);
    m.set_basis_element(2,2, value_type(1) - xx2 - yy2);
}

//////////////////////////////////////////////////////////////////////////////
// 3D rotation from Euler angles
//////////////////////////////////////////////////////////////////////////////

/** Build a rotation matrix from an Euler-angle triple
 *
 * The rotations are applied about the cardinal axes in the order specified by
 * the 'order' argument, where 'order' is one of the following enumerants:
 *
 * euler_order_xyz
 * euler_order_xzy
 * euler_order_xyx
 * euler_order_xzx
 * euler_order_yzx
 * euler_order_yxz
 * euler_order_yzy
 * euler_order_yxy
 * euler_order_zxy
 * euler_order_zyx
 * euler_order_zxz
 * euler_order_zyz
 *
 * e.g. euler_order_xyz means compute the column-basis rotation matrix
 * equivalent to R_x * R_y * R_z, where R_i is the rotation matrix above
 * axis i (the row-basis matrix would be R_z * R_y * R_x).
 */
template < typename E, class A, class B, class L > void
matrix_rotation_euler(matrix<E,A,B,L>& m, E angle_0, E angle_1, E angle_2,
    EulerOrder order)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear3D(m);

    identity_transform(m);
    
    size_t i, j, k;
    bool odd, repeat;
    detail::unpack_euler_order(order, i, j, k, odd, repeat);

    if (odd) {
        angle_0 = -angle_0;
        angle_1 = -angle_1;
        angle_2 = -angle_2;
    }
    
    value_type s0 = std::sin(angle_0);
    value_type c0 = std::cos(angle_0);
    value_type s1 = std::sin(angle_1);
    value_type c1 = std::cos(angle_1);
    value_type s2 = std::sin(angle_2);
    value_type c2 = std::cos(angle_2);
    
    value_type s0s2 = s0 * s2;
    value_type s0c2 = s0 * c2;
    value_type c0s2 = c0 * s2;
    value_type c0c2 = c0 * c2;

    if (repeat) {
        m.set_basis_element(i,i, c1              );
        m.set_basis_element(i,j, s1 * s2         );
        m.set_basis_element(i,k,-s1 * c2         );
        m.set_basis_element(j,i, s0 * s1         );
        m.set_basis_element(j,j,-c1 * s0s2 + c0c2);
        m.set_basis_element(j,k, c1 * s0c2 + c0s2);
        m.set_basis_element(k,i, c0 * s1         );
        m.set_basis_element(k,j,-c1 * c0s2 - s0c2);
        m.set_basis_element(k,k, c1 * c0c2 - s0s2);
    } else {
        m.set_basis_element(i,i, c1 * c2         );
        m.set_basis_element(i,j, c1 * s2         );
        m.set_basis_element(i,k,-s1              );
        m.set_basis_element(j,i, s1 * s0c2 - c0s2);
        m.set_basis_element(j,j, s1 * s0s2 + c0c2);
        m.set_basis_element(j,k, s0 * c1         );
        m.set_basis_element(k,i, s1 * c0c2 + s0s2);
        m.set_basis_element(k,j, s1 * c0s2 - s0c2);
        m.set_basis_element(k,k, c0 * c1         );
    }
}

/** Build a matrix of derivatives of Euler angles about the specified axis.
 *
 * The rotation derivatives are applied about the cardinal axes in the
 * order specified by the 'order' argument, where 'order' is one of the
 * following enumerants:
 *
 * euler_order_xyz
 * euler_order_xzy
 * euler_order_yzx
 * euler_order_yxz
 * euler_order_zxy
 * euler_order_zyx
 *
 * e.g. euler_order_xyz means compute the column-basis rotation matrix
 * equivalent to R_x * R_y * R_z, where R_i is the rotation matrix above
 * axis i (the row-basis matrix would be R_z * R_y * R_x).
 *
 * The derivative is taken with respect to the specified 'axis', which is
 * the position of the axis in the triple; e.g. if order = euler_order_xyz,
 * then axis = 0 would mean take the derivative with respect to x.  Note
 * that repeated axes are not currently supported.
 */
template < typename E, class A, class B, class L > void
matrix_rotation_euler_derivatives(
    matrix<E,A,B,L>& m, int axis, E angle_0, E angle_1, E angle_2,
    EulerOrder order)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear3D(m);

    size_t i, j, k;
    bool odd, repeat;
    detail::unpack_euler_order(order, i, j, k, odd, repeat);
    if(repeat) throw std::invalid_argument(
	"matrix_rotation_euler_derivatives does not support repeated axes");

    if (odd) {
        angle_0 = -angle_0;
        angle_1 = -angle_1;
        angle_2 = -angle_2;
    }

    value_type s0 = std::sin(angle_0);
    value_type c0 = std::cos(angle_0);
    value_type s1 = std::sin(angle_1);
    value_type c1 = std::cos(angle_1);
    value_type s2 = std::sin(angle_2);
    value_type c2 = std::cos(angle_2);
    
    value_type s0s2 = s0 * s2;
    value_type s0c2 = s0 * c2;
    value_type c0s2 = c0 * s2;
    value_type c0c2 = c0 * c2;

    if(axis == 0) {
      m.set_basis_element(i,i, 0.              );
      m.set_basis_element(i,j, 0.              );
      m.set_basis_element(i,k, 0.              );
      m.set_basis_element(j,i, s1 * c0*c2 + s0*s2);
      m.set_basis_element(j,j, s1 * c0*s2 - s0*c2);
      m.set_basis_element(j,k, c0 * c1         );
      m.set_basis_element(k,i,-s1 * s0*c2 + c0*s2);
      m.set_basis_element(k,j,-s1 * s0*s2 - c0*c2);
      m.set_basis_element(k,k,-s0 * c1         );
    } else if(axis == 1) {
      m.set_basis_element(i,i,-s1 * c2         );
      m.set_basis_element(i,j,-s1 * s2         );
      m.set_basis_element(i,k,-c1              );
      m.set_basis_element(j,i, c1 * s0*c2      );
      m.set_basis_element(j,j, c1 * s0*s2      );
      m.set_basis_element(j,k,-s0 * s1         );
      m.set_basis_element(k,i, c1 * c0*c2      );
      m.set_basis_element(k,j, c1 * c0*s2      );
      m.set_basis_element(k,k,-c0 * s1         );
    } else if(axis == 2) {
      m.set_basis_element(i,i,-c1 * s2         );
      m.set_basis_element(i,j, c1 * c2         );
      m.set_basis_element(i,k, 0.              );
      m.set_basis_element(j,i,-s1 * s0*s2 - c0*c2);
      m.set_basis_element(j,j, s1 * s0*c2 - c0*s2);
      m.set_basis_element(j,k, 0.              );
      m.set_basis_element(k,i,-s1 * c0*s2 + s0*c2);
      m.set_basis_element(k,j, s1 * c0*c2 + s0*s2);
      m.set_basis_element(k,k, 0.              );
    }
}

//////////////////////////////////////////////////////////////////////////////
// 3D rotation to align with a vector, multiple vectors, or the view plane
//////////////////////////////////////////////////////////////////////////////

/** See vector_ortho.h for details */
template < typename E,class A,class B,class L,class VecT_1,class VecT_2 > void
matrix_rotation_align(
    matrix<E,A,B,L>& m,
    const VecT_1& align,
    const VecT_2& reference,
    bool normalize = true,
    AxisOrder order = axis_order_zyx)
{
    typedef vector< E,fixed<3> > vector_type;

    identity_transform(m);
    
    vector_type x, y, z;

    orthonormal_basis(align, reference, x, y, z, normalize, order);
    matrix_set_basis_vectors(m, x, y, z);
}

/** See vector_ortho.h for details */
template < typename E, class A, class B, class L, class VecT > void
matrix_rotation_align(matrix<E,A,B,L>& m, const VecT& align,
    bool normalize = true, AxisOrder order = axis_order_zyx)
{
    typedef vector< E,fixed<3> > vector_type;

    identity_transform(m);
    
    vector_type x, y, z;

    orthonormal_basis(align, x, y, z, normalize, order);
    matrix_set_basis_vectors(m, x, y, z);
}

/** See vector_ortho.h for details */
template < typename E,class A,class B,class L,class VecT_1,class VecT_2 > void
matrix_rotation_align_axial(matrix<E,A,B,L>& m, const VecT_1& align,
    const VecT_2& axis, bool normalize = true,
    AxisOrder order = axis_order_zyx)
{
    typedef vector< E,fixed<3> > vector_type;

    identity_transform(m);
    
    vector_type x, y, z;

    orthonormal_basis_axial(align, axis, x, y, z, normalize, order);
    matrix_set_basis_vectors(m, x, y, z);
}

/** See vector_ortho.h for details */
template < typename E, class A, class B, class L, class MatT > void
matrix_rotation_align_viewplane(
    matrix<E,A,B,L>& m,
    const MatT& view_matrix,
    Handedness handedness,
    AxisOrder order = axis_order_zyx)
{
    typedef vector< E, fixed<3> > vector_type;

    identity_transform(m);
    
    vector_type x, y, z;

    orthonormal_basis_viewplane(view_matrix, x, y, z, handedness, order);
    matrix_set_basis_vectors(m, x, y, z);
}

/** See vector_ortho.h for details */
template < typename E, class A, class B, class L, class MatT > void
matrix_rotation_align_viewplane_LH(
    matrix<E,A,B,L>& m,
    const MatT& view_matrix,
    AxisOrder order = axis_order_zyx)
{
    matrix_rotation_align_viewplane(
        m,view_matrix,left_handed,order);
}

/** See vector_ortho.h for details */
template < typename E, class A, class B, class L, class MatT > void
matrix_rotation_align_viewplane_RH(
    matrix<E,A,B,L>& m,
    const MatT& view_matrix,
    AxisOrder order = axis_order_zyx)
{
    matrix_rotation_align_viewplane(
        m,view_matrix,right_handed,order);
}

//////////////////////////////////////////////////////////////////////////////
// 3D rotation to aim at a target
//////////////////////////////////////////////////////////////////////////////

/** See vector_ortho.h for details */
template < typename E, class A, class B, class L,
    class VecT_1, class VecT_2, class VecT_3 > void
matrix_rotation_aim_at(
    matrix<E,A,B,L>& m,
    const VecT_1& pos,
    const VecT_2& target,
    const VecT_3& reference,
    AxisOrder order = axis_order_zyx)
{
    matrix_rotation_align(m, target - pos, reference, true, order);
}

/** See vector_ortho.h for details */
template < typename E, class A, class B, class L,
    class VecT_1, class VecT_2 > void
matrix_rotation_aim_at(
    matrix<E,A,B,L>& m,
    const VecT_1& pos,
    const VecT_2& target,
    AxisOrder order = axis_order_zyx)
{
    matrix_rotation_align(m, target - pos, true, order);
}

/** See vector_ortho.h for details */
template < typename E, class A, class B, class L,
    class VecT_1, class VecT_2, class VecT_3 > void
matrix_rotation_aim_at_axial(
    matrix<E,A,B,L>& m,
    const VecT_1& pos,
    const VecT_2& target,
    const VecT_3& axis,
    AxisOrder order = axis_order_zyx)
{
    matrix_rotation_align_axial(m, target - pos, axis, true, order);
}

//////////////////////////////////////////////////////////////////////////////
// 2D rotation
//////////////////////////////////////////////////////////////////////////////

/** Build a matrix representing a 2D rotation */
template < typename E, class A, class B, class L > void
matrix_rotation_2D( matrix<E,A,B,L>& m, E angle)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear2D(m);

    value_type s = value_type(std::sin(angle));
    value_type c = value_type(std::cos(angle));
    
    identity_transform(m);

    m.set_basis_element(0,0, c);
    m.set_basis_element(0,1, s);
    m.set_basis_element(1,0,-s);
    m.set_basis_element(1,1, c);
}

//////////////////////////////////////////////////////////////////////////////
// 2D rotation to align with a vector
//////////////////////////////////////////////////////////////////////////////

/** See vector_ortho.h for details */
template < typename E, class A, class B, class L, class VecT > void
matrix_rotation_align_2D(matrix<E,A,B,L>& m, const VecT& align,
    bool normalize = true, AxisOrder2D order = axis_order_xy)
{
    typedef vector< E, fixed<2> > vector_type;

    identity_transform(m);
    
    vector_type x, y;

    orthonormal_basis_2D(align, x, y, normalize, order);
    matrix_set_basis_vectors_2D(m, x, y);
}

//////////////////////////////////////////////////////////////////////////////
// 3D relative rotation about world axes
//////////////////////////////////////////////////////////////////////////////

/** Rotate a rotation matrix about the given world axis */
template < typename E, class A, class B, class L > void
matrix_rotate_about_world_axis(matrix<E,A,B,L>& m, size_t axis, E angle)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckIndex3(axis);

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);

    value_type s = value_type(std::sin(angle));
    value_type c = value_type(std::cos(angle));

    value_type ij = c * m.basis_element(i,j) - s * m.basis_element(i,k);
    value_type jj = c * m.basis_element(j,j) - s * m.basis_element(j,k);
    value_type kj = c * m.basis_element(k,j) - s * m.basis_element(k,k);
    
    m.set_basis_element(i,k, s*m.basis_element(i,j) + c*m.basis_element(i,k));
    m.set_basis_element(j,k, s*m.basis_element(j,j) + c*m.basis_element(j,k));
    m.set_basis_element(k,k, s*m.basis_element(k,j) + c*m.basis_element(k,k));
    
    m.set_basis_element(i,j,ij);
    m.set_basis_element(j,j,jj);
    m.set_basis_element(k,j,kj);
}

/** Rotate a rotation matrix about the world x axis */
template < typename E, class A, class B, class L > void
matrix_rotate_about_world_x(matrix<E,A,B,L>& m, E angle) {
    matrix_rotate_about_world_axis(m,0,angle);
}

/** Rotate a rotation matrix about the world y axis */
template < typename E, class A, class B, class L > void
matrix_rotate_about_world_y(matrix<E,A,B,L>& m, E angle) {
    matrix_rotate_about_world_axis(m,1,angle);
}

/** Rotate a rotation matrix about the world z axis */
template < typename E, class A, class B, class L > void
matrix_rotate_about_world_z(matrix<E,A,B,L>& m, E angle) {
    matrix_rotate_about_world_axis(m,2,angle);
}

//////////////////////////////////////////////////////////////////////////////
// 3D relative rotation about local axes
//////////////////////////////////////////////////////////////////////////////

/** Rotate a rotation matrix about the given local axis */
template < typename E, class A, class B, class L > void
matrix_rotate_about_local_axis(matrix<E,A,B,L>& m, size_t axis, E angle)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear3D(m);
    detail::CheckIndex3(axis);

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);

    value_type s = value_type(std::sin(angle));
    value_type c = value_type(std::cos(angle));

    value_type j0 = c * m.basis_element(j,0) + s * m.basis_element(k,0);
    value_type j1 = c * m.basis_element(j,1) + s * m.basis_element(k,1);
    value_type j2 = c * m.basis_element(j,2) + s * m.basis_element(k,2);

    m.set_basis_element(k,0, c*m.basis_element(k,0) - s*m.basis_element(j,0));
    m.set_basis_element(k,1, c*m.basis_element(k,1) - s*m.basis_element(j,1));
    m.set_basis_element(k,2, c*m.basis_element(k,2) - s*m.basis_element(j,2));

    m.set_basis_element(j,0,j0);
    m.set_basis_element(j,1,j1);
    m.set_basis_element(j,2,j2);
}

/** Rotate a rotation matrix about its local x axis */
template < typename E, class A, class B, class L > void
matrix_rotate_about_local_x(matrix<E,A,B,L>& m, E angle) {
    matrix_rotate_about_local_axis(m,0,angle);
}

/** Rotate a rotation matrix about its local y axis */
template < typename E, class A, class B, class L > void
matrix_rotate_about_local_y(matrix<E,A,B,L>& m, E angle) {
    matrix_rotate_about_local_axis(m,1,angle);
}

/** Rotate a rotation matrix about its local z axis */
template < typename E, class A, class B, class L > void
matrix_rotate_about_local_z(matrix<E,A,B,L>& m, E angle) {
    matrix_rotate_about_local_axis(m,2,angle);
}

//////////////////////////////////////////////////////////////////////////////
// 2D relative rotation
//////////////////////////////////////////////////////////////////////////////

template < typename E, class A, class B, class L > void
matrix_rotate_2D(matrix<E,A,B,L>& m, E angle)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear2D(m);

    value_type s = value_type(std::sin(angle));
    value_type c = value_type(std::cos(angle));

    value_type m00 = c * m.basis_element(0,0) - s * m.basis_element(0,1);
    value_type m10 = c * m.basis_element(1,0) - s * m.basis_element(1,1);

    m.set_basis_element(0,1, s*m.basis_element(0,0) + c*m.basis_element(0,1));
    m.set_basis_element(1,1, s*m.basis_element(1,0) + c*m.basis_element(1,1));

    m.set_basis_element(0,0,m00);
    m.set_basis_element(1,0,m10);
}

//////////////////////////////////////////////////////////////////////////////
// Rotation from vector to vector
//////////////////////////////////////////////////////////////////////////////

/** Build a rotation matrix to rotate from one vector to another
 *
 * Note: The quaternion algorithm is more stable than the matrix algorithm, so
 * we simply pass off to the quaternion function here.
 */
template < class E,class A,class B,class L,class VecT_1,class VecT_2 > void
matrix_rotation_vec_to_vec(
    matrix<E,A,B,L>& m,
    const VecT_1& v1,
    const VecT_2& v2,
    bool unit_length_vectors = false)
{
    typedef quaternion< E,fixed<>,vector_first,positive_cross >
        quaternion_type;
    
    quaternion_type q;
    quaternion_rotation_vec_to_vec(q,v1,v2,unit_length_vectors);
    matrix_rotation_quaternion(m,q);
}

//////////////////////////////////////////////////////////////////////////////
// Scale the angle of a rotation matrix
//////////////////////////////////////////////////////////////////////////////

/** Scale the angle of a 3D rotation matrix */
template < typename E, class A, class B, class L > void
matrix_scale_rotation_angle(matrix<E,A,B,L>& m, E t,
    E tolerance = epsilon<E>::placeholder())
{
    typedef vector< E,fixed<3> > vector_type;
    typedef typename vector_type::value_type value_type;
    
    vector_type axis;
    value_type angle;
    matrix_to_axis_angle(m, axis, angle, tolerance);
    matrix_rotation_axis_angle(m, axis, angle * t);
}

/** Scale the angle of a 2D rotation matrix */
template < typename E, class A, class B, class L > void
matrix_scale_rotation_angle_2D(
    matrix<E,A,B,L>& m, E t, E tolerance = epsilon<E>::placeholder())
{
    typedef vector< E,fixed<2> > vector_type;
    typedef typename vector_type::value_type value_type;

    value_type angle = matrix_to_rotation_2D(m);
    matrix_rotation_2D(m, angle * t);
}

//////////////////////////////////////////////////////////////////////////////
// Support functions for uniform handling of row- and column-basis matrices
//////////////////////////////////////////////////////////////////////////////

/* Note: The matrix rotation slerp, difference and concatenation functions do
 * not use et::MatrixPromote<M1,M2>::temporary_type as the return type, even
 * though that is the return type of the underlying matrix multiplication.
 * This is because the sizes of these matrices are known at compile time (3x3
 * and 2x2), and using fixed<> obviates the need for resizing of intermediate
 * temporaries.
 *
 * Also, no size- or type-checking is done on the arguments to these
 * functions, as any such errors will be caught by the matrix multiplication
 * and assignment to the 3x3 temporary.
 */

/** A fixed-size temporary 3x3 matrix */
#define MAT_TEMP_3X3 matrix<         \
    typename et::ScalarPromote<      \
        typename MatT_1::value_type, \
        typename MatT_2::value_type  \
    >::type,                         \
    fixed<3,3>,                      \
    typename MatT_1::basis_orient,   \
    row_major                        \
>

/** A fixed-size temporary 2x2 matrix */
#define MAT_TEMP_2X2 matrix<         \
    typename et::ScalarPromote<      \
        typename MatT_1::value_type, \
        typename MatT_2::value_type  \
    >::type,                         \
    fixed<2,2>,                      \
    typename MatT_1::basis_orient,   \
    row_major                        \
>

namespace detail {

/** Concatenate two 3D row-basis rotation matrices in the order m1->m2 */
template < class MatT_1, class MatT_2 > MAT_TEMP_3X3
matrix_concat_rotations(const MatT_1& m1, const MatT_2& m2, row_basis) {
    return m1*m2;
}

/** Concatenate two 3D col-basis rotation matrices in the order m1->m2 */
template < class MatT_1, class MatT_2 > MAT_TEMP_3X3
matrix_concat_rotations(const MatT_1& m1, const MatT_2& m2, col_basis) {
    return m2*m1;
}

/** Concatenate two 3D rotation matrices in the order m1->m2 */
template < class MatT_1, class MatT_2 > MAT_TEMP_3X3
matrix_concat_rotations(const MatT_1& m1, const MatT_2& m2) {
    return matrix_concat_rotations(m1,m2,typename MatT_1::basis_orient());
}

/** Concatenate two 2D row-basis rotation matrices in the order m1->m2 */
template < class MatT_1, class MatT_2 > MAT_TEMP_2X2
matrix_concat_rotations_2D(const MatT_1& m1, const MatT_2& m2, row_basis) {
    return m1*m2;
}

/** Concatenate two 2D col-basis rotation matrices in the order m1->m2 */
template < class MatT_1, class MatT_2 > MAT_TEMP_2X2
matrix_concat_rotations_2D(const MatT_1& m1, const MatT_2& m2, col_basis) {
    return m2*m1;
}

/** Concatenate two 2D rotation matrices in the order m1->m2 */
template < class MatT_1, class MatT_2 > MAT_TEMP_2X2
matrix_concat_rotations_2D(const MatT_1& m1, const MatT_2& m2) {
    return matrix_concat_rotations_2D(m1,m2,typename MatT_1::basis_orient());
}

} // namespace detail

//////////////////////////////////////////////////////////////////////////////
// Matrix rotation difference
//////////////////////////////////////////////////////////////////////////////

/** Return the rotational 'difference' between two 3D rotation matrices */
template < class MatT_1, class MatT_2 > MAT_TEMP_3X3
matrix_rotation_difference(const MatT_1& m1, const MatT_2& m2) {
    return detail::matrix_concat_rotations(transpose(m1),m2);
}

/** Return the rotational 'difference' between two 2D rotation matrices */
template < class MatT_1, class MatT_2 > MAT_TEMP_2X2
matrix_rotation_difference_2D(const MatT_1& m1, const MatT_2& m2) {
    return detail::matrix_concat_rotations_2D(transpose(m1),m2);
}

//////////////////////////////////////////////////////////////////////////////
// Spherical linear interpolation of rotation matrices
//////////////////////////////////////////////////////////////////////////////

/* @todo: It might be as fast or faster to simply convert the matrices to
 * quaternions, interpolate, and convert back.
 *
 * @todo: The behavior of matrix slerp is currently a little different than
 * for quaternions: in the matrix function, when the two matrices are close
 * to identical the first is returned, while in the quaternion function the
 * quaternions are nlerp()'d in this case.
 *
 * I still need to do the equivalent of nlerp() for matrices, in which case
 * these functions could be revised to pass off to nlerp() when the matrices
 * are nearly aligned.
*/

/** Spherical linear interpolation of two 3D rotation matrices */
template < class MatT_1, class MatT_2, typename E > MAT_TEMP_3X3
matrix_slerp(const MatT_1& m1, const MatT_2& m2, E t,
    E tolerance = epsilon<E>::placeholder())
{
    typedef MAT_TEMP_3X3 temporary_type;

    temporary_type m = matrix_rotation_difference(m1,m2);
    matrix_scale_rotation_angle(m,t,tolerance);
    return detail::matrix_concat_rotations(m1,m);
}

/** Spherical linear interpolation of two 2D rotation matrices */
template < class MatT_1, class MatT_2, typename E > MAT_TEMP_2X2
matrix_slerp_2D(const MatT_1& m1, const MatT_2& m2, E t,
    E tolerance = epsilon<E>::placeholder())
{
    typedef MAT_TEMP_2X2 temporary_type;

    temporary_type m = matrix_rotation_difference_2D(m1,m2);
    matrix_scale_rotation_angle_2D(m,t,tolerance);
    return detail::matrix_concat_rotations_2D(m1,m);
}

#undef MAT_TEMP_3X3
#undef MAT_TEMP_2X2

//////////////////////////////////////////////////////////////////////////////
// Conversions
//////////////////////////////////////////////////////////////////////////////

/** Convert a 3D rotation matrix to an axis-angle pair */
template < class MatT, typename E, class A > void
matrix_to_axis_angle(
    const MatT& m,
    vector<E,A >& axis,
    E& angle,
    E tolerance = epsilon<E>::placeholder())
{
    typedef MatT matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear3D(m);

    axis.set(
        m.basis_element(1,2) - m.basis_element(2,1),
        m.basis_element(2,0) - m.basis_element(0,2),
        m.basis_element(0,1) - m.basis_element(1,0)
    );
	value_type l = length(axis);
    value_type tmo = trace_3x3(m) - value_type(1);

	if (l > tolerance) {
		axis /= l;
        angle = std::atan2(l, tmo); // l=2sin(theta),tmo=2cos(theta)
	} else if (tmo > value_type(0)) {
		axis.zero();
		angle = value_type(0);
	} else {
        size_t largest_diagonal_element =
            index_of_max(
                m.basis_element(0,0),
                m.basis_element(1,1),
                m.basis_element(2,2)
            );
        size_t i, j, k;
        cyclic_permutation(largest_diagonal_element, i, j, k);
		axis[i] =
            std::sqrt(
                m.basis_element(i,i) -
                m.basis_element(j,j) -
                m.basis_element(k,k) +
                value_type(1)
            ) * value_type(.5);
		value_type s = value_type(.5) / axis[i];
		axis[j] = m.basis_element(i,j) * s;
		axis[k] = m.basis_element(i,k) * s;
        angle = constants<value_type>::pi();
	}
}

/** Convert a 3D rotation matrix to an Euler-angle triple */
template < class MatT, typename Real >
void matrix_to_euler(
    const MatT& m,
    Real& angle_0,
    Real& angle_1,
    Real& angle_2,
    EulerOrder order,
    Real tolerance = epsilon<Real>::placeholder())
{
    typedef MatT matrix_type;
    typedef typename matrix_type::value_type value_type;

    /* Checking */
    detail::CheckMatLinear3D(m);

    size_t i, j, k;
    bool odd, repeat;
    detail::unpack_euler_order(order, i, j, k, odd, repeat);

    if (repeat) {
        value_type s1 = length(m.basis_element(j,i),m.basis_element(k,i));
        value_type c1 = m.basis_element(i,i);

        angle_1 = std::atan2(s1, c1);
        if (s1 > tolerance) {
            angle_0 = std::atan2(m.basis_element(j,i),m.basis_element(k,i));
            angle_2 = std::atan2(m.basis_element(i,j),-m.basis_element(i,k));
        } else {
            angle_0 = value_type(0);
            angle_2 = sign(c1) *
                std::atan2(-m.basis_element(k,j),m.basis_element(j,j));
        }
    } else {
        value_type s1 = -m.basis_element(i,k);
        value_type c1 = length(m.basis_element(i,i),m.basis_element(i,j));

        angle_1 = std::atan2(s1, c1);
        if (c1 > tolerance) {
            angle_0 = std::atan2(m.basis_element(j,k),m.basis_element(k,k));
            angle_2 = std::atan2(m.basis_element(i,j),m.basis_element(i,i));
        } else {
            angle_0 = value_type(0);
            angle_2 = -sign(s1) *
                std::atan2(-m.basis_element(k,j),m.basis_element(j,j));
        }
    }
    
    if (odd) {
        angle_0 = -angle_0;
        angle_1 = -angle_1;
        angle_2 = -angle_2;
    }
}

/** Convenience function to return a 3D vector containing the Euler angles
 * in the requested order.
 */
template < class MatT > vector< typename MatT::value_type, fixed<3> >
matrix_to_euler(
    const MatT& m,
    EulerOrder order,
    const typename MatT::value_type&
        tolerance = epsilon<typename MatT::value_type>::placeholder())
{
  typename MatT::value_type e0, e1, e2;
  matrix_to_euler(m, e0, e1, e2, order, tolerance);
  return vector< typename MatT::value_type, fixed<3> >(e0, e1, e2);
}

/** Convert a 2D rotation matrix to a rotation angle */
template < class MatT > typename MatT::value_type
matrix_to_rotation_2D(const MatT& m)
{
    /* Checking */
    detail::CheckMatLinear2D(m);
    
    return std::atan2(m.basis_element(0,1),m.basis_element(0,0));
}

} // namespace cml

#endif
