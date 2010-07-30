/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef quaternion_rotation_h
#define quaternion_rotation_h

#include <cml/mathlib/checking.h>

/* Functions related to quaternion rotations.
 *
 * Note: A number of these functions simply wrap calls to the corresponding
 * matrix functions. Some of them (the 'aim-at' and 'align' functions in
 * particular) might be considered a bit superfluous, since the resulting
 * quaternion will most likely be converted to a matrix at some point anyway.
 * However, they're included here for completeness, and for convenience in
 * cases where a quaternion is being used as the primary rotation
 * representation.
*/

namespace cml {

//////////////////////////////////////////////////////////////////////////////
// Rotation about world axes
//////////////////////////////////////////////////////////////////////////////

/** Build a quaternion representing a rotation about the given world axis */
template < class E, class A, class O, class C > void
quaternion_rotation_world_axis(quaternion<E,A,O,C>& q, size_t axis, E angle)
{
    typedef quaternion<E,A,O,C> quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef typename quaternion_type::order_type order_type;

    /* Checking */
    detail::CheckIndex3(axis);

    q.identity();

    const size_t W = order_type::W;
    const size_t I = order_type::X + axis;
    
    angle *= value_type(.5);
    q[I] = std::sin(angle);
    q[W] = std::cos(angle);
}

/** Build a quaternion representing a rotation about the world x axis */
template < class E, class A, class O, class C > void
quaternion_rotation_world_x(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotation_world_axis(q,0,angle);
}

/** Build a quaternion representing a rotation about the world y axis */
template < class E, class A, class O, class C > void
quaternion_rotation_world_y(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotation_world_axis(q,1,angle);
}

/** Build a quaternion representing a rotation about the world z axis */
template < class E, class A, class O, class C > void
quaternion_rotation_world_z(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotation_world_axis(q,2,angle);
}

//////////////////////////////////////////////////////////////////////////////
// Rotation from an axis-angle pair
//////////////////////////////////////////////////////////////////////////////

/** Build a quaternion from an axis-angle pair */
template < class E, class A, class O, class C, class VecT > void
quaternion_rotation_axis_angle(
    quaternion<E,A,O,C>& q, const VecT& axis, E angle)
{
    typedef quaternion<E,A,O,C> quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef typename quaternion_type::order_type order_type;
    
    /* Checking */
    detail::CheckVec3(axis);

    enum {
        W = order_type::W,
        X = order_type::X,
        Y = order_type::Y,
        Z = order_type::Z
    };
    
    angle *= value_type(.5);
    
    /* @todo: If and when we have a set() function that takes a vector and a
     * scalar, this can be written as:
     *
     * q.set(std::cos(angle), axis * std::sin(angle));
     *
     * In which case the enum will also not be necessary.
     */
    
    q[W] = std::cos(angle);
    value_type s = std::sin(angle);
    q[X] = axis[0] * s;
    q[Y] = axis[1] * s;
    q[Z] = axis[2] * s;
}

//////////////////////////////////////////////////////////////////////////////
// Rotation from a matrix
//////////////////////////////////////////////////////////////////////////////

/** Build a quaternion from a rotation matrix */
template < class E, class A, class O, class C, class MatT > void
quaternion_rotation_matrix(quaternion<E,A,O,C>& q, const MatT& m)
{
    typedef quaternion<E,A,O,C> quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef typename quaternion_type::order_type order_type;

    /* Checking */
    detail::CheckMatLinear3D(m);

    enum {
        W = order_type::W,
        X = order_type::X,
        Y = order_type::Y,
        Z = order_type::Z
    };

    value_type tr = trace_3x3(m);
    if (tr >= value_type(0)) {
        q[W] = std::sqrt(tr + value_type(1)) * value_type(.5);
        value_type s = value_type(.25) / q[W];
        q[X] = (m.basis_element(1,2) - m.basis_element(2,1)) * s;
        q[Y] = (m.basis_element(2,0) - m.basis_element(0,2)) * s;
        q[Z] = (m.basis_element(0,1) - m.basis_element(1,0)) * s;
    } else {
        size_t largest_diagonal_element =
            index_of_max(
                m.basis_element(0,0),
                m.basis_element(1,1),
                m.basis_element(2,2)
            );
        size_t i, j, k;
        cyclic_permutation(largest_diagonal_element, i, j, k);
        const size_t I = X + i;
        const size_t J = X + j;
        const size_t K = X + k;
        q[I] =
            std::sqrt(
                m.basis_element(i,i) -
                m.basis_element(j,j) -
                m.basis_element(k,k) +
                value_type(1)
            ) * value_type(.5);
        value_type s = value_type(.25) / q[I];
        q[J] = (m.basis_element(i,j) + m.basis_element(j,i)) * s;
        q[K] = (m.basis_element(i,k) + m.basis_element(k,i)) * s;
        q[W] = (m.basis_element(j,k) - m.basis_element(k,j)) * s;
    }
}

//////////////////////////////////////////////////////////////////////////////
// Rotation from Euler angles
//////////////////////////////////////////////////////////////////////////////

/** Build a quaternion from an Euler-angle triple */
template < class E, class A, class O, class C > void
quaternion_rotation_euler(
    quaternion<E,A,O,C>& q, E angle_0, E angle_1, E angle_2,
    EulerOrder order)
{
    typedef quaternion<E,A,O,C> quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef typename quaternion_type::order_type order_type;

    size_t i, j, k;
    bool odd, repeat;
    detail::unpack_euler_order(order, i, j, k, odd, repeat);
    
    const size_t W = order_type::W;
    const size_t I = order_type::X + i;
    const size_t J = order_type::X + j;
    const size_t K = order_type::X + k;

    if (odd) {
        angle_1 = -angle_1;
    }

    angle_0 *= value_type(.5);
    angle_1 *= value_type(.5);
    angle_2 *= value_type(.5);
    
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
        q[I] = c1 * (c0s2 + s0c2);
        q[J] = s1 * (c0c2 + s0s2);
        q[K] = s1 * (c0s2 - s0c2);
        q[W] = c1 * (c0c2 - s0s2);
    } else {
        q[I] = c1 * s0c2 - s1 * c0s2;
        q[J] = c1 * s0s2 + s1 * c0c2;
        q[K] = c1 * c0s2 - s1 * s0c2;
        q[W] = c1 * c0c2 + s1 * s0s2;
    }
    if (odd) {
        q[J] = -q[J];
    }
}

//////////////////////////////////////////////////////////////////////////////
// Rotation to align with a vector, multiple vectors, or the view plane
//////////////////////////////////////////////////////////////////////////////

/** See vector_ortho.h for details */
template < typename E,class A,class O,class C,class VecT_1,class VecT_2 > void
quaternion_rotation_align(
    quaternion<E,A,O,C>& q,
    const VecT_1& align,
    const VecT_2& reference,
    bool normalize = true,
    AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_align(m,align,reference,normalize,order);
    quaternion_rotation_matrix(q,m);
}

/** See vector_ortho.h for details */
template < typename E, class A, class O, class C, class VecT > void
quaternion_rotation_align(quaternion<E,A,O,C>& q, const VecT& align,
    bool normalize = true, AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_align(m,align,normalize,order);
    quaternion_rotation_matrix(q,m);
}

/** See vector_ortho.h for details */
template < typename E,class A,class O,class C,class VecT_1,class VecT_2 > void
quaternion_rotation_align_axial(quaternion<E,A,O,C>& q, const VecT_1& align,
    const VecT_2& axis, bool normalize = true,
    AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_align_axial(m,align,axis,normalize,order);
    quaternion_rotation_matrix(q,m);
}

/** See vector_ortho.h for details */
template < typename E, class A, class O, class C, class MatT > void
quaternion_rotation_align_viewplane(
    quaternion<E,A,O,C>& q,
    const MatT& view_matrix,
    Handedness handedness,
    AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_align_viewplane(m,view_matrix,handedness,order);
    quaternion_rotation_matrix(q,m);
}

/** See vector_ortho.h for details */
template < typename E, class A, class O, class C, class MatT > void
quaternion_rotation_align_viewplane_LH(
    quaternion<E,A,O,C>& q,
    const MatT& view_matrix,
    AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_align_viewplane_LH(m,view_matrix,order);
    quaternion_rotation_matrix(q,m);
}

/** See vector_ortho.h for details */
template < typename E, class A, class O, class C, class MatT > void
quaternion_rotation_align_viewplane_RH(
    quaternion<E,A,O,C>& q,
    const MatT& view_matrix,
    AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_align_viewplane_RH(m,view_matrix,order);
    quaternion_rotation_matrix(q,m);
}

//////////////////////////////////////////////////////////////////////////////
// Rotation to aim at a target
//////////////////////////////////////////////////////////////////////////////

/** See vector_ortho.h for details */
template < typename E, class A, class O, class C,
    class VecT_1, class VecT_2, class VecT_3 > void
quaternion_rotation_aim_at(
    quaternion<E,A,O,C>& q,
    const VecT_1& pos,
    const VecT_2& target,
    const VecT_3& reference,
    AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_aim_at(m,pos,target,reference,order);
    quaternion_rotation_matrix(q,m);
}

/** See vector_ortho.h for details */
template < typename E, class A, class O, class C,
    class VecT_1, class VecT_2 > void
quaternion_rotation_aim_at(
    quaternion<E,A,O,C>& q,
    const VecT_1& pos,
    const VecT_2& target,
    AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_aim_at(m,pos,target,order);
    quaternion_rotation_matrix(q,m);
}

/** See vector_ortho.h for details */
template < typename E, class A, class O, class C,
    class VecT_1, class VecT_2, class VecT_3 > void
quaternion_rotation_aim_at_axial(
    quaternion<E,A,O,C>& q,
    const VecT_1& pos,
    const VecT_2& target,
    const VecT_3& axis,
    AxisOrder order = axis_order_zyx)
{
    typedef matrix< E,fixed<3,3>,row_basis,row_major > matrix_type;
    
    matrix_type m;
    matrix_rotation_aim_at_axial(m,pos,target,axis,order);
    quaternion_rotation_matrix(q,m);
}

//////////////////////////////////////////////////////////////////////////////
// Relative rotation about world axes
//////////////////////////////////////////////////////////////////////////////

/* Rotate a quaternion about the given world axis */
template < class E, class A, class O, class C > void
quaternion_rotate_about_world_axis(quaternion<E,A,O,C>& q,size_t axis,E angle)
{
    typedef quaternion<E,A,O,C> quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef typename quaternion_type::order_type order_type;

    /* Checking */
    detail::CheckIndex3(axis);

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);
    
    const size_t W = order_type::W;
    const size_t I = order_type::X + i;
    const size_t J = order_type::X + j;
    const size_t K = order_type::X + k;
    
    angle *= value_type(.5);
    value_type s = value_type(std::sin(angle));
    value_type c = value_type(std::cos(angle));

    quaternion_type result;
    result[I] = c * q[I] + s * q[W];
    result[J] = c * q[J] - s * q[K];
    result[K] = c * q[K] + s * q[J];
    result[W] = c * q[W] - s * q[I];
    q = result;
}

/* Rotate a quaternion about the world x axis */
template < class E, class A, class O, class C > void
quaternion_rotate_about_world_x(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotate_about_world_axis(q,0,angle);
}

/* Rotate a quaternion about the world y axis */
template < class E, class A, class O, class C > void
quaternion_rotate_about_world_y(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotate_about_world_axis(q,1,angle);
}

/* Rotate a quaternion about the world z axis */
template < class E, class A, class O, class C > void
quaternion_rotate_about_world_z(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotate_about_world_axis(q,2,angle);
}

//////////////////////////////////////////////////////////////////////////////
// Relative rotation about local axes
//////////////////////////////////////////////////////////////////////////////

/* Rotate a quaternion about the given local axis */
template < class E, class A, class O, class C > void
quaternion_rotate_about_local_axis(quaternion<E,A,O,C>& q,size_t axis,E angle)
{
    typedef quaternion<E,A,O,C> quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef typename quaternion_type::order_type order_type;

    /* Checking */
    detail::CheckIndex3(axis);

    size_t i, j, k;
    cyclic_permutation(axis, i, j, k);
    
    const size_t W = order_type::W;
    const size_t I = order_type::X + i;
    const size_t J = order_type::X + j;
    const size_t K = order_type::X + k;
    
    angle *= value_type(.5);
    value_type s = value_type(std::sin(angle));
    value_type c = value_type(std::cos(angle));

    quaternion_type result;
    result[I] = c * q[I] + s * q[W];
    result[J] = c * q[J] + s * q[K];
    result[K] = c * q[K] - s * q[J];
    result[W] = c * q[W] - s * q[I];
    q = result;
}

/* Rotate a quaternion about its local x axis */
template < class E, class A, class O, class C > void
quaternion_rotate_about_local_x(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotate_about_local_axis(q,0,angle);
}

/* Rotate a quaternion about its local y axis */
template < class E, class A, class O, class C > void
quaternion_rotate_about_local_y(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotate_about_local_axis(q,1,angle);
}

/* Rotate a quaternion about its local z axis */
template < class E, class A, class O, class C > void
quaternion_rotate_about_local_z(quaternion<E,A,O,C>& q, E angle) {
    quaternion_rotate_about_local_axis(q,2,angle);
}

//////////////////////////////////////////////////////////////////////////////
// Rotation from vector to vector
//////////////////////////////////////////////////////////////////////////////

/* http://www.martinb.com/maths/algebra/vectors/angleBetween/index.htm. */

/** Build a quaternion to rotate from one vector to another */
template < class E,class A,class O,class C,class VecT_1,class VecT_2 > void
quaternion_rotation_vec_to_vec(
    quaternion<E,A,O,C>& q,
    const VecT_1& v1,
    const VecT_2& v2,
    bool unit_length_vectors = false)
{
    typedef quaternion<E,A,O,C> quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef vector< value_type, fixed<3> > vector_type;
    
    /* Checking handled by cross() */

    /* @todo: If at some point quaternion<> has a set() function that takes a
     * vector and a scalar, this can then be written as:
     *
     * if (...) {
     *     q.set(value_type(1)+dot(v1,v2), cross(v1,v2));
     * } else {
     *     q.set(std::sqrt(...)+dot(v1,v2), cross(v1,v2));
     * }
     */
     
    vector_type c = cross(v1,v2);
    if (unit_length_vectors) {
        q = quaternion_type(value_type(1) + dot(v1,v2), c.data());
    } else {
        q = quaternion_type(
            std::sqrt(v1.length_squared() * v2.length_squared()) + dot(v1,v2),
            c/*.data()*/
        );
    }
    q.normalize();
}

//////////////////////////////////////////////////////////////////////////////
// Scale the angle of a rotation matrix
//////////////////////////////////////////////////////////////////////////////

template < typename E, class A, class O, class C > void
quaternion_scale_angle(quaternion<E,A,O,C>& q, E t,
    E tolerance = epsilon<E>::placeholder())
{
    typedef vector< E,fixed<3> > vector_type;
    typedef typename vector_type::value_type value_type;
    
    vector_type axis;
    value_type angle;
    quaternion_to_axis_angle(q, axis, angle, tolerance);
    quaternion_rotation_axis_angle(q, axis, angle * t);
}

//////////////////////////////////////////////////////////////////////////////
// Support functions for uniform handling of pos- and neg-cross quaternions
//////////////////////////////////////////////////////////////////////////////

namespace detail {

/** Concatenate two quaternions in the order q1->q2 */
template < class QuatT_1, class QuatT_2 >
typename et::QuaternionPromote2<QuatT_1,QuatT_2>::temporary_type
quaternion_rotation_difference(
    const QuatT_1& q1, const QuatT_2& q2, positive_cross)
{
    return q2 * conjugate(q1);
}

/** Concatenate two quaternions in the order q1->q2 */
template < class QuatT_1, class QuatT_2 >
typename et::QuaternionPromote2<QuatT_1,QuatT_2>::temporary_type
quaternion_rotation_difference(
    const QuatT_1& q1, const QuatT_2& q2, negative_cross)
{
    return conjugate(q1) * q2;
}

} // namespace detail

//////////////////////////////////////////////////////////////////////////////
// Quaternions rotation difference
//////////////////////////////////////////////////////////////////////////////

/** Return the rotational 'difference' between two quaternions */
template < class QuatT_1, class QuatT_2 >
typename et::QuaternionPromote2<QuatT_1,QuatT_2>::temporary_type
quaternion_rotation_difference(const QuatT_1& q1, const QuatT_2& q2) {
    return detail::quaternion_rotation_difference(
        q1, q2, typename QuatT_1::cross_type());
}

//////////////////////////////////////////////////////////////////////////////
// Conversions
//////////////////////////////////////////////////////////////////////////////

/** Convert a quaternion to an axis-angle pair */
template < class QuatT, typename E, class A > void
quaternion_to_axis_angle(
    const QuatT& q,
    vector<E,A>& axis,
    E& angle,
    E tolerance = epsilon<E>::placeholder())
{
    typedef QuatT quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef typename quaternion_type::order_type order_type;
    
    /* Checking */
    detail::CheckQuat(q);

    axis = q.imaginary();
    value_type l = length(axis);
    if (l > tolerance) {
        axis /= l;
        angle = value_type(2) * std::atan2(l,q.real());
    } else {
        axis.zero();
        angle = value_type(0);
    }
}

/** Convert a quaternion to an Euler-angle triple
 *
 * Note: I've implemented direct quaternion-to-Euler conversion, but as far as
 * I can tell it more or less reduces to converting the quaternion to a matrix
 * as you go. The direct method is a little more efficient in that it doesn't
 * require a temporary and only the necessary matrix elements need be
 * computed. However, the implementation is complex and there's considerable
 * opportunity for error, so from a development and debugging standpoint I
 * think it's better to just perform the conversion via matrix_to_euler(),
 * which is already known to be correct.
*/

template < class QuatT, typename Real > void
quaternion_to_euler(
    const QuatT& q,
    Real& angle_0,
    Real& angle_1,
    Real& angle_2,
    EulerOrder order,
    Real tolerance = epsilon<Real>::placeholder())
{
    typedef QuatT quaternion_type;
    typedef typename quaternion_type::value_type value_type;
    typedef matrix< value_type,fixed<3,3>,row_basis,row_major > matrix_type;

    matrix_type m;
    matrix_rotation_quaternion(m, q);
    matrix_to_euler(m, angle_0, angle_1, angle_2, order, tolerance);
}

} // namespace cml

#endif
