/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef helper_h
#define helper_h

#include <cstddef>
#include <cml/constants.h>

namespace cml {

/* Helper classes for axis order, coordinate system handedness, z-clipping
 * range and spherical coordinate type.
 */
 
//////////////////////////////////////////////////////////////////////////////
// Euler order
//////////////////////////////////////////////////////////////////////////////

enum EulerOrder {
    euler_order_xyz, // 0x00 [0000]
    euler_order_xyx, // 0x01 [0001]
    euler_order_xzy, // 0x02 [0010]
    euler_order_xzx, // 0x03 [0011]
    euler_order_yzx, // 0x04 [0100]
    euler_order_yzy, // 0x05 [0101]
    euler_order_yxz, // 0x06 [0110]
    euler_order_yxy, // 0x07 [0111]
    euler_order_zxy, // 0x08 [1000]
    euler_order_zxz, // 0x09 [1001]
    euler_order_zyx, // 0x0A [1010]
    euler_order_zyz  // 0x0B [1011]
};

namespace detail {

inline void unpack_euler_order(
    EulerOrder order,
    size_t& i,
    size_t& j,
    size_t& k,
    bool& odd,
    bool& repeat)
{
    enum { REPEAT = 0x01, ODD = 0x02, AXIS = 0x0C };

    repeat = order & REPEAT;
    odd = ((order & ODD) == ODD);
    size_t offset = size_t(odd);
    i = (order & AXIS) % 3;
    j = (i + 1 + offset) % 3;
    k = (i + 2 - offset) % 3;
}

} // namespace detail

//////////////////////////////////////////////////////////////////////////////
// Axis order
//////////////////////////////////////////////////////////////////////////////

enum AxisOrder {
    axis_order_xyz = euler_order_xyz, // 0x00 [0000]
    axis_order_xzy = euler_order_xzy, // 0x02 [0010]
    axis_order_yzx = euler_order_yzx, // 0x04 [0100]
    axis_order_yxz = euler_order_yxz, // 0x06 [0110]
    axis_order_zxy = euler_order_zxy, // 0x08 [1000]
    axis_order_zyx = euler_order_zyx, // 0x0A [1010]
};

namespace detail {

inline void unpack_axis_order(
    AxisOrder order,
    size_t& i,
    size_t& j,
    size_t& k,
    bool& odd)
{
    enum { ODD = 0x02, AXIS = 0x0C };

    odd = ((order & ODD) == ODD);
    size_t offset = size_t(odd);
    i = (order & AXIS) % 3;
    j = (i + 1 + offset) % 3;
    k = (i + 2 - offset) % 3;
}

inline AxisOrder pack_axis_order(size_t i, bool odd) {
    return AxisOrder((i << 2) | (size_t(odd) << 1));
}

inline AxisOrder swap_axis_order(AxisOrder order)
{
    size_t i, j, k;
    bool odd;
    unpack_axis_order(order, i, j, k, odd);
    return pack_axis_order(j, !odd);
}

} // namespace detail

//////////////////////////////////////////////////////////////////////////////
// Axis order 2D
//////////////////////////////////////////////////////////////////////////////

enum AxisOrder2D {
    axis_order_xy = axis_order_xyz, // 0x00 [0000]
    axis_order_yx = axis_order_yxz, // 0x06 [0110]
};

namespace detail {

inline void unpack_axis_order_2D(
    AxisOrder2D order,
    size_t& i,
    size_t& j,
    bool& odd)
{
    enum { ODD = 0x02, AXIS = 0x0C };

    odd = ((order & ODD) == ODD);
    size_t offset = size_t(odd);
    i = (order & AXIS) % 3;
    j = (i + 1 + offset) % 3;
}

} // namespace detail

//////////////////////////////////////////////////////////////////////////////
// Handedness
//////////////////////////////////////////////////////////////////////////////

enum Handedness { left_handed, right_handed };

//////////////////////////////////////////////////////////////////////////////
// Z clip
//////////////////////////////////////////////////////////////////////////////

enum ZClip { z_clip_neg_one, z_clip_zero };

//////////////////////////////////////////////////////////////////////////////
// Spherical coordinate type
//////////////////////////////////////////////////////////////////////////////

enum SphericalType { latitude, colatitude };

} // namespace cml

#endif
