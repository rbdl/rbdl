/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef cml_util_h
#define cml_util_h

#include <algorithm>   // For std::min and std::max.
#include <cstdlib>     // For std::rand.
#include <cml/constants.h>

#if defined(_MSC_VER)
#pragma push_macro("min")
#pragma push_macro("max")
#undef min
#undef max
#endif

namespace cml {

/** Sign of input value as double. */
template < typename T >
double sign(T value) {
    return value < T(0) ? -1.0 : (value > T(0) ? 1.0 : 0.0);
}

/** Clamp input value to the range [min, max]. */
template < typename T >
T clamp(T value, T min, T max) {
    return std::max(std::min(value, max), min);
}

/** Test input value for inclusion in [min, max]. */
template < typename T >
bool in_range(T value, T min, T max) {
    return !(value < min) && !(value > max);
}

/** Map input value from [min1, max1] to [min2, max2]. */
template < typename T >
T map_range(T value, T min1, T max1, T min2, T max2) {
    return min2 + ((value - min1) / (max1 - min1)) * (max2 - min2);
}


/** Wrap std::acos() and clamp argument to [-1, 1]. */
template < typename T >
T acos_safe(T theta) {
    return T(std::acos(clamp(theta, T(-1.0), T(1.0))));
}

/** Wrap std::asin() and clamp argument to [-1, 1]. */
template < typename T >
T asin_safe(T theta) {
    return T(std::asin(clamp(theta, T(-1.0), T(1.0))));
}

/** Wrap std::sqrt() and clamp argument to [0, inf). */
template < typename T >
T sqrt_safe(T value) {
    return T(std::sqrt(std::max(value, T(0.0))));
}


/** Square a value. */
template < typename T >
T sqr(T value) {
    return value * value;
}

/** Cube a value. */
template < typename T >
T cub(T value) {
    return value * value * value;
}

/** Inverse square root. */
template < typename T >
T inv_sqrt(T value) {
    return T(1.0 / std::sqrt(value));
}


/* The next few functions deal with indexing. next() and prev() are useful
 * for operations involving the vertices of a polygon or other cyclic set,
 * and cyclic_permutation() is used by various functions that deal with
 * axes or basis vectors in a generic way. As these functions are only
 * relevant for unsigned integer types, I've just used size_t, but there
 * may be reasons I haven't thought of that they should be templated.
 */

/** Return next, with cycling, in a series of N non-negative integers. */
inline size_t next(size_t i, size_t N) {
    return (i + 1) % N;
}

/** Return previous, with cycling, in a series of N non-negative integers. */
inline size_t prev(size_t i, size_t N) {
    return i ? (i - 1) : (N - 1);
}

/** Cyclic permutation of the set { 0, 1 }, starting with 'first'. */
inline void cyclic_permutation(size_t first, size_t& i, size_t& j) {
    i = first;
    j = next(i, 2);
}

/** Cyclic permutation of the set { 0, 1, 2 }, starting with 'first'. */
inline void cyclic_permutation(size_t first, size_t& i, size_t& j, size_t& k)
{
    i = first;
    j = next(i, 3);
    k = next(j, 3);
}

/** Cyclic permutation of the set { 0, 1, 2, 3 }, starting with 'first'. */
inline void cyclic_permutation(
        size_t first, size_t& i, size_t& j, size_t& k, size_t& l)
{
    i = first;
    j = next(i, 4);
    k = next(j, 4);
    l = next(k, 4);
}


/** Convert radians to degrees. */
template < typename T >
T deg(T theta) {
    return theta * constants<T>::deg_per_rad();
}

/** Convert degrees to radians. */
template < typename T >
T rad(T theta) {
    return theta * constants<T>::rad_per_deg();
}

/* Note: Moving interpolation functions to interpolation.h */

#if 0
/** Linear interpolation of 2 values.
 *
 * @note The data points are assumed to be sampled at u = 0 and u = 1, so
 * for interpolation u must lie between 0 and 1.
 */
template <typename T, typename Scalar>
T lerp(const T& f0, const T& f1, Scalar u) {
    return (Scalar(1.0) - u) * f0 + u * f1;
}
#endif

#if 0
/** Bilinear interpolation of 4 values.
 *
 * @note The data points are assumed to be sampled at the corners of a unit
 * square, so for interpolation u and v must lie between 0 and 1,
 */
template <typename T, typename Scalar>
T bilerp(const T& f00, const T& f10,
         const T& f01, const T& f11,
         Scalar u, Scalar v)
{
    Scalar uv = u * v;
    return (
        (Scalar(1.0) - u - v + uv) * f00 +
                          (u - uv) * f10 +
                          (v - uv) * f01 +
                                uv * f11
    );
}
#endif

#if 0
/** Trilinear interpolation of 8 values.
 *
 * @note The data values are assumed to be sampled at the corners of a unit
 * cube, so for interpolation, u, v, and w must lie between 0 and 1.
 */
template <typename T, typename Scalar>
T trilerp(const T& f000, const T& f100,
          const T& f010, const T& f110,
          const T& f001, const T& f101,
          const T& f011, const T& f111,
          Scalar u, Scalar v, Scalar w)
{
    Scalar uv = u * v;
    Scalar vw = v * w;
    Scalar wu = w * u;
    Scalar uvw = uv * w;
    
    return (
        (Scalar(1.0) - u - v - w + uv + vw + wu - uvw) * f000 +
                                   (u - uv - wu + uvw) * f100 +
                                   (v - uv - vw + uvw) * f010 +
                                            (uv - uvw) * f110 +
                                   (w - vw - wu + uvw) * f001 +
                                            (wu - uvw) * f101 +
                                            (vw - uvw) * f011 +
                                                   uvw * f111
    );
}
#endif

/** Random binary (0,1) value. */
inline size_t random_binary() {
    return std::rand() % 2;
}

/** Random polar (-1,1) value. */
inline int random_polar() {
    return random_binary() ? 1 : -1;
}

/** Random real in [0,1]. */
inline double random_unit() {
    return double(std::rand()) / double(RAND_MAX);
}

/* Random integer in the range [min, max] */
inline long random_integer(long min, long max) {
    return min + std::rand() % (max - min + 1);
}

/* Random real number in the range [min, max] */
template < typename T >
T random_real(T min, T max) {
    return min + random_unit() * (max - min);
}

/** Squared length in R2. */
template < typename T >
T length_squared(T x, T y) {
    return x * x + y * y;
}

/** Squared length in R3. */
template < typename T >
T length_squared(T x, T y, T z) {
    return x * x + y * y + z * z;
}

/** Length in R2. */
template < typename T >
T length(T x, T y) {
    return std::sqrt(length_squared(x,y));
}

/** Length in R3. */
template < typename T >
T length(T x, T y, T z) {
    return std::sqrt(length_squared(x,y,z));
}

/** Index of maximum of 2 values. */
template < typename T >
size_t index_of_max(T a, T b) {
    return a > b ? 0 : 1;
}

/** Index of maximum of 2 values by magnitude. */
template < typename T >
size_t index_of_max_abs(T a, T b) {
    return index_of_max(std::fabs(a),std::fabs(b));
}

/** Index of minimum of 2 values. */
template < typename T >
size_t index_of_min(T a, T b) {
    return a < b ? 0 : 1;
}

/** Index of minimum of 2 values by magnitude. */
template < typename T >
size_t index_of_min_abs(T a, T b) {
    return index_of_min(std::fabs(a),std::fabs(b));
}

/** Index of maximum of 3 values. */
template < typename T >
size_t index_of_max(T a, T b, T c) {
    return a > b ? (c > a ? 2 : 0) : (b > c ? 1 : 2);
}

/** Index of maximum of 3 values by magnitude. */
template < typename T >
size_t index_of_max_abs(T a, T b, T c) {
    return index_of_max(std::fabs(a),std::fabs(b),std::fabs(c));
}

/** Index of minimum of 3 values. */
template < typename T >
size_t index_of_min(T a, T b, T c) {
    return a < b ? (c < a ? 2 : 0) : (b < c ? 1 : 2);
}

/** Index of minimum of 3 values by magnitude. */
template < typename T >
size_t index_of_min_abs(T a, T b, T c) {
    return index_of_min(std::fabs(a),std::fabs(b),std::fabs(c));
}

/** Wrap input value to the range [min,max]. */
template < typename T >
T wrap(T value, T min, T max) {
    max -= min;
    value = std::fmod(value - min, max);
    if (value < T(0)) {
        value += max;
    }
    return min + value;
}

/** Convert horizontal field of view to vertical field of view. */
template < typename T >
T xfov_to_yfov(T xfov, T aspect) {
    return T(2.0 * std::atan(std::tan(xfov * T(.5)) / double(aspect)));
}

/** Convert vertical field of view to horizontal field of view. */
template < typename T >
T yfov_to_xfov(T yfov, T aspect) {
    return T(2.0 * std::atan(std::tan(yfov * T(.5)) * double(aspect)));
}

/** Convert horizontal zoom to vertical zoom. */
template < typename T >
T xzoom_to_yzoom(T xzoom, T aspect) {
    return xzoom * aspect;
}

/** Convert vertical zoom to horizontal zoom. */
template < typename T >
T yzoom_to_xzoom(T yzoom, T aspect) {
    return yzoom / aspect;
}

/** Convert zoom factor to field of view. */
template < typename T >
T zoom_to_fov(T zoom) {
    return T(2) * T(std::atan(T(1) / zoom));
}

/** Convert field of view to zoom factor. */
template < typename T >
T fov_to_zoom(T fov) {
    return T(1) / T(std::tan(fov * T(.5)));
}

} // namespace cml

#if defined(_MSC_VER)
#pragma pop_macro("min")
#pragma pop_macro("max")
#endif

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
