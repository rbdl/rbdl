/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Useful constants.
 */

#ifndef cml_constants_h
#define cml_constants_h

#include <cmath>

#if !defined(M_PI)
#define M_PI 3.14159265358979323846264338327950288
#endif

#if !defined(M_SQRT2)
#define M_SQRT2 1.41421356237309504880168872420969808
#endif

#if !defined(M_E)
#define M_E 2.71828182845904523536028747135266250
#endif

namespace cml {

#if 1

/** Templated constants struct.
 *
 * Either float or double can be used.
 */
template<typename Float>
struct constants {
    static Float pi()          { return Float(M_PI); }
    static Float two_pi()      { return Float(2.*M_PI); }
    static Float inv_pi()      { return Float(1./M_PI); }
    static Float inv_two_pi()  { return Float(1./(2.*M_PI)); }
    static Float pi_over_2()   { return Float(M_PI/2.); }
    static Float pi_over_4()   { return Float(M_PI/4.); }
    static Float deg_per_rad() { return Float(180./M_PI); }
    static Float rad_per_deg() { return Float(M_PI/180.); }

    static Float sqrt_2() { return Float(M_SQRT2); }
    static Float sqrt_3() { return Float(1.732050807568877293527446341505); }
    static Float sqrt_5() { return Float(2.236067977499789696409173668731); }
    static Float sqrt_6() { return Float(2.449489742783178098197284074705); }

    static Float e() { return Float(M_E); }
};

#else

/* XXX This version requires an explicit instantiation of *every* constant
 * below, e.g.:
 *
 * template<typename F> const F cml::constants<F>::pi;
 */
/** Templated constants struct.
 *
 * Either float or double can be used.
 */
template<typename Float>
struct constants {
    static const Float pi = M_PI;
    static const Float two_pi = 2.*M_PI;
    static const Float inv_pi = 1./M_PI;                /* 1/pi */
    static const Float inv_two_pi = 1./(2.*M_PI);       /* 1/(2*pi) */
    static const Float pi_over_2 = M_PI/2.;             /* pi/2 */
    static const Float pi_over_4 = M_PI/4.;             /* pi/4 */
    static const Float deg_per_rad = 180./M_PI;
    static const Float rad_per_deg = M_PI/180.;
    static const Float sqrt_2 = M_SQRT2;
    static const Float sqrt_3 = 1.73205080756887729352744634150587237;
    static const Float sqrt_5 = 2.23606797749978969640917366873127624;
    static const Float sqrt_6 = 2.44948974278317809819728407470589139;
    static const Float e = M_E;
};

#endif

} // namespace cml

#endif
