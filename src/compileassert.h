/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _COMPILE_ASSERT_H
#define _COMPILE_ASSERT_H

/* 
 * This is a simple compile time assertion tool taken from:
 *   http://blogs.msdn.com/b/abhinaba/archive/2008/10/27/c-c-compile-time-asserts.aspx
 * written by Abhinaba Basu!
 *
 * Thanks!
 */

#ifdef __cplusplus

#define JOIN( X, Y ) JOIN2(X,Y)
#define JOIN2( X, Y ) X##Y

namespace static_assert
{
    template <bool> struct STATIC_ASSERT_FAILURE;
    template <> struct STATIC_ASSERT_FAILURE<true> { enum { value = 1 }; };

    template<int x> struct static_assert_test{};
}

#define COMPILE_ASSERT(x) \
    typedef ::static_assert::static_assert_test<\
        sizeof(::static_assert::STATIC_ASSERT_FAILURE< (bool)( x ) >)>\
            JOIN(_static_assert_typedef, __LINE__)

#else // __cplusplus

#define COMPILE_ASSERT(x) extern int __dummy[(int)x]

#endif // __cplusplus

#define VERIFY_EXPLICIT_CAST(from, to) COMPILE_ASSERT(sizeof(from) == sizeof(to)) 

// _COMPILE_ASSERT_H_
#endif
