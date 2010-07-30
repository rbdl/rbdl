/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef core_meta_common_h
#define core_meta_common_h

namespace cml {

/** Type of a true statement. */
struct true_type {};

/** Type of a false statement. */
struct false_type {};

template<bool B> struct is_true {
    typedef false_type result;
};

template<> struct is_true<true> {
    typedef true_type result;
};

/** A "type pair". */
template<typename T1, typename T2> struct type_pair {
    typedef T1 first;
    typedef T2 second;
};

/** A "type quadruple". */
template<typename T1, typename T2, typename T3, typename T4>
struct type_quad {
    typedef T1 first;
    typedef T2 second;
    typedef T3 third;
    typedef T3 fourth;
};

/** Match any type (for use with same_type<> and select_switch<>). */
struct any_type {};

/** Determine if two types are the same.
 *
 * Defaults to false.
 */
template<typename T, typename U> struct same_type {
    typedef false_type result;
    enum { is_true = false, is_false = true };
};

/** Match the same type for both of same_type's template arguments. */
template<typename T> struct same_type<T,T> {
    typedef true_type result;
    enum { is_true = true, is_false = false };
};

/** Match a type and any_type. */
template<typename T> struct same_type<T,any_type> {
    typedef true_type result;
    enum { is_true = true, is_false = false };
};

/** Match a type and any_type. */
template<typename T> struct same_type<any_type,T> {
    typedef true_type result;
    enum { is_true = true, is_false = false };
};

/** Disambiguate pair of any_type's. */
template<> struct same_type<any_type,any_type> {
    typedef true_type result;
    enum { is_true = true, is_false = false };
};

/** Remove a reference qualifier from a type. */
template<typename T> struct remove_reference {
    template<typename Q, typename Dummy> struct helper {
        typedef Q type;
    };

    template<typename Q> struct helper<Q&, void> {
        typedef Q type;
    };

    template<typename Q> struct helper<const Q&, void> {
        typedef const Q type;
    };

    typedef typename helper<T,void>::type type;
};

/** Remove a const qualifier from a type. */
template<typename T> struct remove_const {
    template<typename Q, typename Dummy> struct helper {
        typedef Q type;
    };

    template<typename Q> struct helper<const Q, void> {
        typedef Q type;
    };

    typedef typename helper<T,void>::type type;
};

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
