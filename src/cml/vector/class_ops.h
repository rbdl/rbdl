/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Vector class operators.
 */

#ifndef vector_class_ops_h
#define vector_class_ops_h

#if defined(_MSC_VER) && _MSC_VER < 1400
#pragma warning(disable:4003)
// XXX Horrible hack to turn off warnings about "not enough actual params"
// for the macros below.
#endif

/* XXX HACK!!! This is a hack to resize in the assign() functions only when
 * auto resizing is turned off.
 */
#if !defined(CML_VECTOR_RESIZE_ON_ASSIGNMENT)
#define _DO_VECTOR_SET_RESIZE(_N_)  cml::et::detail::Resize(*this,_N_)
#else
#define _DO_VECTOR_SET_RESIZE(_N_)
#endif

/** Set a vector from 2 values. */
#define CML_ASSIGN_VEC_2                                                \
vector_type&                                                            \
set(ELEMENT_ARG_TYPE e0, ELEMENT_ARG_TYPE e1) {                         \
    _DO_VECTOR_SET_RESIZE(2);                                           \
    /* This is overkill, but simplifies size checking: */               \
    value_type v[] = {e0,e1};                                           \
    typedef et::OpAssign<Element,Element> OpT;                          \
    cml::vector< const value_type, external<2> > src(v);                \
    et::UnrollAssignment<OpT>(*this,src);                               \
    return *this;                                                       \
}

/** Set a vector from 3 values. */
#define CML_ASSIGN_VEC_3                                                \
vector_type&                                                            \
set(                                                                    \
        ELEMENT_ARG_TYPE e0,                                            \
        ELEMENT_ARG_TYPE e1,                                            \
        ELEMENT_ARG_TYPE e2                                             \
        )                                                               \
{                                                                       \
    _DO_VECTOR_SET_RESIZE(3);                                           \
    /* This is overkill, but simplifies size checking: */               \
    value_type v[] = {e0,e1,e2};                                        \
    typedef et::OpAssign<Element,Element> OpT;                          \
    cml::vector< const value_type, external<3> > src(v);                \
    et::UnrollAssignment<OpT>(*this,src);                               \
    return *this;                                                       \
}

/** Create a vector from 4 values. */
#define CML_ASSIGN_VEC_4                                                \
vector_type&                                                            \
set(                                                                    \
        ELEMENT_ARG_TYPE e0,                                            \
        ELEMENT_ARG_TYPE e1,                                            \
        ELEMENT_ARG_TYPE e2,                                            \
        ELEMENT_ARG_TYPE e3                                             \
        )                                                               \
{                                                                       \
    _DO_VECTOR_SET_RESIZE(4);                                           \
    /* This is overkill, but simplifies size checking: */               \
    value_type v[] = {e0,e1,e2,e3};                                     \
    typedef et::OpAssign<Element,Element> OpT;                          \
    cml::vector< const value_type, external<4> > src(v);                \
    et::UnrollAssignment<OpT>(*this,src);                               \
    return *this;                                                       \
}


/** Create a vector from 2 values. */
#define CML_CONSTRUCT_VEC_2(_add_)                                      \
vector(ELEMENT_ARG_TYPE e0, ELEMENT_ARG_TYPE e1) _add_ {                \
    set(e0,e1);                                                         \
}

/** Create a vector from 3 values. */
#define CML_CONSTRUCT_VEC_3(_add_)                                      \
vector(                                                                 \
        ELEMENT_ARG_TYPE e0,                                            \
        ELEMENT_ARG_TYPE e1,                                            \
        ELEMENT_ARG_TYPE e2                                             \
        ) _add_                                                         \
{                                                                       \
    set(e0,e1,e2);                                                      \
}

/** Create a vector from 4 values. */
#define CML_CONSTRUCT_VEC_4(_add_)                                      \
vector(                                                                 \
        ELEMENT_ARG_TYPE e0,                                            \
        ELEMENT_ARG_TYPE e1,                                            \
        ELEMENT_ARG_TYPE e2,                                            \
        ELEMENT_ARG_TYPE e3                                             \
        ) _add_                                                         \
{                                                                       \
    set(e0,e1,e2,e3);                                                   \
}

/** Create a (fixed-size) N vector from an N-1-vector and a scalar. */
#define CML_CONSTRUCT_FROM_SUBVEC(_add_)                                \
vector(                                                                 \
        const subvector_type& s,                                        \
        ELEMENT_ARG_TYPE e                                              \
        ) _add_                                                         \
{                                                                       \
    _DO_VECTOR_SET_RESIZE(s.size()+1);                                  \
    for(size_t i = 0; i < s.size(); ++ i)                               \
        (*this)[i] = s[i];                                              \
    (*this)[s.size()] = e;                                              \
}

/** Copy-construct a vector from a fixed-size array of values. */
#define CML_VEC_COPY_FROM_FIXED_ARRAY(_N_,_add_)                        \
vector(const value_type v[_N_]) _add_ {                                 \
    typedef et::OpAssign<Element,Element> OpT;                          \
    cml::vector< const value_type, external<_N_> > src(v);              \
    et::UnrollAssignment<OpT>(*this,src);                               \
}

/** Copy-construct a vector from a runtime-sized array of values. */
#define CML_VEC_COPY_FROM_ARRAY(_add_)                                  \
vector(const value_type* const v, size_t N) _add_ {                     \
    typedef et::OpAssign<Element,Element> OpT;                          \
    cml::vector<const value_type, external<> > src(v,N);                \
    et::UnrollAssignment<OpT>(*this,src);                               \
}

/** Copy-construct a vector.
 *
 * @internal This is required for GCC4, since it won't elide the default
 * copy constructor.
 */
#define CML_VEC_COPY_FROM_VECTYPE(_add_)                                \
vector(const vector_type& v) _add_ {                                    \
    typedef et::OpAssign<Element,Element> OpT;                          \
    et::UnrollAssignment<OpT>(*this,v);                                 \
}

/** Construct from an arbitrary vector.
 *
 * @param v the vector to copy from.
 */
#define CML_VEC_COPY_FROM_VEC                                           \
template<typename E, class AT>                                          \
vector(const vector<E,AT>& m) {                                         \
    typedef et::OpAssign<Element,E> OpT;                                \
    et::UnrollAssignment<OpT>(*this,m);                                 \
}

/** Construct from a vector expression.
 *
 * @param expr the expression to copy from.
 */
#define CML_VEC_COPY_FROM_VECXPR                                        \
template<class XprT>                                                    \
vector(VECXPR_ARG_TYPE e) {                                             \
    /* Verify that a promotion exists at compile time: */               \
    typedef typename et::VectorPromote<                                 \
        vector_type, typename XprT::result_type>::type result_type;     \
    typedef typename XprT::value_type src_value_type;                   \
    typedef et::OpAssign<Element,src_value_type> OpT;                   \
    et::UnrollAssignment<OpT>(*this,e);                                 \
}

/** Assign from the same vector type.
 *
 * @param v the vector to copy from.
 */
#define CML_VEC_ASSIGN_FROM_VECTYPE                                     \
vector_type& operator=(const vector_type& v) {                          \
    typedef et::OpAssign<Element,Element> OpT;                          \
    et::UnrollAssignment<OpT>(*this,v);                                 \
    return *this;                                                       \
}

/** Assign this vector from another using the given elementwise op.
 *
 * This allows assignment from arbitrary vector types.
 *
 * @param _op_ the operator (e.g. +=)
 * @param _op_name_ the op functor (e.g. et::OpAssign)
 */
#define CML_VEC_ASSIGN_FROM_VEC(_op_, _op_name_)                        \
template<typename E, class AT> vector_type&                             \
operator _op_ (const cml::vector<E,AT>& m) {                            \
    typedef _op_name_ <Element,E> OpT;                                  \
    cml::et::UnrollAssignment<OpT>(*this,m);                            \
    return *this;                                                       \
}

/** Declare a function to assign this vector from a vector expression.
 *
 * @param _op_ the operator (e.g. +=)
 * @param _op_name_ the op functor (e.g. et::OpAssign)
 */
#define CML_VEC_ASSIGN_FROM_VECXPR(_op_, _op_name_)                     \
template<class XprT> vector_type&                                       \
operator _op_ (VECXPR_ARG_TYPE e) {                                     \
    /* Verify that a promotion exists at compile time: */               \
    typedef typename et::VectorPromote<                                 \
        vector_type, typename XprT::result_type>::type result_type;     \
    typedef typename XprT::value_type src_value_type;                   \
    typedef _op_name_ <Element,src_value_type> OpT;                     \
    cml::et::UnrollAssignment<OpT>(*this,e);                            \
    return *this;                                                       \
}

/** Declare a function to assign this vector from a scalar.
 *
 * @param _op_ the operator (e.g. *=)
 * @param _op_name_ the op functor (e.g. et::OpAssign)
 *
 * @internal This shouldn't be used for ops, like +=, which aren't
 * defined in vector algebra.
 */
#define CML_VEC_ASSIGN_FROM_SCALAR(_op_, _op_name_)                     \
vector_type& operator _op_ (ELEMENT_ARG_TYPE s) {                       \
    typedef _op_name_ <Element,Element> OpT;                            \
    cml::et::UnrollAssignment<OpT>(*this,s);                            \
    return *this;                                                       \
}

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
