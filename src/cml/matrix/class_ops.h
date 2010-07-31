/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * @note GCC4 requires a copy constructor to elide---it won't elide a
 * compiler-generated copy constructor!
 */

#ifndef matrix_class_ops_h
#define matrix_class_ops_h

#if defined(_MSC_VER) && _MSC_VER < 1400
#pragma warning(disable:4003)
// XXX Horrible hack to turn off warnings about "not enough actual params"
// for the macros below.
#endif

/* This is to circumvent the problem of commas in a macro argument.  It's
 * used to instantiate CML_ACCUMULATED_MATRIX_MULT:
 */
#define TEMPLATED_MATRIX_MACRO matrix<E,AT,BO,L>

/* XXX HACK!!! This is a hack to resize in the assign() functions only when
 * auto resizing is turned off.
 */
#if !defined(CML_MATRIX_RESIZE_ON_ASSIGNMENT)
#define _DO_MATRIX_SET_RESIZE(_R_,_C_) cml::et::detail::Resize(*this,_R_,_C_)
#else
#define _DO_MATRIX_SET_RESIZE(_R_,_C_)
#endif

/** Set a matrix from 2x2 values.
 *
 * The layout assumed for the values is that of the matrix being assigned.
 */
#define CML_ASSIGN_MAT_22                                               \
matrix_type&                                                            \
set(                                                                    \
    ELEMENT_ARG_TYPE e00, ELEMENT_ARG_TYPE e01,                         \
    ELEMENT_ARG_TYPE e10, ELEMENT_ARG_TYPE e11                          \
    )                                                                   \
{                                                                       \
    _DO_MATRIX_SET_RESIZE(2,2);                                         \
    /* This is overkill, but simplifies size checking: */               \
    value_type v[2][2] = {{e00,e01},{e10,e11}};                         \
    typedef et::OpAssign<Element,Element> OpT;                          \
    typedef const value_type element;                                   \
    cml::matrix<element, external<2,2>, basis_orient, row_major>        \
        src(&v[0][0]);                                                  \
    et::UnrollAssignment<OpT>(*this,src);                               \
    return *this;                                                       \
}

/** Create a matrix from 3x3 values.
 *
 * The layout assumed for the values is that of the matrix being assigned.
 */
#define CML_ASSIGN_MAT_33                                               \
matrix_type&                                                            \
set(                                                                    \
    ELEMENT_ARG_TYPE e00, ELEMENT_ARG_TYPE e01, ELEMENT_ARG_TYPE e02,   \
    ELEMENT_ARG_TYPE e10, ELEMENT_ARG_TYPE e11, ELEMENT_ARG_TYPE e12,   \
    ELEMENT_ARG_TYPE e20, ELEMENT_ARG_TYPE e21, ELEMENT_ARG_TYPE e22    \
    )                                                                   \
{                                                                       \
    _DO_MATRIX_SET_RESIZE(3,3);                                         \
    /* This is overkill, but simplifies size checking: */               \
    value_type v[3][3] = {                                              \
        {e00,e01,e02},                                                  \
        {e10,e11,e12},                                                  \
        {e20,e21,e22}                                                   \
    };                                                                  \
    typedef et::OpAssign<Element,Element> OpT;                          \
    typedef const value_type element;                                   \
    cml::matrix<element, external<3,3>, basis_orient, row_major>        \
        src(&v[0][0]);                                                  \
    et::UnrollAssignment<OpT>(*this,src);                               \
    return *this;                                                       \
}

/** Create a matrix from 4x4 values.
 *
 * The layout assumed for the values is that of the matrix being assigned.
 */
#define CML_ASSIGN_MAT_44                                               \
matrix_type&                                                            \
set(                                                                    \
    ELEMENT_ARG_TYPE e00, ELEMENT_ARG_TYPE e01,                         \
        ELEMENT_ARG_TYPE e02, ELEMENT_ARG_TYPE e03,                     \
    ELEMENT_ARG_TYPE e10, ELEMENT_ARG_TYPE e11,                         \
        ELEMENT_ARG_TYPE e12, ELEMENT_ARG_TYPE e13,                     \
    ELEMENT_ARG_TYPE e20, ELEMENT_ARG_TYPE e21,                         \
        ELEMENT_ARG_TYPE e22, ELEMENT_ARG_TYPE e23,                     \
    ELEMENT_ARG_TYPE e30, ELEMENT_ARG_TYPE e31,                         \
        ELEMENT_ARG_TYPE e32, ELEMENT_ARG_TYPE e33                      \
    )                                                                   \
{                                                                       \
    _DO_MATRIX_SET_RESIZE(4,4);                                         \
    /* This is overkill, but simplifies size checking: */               \
    value_type v[4][4] = {                                              \
        {e00,e01,e02,e03},                                              \
        {e10,e11,e12,e13},                                              \
        {e20,e21,e22,e23},                                              \
        {e30,e31,e32,e33}                                               \
    };                                                                  \
    typedef et::OpAssign<Element,Element> OpT;                          \
    typedef const value_type element;                                   \
    cml::matrix<element, external<4,4>, basis_orient, row_major>        \
        src(&v[0][0]);                                                  \
    et::UnrollAssignment<OpT>(*this,src);                               \
    return *this;                                                       \
}

/** Create a matrix from 2x2 values.
 *
 * The layout assumed for the values is that of the matrix being assigned.
 */
#define CML_CONSTRUCT_MAT_22                                            \
matrix(                                                                 \
    ELEMENT_ARG_TYPE e00, ELEMENT_ARG_TYPE e01,                         \
    ELEMENT_ARG_TYPE e10, ELEMENT_ARG_TYPE e11                          \
    )                                                                   \
{                                                                       \
    set(                                                                \
         e00,e01,                                                       \
         e10,e11                                                        \
    );                                                                  \
}

/** Create a matrix from 3x3 values.
 *
 * The layout assumed for the values is that of the matrix being assigned.
 */
#define CML_CONSTRUCT_MAT_33                                            \
matrix(                                                                 \
    ELEMENT_ARG_TYPE e00, ELEMENT_ARG_TYPE e01, ELEMENT_ARG_TYPE e02,   \
    ELEMENT_ARG_TYPE e10, ELEMENT_ARG_TYPE e11, ELEMENT_ARG_TYPE e12,   \
    ELEMENT_ARG_TYPE e20, ELEMENT_ARG_TYPE e21, ELEMENT_ARG_TYPE e22    \
    )                                                                   \
{                                                                       \
    set(                                                                \
         e00,e01,e02,                                                   \
         e10,e11,e12,                                                   \
         e20,e21,e22                                                    \
    );                                                                  \
}

/** Create a matrix from 4x4 values.
 *
 * The layout assumed for the values is that of the matrix being assigned.
 */
#define CML_CONSTRUCT_MAT_44                                            \
matrix(                                                                 \
    ELEMENT_ARG_TYPE e00, ELEMENT_ARG_TYPE e01,                         \
        ELEMENT_ARG_TYPE e02, ELEMENT_ARG_TYPE e03,                     \
    ELEMENT_ARG_TYPE e10, ELEMENT_ARG_TYPE e11,                         \
        ELEMENT_ARG_TYPE e12, ELEMENT_ARG_TYPE e13,                     \
    ELEMENT_ARG_TYPE e20, ELEMENT_ARG_TYPE e21,                         \
        ELEMENT_ARG_TYPE e22, ELEMENT_ARG_TYPE e23,                     \
    ELEMENT_ARG_TYPE e30, ELEMENT_ARG_TYPE e31,                         \
        ELEMENT_ARG_TYPE e32, ELEMENT_ARG_TYPE e33                      \
    )                                                                   \
{                                                                       \
    set(                                                                \
         e00,e01,e02,e03,                                               \
         e10,e11,e12,e13,                                               \
         e20,e21,e22,e23,                                               \
         e30,e31,e32,e33                                                \
    );                                                                  \
}

/** Copy-construct a matrix from a fixed-size array of values. */
#define CML_MAT_COPY_FROM_FIXED_ARRAY(_R_,_C_)                          \
matrix(const value_type m[_R_][_C_]) {                                  \
    typedef et::OpAssign<Element,Element> OpT;                          \
    cml::matrix<const value_type, external<_R_,_C_>,                    \
        basis_orient, row_major> src(&m[0][0]);                         \
    et::UnrollAssignment<OpT>(*this,src);                               \
}

/** Copy-construct a matrix from a runtime-sized array of values. */
#define CML_MAT_COPY_FROM_ARRAY(_add_)                                  \
matrix(const value_type* const v, size_t R, size_t C) _add_ {           \
    typedef et::OpAssign<Element,Element> OpT;                          \
    cml::matrix<value_type, external<>, basis_orient,                   \
        row_major > src(const_cast<value_type*>(v),R,C);                \
    et::UnrollAssignment<OpT>(*this,src);                               \
}

/** Copy this matrix from another using the given elementwise op.
 *
 * @internal This is required for GCC4, since it won't elide the default
 * copy constructor.
 */
#define CML_MAT_COPY_FROM_MATTYPE                                       \
matrix(const matrix_type& m) : array_type() {                           \
    typedef et::OpAssign <Element,Element> OpT;                         \
    et::UnrollAssignment<OpT>(*this,m);                                 \
}

/** Copy this matrix from another using the given elementwise op.
 *
 * This allows copies from arbitrary matrix types.
 */
#define CML_MAT_COPY_FROM_MAT                                           \
template<typename E, class AT, typename BO, typename L>                 \
matrix(const TEMPLATED_MATRIX_MACRO& m) {                               \
    typedef et::OpAssign <Element,E> OpT;                               \
    et::UnrollAssignment<OpT>(*this,m);                                 \
}

/** Declare a function to copy this matrix from a matrix expression. */
#define CML_MAT_COPY_FROM_MATXPR                                        \
template<class XprT>                                                    \
matrix(MATXPR_ARG_TYPE e) {                                             \
    /* Verify that a promotion exists at compile time: */               \
    typedef typename et::MatrixPromote<                                 \
        matrix_type, typename XprT::result_type>::type result_type;     \
    typedef typename XprT::value_type src_value_type;                   \
    typedef et::OpAssign <Element,src_value_type> OpT;                  \
    et::UnrollAssignment<OpT>(*this,e);                                 \
}

#if defined(CML_USE_GENERATED_MATRIX_ASSIGN_OP)
#define CML_MAT_ASSIGN_FROM_MATTYPE
#else
/** Assign from the same matrix type.
 *
 * @param m the matrix to copy from.
 *
 * @note This is required for GCC4, otherwise it generates a slow
 * assignment operator using memcpy.
 *
 * @note ICC9/Linux-x86 seems to prefer its own assignment operator (need
 * to figure out why).
 */
#define CML_MAT_ASSIGN_FROM_MATTYPE                                     \
matrix_type& operator=(const matrix_type& m) {                          \
    typedef et::OpAssign<Element,Element> OpT;                          \
    et::UnrollAssignment<OpT>(*this,m);                                 \
    return *this;                                                       \
}
#endif


/** Assign this matrix from another using the given elementwise op.
 *
 * This allows assignment from arbitrary matrix types.
 *
 * @param _op_ the operator (e.g. +=)
 * @param _op_name_ the op functor (e.g. et::OpAssign)
 */
#define CML_MAT_ASSIGN_FROM_MAT(_op_, _op_name_)                        \
template<typename E, class AT, typename BO, typename L> matrix_type&    \
operator _op_ (const TEMPLATED_MATRIX_MACRO& m) {                       \
    typedef _op_name_ <Element,E> OpT;                                  \
    et::UnrollAssignment<OpT>(*this,m);                                 \
    return *this;                                                       \
}

/** Declare a function to assign this matrix from a matrix expression.
 *
 * @param _op_ the operator (e.g. +=)
 * @param _op_name_ the op functor (e.g. et::OpAssign)
 */
#define CML_MAT_ASSIGN_FROM_MATXPR(_op_, _op_name_)                     \
template<class XprT> matrix_type&                                       \
operator _op_ (MATXPR_ARG_TYPE e) {                                     \
    /* Verify that a promotion exists at compile time: */               \
    typedef typename et::MatrixPromote<                                 \
        matrix_type, typename XprT::result_type>::type result_type;     \
    typedef typename XprT::value_type src_value_type;                   \
    typedef _op_name_ <Element,src_value_type> OpT;                     \
    et::UnrollAssignment<OpT>(*this,e);                                 \
    return *this;                                                       \
}

/** Declare a function to assign this matrix from a scalar.
 *
 * @param _op_ the operator (e.g. +=)
 * @param _op_name_ the op functor (e.g. et::OpAssign)
 *
 * @internal This shouldn't be used for ops, like +=, which aren't
 * defined in vector algebra.
 */
#define CML_MAT_ASSIGN_FROM_SCALAR(_op_, _op_name_)                     \
matrix_type& operator _op_ (ELEMENT_ARG_TYPE s) {                       \
    typedef _op_name_ <Element,value_type> OpT;                         \
    et::UnrollAssignment<OpT>(*this,s);                                 \
    return *this;                                                       \
}


/** Accumulated matrix multiplication.
 *
 * @throws std::invalid_argument if the matrices are not square.
 */
#define CML_ACCUMULATED_MATRIX_MULT(_arg_type_)                         \
matrix_type& operator*=(_arg_type_ m) {                                 \
    typedef typename et::MatrixPromote<                                 \
        matrix_type, _arg_type_>::type result_type;                     \
    cml::et::CheckedSquare(*this, typename result_type::size_tag());    \
    return (*this = (*this)*m);                                         \
}


/* These should only be used for testing: */
#define CML_MATRIX_BRACE_OPERATORS                                      \
template<class Matrix> struct row_ref {                                 \
    typedef typename Matrix::reference reference;                       \
    reference operator[](size_t col) { return m(row,col); }             \
    Matrix& m;                                                          \
    size_t row;                                                         \
};                                                                      \
                                                                        \
template<class Matrix> struct const_row_ref {                           \
    typedef typename Matrix::const_reference const_reference;           \
    const_reference operator[](size_t col) const { return m(row,col); } \
    const Matrix& m;                                                    \
    size_t row;                                                         \
};                                                                      \
                                                                        \
row_ref<matrix_type> operator[](size_t row) {                           \
    row_ref<matrix_type> ref = { *this, row }; return ref;              \
}                                                                       \
                                                                        \
const_row_ref<matrix_type> operator[](size_t row) const {               \
    const_row_ref<matrix_type> ref = { *this, row }; return ref;        \
}

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
