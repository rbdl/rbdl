/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef ops_h
#define ops_h

#include <cml/et/traits.h>
#include <cml/et/scalar_promotions.h>

/** Declare a unary scalar operator, like negation. */
#define CML_UNARY_SCALAR_OP(_op_, _op_name_)                            \
template<typename ArgT> struct _op_name_ {                              \
    typedef ExprTraits<ArgT> arg_traits;                                \
    typedef typename arg_traits::const_reference arg_reference;         \
    typedef typename arg_traits::value_type value_type;                 \
    typedef scalar_result_tag result_tag;                               \
    value_type apply(arg_reference arg) const { return _op_ arg; }      \
};

/** Declare a binary scalar operator, like addition, s1+s2. */
#define CML_BINARY_SCALAR_OP(_op_, _op_name_)                            \
template<typename LeftT, typename RightT> struct _op_name_ {             \
    typedef ExprTraits<LeftT> left_traits;                               \
    typedef ExprTraits<RightT> right_traits;                             \
    typedef typename left_traits::const_reference left_reference;        \
    typedef typename right_traits::const_reference right_reference;      \
    typedef typename left_traits::value_type left_value;                 \
    typedef typename right_traits::value_type right_value;               \
    typedef typename ScalarPromote<left_value,right_value>::type value_type; \
    typedef scalar_result_tag result_tag;                               \
    value_type apply(left_reference left, right_reference right) const { \
        return left _op_ right; }                                        \
};

/** Declare an op-assignment operator.
 *
 * @note The ExprTraits for both argument types must be defined, LeftT must
 * have an assignment operator, and ExprTraits<LeftT>::reference must specify
 * a type that allows assignment.
 */
#define CML_BINARY_SCALAR_OP_ASSIGN(_op_, _op_name_)                     \
template<typename LeftT, typename RightT> struct _op_name_ {             \
    typedef ExprTraits<LeftT> left_traits;                               \
    typedef ExprTraits<RightT> right_traits;                             \
    typedef typename left_traits::reference left_reference;              \
    typedef typename right_traits::const_reference right_reference;      \
    typedef typename left_traits::value_type left_value;                 \
    typedef typename right_traits::value_type right_value;               \
    typedef typename ScalarPromote<left_value,right_value>::type value_type; \
    typedef scalar_result_tag result_tag;                                \
    value_type apply(left_reference left, right_reference right) const { \
        return left _op_ (LeftT) right; }                                \
};

/** Declare a binary boolean operator, like less-than, s1 < s2.
 *
 * The operator should return the appropriate truth value for the operator.
 *
 * @note Both scalar types must have operator<() defined.
 */
#define CML_BOOLEAN_SCALAR_OP(_op_, _op_name_)                           \
template<typename LeftT, typename RightT> struct _op_name_ {             \
    typedef ExprTraits<LeftT> left_traits;                               \
    typedef ExprTraits<RightT> right_traits;                             \
    typedef typename left_traits::const_reference left_reference;        \
    typedef typename right_traits::const_reference right_reference;      \
    typedef scalar_result_tag result_tag;                                \
    bool apply(left_reference left, right_reference right) const {       \
        return left _op_ right; }                                        \
};

namespace cml {
namespace et {

/* Define the operators: */

/* Unary scalar ops: */
CML_UNARY_SCALAR_OP(-, OpNeg)
CML_UNARY_SCALAR_OP(+, OpPos)

/* Binary scalar ops: */
CML_BINARY_SCALAR_OP(+, OpAdd)
CML_BINARY_SCALAR_OP(-, OpSub)
CML_BINARY_SCALAR_OP(*, OpMul)

#if defined(CML_RECIPROCAL_OPTIMIZATION)
/* XXX Yikes... this should really be written out in full. *= 1./ is the
 * "_op_" parameter to the macro (see above):
 */
CML_BINARY_SCALAR_OP(* value_type(1)/, OpDiv)
#else
CML_BINARY_SCALAR_OP(/, OpDiv)
#endif

/* Binary scalar op-assigns: */
CML_BINARY_SCALAR_OP_ASSIGN( =, OpAssign)
CML_BINARY_SCALAR_OP_ASSIGN(+=, OpAddAssign)
CML_BINARY_SCALAR_OP_ASSIGN(-=, OpSubAssign)
CML_BINARY_SCALAR_OP_ASSIGN(*=, OpMulAssign)

#if defined(CML_RECIPROCAL_OPTIMIZATION)
/* XXX Yikes... this should really be written out in full. *= 1./ is the
 * "_op_" parameter to the macro (see above):
 */
CML_BINARY_SCALAR_OP_ASSIGN(*= value_type(1)/, OpDivAssign)
#else
CML_BINARY_SCALAR_OP_ASSIGN(/=, OpDivAssign)
#endif

/* Boolean operators for scalars: */
CML_BOOLEAN_SCALAR_OP(==, OpEqual)
CML_BOOLEAN_SCALAR_OP(!=, OpNotEqual)
CML_BOOLEAN_SCALAR_OP( <, OpLess)
CML_BOOLEAN_SCALAR_OP( >, OpGreater)
CML_BOOLEAN_SCALAR_OP(<=, OpLessEqual)
CML_BOOLEAN_SCALAR_OP(>=, OpGreaterEqual)

#undef CML_UNARY_SCALAR_OP
#undef CML_BINARY_SCALAR_OP
#undef CML_BINARY_SCALAR_OP_ASSIGN
#undef CML_BOOLEAN_SCALAR_OP

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
