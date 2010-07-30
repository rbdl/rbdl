/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Defines the various combinations of vector expressions.
 *
 * Create unary and binary operators with macros.  The available combinations
 * are:
 *
 * Unary expressions:
 *
 * op Vector -> Vector
 * op VecXpr -> VecXpr
 *
 * Binary expressions:
 *
 * Vector op Vector -> Vector
 * VecXpr op Vector -> VecXpr
 * Vector op VecXpr -> VecXpr
 * VecXpr op VecXpr -> VecXpr
 *
 * Vector op Scalar -> Vector
 * Scalar op Vector -> Vector
 * VecXpr op Scalar -> VecXpr
 * Scalar op VecXpr -> VecXpr
 *
 * All of the generator functions compress the expression tree by hoisting
 * subexpressions into the containing expression.  This has the effect of
 * forcing only the root node of the expression tree to be a VectorXpr.
 * Every other node is a Unary or BinaryVectorOp.
 *
 * @todo Should ScalarT in expressions be passed by reference or by value?
 */

#ifndef vecop_macros_h
#define vecop_macros_h

/** Declare a unary operator taking a vector operand. */
#define CML_VEC_UNIOP(_op_, _OpT_)                                      \
template<typename E, class AT>                                          \
inline et::VectorXpr<                                                   \
    et::UnaryVectorOp< vector<E,AT>, _OpT_ <E> >                        \
>                                                                       \
                                                                        \
_op_ (const vector<E,AT>& arg)                                          \
{                                                                       \
    typedef et::UnaryVectorOp<                                          \
            vector<E,AT>, _OpT_ <E>                                     \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(arg));                            \
}


/** Declare a unary operator taking a et::VectorXpr operand. */
#define CML_VECXPR_UNIOP(_op_, _OpT_)                                   \
template<class XprT>                                                    \
inline et::VectorXpr<                                                   \
    et::UnaryVectorOp< XprT, _OpT_ <typename XprT::value_type> >        \
>                                                                       \
                                                                        \
_op_ (VECXPR_ARG_TYPE arg)                                              \
{                                                                       \
    typedef et::UnaryVectorOp<                                          \
        XprT, _OpT_ <typename XprT::value_type>                         \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(arg.expression()));               \
}


/** Declare an operator taking two vector operands. */
#define CML_VEC_VEC_BINOP(_op_, _OpT_)                                  \
template<typename E1, class AT1, typename E2, class AT2>                \
inline et::VectorXpr<                                                   \
    et::BinaryVectorOp<                                                 \
        vector<E1,AT1>, vector<E2,AT2>, _OpT_ <E1,E2>                   \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        const vector<E1,AT1>& left,                                     \
        const vector<E2,AT2>& right)                                    \
{                                                                       \
    typedef et::BinaryVectorOp<                                         \
            vector<E1,AT1>, vector<E2,AT2>, _OpT_ <E1,E2>               \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(left,right));                     \
}


/** Declare an operator taking a vector and a et::VectorXpr. */
#define CML_VEC_VECXPR_BINOP(_op_, _OpT_)                               \
template<typename E, class AT, class XprT>                              \
inline et::VectorXpr<                                                   \
    et::BinaryVectorOp<                                                 \
        vector<E,AT>, XprT, _OpT_ <E, typename XprT::value_type>        \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        const vector<E,AT>& left,                                       \
        VECXPR_ARG_TYPE right)                                          \
{                                                                       \
    typedef et::BinaryVectorOp<                                         \
            vector<E,AT>, XprT,                                         \
            _OpT_ <E, typename XprT::value_type>                        \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(left,right.expression()));        \
}


/** Declare an operator taking an et::VectorXpr and a vector. */
#define CML_VECXPR_VEC_BINOP(_op_, _OpT_)                               \
template<class XprT, typename E, class AT>                              \
inline et::VectorXpr<                                                   \
    et::BinaryVectorOp<                                                 \
        XprT, vector<E,AT>, _OpT_ <typename XprT::value_type, E>        \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        VECXPR_ARG_TYPE left,                                           \
        const vector<E,AT>& right)                                      \
{                                                                       \
    typedef et::BinaryVectorOp<                                         \
            XprT, vector<E,AT>,                                         \
            _OpT_ <typename XprT::value_type, E>                        \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(left.expression(),right));        \
}


/** Declare an operator taking two et::VectorXpr operands. */
#define CML_VECXPR_VECXPR_BINOP(_op_, _OpT_)                            \
template<class XprT1, class XprT2>                                      \
inline et::VectorXpr<                                                   \
    et::BinaryVectorOp<                                                 \
        XprT1, XprT2,                                                   \
        _OpT_ <                                                         \
            typename XprT1::value_type,                                 \
            typename XprT2::value_type                                  \
        >                                                               \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        VECXPR_ARG_TYPE_N(1) left,                                      \
        VECXPR_ARG_TYPE_N(2) right)                                     \
{                                                                       \
    typedef et::BinaryVectorOp<                                         \
            XprT1, XprT2,                                               \
            _OpT_ <                                                     \
                typename XprT1::value_type,                             \
                typename XprT2::value_type>                             \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(                                        \
            ExprT(left.expression(),right.expression()));               \
}


/** Declare an operator taking a vector and a scalar. */
#define CML_VEC_SCALAR_BINOP(_op_, _OpT_)                               \
template<typename E, class AT, typename ScalarT>                        \
inline et::VectorXpr<                                                   \
    et::BinaryVectorOp<                                                 \
        vector<E,AT>, ScalarT, _OpT_ <E,ScalarT>                        \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        const vector<E,AT>& left,                                       \
        SCALAR_ARG_TYPE right)                                          \
{                                                                       \
    typedef et::BinaryVectorOp<                                         \
            vector<E,AT>, ScalarT, _OpT_ <E,ScalarT>                    \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(left,right));                     \
}


/** Declare an operator taking a scalar and a vector. */
#define CML_SCALAR_VEC_BINOP(_op_, _OpT_)                               \
template<typename ScalarT, typename E, class AT>                        \
inline et::VectorXpr<                                                   \
    et::BinaryVectorOp<                                                 \
        ScalarT, vector<E,AT>, _OpT_ <ScalarT,E>                        \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        SCALAR_ARG_TYPE left,                                           \
        const vector<E,AT>& right)                                      \
{                                                                       \
    typedef et::BinaryVectorOp<                                         \
            ScalarT, vector<E,AT>, _OpT_ <ScalarT,E>                    \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(left,right));                     \
}


/** Declare an operator taking a et::VectorXpr and a scalar. */
#define CML_VECXPR_SCALAR_BINOP(_op_, _OpT_)                            \
template<class XprT, typename ScalarT>                                  \
inline et::VectorXpr<                                                   \
    et::BinaryVectorOp<                                                 \
        XprT, ScalarT, _OpT_ <typename XprT::value_type,ScalarT>        \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        VECXPR_ARG_TYPE left,                                           \
        SCALAR_ARG_TYPE right)                                          \
{                                                                       \
    typedef et::BinaryVectorOp<                                         \
            XprT, ScalarT, _OpT_ <typename XprT::value_type,ScalarT>    \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(left.expression(),right));        \
}


/** Declare an operator taking a scalar and a et::VectorXpr. */
#define CML_SCALAR_VECXPR_BINOP(_op_, _OpT_)                            \
template<typename ScalarT, class XprT>                                  \
inline et::VectorXpr<                                                   \
    et::BinaryVectorOp<                                                 \
        ScalarT, XprT, _OpT_ <ScalarT, typename XprT::value_type>       \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        SCALAR_ARG_TYPE left,                                           \
        VECXPR_ARG_TYPE right)                                          \
{                                                                       \
    typedef et::BinaryVectorOp<                                         \
            ScalarT, XprT,                                              \
            _OpT_ <ScalarT, typename XprT::value_type>                  \
        > ExprT;                                                        \
    return et::VectorXpr<ExprT>(ExprT(left,right.expression()));        \
}

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
