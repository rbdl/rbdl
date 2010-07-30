/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * Create unary and binary operators with macros.
 *
 * These macros work just like those in cml/quaternion/vecop_macros.h.
 */

#ifndef quatop_macros_h
#define quatop_macros_h

/** Declare a unary operator taking a quaternion operand. */
#define CML_QUAT_UNIOP(_op_, _OpT_)                                     \
template<typename E, class AT, class OT, class CT>                      \
inline et::QuaternionXpr<                                               \
    et::UnaryQuaternionOp< quaternion<E,AT,OT,CT>, _OpT_ <E> >          \
>                                                                       \
                                                                        \
_op_ (const quaternion<E,AT,OT,CT>& arg)                                \
{                                                                       \
    typedef et::UnaryQuaternionOp<                                      \
            quaternion<E,AT,OT,CT>, _OpT_ <E>                           \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(arg));                        \
}


/** Declare a unary operator taking a et::QuaternionXpr operand. */
#define CML_QUATXPR_UNIOP(_op_, _OpT_)                                  \
template<class XprT>                                                    \
inline et::QuaternionXpr<                                               \
    et::UnaryQuaternionOp< XprT, _OpT_ <typename XprT::value_type> >    \
>                                                                       \
                                                                        \
_op_ (QUATXPR_ARG_TYPE arg)                                             \
{                                                                       \
    typedef et::UnaryQuaternionOp<                                      \
        XprT, _OpT_ <typename XprT::value_type>                         \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(arg.expression()));           \
}



/** Declare an operator taking two quaternion operands. */
#define CML_QUAT_QUAT_BINOP(_op_, _OpT_)                                \
template<typename E1, class AT1, typename E2, class AT2,                \
    class OT, class CT>                                                 \
inline et::QuaternionXpr<                                               \
    et::BinaryQuaternionOp<                                             \
        quaternion<E1,AT1,OT,CT>, quaternion<E2,AT2,OT,CT>,             \
        _OpT_ <E1,E2>                                                   \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        const quaternion<E1,AT1,OT,CT>& left,                           \
        const quaternion<E2,AT2,OT,CT>& right)                          \
{                                                                       \
    typedef et::BinaryQuaternionOp<                                     \
            quaternion<E1,AT1,OT,CT>, quaternion<E2,AT2,OT,CT>,         \
            _OpT_ <E1,E2>                                               \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(left,right));                 \
}


/** Declare an operator taking a quaternion and a et::QuaternionXpr. */
#define CML_QUAT_QUATXPR_BINOP(_op_, _OpT_)                             \
template<typename E, class AT, class OT, class CT, class XprT>          \
inline et::QuaternionXpr<                                               \
    et::BinaryQuaternionOp<                                             \
        quaternion<E,AT,OT,CT>, XprT,                                   \
        _OpT_ <E, typename XprT::value_type>                            \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        const quaternion<E,AT,OT,CT>& left,                             \
        QUATXPR_ARG_TYPE right)                                         \
{                                                                       \
    typedef et::BinaryQuaternionOp<                                     \
            quaternion<E,AT,OT,CT>, XprT,                               \
            _OpT_ <E, typename XprT::value_type>                        \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(left,right.expression()));    \
}


/** Declare an operator taking an et::QuaternionXpr and a quaternion. */
#define CML_QUATXPR_QUAT_BINOP(_op_, _OpT_)                             \
template<class XprT, typename E, class AT, class OT, class CT>          \
inline et::QuaternionXpr<                                               \
    et::BinaryQuaternionOp<                                             \
        XprT, quaternion<E,AT,OT,CT>,                                   \
        _OpT_ <typename XprT::value_type, E>                            \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        QUATXPR_ARG_TYPE left,                                          \
        const quaternion<E,AT,OT,CT>& right)                            \
{                                                                       \
    typedef et::BinaryQuaternionOp<                                     \
            XprT, quaternion<E,AT,OT,CT>,                               \
            _OpT_ <typename XprT::value_type, E>                        \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(left.expression(),right));    \
}


/** Declare an operator taking two et::QuaternionXpr operands. */
#define CML_QUATXPR_QUATXPR_BINOP(_op_, _OpT_)                          \
template<class XprT1, class XprT2>                                      \
inline et::QuaternionXpr<                                               \
    et::BinaryQuaternionOp<                                             \
        XprT1, XprT2,                                                   \
        _OpT_ <                                                         \
            typename XprT1::value_type,                                 \
            typename XprT2::value_type                                  \
        >                                                               \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        QUATXPR_ARG_TYPE_N(1) left,                                     \
        QUATXPR_ARG_TYPE_N(2) right)                                    \
{                                                                       \
    typedef et::BinaryQuaternionOp<                                     \
            XprT1, XprT2,                                               \
            _OpT_ <                                                     \
                typename XprT1::value_type,                             \
                typename XprT2::value_type>                             \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(                                    \
            ExprT(left.expression(),right.expression()));               \
}


/** Declare an operator taking a quaternion and a scalar. */
#define CML_QUAT_SCALAR_BINOP(_op_, _OpT_)                              \
template<typename E, class AT, class OT, class CT, typename ScalarT>    \
inline et::QuaternionXpr<                                               \
    et::BinaryQuaternionOp<                                             \
        quaternion<E,AT,OT,CT>, ScalarT,                                \
        _OpT_ <E,ScalarT>                                               \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        const quaternion<E,AT,OT,CT>& left,                             \
        SCALAR_ARG_TYPE right)                                          \
{                                                                       \
    typedef et::BinaryQuaternionOp<                                     \
            quaternion<E,AT,OT,CT>, ScalarT, _OpT_ <E,ScalarT>          \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(left,right));                 \
}


/** Declare an operator taking a scalar and a quaternion. */
#define CML_SCALAR_QUAT_BINOP(_op_, _OpT_)                              \
template<typename ScalarT, typename E, class AT, class OT, class CT>    \
inline et::QuaternionXpr<                                               \
    et::BinaryQuaternionOp<                                             \
        ScalarT, quaternion<E,AT,OT,CT>, _OpT_ <ScalarT,E>              \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        SCALAR_ARG_TYPE left,                                           \
        const quaternion<E,AT,OT,CT>& right)                            \
{                                                                       \
    typedef et::BinaryQuaternionOp<                                     \
            ScalarT, quaternion<E,AT,OT,CT>, _OpT_ <ScalarT,E>          \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(left,right));                 \
}


/** Declare an operator taking a et::QuaternionXpr and a scalar. */
#define CML_QUATXPR_SCALAR_BINOP(_op_, _OpT_)                           \
template<class XprT, typename ScalarT>                                  \
inline et::QuaternionXpr<                                               \
    et::BinaryQuaternionOp<                                             \
        XprT, ScalarT, _OpT_ <typename XprT::value_type,ScalarT>        \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        QUATXPR_ARG_TYPE left,                                          \
        SCALAR_ARG_TYPE right)                                          \
{                                                                       \
    typedef et::BinaryQuaternionOp<                                     \
            XprT, ScalarT, _OpT_ <typename XprT::value_type,ScalarT>    \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(left.expression(),right));    \
}


/** Declare an operator taking a scalar and a et::QuaternionXpr. */
#define CML_SCALAR_QUATXPR_BINOP(_op_, _OpT_)                           \
template<typename ScalarT, class XprT>                                  \
inline et::QuaternionXpr<                                               \
    et::BinaryQuaternionOp<                                             \
        ScalarT, XprT, _OpT_ <ScalarT, typename XprT::value_type>       \
    >                                                                   \
>                                                                       \
                                                                        \
_op_ (                                                                  \
        SCALAR_ARG_TYPE left,                                           \
        QUATXPR_ARG_TYPE right)                                         \
{                                                                       \
    typedef et::BinaryQuaternionOp<                                     \
            ScalarT, XprT,                                              \
            _OpT_ <ScalarT, typename XprT::value_type>                  \
        > ExprT;                                                        \
    return et::QuaternionXpr<ExprT>(ExprT(left,right.expression()));    \
}

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
