/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Defines an operator for quaternion conjugation.
 */

#ifndef conjugate_h
#define conjugate_h

#include <cml/quaternion/quaternion_expr.h>

namespace cml {
namespace et {

/** An expression node for conjugating a quaternion. */
template<class ExprT>
class ConjugateOp
{
  public:

    typedef ConjugateOp<ExprT> expr_type;

    /* Record ary-ness of the expression: */
    typedef unary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename ExprT::value_type value_type;
    typedef quaternion_result_tag result_tag;
    typedef typename ExprT::size_tag size_tag;

    /* Store the expression traits for the subexpression: */
    typedef ExprTraits<ExprT> expr_traits;

    /* Reference type for the subexpression: */
    typedef typename expr_traits::const_reference expr_reference;

    /* Get the result type (same as for subexpression): */
    typedef typename expr_traits::result_type result_type;

    /* For matching by assignability: */
    typedef cml::et::not_assignable_tag assignable_tag;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;

    /* Get the vector type: */
    typedef typename result_type::vector_type vector_type;

    /* Get the imaginary part type: */
    typedef typename vector_type::subvector_type imaginary_type;

    /* Record the order type: */
    typedef typename result_type::order_type order_type;


  public:

    /** Record result size as an enum. */
    enum { array_size = ExprT::array_size };

    /** Localize the ordering as an enum. */
    enum {
        W = order_type::W,
        X = order_type::X,
        Y = order_type::Y,
        Z = order_type::Z
    };


  public:

    /** Return the real part of the expression. */
    value_type real() const {
        return m_expr.real();
    }

    /** Return the vector part of the expression. */
    imaginary_type imaginary() const {
        return -m_expr.imaginary();
    }

    /** Return the Cayley norm of the expression. */
    value_type norm() const {
        return length_squared();
    }

    /** Return square of the quaternion length. */
    value_type length_squared() const {
        return dot(
                QuaternionXpr<expr_type>(*this),
                QuaternionXpr<expr_type>(*this));
    }

    /** Return the quaternion length. */
    value_type length() const {
        return std::sqrt(length_squared());
    }

    /** Return the result as a normalized quaternion. */
    temporary_type normalize() const {
        temporary_type q(QuaternionXpr<expr_type>(*this));
        return q.normalize();
    }

    /** Compute conjugated result at index i.
     *
     * The conjugate of quaternion s + v is s - v.
     */
    value_type operator[](size_t i) const {
        return (i == W) ? m_expr[W] : - m_expr[i] ;
    }


  public:

    /** Return size of this expression (same as argument's size). */
    size_t size() const {
        return m_expr.size();
    }

    /** Return reference to contained expression. */
    expr_reference expression() const { return m_expr; }


  public:

    /** Construct from the subexpression. */
    explicit ConjugateOp(expr_reference expr) : m_expr(expr) {}

    /** Copy constructor. */
    ConjugateOp(const expr_type& e) : m_expr(e.m_expr) {}


  protected:

    expr_reference m_expr;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for ConjugateOp<>. */
template<class ExprT>
struct ExprTraits< ConjugateOp<ExprT> >
{
    typedef ConjugateOp<ExprT> expr_type;
    typedef ExprT arg_type;

    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef typename expr_type::assignable_tag assignable_tag;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& v, size_t i) const { return v[i]; }
    size_t size(const expr_type& e) const { return e.size(); }
};

} // namespace et

/** Conjugation of a quaternion. */
template<typename E, class AT, class OT, class CT> inline
et::QuaternionXpr< et::ConjugateOp< quaternion<E,AT,OT,CT> > >
conjugate(const quaternion<E,AT,OT,CT>& arg)
{
    typedef et::ConjugateOp< quaternion<E,AT,OT,CT> > ExprT;
    return et::QuaternionXpr<ExprT>(ExprT(arg));
}

/** Conjugation of a QuaternionXpr. */
template<class XprT> inline
et::QuaternionXpr< et::ConjugateOp<XprT> >
conjugate(QUATXPR_ARG_TYPE arg)
{
    typedef et::ConjugateOp<XprT> ExprT;
    return et::QuaternionXpr<ExprT>(ExprT(arg.expression()));
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
