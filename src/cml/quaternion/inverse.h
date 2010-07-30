/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Defines an operator for quaternion inverse.
 */

#ifndef quaternion_inverse_h
#define quaternion_inverse_h

#include <cml/quaternion/quaternion_expr.h>
#include <cml/quaternion/quaternion_functions.h>

namespace cml {
namespace et {

/** An expression node for inverting a quaternion.
 *
 * This internally creates a ConjugateOp node to process the conjugate
 * of the given expression.  The values produced by the ConjugateOp are then
 * divided by the Cayley norm of the expression on the fly.
 */
template<class ExprT>
class QuaternionInverseOp
{
  public:

    typedef QuaternionInverseOp<ExprT> expr_type;

    /* Record ary-ness of the expression: */
    typedef unary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    /* The subexpression is a ConjugateOp: */
    typedef et::ConjugateOp<ExprT> subexpression_type;
    typedef ExprTraits<subexpression_type> expr_traits;

    /* Get traits for the ExprT: */
    typedef ExprTraits<ExprT> arg_traits;
    typedef typename arg_traits::const_reference arg_reference;

    typedef typename subexpression_type::value_type value_type;
    typedef quaternion_result_tag result_tag;
    typedef typename subexpression_type::size_tag size_tag;

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
        return m_expr.real()/m_norm;
    }

    /** Return the vector part of the expression.
     *
     * @todo This could be returned as a VectorXpr also.
     */
    imaginary_type imaginary() const {
        return m_expr.imaginary()/m_norm;
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

    /** Compute inverse result at index i.
     *
     * The inverse of a quaternion p is ~p/norm(p).
     */
    value_type operator[](size_t i) const {
        return m_expr[i]/m_norm;
    }


  public:

    /** Return size of this expression (same as argument's size). */
    size_t size() const {
        return m_expr.size();
    }

    /** Return reference to contained expression. */
    expr_reference expression() const { return m_expr; }


  public:

    /** Construct from an input expression. */
    explicit QuaternionInverseOp(arg_reference arg)
        //: m_expr(arg), m_norm(cml::norm(arg)) {}
        : m_expr(arg), m_norm(arg.norm()) {}

    /** Copy constructor. */
    QuaternionInverseOp(const expr_type& e)
        : m_expr(e.m_expr), m_norm(e.m_norm) {}


  protected:

    subexpression_type m_expr;
    value_type m_norm;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for QuaternionInverseOp<>. */
template<class ExprT>
struct ExprTraits< QuaternionInverseOp<ExprT> >
{
    typedef QuaternionInverseOp<ExprT> expr_type;
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

/** Inverse of a quaternion. */
template<typename E, class AT, class OrderT, class CrossT> inline
et::QuaternionXpr< et::QuaternionInverseOp< quaternion<E,AT,OrderT,CrossT> > >
inverse(const quaternion<E,AT,OrderT,CrossT>& arg)
{
    typedef et::QuaternionInverseOp< quaternion<E,AT,OrderT,CrossT> > ExprT;
    return et::QuaternionXpr<ExprT>(ExprT(arg));
}

/** Inverse of a QuaternionXpr. */
template<class XprT> inline
et::QuaternionXpr< et::QuaternionInverseOp<XprT> >
inverse(QUATXPR_ARG_TYPE arg)
{
    typedef et::QuaternionInverseOp<XprT> ExprT;
    return et::QuaternionXpr<ExprT>(ExprT(arg.expression()));
}

/* NOTE: Quaternion division no longer supported, but I'm leaving the
   code here for reference (Jesse) */

#if 0
/** Declare div taking two quaternion operands. */
template<typename E1, class AT1, typename E2, class AT2, class OT, class CT>
inline typename et::QuaternionPromote<
    quaternion<E1,AT1,OT,CT>, quaternion<E2,AT2,OT,CT>
>::temporary_type
operator/(
        const quaternion<E1,AT1,OT,CT>& left,
        const quaternion<E2,AT2,OT,CT>& right)
{
    return left*inverse(right);
}

/** Declare div taking a quaternion and a et::QuaternionXpr. */
template<typename E, class AT, class OT, class CT, class XprT>
inline typename et::QuaternionPromote<
    quaternion<E,AT,OT,CT>, typename XprT::result_type
>::temporary_type
operator/(
        const quaternion<E,AT,OT,CT>& left,
        QUATXPR_ARG_TYPE right)
{
    return left*inverse(right);
}

/** Declare div taking an et::QuaternionXpr and a quaternion. */
template<class XprT, typename E, class AT, class OT, class CT>
inline typename et::QuaternionPromote<
    typename XprT::result_type, quaternion<E,AT,OT,CT>
>::temporary_type
operator/(
        QUATXPR_ARG_TYPE left,
        const quaternion<E,AT,OT,CT>& right)
{
    return left*inverse(right);
}

/** Declare div taking two et::QuaternionXpr operands. */
template<class XprT1, class XprT2>
inline typename et::QuaternionPromote<
    typename XprT1::result_type, typename XprT2::result_type
>::temporary_type
operator/(
        QUATXPR_ARG_TYPE_N(1) left,
        QUATXPR_ARG_TYPE_N(2) right)
{
    return left*inverse(right);
}
#endif

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
