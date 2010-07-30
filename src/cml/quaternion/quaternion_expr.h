/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef quaternion_expr_h
#define quaternion_expr_h

#include <cml/et/size_checking.h>
#include <cml/mathlib/epsilon.h>
#include <cml/quaternion/quaternion_traits.h>
#include <cml/quaternion/quaternion_promotions.h>
#include <cml/util.h>

#define QUATXPR_ARG_TYPE  const et::QuaternionXpr<XprT>&
#define QUATXPR_ARG_TYPE_N(_N_)  const et::QuaternionXpr<XprT##_N_>&

namespace cml {
namespace et {

/** A placeholder for a quaternion expression in an expression tree. */
template<class ExprT>
class QuaternionXpr
{
  public:

    typedef QuaternionXpr<ExprT> expr_type;

    /* Record ary-ness of the expression: */
    typedef typename ExprT::expr_ary expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename ExprT::value_type value_type;
    typedef quaternion_result_tag result_tag;
    typedef typename ExprT::size_tag size_tag;

    /* Store the expression traits: */
    typedef ExprTraits<ExprT> expr_traits;

    /* Get the reference type: */
    typedef typename expr_traits::const_reference expr_reference;

    /* Get the result type: */
    typedef typename expr_traits::result_type result_type;

    /* Get the vector type: */
    typedef typename result_type::vector_type vector_type;

    /* Get the imaginary part type: */
    typedef typename vector_type::subvector_type imaginary_type;

    /* For matching by assignability: */
    typedef cml::et::not_assignable_tag assignable_tag;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;

    /* Record the order type: */
    typedef typename result_type::order_type order_type;
    
    /* Record the cross type: */
    typedef typename result_type::cross_type cross_type;


  public:

    /** Record result size as an enum. */
    enum { array_size = ExprT::array_size };


  public:

    /** Return the real part of the expression. */
    value_type real() const {
        return m_expr.real();
    }

    /** Return the vector part of the expression. */
    imaginary_type imaginary() const {
        return m_expr.imaginary();
    }

    /** Return the Cayley norm of the expression. */
    value_type norm() const {
        return m_expr.length_squared();
    }

    /** Return square of the quaternion length. */
    value_type length_squared() const {
        return m_expr.length_squared();
    }

    /** Return the quaternion length. */
    value_type length() const {
        return m_expr.length();
    }

    /** Return the result as a normalized quaternion. */
    temporary_type normalize() const {
        return m_expr.normalize();
    }

    /** Return the log of the expression. */
    temporary_type log(
        value_type tolerance = epsilon<value_type>::placeholder()) const
    {
        return m_expr.log(tolerance);
    }

    /** 
     * Return the result of the exponential function as applied to
     * this expression.
     */
    temporary_type exp(
        value_type tolerance = epsilon<value_type>::placeholder()) const
    {
        return m_expr.exp(tolerance);
    }

    /** Compute value at index i of the result quaternion. */
    value_type operator[](size_t i) const {
        return m_expr[i];
    }


  public:

    /** Return size of this expression (same as subexpression's size). */
    size_t size() const {
        return m_expr.size();
    }

    /** Return reference to contained expression. */
    expr_reference expression() const { return m_expr; }


  public:

    /** Construct from the subexpression to store. */
    explicit QuaternionXpr(expr_reference expr) : m_expr(expr) {}

    /** Copy constructor. */
    QuaternionXpr(const expr_type& e) : m_expr(e.m_expr) {}


  protected:

    expr_reference m_expr;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for QuaternionXpr<>. */
template<class ExprT>
struct ExprTraits< QuaternionXpr<ExprT> >
{
    typedef QuaternionXpr<ExprT> expr_type;
    typedef ExprT arg_type;
    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef typename expr_type::assignable_tag not_assignable_tag;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& v, size_t i) const { return v[i]; }
    size_t size(const expr_type& e) const { return e.size(); }
};


/** A unary quaternion expression.
 *
 * The operator's operator() method must take exactly one argument.
 */
template<class ExprT, class OpT>
class UnaryQuaternionOp
{
  public:

    typedef UnaryQuaternionOp<ExprT,OpT> expr_type;

    /* Record ary-ness of the expression: */
    typedef unary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename OpT::value_type value_type;
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
        return (*this)[W];
    }

    /** Return the vector part of the expression. */
    imaginary_type imaginary() const {
        imaginary_type v;
        v[0] = (*this)[X]; v[1] = (*this)[Y]; v[2] = (*this)[Z];
        return v;
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

    /** Return the log of this expression. */
    temporary_type log(
        value_type tolerance = epsilon<value_type>::placeholder()) const
    {
        value_type a = acos_safe(real());
        value_type s = std::sin(a);
        
        if (s > tolerance) {
            return temporary_type(value_type(0), imaginary() * (a / s));
        } else {
            return temporary_type(value_type(0), imaginary());
        }
    }
    
    /** 
     * Return the result of the exponential function as applied to
     * this expression.
     */
    temporary_type exp(
        value_type tolerance = epsilon<value_type>::placeholder()) const
    {
        imaginary_type v = imaginary();
        value_type a = cml::length(v);
        
        if (a > tolerance) {
            return temporary_type(std::cos(a), v * (std::sin(a) / a));
        } else {
            return temporary_type(std::cos(a), v);
        }
    }

    /** Compute value at index i of the result quaternion. */
    value_type operator[](size_t i) const {

        /* This uses the expression traits to figure out how to access the
         * i'th index of the subexpression:
         */
        return OpT().apply(expr_traits().get(m_expr,i));
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
    explicit UnaryQuaternionOp(expr_reference expr) : m_expr(expr) {}

    /** Copy constructor. */
    UnaryQuaternionOp(const expr_type& e) : m_expr(e.m_expr) {}


  protected:

    expr_reference m_expr;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for UnaryQuaternionOp<>. */
template<class ExprT, class OpT>
struct ExprTraits< UnaryQuaternionOp<ExprT,OpT> >
{
    typedef UnaryQuaternionOp<ExprT,OpT> expr_type;
    typedef ExprT arg_type;

    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef typename expr_type::assignable_tag not_assignable_tag;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& v, size_t i) const { return v[i]; }
    size_t size(const expr_type& e) const { return e.size(); }
};


/** A binary quaternion expression.
 *
 * The operator's operator() method must take exactly two arguments.
 */
template<class LeftT, class RightT, class OpT>
class BinaryQuaternionOp
{
  public:

    typedef BinaryQuaternionOp<LeftT,RightT,OpT> expr_type;

    /* Record ary-ness of the expression: */
    typedef binary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename OpT::value_type value_type;
    typedef quaternion_result_tag result_tag;

    /* Store the expression traits types for the two subexpressions: */
    typedef ExprTraits<LeftT> left_traits;
    typedef ExprTraits<RightT> right_traits;

    /* Reference types for the two subexpressions: */
    typedef typename left_traits::const_reference left_reference;
    typedef typename right_traits::const_reference right_reference;

    /* Figure out the expression's resulting (quaternion) type: */
    typedef typename left_traits::result_type left_result;
    typedef typename right_traits::result_type right_result;
    typedef typename QuaternionPromote<left_result,right_result>::type
        result_type;
    typedef typename result_type::size_tag size_tag;

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

    /* Define a size checker: */
    typedef GetCheckedSize<LeftT,RightT,size_tag> checked_size;


  public:

    /** Record result size as an enum. */
    enum { array_size = 4 };

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
        return (*this)[W];
    }

    /** Return the vector part of the expression. */
    imaginary_type imaginary() const {
        imaginary_type v;
        v[0] = (*this)[X]; v[1] = (*this)[Y]; v[2] = (*this)[Z];
        return v;
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

    /** Return the log of this expression. */
    temporary_type log(
        value_type tolerance = epsilon<value_type>::placeholder()) const
    {
        value_type a = acos_safe(real());
        value_type s = std::sin(a);
        
        if (s > tolerance) {
            return temporary_type(value_type(0), imaginary() * (a / s));
        } else {
            return temporary_type(value_type(0), imaginary());
        }
    }
    
    /** 
     * Return the result of the exponential function as applied to
     * this expression.
     */
    temporary_type exp(
        value_type tolerance = epsilon<value_type>::placeholder()) const
    {
        imaginary_type v = imaginary();
        value_type a = cml::length(v);
        
        if (a > tolerance) {
            return temporary_type(std::cos(a), v * (std::sin(a) / a));
        } else {
            return temporary_type(std::cos(a), v);
        }
    }

    /** Compute value at index i of the result quaternion. */
    value_type operator[](size_t i) const {

        /* This uses the expression traits to figure out how to access the
         * i'th index of the two subexpressions:
         */
        return OpT().apply(
                left_traits().get(m_left,i),
                right_traits().get(m_right,i));
    }


  public:

    /** Return the size of the quaternion result.
     *
     * @throws std::invalid_argument if the expressions do not have the same
     * size.
     */
    size_t size() const {
        /* Note: This actually does a check only if
         * CML_CHECK_VECTOR_EXPR_SIZES is set:
         */
        CheckedSize(m_left,m_right,size_tag());

        /* The size is always 4: */
        return 4;
    }

    /** Return reference to left expression. */
    left_reference left_expression() const { return m_left; }

    /** Return reference to right expression. */
    right_reference right_expression() const { return m_right; }


  public:

    /** Construct from the two subexpressions. */
    explicit BinaryQuaternionOp(left_reference left, right_reference right)
        : m_left(left), m_right(right) {}

    /** Copy constructor. */
    BinaryQuaternionOp(const expr_type& e)
        : m_left(e.m_left), m_right(e.m_right) {}


  protected:

    left_reference m_left;
    right_reference m_right;


  private:

    /* This ensures that a compile-time size check is executed: */
    typename checked_size::check_type _dummy;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for BinaryQuaternionOp<>. */
template<class LeftT, class RightT, class OpT>
struct ExprTraits< BinaryQuaternionOp<LeftT,RightT,OpT> >
{
    typedef BinaryQuaternionOp<LeftT,RightT,OpT> expr_type;
    typedef LeftT left_type;
    typedef RightT right_type;

    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef typename expr_type::imaginary_type imaginary_type;
    typedef typename expr_type::assignable_tag not_assignable_tag;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& v, size_t i) const { return v[i]; }
    size_t size(const expr_type& e) const { return e.size(); }
};


/* Helper struct to verify that both arguments are quaternion expressions: */
template<class LeftTraits, class RightTraits>
struct QuaternionExpressions
{
    /* Require that both arguments are quaternion expressions: */
    typedef typename LeftTraits::result_tag left_result;
    typedef typename RightTraits::result_tag right_result;
    enum { is_true = (same_type<left_result,et::quaternion_result_tag>::is_true
            && same_type<right_result,et::quaternion_result_tag>::is_true) };
};

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
