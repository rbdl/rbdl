/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Vector linear expression classes.
 */

#ifndef vector_expr_h
#define vector_expr_h

#include <cmath>
#include <cml/et/size_checking.h>
#include <cml/vector/vector_traits.h>
#include <cml/vector/vector_promotions.h>

/* XXX Don't know which it should be just yet, since RVO seems to obviate need
 * for a reference type.  However, copy by value copies the *entire expression
 * tree rooted at the VectorXpr<>, so this choice is bound to affect
 * performace for some compiler or another:
 */
#define VECXPR_ARG_TYPE  const et::VectorXpr<XprT>&
#define VECXPR_ARG_TYPE_N(_N_)  const et::VectorXpr<XprT##_N_>&

//#define VECXPR_ARG_TYPE         const et::VectorXpr<XprT>
//#define VECXPR_ARG_TYPE_N(_N_)  const et::VectorXpr<XprT##_N_>

namespace cml {
namespace et {

/** A placeholder for a vector expression in an expression tree. */
template<class ExprT>
class VectorXpr
{
  public:

    typedef VectorXpr<ExprT> expr_type;

    /* Record ary-ness of the expression: */
    typedef typename ExprT::expr_ary expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename ExprT::value_type value_type;
    typedef vector_result_tag result_tag;
    typedef typename ExprT::size_tag size_tag;

    /* Store the expression traits: */
    typedef ExprTraits<ExprT> expr_traits;

    /* Get the reference type: */
    typedef typename expr_traits::const_reference expr_reference;

    /* Get the result type: */
    typedef typename expr_traits::result_type result_type;

    /* For matching by assignability: */
    typedef cml::et::not_assignable_tag assignable_tag;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;


  public:

    /** Record result size as an enum. */
    enum { array_size = ExprT::array_size };


  public:

    /** Return square of the length. */
    value_type length_squared() const {
        return m_expr.length_squared();
    }

    /** Return the length. */
    value_type length() const {
        return m_expr.length();
    }

    /** Return the result as a normalized vector. */
    result_type normalize() const {
        return m_expr.normalize();
    }

    /** Compute value at index i of the result vector. */
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
    explicit VectorXpr(expr_reference expr) : m_expr(expr) {}

    /** Copy constructor. */
    VectorXpr(const expr_type& e) : m_expr(e.m_expr) {}


  protected:

    expr_reference m_expr;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for VectorXpr<>. */
template<class ExprT>
struct ExprTraits< VectorXpr<ExprT> >
{
    typedef VectorXpr<ExprT> expr_type;
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


/** A unary vector expression.
 *
 * The operator's operator() method must take exactly one argument.
 */
template<class ExprT, class OpT>
class UnaryVectorOp
{
  public:

    typedef UnaryVectorOp<ExprT,OpT> expr_type;

    /* Record ary-ness of the expression: */
    typedef unary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename OpT::value_type value_type;
    typedef vector_result_tag result_tag;
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


  public:

    /** Record result size as an enum. */
    enum { array_size = ExprT::array_size };


  public:

    /** Return square of the length. */
    value_type length_squared() const {
        return dot(
                VectorXpr<expr_type>(*this),
                VectorXpr<expr_type>(*this));
    }

    /** Return the length. */
    value_type length() const {
        return std::sqrt(length_squared());
    }

    /** Return the result as a normalized vector. */
    result_type normalize() const {
        result_type v(VectorXpr<expr_type>(*this));
        return v.normalize();
    }

    /** Compute value at index i of the result vector. */
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
    explicit UnaryVectorOp(expr_reference expr) : m_expr(expr) {}

    /** Copy constructor. */
    UnaryVectorOp(const expr_type& e) : m_expr(e.m_expr) {}


  protected:

    expr_reference m_expr;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for UnaryVectorOp<>. */
template<class ExprT, class OpT>
struct ExprTraits< UnaryVectorOp<ExprT,OpT> >
{
    typedef UnaryVectorOp<ExprT,OpT> expr_type;
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


/** A binary vector expression.
 *
 * The operator's operator() method must take exactly two arguments.
 */
template<class LeftT, class RightT, class OpT>
class BinaryVectorOp
{
  public:

    typedef BinaryVectorOp<LeftT,RightT,OpT> expr_type;

    /* Record ary-ness of the expression: */
    typedef binary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename OpT::value_type value_type;
    typedef vector_result_tag result_tag;

    /* Store the expression traits types for the two subexpressions: */
    typedef ExprTraits<LeftT> left_traits;
    typedef ExprTraits<RightT> right_traits;

    /* Reference types for the two subexpressions: */
    typedef typename left_traits::const_reference left_reference;
    typedef typename right_traits::const_reference right_reference;

    /* Figure out the expression's resulting (vector) type: */
    typedef typename left_traits::result_type left_result;
    typedef typename right_traits::result_type right_result;
    typedef typename VectorPromote<left_result,right_result>::type result_type;
    typedef typename result_type::size_tag size_tag;

    /* For matching by assignability: */
    typedef cml::et::not_assignable_tag assignable_tag;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;

    /* Define a size checker: */
    typedef GetCheckedSize<LeftT,RightT,size_tag> checked_size;


  public:

    /** Record result size as an enum (if applicable). */
    enum { array_size = result_type::array_size };


  public:

    /** Return square of the length. */
    value_type length_squared() const {
        return dot(
                VectorXpr<expr_type>(*this),
                VectorXpr<expr_type>(*this));
    }

    /** Return the length. */
    value_type length() const {
        return std::sqrt(length_squared());
    }

    /** Return the result as a normalized vector. */
    result_type normalize() const {
        result_type v(VectorXpr<expr_type>(*this));
        return v.normalize();
    }

    /** Compute value at index i of the result vector. */
    value_type operator[](size_t i) const {

        /* This uses the expression traits to figure out how to access the
         * i'th index of the two subexpressions:
         */
        return OpT().apply(
                left_traits().get(m_left,i),
                right_traits().get(m_right,i));
    }


  public:

    /** Return the size of the vector result.
     *
     * @throws std::invalid_argument if the expressions do not have the same
     * size.
     */
    size_t size() const {
        /* Note: This actually does a check only if
         * CML_CHECK_VECTOR_EXPR_SIZES is set:
         */
        return CheckedSize(m_left,m_right,size_tag());
    }

    /** Return reference to left expression. */
    left_reference left_expression() const { return m_left; }

    /** Return reference to right expression. */
    right_reference right_expression() const { return m_right; }


  public:

    /** Construct from the two subexpressions. */
    explicit BinaryVectorOp(left_reference left, right_reference right)
        : m_left(left), m_right(right) {}

    /** Copy constructor. */
    BinaryVectorOp(const expr_type& e)
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

/** Expression traits class for BinaryVectorOp<>. */
template<class LeftT, class RightT, class OpT>
struct ExprTraits< BinaryVectorOp<LeftT,RightT,OpT> >
{
    typedef BinaryVectorOp<LeftT,RightT,OpT> expr_type;
    typedef LeftT left_type;
    typedef RightT right_type;

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

/* Helper struct to verify that both arguments are vector expressions: */
template<typename LeftTraits, typename RightTraits>
struct VectorExpressions
{
    /* Require that both arguments are vector expressions: */
    typedef typename LeftTraits::result_tag left_result;
    typedef typename RightTraits::result_tag right_result;
    enum { is_true = (same_type<left_result,et::vector_result_tag>::is_true
            && same_type<right_result,et::vector_result_tag>::is_true) };
};

namespace detail {

template<typename VecT, typename RT, typename MT> inline
void Resize(VecT&,size_t,RT,MT) {}

template<typename VecT> inline
void Resize(VecT& v, size_t S, resizable_tag, dynamic_memory_tag) {
    v.resize(S);
}

template<typename VecT> inline
void Resize(VecT& v, size_t S) {
    Resize(v, S, typename VecT::resizing_tag(), typename VecT::memory_tag());
}

} // namespace detail

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
