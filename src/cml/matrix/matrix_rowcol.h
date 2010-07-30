/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Expressions to extract a row or column of a matrix.
 */

#ifndef matrix_rowcol_h
#define matrix_rowcol_h

#include <cml/vector/vector_expr.h>
#include <cml/matrix/matrix_expr.h>

namespace cml {
namespace et {

template<class ExprT>
class MatrixRowOp
{
  public:

    typedef MatrixRowOp<ExprT> expr_type;

    /* Record ary-ness of the expression: */
    typedef unary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename ExprT::value_type value_type;
    typedef vector_result_tag result_tag;
    typedef typename ExprT::size_tag size_tag;

    /* Store the expression traits: */
    typedef ExprTraits<ExprT> expr_traits;

    /* Get the reference type: */
    typedef typename expr_traits::const_reference expr_reference;

    /* Get the result vector type: */
    typedef typename expr_traits::result_type::row_vector_type result_type;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;


  public:

    /** Record result size as an enum. */
    enum { array_size = result_type::array_size };


  public:

    /** Return the expression size as a pair. */
    matrix_size size() const {
        return expr_traits().rows(m_expr);
    }

    /** Return reference to contained expression. */
    expr_reference expression() const { return m_expr; }

    /** Compute value at index i of the row vector. */
    value_type operator[](size_t i) const {
        return expr_traits().get(m_expr,m_row,i);
    }


  public:

    /** Construct from the subexpression to store. */
    explicit MatrixRowOp(const ExprT& expr, size_t row)
        : m_expr(expr), m_row(row) {}

    /** Copy constructor. */
    MatrixRowOp(const expr_type& e)
        : m_expr(e.m_expr), m_row(e.m_row) {}


  protected:

    expr_reference m_expr;
    const size_t m_row;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for MatrixRowOp<>. */
template<class ExprT>
struct ExprTraits< MatrixRowOp<ExprT> >
{
    typedef MatrixRowOp<ExprT> expr_type;
    typedef ExprT arg_type;

    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& v, size_t i) const { return v[i]; }
    size_t size(const expr_type& e) const { return e.size(); }
};

template<class ExprT>
class MatrixColOp
{
  public:

    typedef MatrixColOp<ExprT> expr_type;

    /* Record ary-ness of the expression: */
    typedef unary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename ExprT::value_type value_type;
    typedef vector_result_tag result_tag;
    typedef typename ExprT::size_tag size_tag;

    /* Store the expression traits: */
    typedef ExprTraits<ExprT> expr_traits;

    /* Get the reference type: */
    typedef typename expr_traits::const_reference expr_reference;

    /* Get the result vector type: */
    typedef typename expr_traits::result_type::col_vector_type result_type;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;


  public:

    /** Record result size as an enum. */
    enum { array_size = result_type::array_size };


  public:

    /** Return the expression size as a pair. */
    matrix_size size() const {
        return expr_traits().cols(m_expr);
    }

    /** Return reference to contained expression. */
    expr_reference expression() const { return m_expr; }

    /** Compute value at index i of the col vector. */
    value_type operator[](size_t i) const {
        return expr_traits().get(m_expr,i,m_col);
    }


  public:

    /** Construct from the subexpression to store. */
    explicit MatrixColOp(const ExprT& expr, size_t col)
        : m_expr(expr), m_col(col) {}

    /** Copy constructor. */
    MatrixColOp(const expr_type& e)
        : m_expr(e.m_expr), m_col(e.m_col) {}


  protected:

    expr_reference m_expr;
    const size_t m_col;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits class for MatrixColOp<>. */
template<class ExprT>
struct ExprTraits< MatrixColOp<ExprT> >
{
    typedef MatrixColOp<ExprT> expr_type;
    typedef ExprT arg_type;

    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& v, size_t i) const { return v[i]; }
    size_t size(const expr_type& e) const { return e.size(); }
};

} // namespace et

/* Define the row and column operators in the cml namespace: */

/** Matrix row operator taking a matrix operand. */
template<typename E, class AT, typename BO, typename L>
et::VectorXpr< et::MatrixRowOp< matrix<E,AT,BO,L> > >
row(const matrix<E,AT,BO,L>& expr, size_t i)
{
    typedef et::MatrixRowOp< matrix<E,AT,BO,L> > ExprT;
    return et::VectorXpr<ExprT>(ExprT(expr,i));
}

/** Matrix row operator taking an et::MatrixXpr operand.
 *
 * The parse tree is automatically compressed by hoisting the MatrixXpr's
 * subexpression into the subexpression of the MatrixRowOp.
 */
template<class XprT>
et::VectorXpr< et::MatrixRowOp<XprT> >
row(const et::MatrixXpr<XprT>& expr, size_t i)
{
    typedef et::MatrixRowOp<XprT> ExprT;
    return et::MatrixXpr<ExprT>(ExprT(expr.expression(),i));
}

/** Matrix col operator taking a matrix operand. */
template<typename E, class AT, typename BO, typename L>
et::VectorXpr< et::MatrixColOp< matrix<E,AT,BO,L> > >
col(const matrix<E,AT,BO,L>& expr, size_t i)
{
    typedef et::MatrixColOp< matrix<E,AT,BO,L> > ExprT;
    return et::VectorXpr<ExprT>(ExprT(expr,i));
}

/** Matrix col operator taking an et::MatrixXpr operand.
 *
 * The parse tree is automatically compressed by hoisting the MatrixXpr's
 * subexpression into the subexpression of the MatrixColOp.
 */
template<class XprT>
et::VectorXpr< et::MatrixColOp<XprT> >
col(const et::MatrixXpr<XprT>& expr, size_t i)
{
    typedef et::MatrixColOp<XprT> ExprT;
    return et::VectorXpr<ExprT>(ExprT(expr.expression(),i));
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
