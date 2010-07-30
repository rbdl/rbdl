/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Matrix linear expression classes.
 *
 * @todo Dynamic resizing needs to be integrated more naturally into
 * mul() and matrix transpose():
 */

#ifndef matrix_expr_h
#define matrix_expr_h


#include <cml/et/size_checking.h>
#include <cml/matrix/matrix_traits.h>
#include <cml/matrix/matrix_promotions.h>

/* XXX Don't know which it should be just yet, since RVO seems to obviate the
 * need for a reference type.  However, copy by value copies the *entire
 * expression tree rooted at the MatrixXpr<>, so this choice is bound to affect
 * performance for some compiler or another:
 */
#define MATXPR_ARG_TYPE               const et::MatrixXpr<XprT>&
#define MATXPR_ARG_TYPE_N(_N_)        const et::MatrixXpr<XprT##_N_>&

//#define MATXPR_ARG_TYPE               const et::MatrixXpr<XprT>
//#define MATXPR_ARG_TYPE_N(_N_)        const et::MatrixXpr<XprT##_N_>

namespace cml {
namespace et {

/** A placeholder for a matrix expression in the expression tree. */
template<class ExprT>
class MatrixXpr
{
  public:

    typedef MatrixXpr<ExprT> expr_type;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename ExprT::value_type value_type;
    typedef matrix_result_tag result_tag;
    typedef typename ExprT::size_tag size_tag;  // Just inherit size type.

    /* Store the expression traits: */
    typedef ExprTraits<ExprT> expr_traits;

    /* Get the reference type: */
    typedef typename expr_traits::const_reference expr_reference;

    /* Get the result type: */
    typedef typename expr_traits::result_type result_type;
    
    /* Get the basis type: */
    typedef typename result_type::basis_orient basis_orient;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;

    /* For matching by assignability: */
    typedef cml::et::not_assignable_tag assignable_tag;


  public:

    /** Record result size as an enum (if applicable). */
    enum { array_rows = ExprT::array_rows, array_cols = ExprT::array_cols };


  public:

    /** Return the expression size as a pair. */
    matrix_size size() const {
        return matrix_size(this->rows(),this->cols());
    }

    /** Return number of rows in the expression (same as subexpression). */
    size_t rows() const { 
        return expr_traits().rows(m_expr);
    }

    /** Return number of columns in the expression (same as subexpression). */
    size_t cols() const {
        return expr_traits().cols(m_expr);
    }

    /** Return reference to contained expression. */
    expr_reference expression() const { return m_expr; }

    /** Compute value at index i,j of the result matrix. */
    value_type operator()(size_t i, size_t j) const {
        return expr_traits().get(m_expr,i,j);
    }
    
    /** Return element j of basis vector i. */
    value_type basis_element(size_t i, size_t j) const {
        return basis_element(i,j,basis_orient());
    }


  public:

    /** Construct from the subexpression to store. */
    explicit MatrixXpr(expr_reference expr) : m_expr(expr) {}

    /** Copy constructor. */
    MatrixXpr(const expr_type& e) : m_expr(e.m_expr) {}


  protected:

    value_type basis_element(size_t i, size_t j, row_basis) const {
        return (*this)(i,j);
    }

    value_type basis_element(size_t i, size_t j, col_basis) const {
        return (*this)(j,i);
    }


  protected:

    expr_reference m_expr;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits for MatrixXpr<>. */
template<class ExprT>
struct ExprTraits< MatrixXpr<ExprT> >
{
    typedef MatrixXpr<ExprT> expr_type;
    typedef ExprT arg_type;

    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef typename expr_type::assignable_tag assignable_tag;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& e, size_t i, size_t j) const {
        return e(i,j);
    }


    matrix_size size(const expr_type& e) const { return e.size(); }
    size_t rows(const expr_type& e) const { return e.rows(); }
    size_t cols(const expr_type& e) const { return e.cols(); }
};


/** A unary matrix expression operating on matrix elements as a list.
 *
 * The operator must take exactly one argument.
 */
template<class ExprT, class OpT>
class UnaryMatrixOp
{
  public:

    typedef UnaryMatrixOp<ExprT,OpT> expr_type;

    /* Record ary-ness of the expression: */
    typedef unary_expression expr_ary;

    /* Copy the expression by value into higher-up expressions: */
    typedef expr_type expr_const_reference;

    typedef typename OpT::value_type value_type;
    typedef matrix_result_tag result_tag;
    typedef typename ExprT::size_tag size_tag;

    /* Store the expression traits for the subexpression: */
    typedef ExprTraits<ExprT> expr_traits;

    /* Reference type for the subexpression: */
    typedef typename expr_traits::const_reference expr_reference;

    /* Get the result type: */
    typedef typename expr_traits::result_type result_type;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;

    /* For matching by assignability: */
    typedef cml::et::not_assignable_tag assignable_tag;


  public:

    /** Record result size as an enum (if applicable). */
    enum { array_rows = ExprT::array_rows, array_cols = ExprT::array_cols };


  public:

    /** Return the expression size as a pair. */
    matrix_size size() const {
        return matrix_size(this->rows(),this->cols());
    }

    /** Return number of rows in the expression (same as argument). */
    size_t rows() const {
        return expr_traits().rows(m_expr);
    }

    /** Return number of columns in the expression (same as argument). */
    size_t cols() const {
        return expr_traits().cols(m_expr);
    }

    /** Compute value at index i,j of the result matrix. */
    value_type operator()(size_t i, size_t j) const {

        /* This uses the expression traits to figure out how to access the
         * i,j'th element of the subexpression:
         */
        return OpT().apply(expr_traits().get(m_expr,i,j));
    }


  public:

    /** Construct from the subexpression. */
    explicit UnaryMatrixOp(expr_reference expr) : m_expr(expr) {}

    /** Copy constructor. */
    UnaryMatrixOp(const expr_type& e) : m_expr(e.m_expr) {}


  protected:

    expr_reference m_expr;


  private:

    /* Cannot be assigned to: */
    expr_type& operator=(const expr_type&);
};

/** Expression traits for UnaryMatrixOp<>. */
template<class ExprT, class OpT>
struct ExprTraits< UnaryMatrixOp<ExprT,OpT> >
{
    typedef UnaryMatrixOp<ExprT,OpT> expr_type;
    typedef ExprT arg_type;

    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef typename expr_type::assignable_tag assignable_tag;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& e, size_t i, size_t j) const {
        return e(i,j);
    }

    matrix_size size(const expr_type& e) const { return e.size(); }
    size_t rows(const expr_type& e) const { return e.rows(); }
    size_t cols(const expr_type& e) const { return e.cols(); }
};


/** A binary matrix expression. */
template<class LeftT, class RightT, class OpT>
class BinaryMatrixOp
{
  public:

    typedef BinaryMatrixOp<LeftT,RightT,OpT> expr_type;

    /* Copy the UnaryMatrixOp expression by value into parent
     * expression tree nodes:
     */
    typedef expr_type expr_const_reference;

    typedef typename OpT::value_type value_type;
    typedef matrix_result_tag result_tag;

    /* For matching by assignability: */
    typedef cml::et::not_assignable_tag assignable_tag;

    /* Record the expression traits for the two subexpressions: */
    typedef ExprTraits<LeftT> left_traits;
    typedef ExprTraits<RightT> right_traits;

    /* Reference types for the two subexpressions: */
    typedef typename left_traits::const_reference left_reference;
    typedef typename right_traits::const_reference right_reference;

    /* Figure out the expression's resulting (matrix) type: */
    typedef typename left_traits::result_type left_result;
    typedef typename right_traits::result_type right_result;
    typedef typename MatrixPromote<left_result,right_result>::type result_type;
    typedef typename result_type::size_tag size_tag;

    /* Get the temporary type: */
    typedef typename result_type::temporary_type temporary_type;

    /* Define a size checker: */
    typedef GetCheckedSize<LeftT,RightT,size_tag> checked_size;


  public:

    /** Record result size as an enum (if applicable).
     *
     * CheckExprSizes<> ensures that this works as expected.
     */
    enum {
        array_rows = result_type::array_rows,
        array_cols = result_type::array_cols
    };


  public:

    /** Return the expression size as a pair. */
    matrix_size size() const {
        return CheckedSize(m_left,m_right,size_tag());
    }

    /** Return number of rows in the result.
     *
     * @note Because this calls size() internally, calling both rows()
     * and cols() with CML_CHECK_MATRIX_EXPR_SIZES defined will cause the size
     * checking code to be executed twice.
     */
    size_t rows() const {
#if defined(CML_CHECK_MATRIX_EXPR_SIZES)
        return this->size().first;
#else
        return left_traits().rows(m_left);
#endif
    }

    /** Return number of cols in the result.
     *
     * @note Because this calls size() internally, calling both rows()
     * and cols() with CML_CHECK_MATRIX_EXPR_SIZES defined will cause the size
     * checking code to be executed twice.
     */
    size_t cols() const {
#if defined(CML_CHECK_MATRIX_EXPR_SIZES)
        return this->size().second;
#else
        return right_traits().cols(m_right);
#endif
    }

    /** Compute value at index i,j of the result matrix. */
    value_type operator()(size_t i, size_t j) const {

        /* This uses the expression traits to figure out how to access the
         * i'th index of the two subexpressions:
         */
        return OpT().apply(
                left_traits().get(m_left,i,j),
                right_traits().get(m_right,i,j));
    }


  public:

    /** Construct from the two subexpressions.
     *
     * @throws std::invalid_argument if the subexpression sizes don't
     * match.
     */
    explicit BinaryMatrixOp(left_reference left, right_reference right)
        : m_left(left), m_right(right) {}

    /** Copy constructor. */
    BinaryMatrixOp(const expr_type& e)
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

/** Expression traits for BinaryMatrixOp<>. */
template<class LeftT, class RightT, class OpT>
struct ExprTraits< BinaryMatrixOp<LeftT,RightT,OpT> >
{
    typedef BinaryMatrixOp<LeftT,RightT,OpT> expr_type;
    typedef LeftT left_type;
    typedef RightT right_type;

    typedef typename expr_type::value_type value_type;
    typedef typename expr_type::expr_const_reference const_reference;
    typedef typename expr_type::result_tag result_tag;
    typedef typename expr_type::size_tag size_tag;
    typedef typename expr_type::result_type result_type;
    typedef typename expr_type::assignable_tag assignable_tag;
    typedef expr_node_tag node_tag;

    value_type get(const expr_type& e, size_t i, size_t j) const {
        return e(i,j);
    }

    matrix_size size(const expr_type& e) const { return e.size(); }
    size_t rows(const expr_type& e) const { return e.rows(); }
    size_t cols(const expr_type& e) const { return e.cols(); }
};

/* Helper struct to verify that both arguments are matrix expressions: */
template<typename LeftTraits, typename RightTraits>
struct MatrixExpressions
{
    /* Require that both arguments are matrix expressions: */
    typedef typename LeftTraits::result_tag left_result;
    typedef typename RightTraits::result_tag right_result;
    enum { is_true = (same_type<left_result,et::matrix_result_tag>::is_true
            && same_type<right_result,et::matrix_result_tag>::is_true) };
};

namespace detail {

/* XXX These are temporary helpers until dynamic resizing is integrated more
 * naturally into mul() and matrix transpose():
 */
template<typename MatT, typename MT> inline
void Resize(MatT&, size_t, size_t, fixed_size_tag, MT) {}

template<typename MatT> inline
void Resize(MatT& m,
        size_t R, size_t C, dynamic_size_tag, dynamic_memory_tag)
{
    m.resize(R,C);
}

template<typename MatT> inline
void Resize(MatT& m, size_t R, size_t C) {
    Resize(m, R, C, typename MatT::size_tag(), typename MatT::memory_tag());
}

template<typename MatT> inline
void Resize(MatT& m, matrix_size N) {
    Resize(m, N.first, N.second,
            typename MatT::size_tag(), typename MatT::memory_tag());
}

} // namespace detail

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
