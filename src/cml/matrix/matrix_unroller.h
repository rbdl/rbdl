/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * @todo Need to implement unrolling for efficient col-major array access.
 *
 * @todo Does it make sense to unroll an assignment if either side of the
 * assignment has a fixed size, or just when the target matrix is fixed
 * size?
 */

#ifndef matrix_unroller_h
#define matrix_unroller_h

#include <cml/et/traits.h>
#include <cml/et/size_checking.h>
#include <cml/et/scalar_ops.h>

#if !defined(CML_2D_UNROLLER) && !defined(CML_NO_2D_UNROLLER)
#error "The matrix unroller has not been defined."
#endif

#if defined(CML_2D_UNROLLER) && !defined(CML_MATRIX_UNROLL_LIMIT)
#error "CML_MATRIX_UNROLL_LIMIT is undefined."
#endif

namespace cml {
namespace et {
namespace detail {

/** Unroll a binary assignment operator on a fixed-size matrix.
 *
 * This uses a forward iteration to make better use of the cache.
 *
 * @sa cml::matrix
 * @sa cml::et::OpAssign
 *
 * @bug Need to verify that OpT is actually an assignment operator.
 * @bug The 2D unroller needs to be specified for efficient col-major
 * access.
 */
template<class OpT, typename E, class AT, typename BO, typename L, class SrcT>
class MatrixAssignmentUnroller
{
  protected:

    /* The matrix type being assigned to: */
    typedef cml::matrix<E,AT,BO,L> matrix_type;

    /* Record traits for the arguments: */
    typedef ExprTraits<matrix_type> dest_traits;
    typedef ExprTraits<SrcT> src_traits;

#if defined(CML_2D_UNROLLER)

    /* Forward declare: */
    template<int R, int C, int LastRow, int LastCol, bool can_unroll>
        struct Eval;

    /* XXX This needs to be specified for efficient col-major access also! */

    /** Evaluate the binary operator at element R,C. */
    template<int R, int C, int LastRow, int LastCol>
        struct Eval<R,C,LastRow,LastCol,true> {
            void operator()(matrix_type& dest, const SrcT& src) const {

                /* Apply to current R,C: */
                OpT().apply(dest(R,C), src_traits().get(src,R,C));

                /* Evaluate at R,C+1: */
                Eval<R,C+1,LastRow,LastCol,true>()(dest,src);
            }
        };

    /** Evaluate the binary operator at element R,LastCol. */
    template<int R, int LastRow, int LastCol>
        struct Eval<R,LastCol,LastRow,LastCol,true> {
            void operator()(matrix_type& dest, const SrcT& src) const {

                /* Apply to R,LastCol: */
                OpT().apply(dest(R,LastCol), src_traits().get(src,R,LastCol));

                /* Evaluate at R+1,0; i.e. move to next row and start the
                 * col iteration from 0:
                 */
                Eval<R+1,0,LastRow,LastCol,true>()(dest,src);
            }
        };

    /** Evaluate the binary operator at element LastRow,C. */
    template<int C, int LastRow, int LastCol>
        struct Eval<LastRow,C,LastRow,LastCol,true> {
            void operator()(matrix_type& dest, const SrcT& src) const {

                /* Apply to LastRow,C: */
                OpT().apply(dest(LastRow,C), src_traits().get(src,LastRow,C));

                /* Evaluate at LastRow,C+1: */
                Eval<LastRow,C+1,LastRow,LastCol,true>()(dest,src);
            }
        };

    /** Evaluate the binary operator at element LastRow,LastCol. */
    template<int LastRow, int LastCol>
        struct Eval<LastRow,LastCol,LastRow,LastCol,true> {
            void operator()(matrix_type& dest, const SrcT& src) const {

                /* Apply to LastRow,LastCol: */
                OpT().apply(
                        dest(LastRow,LastCol),
                        src_traits().get(src,LastRow,LastCol));
            }
        };


    /** Evaluate operators on large matrices using a loop. */
    template<int R, int C, int LastRow, int LastCol>
        struct Eval<R,C,LastRow,LastCol,false> {
            void operator()(matrix_type& dest, const SrcT& src) const {
                for(size_t i = 0; i <= LastRow; ++i) {
                    for(size_t j = 0; j <= LastCol; ++j) {
                        OpT().apply(dest(i,j), src_traits().get(src,i,j));
                    }
                }
            }
        };

#endif // CML_2D_UNROLLER

#if defined(CML_NO_2D_UNROLLER)

    /** Evaluate the binary operator using a loop. */
    template<int R, int C, int LastRow, int LastCol> struct Eval {
        void operator()(matrix_type& dest, const SrcT& src) const {
            for(size_t i = 0; i <= LastRow; ++i) {
                for(size_t j = 0; j <= LastCol; ++j) {
                    OpT().apply(dest(i,j), src_traits().get(src,i,j));
                }
            }
        }
    };

#endif // CML_NO_2D_UNROLLER


  public:

    /** Unroll assignment for a fixed-sized matrix. */
    void operator()(
            cml::matrix<E,AT,BO,L>& dest, const SrcT& src, cml::fixed_size_tag)
    {
        typedef cml::matrix<E,AT,BO,L> matrix_type;
        enum {
            LastRow = matrix_type::array_rows-1,
            LastCol = matrix_type::array_cols-1,
            Max = (LastRow+1)*(LastCol+1)
        };

#if defined(CML_2D_UNROLLER)
        typedef typename MatrixAssignmentUnroller<OpT,E,AT,BO,L,SrcT>
            ::template Eval<0, 0, LastRow, LastCol,
            (Max <= CML_MATRIX_UNROLL_LIMIT)> Unroller;
#endif

#if defined(CML_NO_2D_UNROLLER)
        /* Use a loop: */
        typedef typename MatrixAssignmentUnroller<OpT,E,AT,BO,L,SrcT>
            ::template Eval<0, 0, LastRow, LastCol> Unroller;
#endif

        /* Use a run-time check if src is a run-time sized expression: */
        typedef typename ExprTraits<SrcT>::size_tag src_size;
        typedef typename select_if<
            same_type<src_size,dynamic_size_tag>::is_true,
            dynamic_size_tag, fixed_size_tag>::result size_tag;

        /* Check the expression size (the returned size isn't needed): */
        CheckedSize(dest,src,size_tag());
        /* Note: for two fixed-size expressions, the if-statements and
         * comparisons should be completely eliminated as dead code.  If
         * src is a dynamic-sized expression, the check will still happen.
         */

        Unroller()(dest,src);
    }


  private:
    /* XXX Blah, a temp. hack to fix the auto-resizing stuff below. */
    matrix_size hack_actual_size(
        matrix_type& dest, const SrcT& /*src*/, scalar_result_tag
        )
    {
        typedef ExprTraits<matrix_type> dest_traits;
        return dest_traits().size(dest);
    }

    matrix_size hack_actual_size(
        matrix_type& /*dest*/, const SrcT& src, matrix_result_tag
        )
    {
        typedef ExprTraits<SrcT> src_traits;
        return src_traits().size(src);
    }

    matrix_size CheckOrResize(
            matrix_type& dest, const SrcT& src, cml::resizable_tag)
    {
#if defined(CML_AUTOMATIC_MATRIX_RESIZE_ON_ASSIGNMENT)
        /* Get the size of src.  This also causes src to check its size: */
        matrix_size N = hack_actual_size(
            dest, src, typename src_traits::result_tag());

        /* Set the destination matrix's size: */
        dest.resize(N.first,N.second);
#else
        matrix_size N = CheckedSize(dest,src,dynamic_size_tag());
#endif
        return N;
    }

    matrix_size CheckOrResize(
            matrix_type& dest, const SrcT& src, cml::not_resizable_tag)
    {
        return CheckedSize(dest,src,dynamic_size_tag());
    }


  public:


    /** Use a loop for dynamic-sized matrix assignment.
     *
     * @note The target matrix must already have the correct size.
     *
     * @todo This needs to be specialized for efficient row-major or col-major
     * layout access.
     */
    void operator()(matrix_type& dest, const SrcT& src, cml::dynamic_size_tag)
    {
        typedef ExprTraits<SrcT> src_traits;
        matrix_size N = this->CheckOrResize(
                dest,src,typename matrix_type::resizing_tag());
        for(size_t i = 0; i < N.first; ++i) {
            for(size_t j = 0; j < N.second; ++j) {
                OpT().apply(dest(i,j), src_traits().get(src,i,j));
                /* Note: we don't need get(), since dest is a matrix. */
            }
        }
    }
};

}

/** This constructs an assignment unroller for fixed-size arrays.
 *
 * The operator must be an assignment op (otherwise, this doesn't make any
 * sense).  Also, automatic unrolling is only performed for fixed-size
 * matrices; a loop is used for dynamic-sized matrices.
 *
 * @sa cml::matrix
 * @sa cml::et::OpAssign
 *
 * @bug Need to verify that OpT is actually an assignment operator.
 */
template<class OpT, class SrcT, typename E, class AT, typename BO, typename L>
inline void UnrollAssignment(cml::matrix<E,AT,BO,L>& dest, const SrcT& src)
{
    /* Record the destination matrix type, and the expression traits: */
    typedef cml::matrix<E,AT,BO,L> matrix_type;

    /* Record the type of the unroller: */
    typedef detail::MatrixAssignmentUnroller<OpT,E,AT,BO,L,SrcT> unroller;

    /* Finally, do the unroll call: */
    unroller()(dest, src, typename matrix_type::size_tag());
    /* XXX It may make sense to unroll if either side is a fixed size. */
}

} // namespace et
} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
