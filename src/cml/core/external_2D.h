/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * Defines the fixed-size and runtime-sized external 2D arrays.
 *
 * @todo Would casting get better performance in the external_2D<> element
 * access methods?
 */

#ifndef external_2D_h
#define external_2D_h

#include <cml/core/common.h>
#include <cml/core/fixed_1D.h>
#include <cml/core/fixed_2D.h>
#include <cml/core/dynamic_1D.h>
#include <cml/core/dynamic_2D.h>
#include <cml/external.h>

namespace cml {

/** Fixed-size external 2D array.
 *
 * Both the memory and the size are fixed at compile time, and cannot be
 * changed.
 */
template<typename Element, int Rows, int Cols, typename Layout>
class external_2D
{
  public:

    /* Require Rows > 0, Cols > 0: */
    CML_STATIC_REQUIRE((Rows > 0) && (Cols > 0));

    /* Record the generator: */
    typedef external<Rows,Cols> generator_type;

    /* Standard: */
    typedef Element value_type;
    typedef Element* pointer;
    typedef Element& reference;
    typedef const Element& const_reference;
    typedef const Element* const_pointer;

    /* For matching by memory layout: */
    typedef Layout layout;

    /* For matching by memory type: */
    typedef external_memory_tag memory_tag;

    /* For matching by size type: */
    typedef fixed_size_tag size_tag;

    /* For matching by resizability: */
    typedef not_resizable_tag resizing_tag;

    /* For matching by dimensions: */
    typedef twod_tag dimension_tag;

    /* To simplify the matrix transpose operator: */
    typedef fixed_2D<typename cml::remove_const<Element>::type,
            Cols,Rows,Layout> transposed_type;
    /* Note: the transposed type must be fixed_2D, since an external array
     * cannot be specified without a corresponding memory location.
     */

    /* To simplify the matrix row and column operators: */
    typedef fixed_1D<Element,Rows> row_array_type;
    typedef fixed_1D<Element,Cols> col_array_type;
    /* Note: the row types must be fixed_1D, since external arrays cannot be
     * specified without a memory location.
     */


  public:

    enum { array_rows = Rows, array_cols = Cols };


  public:

    /** Construct an external array from a pointer. */
    external_2D(value_type const ptr[Rows][Cols])
        : m_data(const_cast<pointer>(&ptr[0][0])) {}

    /** Construct an external array from a pointer. */
    external_2D(value_type* const ptr) : m_data(ptr) {}


  public:

    /** Return the number of rows in the array. */
    size_t rows() const { return size_t(array_rows); }

    /** Return the number of cols in the array. */
    size_t cols() const { return size_t(array_cols); }


  public:

    /** Access element (row,col) of the matrix.
     *
     * @param row row of element.
     * @param col column of element.
     * @returns mutable reference.
     *
     * @note This function does not range-check the arguments.
     */
    reference operator()(size_t row, size_t col) {
        /* Dispatch to the right function based on layout: */
        return get_element(row,col,layout());
    }

    /** Const access element (row,col) of the matrix.
     *
     * @param row row of element.
     * @param col column of element.
     * @returns const reference.
     *
     * @note This function does not range-check the arguments.
     */
    const_reference operator()(size_t row, size_t col) const {
        /* Dispatch to the right function based on layout: */
        return get_element(row,col,layout());
    }

    /** Return access to the data as a raw pointer. */
    pointer data() { return m_data; }

    /** Return access to the data as a raw pointer. */
    const_pointer data() const { return m_data; }


  protected:

    /* XXX May be able to cast to get better performance? */
    reference get_element(size_t row, size_t col, row_major) {
        return m_data[row*Cols + col];
    }

    const_reference get_element(size_t row, size_t col, row_major) const {
        return m_data[row*Cols + col];
    }

    reference get_element(size_t row, size_t col, col_major) {
        return m_data[col*Rows + row];
    }

    const_reference get_element(size_t row, size_t col, col_major) const {
        return m_data[col*Rows + row];
    }


  protected:

    /* Declare the data array: */
    pointer const               m_data;
};

/** Run-time sized external 2D array.
 *
 * Both the memory and the size are fixed at run-time, but cannot be changed.
 * This is a specialization for the case that Rows and Cols are not specified
 * (i.e. given as the default of -1,-1).
 */
template<typename Element, typename Layout>
class external_2D<Element,-1,-1,Layout>
{
  public:

    /* Record the generator.  Note: this is *not* unique, as it is the same
     * generator used by external_1D.  However, external_1D is used only by
     * vector<> classes, so this is not a problem.
     */
    typedef external<> generator_type;

    /* Standard: */
    typedef Element value_type;
    typedef Element* pointer;
    typedef Element& reference;
    typedef const Element& const_reference;
    typedef const Element* const_pointer;

    /* For matching by memory layout: */
    typedef Layout layout;

    /* For matching by memory type: */
    typedef external_memory_tag memory_tag;

    /* For matching by size type: */
    typedef dynamic_size_tag size_tag;

    /* For matching by resizability: */
    typedef not_resizable_tag resizing_tag;

    /* For matching by dimensions: */
    typedef twod_tag dimension_tag;

    /* To simplify the matrix transpose operator: */
    typedef dynamic_2D<typename cml::remove_const<Element>::type,
        Layout, CML_DEFAULT_ARRAY_ALLOC> transposed_type;

    /* To simplify the matrix row and column operators: */
    typedef dynamic_1D<Element, CML_DEFAULT_ARRAY_ALLOC> row_array_type;
    typedef dynamic_1D<Element, CML_DEFAULT_ARRAY_ALLOC> col_array_type;


  public:

    enum { array_rows = -1, array_cols = -1 };


  public:

    /** Construct an external array with no size. */
    external_2D(pointer const ptr, size_t rows, size_t cols)
        : m_data(ptr), m_rows(rows), m_cols(cols) {}


  public:

    /** Return the number of rows in the array. */
    size_t rows() const { return m_rows; }

    /** Return the number of cols in the array. */
    size_t cols() const { return m_cols; }


  public:

    /** Access element (row,col) of the matrix.
     *
     * @param row row of element.
     * @param col column of element.
     * @returns mutable reference.
     *
     * @note This function does not range-check the arguments.
     */
    reference operator()(size_t row, size_t col) {
        /* Dispatch to the right function based on layout: */
        return get_element(row,col,layout());
    }

    /** Const access element (row,col) of the matrix.
     *
     * @param row row of element.
     * @param col column of element.
     * @returns const reference.
     *
     * @note This function does not range-check the arguments.
     */
    const_reference operator()(size_t row, size_t col) const {
        /* Dispatch to the right function based on layout: */
        return get_element(row,col,layout());
    }

    /** Return access to the data as a raw pointer. */
    pointer data() { return m_data; }

    /** Return access to the data as a raw pointer. */
    const_pointer data() const { return m_data; }


  protected:

    /* XXX May be able to cast to get better performance? */
    reference get_element(size_t row, size_t col, row_major) {
        return m_data[row*m_cols + col];
    }

    const_reference get_element(size_t row, size_t col, row_major) const {
        return m_data[row*m_cols + col];
    }

    reference get_element(size_t row, size_t col, col_major) {
        return m_data[col*m_rows + row];
    }

    const_reference get_element(size_t row, size_t col, col_major) const {
        return m_data[col*m_rows + row];
    }


  protected:

    /* Declare the data array: */
    value_type* const           m_data;
    const size_t                m_rows;
    const size_t                m_cols;
};

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
