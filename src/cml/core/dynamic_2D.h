/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef dynamic_2D_h
#define dynamic_2D_h

#include <memory>
#include <cml/core/common.h>
#include <cml/core/dynamic_1D.h>
#include <cml/dynamic.h>

namespace cml {

/** Dynamically-sized and allocated 2D array.
 *
 * @note The allocator should be an STL-compatible allocator.
 *
 * @internal The internal array type <em>must</em> have the proper copy
 * semantics, otherwise copy construction will fail.
 *
 * @internal This class does not need a destructor.
 */
template<typename Element, typename Layout, class Alloc>
class dynamic_2D
{
  public:

    /* Record the allocator type: */
    typedef typename Alloc::template rebind<Element>::other allocator_type;

    /* Record the generator: */
    typedef dynamic<Alloc> generator_type;

    /* Standard: */
    typedef typename allocator_type::value_type value_type;
    typedef typename allocator_type::pointer pointer; 
    typedef typename allocator_type::reference reference; 
    typedef typename allocator_type::const_reference const_reference; 
    typedef typename allocator_type::const_pointer const_pointer; 

    /* For matching by memory layout: */
    typedef Layout layout;

    /* For matching by memory type: */
    typedef dynamic_memory_tag memory_tag;

    /* For matching by size type: */
    typedef dynamic_size_tag size_tag;

    /* For matching by resizability: */
    typedef resizable_tag resizing_tag;

    /* For matching by dimensions: */
    typedef twod_tag dimension_tag;

    /* To simplify the matrix transpose operator: */
    typedef dynamic_2D<typename cml::remove_const<Element>::type,
            Layout,Alloc> transposed_type;

    /* To simplify the matrix row and column operators: */
    typedef dynamic_1D<Element,Alloc> row_array_type;
    typedef dynamic_1D<Element,Alloc> col_array_type;


  protected:

    /** Construct a dynamic array with no size. */
    dynamic_2D() : m_rows(0), m_cols(0), m_data(0), m_alloc() {}

    /** Construct a dynamic matrix given the dimensions. */
    explicit dynamic_2D(size_t rows, size_t cols) 
        : m_rows(0), m_cols(0), m_data(0), m_alloc()
       	{
	  this->resize(rows, cols);
	}

    /** Copy construct a dynamic matrix. */
    dynamic_2D(const dynamic_2D& other)
        : m_rows(0), m_cols(0), m_data(0), m_alloc()
       	{
	  this->copy(other);
	}

    ~dynamic_2D() {
      this->destroy();
    }


  public:

    enum { array_rows = -1, array_cols = -1 };


  public:

    /** Return the number of rows in the array. */
    size_t rows() const { return m_rows; }

    /** Return the number of cols in the array. */
    size_t cols() const { return m_cols; }


  public:

    /** Access the given element of the matrix.
     *
     * @param row row of element.
     * @param col column of element.
     * @returns mutable reference.
     */
    reference operator()(size_t row, size_t col) {
        return this->get_element(row, col, layout());
    }

    /** Access the given element of the matrix.
     *
     * @param row row of element.
     * @param col column of element.
     * @returns const reference.
     */
    const_reference operator()(size_t row, size_t col) const {
        return this->get_element(row, col, layout());
    }

    /** Return access to the data as a raw pointer. */
    pointer data() { return &m_data[0]; }

    /** Return access to the data as a raw pointer. */
    const_pointer data() const { return &m_data[0]; }


  public:

    /** Set the array dimensions.  The previous contents are destroyed
     * before reallocating the array.  If the number of rows and columns
     * isn't changing, nothing happens.  Also, if either rows or cols is 0,
     * the array is cleared.
     *
     * @warning This is not guaranteed to preserve the original data.
     */
    void resize(size_t rows, size_t cols) {

      /* Nothing to do if the size isn't changing: */
      if(rows == m_rows && cols == m_cols) return;

      /* Destroy the current array contents: */
      this->destroy();

      /* Set the new size if non-zero: */
      if(rows*cols > 0) {
	value_type* data = m_alloc.allocate(rows*cols);
	for(size_t i = 0; i < rows*cols; ++ i)
	  m_alloc.construct(&data[i], value_type());

	/* Success, so save the new array and the dimensions: */
	m_rows = rows;
	m_cols = cols;
	m_data = data;
      }
    }

    /** Copy the other array.  The previous contents are destroyed before
     * reallocating the array.  If other == *this, nothing happens.  Also,
     * if either other.rows() or other.cols() is 0, the array is cleared.
     */
    void copy(const dynamic_2D& other) {

      /* Nothing to do if it's the same array: */
      if(&other == this) return;

      /* Destroy the current array contents: */
      this->destroy();

      /* Set the new size if non-zero: */
      size_t rows = other.rows(), cols = other.cols();
      if(rows*cols > 0) {
	value_type* data = m_alloc.allocate(rows*cols);
	for(size_t i = 0; i < rows*cols; ++ i)
	  m_alloc.construct(&data[i], other[i]);

	/* Success, so save the new array and the dimensions: */
	m_rows = rows;
	m_cols = cols;
	m_data = data;
      }
    }


  protected:

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

    /** Destroy the current contents of the array. */
    void destroy() {
      if(m_data) {
	for(size_t i = 0; i < m_rows*m_cols; ++ i)
	  m_alloc.destroy(&m_data[i]);
	m_alloc.deallocate(m_data, m_rows*m_cols);
	m_rows = m_cols = 0;
	m_data = 0;
      }
    }


  protected:

    /** Current array dimensions (may be 0,0). */
    size_t                      m_rows, m_cols;

    /** Array data (may be NULL). */
    value_type*			m_data;

    /** Allocator for the array. */
    allocator_type		m_alloc;
};

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
