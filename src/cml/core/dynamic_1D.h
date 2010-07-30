/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef dynamic_1D_h
#define dynamic_1D_h

#include <memory>
#include <cml/core/common.h>
#include <cml/dynamic.h>

namespace cml {

/** Dynamically-sized and allocated 1D array.
 *
 * @note The allocator should be an STL-compatible allocator.
 *
 * @internal The internal array type <em>must</em> have the proper copy
 * semantics, otherwise copy construction will fail.
 */
template<typename Element, class Alloc>
class dynamic_1D
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

    /* For matching by memory type: */
    typedef dynamic_memory_tag memory_tag;

    /* For matching by size type: */
    typedef dynamic_size_tag size_tag;

    /* For matching by resizability: */
    typedef resizable_tag resizing_tag;

    /* For matching by dimensions: */
    typedef oned_tag dimension_tag;


  public:

    /** Dynamic arrays have no fixed size. */
    enum { array_size = -1 };


  public:

    /** Construct a dynamic array with no size. */
    dynamic_1D() : m_size(0), m_data(0), m_alloc() {}

    /** Construct a dynamic array given the size. */
    explicit dynamic_1D(size_t size) : m_size(0), m_data(0), m_alloc() {
      this->resize(size);
    }

    /** Copy construct a dynamic array. */
    dynamic_1D(const dynamic_1D& other)
      : m_size(0), m_data(0), m_alloc()
    {
      this->copy(other);
    }

    ~dynamic_1D() {
      this->destroy();
    }


  public:

    /** Return the number of elements in the array. */
    size_t size() const { return m_size; }

    /** Access to the data as a C array.
     *
     * @param i a size_t index into the array.
     * @return a mutable reference to the array value at i.
     *
     * @note This function does not range-check the argument.
     */
    reference operator[](size_t i) { return m_data[i]; }

    /** Const access to the data as a C array.
     *
     * @param i a size_t index into the array.
     * @return a const reference to the array value at i.
     *
     * @note This function does not range-check the argument.
     */
    const_reference operator[](size_t i) const { return m_data[i]; }

    /** Return access to the data as a raw pointer. */
    pointer data() { return &m_data[0]; }

    /** Return access to the data as a raw pointer. */
    const_pointer data() const { return &m_data[0]; }


  public:

    /** Set the array size to the given value.  The previous contents are
     * destroyed before reallocating the array.  If s == size(),
     * nothing happens.
     *
     * @warning This is not guaranteed to preserve the original data.
     */
    void resize(size_t s) {

      /* Nothing to do if the size isn't changing: */
      if(s == m_size) return;

      /* Destroy the current array contents: */
      this->destroy();

      /* Set the new size if non-zero: */
      if(s > 0) {
	value_type* data = m_alloc.allocate(s);
	for(size_t i = 0; i < s; ++ i)
	  m_alloc.construct(&data[i], value_type());

	/* Success, save s and data: */
	m_size = s;
	m_data = data;
      }
    }

    /** Copy the source array. The previous contents are destroyed before
     * reallocating the array.  If other == *this, nothing happens.
     */
    void copy(const dynamic_1D& other) {

      /* Nothing to do if it's the same array: */
      if(&other == this) return;

      /* Destroy the current array contents: */
      this->destroy();

      /* Set the new size if non-zero: */
      size_t s = other.size();
      if(s > 0) {
	value_type* data = m_alloc.allocate(s);
	for(size_t i = 0; i < s; ++ i)
	  m_alloc.construct(&data[i], other[i]);

	/* Success, so save the new array and the size: */
	m_size = s;
	m_data = data;
      }
    }


  protected:

    /** Destroy the current contents of the array. */
    void destroy() {
      if(m_data) {
	for(size_t i = 0; i < m_size; ++ i)
	  m_alloc.destroy(&m_data[i]);
	m_alloc.deallocate(m_data, m_size);
	m_size = 0;
	m_data = 0;
      }
    }


  protected:

    /** Current array size (may be 0). */
    size_t			m_size;

    /** Array data (may be NULL). */
    value_type*			m_data;

    /** Allocator for the array. */
    allocator_type		m_alloc;
};

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
