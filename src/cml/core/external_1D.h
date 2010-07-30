/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 *
 * Defines the fixed-size and runtime-sized external 1D arrays.
 *
 * @todo Need a better way to designate non-resizable, run-time sized
 * arrays (e.g. by a resizeable tag).
 */

#ifndef external_1D_h
#define external_1D_h

#include <cml/core/common.h>
#include <cml/core/cml_meta.h>
#include <cml/core/cml_assert.h>
#include <cml/external.h>

namespace cml {

/** Fixed-size external 1D array.
 *
 * Both the memory and the size are fixed at compile time, and cannot be
 * changed.
 */
template<typename Element, int Size = -1>
class external_1D
{
  public:

    /* Require Size > 0: */
    CML_STATIC_REQUIRE(Size > 0);

    /* Record the generator: */
    typedef external<Size,-1> generator_type;

    /* Standard: */
    typedef Element value_type;
    typedef Element* pointer;
    typedef Element& reference;
    typedef const Element& const_reference;
    typedef const Element* const_pointer;

    /* Array implementation: */
    typedef value_type array_impl[Size];

    /* For matching by memory type: */
    typedef external_memory_tag memory_tag;

    /* For matching by size type: */
    typedef fixed_size_tag size_tag;

    /* For matching by resizability: */
    typedef not_resizable_tag resizing_tag;

    /* For matching by dimensions: */
    typedef oned_tag dimension_tag;


  public:

    /** The length as an enumerated value. */
    enum { array_size = Size };


  public:

    external_1D(pointer const ptr)
        : m_data(ptr) {}


  public:

    /** Return the number of elements in the array. */
    size_t size() const { return size_t(array_size); }

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
    pointer data() { return m_data; }

    /** Return access to the data as a raw pointer. */
    const_pointer data() const { return m_data; }


  protected:

    pointer const               m_data;


  private:

    /* Initialization without an argument isn't allowed: */
    external_1D();
};

/** Run-time sized external 1D array.
 *
 * Both the memory and the size are fixed at run-time, and cannot be
 * changed.  This is a specialization for the case that Rows and Cols are
 * not specified (i.e. given as the default of -1,-1).
 */
template<typename Element>
class external_1D<Element,-1>
{
  public:

    /* Record the generator.  Note: this is *not* unique, as it is the same
     * generator used by external_2D.  However, external_2D is used only by
     * matrix<> classes, so this is not a problem.
     */
    typedef external<> generator_type;

    /* Standard: */
    typedef Element value_type;
    typedef Element* pointer;
    typedef Element& reference;
    typedef const Element& const_reference;
    typedef const Element* const_pointer;

    /* For matching by memory type: */
    typedef external_memory_tag memory_tag;

    /* For matching by size type: */
    typedef dynamic_size_tag size_tag;

    /* For matching by resizability: */
    typedef not_resizable_tag resizing_tag;

    /* For matching by dimensions: */
    typedef oned_tag dimension_tag;


  public:

    /** The length as an enumerated value. */
    enum { array_size = -1 };


  public:

    external_1D(pointer const ptr, size_t size)
        : m_data(ptr), m_size(size) {}


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
    pointer data() { return m_data; }

    /** Return access to the data as a raw pointer. */
    const_pointer data() const { return m_data; }


  protected:

    pointer const               m_data;
    const size_t                m_size;


  private:

    /* Initialization without an argument isn't allowed: */
    external_1D();
};

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
