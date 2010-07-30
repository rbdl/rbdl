/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef fixed_1D_h
#define fixed_1D_h

#include <cml/core/common.h>
#include <cml/core/cml_meta.h>
#include <cml/core/cml_assert.h>
#include <cml/fixed.h>

namespace cml {

/** Statically-allocated array.
 *
 * @note This class is designed to have the same size as a C array with the
 * same length.  It's therefore possible (but not recommended!) to coerce
 * a normal C array into a fixed_1D<> like this:
 *
 * typedef fixed_1D<double,10> array;
 * double c_array[10];
 * array& array_object = *((array*)&c_array);
 * double e1 = array_object[1];
 *
 * It's also possible to do this with a pointer to an array of values (e.g. a
 * double*), whether or not it was actually declared as a fixed C array.  This
 * is HIGHLY DISCOURAGED, though. It's relatively straightforward to implement
 * a separate class to take a C array (or pointer) and turn it into an array
 * object.
 *
 * @sa cml::fixed
 *
 * @internal Do <em>not</em> add the empty constructor and destructor; at
 * least one compiler (Intel C++ 9.0) fails to optimize them away, and they
 * aren't needed anyway here.
 */
template<typename Element, int Size>
class fixed_1D
{
  public:

    /* Require Size > 0: */
    CML_STATIC_REQUIRE(Size > 0);

    /* Record the generator: */
    typedef fixed<Size,-1> generator_type;

    /* Standard: */
    typedef Element value_type;
    typedef Element* pointer;
    typedef Element& reference;
    typedef const Element& const_reference;
    typedef const Element* const_pointer;

    /* Array implementation: */
    typedef value_type array_impl[Size];

    /* For matching by memory type: */
    typedef fixed_memory_tag memory_tag;

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
    pointer data() { return &m_data[0]; }

    /** Return access to the data as a raw pointer. */
    const_pointer data() const { return &m_data[0]; }

  protected:

    fixed_1D() {}


  protected:

    array_impl                  m_data;
};

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
