/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_translation_h
#define matrix_translation_h

#include <cml/mathlib/checking.h>

/* Functions for getting and setting the translation of a 3D or 2D affine
 * transform.
 */

namespace cml {

//////////////////////////////////////////////////////////////////////////////
// Functions for setting the translation of a 3D or 2D affine transform matrix
//////////////////////////////////////////////////////////////////////////////

/** Set the translation of a 3D affine transform */
template < typename E, class A, class B, class L > void
matrix_set_translation(matrix<E,A,B,L>& m, E x, E y, E z)
{
    /* Checking */
    detail::CheckMatAffine3D(m);
    
    m.set_basis_element(3,0,x);
    m.set_basis_element(3,1,y);
    m.set_basis_element(3,2,z);
}

/** Set the translation of a 3D affine transform with z set to 0 */
template < typename E, class A, class B, class L > void
matrix_set_translation(matrix<E,A,B,L>& m, E x, E y)
{
    typedef matrix<E,A,B,L> matrix_type;
    typedef typename matrix_type::value_type value_type;
    
    matrix_set_translation(m, x, y, value_type(0));
}

/** Set the translation of a 3D affine transform from a 3D or 2D vector */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_translation(matrix<E,A,B,L>& m, const VecT& translation)
{
    /* Checking */
    detail::CheckVec2Or3(translation);
    
    if (translation.size() == 3) {
        matrix_set_translation(
            m,translation[0], translation[1], translation[2]);
    } else { // translation.size() == 2
        matrix_set_translation(m, translation[0], translation[1]);
    }
}

/** Set the translation of a 2D affine transform */
template < typename E, class A, class B, class L > void
matrix_set_translation_2D(matrix<E,A,B,L>& m, E x, E y)
{
    /* Checking */
    detail::CheckMatAffine2D(m);
    
    m.set_basis_element(2,0,x);
    m.set_basis_element(2,1,y);
}

/** Set the translation of a 2D affine transform from a 2D vector */
template < typename E, class A, class B, class L, class VecT > void
matrix_set_translation_2D(matrix<E,A,B,L>& m, const VecT& translation)
{
    /* Checking */
    detail::CheckVec2(translation);
    
    matrix_set_translation_2D(m, translation[0], translation[1]);
}

//////////////////////////////////////////////////////////////////////////////
// Functions for getting the translation of a 3D or 2D affine transform matrix
//////////////////////////////////////////////////////////////////////////////

/** Get the translation of a 3D affine transform */
template < class MatT > vector< typename MatT::value_type, fixed<3> >
matrix_get_translation(const MatT& m)
{
    typedef typename MatT::value_type value_type;
    typedef vector< value_type, fixed<3> > vector_type;

    /* Checking */
    detail::CheckMatAffine3D(m);
    
    return vector_type(
        m.basis_element(3,0),
        m.basis_element(3,1),
        m.basis_element(3,2)
    );
}

/** Get the translation of a 3D affine transform */
template < class MatT > void
matrix_get_translation(
    const MatT& m,
    typename MatT::value_type& t1,
    typename MatT::value_type& t2,
    typename MatT::value_type& t3
    )
{
    typedef typename MatT::value_type value_type;
    typedef vector< value_type, fixed<3> > vector_type;

    /* Checking */
    detail::CheckMatAffine3D(m);
    
    t1 = m.basis_element(3,0);
    t2 = m.basis_element(3,1);
    t3 = m.basis_element(3,2);
}

/** Get the translation of a 2D affine transform */
template < class MatT > vector< typename MatT::value_type, fixed<2> >
matrix_get_translation_2D(const MatT& m)
{
    typedef typename MatT::value_type value_type;
    typedef vector< value_type, fixed<2> > vector_type;

    /* Checking */
    detail::CheckMatAffine2D(m);
    
    return vector_type(m.basis_element(2,0), m.basis_element(2,1));
}

/** Get the translation of a 2D affine transform */
template < class MatT > void
matrix_get_translation_2D(
    const MatT& m,
    typename MatT::value_type& t1,
    typename MatT::value_type& t2
    )
{
    typedef typename MatT::value_type value_type;
    typedef vector< value_type, fixed<2> > vector_type;

    /* Checking */
    detail::CheckMatAffine2D(m);
    
    t1 = m.basis_element(2,0);
    t2 = m.basis_element(2,1);
}

//////////////////////////////////////////////////////////////////////////////
// Function for getting the translation of a 3D view matrix
//////////////////////////////////////////////////////////////////////////////

/** Get the translation of a 3D affine transform */
template < class MatT > vector< typename MatT::value_type, fixed<3> >
matrix_get_view_translation(const MatT& m)
{
    typedef typename MatT::value_type value_type;
    typedef vector< value_type, fixed<3> > vector_type;
    
    vector_type x, y, z;
    matrix_get_basis_vectors(m,x,y,z);
    vector_type p = matrix_get_translation(m);
    return vector_type(-dot(p,x),-dot(p,y),-dot(p,z));
}

} // namespace cml

#endif
