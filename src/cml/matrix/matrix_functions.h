/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_functions_h
#define matrix_functions_h

namespace cml {

/** Set the given matrix to the identity matrix.
 *
 * This only makes sense for a square matrix, but no error will be
 * signaled if the matrix is not square.
 *
 * @todo This should return a MatrixXpr to allow loop unrolling, as should
 * the class method.
 */
template<typename E, class AT, typename BO, typename L>
inline matrix<E,AT,BO,L>
identity(const matrix<E,AT,BO,L>& m)
{
    typename matrix<E,AT,BO,L>::temporary_type result;

    /* This is a no-op for fixed-size matrices: */
    cml::et::detail::Resize(result, m.size());
    result.identity();
    return result;
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
