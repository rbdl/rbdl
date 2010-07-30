/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef matrix_print_h
#define matrix_print_h

#include <iostream>

namespace cml {

/** Output a matrix to a std::ostream. */
template<typename E, class AT, typename BO, class L> inline std::ostream&
operator<<(std::ostream& os, const matrix<E,AT,BO,L>& m)
{
    for(size_t i = 0; i < m.rows(); ++i) {
        os << "[";
        for(size_t j = 0; j < m.cols(); ++j) {
            os << " " << m(i,j);
        }
        os << " ]";
        if (i != m.rows()-1) {
            os << std::endl;
        }
    }
    return os;
}

/** Output a matrix expression to a std::ostream. */
template< class XprT > inline std::ostream&
operator<<(std::ostream& os, const et::MatrixXpr<XprT>& m)
{
    for(size_t i = 0; i < m.rows(); ++i) {
        os << "[";
        for(size_t j = 0; j < m.cols(); ++j) {
            os << " " << m(i,j);
        }
        os << " ]";
        if (i != m.rows()-1) {
            os << std::endl;
        }
    }
    return os;
}

} // namespace cml

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
