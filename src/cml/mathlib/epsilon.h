/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief
 */

#ifndef epsilon_h
#define epsilon_h

namespace cml {

/* @todo: epsilon and tolerance handling.
 *
 * @note This is a placeholder for a more sophisticated epsilon/tolerance
 * system.
 */

template < typename Real >
struct epsilon
{
    typedef Real value_type;
    
private:

    /** For convenience */
    typedef value_type T;
    
public:

    static T placeholder() {
        /* Completely arbitrary placeholder value: */
        return T(0.0001);
    }
};

} // namespace cml

#endif
