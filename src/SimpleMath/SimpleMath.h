#ifndef _SIMPLEMATH_H
#define _SIMPLEMATH_H

/** \brief A highly inefficient (but fast compiling!) math library.
 * 
 * It was conceived by Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 * while he was compiling code that uses a highly efficient math library.
 * 
 * It is intended to be used as a fast compiling substitute for the
 * blazingly fast Eigen3 library and tries to mimic its API to a certain
 * extent.
 *
 * Feel free to use it wherever you like. However, no guarantees are given
 * that this code does what it says it would.
 */
namespace SimpleMath {
}

#include "SimpleMathFixed.h"
#include "SimpleMathDynamic.h"
#include "SimpleMathMixed.h"

#endif /* _SIMPLEMATH_H */
