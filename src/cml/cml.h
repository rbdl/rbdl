/* -*- C++ -*- ------------------------------------------------------------
 
Copyright (c) 2007 Jesse Anders and Demian Nave http://cmldev.net/

The Configurable Math Library (CML) is distributed under the terms of the
Boost Software License, v1.0 (see cml/LICENSE for details).

 *-----------------------------------------------------------------------*/
/** @file
 *  @brief Main CML header to include all CML functionality.
 *
 * @todo load vectors, matrices, and quaternions from a stream.
 *
 * @todo Move common vector and matrix class ops to a base class (requires
 * SCOOP-like programming, see below).
 *
 * @todo Implement matrix<>::orthogonalize().
 *
 * @todo Add is_square<>, is_rectangular<>, etc. to make it easier to
 * detect specific matrix types.
 *
 * @todo Implement dedicated square matrix classes to get rid of duplicated
 * code in the specialized matrix classes.
 *
 * @todo Implement automatic temporary generation, along with expression
 * node return types for mat-vec and mat-mat operators.
 *
 * @todo switch to ssize_t instead of size_t to avoid having to explicitly
 * deal with wrap-arounds to 2^32-1 when a size_t is subtracted from.
 *
 * @todo Finish tests for mat-vec multiply.
 *
 * @todo Differentiate between references used for function arguments, and
 * those used for variable types.  In particular, GCC 3.4 requires const T &
 * function arguments to ensure complete unrolling/inlining of expressions.
 *
 * @todo Specialize matrix multiplication based upon the size type (fixed or
 * dynamic). This makes a difference for at least GCC 3.4.
 *
 * @todo need a build system for the tests/ and examples/ directories.
 *
 * @todo clean up the testing infrastructure, and make it easier to add new
 * tests
 *
 * @todo figure out if scalars should be passed by value or reference, or
 * if it should be determined by traits
 *
 * @todo change use of typename and class to be like Alexandrescu book
 *
 * @todo figure out if it makes sense to unroll assignment if either the
 * source expression or the target vector/matrix has a fixed size (right
 * now, unrolling happens only if the target has a fixed size)
 *
 * @todo Allow addition of new types, a la glommable ETs (but simpler).
 * Can use ideas from "SCOOP" method: Nicolas Burrus, Alexandre Duret-Lutz,
 * Thierry Géraud, David Lesage and Raphaël Poss. A Static C++
 * Object-Oriented Programming (SCOOP) Paradigm Mixing Benefits of
 * Traditional OOP and Generic Programming. In the Proceedings of the
 * Workshop on Multiple Paradigm with OO Languages (MPOOL'03) Anaheim, CA,
 * USA Oct. 2003
 */

#ifndef cml_h
#define cml_h

#include <cml/vector.h>
#include <cml/matrix.h>
#include <cml/quaternion.h>
#include <cml/util.h>
#include <cml/mathlib/mathlib.h>

#endif

// -------------------------------------------------------------------------
// vim:ft=cpp
