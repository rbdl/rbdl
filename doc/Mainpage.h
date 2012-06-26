/** \file Mainpage.h 
 * \mainpage RBDL - Rigid Body Dynamics Library
 * \image html rbdl_logo.png
 *
 * This is the documentation of RBDL, the Rigid Body Dynamics Library. The
 * library contains highly efficient code for both forward and inverse
 * dynamics. It includes:
 *
 * \li Recursive Newton Euler Algorithm
 * \li Composite Rigid Body Algorithm
 * \li Articulated Body Algorithm.
 *
 * Furthermore it contains functions for forward and inverse kinematics and
 * contact handling.
 *
 * The code is written by <a
 * href="mailto:martin.felis@iwr.uni-heidelberg.de">Martin Felis
 * <martin.felis@iwr.uni-heidelberg.de></a> and heavily inspired by the
 * pseudo code of the book "Rigid Body Dynamics Algorithms" of <a
 * href="http://royfeatherstone.org" target="_parent">Roy Featherstone</a>.
 * 
 * Development is taking place on <a
 * href="http://bitbucket.org" target="_parent">bitbucket.org</a>. The official repository
 * can be found at:
 * <a href="http://bitbucket.org/rbdl/rbdl/" target="_parent">http://bitbucket.org/rbdl/rbdl/</a>.
 * 
 * The library comes with version 3 of the the
 * <a href="http://eigen.tuxfamily.org/" target="_parent">Eigen</a> math library. More
 * information about it can be found here:
 * <a href="http://eigen.tuxfamily.org/" target="_parent">http://eigen.tuxfamily.org/</a>
 *
 * \section recent_changes Recent Changes :
 * <ul>
 * <li> 18. June 2012: added support of \ref luamodel_introduction</li>
 * <li> 01. June 2012: added support of \ref joint_models_fixed</li>
 * <li> 14. May 2012: fixed Body constructor as reported by Maxime Reis</li>
 * <li> 04. April 2012: added benchmark tool for CRBA</li>
 * <li> 01. March 2012: added multi degree of freedom \ref joint_models</li>
 * <li> 06. Februry 2012: restructured constraint handling using \ref RigidBodyDynamics::ConstraintSet</li>
 * <li> 24. January 2012: implemented compact and fast representation of \ref RigidBodyDynamics::Math::SpatialTransform </li>
 * </ul>
 *
 * \section Example Example
 *
 * A simple example for creation of a model and computation of the forward
 * dynamics using the C++ API can be found \ref SimpleExample "here".
 *
 * Another example that uses the \ref luamodel_introduction Addon can be found \ref
 * LuaModelExample "here".
 * 
 * \section ModuleOverview API reference separated by functional modules
 * 
 * \li \ref model_group
 * \li \ref kinematics_group
 * \li \ref dynamics_group
 * \li \ref contacts_group
 *
 * \section Licensing Licensing
 *
 * The library is published under the very permissive zlib free software
 * license which should allow you to use the software wherever you need.
 * Here is the full license text:
 * \verbatim
RBDL - Rigid Body Dynamics Library
Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source
   distribution.
\endverbatim
 */
