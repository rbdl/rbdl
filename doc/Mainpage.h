/** \file Mainpage.h 
 * \mainpage Mainpage
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
 * For optimal performance it is advised to use version 3 of the Eigen 
 * <a href="http://eigen.tuxfamily.org/" target="_parent">Eigen</a> math library. More
 * information about it can be found here:
 * <a href="http://eigen.tuxfamily.org/" target="_parent">http://eigen.tuxfamily.org/</a>.
 * The library must be obtained and installed separately.
 *
 * \section download Download :
 *
 * You can download the most recent stable version as zip file from
 * here:<br>
 *   <a href="https://bitbucket.org/rbdl/rbdl/get/default.zip">https://bitbucket.org/rbdl/rbdl/get/default.zip</a>
 *
 * All development takes place on Bitbucket and you can follow RBDL's
 * development here:<br>
 *   <a href="https://bitbucket.org/rbdl/rbdl">https://bitbucket.org/rbdl/rbdl</a>
 *
 * \section recent_changes Recent Changes :
 * <ul>
 * <li>13 July 2014: New version: 2.3.1:
 *   <ul>
 *     <li><b>critical</b>: fixed angular momentum computation. Version 2.3.0 produced wrong
 *       results. (Thanks to Hilario Tome and Benjamin Michaud for reporting!)</li>
 *     <li><b>critical</b>: fixed JointTypeEulerZYX. Previous versions produce wrong results!
 *     <li>fixed library version number for the LuaModel addon.</li>
 *   </ul>
 * </li>
 * <li>17 March 2014: New version: 2.3.0:
 *   <ul>
 *     <li>Joint Space Inertia Matrix does \b not get cleared anymore when
 *     calling \ref RigidBodyDynamics::CompositeRigidBodyAlgorithm "CompositeRigidBodyAlgorithm"</li>
 *     <li>using the default <a href="http://eigen.tuxfamily.org/dox-devel/group__TopicStorageOrders.html">column-major</a> ordering when using Eigen3</li>
 *     <li>added experimental joint type \ref RigidBodyDynamics::JointTypeEulerZYX "JointTypeEulerZYX"</li>
 *     <li> added energy computations
 *     \ref RigidBodyDynamics::Utils::CalcCenterOfMass "Utils::CalcCenterOfMass",
 *     \ref RigidBodyDynamics::Utils::CalcPotentialEnergy "Utils::CalcPotentialEnergy",
 *     \ref RigidBodyDynamics::Utils::CalcKineticEnergy "Utils::CalcKineticEnergy", and
 *     \ref RigidBodyDynamics::Utils::CalcAngularMomentum "Utils::CalcAngularMomentum".</li>
 *     <li> Updated URDF loader for ROS Groovy/Hydro (thanks to Benjamin Chrétien!)
 *   </ul>
 * <li>06 November 2013: New version 2.2.2: adjusted Body default constructor (inertia matrix now 3x3 identity instead of zero matrix)</li>
 * <li> 4 November 2013: New version 2.2.1: fixed exported library version</li>
 * <li> 28 October 2013: New version 2.2.0: added support for spherical joints that do not suffer from \ref joint_singularities</li>
 * <li> 29 September 2013: New version 2.1.0: adjusted build settings and symbol export to be debian compatible. Removed vendor code such as Lua 5.2 and UnitTest++. Must be pre-installed if tests or LuaModel Addon is enabled.</li>
 * <li>05 September 2013: New version 2.0.1: fixed some errors on older compilers and CMake configuration of examples. No changes required when migrating from 2.0.0.</li>
 * <li> 18. July 2013: new API version 2.0.0 for details see (\ref api_version_checking_page) </li>
 * <li> 20. February 2013: removed too specialized RigidBodyDynamics::Body constructor (API version 1.1.0)</li>
 * <li> 29. January 2013: added code for \ref api_version_checking_page. Current is 1.0.0.</li>
 * <li> 18. June 2012: added support of \ref luamodel_introduction</li>
 * <li> 01. June 2012: added support of \ref joint_models_fixed</li>
 * <li> 14. May 2012: fixed Body constructor as reported by Maxime Reis</li>
 * <li> 04. April 2012: added benchmark tool for CRBA</li>
 * <li> 01. March 2012: added multi degree of freedom \ref joint_description</li>
 * <li> 06. Februry 2012: restructured constraint handling using \ref RigidBodyDynamics::ConstraintSet</li>
 * <li> 24. January 2012: implemented compact and fast representation of \ref RigidBodyDynamics::Math::SpatialTransform </li>
 * </ul>
 *
 * \section Example Examples
 *
 * A simple example for creation of a model and computation of the forward
 * dynamics using the C++ API can be found \ref SimpleExample "here".
 *
 * Another example that uses the \ref addon_luamodel_page "LuaModel Addon" can be found \ref
 * LuaModelExample "here".
 * 
 * \section ModuleOverview API reference separated by functional modules
 * 
 * \li \subpage modeling_page
 * \li \subpage joint_description
 * \li \subpage kinematics_page
 * \li \subpage dynamics_page
 * \li \subpage contacts_page
 * \li \subpage addon_luamodel_page 
 *
 * The page \subpage api_version_checking_page contains information about
 * incompatibilities of the existing versions and how to migrate.
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
