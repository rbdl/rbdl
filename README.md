[![Build Status](https://travis-ci.org/rbdl/rbdl.svg?branch=master)](https://travis-ci.org/rbdl/rbdl)

RBDL - Rigid Body Dynamics Library
Copyright (c) 2011-2020 Martin Felis <martin@fysx.org>

Introduction
============

RBDL is a highly efficient C++ library that contains some essential rigid
body dynamics algorithms such as the Articulated Body Algorithm (ABA) for
forward dynamics, Recursive Newton-Euler Algorithm (RNEA) for inverse
dynamics and the Composite Rigid Body Algorithm (CRBA) for the efficient
computation of the joint space inertia matrix. It further contains code for
Jacobians, forward and inverse kinematics, handling of external
constraints such as contacts and collisions, and closed-loop models.

The code is developed by Martin Felis <martin@fysx.org>
at the research group [Optimization in Robotics and Biomechanics
(ORB)](http://orb.iwr.uni-heidelberg.de) of the [Interdisciplinary Center
for Scientific Computing (IWR)](http://www.iwr.uni-heidelberg.de) at
[Heidelberg University](http://www.uni-heidelberg.de). The code tightly
follows the notation used in Roy Featherstone''s book "Rigid Body Dynamics
Algorithm".

Recent Changes
==============
  * 02 May 2018: New version 2.6.0:
    * Added support for closed-loop models by replacing Contacts API by a new
      Constraints API. Loop constraints can be stabilized using Baumgarte
      stabilization. Special thanks to Davide Corradi for this contribution!
    * New constraint type CustomConstraint: a versatile interface to define
      more general types of constraints (e.g. time dependent), contributed by
      Matthew J. Millard.
    * New joint type JointTypeHelical that can be used for screwing motions
      (translations and simultaneous rotations), contributed by Stuart Anderson.
    * Added support to specify external forces on bodies on constrained forward
      dynamics and NonlinearEffects() (contributed by Matthew J. Millard)
    * Changed Quaternion multiplication behaviour for a more standard
      convention: multiplying q1 (1,0,0,0) with q2 (0,1,0,0) results now in
      (0,0,1,0) instead of the previous (0,0,-1,0).
    * Removed Model::SetFloatingBaseBody(). Use JointTypeFloatingBase instead.
    * LuaModel: extended specification to support ConstraintSets.
  * 28 April 2016: New version 2.5.0:
    * Added an experimental Cython based Python wrapper of RBDL. The API is
      very close to the C++ API. For a brief glimpse of the API see the file
      python/test_wrapper.py.
    * Matthew Millard added CustomJoints which allow to create different joint
      types completely by user code. They are implemented as proxy joints for
      which their behaviour is specified using virtual functions.
    * Added CalcMInvTimesTau() that evaluates multiplication of the inverse of
      the joint space inertia matrix with a vector in O(n) time.
    * Added JointTypeFloatingBase which uses TX,TY,TZ and a spherical joint for
      the floating base joint.
    * Loading of floating base URDF models must now be specified as a third
      parameter to URDFReadFromFile() and URDFReadFromString()
    * Added the URDF code from Bullet3 which gets used when ROS is not found.
      Otherwise use the URDF libraries found via Catkin.
    * Added CalcPointVelocity6D, CalcPointAcceleration6D, and CalcPointJacobian6D
      that compute both linear and angular quantities
    * Removed Model::SetFloatingBase (body). Use a 6-DoF joint or
      JointTypeFloatingBase instead.
    * Fixed building issues when building DLL with MSVC++.
  * 20 April 2016: New version 2.4.1:
    * This is a bugfix release that maintains binary compatibility and only fixes
    erroneous behaviour.
    * critical: fixed termination criterion for InverseKinematics. The termination
      criterion would be evaluated too early and thus report convergence too
      early. This was reported independently by Kevin Stein, Yun Fei, and Davide
      Corradi. Thanks for the reports!
    * critical: fixed CompositeRigidBodyAlgorithm when using spherical joints
      (thanks to Sébastien Barthélémy for reporting!)
  * 23 February 2015: New version 2.4.0:
    * Added sparse range-space method ForwardDynamicsContactsRangeSpaceSparse()
      and ComputeContactImpulsesRangeSpaceSparse()
    * Added null-space method ForwardDynamicsContactsNullSpace()
      and ComputeContactImpulsesNullSpace()
    * Renamed ForwardDynamicsContactsLagrangian() to
      ForwardDynamicsContactsDirect() and
      ComputeContactImpulsesLagrangian() to ComputeContactImpulsesDirect()
    * Renamed ForwardDynamicsContacts() to ForwardDynamicsContactsKokkevis()
    * Removed/Fixed CalcAngularMomentum(). The function produced wrong values. The
      functionality has been integrated into CalcCenterOfMass().
    * CalcPointJacobian() does not clear the argument of the result anymore.
      Caller has to ensure that the matrix was set to zero before using this
      function.
    * Added optional workspace parameters for ForwardDynamicsLagrangian() to
      optionally reduce memory allocations
    * Added JointTypeTranslationXYZ, JointTypeEulerXYZ, and JointTypeEulerYXZ
      which are equivalent to the emulated multidof joints but faster.
    * Added optional parameter to CalcCenterOfMass to compute angular momentum.
    * Added CalcBodySpatialJacobian()
    * Added CalcContactJacobian()
    * Added NonlinearEffects()
    * Added solving of linear systems using standard Householder QR
    * LuaModel: Added LuaModelReadFromLuaState()
    * URDFReader: Fixed various issues and using faster joints for floating
      base models
    * Various performance improvements

For a complete history see doc/api_changes.txt.

Documentation
=============

The documentation is contained in the code and can be extracted with the
tool [doxygen](http://www.doxygen.org).

To create the documentation simply run

    doxygen Doxyfile

which will generate the documentation in the subdirectory ./doc/html. The
main page will then be located in ./doc/html/index.html.

An online version of the generated documentation can be found at
[https://rbdl.github.io](https://rbdl.github.io).

Getting RBDL
============

The latest stable code can be obtained from the master branch at

    https://github.com/rbdl/rbdl

Building and Installation
=========================

The RBDL is built using CMake
([http://www.cmake.org](http://www.cmake.org)). To compile the library in
a separate directory in Release mode use:

    mkdir build
    cd build/
    cmake -D CMAKE_BUILD_TYPE=Release ../
    make

For optimal performance it is highly recommended to install the Eigen3
linear algebra library from
[http://eigen.tuxfamily.org](http://eigen.tuxfamily.org). RBDL also
comes with a simple, albeit much slower math library (SimpleMath) that can
be used by enabling `RBDL_USE_SIMPLE_MATH`, i.e.:

    cmake -D RBDL_USE_SIMPLE_MATH=TRUE ../

Python Bindings
===============

RBDL can also build an experimental python wrapper that works with python 3 and python 2. To do this enable the
the `RBDL_BUILD_PYTHON_WRAPPER` cmake options. This will build the wrapper for python 3, if you want to use python 2 instead
you will also have to enable the `RBDL_USE_PYTHON_2` cmake option. The result of this is an extra python directory in the build
directory. From within which you can install it using setup.py. This is done automically when using `make install`

Citation
========

An overview of the theoretical and implementation details has been
published in [https://doi.org/10.1007/s10514-016-9574-0](Felis,
M.L. Auton Robot (2017) 41: 495). To cite RBDL in your academic
research you can use the following BibTeX entry:

    @Article{Felis2016,
      author="Felis, Martin L.",
      title="RBDL: an efficient rigid-body dynamics library using recursive algorithms",
      journal="Autonomous Robots",
      year="2016",
      pages="1--17",
      issn="1573-7527",
      doi="10.1007/s10514-016-9574-0",
      url="http://dx.doi.org/10.1007/s10514-016-9574-0"
    }

Licensing
=========

The library is published under the very permissive zlib free software
license which should allow you to use the software wherever you need.

This is the full license text (zlib license):

    RBDL - Rigid Body Dynamics Library
    Copyright (c) 2011-2020 Martin Felis <martin@fysx.org>

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

       2. Altered source versions must be plainly marked as such, and must not
       be misrepresented as being the original software.

       3. This notice may not be removed or altered from any source
       distribution.

Acknowledgements
================

Work on this library was originally funded by the [Heidelberg Graduate
School of Mathematical and Computational Methods for the Sciences
(HGS)](http://hgs.iwr.uni-heidelberg.de/hgs.mathcomp/), and the European
FP7 projects [ECHORD](http://echord.eu) (grant number 231143) and
[Koroibot](http://koroibot.eu) (grant number 611909).
