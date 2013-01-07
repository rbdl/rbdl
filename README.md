RBDL - Rigid Body Dynamics Library
Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>

Introduction
============

The RBDL is a highly efficient C++ library that contains some essential
rigid body dynamics algorithms such as the Articulated Body Algorithm (ABA)
for forward dynamics, Newton-Euler Algorithm for inverse dynamics and the
Composite Rigid Body Algorithm (CRBA) for the efficient computation of
the joint space inertia matrix. It further contains code for forward and
inverse kinematics and handling of external constraints such as contacts
and collisions.

The code was written by Martin Felis <martin.felis@iwr.uni-heidelberg.de>
and tightly follows the notation used in Roy Featherstone''s book "Rigid
Body Dynamics Algorithm".

Documentation
=============

The documentation is contained in the code and can be extracted with the
tool [doxygen](http://www.doxygen.org).

To create the documentation simply run

    doxygen Doxyfile

which will generate the documentation in the subdirectory ./doc/html. The
main page will then be located in ./doc/html/index.html.

An online version of the generated documentation can be found at
[http://rbdl.bitbucket.org](http://rbdl.bitbucket.org).

Getting RBDL
============

The latest stable code can be obtained from

    https://bitbucket.org/rbdl/rbdl/get/default.zip

The official mercurial repository can be cloned from

    https://bitbucket.org/rbdl/rbdl

(See [http://mercurial.selenic.com/](http://mercurial.selenic.com) for
mercurial clients.)

Building and Installation
=========================

The RBDL is built using CMake (http://www.cmake.org). To compile the
library in a separate directory in Release mode use:

    mkdir build
    cd build/
    cmake ../ -D CMAKE_BUILD_TYPE=Release
    make

Licensing
=========

The library is published under the very permissive zlib free software
license which should allow you to use the software wherever you need. 

This is the full license text (zlib license):

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
    
       2. Altered source versions must be plainly marked as such, and must not
			 be misrepresented as being the original software.
    
       3. This notice may not be removed or altered from any source
       distribution.

Please note that this library also comes with the Eigen3 library which is
primarily licensed under the MPL2 license. More information about these
can be found at:
	
 * [http://www.mozilla.org/MPL/2.0/](http://www.mozilla.org/MPL/2.0/)
 * [http://www.mozilla.org/MPL/2.0/FAQ.html](http://www.mozilla.org/MPL/2.0/FAQ.html)
 * [http://eigen.tuxfamily.org]([http://eigen.tuxfamily.org])
