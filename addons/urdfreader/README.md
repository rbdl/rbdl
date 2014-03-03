urdfreader - load models from (URDF Unified Robot Description Format) files
Copyright (c) 2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>

Requirements
============

This addon depends on urdfdom to load and access the model data in the URDF
files.

See https://github.com/ros/urdfdom for more details on how to
install urdfdom.

Warning
=======

This code is not properly tested as I do not have a proper urdf robot
model. If anyone has one and also some reference values that should come
out for the dynamics computations, please let me know.

Licensing
=========

This code is published under the zlib license, however some parts of the
CMake scripts are taken from other projects and are licensed under
different terms.

Full license text:

-------
urdfreader - load models from URDF (Unified Robot Description Format) files
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
