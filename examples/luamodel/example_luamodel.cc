/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>

#include <rbdl.h>

#ifndef BUILD_ADDON_LUAMODEL
	#error "Error: RBDL addon BUILD_LUAMODELS not activated."
#endif

#include <addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
	Model* model = NULL;

	model = new Model();
	model->Init();

	if (!Addons::LuaModelReadFromFile ("./samplemodel.lua", model, false)) {
		std::cerr << "Error loading model ./samplemodel.lua" << std::endl;
		abort();
	}

	VectorNd Q = VectorNd::Zero (model->dof_count);
	VectorNd QDot = VectorNd::Zero (model->dof_count);
	VectorNd Tau = VectorNd::Zero (model->dof_count);
	VectorNd QDDot = VectorNd::Zero (model->dof_count);

 	ForwardDynamics (*model, Q, QDot, Tau, QDDot);

	std::cout << QDDot.transpose() << std::endl;

	delete model;

 	return 0;
}

