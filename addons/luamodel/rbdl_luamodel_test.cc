#include "rbdl.h"
#include "rbdl_luamodel.h"

#include <iostream>

using namespace std;

int main (int argc, char *argv[]) {
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <model.lua>" << endl;
		return -1;
	}

	RigidBodyDynamics::Model model;
	if (!RigidBodyDynamics::Addons::read_luamodel(argv[1], &model, true)) {
		cerr << "Loading of lua model failed!" << endl;
		return -1;
	}

	cout << "Model loading successful: " << model.dof_count << " dofs." << endl;

	return 0;
}
