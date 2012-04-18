#include "rbdl.h"
#include "rbdl_urdfreader.h"

#include <iostream>

using namespace std;

int main (int argc, char *argv[]) {
	if (argc != 2) {
		cerr << "Usage: " << argv[0] << " <robot.urdf>" << endl;
		return -1;
	}

	RigidBodyDynamics::Model model;
	if (!RigidBodyDynamics::Addons::read_urdf_model(argv[1], &model, true)) {
		cerr << "Loading of urdf model failed!" << endl;
		return -1;
	}

	return 0;
}
