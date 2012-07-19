#include "rbdl.h"
#include "rbdl_urdfreader.h"

#include <iostream>

using namespace std;

bool verbose = false;
string filename = "";

int main (int argc, char *argv[]) {
	if (argc < 2) {
		cerr << "Usage: " << argv[0] << "[-v] <robot.urdf>" << endl;
		return -1;
	}

	for (int i = 1; i < argc; i++) {
		string arg(argv[i]);
		if (arg == "-v")
			verbose = true;
		else
			filename = arg;
	}

	RigidBodyDynamics::Model model;
	if (!RigidBodyDynamics::Addons::read_urdf_model(filename.c_str(), &model, verbose)) {
		cerr << "Loading of urdf model failed!" << endl;
		return -1;
	}

	return 0;
}
