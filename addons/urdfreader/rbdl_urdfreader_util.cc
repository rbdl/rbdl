#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>

#include "rbdl_urdfreader.h"

#include <iostream>

using namespace std;

bool verbose = false;
string filename = "";

void usage (const char* argv_0) {
	cerr << "Usage: " << argv_0 << "[-v] [-m] [-d] <robot.urdf>" << endl;
	cerr << "  -v | --verbose            enable additional output" << endl;
	cerr << "  -d | --dof-overview       print an overview of the degress of freedom" << endl;
	cerr << "  -m | --model-hierarchy    print the hierarchy of the model" << endl;
	cerr << "  -h | --help               print this help" << endl;
	exit (1);
}


int main (int argc, char *argv[]) {
	if (argc < 2 || argc > 4) {
		usage(argv[0]);
	}

	bool verbose = false;
	bool dof_overview = false;
	bool model_hierarchy = false;

	string filename = argv[1];

	for (int i = 1; i < argc; i++) {
		if (string(argv[i]) == "-v" || string (argv[i]) == "--verbose")
			verbose = true;
		else if (string(argv[i]) == "-d" || string (argv[i]) == "--dof-overview")
			dof_overview = true;
		else if (string(argv[i]) == "-m" || string (argv[i]) == "--model-hierarchy")
			model_hierarchy = true;
		else if (string(argv[i]) == "-h" || string (argv[i]) == "--help")
			usage(argv[0]);
		else
			filename = argv[i];
	}

	RigidBodyDynamics::Model model;

	if (!RigidBodyDynamics::Addons::read_urdf_model(filename.c_str(), &model, verbose)) {
		cerr << "Loading of urdf model failed!" << endl;
		return -1;
	}

	cout << "Model loading successful!" << endl;

	if (dof_overview) {
		cout << "Degree of freedom overview:" << endl;
		cout << RigidBodyDynamics::Utils::GetModelDOFOverview(model);
	}

	if (model_hierarchy) {
		cout << "Model Hierarchy:" << endl;
		cout << RigidBodyDynamics::Utils::GetModelHierarchy (model);
	}

	return 0;
}
