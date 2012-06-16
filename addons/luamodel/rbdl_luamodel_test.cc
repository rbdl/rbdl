#include "rbdl.h"
#include "rbdl_luamodel.h"

#include <iostream>
#include <iomanip>
#include <sstream>

using namespace std;

using namespace RigidBodyDynamics::Math;

string get_body_name (const RigidBodyDynamics::Model &model, unsigned int body_id) {
	if (model.mBodies[body_id].mMass == 0.) {
		// this seems to be a virtual body that was added by a multi dof joint
		unsigned int child_index = 0;

		// if there is not a unique child we do not know what to do...
		if (model.mu[body_id].size() != 1)
			return "";

		unsigned int child_id = model.mu[body_id][0];

		return get_body_name (model, model.mu[body_id][0]);
	}

	return model.GetBodyName(body_id);
}

string get_dof_name (const SpatialVector &joint_dof) {
	if (joint_dof == SpatialVector (1., 0., 0., 0., 0., 0.)) 
		return "RX";
	else if (joint_dof == SpatialVector (0., 1., 0., 0., 0., 0.))
		return "RY";
	else if (joint_dof == SpatialVector (0., 0., 1., 0., 0., 0.))
		return "RZ";
	else if (joint_dof == SpatialVector (0., 0., 0., 1., 0., 0.))
		return "TX";
	else if (joint_dof == SpatialVector (0., 0., 0., 0., 1., 0.))
		return "TY";
	else if (joint_dof == SpatialVector (0., 0., 0., 0., 0., 1.))
		return "TZ";

	ostringstream dof_stream(ostringstream::out);
	dof_stream << "custom (" << joint_dof.transpose() << ")";
	return dof_stream.str();
}

void usage (const char* argv_0) {
	cerr << "Usage: " << argv_0 << "[-v] <model.lua>" << endl;
	exit (1);
}

int main (int argc, char *argv[]) {
	if (argc < 2 || argc > 3) {
		usage(argv[0]);
	}

	bool verbose = false;
	string filename = argv[1];

	if (argc == 3) {
		if (string(argv[1]) == "-v" || string(argv[1]) == "--verbose") {
			verbose = true;
			filename = argv[2];
		} else if (string(argv[2]) == "-v" || string(argv[2]) == "--verbose") {
			verbose = true;
			filename = argv[1];
		} else
			usage(argv[0]);
	}

	RigidBodyDynamics::Model model;
	if (!RigidBodyDynamics::Addons::read_luamodel(filename.c_str(), &model, verbose)) {
		cerr << "Loading of lua model failed!" << endl;
		return -1;
	}

	cout << "Model loading successful!" << endl;

	cout << "Degree of freedom overview:" << endl;
	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		cout << setfill(' ') << setw(3) << i - 1 << ": " << get_body_name (model, i) << "_" << get_dof_name (model.S[i]) << endl;
	}

	return 0;
}
