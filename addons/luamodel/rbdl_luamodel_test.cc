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

void print_dof_overview (const RigidBodyDynamics::Model &model) {
	cout << "Degree of freedom overview:" << endl;
	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		cout << setfill(' ') << setw(3) << i - 1 << ": " << get_body_name (model, i) << "_" << get_dof_name (model.S[i]) << endl;
	}
}

void print_hierarchy (const RigidBodyDynamics::Model &model, unsigned int body_index = 0, int indent = 0) {
	for (int j = 0; j < indent; j++)
		cout << "  ";

	if (body_index == 0) {
		cout << "BASE [fixed]" << endl;
		print_hierarchy (model, 1, 1);
		return;
	} else {
		cout << get_body_name (model, body_index);
	}

	// print the dofs
	cout << " [ ";

	while (model.mBodies[body_index].mMass == 0.) {
		if (model.mu[body_index].size() != 1) {
			cerr << endl << "Error: Cannot determine multi-dof joint as massless body with id " << body_index << " has more than 1 child." << endl;
			abort();
		}

		cout << get_dof_name(model.S[body_index]) << ", ";

		body_index = model.mu[body_index][0];
	}
	cout << get_dof_name(model.S[body_index]);

	cout << " ]" << endl;

	// print fixed children
	for (unsigned int fbody_index = 0; fbody_index < model.mFixedBodies.size(); fbody_index++) {
		if (model.mFixedBodies[fbody_index].mMovableParent == body_index) {
			for (int j = 0; j < indent; j++)
				cout << "  ";

			cout << model.GetBodyName(model.fixed_body_discriminator + fbody_index) << " [fixed]" << endl;
		}
	}

	unsigned int child_index = 0;
	for (child_index = 0; child_index < model.mu[body_index].size(); child_index++) {
		print_hierarchy (model, model.mu[body_index][child_index], indent + 1);
	}
}

void usage (const char* argv_0) {
	cerr << "Usage: " << argv_0 << "[-v] [-m] [-d] <model.lua>" << endl;
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
	if (!RigidBodyDynamics::Addons::read_luamodel(filename.c_str(), &model, verbose)) {
		cerr << "Loading of lua model failed!" << endl;
		return -1;
	}

	cout << "Model loading successful!" << endl;

	if (dof_overview)
		print_dof_overview(model);
	if (model_hierarchy)
		print_hierarchy(model);

	return 0;
}
