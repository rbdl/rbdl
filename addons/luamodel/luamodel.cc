#include "rbdl/rbdl.h"
#include "luamodel.h"

#include <iostream>
#include <map>

#include "luatables.h"

extern "C"
{
   #include <lua.h>
   #include <lauxlib.h>
   #include <lualib.h>
}

using namespace std;

static void bail(lua_State *L, const char *msg){
	std::cerr << msg << lua_tostring(L, -1) << endl;
	abort();
}

namespace RigidBodyDynamics {

namespace Addons {

using namespace Math;

typedef map<string, unsigned int> StringIntMap;
StringIntMap body_table_id_map;

SpatialVector get_spatial_vector (lua_State *L, const string &path, int index = -1) {
	SpatialVector result (0., 0., 0., 0., 0., 0.);

	std::vector<double> array = ltGetDoubleArrayAt (L, path.c_str(), index);
	if (array.size() != 6) {
		cerr << "Invalid array size for spatial vector variable '" << path << "'." << endl;
		abort();
	}

	for (unsigned int i = 0; i < 6; i++) {
		result[i] = array[i];
	}

	return result;
}

Vector3d get_vector3d (lua_State *L, const string &path, int index = -1) {
	Vector3d result;

	std::vector<double> array = ltGetDoubleArrayAt (L, path.c_str(), index);
	if (array.size() != 3) {
		cerr << "Invalid array size for 3d vector variable '" << path << "'." << endl;
		abort();
	}

	for (unsigned int i = 0; i < 3; i++) {
		result[i] = array[i];
	}

	return result;
}

Matrix3d get_matrix3d (lua_State *L, const string &path) {
	Matrix3d result;

	// two ways either as flat array or as a lua table with three columns
	if (ltGetLengthAt (L, path.c_str(), -1) == 3) {
		Vector3d row = get_vector3d (L, path, 1);
		result(0,0) = row[0];
		result(0,1) = row[1];
		result(0,2) = row[2];

		row = get_vector3d (L, path, 2);
		result(1,0) = row[0];
		result(1,1) = row[1];
		result(1,2) = row[2];

		row = get_vector3d (L, path, 3);
		result(1,0) = row[0];
		result(1,1) = row[1];
		result(1,2) = row[2];

		return result;
	}

	std::vector<double> array = ltGetDoubleArrayAt (L, path.c_str(), -1);
	if (array.size() != 9) {
		cerr << "Invalid array size for 3d matrix variable '" << path << "'." << endl;
		abort();
	}

	for (unsigned int i = 0; i < 9; i++) {
		result.data()[i] = array[i];
	}

	return result;
}

bool read_frame_params (lua_State *L,
		const string &path,
		unsigned int &parent_id,
		SpatialTransform &joint_frame,
		Joint &joint,
		Body &body,
		std::string &body_name,
		bool verbose) {

	if (!ltIsExisting (L, (path + ".name").c_str())) {
		cerr << "Error: could not find required value '" << path << ".name'." << endl;
		return false;
	}
	body_name = ltGetString (L, (path + ".name").c_str());

	if (!ltIsExisting (L, (path + ".parent").c_str())) {
		cerr << "Error: could not find required value '" << path << ".parent' for body '" << body_name << "'." << endl;
		return false;
	}

	string parent_frame = ltGetString (L, (path + ".parent").c_str());

	StringIntMap::iterator parent_iter = body_table_id_map.find (parent_frame);
	if (parent_iter == body_table_id_map.end()) {
		cerr << "Error: could not find the parent frame for frame '" << body_name << "'!" << endl;
		return false;
	}
	parent_id = body_table_id_map[parent_frame];

	if (verbose) {
		cout << "frame name = " << body_name << endl;
		cout << "  parent = "  << parent_frame << endl;
		cout << "  parent_id = " << parent_id << endl;
	}

	// create the joint_frame
	if (!ltIsExisting(L, (path + ".joint_frame").c_str())) {
		joint_frame = SpatialTransform();
	} else {
		Vector3d r (0., 0., 0.);
		Matrix3d E (Matrix3d::Identity(3,3));
	
		if (ltIsExisting(L, (path + ".joint_frame.r").c_str())) {
			r = get_vector3d (L, path + ".joint_frame.r");
		}

		if (ltIsExisting(L, (path + ".joint_frame.E").c_str())) {
			E = get_matrix3d (L, path + ".joint_frame.E");
		}

		joint_frame = SpatialTransform (E, r);
	}

	if (verbose)
		cout << "  joint_frame = " << joint_frame << endl;

	// create the joint
	if (!ltIsExisting (L, (path + ".joint").c_str())) {
		joint = Joint(JointTypeFixed);
	} else {
		unsigned int joint_dofs = static_cast<unsigned int> (ltGetLength (L, (path + ".joint").c_str()));

		// special case: joint_dof specified as { X., X., X., X., X., X.}. In
		if (ltIsNumberAt (L, (path + ".joint").c_str(), 1)
				&& ltIsNumberAt (L, (path + ".joint").c_str(), 2)
				&& ltIsNumberAt (L, (path + ".joint").c_str(), 3)
				&& ltIsNumberAt (L, (path + ".joint").c_str(), 4)
				&& ltIsNumberAt (L, (path + ".joint").c_str(), 5)
				&& ltIsNumberAt (L, (path + ".joint").c_str(), 6) ) {
			joint = Joint (get_spatial_vector (L, path + ".joint"));
		} else {
			// otherwise: joint_dof specified as { { DOF1}, { DOF2}, ... }. In
			if (verbose)
				cout << "  joint_dofs   = " << joint_dofs << endl;

			switch (joint_dofs) {
				case 0: joint = Joint(JointTypeFixed);
								break;
				case 1: joint = Joint(get_spatial_vector(L, path + ".joint", 1));
								break;
				case 2: joint = Joint(
										get_spatial_vector(L, path + ".joint", 1),
										get_spatial_vector(L, path + ".joint", 2)
										);
								break;
				case 3: joint = Joint(
										get_spatial_vector(L, path + ".joint", 1),
										get_spatial_vector(L, path + ".joint", 2),
										get_spatial_vector(L, path + ".joint", 3)
										);
								break;
				case 4: joint = Joint(
										get_spatial_vector(L, path + ".joint", 1),
										get_spatial_vector(L, path + ".joint", 2),
										get_spatial_vector(L, path + ".joint", 3),
										get_spatial_vector(L, path + ".joint", 4)
										);
								break;
				case 5: joint = Joint(
										get_spatial_vector(L, path + ".joint", 1),
										get_spatial_vector(L, path + ".joint", 2),
										get_spatial_vector(L, path + ".joint", 3),
										get_spatial_vector(L, path + ".joint", 4),
										get_spatial_vector(L, path + ".joint", 5)
										);
								break;
				case 6: joint = Joint(
										get_spatial_vector(L, path + ".joint", 1),
										get_spatial_vector(L, path + ".joint", 2),
										get_spatial_vector(L, path + ".joint", 3),
										get_spatial_vector(L, path + ".joint", 4),
										get_spatial_vector(L, path + ".joint", 5),
										get_spatial_vector(L, path + ".joint", 6)
										);
								break;
				default:
								cerr << "Invalid number of DOFs for joint in frame '" << path << ".joint'" << endl;
								return false;
			}
		}
	}

	if (!ltIsExisting (L, (path + ".body").c_str())) {
		body = Body();
	} else {
		double mass = 0.;
		Vector3d com (0., 0., 0.);
		Matrix3d inertia (Matrix3d::Zero(3,3));

		if (ltIsExisting (L, (path + ".body.mass").c_str()) ) {
			mass = ltGetDouble (L, (path + ".body.mass").c_str());
		}

		if (ltIsExisting (L, (path + ".body.com").c_str() )) {
			com = get_vector3d (L, path + ".body.com");
		}

		if (ltIsExisting (L, (path + ".body.inertia").c_str() )) {
			inertia = get_matrix3d (L, path + ".body.inertia");
		}

		if (verbose) {
			cout << "  mass = " << mass << endl;
			cout << "  com = " << com << endl;
			cout << "  inertia = " << inertia << endl;
		}

		body = Body (mass, com, inertia);
	}

//	if (verbose)
//		cout << "  Body = " << endl << body.mSpatialInertia << endl;

	return true;
}

bool LuaModelReadFromFile (const char* filename, Model* model, bool verbose) {
	assert (model);

	body_table_id_map["ROOT"] = 0;

	lua_State *L;

	L = luaL_newstate();
	luaL_openlibs(L);

	if (luaL_loadfile(L, filename) || lua_pcall (L, 0, 1, 0)) {
		bail (L, "Error running file: ");
	}

	if (ltIsExisting (L, "gravity")) {
		model->gravity = get_vector3d (L, "gravity");

		if (verbose)
			cout << "gravity = " << model->gravity.transpose() << endl;
	}

	vector<string> frame_names = ltGetKeys (L, "frames");

	unsigned int parent_id;
	Math::SpatialTransform joint_frame;
	Joint joint;
	Body body;
	string body_name;

	for (unsigned int i = 0; i < frame_names.size(); i++) {
		string frame_path = string ("frames") + string (".") + string (frame_names[i]);

		if (!read_frame_params(L, frame_path,
					parent_id,
					joint_frame,
					joint,
					body,
					body_name,
					verbose
					)) {
			cerr << "Error reading frame " << frame_names[i] << "." << endl;
			return false;
		}

		if (verbose) {
			cout << "+ Adding Body " << endl;
			cout << "  parent_id  : " << parent_id << endl;
			cout << "  joint_frame: " << joint_frame << endl;
			cout << "  joint dofs : " << joint.mDoFCount << endl;
			for (unsigned int j = 0; j < joint.mDoFCount; j++) {
				cout << "    " << j << ": " << joint.mJointAxes[j].transpose() << endl;
			}
			cout << "  body inertia: " << endl << body.mSpatialInertia << endl;
			cout << "  body_name  : " << body_name << endl;

		}

		unsigned int body_id = model->AddBody (parent_id, joint_frame, joint, body, body_name);
		body_table_id_map[body_name] = body_id;

		if (verbose)
			cout << "  body id    : " << body_id << endl;
	}

	return true;
}

}

}
