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
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;


static void bail(lua_State *L, const char *msg){
	std::cerr << msg << lua_tostring(L, -1) << endl;
	abort();
}

template<> Vector3d LuaTableNode::getDefault<Vector3d>(const Vector3d &default_value) { 
	Vector3d result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

		if (vector_table.length() != 3) {
			cerr << "LuaModel Error: invalid 3d vector!" << endl;
			abort();
		}
		
		result[0] = vector_table[1];
		result[1] = vector_table[2];
		result[2] = vector_table[3];
	}

	stackRestore();

	return result;
}

template<> SpatialVector LuaTableNode::getDefault<SpatialVector>(const SpatialVector &default_value) {
	SpatialVector result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);
		
		if (vector_table.length() != 6) {
			cerr << "LuaModel Error: invalid 6d vector!" << endl;
			abort();
		}
		result[0] = vector_table[1];
		result[1] = vector_table[2];
		result[2] = vector_table[3];
		result[3] = vector_table[4];
		result[4] = vector_table[5];
		result[5] = vector_table[6];
	}

	stackRestore();

	return result;
}

template<> Matrix3d LuaTableNode::getDefault<Matrix3d>(const Matrix3d &default_value) {
	Matrix3d result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);
		
		if (vector_table.length() != 3) {
			cerr << "LuaModel Error: invalid 3d matrix!" << endl;
			abort();
		}

		if (vector_table[1].length() != 3
				|| vector_table[2].length() != 3
				|| vector_table[3].length() != 3) {
			cerr << "LuaModel Error: invalid 3d matrix!" << endl;
			abort();
		}

		result(0,0) = vector_table[1][1];
		result(0,1) = vector_table[1][2];
		result(0,2) = vector_table[1][3];

		result(1,0) = vector_table[2][1];
		result(1,1) = vector_table[2][2];
		result(1,2) = vector_table[2][3];

		result(2,0) = vector_table[3][1];
		result(2,1) = vector_table[3][2];
		result(2,2) = vector_table[3][3];
	}

	stackRestore();

	return result;
}

template<> SpatialTransform LuaTableNode::getDefault<SpatialTransform>(const SpatialTransform &default_value) {
	SpatialTransform result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);
	
		result.r = vector_table["r"].getDefault<Vector3d>(Vector3d::Zero(3));
		result.E = vector_table["E"].getDefault<Matrix3d>(Matrix3d::Identity (3,3));
	}

	stackRestore();

	return result;
}

template<> Joint LuaTableNode::getDefault<Joint>(const Joint &default_value) {
	Joint result = default_value;
	
	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

		int joint_dofs = vector_table.length();

		if (joint_dofs == 1) {
			string dof_string = vector_table[1].getDefault<std::string>("");
			if (dof_string == "JointTypeSpherical") {
				stackRestore();
				return Joint(JointTypeSpherical);
			}
			if (dof_string == "JointTypeEulerZYX") {
				stackRestore();
				return Joint(JointTypeEulerZYX);
			}
		}

		if (joint_dofs > 0) {
			if (vector_table[1].length() != 6) {
				cerr << "LuaModel Error: invalid joint motion subspace description at " << this->keyStackToString() << endl;
				abort();
			}
		}

		switch (joint_dofs) {
			case 0: result = Joint(JointTypeFixed);
							break;
			case 1: result = Joint(vector_table[1].get<SpatialVector>());
							break;
			case 2: result = Joint(
									vector_table[1].get<SpatialVector>(),
									vector_table[2].get<SpatialVector>()
									);
							break;
			case 3: result = Joint(
									vector_table[1].get<SpatialVector>(),
									vector_table[2].get<SpatialVector>(),
									vector_table[3].get<SpatialVector>()
									);
							break;
			case 4: result = Joint(
									vector_table[1].get<SpatialVector>(),
									vector_table[2].get<SpatialVector>(),
									vector_table[3].get<SpatialVector>(),
									vector_table[4].get<SpatialVector>()
									);
							break;
			case 5: result = Joint(
									vector_table[1].get<SpatialVector>(),
									vector_table[2].get<SpatialVector>(),
									vector_table[3].get<SpatialVector>(),
									vector_table[4].get<SpatialVector>(),
									vector_table[5].get<SpatialVector>()
									);
							break;
			case 6: result = Joint(
									vector_table[1].get<SpatialVector>(),
									vector_table[2].get<SpatialVector>(),
									vector_table[3].get<SpatialVector>(),
									vector_table[4].get<SpatialVector>(),
									vector_table[5].get<SpatialVector>(),
									vector_table[6].get<SpatialVector>()
									);
							break;
			default:
							cerr << "Invalid number of DOFs for joint." << endl;
							abort();
		}
	}

	stackRestore();

	return result;
}

template<> Body LuaTableNode::getDefault<Body>(const Body &default_value) {
	Body result = default_value;

	if (stackQueryValue()) {
		LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

		double mass = 0.;
		Vector3d com (Vector3d::Zero(3));
		Matrix3d inertia (Matrix3d::Identity(3,3));

		mass = vector_table["mass"];
		com = vector_table["com"].getDefault<Vector3d>(com);
		inertia = vector_table["inertia"].getDefault<Matrix3d>(inertia);

		result = Body (mass, com, inertia);
	}

	stackRestore();

	return result;
}
namespace RigidBodyDynamics {

namespace Addons {

typedef map<string, unsigned int> StringIntMap;
StringIntMap body_table_id_map;

RBDL_DLLAPI
bool LuaModelReadFromFile (const char* filename, Model* model, bool verbose) {
	assert (model);

	LuaTable model_table = LuaTable::fromFile (filename);

	if (model_table["gravity"].exists()) {
		model->gravity = model_table["gravity"].get<Vector3d>();

		if (verbose)
			cout << "gravity = " << model->gravity.transpose() << endl;
	}

	int frame_count = model_table["frames"].length();

	body_table_id_map["ROOT"] = 0;

	for (int i = 1; i <= frame_count; i++) {
		if (!model_table["frames"][i]["parent"].exists()) {
			cerr << "Parent not defined for frame " << i << "." << endl;
			abort();
		}

		string body_name = model_table["frames"][i]["name"].getDefault<string>("");
		string parent_name = model_table["frames"][i]["parent"].get<string>();
		unsigned int parent_id = body_table_id_map[parent_name];

		SpatialTransform joint_frame = model_table["frames"][i]["joint_frame"].getDefault(SpatialTransform());
		Joint joint = model_table["frames"][i]["joint"].getDefault(Joint(JointTypeFixed));
		Body body = model_table["frames"][i]["body"].getDefault<Body>(Body());

		unsigned int body_id = model->AddBody (parent_id, joint_frame, joint, body, body_name);
		body_table_id_map[body_name] = body_id;

		if (verbose) {
			cout << "==== Added Body ====" << endl;
			cout << "  body_name  : " << body_name << endl;
			cout << "  body id    : " << body_id << endl;
			cout << "  parent_id  : " << parent_id << endl;
			cout << "  joint dofs : " << joint.mDoFCount << endl;
			for (unsigned int j = 0; j < joint.mDoFCount; j++) {
				cout << "    " << j << ": " << joint.mJointAxes[j].transpose() << endl;
			}
			cout << "  joint_frame: " << joint_frame << endl;
			cout << "  body inertia: " << endl << body.mSpatialInertia << endl;
		}
	}

	return true;
}

}

}
