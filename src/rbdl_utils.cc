#include "rbdl/rbdl_utils.h"

#include "rbdl/rbdl_math.h"
#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"

#include <sstream>
#include <iomanip>

namespace RigidBodyDynamics {

namespace Utils {

using namespace std;
using namespace Math;

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

RBDL_DLLAPI std::string GetModelDOFOverview (const Model &model) {
	stringstream result ("");

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		result << setfill(' ') << setw(3) << i - 1 << ": " << get_body_name(model, i) << "_" << get_dof_name (model.S[i]) << endl;
	}

	return result.str();
}

std::string print_hierarchy (const RigidBodyDynamics::Model &model, unsigned int body_index = 0, int indent = 0) {
	stringstream result ("");

	for (int j = 0; j < indent; j++)
		result << "  ";

	result << get_body_name (model, body_index);

	if (body_index > 0)
		result << " [ ";

	while (model.mBodies[body_index].mMass == 0.) {
		if (model.mu[body_index].size() == 0) {
			result << " end";
			break;
		} else if (model.mu[body_index].size() > 1) {
			cerr << endl << "Error: Cannot determine multi-dof joint as massless body with id " << body_index << " (name: " << model.GetBodyName(body_index) << ") has more than one child:" << endl;
			for (unsigned int ci = 0; ci < model.mu[body_index].size(); ci++) {
				cerr << "  id: " << model.mu[body_index][ci] << " name: " << model.GetBodyName(model.mu[body_index][ci]) << endl;
			}
			abort();
		}

		result << get_dof_name(model.S[body_index]) << ", ";

		body_index = model.mu[body_index][0];
	}

	if (body_index > 0)
		result << get_dof_name(model.S[body_index]) << " ]";
	result << endl;

	unsigned int child_index = 0;
	for (child_index = 0; child_index < model.mu[body_index].size(); child_index++) {
		result << print_hierarchy (model, model.mu[body_index][child_index], indent + 1);
	}

	// print fixed children
	for (unsigned int fbody_index = 0; fbody_index < model.mFixedBodies.size(); fbody_index++) {
		if (model.mFixedBodies[fbody_index].mMovableParent == body_index) {
			for (int j = 0; j < indent + 1; j++)
				result << "  ";

			result << model.GetBodyName(model.fixed_body_discriminator + fbody_index) << " [fixed]" << endl;
		}
	}


	return result.str();
}

RBDL_DLLAPI std::string GetModelHierarchy (const Model &model) {
	stringstream result ("");

	result << print_hierarchy (model);

	return result.str();
}

RBDL_DLLAPI std::string GetNamedBodyOriginsOverview (Model &model) {
	stringstream result ("");

	VectorNd Q (VectorNd::Zero(model.dof_count));
	UpdateKinematicsCustom (model, &Q, NULL, NULL);

	for (unsigned int body_id = 0; body_id < model.mBodies.size(); body_id++) {
		std::string body_name = model.GetBodyName (body_id);

		if (body_name.size() == 0) 
			continue;

		Vector3d position = CalcBodyToBaseCoordinates (model, Q, body_id, Vector3d (0., 0., 0.), false);

		result << body_name << ": " << position.transpose() << endl;
	}

	return result.str();
}

RBDL_DLLAPI void CalcCenterOfMass (Model &model, const Math::VectorNd &q, const Math::VectorNd &qdot, double &mass, Math::Vector3d &com, Math::Vector3d *com_velocity, bool update_kinematics) {
	if (update_kinematics)
		UpdateKinematicsCustom (model, &q, &qdot, NULL);

	for (size_t i = 1; i < model.mBodies.size(); i++) {
		model.Ic[i].createFromMatrix(model.mBodies[i].mSpatialInertia);
		model.hc[i] = model.Ic[i].toMatrix() * model.v[i];
	}

	SpatialRigidBodyInertia Itot (0., Vector3d (0., 0., 0.), Matrix3d::Zero(3,3));
	SpatialVector htot (SpatialVector::Zero(6));

	for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int lambda = model.lambda[i];

		if (lambda != 0) {
			model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].applyTranspose (model.Ic[i]);
			model.hc[lambda] = model.hc[lambda] + model.X_lambda[i].applyTranspose (model.hc[i]);
		} else {
			Itot = Itot + model.X_lambda[i].applyTranspose (model.Ic[i]);
			htot = htot + model.X_lambda[i].applyTranspose (model.hc[i]);
		}
	}

	mass = Itot.m;
	com = Itot.h / mass;
	LOG << "mass = " << mass << " com = " << com.transpose() << " htot = " << htot.transpose() << std::endl;

	if (com_velocity) 
		*com_velocity = Vector3d (htot[3] / mass, htot[4] / mass, htot[5] / mass);
}

RBDL_DLLAPI double CalcPotentialEnergy (Model &model, const Math::VectorNd &q, bool update_kinematics) {
	double mass;
	Vector3d com;
	CalcCenterOfMass (model, q, VectorNd::Zero (model.qdot_size), mass, com, NULL, update_kinematics);

	Vector3d g = - Vector3d (model.gravity[0], model.gravity[1], model.gravity[2]);
	LOG << "pot_energy: " << " mass = " << mass << " com = " << com.transpose() << std::endl;

	return mass * com.dot(g);
}

RBDL_DLLAPI double CalcKineticEnergy (Model &model, const Math::VectorNd &q, const Math::VectorNd &qdot, bool update_kinematics) {
	if (update_kinematics)
		UpdateKinematicsCustom (model, &q, &qdot, NULL);

	double result = 0.;

	for (size_t i = 1; i < model.mBodies.size(); i++) {
		result += 0.5 * model.v[i].transpose() * model.mBodies[i].mSpatialInertia * model.v[i];
	}
	return result;
}

RBDL_DLLAPI Vector3d CalcAngularMomentum (Model &model, const Math::VectorNd &q, const Math::VectorNd &qdot, bool update_kinematics) {
	if (update_kinematics)
		UpdateKinematicsCustom (model, &q, &qdot, NULL);

	for (size_t i = 1; i < model.mBodies.size(); i++) {
		model.Ic[i].createFromMatrix(model.mBodies[i].mSpatialInertia);
		model.hc[i] = model.Ic[i].toMatrix() * model.v[i];
	}

	SpatialVector htot (SpatialVector::Zero(6));

	for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int lambda = model.lambda[i];

		if (lambda != 0) {
			model.hc[lambda] = model.hc[lambda] + model.X_lambda[i].applyTranspose (model.hc[i]);
		} else {
			htot = htot + model.X_lambda[i].applyTranspose (model.hc[i]);
		}
	}

	double mass;
	Vector3d com;
	CalcCenterOfMass (model, q, qdot, mass, com, NULL, false);

	LOG << "com : " << com.transpose() << std::endl;
	LOG << "htot: " << htot.transpose() << std::endl;
	SpatialTransform X_to_COM (Xtrans (com));
	htot = X_to_COM.applyAdjoint (htot);
	LOG << "htot (at COM): " << htot.transpose() << std::endl;

	return Vector3d (htot[0], htot[1], htot[2]);
}

}
}
