#include <iostream>

#include <assert.h>

#include "mathutils.h"
#include "ArticulatedFigure.h"
#include "Logging.h"

void Model::Init() {
	Body root_body;
	Joint root_joint;

	Vector3d zero_position (0., 0., 0.);
	SpatialVector zero_spatial (0., 0., 0., 0., 0., 0.);

	// structural information
	lambda.push_back(0.);

	// state information
	q.push_back(0.);
	qdot.push_back(0.);
	qddot.push_back(0.);
	tau.push_back(0.);
	v.push_back(zero_spatial);

	// Joints
	mJoints.push_back(root_joint);
	S.push_back (zero_spatial);
	
	// Dynamic variables
	mSpatialInertia.push_back(SpatialMatrixIdentity);

	// Bodies
	X_lambda.push_back(SpatialMatrixIdentity);
	X_base.push_back(SpatialMatrixIdentity);

	mBodies.push_back(root_body);
	mBodyOrientation.push_back(Matrix3dIdentity);
}

void Model::AddBody (const unsigned int parent_id, const Joint &joint, const Body &body) {
	assert (lambda.size() > 0);
	assert (joint.mJointType != JointTypeUndefined);

	// structural information
	lambda.push_back(parent_id);

	// state information
	q.push_back(0.);
	qdot.push_back(0.);
	qddot.push_back(0.);
	tau.push_back(0.);
	v.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));

	// Joints
	mJoints.push_back(joint);
	S.push_back (joint.mJointAxis);

	// Dynamic variables
	mSpatialInertia.push_back(body.mSpatiaInertia);

	// Bodies
	X_lambda.push_back(SpatialMatrixIdentity);
	X_base.push_back(SpatialMatrixIdentity);
	mBodies.push_back(body);
	mBodyOrientation.push_back(Matrix3dIdentity);
}

void jcalc (
		const Model &model,
		const unsigned int &joint_id,
		SpatialMatrix &XJ,
		SpatialVector &v_i,
		const double &q,
		const double &qdot
		) {
	// exception if we calculate it for the root body
	assert (joint_id > 0);

	Joint joint = model.mJoints[joint_id];

	// Calculate the spatial joint velocity
	v_i = model.S.at(joint_id);

	if (joint.mJointType == JointTypeFixed) {
		XJ = SpatialMatrixIdentity;
		v_i.zero();

		return;
	} else if (joint.mJointType == JointTypeRevolute) {
		// Only z-rotations supported so far!
		assert (
				joint.mJointAxis[0] == 0.
				&& joint.mJointAxis[1] == 0.
				&& joint.mJointAxis[2] == 1.
				);

		XJ = Xrotz (q);
	} else {
		// Only revolute joints supported so far
		assert (0);
	}

	v_i *= qdot;
}

void ForwardDynamics (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &Tau,
		std::vector<double> &QDDot
		) {
	SpatialVector result;
	result.zero();

	unsigned int i;
	
	// Copy state values from the input to the variables in model
	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);
	assert (model.qddot.size() == QDDot.size() + 1);
	assert (model.tau.size() == Tau.size() + 1);

	for (i = 0; i < Q.size(); i++) {
		model.q.at(i+1) = Q.at(i);
		model.qdot.at(i+1) = QDot.at(i);
		model.qddot.at(i+1) = QDDot.at(i);
		model.tau.at(i+1) = Tau.at(i);
	}

	// Reset the velocity of the root body
	model.v[0].zero();

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_j;
		SpatialVector v_j;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_j, v_j, model.q.at(i), model.qdot.at(i));
		SpatialMatrix X_T (joint.mJointTransform);
		LOG << "X_T (" << i << "):" << std::endl << X_T << std::endl;

		model.X_lambda.at(i) = X_j * X_T;

		if (lambda != 0)
			model.X_base.at(i) = model.X_lambda.at(i) * model.X_base.at(lambda);

		LOG << "X_j (" << i << "):" << std::endl << X_j << std::endl;
		LOG << "v_" << i << ":" << std::endl << v_j << std::endl;
		LOG << "v_lambda" << i << ":" << std::endl << model.v.at(lambda) << std::endl;
		LOG << "X_base (" << i << "):" << std::endl << model.X_base.at(i) << std::endl;
		LOG << "X_lambda (" << i << "):" << std::endl << model.X_lambda.at(i) << std::endl;

		model.v.at(i) = model.X_lambda.at(i) * model.v.at(lambda) + v_j;
		LOG << "SpatialVelocity (" << i << "): " << model.v.at(i) << std::endl;
	}
}

