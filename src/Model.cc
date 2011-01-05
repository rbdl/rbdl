#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Body.h"
#include "Joint.h"

using namespace SpatialAlgebra;

void Model::Init() {
	floating_base = false;

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
	a.push_back(zero_spatial);

	// Joints
	mJoints.push_back(root_joint);
	S.push_back (zero_spatial);
	X_T.push_back(SpatialMatrixIdentity);
	
	// Dynamic variables
	c.push_back(zero_spatial);
	IA.push_back(SpatialMatrixIdentity);
	pA.push_back(zero_spatial);
	U.push_back(zero_spatial);
	d.push_back(0.);
	u.push_back(0.);

	// Bodies
	X_lambda.push_back(SpatialMatrixIdentity);
	X_base.push_back(SpatialMatrixIdentity);

	mBodies.push_back(root_body);
	mBodyOrientation.push_back(Matrix3dIdentity);
}

unsigned int Model::AddBody (const unsigned int parent_id,
		const SpatialMatrix &joint_frame,
		const Joint &joint,
		const Body &body) {
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
	a.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));

	// Joints
	mJoints.push_back(joint);
	S.push_back (joint.mJointAxis);
	// we have to invert the transformation as it is later always used from the
	// child bodies perspective.
	X_T.push_back(joint_frame.inverse());

	// Dynamic variables
	c.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	IA.push_back(body.mSpatialInertia);
	pA.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	U.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	d.push_back(0.);
	u.push_back(0.);

	// Bodies
	X_lambda.push_back(SpatialMatrixIdentity);
	X_base.push_back(SpatialMatrixIdentity);
	mBodies.push_back(body);
	mBodyOrientation.push_back(Matrix3dIdentity);

	return q.size() - 1;
}

void Model::SetFloatingBody (const Body &body) {
	assert (lambda.size() >= 0);

	// mark the model such that we know it interprets body 0 as floating base
	floating_base = true;

	// parent is the maximum possible value to mark it as having no parent
	lambda.at(0) = std::numeric_limits<unsigned int>::max();

	// Bodies
	X_lambda.at(0) = SpatialMatrixIdentity;
	X_base.at(0) = SpatialMatrixIdentity;
	mBodies.at(0) = body;
	mBodyOrientation.at(0) = Matrix3dIdentity;
}

Vector3d Model::GetBodyOrigin (const unsigned int body_id) {
	assert (body_id > 0 && body_id < mBodies.size());
	Matrix3d rotation = X_base[body_id].get_rotation().transpose();
	return rotation * X_base[body_id].get_translation() * -1.;
}

