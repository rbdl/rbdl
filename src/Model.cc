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
	mu.push_back(std::vector<unsigned int>(0));

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
	f_ext.push_back (zero_spatial);

	// Bodies
	X_lambda.push_back(SpatialMatrixIdentity);
	X_base.push_back(SpatialMatrixIdentity);

	mBodies.push_back(root_body);
}

unsigned int Model::AddBody (const unsigned int parent_id,
		const SpatialMatrix &joint_frame,
		const Joint &joint,
		const Body &body) {
	assert (lambda.size() > 0);
	assert (joint.mJointType != JointTypeUndefined);

	// structural information
	lambda.push_back(parent_id);
	mu.push_back(std::vector<unsigned int>(0));
	mu.at(parent_id).push_back(q.size());

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
	X_T.push_back(joint_frame);

	// Dynamic variables
	c.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	IA.push_back(body.mSpatialInertia);
	pA.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	U.push_back(SpatialVector(0., 0., 0., 0., 0., 0.));
	d.push_back(0.);
	u.push_back(0.);
	f_ext.push_back (SpatialVector (0., 0., 0., 0., 0., 0.));

	// Bodies
	X_lambda.push_back(SpatialMatrixIdentity);
	X_base.push_back(SpatialMatrixIdentity);
	mBodies.push_back(body);

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
}

Vector3d Model::GetBodyOrigin (const unsigned int body_id) {
	assert (body_id > 0 && body_id < mBodies.size());

	// We use the information from the X_base vector. In the upper left 3x3
	// matrix contains the orientation as a 3x3 matrix. The lower left 3x3
	// matrix is the matrix form of the cross product of the origin
	// coordinate vector, however in body coordinates. We have to rotate it
	// by its orientation to be able to retrieve the bodies origin
	// coordinates in base coordinates. 
	Matrix3d upper_left = X_base[body_id].get_upper_left().transpose();
	Matrix3d rx = upper_left * X_base[body_id].get_lower_left();

	return Vector3d (rx(1,2), -rx(0,2), rx(0,1));
}

Matrix3d Model::GetBodyWorldOrientation (const unsigned int body_id) {
	assert (body_id > 0 && body_id < mBodies.size());

	// We use the information from the X_base vector. In the upper left 3x3
	// matrix contains the orientation as a 3x3 matrix which we are asking
	// for.
	return X_base[body_id].get_upper_left();
}

