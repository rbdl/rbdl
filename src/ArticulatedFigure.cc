#include <iostream>

#include <assert.h>

#include "mathutils.h"
#include "ArticulatedFigure.h"

void ArticulatedFigure::Init() {
	Body root_body;

	mParentId.push_back(0);
	mBodies.push_back(root_body);

	SpatialMatrix root_inertia (SpatialMatrixIdentity);
	mSpatialInertia.push_back(root_inertia);

	Matrix3d orientation (Matrix3dIdentity);
	mBodyOrientation.push_back(orientation);

	Vector3d position (0, 0, 0);
	mBodyPosition.push_back(position);

	Vector3d velocity (0., 0., 0.);
	mBodyVelocity.push_back(velocity);
}

void ArticulatedFigure::AddBody (const unsigned int parent_id, const Joint &joint, const Body &body) {
	assert (mParentId.size() > 0);
	assert (joint.mJointType != JointTypeUndefined);

	mParentId.push_back(parent_id);

	// Compute Spatial Inertia of the body
	SpatialMatrix spatial_inertia = SpatialMatrixIdentity;
	mSpatialInertia.push_back(spatial_inertia);

	// Joint
	mJoints.push_back(joint);
	mJointType.push_back(joint.mJointType);

	// Build the joint transform spatial matrix
	SpatialMatrix joint_transform (SpatialMatrixIdentity);
	mJointTransform.push_back(joint_transform);

	// joint positions, velocities, etc.
	q.push_back(0.);
	qdot.push_back(0.);
	qddot.push_back(0.);
	tau.push_back(0.);

	// general body information
	mBodies.push_back(body);

	Matrix3d body_orientation (Matrix3dIdentity);
	mBodyOrientation.push_back(body_orientation);

	Vector3d body_position (Vector3dZero);
	mBodyPosition.push_back(body_position);

	Vector3d body_velocity (Vector3dZero);
	mBodyVelocity.push_back(body_velocity);
}

void ArticulatedFigure::CalcVelocities() {
}
