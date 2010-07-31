#include <iostream>

#include <assert.h>

#include "mathutils.h"
#include "ArticulatedFigure.h"
#include "Logging.h"

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

	SpatialMatrix joint_transform;
	joint_transform.zero();

	mJointTransform.push_back(joint_transform);

	Joint root_joint;
	mJoints.push_back(root_joint);

	SpatialVector root_joint_axes;
	root_joint_axes.zero();
	mSpatialJointAxes.push_back (root_joint_axes);

	SpatialVector root_spatial_velocity;
	root_spatial_velocity.zero();

	mSpatialVelocities.push_back(root_spatial_velocity);
}

void ArticulatedFigure::AddBody (const unsigned int parent_id, const Joint &joint, const Body &body) {
	assert (mParentId.size() > 0);
	assert (joint.mJointType != JointTypeUndefined);

	mParentId.push_back(parent_id);

	// Compute Spatial Inertia of the body
	SpatialMatrix spatial_inertia = SpatialMatrixIdentity;
	mSpatialInertia.push_back(spatial_inertia);

	// Allocate memory for the spatial joint axes
	SpatialVector spatial_joint_axes;
	spatial_joint_axes.zero();
	if (joint.mJointType == JointTypeRevolute) {
		spatial_joint_axes[0] = joint.mJointAxis[0];
		spatial_joint_axes[1] = joint.mJointAxis[1];
		spatial_joint_axes[2] = joint.mJointAxis[2];
	}
	mSpatialJointAxes.push_back (spatial_joint_axes);

	// Allocate memory for the spatial velocity
	SpatialVector body_spatial_velocity;
	body_spatial_velocity.zero();
	mSpatialVelocities.push_back(body_spatial_velocity);

	// Joint
	mJoints.push_back(joint);
	mJointType.push_back(joint.mJointType);

	// Build the joint transform spatial matrix
	SpatialMatrix joint_transform (SpatialMatrixIdentity);
	Matrix3d joint_center_cross (VectorCrossMatrix(joint.mJointCenter));
	joint_center_cross *= -1.;
	SpatialMatrixSetSubmatrix(joint_transform, 1, 0, joint_center_cross);	

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

SpatialMatrix ArticulatedFigure::JointComputeTransform (const unsigned int joint_index) {
	Joint joint = mJoints.at(joint_index);

	assert (joint_index > 0);
	assert (joint.mJointType != JointTypeUndefined);

	SpatialMatrix result (SpatialMatrixIdentity);

	if (joint.mJointType == JointTypeRevolute) {
		assert (joint.mJointAxis == Vector3d(0., 0., 1.));

		Matrix3d rotation_matrix;
		cml::matrix_rotation_euler(rotation_matrix, q.at(joint_index - 1), 0., 0., cml::euler_order_zyx);

		SpatialMatrixSetSubmatrix (result, 0, 0, rotation_matrix);
		SpatialMatrixSetSubmatrix (result, 1, 1, rotation_matrix);
	}

	return result;
}

SpatialVector ArticulatedFigure::JointComputeVelocity (const unsigned int body_index) {
	Joint joint = mJoints[body_index];
	
	if (body_index == 0)
		return mSpatialVelocities[0];

	return mSpatialJointAxes.at(body_index) * qdot.at(body_index - 1);
}

void ArticulatedFigure::CalcVelocities() {
	SpatialVector result;
	result.zero();

	// Reset the velocity of the root body
	mSpatialVelocities.at(0).zero();

	unsigned int i;

	for (i = 1; i < mBodies.size(); i++) {
		SpatialMatrix X_j (JointComputeTransform(i));
		LOG << "X_j (" << i << "):" << std::endl << X_j << std::endl;
		SpatialVector v_j (JointComputeVelocity (i));
		LOG << "v_j: " << std::endl << v_j << std::endl;

		X_j = X_j * mJointTransform.at(i);
		LOG << "^i X i-1" << std::endl << X_j << std::endl;

		mSpatialVelocities.at(i) = X_j *mSpatialVelocities.at(i-1) + v_j;
		LOG << "SpatialVelocities (" << i << "): " << mSpatialVelocities.at(i) << std::endl;
	}
}

