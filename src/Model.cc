#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Body.h"
#include "Joint.h"

using namespace SpatialAlgebra;
using namespace RigidBodyDynamics;

void Model::Init() {
	experimental_floating_base = false;

	Body root_body;
	Joint root_joint;

	Vector3d zero_position (0., 0., 0.);
	SpatialVector zero_spatial (0., 0., 0., 0., 0., 0.);

	// structural information
	lambda.push_back(0.);
	mu.push_back(std::vector<unsigned int>(0));
	dof_count = 0;

	// state information
	q.resize(1);
	qdot.resize(1);
	qddot.resize(1);
	tau.resize(1);

	q.zero();
	qdot.zero();
	qddot.zero();
	tau.zero();

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

	d.resize(1);
	u.resize(1);

	d.zero();
	u.zero();

	f.push_back (zero_spatial);
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
	dof_count += 1;

	// state information
	q.resize (dof_count + 1);
	qdot.resize (dof_count + 1);
	qddot.resize (dof_count + 1);
	tau.resize (dof_count + 1);

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
	d.resize (dof_count + 1);
	u.resize (dof_count + 1);
	f.push_back (SpatialVector (0., 0., 0., 0., 0., 0.));
	f_ext.push_back (SpatialVector (0., 0., 0., 0., 0., 0.));

	// Bodies
	X_lambda.push_back(SpatialMatrixIdentity);
	X_base.push_back(SpatialMatrixIdentity);
	mBodies.push_back(body);

	return q.size() - 1;
}

unsigned int Model::SetFloatingBaseBody (const Body &body) {
	assert (lambda.size() >= 0);

	if (experimental_floating_base) {
		// we also will have 6 more degrees of freedom
		dof_count += 6;

		q.resize (dof_count + 1);
		qdot.resize (dof_count + 1);
		qddot.resize (dof_count + 1);
		tau.resize (dof_count + 1);

		// parent is the maximum possible value to mark it as having no parent
		lambda.at(0) = std::numeric_limits<unsigned int>::max();

		// Bodies
		X_lambda.at(0) = SpatialMatrixIdentity;
		X_base.at(0) = SpatialMatrixIdentity;
		mBodies.at(0) = body;
		return 0;
	} else {
		// Add five zero weight bodies and append the given body last. Order of
		// the degrees of freedom is:
		// 		tx ty tz rz ry rx
		//

		unsigned body_tx_id;
		Body body_tx (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
		Joint joint_tx (JointTypePrismatic, Vector3d (1., 0., 0.));
		body_tx_id = this->AddBody(0, Xtrans (Vector3d (0., 0., 0.)), joint_tx, body_tx);

		unsigned body_ty_id;
		Body body_ty (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
		Joint joint_ty (JointTypePrismatic, Vector3d (0., 1., 0.));
		body_ty_id = this->AddBody(body_tx_id, Xtrans (Vector3d (0., 0., 0.)), joint_ty, body_ty);

		unsigned body_tz_id;
		Body body_tz (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
		Joint joint_tz (JointTypePrismatic, Vector3d (0., 0., 1.));
		body_tz_id = this->AddBody(body_ty_id, Xtrans (Vector3d (0., 0., 0.)), joint_tz, body_tz);

		unsigned body_rz_id;
		Body body_rz (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
		Joint joint_rz (JointTypeRevolute, Vector3d (0., 0., 1.));
		body_rz_id = this->AddBody(body_tz_id, Xtrans (Vector3d (0., 0., 0.)), joint_rz, body_rz);

		unsigned body_ry_id;
		Body body_ry (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
		Joint joint_ry (JointTypeRevolute, Vector3d (0., 1., 0.));
		body_ry_id = this->AddBody(body_rz_id, Xtrans (Vector3d (0., 0., 0.)), joint_ry, body_ry);

		unsigned body_rx_id;
		Joint joint_rx (JointTypeRevolute, Vector3d (1., 0., 0.));
		body_rx_id = this->AddBody(body_ry_id, Xtrans (Vector3d (0., 0., 0.)), joint_rx, body);

		return body_rx_id;
	}
}

Vector3d Model::GetBodyOrigin (const unsigned int body_id) {
	if (experimental_floating_base) {
		assert (body_id <= mBodies.size());
	} else {
		assert (body_id > 0 && body_id < mBodies.size());
	}

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
	if (experimental_floating_base) {
		assert (body_id <= mBodies.size());
	} else {
		assert (body_id > 0);
		assert (body_id < mBodies.size());
	}

	// We use the information from the X_base vector. In the upper left 3x3
	// matrix contains the orientation as a 3x3 matrix which we are asking
	// for.
	return X_base[body_id].get_upper_left();
}

Vector3d Model::CalcBodyToBaseCoordinates (const unsigned int body_id, const Vector3d &body_point) {
	Matrix3d body_rotation = this->X_base[body_id].get_rotation().transpose();
	Vector3d body_position = GetBodyOrigin (body_id);

	return body_position + body_rotation * body_point;
}

Vector3d Model::CalcBaseToBodyCoordinates (const unsigned int body_id, const Vector3d &base_point) {
	Matrix3d body_rotation = this->X_base[body_id].get_rotation();
	Vector3d body_position = GetBodyOrigin (body_id);

	return body_rotation * base_point - body_rotation * body_position;
}
