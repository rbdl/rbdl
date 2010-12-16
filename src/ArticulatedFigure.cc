#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "ArticulatedFigure.h"
#include "Logging.h"

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

void jcalc (
		const Model &model,
		const unsigned int &joint_id,
		SpatialMatrix &XJ,
		SpatialVector &S,
		SpatialVector &v_J,
		SpatialVector &c_J,
		const double &q,
		const double &qdot
		) {
	// exception if we calculate it for the root body
	assert (joint_id > 0);

	Joint joint = model.mJoints[joint_id];

	// Calculate the spatial joint velocity
	v_J = model.S.at(joint_id);

	// Set the joint axis
	S = joint.mJointAxis;

	// the velocity dependent spatial acceleration is != 0 only for rhenomic
	// constraints (see RBDA, p. 55)
	c_J.zero();

	if (joint.mJointType == JointTypeFixed) {
		XJ = SpatialMatrixIdentity;
		v_J.zero();

		return;
	} else if (joint.mJointType == JointTypeRevolute) {
		// Only rotations around coordinate axes are supported so far!
		if (S == SpatialVector(1., 0., 0., 0., 0., 0.)) {
			XJ = Xrotx (q);
		} else if (S == SpatialVector(0., 1., 0., 0., 0., 0.)) {
			XJ = Xroty (q);
		} else if (S == SpatialVector(0., 0., 1., 0., 0., 0.)) {
			XJ = Xrotz (q);
		} else {
			assert (0 && !"Invalid joint axis!");
		}

	} else {
		// Only revolute joints supported so far
		assert (0);
	}

	v_J = S * qdot;
}

void ForwardDynamics (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &Tau,
		std::vector<double> &QDDot
		) {
	if (model.floating_base) {
		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase
		assert (0);

		// ForwardDynamicsFloatingBase(model, Q, QDot, Tau, QDDot);
		return;
	}

	SpatialVector result;
	result.zero();
	SpatialVector gravity (0., 0., 0., 0., -9.81, 0.);

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
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_J, model.S.at(i), v_J, c_J, model.q.at(i), model.qdot.at(i));
//		SpatialMatrix X_T (joint.mJointTransform);
		LOG << "X_T (" << i << "):" << std::endl << model.X_T.at(i) << std::endl;

		model.X_lambda.at(i) = X_J * model.X_T.at(i);

		if (lambda != 0)
			model.X_base.at(i) = model.X_lambda.at(i) * model.X_base.at(lambda);
		else
			model.X_base.at(i) = model.X_lambda.at(i);

		model.v.at(i) = model.X_lambda.at(i) * model.v.at(lambda) + v_J;

		/*
		LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
		LOG << "v_J (" << i << "):" << std::endl << v_J << std::endl;
		LOG << "v_lambda" << i << ":" << std::endl << model.v.at(lambda) << std::endl;
		LOG << "X_base (" << i << "):" << std::endl << model.X_base.at(i) << std::endl;
		LOG << "X_lambda (" << i << "):" << std::endl << model.X_lambda.at(i) << std::endl;
		LOG << "SpatialVelocity (" << i << "): " << model.v.at(i) << std::endl;
		*/

		model.c.at(i) = c_J + model.v.at(i).crossm() * v_J;
		model.IA.at(i) = model.mBodies.at(i).mSpatialInertia;

		// todo: external forces are ignored so far:
		model.pA.at(i) = model.v.at(i).crossf() * model.IA.at(i) * model.v.at(i);
	}

// ClearLogOutput();

	LOG << "--- first loop ---" << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "Xup[" << i << "] = " << model.X_lambda[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "v[" << i << "]   = " << model.v[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "IA[" << i << "]  = " << model.IA[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "pA[" << i << "]  = " << model.pA[i] << std::endl;
	}

	LOG << std::endl;

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		model.U[i] = model.IA[i] * model.S[i];
		model.d[i] = model.S[i] * model.U[i];
		model.u[i] = model.tau[i] - model.S[i] * model.pA[i];

		if (model.d[i] == 0. ) {
			std::cerr << "Warning d[i] == 0.!" << std::endl;
			continue;
		}

		unsigned int lambda = model.lambda.at(i);
		if (lambda != 0) {
			SpatialMatrix Ia = model.IA[i] - model.U[i].outer_product(model.U[i] / model.d[i]);
			SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];

			SpatialMatrix X_lambda = model.X_lambda[i];
			model.IA[lambda] = model.IA[lambda] + X_lambda.transpose() * Ia * X_lambda;
			model.pA[lambda] = model.pA[lambda] + X_lambda.transpose() * pa;
		}
	}

//	ClearLogOutput();

	LOG << "--- second loop ---" << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "U[" << i << "]   = " << model.U[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "d[" << i << "]   = " << model.d[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "u[" << i << "]   = " << model.u[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "IA[" << i << "]  = " << model.IA[i] << std::endl;
	}
	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "pA[" << i << "]  = " << model.pA[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		SpatialMatrix X_lambda = model.X_lambda[i];

		if (lambda == 0)
			model.a[i] = X_lambda * gravity * (-1.) + model.c[i];
		else {
			model.a[i] = X_lambda * model.a[lambda] + model.c[i];
		}

		model.qddot[i] = (1./model.d[i]) * (model.u[i] - model.U[i] * model.a[i]);
		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		QDDot[i - 1] = model.qddot[i];
	}
}

void ForwardDynamicsFloatingBase (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &Tau,
		const SpatialMatrix &X_B,
		const SpatialVector &v_B,
		const SpatialVector &f_B,
		SpatialVector &a_B,
		std::vector<double> &QDDot
		)
{
	SpatialVector result;
	result.zero();

	SpatialVector gravity (0., 0., 0., 0., -9.81, 0.);

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
	model.v[0] = v_B;
	model.X_lambda[0] = X_B;
	model.X_base[0] = X_B;

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_J, model.S.at(i), v_J, c_J, model.q.at(i), model.qdot.at(i));
//		SpatialMatrix X_T (joint.mJointTransform);
//		LOG << "X_T (" << i << "):" << std::endl << model.X_T.at(i) << std::endl;

		model.X_lambda.at(i) = X_J * model.X_T.at(i);

		if (lambda != 0) 
			model.X_base.at(i) = model.X_lambda.at(i) * model.X_base.at(lambda);

		model.v.at(i) = model.X_lambda.at(i) * model.v.at(lambda) + v_J;

		/*
		LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
		LOG << "v_J (" << i << "):" << std::endl << v_J << std::endl;
		LOG << "v_lambda" << i << ":" << std::endl << model.v.at(lambda) << std::endl;
		LOG << "X_base (" << i << "):" << std::endl << model.X_base.at(i) << std::endl;
		LOG << "X_lambda (" << i << "):" << std::endl << model.X_lambda.at(i) << std::endl;
		LOG << "SpatialVelocity (" << i << "): " << model.v.at(i) << std::endl;
		*/

		model.c.at(i) = c_J + model.v.at(i).crossm() * v_J;
		model.IA.at(i) = model.mBodies.at(i).mSpatialInertia;

		// todo: external forces are ignored so far:
		model.pA.at(i) = model.v.at(i).crossf() * model.IA.at(i) * model.v.at(i);
	}

// ClearLogOutput();

	model.IA[0] = model.mBodies[0].mSpatialInertia;
	model.pA[0] = v_B.crossf() * model.IA[0] * v_B - f_B;

	LOG << "--- first loop ---" << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "Xup[" << i << "] = " << model.X_lambda[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "v[" << i << "]   = " << model.v[i] << std::endl;
	}

	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "IA[" << i << "]  = " << model.IA[i] << std::endl;
	}

	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "pA[" << i << "]  = " << model.pA[i] << std::endl;
	}

	LOG << std::endl;

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		model.U[i] = model.IA[i] * model.S[i];
		model.d[i] = model.S[i] * model.U[i];
		model.u[i] = model.tau[i] - model.S[i] * model.pA[i];

		if (model.d[i] == 0. ) {
			std::cerr << "Warning d[i] == 0.!" << std::endl;
			continue;
		}

		unsigned int lambda = model.lambda.at(i);
		SpatialMatrix Ia = model.IA[i] - model.U[i].outer_product(model.U[i] / model.d[i]);
		SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];

		SpatialMatrix X_lambda = model.X_lambda[i];
		model.IA[lambda] = model.IA[lambda] + X_lambda.transpose() * Ia * X_lambda;
		model.pA[lambda] = model.pA[lambda] + X_lambda.transpose() * pa;
	}

//	ClearLogOutput();
	model.a[0] = SpatialLinSolve(model.IA[0], model.pA[0]) * 1.;

	LOG << "--- second loop ---" << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "U[" << i << "]   = " << model.U[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "d[" << i << "]   = " << model.d[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "u[" << i << "]   = " << model.u[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "IA[" << i << "]  = " << model.IA[i] << std::endl;
	}
	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "pA[" << i << "]  = " << model.pA[i] << std::endl;
	}

	model.a[0] = SpatialLinSolve (model.IA[0], model.pA[0]) * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		SpatialMatrix X_lambda = model.X_lambda[i];

		model.a[i] = X_lambda * model.a[lambda] + model.c[i];
		model.qddot[i] = (1./model.d[i]) * (model.u[i] - model.U[i] * model.a[i]);
		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		QDDot[i - 1] = model.qddot[i];
	}

	model.a[0] += X_B * gravity;

	a_B = model.a[0];
}

void CalcPointVelocity (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_velocity
	) {
	if (model.floating_base) {
		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase
		assert (0);

		// ForwardDynamicsFloatingBase(model, Q, QDot, Tau, QDDot);
		return;
	}

	unsigned int i;
	
	// Copy state values from the input to the variables in model
	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);

	for (i = 0; i < Q.size(); i++) {
		model.q.at(i+1) = Q.at(i);
		model.qdot.at(i+1) = QDot.at(i);
	}

	// Reset the velocity of the root body
	model.v[0].zero();

	// this will contain the global velocities of the bodies
	std::vector<SpatialVector> global_velocities (model.mBodies.size() + 1, SpatialVector(0., 0., 0., 0., 0., 0.));

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_J, model.S.at(i), v_J, c_J, model.q.at(i), model.qdot.at(i));
		LOG << "q(" << i << "):" << model.q.at(i) << std::endl;

		model.X_lambda.at(i) = X_J * model.X_T.at(i);

		if (lambda != 0)
			model.X_base.at(i) = model.X_lambda.at(i) * model.X_base.at(lambda);
		else
			model.X_base.at(i) = model.X_lambda.at(i);

		LOG << "X_J (" << i << "):" << X_J << std::endl;
		LOG << "v_J (" << i << "):" << v_J << std::endl;
		LOG << "X_base (" << i << "):" << model.X_base.at(i) << std::endl;
		model.v.at(i) = model.X_lambda.at(i) * model.v.at(lambda) + v_J;
		global_velocities.at(i) = global_velocities.at(lambda) + model.X_base.at(i).inverse() * v_J;
		LOG << "^0v (" << i << "): " << global_velocities.at(i) << std::endl;
	}

	LOG << std::endl;

	SpatialVector body_velocity (model.v[body_id]);
	body_velocity = global_velocities.at(body_id);
	Vector3d body_rot_velocity (body_velocity[0], body_velocity[1], body_velocity[2]);
	Vector3d body_lin_velocity (body_velocity[3], body_velocity[4], body_velocity[5]);

	SpatialMatrix X_body (model.X_base[body_id]);
	Matrix3d body_rotation;
	Vector3d body_translation;

	// we now compute the transformation from the local to the global frame of
	// the body. We split this up into the translation and rotation of the body
	
	// the rotation is the transpose of the rotation part (upper left) of
	// X_base[i].
	body_rotation = model.X_base[body_id].get_rotation().transpose();
	// the translation is the bottom left part which still has to be transformed
	// into a global translation
	body_translation = body_rotation * model.X_base[body_id].get_translation() * -1.;

	LOG << "body_index   = " << body_id << std::endl;
	LOG << "global_velo  = " << global_velocities.at(body_id) << std::endl;
	LOG << "body_transf  = " << model.X_base[body_id] << std::endl;
	LOG << "body_rotation= " << std::endl << body_rotation << std::endl;
	LOG << "body_tranlat = " << body_translation << std::endl;
	LOG << "body_rot_vel = " << body_rot_velocity << std::endl;
	LOG << "body_lin_vel = " << body_lin_velocity << std::endl;

	Vector3d point_abs_pos = body_translation + body_rotation * point_position;
	LOG << "point_abs_ps = " << point_abs_pos << std::endl;
	point_velocity = body_lin_velocity + cross (body_rot_velocity, point_abs_pos);

	LOG << "point_vel    = " << point_velocity << std::endl;
}

void CalcPointAcceleration (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_acceleration
		)
{
	if (model.floating_base) {
		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase
		assert (0);

		// ForwardDynamicsFloatingBase(model, Q, QDot, Tau, QDDot);
		return;
	}

	unsigned int i;
	
	// Copy state values from the input to the variables in model
	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);
	assert (model.qddot.size() == QDDot.size() + 1);

	for (i = 0; i < Q.size(); i++) {
		model.q.at(i+1) = Q.at(i);
		model.qdot.at(i+1) = QDot.at(i);
		model.qddot.at(i+1) = QDDot.at(i);
	}

	// Reset the velocity of the root body
	model.v[0].zero();
	model.a[0].zero();

	// this will contain the global accelerations of the bodies
	std::vector<SpatialVector> global_accelerations (
			model.mBodies.size() + 1,
			SpatialVector(0., 0., 0., 0., 0., 0.)
			);
	
	// this will contain the global velocities of the bodies
	std::vector<SpatialVector> global_velocities (
			model.mBodies.size() + 1,
			SpatialVector(0., 0., 0., 0., 0., 0.)
			);

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_J, model.S.at(i), v_J, c_J, model.q.at(i), model.qdot.at(i));
		LOG << "q(" << i << "):" << model.q.at(i) << std::endl;

		model.X_lambda.at(i) = X_J * model.X_T.at(i);

		if (lambda != 0)
			model.X_base.at(i) = model.X_lambda.at(i) * model.X_base.at(lambda);
		else
			model.X_base.at(i) = model.X_lambda.at(i);

		LOG << "X_J (" << i << "):" << X_J << std::endl;
		LOG << "v_J (" << i << "):" << v_J << std::endl;
		LOG << "X_base (" << i << "):" << model.X_base.at(i) << std::endl;
		model.v.at(i) = model.X_lambda.at(i) * model.v.at(lambda) + v_J;

		// we need to compute the global velocities as they may contribute to the
		// acceleration (e.g. in rotational motions where there is always an
		// acceleration towards the center
		global_velocities.at(i) = global_velocities.at(lambda) + model.X_base.at(i).inverse() * v_J;
		LOG << "^0v (" << i << "): " << global_velocities.at(i) << std::endl;

		// v_J = S_i * qdot
		global_accelerations.at(i) = global_accelerations.at(lambda) + model.X_base.at(i).inverse() * model.S.at(i) * model.qddot[i];
		LOG << "^0a (" << i << "): " << global_accelerations.at(i) << std::endl;

		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	LOG << std::endl;

	SpatialMatrix X_body (model.X_base[body_id]);
	Matrix3d body_rotation;
	Vector3d body_translation;

	// we now compute the transformation from the local to the global frame of
	// the body. We split this up into the translation and rotation of the body
	
	// the rotation is the transpose of the rotation part (upper left) of
	// X_base[i].
	body_rotation = model.X_base[body_id].get_rotation().transpose();
	// the translation is the bottom left part which still has to be transformed
	// into a global translation
	body_translation = body_rotation * model.X_base[body_id].get_translation() * -1.;

	LOG << "body_transf  = " << model.X_base[body_id] << std::endl;
	LOG << "body_rotation= " << std::endl << body_rotation << std::endl;
	LOG << "body_tranlat = " << body_translation << std::endl;

	// computation of the global position of the point
	Vector3d point_abs_pos = body_translation + body_rotation * point_position;
	LOG << "point_abs_ps = " << point_abs_pos << std::endl;

	// here we split the spatial velocity into its 3D counterparts
	SpatialVector body_velocity (model.v[body_id]);
	body_velocity = global_velocities.at(body_id);
	Vector3d body_rot_velocity (
			body_velocity[0],
			body_velocity[1],
			body_velocity[2]
			);
	Vector3d body_lin_velocity (
			body_velocity[3],
			body_velocity[4],
			body_velocity[5]
			);

	// computation of the global linear and rotational velocity
	Vector3d point_global_velocity = body_lin_velocity + cross (body_rot_velocity, point_abs_pos);
	LOG << "point_gl_vel = " << point_global_velocity << std::endl;

	// here we split the spatial acceleration into its 3D counterparts
	SpatialVector body_acceleration (model.a[body_id]);
	LOG << "body_acceleration = " << body_acceleration << std::endl;
	body_acceleration = global_accelerations.at(body_id);

	LOG << "global_acce  = " << global_accelerations.at(body_id) << std::endl;

	Vector3d body_rot_acceleration (
			body_acceleration[0],
			body_acceleration[1],
			body_acceleration[2]
			);
	Vector3d body_lin_acceleration (
			body_acceleration[3], 
			body_acceleration[4], 
			body_acceleration[5]
			);

	LOG << "body_rot_acc = " << body_rot_acceleration << std::endl;
	LOG << "body_lin_acc = " << body_lin_acceleration << std::endl;

	SpatialVector rdot (0., 0., 0.,
			point_global_velocity[0],
			point_global_velocity[1],
			point_global_velocity[2]
			);

	LOG << "rdot = " << rdot << std::endl;
	LOG << "body_velocity = " << body_velocity << std::endl;
	SpatialVector spatial_accel = body_acceleration + rdot.crossm() * body_velocity;
	point_acceleration = body_lin_acceleration + cross (body_rot_acceleration, point_global_velocity);

	Vector3d screw_accel (
			spatial_accel[3],
			spatial_accel[4],
			spatial_accel[5]
			);

	LOG << "point_accel_std    = " << point_acceleration << std::endl;
	LOG << "point_accel_scrw   = " << screw_accel << std::endl;

	SpatialVector res = body_acceleration - rdot.crossm() * body_velocity;

	LOG << "res = " << res << std::endl;

	point_acceleration.set (res[3], res[4], res[5]);

	Vector3d std_accel = cross (body_rot_acceleration, point_abs_pos);
	point_acceleration += std_accel;

	LOG << "returning " << point_acceleration <<  std::endl;
}


