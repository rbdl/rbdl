#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"

#include "Dynamics_stdvec.h"
#include "Kinematics.h"

using namespace SpatialAlgebra;

/** \brief Computes forward dynamics for models with a fixed base
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internals joints (output)
 */
void ForwardDynamics (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		cmlVector &QDDot
		) {
	std::vector<double> Q_stdvec (Q.size());
	std::vector<double> QDot_stdvec (QDot.size());
	std::vector<double> QDDot_stdvec (QDDot.size());
	std::vector<double> Tau_stdvec (Tau.size());

	unsigned int i;
	for (i = 0; i < Q.size(); i++)
		Q_stdvec[i] = Q[i];

	for (i = 0; i < QDot.size(); i++)
		QDot_stdvec[i] = QDot[i];

	for (i = 0; i < QDDot.size(); i++)
		QDDot_stdvec[i] = QDDot[i];

	for (i = 0; i < Tau.size(); i++)
		Tau_stdvec[i] = Tau[i];

	ForwardDynamics (model, Q_stdvec, QDot_stdvec, Tau_stdvec, QDDot_stdvec);

	for (i = 0; i < QDDot.size(); i++)
		QDDot[i] = QDDot_stdvec[i];
}

/** \brief Computes forward dynamics for models with a floating base
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param X_B   transformation into base coordinates
 * \param v_B   velocity of the base (in base coordinates)
 * \param f_B   forces acting on the base (in base coordinates)
 * \param a_B   accelerations of the base (output, in base coordinates)
 * \param QDDot accelerations of the internals joints (output)
 */
void ForwardDynamicsFloatingBase (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		const SpatialMatrix &X_B,
		const SpatialVector &v_B,
		const SpatialVector &f_B,
		SpatialVector &a_B,
		cmlVector &QDDot
		) {
	std::vector<double> Q_stdvec (Q.size());
	std::vector<double> QDot_stdvec (QDot.size());
	std::vector<double> QDDot_stdvec (QDDot.size());
	std::vector<double> Tau_stdvec (Tau.size());

	unsigned int i;
	for (i = 0; i < Q.size(); i++)
		Q_stdvec[i] = Q[i];

	for (i = 0; i < QDot.size(); i++)
		QDot_stdvec[i] = QDot[i];

	for (i = 0; i < QDDot.size(); i++)
		QDDot_stdvec[i] = QDDot[i];

	for (i = 0; i < Tau.size(); i++)
		Tau_stdvec[i] = Tau[i];

	ForwardDynamicsFloatingBase (model, Q_stdvec, QDot_stdvec, Tau_stdvec, X_B, v_B, f_B, a_B, QDDot_stdvec);

	for (i = 0; i < QDDot.size(); i++)
		QDDot[i] = QDDot_stdvec[i];
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

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

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

		model.pA.at(i) = model.v.at(i).crossf() * model.IA.at(i) * model.v.at(i) - model.X_base[i].conjugate() * model.f_ext.at(i);
	}

// ClearLogOutput();

	LOG << "--- first loop ---" << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "X_base[" << i << "] = " << model.X_base[i] << std::endl;
	}

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
			model.IA[lambda] = model.IA[lambda] + X_lambda.conjugate() * Ia * X_lambda;
			model.pA[lambda] = model.pA[lambda] + X_lambda.conjugate() * pa;
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
			model.a[i] = X_lambda * spatial_gravity * (-1.) + model.c[i];
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

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

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

		model.pA.at(i) = model.v.at(i).crossf() * model.IA.at(i) * model.v.at(i) - model.X_base[i].conjugate() * model.f_ext.at(i);
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
		model.IA[lambda] = model.IA[lambda] + X_lambda.conjugate() * Ia * X_lambda;
		model.pA[lambda] = model.pA[lambda] + X_lambda.conjugate() * pa;
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

	model.a[0] += X_B * spatial_gravity;

	a_B = model.a[0];
}

void ForwardDynamicsContacts (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		cmlVector &QDDot
		) {

	// so far we only allow one constraint
	unsigned int contact_count = 0;
	ContactInfo contact_info;

	Model::ContactMapIter iter = model.mContactInfoMap.begin();
	while (iter != model.mContactInfoMap.end()) {
		contact_count = iter->second.size();

		if (contact_count == 1)
			contact_info = iter->second[0];

		iter++;
	}

	assert (contact_count == 1);

	// Steps to perform the contact algorithm suggested by Kokkevis and
	// Metaxas
	//
	// 1. Set external forces at P to zero and compute link accelerations and
	// compute a^0_P (the magnitude of the acceleration of P) and evaluate
	// C^0
	//
	// 2. Apply a unit force at P and compute a^1_P (the magnitude of the
	// resulting acceleration) and compute the net effect a^e_P of f^1.
	//
	// 3. Compute the required constraint force.
	
	// Step one, compute the standard forward dynamics without external
	// forces at P.
	
	// Step 1:
	
	// save current external forces:
	std::vector<SpatialVector> current_f_ext (model.f_ext);
	std::vector<SpatialVector> zero_f_ext (model.f_ext.size(), SpatialVector (0., 0., 0., 0., 0., 0.));

	model.f_ext = zero_f_ext;

	// compute forward dynamics with zero external forces
	cmlVector QDDot_zero_ext (QDDot);
	{
//		SUPPRESS_LOGGING;
		LOG << " -------- ZERO_EXT ------" << std::endl;
		ForwardDynamics (model, Q, QDot, Tau, QDDot_zero_ext);
	}

	// compute point accelerations
	Vector3d point_accel;
	{
		SUPPRESS_LOGGING;
		CalcPointAcceleration (model, Q, QDot, QDDot_zero_ext, contact_info.body_id, contact_info.point, point_accel);
	}

	LOG << "point_accel    = " << point_accel << std::endl;

	// evaluate a0 and C0
	double a0 = cml::dot(contact_info.normal,point_accel);
	double C0 = a0 - 0.;

	LOG << "a0 = " << a0 << std::endl;

	// Step 2:
	
	// Compute the test force
	SpatialVector test_force (0., 0., 0., contact_info.normal[0], contact_info.normal[1], contact_info.normal[2]);
	// transform the test force from the point coordinates to base
	// coordinates
	Matrix3d body_rotation = model.X_base[contact_info.body_id].get_rotation().transpose();
	Vector3d body_position = model.X_base[contact_info.body_id].get_translation() * -1.; 
	Vector3d contact_point_position = body_position + body_rotation * contact_info.point;

	SpatialVector test_force_base = Xtrans (contact_point_position).transpose() * test_force;

	LOG << "body_id         = " << contact_info.body_id << std::endl;
	LOG << "body_position   = " << body_position << std::endl;
	LOG << "point_position  = " << contact_point_position << std::endl; 
	LOG << "test_force_base = " << test_force_base << std::endl;

	// apply the test force
	model.f_ext[contact_info.body_id] = test_force_base;
	cmlVector QDDot_test_ext (QDDot);
	{
//		SUPPRESS_LOGGING;
		LOG << "-------- TEST_EXT ------" << std::endl;
		ForwardDynamics (model, Q, QDot, Tau, QDDot_test_ext);
	}

	// compute point accelerations after the test force
	Vector3d point_test_accel;
	{
		SUPPRESS_LOGGING;
		CalcPointAcceleration (model, Q, QDot, QDDot_zero_ext, contact_info.body_id, contact_info.point, point_test_accel);
	}

	LOG << "point_test_accel= " << point_test_accel << std::endl;
	// evaluate a0 and C0
	double ae = cml::dot(contact_info.normal,point_test_accel);

	assert (fabs (ae) > 1.0e-6);
	double fc = -C0 / ae;
	LOG << "ae = " << ae << std::endl;
	LOG << "fc = " << fc << std::endl;

	model.f_ext[contact_info.body_id] *= fc;
	{
	//	SUPPRESS_LOGGING;
		LOG << "-------- APPLY_EXT ------" << std::endl;
		ForwardDynamics (model, Q, QDot, Tau, QDDot);
	}
}


