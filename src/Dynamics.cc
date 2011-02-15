#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Contacts.h"
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

		model.pA.at(i) = model.v.at(i).crossf() * model.IA.at(i) * model.v.at(i);

		if (model.f_ext.at(i) != SpatialVectorZero) {
			LOG << "External force (" << i << ") = " << model.X_base[i].adjoint() * model.f_ext.at(i) << std::endl;
			model.pA.at(i) -= model.X_base[i].adjoint() * model.f_ext.at(i);
		}
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

			// note: X_lambda.inverse().adjoint() = X_lambda.transpose()
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

		model.pA.at(i) = model.v.at(i).crossf() * model.IA.at(i) * model.v.at(i) - model.X_base[i].transpose() * model.f_ext.at(i);
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

		// note: X_lambda.inverse().adjoint() = X_lambda.transpose()
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

	model.a[0] += X_B * spatial_gravity;

	a_B = model.a[0];
}

/*
void ComputeContactAccelerations (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		const std::vector<ContactInfo> &ContactData,
		cmlMatrix &Ae,
		cmlVector &C
		) {

	cmlVector a0 (contact_count);

	LOG << "-------- ZERO_EXT ------" << std::endl;
	cmlVector QDDot_zero_ext (QDot);
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, QDDot_zero_ext);
	}

	unsigned int ci;
	for (ci = 0; ci < contact_count; ci++) {
		ContactInfo contact_info = ContactData[ci];

		// compute point accelerations
		Vector3d point_accel;
		{
			SUPPRESS_LOGGING;
			CalcPointAcceleration (model, Q, QDot, QDDot_zero_ext, contact_info.body_id, contact_info.point, point_accel);
		}

		// evaluate a0 and C0
		double a0i = cml::dot(contact_info.normal,point_accel);

		a0[ci] = a0i;
		C0[ci] = - (a0i - contact_info.acceleration);
	}
}
*/

void ComputeContactForces (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		const std::vector<ContactInfo> &ContactData,
		std::vector<SpatialAlgebra::SpatialVector> &Fext
		) {
	LOG << "-------- ComputeContactForces ------" << std::endl;

	// so far we only allow one constraint
	unsigned int contact_count = ContactData.size();

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

	cmlMatrix Ae;
	Ae.resize(contact_count, contact_count);
	cmlVector C0 (contact_count);
	cmlVector a0 (contact_count);

	std::vector<SpatialVector> current_f_ext (model.f_ext);
	std::vector<SpatialVector> zero_f_ext (model.f_ext.size(), SpatialVector (0., 0., 0., 0., 0., 0.));
	Vector3d gravity_backup = model.gravity;

	model.f_ext = zero_f_ext;

	LOG << "-------- ZERO_EXT ------" << std::endl;
	cmlVector QDDot_zero_ext (QDot);
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, QDDot_zero_ext);
	}

	unsigned int ci;
	for (ci = 0; ci < contact_count; ci++) {
		ContactInfo contact_info = ContactData[ci];
		LOG << "ContactData[" << ci << "].acceleration = " << contact_info.acceleration << std::endl;

		// compute point accelerations
		Vector3d point_accel;
		{
			SUPPRESS_LOGGING;
			CalcPointAcceleration (model, Q, QDot, QDDot_zero_ext, contact_info.body_id, contact_info.point, point_accel);
		}

		// evaluate a0 and C0
		double a0i = cml::dot(contact_info.normal,point_accel);

		a0[ci] = a0i;
		C0[ci] = - (a0i - contact_info.acceleration);
	}

	// Step 2:
	std::vector<SpatialVector> test_forces (contact_count);

	unsigned int cj;
	// Compute the test force
	for (cj = 0; cj < contact_count; cj++) {
		ContactInfo contact_info = ContactData[cj];
		SpatialVector test_force (0., 0., 0., contact_info.normal[0], contact_info.normal[1], contact_info.normal[2]);

		// transform the test force from the point coordinates to base
		// coordinates
		Vector3d contact_point_position = model.GetBodyPointPosition(contact_info.body_id, contact_info.point);

		test_forces[cj] = Xtrans (contact_point_position * -1.).adjoint() * test_force;
		LOG << "body_id         = " << contact_info.body_id << std::endl;
		LOG << "test_force_base = " << test_forces[ci] << std::endl;
		// LOG << "test_force_body = " << Xtrans (model.GetBodyOrigin(contact_info.body_id)).adjoint() * test_force_base << std::endl;

		// apply the test force
		model.f_ext[contact_info.body_id] = test_forces[cj];
		cmlVector QDDot_test_ext (QDot);

		LOG << "-------- TEST_EXT ------" << std::endl;
		{
			SUPPRESS_LOGGING;
			ForwardDynamics (model, Q, QDot, Tau, QDDot_test_ext);
		}
		LOG << "QDDot_test_ext  = " << QDDot_test_ext << std::endl;

		for (ci = 0; ci < contact_count; ci++) {
			ContactInfo test_contact_info = ContactData[ci];
			// compute point accelerations after the test force
			Vector3d point_test_accel;
			{
				SUPPRESS_LOGGING;
				CalcPointAcceleration (model, Q, QDot, QDDot_test_ext, test_contact_info.body_id, test_contact_info.point, point_test_accel);
			}

			// acceleration due to the test force
			double a1j_i = cml::dot(test_contact_info.normal, point_test_accel);
			LOG << "test_accel a1j_i = " << a1j_i - Ae(ci,cj) << std::endl;
			Ae(ci,cj) = a1j_i - a0[ci];
			LOG << "updating (" << ci << ", " << cj << ")" << std::endl;
		}

		// clear the test force
		model.f_ext[contact_info.body_id].zero();
	}
	
	// solve the system!!!
	cmlVector u (contact_count);

	LOG << "Ae = " << std::endl << Ae << std::endl;
	LOG << "C0 = " << C0 << std::endl;
	LinSolveGaussElim (Ae, C0, u);

	LOG << "u = " << u << std::endl;

	// compute and apply the constraint forces to the system
	model.f_ext = current_f_ext;

	Fext = zero_f_ext;

	for (ci = 0; ci < contact_count; ci++) {
		ContactInfo contact_info = ContactData[ci];
	
		test_forces[ci] = test_forces[ci] * u[ci];
		// it is important to *add* the constraint force as multiple forces
		// might act on the same body
		Fext[contact_info.body_id] += test_forces[ci];
		LOG << "test_forces[" << ci << "] = " << test_forces[ci] << std::endl;
		LOG << "f_ext[" << contact_info.body_id << "] = " << Fext[contact_info.body_id] << std::endl;
	}
}

void ForwardDynamicsContacts (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		const std::vector<ContactInfo> &ContactData,
		cmlVector &QDDot
		) {
	LOG << "-------- ForwardDynamicsContacts ------" << std::endl;

	std::vector<SpatialVector> contact_f_ext (model.f_ext.size(), SpatialVector (0., 0., 0., 0., 0., 0.));

	ComputeContactForces (model, Q, QDot, Tau, ContactData, contact_f_ext);

	assert (contact_f_ext.size() == model.f_ext.size());

	unsigned int i;
	for (i = 0; i < model.f_ext.size(); i++) {
		model.f_ext[i] += contact_f_ext[i];
	}

	LOG << "-------- APPLY_EXT ------" << std::endl;
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, QDDot);
	}
}

void ComputeContactImpulses (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDotPre,
		const std::vector<ContactInfo> &ContactData,
		cmlVector &QDotPost
		) {
	std::vector<ContactInfo> ContactImpulseInfo;
	cmlVector QDotZero (QDotPre.size());
	QDotZero.zero();
	ContactInfo contact_info;
	Vector3d point_velocity;

	unsigned int i;
	for (i = 0; i < ContactData.size(); i++) {
		contact_info = ContactData[i];
		CalcPointVelocity (model, Q, QDotPre, contact_info.body_id, contact_info.point, point_velocity);

		Vector3d point_acceleration;
		{
			SUPPRESS_LOGGING;
			CalcPointAcceleration (model, Q, QDotPre, QDotPre, contact_info.body_id, contact_info.point, point_acceleration);
		}
		LOG << "point_acceleration = " << point_acceleration << std::endl;

		ContactInfo x_velocity_info (contact_info);
		x_velocity_info.normal.set (1., 0., 0.);
		x_velocity_info.acceleration = 0. - point_velocity[0];
//		x_velocity_info.acceleration = 0.03;
		ContactImpulseInfo.push_back (x_velocity_info);

		ContactInfo y_velocity_info (contact_info);
		y_velocity_info.normal.set (0., 1., 0.);
		y_velocity_info.acceleration = 0. - point_velocity[1];
//		y_velocity_info.acceleration = -0.14;
		ContactImpulseInfo.push_back (y_velocity_info);

		ContactInfo z_velocity_info (contact_info);
		z_velocity_info.normal.set (0., 0., 1.);
		z_velocity_info.acceleration = 0. - point_velocity[2];
//		z_velocity_info.acceleration = -0.31;
		ContactImpulseInfo.push_back (z_velocity_info);
	}

	std::vector<SpatialVector> contact_f_ext;
	cmlVector QDDotFext (QDotPre);
	QDDotFext.zero();
	cmlVector Tau_zero (QDDotFext);

	// for debugging
	cmlVector QDDotZeroFext (QDotPre.size());
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDotPre, Tau_zero, QDDotZeroFext);
	}

	contact_info = ContactData[0]; 
	{
		SUPPRESS_LOGGING;
		ComputeContactForces (model, Q, QDotPre, Tau_zero, ContactImpulseInfo, contact_f_ext);
	}

	LOG << "-------- APPLY_EXT ------" << std::endl;
	for (i = 0; i < model.f_ext.size(); i++) {
		model.f_ext[i] = contact_f_ext[i];
		LOG << "f_ext[" << i << "] = " << model.f_ext[i] << std::endl;
	}

	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDotPre, Tau_zero, QDDotFext);
	}
	LOG << "QDotPre       = " << QDotPre << std::endl;
	LOG << "QDDotZeroFext = " << QDDotZeroFext << std::endl;
	LOG << "QDDotFext     = " << QDDotFext << std::endl;

	cmlVector humans_impulse (QDotPre.size());
	humans_impulse[0] = 0.;
	humans_impulse[1] = -0.05;
	humans_impulse[2] = 0.1;
	humans_impulse[3] = 0.;
	humans_impulse[4] = -0.15;
	humans_impulse[5] = -0.15;

	humans_impulse = humans_impulse - QDotPre;
	LOG << "humans impulse= " << humans_impulse << std::endl;

	Vector3d point_accel;
	{
		SUPPRESS_LOGGING;
		CalcPointAcceleration (model, Q, QDotPre, QDDotFext, contact_info.body_id, contact_info.point, point_accel);
	}
	LOG << "Point Accel = " << point_accel << std::endl;
	{
		SUPPRESS_LOGGING;
		CalcPointAcceleration (model, Q, QDotPre, humans_impulse, contact_info.body_id, contact_info.point, point_accel);
	}
	LOG << "humans Point Accel = " << point_accel << std::endl;

	{
		SUPPRESS_LOGGING;
		CalcPointVelocity (model, Q, QDDotFext, contact_info.body_id, contact_info.point, point_velocity);
	}
	LOG << "Point Veloc = " << point_velocity << std::endl;

	{
		SUPPRESS_LOGGING;
		CalcPointVelocity (model, Q, humans_impulse, contact_info.body_id, contact_info.point, point_velocity);
	}
	LOG << "humans Point Veloc = " << point_velocity << std::endl;

	LOG << "------ SCND ITER ------" << std::endl;
	LOG << "QDotPre = " << QDotPre << std::endl;

	{
		SUPPRESS_LOGGING;
		CalcPointVelocity (model, Q, QDotPre, contact_info.body_id, contact_info.point, point_velocity);
	}
	LOG << "QDotPre Vel = " << point_velocity << std::endl;
	// Reset f_ext
	for (i = 0; i < model.f_ext.size(); i++) {
		model.f_ext[i] = SpatialVector (0., 0., 0., 0., 0., 0.);
	}

	ContactImpulseInfo.clear();

	for (i = 0; i < ContactData.size(); i++) {
		ContactInfo contact_info (ContactData[i]);
		Vector3d point_velocity_impulse;
		{
			SUPPRESS_LOGGING;
			CalcPointVelocity (model, Q, QDotPre, contact_info.body_id, contact_info.point, point_velocity);
			CalcPointVelocity (model, Q, QDDotFext, contact_info.body_id, contact_info.point, point_velocity_impulse);
		}

		LOG << "pv = " << point_velocity << std::endl;
		LOG << "pv_imp = " << point_velocity_impulse << std::endl;
		LOG << "accdest = " << (point_velocity) * (-2) - point_velocity_impulse << std::endl;

		ContactInfo x_velocity_info (contact_info);
		x_velocity_info.normal.set (1., 0., 0.);
		x_velocity_info.acceleration = - point_velocity[0] * 2  - point_velocity_impulse[0];
		ContactImpulseInfo.push_back (x_velocity_info);

		ContactInfo y_velocity_info (contact_info);
		y_velocity_info.normal.set (0., 1., 0.);
		y_velocity_info.acceleration = - point_velocity[1] * 2 - point_velocity_impulse[1];
		ContactImpulseInfo.push_back (y_velocity_info);

		ContactInfo z_velocity_info (contact_info);
		z_velocity_info.normal.set (0., 0., 1.);
		z_velocity_info.acceleration = - point_velocity[2] * 2 - point_velocity_impulse[2];
		ContactImpulseInfo.push_back (z_velocity_info);
	}

	{
		SUPPRESS_LOGGING;
		ComputeContactForces (model, Q, QDotPre, Tau_zero, ContactImpulseInfo, contact_f_ext);
	}
	
	LOG << "-------- APPLY_EXT ------" << std::endl;
	for (i = 0; i < model.f_ext.size(); i++) {
		model.f_ext[i] = contact_f_ext[i];
		LOG << "f_ext[" << i << "] = " << model.f_ext[i] << std::endl;
	}

	// Compute the joint space acceleration
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDotPre, Tau_zero, QDDotFext);
	}

	// Compute the cartesian acceleration
	{
		SUPPRESS_LOGGING;
		CalcPointAcceleration (model, Q, QDotPre, QDDotFext, contact_info.body_id, contact_info.point, point_accel);
	}

	// Compute the cartesian velocity
	{
		SUPPRESS_LOGGING;
		CalcPointVelocity (model, Q, QDDotFext, contact_info.body_id, contact_info.point, point_velocity);
	}

	LOG << "QDDotFext2 = " << QDDotFext << std::endl;
	LOG << "accelera 2 = " << point_accel << std::endl;
	LOG << "velo     2 = " << point_velocity << std::endl;
	QDotPost = QDotPre + QDDotFext;
}
