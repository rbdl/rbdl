#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"

using namespace SpatialAlgebra;

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
