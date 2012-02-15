/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Joint.h"
#include "Body.h"
#include "Dynamics.h"
#include "Dynamics_experimental.h"
#include "Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;

void ForwardDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot,
		std::vector<SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	if (model.experimental_floating_base) {
		assert (0 && "Experimental floating base not supported");
	}

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i;

	// Copy state values from the input to the variables in model
	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);
	assert (model.qddot.size() == QDDot.size() + 1);
	assert (model.tau.size() == Tau.size() + 1);

	for (i = 0; i < Q.size(); i++) {
		model.q[i+1] = Q[i];
		model.qdot[i+1] = QDot[i];
		model.qddot[i+1] = QDDot[i];
		model.tau[i+1] = Tau[i];
	}

	// Reset the velocity of the root body
	model.v[0].setZero();

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints[i];
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, model.S[i], v_J, c_J, model.q[i], model.qdot[i]);
		LOG << "X_T (" << i << "):" << std::endl << model.X_T[i] << std::endl;

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda != 0)
			model.X_base[i] = model.X_lambda[i] * model.X_base.at(lambda);
		else
			model.X_base[i] = model.X_lambda[i];

		model.v[i] = model.X_lambda[i].apply( model.v.at(lambda)) + v_J;

		/*
		LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
		LOG << "v_J (" << i << "):" << std::endl << v_J << std::endl;
		LOG << "v_lambda" << i << ":" << std::endl << model.v.at(lambda) << std::endl;
		LOG << "X_base (" << i << "):" << std::endl << model.X_base[i] << std::endl;
		LOG << "X_lambda (" << i << "):" << std::endl << model.X_lambda[i] << std::endl;
		LOG << "SpatialVelocity (" << i << "): " << model.v[i] << std::endl;
		*/

		model.c[i] = c_J + crossm(model.v[i],v_J);
		model.IA[i] = model.mBodies[i].mSpatialInertia;

		model.pA[i] = crossf(model.v[i],model.IA[i] * model.v[i]);

		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero) {
			LOG << "External force (" << i << ") = " << model.X_base[i].toMatrixAdjoint() * (*f_ext)[i] << std::endl;
			model.pA[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
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
		// we can skip further processing if the joint is fixed
		if (model.mJoints[i].mJointType == JointTypeFixed)
			continue;

		model.U[i] = model.IA[i] * model.S[i];
		model.d[i] = model.S[i].dot(model.U[i]);
		model.u[i] = model.tau[i] - model.S[i].dot(model.pA[i]);

		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
			SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];
			SpatialTransform X_lambda = model.X_lambda[i];

			// note: X_lambda.inverse().spatial_adjoint() = X_lambda.transpose()
			model.IA[lambda] = model.IA[lambda] + X_lambda.toMatrixTranspose() * Ia * X_lambda.toMatrix();
			model.pA[lambda] = model.pA[lambda] + X_lambda.toMatrixTranspose() * pa;
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

	LOG << std::endl << "--- third loop ---" << std::endl;

	LOG << "spatial gravity = " << spatial_gravity.transpose() << std::endl;

	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		SpatialTransform X_lambda = model.X_lambda[i];

		model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];

		// we can skip further processing if the joint type is fixed
		if (model.mJoints[i].mJointType == JointTypeFixed) {
			model.qddot[i] = 0.;
			continue;
		}

		model.qddot[i] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "c[" << i << "] = " << model.c[i].transpose() << std::endl;
	}

	LOG << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "a[" << i << "] = " << model.a[i].transpose() << std::endl;
	}


	for (i = 1; i < model.mBodies.size(); i++) {
		QDDot[i - 1] = model.qddot[i];
	}
}

void ForwardDynamicsLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot,
		std::vector<SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	MatrixNd H = MatrixNd::Zero(model.dof_count, model.dof_count);
	VectorNd C = VectorNd::Zero(model.dof_count);

	// we set QDDot to zero to compute C properly with the InverseDynamics
	// method.
	QDDot.setZero();

	InverseDynamics (model, Q, QDot, QDDot, C, f_ext);
	CompositeRigidBodyAlgorithm (model, Q, H, false);

	LOG << "A = " << std::endl << H << std::endl;
	LOG << "b = " << std::endl << C * -1. + Tau << std::endl;

#ifndef RBDL_USE_SIMPLE_MATH
	QDDot = H.colPivHouseholderQr().solve (C * -1. + Tau);
#else
	bool solve_successful = LinSolveGaussElimPivot (H, C * -1. + Tau, QDDot);
	assert (solve_successful);
#endif

	LOG << "x = " << QDDot << std::endl;
}

void InverseDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot,
		VectorNd &Tau,
		std::vector<SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	if (model.experimental_floating_base) {
		assert (0 && !"InverseDynamics not supported for experimental floating base models!");
	}

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i;

	// Copy state values from the input to the variables in model
	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);
	assert (model.qddot.size() == QDDot.size() + 1);
	assert (model.tau.size() == Tau.size() + 1);

	for (i = 0; i < Q.size(); i++) {
		model.q[i+1] = Q[i];
		model.qdot[i+1] = QDot[i];
		model.qddot[i+1] = QDDot[i];
	}

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints[i];
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, model.S[i], v_J, c_J, model.q[i], model.qdot[i]);
		LOG << "X_T (" << i << "):" << std::endl << model.X_T[i] << std::endl;

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda == 0) {
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = v_J;
			model.a[i] = model.X_base[i].apply(spatial_gravity * -1.) + model.S[i] * model.qddot[i];
		}	else {
			model.X_base[i] = model.X_lambda[i] * model.X_base.at(lambda);
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
			model.c[i] = c_J + crossm(model.v[i],v_J);
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.S[i] * model.qddot[i] + model.c[i];
		}

		LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
		LOG << "v (" << i << "):" << std::endl << v_J << std::endl;
		LOG << "a (" << i << "):" << std::endl << v_J << std::endl;

		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i] + crossf(model.v[i],model.mBodies[i].mSpatialInertia * model.v[i]);
		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		model.tau[i] = model.S[i].dot(model.f[i]);
		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			model.f[lambda] = model.f[lambda] + model.X_lambda[i].toMatrixTranspose() * model.f[i];
		}
	}

	for (i = 0; i < Tau.size(); i++) {
		Tau[i] = model.tau[i + 1];
	}
}

void CompositeRigidBodyAlgorithm (Model& model, const VectorNd &Q, MatrixNd &H, bool update_kinematics) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	if (H.rows() != Q.size() || H.cols() != Q.size()) 
		H.resize(Q.size(), Q.size());

	if (update_kinematics)
		UpdateKinematicsCustom (model, &Q, NULL, NULL);

	H.setZero();

	unsigned int i;
	for (i = 1; i < model.mBodies.size(); i++) {
		model.Ic[i] = model.mBodies[i].mSpatialInertia;
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].toMatrixTranspose() * model.Ic[i] * model.X_lambda[i].toMatrix();
		}

		SpatialVector F = model.Ic[i] * model.S[i];
		H(i - 1, i - 1) = model.S[i].dot(F);
		unsigned int j = i;

		while (model.lambda[j] != 0) {
			F = model.X_lambda[j].toMatrixTranspose() * F;
			j = model.lambda[j];
			H(i - 1,j - 1) = F.dot(model.S[j]);
			H(j - 1,i - 1) = H(i - 1,j - 1);
		}
	}
}

} /* namespace RigidBodyDynamics */
