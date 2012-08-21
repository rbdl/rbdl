/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>
#include <string.h>

#include "rbdl_mathutils.h"
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

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i = 0;

	LOG << "Q          = " << Q.transpose() << std::endl;
	LOG << "QDot       = " << QDot.transpose() << std::endl;
	LOG << "Tau        = " << Tau.transpose() << std::endl;
	LOG << "---" << std::endl;

	// Reset the velocity of the root body
	model.v[0].setZero();

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, model.S[i], v_J, c_J, Q[i - 1], QDot[i - 1]);
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
		LOG << "X_lambda[" << i << "] = " << model.X_lambda[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "Xup[" << i << "] = " << model.X_lambda[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "v[" << i << "]   = " << model.v[i].transpose() << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "IA[" << i << "]  = " << model.IA[i] << std::endl;
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "pA[" << i << "]  = " << model.pA[i].transpose() << std::endl;
	}

	LOG << std::endl;

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		model.U[i] = model.IA[i] * model.S[i];
		model.d[i] = model.S[i].dot(model.U[i]);
		model.u[i] = Tau[i - 1] - model.S[i].dot(model.pA[i]);

		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
			SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];
#ifdef EIGEN_CORE_H
			model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
			model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
			model.IA[lambda] += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
			model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
		}
	}

//	ClearLogOutput();

	LOG << "--- second loop ---" << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "U[" << i << "]   = " << model.U[i].transpose() << std::endl;
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
		LOG << "pA[" << i << "]  = " << model.pA[i].transpose() << std::endl;
	}

	LOG << std::endl << "--- third loop ---" << std::endl;

	LOG << "spatial gravity = " << spatial_gravity.transpose() << std::endl;

	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		SpatialTransform X_lambda = model.X_lambda[i];

		model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];

		QDDot[i - 1] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
		model.a[i] = model.a[i] + model.S[i] * QDDot[i - 1];
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "c[" << i << "] = " << model.c[i].transpose() << std::endl;
	}

	LOG << std::endl;

	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "a[" << i << "] = " << model.a[i].transpose() << std::endl;
	}

	LOG << "QDDot = " << QDDot.transpose() << std::endl;
}

void ForwardDynamicsLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot,
		Math::LinearSolver linear_solver,
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
	switch (linear_solver) {
		case (LinearSolverPartialPivLU) :
			QDDot = H.partialPivLu().solve (C * -1. + Tau);
			break;
		case (LinearSolverColPivHouseholderQR) :
			QDDot = H.colPivHouseholderQr().solve (C * -1. + Tau);
			break;
		default:
			LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
			assert (0);
			break;
	}
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

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i;

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, model.S[i], v_J, c_J, Q[i - 1], QDot[i - 1]);

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda == 0) {
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = v_J;
			model.a[i] = model.X_base[i].apply(spatial_gravity * -1.) + model.S[i] * QDDot[i - 1];
		}	else {
			model.X_base[i] = model.X_lambda[i] * model.X_base.at(lambda);
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
			model.c[i] = c_J + crossm(model.v[i],v_J);
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.S[i] * QDDot[i - 1] + model.c[i];
		}

		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i] + crossf(model.v[i],model.mBodies[i].mSpatialInertia * model.v[i]);
		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	LOG << "-- first loop --" << std::endl;
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "X_base[" << i << "] = " << std::endl << model.X_base[i] << std::endl;
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "v[" << i << "] = " << model.v[i].transpose() << std::endl;
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "a[" << i << "] = " << model.a[i].transpose() << std::endl;
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "f[" << i << "] = " << model.f[i].transpose() << std::endl;
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		Tau[i - 1] = model.S[i].dot(model.f[i]);

		unsigned int lambda = model.lambda[i];

		if (lambda != 0) {
			model.f[lambda] = model.f[lambda] + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}

	LOG << "-- second loop" << std::endl;
	LOG << "Tau = " << Tau.transpose() << std::endl;
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "f[" << i << "] = " << model.f[i].transpose() << std::endl;
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "S[" << i << "] = " << model.S[i].transpose() << std::endl;
	}
}

void CompositeRigidBodyAlgorithm (Model& model, const VectorNd &Q, MatrixNd &H, bool update_kinematics) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	assert (H.rows() == model.dof_count && H.cols() == model.dof_count);

	if (update_kinematics)
		UpdateKinematicsCustom (model, &Q, NULL, NULL);

	H.setZero();

	unsigned int i;
	unsigned int dof_i = model.dof_count;

	for (i = 1; i < model.mBodies.size(); i++) {
		model.Ic[i].createFromMatrix(model.mBodies[i].mSpatialInertia);
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int lambda = model.lambda[i];

		if (lambda != 0) {
			model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].apply(model.Ic[i]);
		}

		dof_i = i - 1;

		SpatialVector F = model.Ic[i] * model.S[i];
		H(dof_i, dof_i) = model.S[i].dot(F);

		unsigned int j = i;
		unsigned int dof_j = dof_i;

		while (model.lambda[j] != 0) {
			F = model.X_lambda[j].applyTranspose(F);
			j = model.lambda[j];

			dof_j = j - 1;

			H(dof_i,dof_j) = F.dot(model.S[j]);
			H(dof_j,dof_i) = H(dof_i,dof_j);
		}
	}
}

} /* namespace RigidBodyDynamics */
