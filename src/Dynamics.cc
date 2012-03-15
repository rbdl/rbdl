/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>

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

	if (model.experimental_floating_base) {
		assert (0 && "Experimental floating base not supported");
	}

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i = 0;

	// Copy state values from the input to the variables in model
	CopyDofVectorToModelStateVector (model, model.q, Q);
	CopyDofVectorToModelStateVector (model, model.qdot, QDot);
	CopyDofVectorToModelStateVector (model, model.tau, Tau);

	LOG << "Q          = " << Q.transpose() << std::endl;
	LOG << "QDot       = " << QDot.transpose() << std::endl;
	LOG << "Tau        = " << Tau.transpose() << std::endl;
	LOG << "---" << std::endl;
	LOG << "model.q    = " << model.q.transpose() << std::endl;
	LOG << "model.qdot = " << model.qdot.transpose() << std::endl;
	LOG << "model.tau  = " << model.tau.transpose() << std::endl;
	LOG << "---" << std::endl;

	// Reset the velocity of the root body
	model.v[0].setZero();

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
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
		model.u[i] = model.tau[i] - model.S[i].dot(model.pA[i]);

		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			SpatialTransform X_lambda = model.X_lambda[i];

			// for fixed joints we simply transform the spatial inertia and the
			// spatial bias force to the parent body
			if (model.mJoints[i].mJointType == JointTypeFixed) {
				model.IA[lambda] = model.IA[lambda] + X_lambda.toMatrixTranspose() * model.IA[i] * X_lambda.toMatrix();
				model.pA[lambda] = model.pA[lambda] + X_lambda.toMatrixTranspose() * model.pA[i];
				continue;
			}

			SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
			SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];

			// note: X_lambda.inverse().spatial_adjoint() = X_lambda.transpose()
			model.IA[lambda] = model.IA[lambda] + X_lambda.toMatrixTranspose() * Ia * X_lambda.toMatrix();
			model.pA[lambda] = model.pA[lambda] + X_lambda.toMatrixTranspose() * pa;
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

	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "a[" << i << "] = " << model.a[i].transpose() << std::endl;
	}

	LOG << "qddot = " << model.qddot.transpose() << std::endl;

	// copy back values
	CopyModelStateVectorToDofVector (model, QDDot, model.qddot);
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
	CopyDofVectorToModelStateVector (model, model.q, Q);
	CopyDofVectorToModelStateVector (model, model.qdot, QDot);
	CopyDofVectorToModelStateVector (model, model.qddot, QDDot);

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, model.S[i], v_J, c_J, model.q[i], model.qdot[i]);

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

		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i] + crossf(model.v[i],model.mBodies[i].mSpatialInertia * model.v[i]);
		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];

		if (model.mJoints[i].mJointType == JointTypeFixed) {
			model.f[lambda].setZero();
		}
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
		model.tau[i] = model.S[i].dot(model.f[i]);

		unsigned int lambda = model.lambda[i];

		if (lambda != 0) {
			model.f[lambda] = model.f[lambda] + model.X_lambda[i].toMatrixTranspose() * model.f[i];
		}
	}

	LOG << "-- second loop" << std::endl;
	LOG << "tau = " << model.tau.transpose() << std::endl;
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "f[" << i << "] = " << model.f[i].transpose() << std::endl;
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "S[" << i << "] = " << model.S[i].transpose() << std::endl;
	}

	// copy back values
	CopyModelStateVectorToDofVector (model, Tau, model.tau);
}

void CompositeRigidBodyAlgorithm (Model& model, const VectorNd &Q, MatrixNd &H, bool update_kinematics) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	assert (H.rows() == model.dof_count && H.cols() == model.dof_count);

	if (update_kinematics)
		UpdateKinematicsCustom (model, &Q, NULL, NULL);

	H.setZero();

	std::vector<unsigned int> fixed_joints_count (model.mBodies.size(), 0.);

	unsigned int i;
	for (i = 1; i < model.mBodies.size(); i++) {
		if (model.mJoints[i].mJointType == JointTypeFixed)
			fixed_joints_count[i] = fixed_joints_count[i - 1] + 1;
		else
			fixed_joints_count[i] = fixed_joints_count[i - 1];
		model.Ic[i] = model.mBodies[i].mSpatialInertia;
	}

	LOG << "-- initialization --" << std::endl;
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "Ic[" << i << "] = " << std::endl << model.Ic[i] << std::endl;
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "mFixedJointCount[" << i << "] = " << std::endl << model.mFixedJointCount[i] << std::endl;
	}

	unsigned int dof_i = model.dof_count;

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int lambda = model.lambda[i];

		if (lambda != 0) {
			model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].toMatrixTranspose() * model.Ic[i] * model.X_lambda[i].toMatrix();
		}

		if (model.mJoints[i].mJointType == JointTypeFixed) {
			continue;
		}

		dof_i = i - model.mFixedJointCount[i] - 1;

		SpatialVector F = model.Ic[i] * model.S[i];
		H(dof_i, dof_i) = model.S[i].dot(F);

		unsigned int j = i;
		unsigned int dof_j = dof_i;

		while (model.lambda[j] != 0) {
			F = model.X_lambda[j].toMatrixTranspose() * F;
			j = model.lambda[j];

			if (model.mJoints[j].mJointType == JointTypeFixed) {
				continue;
			}

			dof_j = j - model.mFixedJointCount[j] - 1;

			LOG << "i,j         = " << i << ", " << j << std::endl;
			LOG << "dof_i,dof_j = " << dof_i << ", " << dof_j << std::endl;
			H(dof_i,dof_j) = F.dot(model.S[j]);
			H(dof_j,dof_i) = H(dof_i,dof_j);
		}
	}
}

} /* namespace RigidBodyDynamics */
