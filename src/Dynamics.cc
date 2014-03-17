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

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;

RBDL_DLLAPI
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
		unsigned int q_index = model.mJoints[i].q_index;
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, v_J, c_J, Q, QDot);

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda != 0)
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
		else
			model.X_base[i] = model.X_lambda[i];

		model.v[i] = model.X_lambda[i].apply( model.v[lambda]) + v_J;

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

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int q_index = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 3) {
			model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
#ifdef EIGEN_CORE_H
			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse().eval();
#else
			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse();
#endif
			Vector3d tau_temp (Tau[q_index], Tau[q_index + 1], Tau[q_index + 2]);

			model.multdof3_u[i] = tau_temp - model.multdof3_S[i].transpose() * model.pA[i];

//			LOG << "multdof3_u[" << i << "] = " << model.multdof3_u[i].transpose() << std::endl;
			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia = model.IA[i] - model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_U[i].transpose();
				SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_u[i];
#ifdef EIGEN_CORE_H
				model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
				model.IA[lambda] += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
				LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose() << std::endl;
			}
		} else {
			model.U[i] = model.IA[i] * model.S[i];
			model.d[i] = model.S[i].dot(model.U[i]);
			model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
//			LOG << "u[" << i << "] = " << model.u[i] << std::endl;

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
				LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose() << std::endl;
			}
		}
	}

//	ClearLogOutput();

	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];
		SpatialTransform X_lambda = model.X_lambda[i];

		model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];
		LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

		if (model.mJoints[i].mDoFCount == 3) {
			Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * model.a[i]);
			QDDot[q_index] = qdd_temp[0];
			QDDot[q_index + 1] = qdd_temp[1];
			QDDot[q_index + 2] = qdd_temp[2];
			model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
		} else {
			QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
			model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
		}
	}

	LOG << "QDDot = " << QDDot.transpose() << std::endl;
}

RBDL_DLLAPI
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

RBDL_DLLAPI
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
		unsigned int q_index = model.mJoints[i].q_index;
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, v_J, c_J, Q, QDot);

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda == 0) {
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = v_J;
			model.a[i] = model.X_base[i].apply(spatial_gravity * -1.);
			
			if (model.mJoints[i].mDoFCount == 3) {
				model.a[i] = model.a[i] + model.multdof3_S[i] * Vector3d (QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]); 
			} else {
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			}	

		}	else {
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
			model.c[i] = c_J + crossm(model.v[i],v_J);
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];

			if (model.mJoints[i].mDoFCount == 3) {
				Vector3d omegadot_temp (QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]);
				model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
			} else {
				model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
			}	
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
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

		if (model.mJoints[i].mDoFCount == 3) {
			Vector3d tau_temp = model.multdof3_S[i].transpose() * model.f[i];
			Tau[q_index] = tau_temp[0];
			Tau[q_index+1] = tau_temp[1];
			Tau[q_index+2] = tau_temp[2];
		} else {
			Tau[q_index] = model.S[i].dot(model.f[i]);
		}

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

RBDL_DLLAPI
void CompositeRigidBodyAlgorithm (Model& model, const VectorNd &Q, MatrixNd &H, bool update_kinematics) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	assert (H.rows() == model.dof_count && H.cols() == model.dof_count);

	unsigned int i;

	for (i = 1; i < model.mBodies.size(); i++) {
		if (update_kinematics)
			model.X_lambda[i] = jcalc_XJ (model, i, Q) * model.X_T[i];
		model.Ic[i].createFromMatrix(model.mBodies[i].mSpatialInertia);
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int lambda = model.lambda[i];

		if (lambda != 0) {
			model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].applyTranspose(model.Ic[i]);
		}

		unsigned int dof_index_i = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 3) {
			Matrix63 F_63 = model.Ic[i].toMatrix() * model.multdof3_S[i];
			Matrix3d H_temp = model.multdof3_S[i].transpose() * F_63;

			H.block<3,3>(dof_index_i, dof_index_i) = H_temp;

			unsigned int j = i;
			unsigned int dof_index_j = dof_index_i;

			while (model.lambda[j] != 0) {
				F_63 = model.X_lambda[j].toMatrixTranspose() * (F_63);
				j = model.lambda[j];
				dof_index_j = model.mJoints[j].q_index;

				if (model.mJoints[j].mDoFCount == 3) {
					Matrix3d H_temp2 = F_63.transpose() * (model.multdof3_S[j]);

					H.block<3,3>(dof_index_i,dof_index_j) = H_temp2;
					H.block<3,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
				} else {
					Vector3d H_temp2 = F_63.transpose() * (model.S[j]);

					H.block<3,1>(dof_index_i,dof_index_j) = H_temp2;
					H.block<1,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
				}
			}
		} else {
			SpatialVector F = model.Ic[i] * model.S[i];
			H(dof_index_i, dof_index_i) = model.S[i].dot(F);

			unsigned int j = i;
			unsigned int dof_index_j = dof_index_i;

			while (model.lambda[j] != 0) {
				F = model.X_lambda[j].applyTranspose(F);
				j = model.lambda[j];
				dof_index_j = model.mJoints[j].q_index;

				if (model.mJoints[j].mDoFCount == 3) {
					Vector3d H_temp2 = (F.transpose() * model.multdof3_S[j]).transpose();

					H.block<1,3>(dof_index_i,dof_index_j) = H_temp2.transpose();
 					H.block<3,1>(dof_index_j,dof_index_i) = H_temp2;
				} else {
					H(dof_index_i,dof_index_j) = F.dot(model.S[j]);
					H(dof_index_j,dof_index_i) = H(dof_index_i,dof_index_j);
				}
			}
		}
	}
}

} /* namespace RigidBodyDynamics */
