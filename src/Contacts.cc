/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"
#include "rbdl/Contacts.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;

unsigned int ConstraintSet::AddConstraint (
		unsigned int body_id,
		const Vector3d &body_point,
		const Vector3d &world_normal,
		const char *constraint_name,
		double normal_acceleration) {
	assert (bound == false);

	std::string name_str;
	if (constraint_name != NULL)
		name_str = constraint_name;

	name.push_back (name_str);
	body.push_back (body_id);
	point.push_back (body_point);
	normal.push_back (world_normal);

	unsigned int n_constr = acceleration.size() + 1;

	acceleration.conservativeResize (n_constr);
	acceleration[n_constr - 1] = normal_acceleration;

	force.conservativeResize (n_constr);
	force[n_constr - 1] = 0.;

	impulse.conservativeResize (n_constr);
	impulse[n_constr - 1] = 0.;

	v_plus.conservativeResize (n_constr);
	v_plus[n_constr - 1] = 0.;

	d_multdof3_u = std::vector<Math::Vector3d> (n_constr, Math::Vector3d::Zero());

	return n_constr - 1;
}

bool ConstraintSet::Bind (const Model &model) {
	assert (bound == false);

	if (bound) {
		std::cerr << "Error: binding an already bound constraint set!" << std::endl;
		abort();
	}
	unsigned int n_constr = size();

	H.conservativeResize (model.dof_count, model.dof_count);
	H.setZero();
	C.conservativeResize (model.dof_count);
	C.setZero();
	gamma.conservativeResize (n_constr);
	gamma.setZero();
	G.conservativeResize (n_constr, model.dof_count);
	G.setZero();
	A.conservativeResize (model.dof_count + n_constr, model.dof_count + n_constr);
	A.setZero();
	b.conservativeResize (model.dof_count + n_constr);
	b.setZero();
	x.conservativeResize (model.dof_count + n_constr);
	x.setZero();

#ifdef RBDL_USE_SIMPLE_MATH
	GT_qr = SimpleMath::HouseholderQR<Math::MatrixNd> (G.transpose());
#else
	GT_qr = Eigen::HouseholderQR<Math::MatrixNd> (G.transpose());
#endif
	GT_qr_Q = MatrixNd::Zero (model.dof_count, model.dof_count);
	Y = MatrixNd::Zero (model.dof_count, G.rows());
	Z = MatrixNd::Zero (model.dof_count, model.dof_count - G.rows());
	qddot_y = VectorNd::Zero (model.dof_count);
	qddot_z = VectorNd::Zero (model.dof_count);

	K.conservativeResize (n_constr, n_constr);
	K.setZero();
	a.conservativeResize (n_constr);
	a.setZero();
	QDDot_t.conservativeResize (model.dof_count);
	QDDot_t.setZero();
	QDDot_0.conservativeResize (model.dof_count);
	QDDot_0.setZero();
	f_t.resize (n_constr, SpatialVectorZero);
	f_ext_constraints.resize (model.mBodies.size(), SpatialVectorZero);
	point_accel_0.resize (n_constr, Vector3d::Zero());

	d_pA = std::vector<SpatialVector> (model.mBodies.size(), SpatialVectorZero);
	d_a = std::vector<SpatialVector> (model.mBodies.size(), SpatialVectorZero);
	d_u = VectorNd::Zero (model.mBodies.size());

	d_IA = std::vector<SpatialMatrix> (model.mBodies.size(), SpatialMatrixIdentity);
	d_U = std::vector<SpatialVector> (model.mBodies.size(), SpatialVectorZero);
	d_d = VectorNd::Zero (model.mBodies.size());

	d_multdof3_u = std::vector<Math::Vector3d> (model.mBodies.size(), Math::Vector3d::Zero());

	bound = true;

	return bound;
}

void ConstraintSet::clear() {
	acceleration.setZero();
	force.setZero();
	impulse.setZero();

	H.setZero();
	C.setZero();
	gamma.setZero();
	G.setZero();
	A.setZero();
	b.setZero();
	x.setZero();

	K.setZero();
	a.setZero();
	QDDot_t.setZero();
	QDDot_0.setZero();

	unsigned int i;
	for (i = 0; i < f_t.size(); i++)
		f_t[i].setZero();

	for (i = 0; i < f_ext_constraints.size(); i++)
		f_ext_constraints[i].setZero();

	for (i = 0; i < point_accel_0.size(); i++)
		point_accel_0[i].setZero();

	for (i = 0; i < d_pA.size(); i++)
		d_pA[i].setZero();

	for (i = 0; i < d_a.size(); i++)
		d_a[i].setZero();

	d_u.setZero();
}

RBDL_DLLAPI
void SolveContactSystemDirect (
		Math::MatrixNd &H, 
		const Math::MatrixNd &G, 
		const Math::VectorNd &c, 
		const Math::VectorNd &gamma, 
		Math::VectorNd &qddot, 
		Math::VectorNd &lambda, 
		Math::MatrixNd &A, 
		Math::VectorNd &b,
		Math::VectorNd &x,
		Math::LinearSolver &linear_solver
		) {
	// Build the system: Copy H
	A.block(0, 0, c.rows(), c.rows()) = H;

	// Copy G and G^T
	A.block(0, c.rows(), c.rows(), gamma.rows()) = G.transpose();
	A.block(c.rows(), 0, gamma.rows(), c.rows()) = G;

	// Build the system: Copy -C + \tau
	b.block(0, 0, c.rows(), 1) = c;
	b.block(c.rows(), 0, gamma.rows(), 1) = gamma;

	LOG << "A = " << std::endl << A << std::endl;
	LOG << "b = " << std::endl << b << std::endl;

	switch (linear_solver) {
		case (LinearSolverPartialPivLU) :
#ifdef RBDL_USE_SIMPLE_MATH
			// SimpleMath does not have a LU solver so just use its QR solver
			x = A.householderQr().solve(b);
#else
			x = A.partialPivLu().solve(b);
#endif
			break;
		case (LinearSolverColPivHouseholderQR) :
			x = A.colPivHouseholderQr().solve(b);
			break;
		case (LinearSolverHouseholderQR) :
			x = A.householderQr().solve(b);
			break;
		default:
			LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
			assert (0);
			break;
	}

	LOG << "x = " << std::endl << x << std::endl;
}

RBDL_DLLAPI
void SolveContactSystemRangeSpaceSparse (
		Model &model, 
		Math::MatrixNd &H, 
		const Math::MatrixNd &G, 
		const Math::VectorNd &c, 
		const Math::VectorNd &gamma, 
		Math::VectorNd &qddot, 
		Math::VectorNd &lambda, 
		Math::MatrixNd &K, 
		Math::VectorNd &a,
		Math::LinearSolver linear_solver
	) {
	SparseFactorizeLTL (model, H);

	MatrixNd Y (G.transpose());

	for (unsigned int i = 0; i < Y.cols(); i++) {
		VectorNd Y_col = Y.block(0,i,Y.rows(),1);
		SparseSolveLTx (model, H, Y_col);
		Y.block(0,i,Y.rows(),1) = Y_col;
	}

	VectorNd z (c);
	SparseSolveLTx (model, H, z);

	K = Y.transpose() * Y;

	a = gamma - Y.transpose() * z;

	lambda = K.llt().solve(a);

	qddot = c + G.transpose() * lambda;
	SparseSolveLTx (model, H, qddot);
	SparseSolveLx (model, H, qddot);
}

RBDL_DLLAPI
void SolveContactSystemNullSpace (
		Math::MatrixNd &H, 
		const Math::MatrixNd &G, 
		const Math::VectorNd &c, 
		const Math::VectorNd &gamma, 
		Math::VectorNd &qddot, 
		Math::VectorNd &lambda,
		Math::MatrixNd &Y,
		Math::MatrixNd &Z,
		Math::VectorNd &qddot_y,
		Math::VectorNd &qddot_z,
		Math::LinearSolver &linear_solver
		) {
	switch (linear_solver) {
		case (LinearSolverPartialPivLU) :
#ifdef RBDL_USE_SIMPLE_MATH
			// SimpleMath does not have a LU solver so just use its QR solver
			qddot_y = (G * Y).householderQr().solve (gamma);
#else
			qddot_y = (G * Y).partialPivLu().solve (gamma);
#endif
			break;
		case (LinearSolverColPivHouseholderQR) :
			qddot_y = (G * Y).colPivHouseholderQr().solve (gamma);
			break;
		case (LinearSolverHouseholderQR) :
			qddot_y = (G * Y).householderQr().solve (gamma);
			break;
		default:
			LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
			assert (0);
			break;
	}

	qddot_z = (Z.transpose() * H * Z).llt().solve(Z.transpose() * (c - H * Y * qddot_y));

	qddot = Y * qddot_y + Z * qddot_z;

	switch (linear_solver) {
		case (LinearSolverPartialPivLU) :
#ifdef RBDL_USE_SIMPLE_MATH
			// SimpleMath does not have a LU solver so just use its QR solver
			qddot_y = (G * Y).householderQr().solve (gamma);
#else
			lambda = (G * Y).partialPivLu().solve (Y.transpose() * (H * qddot - c));
#endif
			break;
		case (LinearSolverColPivHouseholderQR) :
			lambda = (G * Y).colPivHouseholderQr().solve (Y.transpose() * (H * qddot - c));
			break;
		case (LinearSolverHouseholderQR) :
			lambda = (G * Y).householderQr().solve (Y.transpose() * (H * qddot - c));
			break;
		default:
			LOG << "Error: Invalid linear solver: " << linear_solver << std::endl;
			assert (0);
			break;
	}
}

RBDL_DLLAPI
void CalcContactJacobian(
		Model &model,
		const Math::VectorNd &Q,
		const ConstraintSet &CS,
		Math::MatrixNd &G,
		bool update_kinematics
		) {
	if (update_kinematics)
		UpdateKinematicsCustom (model, &Q, NULL, NULL);

	unsigned int i,j;

	// variables to check whether we need to recompute G
	unsigned int prev_body_id = 0;
	Vector3d prev_body_point = Vector3d::Zero();
	MatrixNd Gi (3, model.dof_count);

	for (i = 0; i < CS.size(); i++) {
		// only compute the matrix Gi if actually needed
		if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
			Gi.setZero();
			CalcPointJacobian (model, Q, CS.body[i], CS.point[i], Gi, false);
			prev_body_id = CS.body[i];
			prev_body_point = CS.point[i];
		}

		for (j = 0; j < model.dof_count; j++) {
			Vector3d gaxis (Gi(0,j), Gi(1,j), Gi(2,j));
			G(i,j) = gaxis.transpose() * CS.normal[i];
		}
	}
}

RBDL_DLLAPI
void CalcContactSystemVariables (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDot,
		const Math::VectorNd &Tau,
		ConstraintSet &CS
		) {
	// Compute C
	NonlinearEffects (model, Q, QDot, CS.C);
	assert (CS.H.cols() == model.dof_count && CS.H.rows() == model.dof_count);

	// Compute H
	CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

	// Compute G
	// We have to update model.X_base as they are not automatically computed
	// by NonlinearEffects()
	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		model.X_base[i] = model.X_lambda[i] * model.X_base[model.lambda[i]];
	}
	CalcContactJacobian (model, Q, CS, CS.G, false);

	// Compute gamma
	unsigned int prev_body_id = 0;
	Vector3d prev_body_point = Vector3d::Zero();
	Vector3d gamma_i = Vector3d::Zero();

	CS.QDDot_0.setZero();
	UpdateKinematicsCustom (model, NULL, NULL, &CS.QDDot_0);

	for (unsigned int i = 0; i < CS.size(); i++) {
		// only compute point accelerations when necessary
		if (prev_body_id != CS.body[i] || prev_body_point != CS.point[i]) {
			gamma_i = CalcPointAcceleration (model, Q, QDot, CS.QDDot_0, CS.body[i], CS.point[i], false);
			prev_body_id = CS.body[i];
			prev_body_point = CS.point[i];
		}

		// we also substract ContactData[i].acceleration such that the contact
		// point will have the desired acceleration
		CS.gamma[i] = CS.acceleration[i] - CS.normal[i].dot(gamma_i);
	}
}

RBDL_DLLAPI
void ForwardDynamicsContactsDirect (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		ConstraintSet &CS,
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	CalcContactSystemVariables (model, Q, QDot, Tau, CS);

	SolveContactSystemDirect (CS.H, CS.G, Tau - CS.C, CS.gamma, QDDot, CS.force, CS.A, CS.b, CS.x, CS.linear_solver);

	// Copy back QDDot
	for (unsigned int i = 0; i < model.dof_count; i++)
		QDDot[i] = CS.x[i];

	// Copy back contact forces
	for (unsigned int i = 0; i < CS.size(); i++) {
		CS.force[i] = -CS.x[model.dof_count + i];
	}
}

RBDL_DLLAPI
void ForwardDynamicsContactsRangeSpaceSparse (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDot,
		const Math::VectorNd &Tau,
		ConstraintSet &CS,
		Math::VectorNd &QDDot
		) {
	CalcContactSystemVariables (model, Q, QDot, Tau, CS);

	SolveContactSystemRangeSpaceSparse (model, CS.H, CS.G, Tau - CS.C, CS.gamma, QDDot, CS.force, CS.K, CS.a, CS.linear_solver);
}

RBDL_DLLAPI
void ForwardDynamicsContactsNullSpace (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		ConstraintSet &CS,
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	CalcContactSystemVariables (model, Q, QDot, Tau, CS);

	CS.GT_qr.compute (CS.G.transpose());
#ifdef RBDL_USE_SIMPLE_MATH
	CS.GT_qr_Q = CS.GT_qr.householderQ();
#else
	CS.GT_qr.householderQ().evalTo (CS.GT_qr_Q);
#endif

	CS.Y = CS.GT_qr_Q.block(0,0,QDot.rows(), CS.G.rows());
	CS.Z = CS.GT_qr_Q.block(0,CS.G.rows(),QDot.rows(), QDot.rows() - CS.G.rows());

	SolveContactSystemNullSpace (CS.H, CS.G, Tau - CS.C, CS.gamma, QDDot, CS.force, CS.Y, CS.Z, CS.qddot_y, CS.qddot_z, CS.linear_solver);
}

RBDL_DLLAPI
void ComputeContactImpulsesDirect (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDotMinus,
		ConstraintSet &CS,
		Math::VectorNd &QDotPlus
		) {
	// Compute H
	UpdateKinematicsCustom (model, &Q, NULL, NULL);
	CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

	// Compute G
	CalcContactJacobian (model, Q, CS, CS.G, false);

	SolveContactSystemDirect (CS.H, CS.G, CS.H * QDotMinus, CS.v_plus, QDotPlus, CS.impulse, CS.A, CS.b, CS.x, CS.linear_solver);

	// Copy back QDotPlus
	for (unsigned int i = 0; i < model.dof_count; i++)
		QDotPlus[i] = CS.x[i];

	// Copy back constraint impulses 
	for (unsigned int i = 0; i < CS.size(); i++) {
		CS.impulse[i] = CS.x[model.dof_count + i];
	}
}

RBDL_DLLAPI
void ComputeContactImpulsesRangeSpaceSparse (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDotMinus,
		ConstraintSet &CS,
		Math::VectorNd &QDotPlus
		) {
	// Compute H
	UpdateKinematicsCustom (model, &Q, NULL, NULL);
	CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

	// Compute G
	CalcContactJacobian (model, Q, CS, CS.G, false);

	SolveContactSystemRangeSpaceSparse (model, CS.H, CS.G, CS.H * QDotMinus, CS.v_plus, QDotPlus, CS.impulse, CS.K, CS.a, CS.linear_solver);
}

RBDL_DLLAPI
void ComputeContactImpulsesNullSpace (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDotMinus,
		ConstraintSet &CS,
		Math::VectorNd &QDotPlus
		) {
	// Compute H
	UpdateKinematicsCustom (model, &Q, NULL, NULL);
	CompositeRigidBodyAlgorithm (model, Q, CS.H, false);

	// Compute G
	CalcContactJacobian (model, Q, CS, CS.G, false);

	CS.GT_qr.compute (CS.G.transpose());
	CS.GT_qr_Q = CS.GT_qr.householderQ();

	CS.Y = CS.GT_qr_Q.block(0,0,QDotMinus.rows(), CS.G.rows());
	CS.Z = CS.GT_qr_Q.block(0,CS.G.rows(),QDotMinus.rows(), QDotMinus.rows() - CS.G.rows());

	SolveContactSystemNullSpace (CS.H, CS.G, CS.H * QDotMinus, CS.v_plus, QDotPlus, CS.impulse, CS.Y, CS.Z, CS.qddot_y, CS.qddot_z, CS.linear_solver);
}

/** \brief Compute only the effects of external forces on the generalized accelerations
 *
 * This function is a reduced version of ForwardDynamics() which only
 * computes the effects of the external forces on the generalized
 * accelerations.
 *
 */
RBDL_DLLAPI
void ForwardDynamicsApplyConstraintForces (
		Model &model,
		const VectorNd &Tau,
		ConstraintSet &CS,
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;
	assert (QDDot.size() == model.dof_count);

	unsigned int i = 0;

	for (i = 1; i < model.mBodies.size(); i++) {
		model.IA[i] = model.I[i].toMatrix();;
		model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

		if (CS.f_ext_constraints[i] != SpatialVectorZero) {
			LOG << "External force (" << i << ") = " << model.X_base[i].toMatrixAdjoint() * CS.f_ext_constraints[i] << std::endl;
			model.pA[i] -= model.X_base[i].toMatrixAdjoint() * CS.f_ext_constraints[i];
		}
	}

// ClearLogOutput();

	LOG << "--- first loop ---" << std::endl;

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int q_index = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 3) {
			unsigned int lambda = model.lambda[i];

			model.multdof3_u[i] = Vector3d (Tau[q_index], Tau[q_index + 1], Tau[q_index + 2]) - model.multdof3_S[i].transpose() * model.pA[i];

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
			model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);

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
	
	model.a[0] = SpatialVector (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);

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

/** \brief Computes the effect of external forces on the generalized accelerations.
 *
 * This function is essentially similar to ForwardDynamics() except that it
 * tries to only perform computations of variables that change due to
 * external forces defined in f_t.
 */
RBDL_DLLAPI
void ForwardDynamicsAccelerationDeltas (
		Model &model,
		ConstraintSet &CS,
		VectorNd &QDDot_t,
		const unsigned int body_id,
		const std::vector<SpatialVector> &f_t
		) {
	LOG << "-------- " << __func__ << " ------" << std::endl;

	assert (CS.d_pA.size() == model.mBodies.size());
	assert (CS.d_a.size() == model.mBodies.size());
	assert (CS.d_u.size() == model.mBodies.size());

	// TODO reset all values (debug)
	for (unsigned int i = 0; i < model.mBodies.size(); i++) {
		CS.d_pA[i].setZero();
		CS.d_a[i].setZero();
		CS.d_u[i] = 0.;
		CS.d_multdof3_u[i].setZero();
	}

	for (unsigned int i = body_id; i > 0; i--) {
		if (i == body_id) {
			CS.d_pA[i] = -model.X_base[i].applyAdjoint(f_t[i]);
		}

		if (model.mJoints[i].mDoFCount == 3) {
			CS.d_multdof3_u[i] = - model.multdof3_S[i].transpose() * (CS.d_pA[i]);

			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				CS.d_pA[lambda] = CS.d_pA[lambda] + model.X_lambda[i].applyTranspose (CS.d_pA[i] + model.multdof3_U[i] * model.multdof3_Dinv[i] * CS.d_multdof3_u[i]);
			}
		} else {
			CS.d_u[i] = - model.S[i].dot(CS.d_pA[i]);

			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				CS.d_pA[lambda] = CS.d_pA[lambda] + model.X_lambda[i].applyTranspose (CS.d_pA[i] + model.U[i] * CS.d_u[i] / model.d[i]);
			}
		}
	}

	for (unsigned int i = 0; i < f_t.size(); i++) {
		LOG << "f_t[" << i << "] = " << f_t[i].transpose() << std::endl;
	}

	for (unsigned int i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_pA[i] " << CS.d_pA[i].transpose() << std::endl;
	}
	for (unsigned int i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_u[i] = " << CS.d_u[i] << std::endl;
	}

	QDDot_t[0] = 0.;
	CS.d_a[0] = model.a[0];

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

		SpatialVector Xa = model.X_lambda[i].apply(CS.d_a[lambda]);

		if (model.mJoints[i].mDoFCount == 3) {
			Vector3d qdd_temp = model.multdof3_Dinv[i] * (CS.d_multdof3_u[i] - model.multdof3_U[i].transpose() * Xa);
			QDDot_t[q_index] = qdd_temp[0];
			QDDot_t[q_index + 1] = qdd_temp[1];
			QDDot_t[q_index + 2] = qdd_temp[2];
			model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
			CS.d_a[i] = Xa + model.multdof3_S[i] * qdd_temp;
		} else {
			QDDot_t[q_index] = (CS.d_u[i] - model.U[i].dot(Xa) ) / model.d[i];
			CS.d_a[i] = Xa + model.S[i] * QDDot_t[q_index];
		}
	
		LOG << "QDDot_t[" << i - 1 << "] = " << QDDot_t[i - 1] << std::endl;
		LOG << "d_a[i] = " << CS.d_a[i].transpose() << std::endl;
	}
}

inline void set_zero (std::vector<SpatialVector> &spatial_values) {
	for (unsigned int i = 0; i < spatial_values.size(); i++)
		spatial_values[i].setZero();
}

RBDL_DLLAPI
void ForwardDynamicsContactsKokkevis (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		ConstraintSet &CS,
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " ------" << std::endl;

	assert (CS.f_ext_constraints.size() == model.mBodies.size());
	assert (CS.QDDot_0.size() == model.dof_count);
	assert (CS.QDDot_t.size() == model.dof_count);
	assert (CS.f_t.size() == CS.size());
	assert (CS.point_accel_0.size() == CS.size());
	assert (CS.K.rows() == CS.size());
	assert (CS.K.cols() == CS.size());
	assert (CS.force.size() == CS.size());
	assert (CS.a.size() == CS.size());

	Vector3d point_accel_t;

	unsigned int ci = 0;
	
	// The default acceleration only needs to be computed once
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, CS.QDDot_0);
	}

	LOG << "=== Initial Loop Start ===" << std::endl;
	// we have to compute the standard accelerations first as we use them to
	// compute the effects of each test force
	for (ci = 0; ci < CS.size(); ci++) {
		unsigned int body_id = CS.body[ci];
		Vector3d point = CS.point[ci];
		Vector3d normal = CS.normal[ci];
		double acceleration = CS.acceleration[ci];

		LOG << "body_id = " << body_id << std::endl;
		LOG << "point = " << point << std::endl;
		LOG << "normal = " << normal << std::endl;
		LOG << "QDDot_0 = " << CS.QDDot_0.transpose() << std::endl;
		{
			SUPPRESS_LOGGING;
			UpdateKinematicsCustom (model, NULL, NULL, &CS.QDDot_0);
			CS.point_accel_0[ci] = CalcPointAcceleration (model, Q, QDot, CS.QDDot_0, body_id, point, false);

			CS.a[ci] = - acceleration + normal.dot(CS.point_accel_0[ci]);
		}
		LOG << "point_accel_0 = " << CS.point_accel_0[ci].transpose();
	}

	// Now we can compute and apply the test forces and use their net effect
	// to compute the inverse articlated inertia to fill K.
	for (ci = 0; ci < CS.size(); ci++) {
		LOG << "=== Testforce Loop Start ===" << std::endl;
		unsigned int body_id = CS.body[ci];
		Vector3d point = CS.point[ci];
		Vector3d normal = CS.normal[ci];

		unsigned int movable_body_id = body_id;
		if (model.IsFixedBodyId(body_id)) {
			unsigned int fbody_id = body_id - model.fixed_body_discriminator;
			movable_body_id = model.mFixedBodies[fbody_id].mMovableParent;
		}

		// assemble the test force
		LOG << "normal = " << normal.transpose() << std::endl;

		Vector3d point_global = CalcBodyToBaseCoordinates (model, Q, body_id, point, false);
		LOG << "point_global = " << point_global.transpose() << std::endl;

		CS.f_t[ci] = SpatialTransform (Matrix3d::Identity(), -point_global).applyAdjoint (SpatialVector (0., 0., 0., -normal[0], -normal[1], -normal[2]));
		CS.f_ext_constraints[movable_body_id] = CS.f_t[ci];
		LOG << "f_t[" << movable_body_id << "] = " << CS.f_t[ci].transpose() << std::endl;

		{
//			SUPPRESS_LOGGING;
			ForwardDynamicsAccelerationDeltas (model, CS, CS.QDDot_t, movable_body_id, CS.f_ext_constraints);
			LOG << "QDDot_0 = " << CS.QDDot_0.transpose() << std::endl;
			LOG << "QDDot_t = " << (CS.QDDot_t + CS.QDDot_0).transpose() << std::endl;
			LOG << "QDDot_t - QDDot_0= " << (CS.QDDot_t).transpose() << std::endl;
		}

		CS.f_ext_constraints[movable_body_id].setZero();

		CS.QDDot_t += CS.QDDot_0;

		// compute the resulting acceleration
		{
			SUPPRESS_LOGGING;
			UpdateKinematicsCustom (model, NULL, NULL, &CS.QDDot_t);
		}

		for (unsigned int cj = 0; cj < CS.size(); cj++) {
			{
				SUPPRESS_LOGGING;

				point_accel_t = CalcPointAcceleration (model, Q, QDot, CS.QDDot_t, CS.body[cj], CS.point[cj], false);
			}
	
			LOG << "point_accel_0  = " << CS.point_accel_0[ci].transpose() << std::endl;
			CS.K(ci,cj) = CS.normal[cj].dot(point_accel_t - CS.point_accel_0[cj]);
			LOG << "point_accel_t = " << point_accel_t.transpose() << std::endl;
		}
	}

	LOG << "K = " << std::endl << CS.K << std::endl;
	LOG << "a = " << std::endl << CS.a << std::endl;

#ifndef RBDL_USE_SIMPLE_MATH
	switch (CS.linear_solver) {
		case (LinearSolverPartialPivLU) :
			CS.force = CS.K.partialPivLu().solve(CS.a);
			break;
		case (LinearSolverColPivHouseholderQR) :
			CS.force = CS.K.colPivHouseholderQr().solve(CS.a);
			break;
		case (LinearSolverHouseholderQR) :
			CS.force = CS.K.householderQr().solve(CS.a);
			break;
		default:
			LOG << "Error: Invalid linear solver: " << CS.linear_solver << std::endl;
			assert (0);
			break;
	}
#else
	bool solve_successful = LinSolveGaussElimPivot (CS.K, CS.a, CS.force);
	assert (solve_successful);
#endif

	LOG << "f = " << CS.force.transpose() << std::endl;

	for (ci = 0; ci < CS.size(); ci++) {
		unsigned int body_id = CS.body[ci];
		unsigned int movable_body_id = body_id;

		if (model.IsFixedBodyId(body_id)) {
			unsigned int fbody_id = body_id - model.fixed_body_discriminator;
			movable_body_id = model.mFixedBodies[fbody_id].mMovableParent;
		}

		CS.f_ext_constraints[movable_body_id] -= CS.f_t[ci] * CS.force[ci]; 
		LOG << "f_ext[" << movable_body_id << "] = " << CS.f_ext_constraints[movable_body_id].transpose() << std::endl;
	}

	{
		SUPPRESS_LOGGING;
		ForwardDynamicsApplyConstraintForces (model, Tau, CS, QDDot);
	}

	LOG << "QDDot after applying f_ext: " << QDDot.transpose() << std::endl;
}

} /* namespace RigidBodyDynamics */
