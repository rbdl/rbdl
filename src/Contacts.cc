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
#include "Contacts.h"
#include "Dynamics.h"
#include "Dynamics_experimental.h"
#include "Kinematics.h"

using namespace SpatialAlgebra;
using namespace SpatialAlgebra::Operators;

namespace RigidBodyDynamics {

void ForwardDynamicsContactsLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		std::vector<ContactInfo> &ContactData,
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	// Note: InverseDynamics must be called *before*
	// CompositeRigidBodyAlgorithm() as the latter does not update
	// transformations etc.!

	// Compute C
	VectorNd QDDot_zero = VectorNd::Zero (model.dof_count);
	VectorNd C (model.dof_count);

	InverseDynamics (model, Q, QDot, QDDot_zero, C);

	// Compute H
	MatrixNd H (model.dof_count, model.dof_count);
	CompositeRigidBodyAlgorithm (model, Q, H);

	// Compute G
	MatrixNd G (ContactData.size(), model.dof_count);

	unsigned int i,j;

	// variables to check whether we need to recompute G
	unsigned int prev_body_id = 0;
	Vector3d prev_body_point = Vector3d::Zero();
	MatrixNd Gi (3, model.dof_count);

	for (i = 0; i < ContactData.size(); i++) {
		// Only alow contact normals along the coordinate axes
		unsigned int axis_index = 0;

		if (ContactData[i].normal == Vector3d(1., 0., 0.))
			axis_index = 0;
		else if (ContactData[i].normal == Vector3d(0., 1., 0.))
			axis_index = 1;
		else if (ContactData[i].normal == Vector3d(0., 0., 1.))
			axis_index = 2;
		else
			assert (0 && "Invalid contact normal axis!");

		// only compute the matrix Gi if actually needed
		if (prev_body_id != ContactData[i].body_id || prev_body_point != ContactData[i].point) {
			CalcPointJacobian (model, Q, ContactData[i].body_id, ContactData[i].point, Gi, false);
			prev_body_id = ContactData[i].body_id;
			prev_body_point = ContactData[i].point;
		}

		for (j = 0; j < model.dof_count; j++) {
			G(i,j) = Gi(axis_index, j);
		}
	}

	// Compute gamma
	VectorNd gamma (ContactData.size());
	prev_body_id = 0;
	prev_body_point = Vector3d::Zero();
	Vector3d gamma_i = Vector3d::Zero();

	// update Kinematics just once
	ForwardKinematics (model, Q, QDot, QDDot_zero);

	for (i = 0; i < ContactData.size(); i++) {
		// Only alow contact normals along the coordinate axes
		unsigned int axis_index = 0;

		if (ContactData[i].normal == Vector3d(1., 0., 0.))
			axis_index = 0;
		else if (ContactData[i].normal == Vector3d(0., 1., 0.))
			axis_index = 1;
		else if (ContactData[i].normal == Vector3d(0., 0., 1.))
			axis_index = 2;
		else
			assert (0 && "Invalid contact normal axis!");

		// only compute point accelerations when necessary
		if (prev_body_id != ContactData[i].body_id || prev_body_point != ContactData[i].point) {
			gamma_i = CalcPointAcceleration (model, Q, QDot, QDDot_zero, ContactData[i].body_id, ContactData[i].point, false);
			prev_body_id = ContactData[i].body_id;
			prev_body_point = ContactData[i].point;
		}
	
		// we also substract ContactData[i].acceleration such that the contact
		// point will have the desired acceleration
		gamma[i] = gamma_i[axis_index] - ContactData[i].acceleration;
	}
	
	// Build the system
	MatrixNd A = MatrixNd::Constant (model.dof_count + ContactData.size(), model.dof_count + ContactData.size(), 0.);
	VectorNd b = VectorNd::Constant (model.dof_count + ContactData.size(), 0.);
	VectorNd x = VectorNd::Constant (model.dof_count + ContactData.size(), 0.);

	// Build the system: Copy H
	for (i = 0; i < model.dof_count; i++) {
		for (j = 0; j < model.dof_count; j++) {
			A(i,j) = H(i,j);	
		}
	}

	// Build the system: Copy G, and G^T
	for (i = 0; i < ContactData.size(); i++) {
		for (j = 0; j < model.dof_count; j++) {
			A(i + model.dof_count, j) = G (i,j);
			A(j, i + model.dof_count) = G (i,j);
		}
	}

	// Build the system: Copy -C + \tau
	for (i = 0; i < model.dof_count; i++) {
		b[i] = -C[i] + Tau[i];
	}

	// Build the system: Copy -gamma
	for (i = 0; i < ContactData.size(); i++) {
		b[i + model.dof_count] = - gamma[i];
	}

	LOG << "A = " << std::endl << A << std::endl;
	LOG << "b = " << std::endl << b << std::endl;
	
	// Solve the system
#ifndef RBDL_USE_SIMPLE_MATH
//	x = A.ldlt().solve(b);
//	x = A.householderQr().solve(b);
//	x = A.partialPivLu().solve(b);
	x = A.colPivHouseholderQr().solve (b);

//	LOG << "fullPivLu = " << A.fullPivLu().solve(b) << std::endl;
//	LOG << "householderQr = " << A.householderQr().solve(b) << std::endl;
//	LOG << "colPivHouseholderQr = " << A.colPivHouseholderQr().solve(b) << std::endl;
//	LOG << "partialPivLu = " << A.partialPivLu().solve(b) << std::endl;
//	LOG << "Ainv * b = " << A.inverse() * b << std::endl;
#else
	bool solve_successful = LinSolveGaussElimPivot (A, b, x);
	assert (solve_successful);
#endif

	LOG << "x = " << std::endl << x << std::endl;
	LOG << "res = A*x -b = " << (A * x - b) << std::endl;
	LOG << "A' = " << A << std::endl;
	// Copy back QDDot
	for (i = 0; i < model.dof_count; i++)
		QDDot[i] = x[i];

	// Copy back contact forces
	for (i = 0; i < ContactData.size(); i++) {
		ContactData[i].force = x[model.dof_count + i];
	}
}

void ComputeContactImpulsesLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDotMinus,
		std::vector<ContactInfo> &ContactData,
		VectorNd &QDotPlus
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	// Compute H
	MatrixNd H (model.dof_count, model.dof_count);

	VectorNd QZero = VectorNd::Zero (model.dof_count);
	ForwardKinematics (model, Q, QZero, QZero);

	// Note: ForwardKinematics must have been called beforehand!
	CompositeRigidBodyAlgorithm (model, Q, H);

	// Compute G
	MatrixNd G (ContactData.size(), model.dof_count);

	unsigned int i,j;

	// variables to check whether we need to recompute G
	unsigned int prev_body_id = 0;
	Vector3d prev_body_point = Vector3d::Zero();
	MatrixNd Gi (3, model.dof_count);

	for (i = 0; i < ContactData.size(); i++) {
		// Only alow contact normals along the coordinate axes
		unsigned int axis_index = 0;

		if (ContactData[i].normal == Vector3d(1., 0., 0.))
			axis_index = 0;
		else if (ContactData[i].normal == Vector3d(0., 1., 0.))
			axis_index = 1;
		else if (ContactData[i].normal == Vector3d(0., 0., 1.))
			axis_index = 2;
		else
			assert (0 && "Invalid contact normal axis!");

		// only compute the matrix Gi if actually needed
		if (prev_body_id != ContactData[i].body_id || prev_body_point != ContactData[i].point) {
			CalcPointJacobian (model, Q, ContactData[i].body_id, ContactData[i].point, Gi, false);
			prev_body_id = ContactData[i].body_id;
			prev_body_point = ContactData[i].point;
		}

		for (j = 0; j < model.dof_count; j++) {
			G(i,j) = Gi(axis_index, j);
		}
	}

	// Compute H * \dot{q}^-
	VectorNd Hqdotminus (H * QDotMinus);

	// Build the system
	MatrixNd A = MatrixNd::Constant (model.dof_count + ContactData.size(), model.dof_count + ContactData.size(), 0.);
	VectorNd b = VectorNd::Constant (model.dof_count + ContactData.size(), 0.);
	VectorNd x = VectorNd::Constant (model.dof_count + ContactData.size(), 0.);

	// Build the system: Copy H
	for (i = 0; i < model.dof_count; i++) {
		for (j = 0; j < model.dof_count; j++) {
			A(i,j) = H(i,j);	
		}
	}

	// Build the system: Copy G, and G^T
	for (i = 0; i < ContactData.size(); i++) {
		for (j = 0; j < model.dof_count; j++) {
			A(i + model.dof_count, j) = G (i,j);
			A(j, i + model.dof_count) = G (i,j);
		}
	}

	// Build the system: Copy -C + \tau
	for (i = 0; i < model.dof_count; i++) {
		b[i] = Hqdotminus[i];
	}

	// Build the system: Copy -gamma
	for (i = 0; i < ContactData.size(); i++) {
		b[i + model.dof_count] = ContactData[i].acceleration;
	}
	
	// Solve the system
#ifndef RBDL_USE_SIMPLE_MATH
	x = A.colPivHouseholderQr().solve (b);
#else
	bool solve_successful = LinSolveGaussElimPivot (A, b, x);
	assert (solve_successful);
#endif

	// Copy back QDDot
	for (i = 0; i < model.dof_count; i++)
		QDotPlus[i] = x[i];

	// Copy back contact impulses
	for (i = 0; i < ContactData.size(); i++) {
		ContactData[i].force = x[model.dof_count + i];
	}

}

/*
 * Experimental Code
 */

namespace Experimental {

/** \brief Compute only the effects of external forces on the generalized accelerations
 *
 * This function is a reduced version of ForwardDynamics() which only
 * computes the effects of the external forces on the generalized
 * accelerations.
 */
void ForwardDynamicsAccelerationsOnly (
		Model &model,
		VectorNd &QDDot_t,
		std::vector<SpatialAlgebra::SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	std::vector<SpatialVector> d_pv (model.mBodies.size(), SpatialVectorZero);
	std::vector<SpatialVector> d_pA (model.mBodies.size(), SpatialVectorZero);
	std::vector<SpatialVector> d_a (model.mBodies.size(), SpatialVectorZero);
	std::vector<SpatialVector> d_U (model.mBodies.size(), SpatialVectorZero);
	std::vector<SpatialMatrix> d_IA (model.mBodies.size(), SpatialMatrixZero);
	std::vector<double> d_u (model.mBodies.size(), 0.);
	std::vector<double> d_d (model.mBodies.size(), 0.);

	assert (QDDot_t.size() == model.dof_count);

	// check for proper sizes and perform resizes if necessary
	if (d_pv.size() != model.dof_count + 1) {
		d_pv.resize (model.dof_count + 1);
		d_pA.resize (model.dof_count + 1);
		d_a.resize (model.dof_count + 1);
		d_U.resize (model.dof_count + 1);
		d_IA.resize (model.dof_count + 1);

		d_u.resize (model.dof_count + 1);
		d_d.resize (model.dof_count + 1);
	}

	unsigned int i;

	for (i = 1; i < model.mBodies.size(); i++) {
		d_pA[i] = crossf(model.v[i], model.mBodies[i].mSpatialInertia * model.v[i]);
		d_IA[i] = model.mBodies[i].mSpatialInertia;

		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero) {
			d_pA[i] -= spatial_adjoint(model.X_base[i]) * (*f_ext)[i];
//			LOG << "f_t (local)[" << i << "] = " << spatial_adjoint(model.X_base[i]) * (*f_ext)[i] << std::endl;
		}
//		LOG << "i = " << i << " d_p[i] = " << d_p[i].transpose() << std::endl;
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		// we can skip further processing if the joint is fixed
		if (model.mJoints[i].mJointType == JointTypeFixed)
			continue;

		d_U[i] = d_IA[i] * model.S[i];
		d_d[i] = model.S[i].dot(model.U[i]);
		d_u[i] = model.tau[i] - model.S[i].dot(d_pA[i]);

		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			SpatialMatrix Ia = d_IA[i] - d_U[i] * (d_U[i] / d_d[i]).transpose();
			SpatialVector pa = d_pA[i] + Ia * model.c[i] + d_U[i] * d_u[i] / d_d[i];
			SpatialMatrix X_lambda = model.X_lambda[i];

			// note: X_lambda.inverse().spatial_adjoint() = X_lambda.transpose()
			d_IA[lambda] = d_IA[lambda] + X_lambda.transpose() * Ia * X_lambda;
			d_pA[lambda] = d_pA[lambda] + model.X_lambda[i].transpose() * pa;

			assert (model.IA[lambda] == d_IA[lambda]);
		}
	}

	/*
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_IA[i] " << std::endl << d_IA[i] << std::endl;
	}
	*/

	if (f_ext) {
		for (unsigned int i = 0; i < f_ext->size(); i++) {
			LOG << "f_ext[" << i << "] = " << (*f_ext)[i].transpose();
		}
	}

	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_pA[i] - pA[i] " << (d_pA[i] - model.pA[i]).transpose();
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_u[i] - u[i] = " << (d_u[i] - model.u[i]) << std::endl;
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_d[i] - d[i] = " << (d_d[i] - model.d[i]) << std::endl;
	}
	for (i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_U[i] - U[i] = " << (d_U[i] - model.U[i]).transpose() << std::endl;
	}

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		SpatialMatrix X_lambda = model.X_lambda[i];

		if (lambda == 0) {
			d_a[i] = X_lambda * spatial_gravity * (-1.) + model.c[i];
		} else {
			d_a[i] = X_lambda * d_a[lambda] + model.c[i];
		}

		// we can skip further processing if the joint type is fixed
		if (model.mJoints[i].mJointType == JointTypeFixed) {
			model.qddot[i] = 0.;
			continue;
		}

		QDDot_t[i - 1] = (d_u[i] - model.U[i].dot(d_a[i])) / model.d[i];
		LOG << "QDDot_t[" << i - 1 << "] = " << QDDot_t[i - 1] << std::endl;
		d_a[i] = d_a[i] + model.S[i] * QDDot_t[i - 1];
		LOG << "d_a[i] - a[i] = " << (d_a[i] - X_lambda * model.a[i]).transpose() << std::endl;
	}
}

void ForwardDynamicsContacts (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		std::vector<ContactInfo> &ContactData,
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " ------" << std::endl;

//	LOG << "Q    = " << Q.transpose() << std::endl;
//	LOG << "QDot = " << QDot.transpose() << std::endl;

//	assert (ContactData.size() == 1);
	std::vector<SpatialVector> f_t (ContactData.size(), SpatialVectorZero);
	std::vector<SpatialVector> f_ext_constraints (model.mBodies.size(), SpatialVectorZero);
	std::vector<Vector3d> point_accel_0 (ContactData.size(), Vector3d::Zero());
	VectorNd QDDot_0 = VectorNd::Zero(model.dof_count);
	VectorNd QDDot_t = VectorNd::Zero(model.dof_count);

	MatrixNd K = MatrixNd::Zero(ContactData.size(), ContactData.size());
	VectorNd f = VectorNd::Zero(ContactData.size());
	VectorNd a = VectorNd::Zero(ContactData.size());

	Vector3d point_accel_t;

	if (f_ext_constraints.size() != model.mBodies.size() + 1)
		f_ext_constraints.resize (model.mBodies.size() + 1, SpatialVectorZero);

	if (f_t.size() != ContactData.size()) {
		f_t.resize(ContactData.size(), SpatialVectorZero);
		point_accel_0.resize(ContactData.size(), Vector3d::Zero());
		K = MatrixNd::Zero (ContactData.size(), ContactData.size());
		f = VectorNd::Zero (ContactData.size());
		a = VectorNd::Zero (ContactData.size());
	}

	if (QDDot_0.size() != model.dof_count) {
		QDDot_0.resize(model.dof_count);
		QDDot_t.resize(model.dof_count);
	}

	unsigned int ci = 0;
	
	// The default acceleration only needs to be computed once
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, QDDot_0);
	}

	// The vector f_ext_constraints might contain some values from previous
	// computations so we need to reset it.
	for (unsigned int fi = 0; fi < f_ext_constraints.size(); fi++) {
			f_ext_constraints[fi].setZero();
		}

	// we have to compute the standard accelerations first as we use them to
	// compute the effects of each test force
	LOG << "=== Initial Loop Start ===" << std::endl;
	for (ci = 0; ci < ContactData.size(); ci++) {
		unsigned int body_id = ContactData[ci].body_id;
		Vector3d point = ContactData[ci].point;
		Vector3d normal = ContactData[ci].normal;
		double acceleration = ContactData[ci].acceleration;

		{
			SUPPRESS_LOGGING;
			ForwardKinematicsCustom (model, NULL, NULL, &QDDot_0);
			point_accel_0[ci] = CalcPointAcceleration (model, Q, QDot, QDDot_0, body_id, point, false);

			a[ci] = acceleration - ContactData[ci].normal.dot(point_accel_0[ci]);
		}
		LOG << "point_accel_0 = " << point_accel_0[ci].transpose();
	}

	// Now we can compute and apply the test forces and use their net effect
	// to compute the inverse articlated inertia to fill K.
	for (ci = 0; ci < ContactData.size(); ci++) {
		LOG << "=== Testforce Loop Start ===" << std::endl;
		unsigned int body_id = ContactData[ci].body_id;
		Vector3d point = ContactData[ci].point;
		Vector3d normal = ContactData[ci].normal;
		double acceleration = ContactData[ci].acceleration;

		// assemble the test force
		LOG << "normal = " << normal.transpose() << std::endl;

		Vector3d point_global = model.CalcBodyToBaseCoordinates(body_id, point);
		LOG << "point_global = " << point_global.transpose() << std::endl;

		f_t[ci].set (0., 0., 0., -normal[0], -normal[1], -normal[2]);
		f_t[ci] = spatial_adjoint(Xtrans(-point_global)) * f_t[ci];
		f_ext_constraints[body_id] = f_t[ci];

		LOG << "f_t[" << ci << "] (i = ci) = " << f_t[ci].transpose() << std::endl;
		LOG << "f_t[" << body_id << "] (i = body_id) = " << f_t[body_id].transpose() << std::endl;

		{
//			SUPPRESS_LOGGING;
//			ForwardDynamicsAccelerationsOnly (model, QDDot_t, &f_ext_constraints);
			ForwardDynamics (model, Q, QDot, Tau, QDDot_t, &f_ext_constraints);
			LOG << "QDDot_0 = " << QDDot_0.transpose() << std::endl;
			LOG << "QDDot_t = " << QDDot_t.transpose() << std::endl;
			LOG << "QDDot_t - QDDot_0= " << (QDDot_t - QDDot_0).transpose() << std::endl;
		}
		f_ext_constraints[body_id].setZero();

		// compute the resulting acceleration
		{
			SUPPRESS_LOGGING;
			ForwardKinematicsCustom (model, NULL, NULL, &QDDot_t);
		}

		for (unsigned int cj = 0; cj < ContactData.size(); cj++) {
			{
				SUPPRESS_LOGGING;

				point_accel_t = CalcPointAcceleration (model, Q, QDot, QDDot_t, ContactData[cj].body_id, ContactData[cj].point, false);
			}
	
			LOG << "point_accel_0  = " << point_accel_0[ci].transpose() << std::endl;
			K(cj,ci) = ContactData[cj].normal.dot(- point_accel_0[cj] + point_accel_t);
			LOG << "point_accel_t = " << point_accel_t.transpose() << std::endl;
		}
	}

	LOG << "K = " << std::endl << K << std::endl;
	LOG << "a = " << std::endl << a << std::endl;

#ifndef RBDL_USE_SIMPLE_MATH
//	f = K.ldlt().solve (a);
	f = K.colPivHouseholderQr().solve (a);
#else
	bool solve_successful = LinSolveGaussElimPivot (K, a, f);
	assert (solve_successful);
#endif

	LOG << "f = " << f << std::endl;

	for (unsigned int i = 0; i < f_ext_constraints.size(); i++) {
		f_ext_constraints[i].setZero();
	}

	for (ci = 0; ci < ContactData.size(); ci++) {
		ContactData[ci].force = f[ci];
		unsigned int body_id = ContactData[ci].body_id;

		f_ext_constraints[body_id] += f_t[ci] * f[ci]; 
		LOG << "f_ext[" << body_id << "] = " << f_ext_constraints[body_id].transpose() << std::endl;
	}

	{
		SUPPRESS_LOGGING;
//		ForwardDynamicsAccelerationsOnly (model, QDDot, &f_ext_constraints);
		ForwardDynamics (model, Q, QDot, Tau, QDDot, &f_ext_constraints);
	}
}

/** \brief Computes the effect of external forces on the generalized accelerations.
 *
 * This function is essentially similar to ForwardDynamics() except that it
 * tries to only perform computations of variables that change due to
 * external forces defined in f_t.
 */
void ForwardDynamicsAccelerationDeltas (
		Model &model,
		VectorNd &QDDot_t,
		const unsigned int body_id,
		const std::vector<SpatialVector> &f_t
		) {
	LOG << "-------- " << __func__ << " ------" << std::endl;
	
	std::vector<SpatialVector> d_p_v (model.mBodies.size(), SpatialVectorZero);
	std::vector<SpatialVector> d_pA (model.mBodies.size(), SpatialVectorZero);
	std::vector<SpatialVector> d_a (model.mBodies.size(), SpatialVectorZero);
	std::vector<double> d_u (model.mBodies.size(), 0.);

	if (d_p_v.size() != model.mBodies.size()) {
		d_p_v.resize(model.mBodies.size());
		d_pA.resize(model.mBodies.size());
		d_a.resize(model.mBodies.size());
		d_u.resize(model.mBodies.size());
	}

	// TODO reset all values (debug)
	for (unsigned int i = 0; i < model.mBodies.size(); i++) {
		d_p_v[i].setZero();
		d_pA[i].setZero();
		d_a[i].setZero();
		d_u[i] = 0.;
	}

	for (unsigned int i = body_id; i > 0; i--) {
		if (i == body_id) {
			d_p_v[i] = -spatial_adjoint(model.X_base[i]) * f_t[i];
			d_pA[i] = d_p_v[i];
		}

		d_u[i] = - model.S[i].dot(d_pA[i]);

		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			d_pA[lambda] = d_pA[lambda] + model.X_lambda[i].transpose() * (d_pA[i] + model.U[i] * d_u[i] / model.d[i]);
		}
	}

	for (unsigned int i = 0; i < f_t.size(); i++) {
		LOG << "f_t[" << i << "] = " << f_t[i].transpose();
	}

	for (unsigned int i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_pA[i] " << d_pA[i].transpose();
	}
	for (unsigned int i = 0; i < model.mBodies.size(); i++) {
		LOG << "i = " << i << ": d_u[i] = " << d_u[i] << std::endl;
	}

	QDDot_t[0] = 0.;
	d_a[0] = model.a[0];

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];

		SpatialVector Xa = model.X_lambda[i] * d_a[lambda];
		QDDot_t[i - 1] = (d_u[i] - model.U[i].dot(Xa) ) / model.d[i];
		d_a[i] = Xa + model.S[i] * QDDot_t[i - 1];

		LOG << "QDDot_t[" << i - 1 << "] = " << QDDot_t[i - 1] << std::endl;
		LOG << "d_a[i] = " << d_a[i].transpose() << std::endl;
	}
}

void ForwardDynamicsContactsOpt (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		std::vector<ContactInfo> &ContactData,
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " ------" << std::endl;

//	LOG << "Q    = " << Q.transpose() << std::endl;
//	LOG << "QDot = " << QDot.transpose() << std::endl;
//	assert (ContactData.size() == 1);
	std::vector<SpatialVector> f_t (ContactData.size(), SpatialVectorZero);
	std::vector<SpatialVector> f_ext_constraints (model.mBodies.size(), SpatialVectorZero);
	std::vector<Vector3d> point_accel_0 (ContactData.size(), Vector3d::Zero());
	VectorNd QDDot_0 = VectorNd::Zero(model.dof_count);
	VectorNd QDDot_t = VectorNd::Zero(model.dof_count);

	MatrixNd K = MatrixNd::Zero(ContactData.size(), ContactData.size());
	VectorNd f = VectorNd::Zero(ContactData.size());
	VectorNd a = VectorNd::Zero(ContactData.size());

	Vector3d point_accel_t;

	if (f_ext_constraints.size() != model.mBodies.size() + 1)
		f_ext_constraints.resize (model.mBodies.size() + 1, SpatialVectorZero);

	if (f_t.size() != ContactData.size()) {
		f_t.resize(ContactData.size(), SpatialVectorZero);
		point_accel_0.resize(ContactData.size(), Vector3d::Zero());
		K = MatrixNd::Zero (ContactData.size(), ContactData.size());
		f = VectorNd::Zero (ContactData.size());
		a = VectorNd::Zero (ContactData.size());
	}

	if (QDDot_0.size() != model.dof_count) {
		QDDot_0.resize(model.dof_count);
		QDDot_t.resize(model.dof_count);
	}

	unsigned int ci = 0;
	
	// The default acceleration only needs to be computed once
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, QDDot_0);
	}

	// The vector f_ext_constraints might contain some values from previous
	// computations so we need to reset it.
	for (unsigned int fi = 0; fi < f_ext_constraints.size(); fi++) {
			f_ext_constraints[fi].setZero();
		}

	LOG << "=== Initial Loop Start ===" << std::endl;
	// we have to compute the standard accelerations first as we use them to
	// compute the effects of each test force
	for (ci = 0; ci < ContactData.size(); ci++) {
		unsigned int body_id = ContactData[ci].body_id;
		Vector3d point = ContactData[ci].point;
		Vector3d normal = ContactData[ci].normal;
		double acceleration = ContactData[ci].acceleration;

		{
			SUPPRESS_LOGGING;
			ForwardKinematicsCustom (model, NULL, NULL, &QDDot_0);
			point_accel_0[ci] = CalcPointAcceleration (model, Q, QDot, QDDot_0, body_id, point, false);

			a[ci] = acceleration - ContactData[ci].normal.dot(point_accel_0[ci]);
		}
		LOG << "point_accel_0 = " << point_accel_0[ci].transpose();
	}

	// Now we can compute and apply the test forces and use their net effect
	// to compute the inverse articlated inertia to fill K.
	for (ci = 0; ci < ContactData.size(); ci++) {
		LOG << "=== Testforce Loop Start ===" << std::endl;
		unsigned int body_id = ContactData[ci].body_id;
		Vector3d point = ContactData[ci].point;
		Vector3d normal = ContactData[ci].normal;
		double acceleration = ContactData[ci].acceleration;

		// assemble the test force
		LOG << "normal = " << normal.transpose() << std::endl;

		Vector3d point_global = model.CalcBodyToBaseCoordinates(body_id, point);
		LOG << "point_global = " << point_global.transpose() << std::endl;

		f_t[ci].set (0., 0., 0., -normal[0], -normal[1], -normal[2]);
		f_t[ci] = spatial_adjoint(Xtrans(-point_global)) * f_t[ci];
		f_ext_constraints[body_id] = f_t[ci];
		LOG << "f_t[" << body_id << "] = " << f_t[ci].transpose() << std::endl;

		{
//			SUPPRESS_LOGGING;
			ForwardDynamicsAccelerationDeltas (model, QDDot_t, body_id, f_ext_constraints);
			LOG << "QDDot_0 = " << QDDot_0.transpose() << std::endl;
			LOG << "QDDot_t = " << (QDDot_t + QDDot_0).transpose() << std::endl;
			LOG << "QDDot_t - QDDot_0= " << (QDDot_t).transpose() << std::endl;
		}
		f_ext_constraints[body_id].setZero();

		////////////////////////////
		QDDot_t += QDDot_0;

		// compute the resulting acceleration
		{
			SUPPRESS_LOGGING;
			ForwardKinematicsCustom (model, NULL, NULL, &QDDot_t);
		}

		for (unsigned int cj = 0; cj < ContactData.size(); cj++) {
			{
				SUPPRESS_LOGGING;

				point_accel_t = CalcPointAcceleration (model, Q, QDot, QDDot_t, ContactData[cj].body_id, ContactData[cj].point, false);
			}
	
			LOG << "point_accel_0  = " << point_accel_0[ci].transpose() << std::endl;
			K(ci,cj) = ContactData[cj].normal.dot(point_accel_t - point_accel_0[cj]);
			LOG << "point_accel_t = " << point_accel_t.transpose() << std::endl;
		}
		//////////////////////////////////

		/*
		// update the spatial accelerations due to the test force
		for (unsigned j = 1; j < model.mBodies.size(); j++) {
			if (model.lambda[j] != 0) {
				model.a[j] = model.X_lambda[j] * model.a[model.lambda[j]] + model.c[j];
			}	else {
				model.a[j].setZero();
			}

			model.a[j] = model.a[j] + model.S[j] * QDDot_t[j - 1];
		}
		*/

		/////////////i
		/*

		QDDot_t += QDDot_0;

		{
			SUPPRESS_LOGGING;
			ForwardKinematicsCustom (model, NULL, NULL, &QDDot_t);
		}
		
		LOG << "SUUUHUUM = " << (QDDot_t).transpose() << std::endl;
	
		for (unsigned int cj = 0; cj < ContactData.size(); cj++) {
			static SpatialVector point_spatial_acc;
			{
				SUPPRESS_LOGGING;

				// computation of the net effect (acceleration due to the test
				// force)

				// method 1: simply compute the acceleration by calling
				// CalcPointAcceleration() (slow)
				point_accel_t = CalcPointAcceleration (model, Q, QDot, QDDot_t, ContactData[cj].body_id, ContactData[cj].point, false);

				// method 2: transforming the spatial acceleration
				// appropriately.(faster: 
//				point_global = model.CalcBodyToBaseCoordinates(ContactData[cj].body_id, ContactData[cj].point);
//				point_spatial_acc = Xtrans (point_global) * (spatial_inverse(model.X_base[ContactData[cj].body_id]) * model.a[ContactData[cj].body_id]);
//				point_accel_t.set (point_spatial_acc[3], point_spatial_acc[4], point_spatial_acc[5]);

				// method 3: reduce 1 Matrix-Matrix computation:
				// \todo currently broken!
//				point_global = model.CalcBodyToBaseCoordinates(ContactData[cj].body_id, ContactData[cj].point);
//				point_spatial_acc = spatial_inverse(model.X_base[ContactData[cj].body_id]) * model.a[ContactData[cj].body_id];
//
//				Matrix3d rx (0., point_global[2], -point_global[1], -point_global[2], 0, point_global[0], point_global[1], -point_global[0], 0.);
//				Matrix3d R = model.X_base[ContactData[cj].body_id].block<3,3>(0,0).transpose();
//				point_accel_t = rx * R * Vector3d (point_spatial_acc[0], point_spatial_acc[1], point_spatial_acc[2]) + Vector3d(point_spatial_acc[3], point_spatial_acc[4], point_spatial_acc[5]) ;
			}

			LOG << "point_spatial_a= " << point_spatial_acc.transpose() << std::endl;
			LOG << "point_accel_0  = [" << point_accel_0.transpose() << " ]" << std::endl;
			K(ci,cj) = ContactData[cj].normal.dot(point_accel_t);
			LOG << "point_accel_t = [" << (point_accel_t).transpose() << " ]" << std::endl;
		}
		*/
		////////////////
	}

	LOG << "K = " << std::endl << K << std::endl;
	LOG << "a = " << std::endl << a << std::endl;

#ifndef RBDL_USE_SIMPLE_MATH
//	f = K.ldlt().solve (a);
	f = K.colPivHouseholderQr().solve (a);
#else
	bool solve_successful = LinSolveGaussElimPivot (K, a, f);
	assert (solve_successful);
#endif

	LOG << "f = " << f << std::endl;

	for (ci = 0; ci < ContactData.size(); ci++) {
		ContactData[ci].force = f[ci];
		unsigned int body_id = ContactData[ci].body_id;

		f_ext_constraints[body_id] += f_t[ci] * f[ci]; 
		LOG << "f_ext[" << body_id << "] = " << f_ext_constraints[body_id].transpose() << std::endl;
	}

	{
		SUPPRESS_LOGGING;
		ForwardDynamicsAccelerationsOnly (model, QDDot, &f_ext_constraints);
	}
}


} /* namespace Experimental */

} /* namespace RigidBodyDynamics */
