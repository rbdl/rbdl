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

// forward declaration
namespace Experimental {

void ForwardDynamicsFloatingBase (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot
		);
}

void ForwardDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot,
		std::vector<SpatialAlgebra::SpatialVector> *f_ext
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
		SpatialMatrix X_J;
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

		model.v[i] = model.X_lambda[i] * model.v.at(lambda) + v_J;

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
			LOG << "External force (" << i << ") = " << spatial_adjoint(model.X_base[i]) * (*f_ext)[i] << std::endl;
			model.pA[i] -= spatial_adjoint(model.X_base[i]) * (*f_ext)[i];
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
			SpatialMatrix X_lambda = model.X_lambda[i];

			// note: X_lambda.inverse().spatial_adjoint() = X_lambda.transpose()
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

	LOG << std::endl << "--- third loop ---" << std::endl;

	LOG << "spatial gravity = " << spatial_gravity.transpose() << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		SpatialMatrix X_lambda = model.X_lambda[i];

		if (lambda == 0) {
			model.a[i] = X_lambda * spatial_gravity * (-1.) + model.c[i];
		} else {
			model.a[i] = X_lambda * model.a[lambda] + model.c[i];
		}

		// we can skip further processing if the joint type is fixed
		if (model.mJoints[i].mJointType == JointTypeFixed) {
			model.qddot[i] = 0.;
			continue;
		}

		model.qddot[i] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "c[" << i << "] = " << model.c[i] << std::endl;
	}

	LOG << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "a[" << i << "] = " << model.a[i] << std::endl;
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
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	MatrixNd H = MatrixNd::Zero(model.dof_count, model.dof_count);
	VectorNd C = VectorNd::Zero(model.dof_count);

	// we set QDDot to zero to compute C properly with the InverseDynamics
	// method.
	QDDot.setZero();

	// we first have to call InverseDynamics as it will update the spatial
	// joint axes which CRBA does not do on its own!
	InverseDynamics (model, Q, QDot, QDDot, C);
	CompositeRigidBodyAlgorithm (model, Q, H);

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
		std::vector<SpatialAlgebra::SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	if (model.experimental_floating_base)
		assert (0 && !"InverseDynamics not supported for experimental floating base models!");

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
		SpatialMatrix X_J;
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
			model.a[i] = model.X_base[i] * spatial_gravity * -1. + model.S[i] * model.qddot[i];
		}	else {
			model.X_base[i] = model.X_lambda[i] * model.X_base.at(lambda);
			model.v[i] = model.X_lambda[i] * model.v[lambda] + v_J;
			model.c[i] = c_J + crossm(model.v[i],v_J);
			model.a[i] = model.X_lambda[i] * model.a[lambda] + model.S[i] * model.qddot[i] + model.c[i];
		}

		LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
		LOG << "v (" << i << "):" << std::endl << v_J << std::endl;
		LOG << "a (" << i << "):" << std::endl << v_J << std::endl;

		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i] + crossf(model.v[i],model.mBodies[i].mSpatialInertia * model.v[i]);
		if (f_ext != NULL && (*f_ext)[i] != SpatialVectorZero)
			model.f[i] -= spatial_adjoint(model.X_base[i]) * (*f_ext)[i];
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		model.tau[i] = model.S[i].dot(model.f[i]);
		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			model.f[lambda] = model.f[lambda] + model.X_lambda[i].transpose() * model.f[i];
		}
	}

	for (i = 0; i < Tau.size(); i++) {
		Tau[i] = model.tau[i + 1];
	}
}

void CompositeRigidBodyAlgorithm (Model& model, const VectorNd &Q, MatrixNd &H) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	if (H.rows() != Q.size() || H.cols() != Q.size()) 
		H.resize(Q.size(), Q.size());

	H.setZero();

	unsigned int i;
	for (i = 1; i < model.mBodies.size(); i++) {
		model.Ic[i] = model.mBodies[i].mSpatialInertia;
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].transpose() * model.Ic[i] * model.X_lambda[i];
		}

		SpatialVector F = model.Ic[i] * model.S[i];
		H(i - 1, i - 1) = model.S[i].dot(F);
		unsigned int j = i;

		while (model.lambda[j] != 0) {
			F = model.X_lambda[j].transpose() * F;
			j = model.lambda[j];
			H(i - 1,j - 1) = F.dot(model.S[j]);
			H(j - 1,i - 1) = H(i - 1,j - 1);
		}
	}
}

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
	x = A.colPivHouseholderQr().solve (b);
#else
	bool solve_successful = LinSolveGaussElimPivot (A, b, x);
	assert (solve_successful);
#endif

	LOG << "x = " << std::endl << x << std::endl;

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

void ComputeAccelerationDeltas (
		Model &model,
		const unsigned int body_id,
		const SpatialAlgebra::SpatialVector &f_t,
		VectorNd &QDDot_t
	) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	static std::vector<SpatialVector> d_pv;
	static std::vector<SpatialVector> d_p;
	static std::vector<SpatialVector> d_a;
	static std::vector<double> d_u;

	assert (QDDot_t.size() == model.dof_count);

	// check for proper sizes and perform resizes if necessary
	if (d_pv.size() != model.dof_count + 1) {
		d_pv.resize (model.dof_count + 1);
		d_p.resize (model.dof_count + 1);
		d_a.resize (model.dof_count + 1);
		d_u.resize (model.dof_count + 1);
	}

	unsigned int i;

	for (i = 1; i < model.mBodies.size(); i++) {
		model.IA[i] = model.mBodies[i].mSpatialInertia;
		d_p[i] = crossf(model.v[i], model.mBodies[i].mSpatialInertia * model.v[i]);

		if (i == body_id) {
			d_p[i] -= spatial_adjoint(model.X_base[i]) * f_t;
			LOG << "f_t (local)[" << i << "] = " << spatial_adjoint(model.X_base[i]) * f_t << std::endl;
		}
//		LOG << "i = " << i << " d_p[i] = " << d_p[i].transpose() << std::endl;
	}

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		// we can skip further processing if the joint is fixed
		if (model.mJoints[i].mJointType == JointTypeFixed)
			continue;

		d_u[i] = model.tau[i] - model.S[i].dot(d_p[i]);

		unsigned int lambda = model.lambda[i];
		if (lambda != 0) {
			SpatialVector pa = d_p[i] + model.U[i] * d_u[i] / model.d[i];
			SpatialMatrix X_lambda = model.X_lambda[i];

			// note: X_lambda.inverse().spatial_adjoint() = X_lambda.transpose()
			d_p[lambda] = d_p[lambda] + X_lambda.transpose() * pa;
		}
	}

	// apply gravity
	d_a[0].set (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		SpatialMatrix X_lambda = model.X_lambda[i];

		d_a[i] = X_lambda * d_a[lambda];
//		LOG << "if = " << i << " d_a[i] = " << d_a[i].transpose() << std::endl;

		// we can skip further processing if the joint type is fixed
		if (model.mJoints[i].mJointType == JointTypeFixed) {
			model.qddot[i] = 0.;
			continue;
		}

		QDDot_t[i - 1] = (1./model.d[i]) * (d_u[i] - model.U[i].dot(d_a[i]));
		d_a[i] = d_a[i] + model.S[i] * QDDot_t[i - 1];
//		LOG << "i = " << i << " d_a[i] = " << d_a[i].transpose() << std::endl;
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
	LOG << "-------- ForwardDynamicsContacts ------" << std::endl;

//	assert (ContactData.size() == 1);

	VectorNd QDDot_0 = VectorNd::Zero(model.dof_count);
	VectorNd QDDot_t = VectorNd::Zero(model.dof_count);
	std::vector<SpatialVector> f_t (ContactData.size(), SpatialVectorZero);
	MatrixNd K = MatrixNd::Zero(ContactData.size(), ContactData.size());
	VectorNd f = VectorNd::Zero(ContactData.size());
	VectorNd a = VectorNd::Zero(ContactData.size());

	Vector3d point_accel_0, point_accel_t;
	double k;

	unsigned int ci = 0;

	for (ci = 0; ci < ContactData.size(); ci++) {
		LOG << "=== Loop Start ===" << std::endl;
		unsigned int body_id = ContactData[ci].body_id;
		Vector3d point = ContactData[ci].point;
		Vector3d normal = ContactData[ci].normal;
		double acceleration = ContactData[ci].acceleration;

		{
			SUPPRESS_LOGGING;
			ForwardDynamics (model, Q, QDot, Tau, QDDot_0);
			point_accel_0 = CalcPointAcceleration (model, Q, QDot, QDDot_0, body_id, point, true);
			a[ci] = acceleration - ContactData[ci].normal.dot(point_accel_0);
		}
		LOG << "point_accel_0 = " << point_accel_0.transpose() << std::endl;

		// assemble the test force
		Vector3d contact_normal = normal;
		LOG << "contact_normal = " << contact_normal.transpose() << std::endl;

		Vector3d point_global = model.CalcBodyToBaseCoordinates(body_id, point);
		f_t[ci].set (0., 0., 0., -contact_normal[0], -contact_normal[1], -contact_normal[2]);
		f_t[ci] = spatial_adjoint(Xtrans(Vector3d (-point_global))) * f_t[ci];
		LOG << "f_t[" << ci << "] = " << f_t[ci].transpose() << std::endl;

		{
//			SUPPRESS_LOGGING;
			ComputeAccelerationDeltas (model, body_id, f_t[ci], QDDot_t);
		}

		// compute the resulting acceleration
		{
			SUPPRESS_LOGGING;
			ForwardKinematics (model, Q, QDot, QDDot_t);
		}

		for (unsigned int cj = 0; cj < ContactData.size(); cj++) {
			{
				SUPPRESS_LOGGING;
				point_accel_t = CalcPointAcceleration (model, Q, QDot, QDDot_t, ContactData[cj].body_id, ContactData[cj].point, false);
			}
			K(ci,cj) = ContactData[cj].normal.dot(point_accel_t - point_accel_0);
			LOG << "point_accel_t = " << point_accel_t.transpose() << std::endl;
		}
	}

	LOG << "K = " << std::endl << K << std::endl;
	LOG << "a = " << std::endl << a << std::endl;

#ifndef RBDL_USE_SIMPLE_MATH
	f = K.colPivHouseholderQr().solve (a);
#else
	bool solve_successful = LinSolveGaussElimPivot (K, a, f);
	assert (solve_successful);
#endif

	LOG << "f = " << f << std::endl;

	std::vector<SpatialVector> f_ext_constraints (model.mBodies.size(), SpatialVector (0., 0., 0., 0., 0., 0.));

	for (ci = 0; ci < ContactData.size(); ci++) {
		ContactData[ci].force = f[ci];
		f_ext_constraints[ContactData[ci].body_id] += f_t[ci] * f[ci];
		LOG << "f_ext[" << ContactData[ci].body_id << "] = " << f_ext_constraints[ContactData[ci].body_id].transpose() << std::endl;
	}

	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, QDDot, &f_ext_constraints);
	}
}

} /* namespace Experimental */

} /* namespace RigidBodyDynamics */
