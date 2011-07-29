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
		VectorNd &QDDot
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

		if (model.f_ext[i] != SpatialVectorZero) {
			LOG << "External force (" << i << ") = " << spatial_adjoint(model.X_base[i]) * model.f_ext[i] << std::endl;
			model.pA[i] -= spatial_adjoint(model.X_base[i]) * model.f_ext[i];
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
		VectorNd &Tau
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

		model.f[i] = model.mBodies[i].mSpatialInertia * model.a[i] + crossf(model.v[i],model.mBodies[i].mSpatialInertia * model.v[i]) - spatial_adjoint(model.X_base[i]) * model.f_ext[i];
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

/** Prepares and computes forward dynamics by using ForwardDynamicsFloatingBaseExpl()
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internals joints (output)
 */
void ForwardDynamicsFloatingBase (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	VectorNd q_expl (Q.size() - 6);
	VectorNd qdot_expl (QDot.size() - 6);
	VectorNd tau_expl (Tau.size() - 6);
	VectorNd qddot_expl (QDDot.size() - 6);

	LOG << "Q = " << Q << std::endl;
	LOG << "QDot = " << QDot << std::endl;

	SpatialMatrix permutation (
			0., 0., 0., 0., 0., 1.,
			0., 0., 0., 0., 1., 0.,
			0., 0., 0., 1., 0., 0.,
			1., 0., 0., 0., 0., 0.,
			0., 1., 0., 0., 0., 0.,
			0., 0., 1., 0., 0., 0.
			);

	SpatialMatrix X_B = XtransRotZYXEuler (Vector3d (Q[0], Q[1], Q[2]), Vector3d (Q[3], Q[4], Q[5]));
	SpatialVector v_B (QDot[5], QDot[4], QDot[3], QDot[0], QDot[1], QDot[2]);
	SpatialVector a_B (0., 0., 0., 0., 0., 0.);

	SpatialVector f_B (Tau[5], Tau[4], Tau[3], Tau[0], Tau[1], Tau[2]);

	// we also have to add any external force onto 
	f_B += model.f_ext[0];

	LOG << "X_B = " << X_B << std::endl;
	LOG << "v_B = " << v_B << std::endl;
	LOG << "Tau = " << Tau << std::endl;

	unsigned int i;

	if (Q.size() > 6) {
		for (i = 0; i < q_expl.size(); i++) {
			q_expl[i] = Q[i + 6];
		}
		for (i = 0; i < qdot_expl.size(); i++) {
			qdot_expl[i] = QDot[i + 6];
		}

		for (i = 0; i < tau_expl.size(); i++) {
			tau_expl[i] = Tau[i + 6];
		}
	}
	
	ForwardDynamicsFloatingBaseExpl (model, q_expl, qdot_expl, tau_expl, X_B, v_B, f_B, a_B, qddot_expl);

	LOG << "FloatingBaseExplRes a_B = " << a_B << std::endl;

	// we have to transform the acceleration back to base coordinates
	a_B = spatial_inverse(X_B) * a_B;

	QDDot[0] = a_B[5];
	QDDot[1] = a_B[4];
	QDDot[2] = a_B[3];
	QDDot[3] = a_B[0];
	QDDot[4] = a_B[1];
	QDDot[5] = a_B[2];

	if (Q.size() > 6) {
		for (i = 0; i < qddot_expl.size(); i++) {
			QDDot[i + 6] = qddot_expl[i];
		}
	}
}

void ForwardDynamicsFloatingBaseExpl (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		const SpatialMatrix &X_B,
		const SpatialVector &v_B,
		const SpatialVector &f_B,
		SpatialVector &a_B,
		VectorNd &QDDot
		)
{
	assert (model.experimental_floating_base);

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i;

	// Copy state values from the input to the variables in model
	assert (model.dof_count == Q.size() + 6);
	assert (model.dof_count == QDot.size() + 6);
	assert (model.dof_count == QDDot.size() + 6);
	assert (model.dof_count == Tau.size() + 6);

	for (i = 0; i < Q.size(); i++) {
		model.q[i+1] = Q[i];
		model.qdot[i+1] = QDot[i];
		model.qddot[i+1] = QDDot[i];
		model.tau[i+1] = Tau[i];
	}

	// Reset the velocity of the root body
	model.v[0] = v_B;
	model.X_lambda[0] = X_B;
	model.X_base[0] = X_B;

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints[i];
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, model.S[i], v_J, c_J, model.q[i], model.qdot[i]);
//		SpatialMatrix X_T (joint.mJointTransform);
//		LOG << "X_T (" << i << "):" << std::endl << model.X_T[i] << std::endl;

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda != 0) 
			model.X_base[i] = model.X_lambda[i] * model.X_base.at(lambda);

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

		model.pA[i] = crossf(model.v[i],model.IA[i] * model.v[i]) - model.X_base[i].transpose() * model.f_ext[i];
	}

// ClearLogOutput();

	model.IA[0] = model.mBodies[0].mSpatialInertia;

	LOG << "v[0] = " << model.v[0] << std::endl;

	model.pA[0] = crossf(model.v[0],model.IA[0] * model.v[0]) - model.f_ext[0]; 

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
		model.d[i] = model.S[i].dot(model.U[i]);

		if (model.d[i] == 0. ) {
			std::cerr << "Warning d[i] == 0.!" << std::endl;
			continue;
		}

		unsigned int lambda = model.lambda[i];
		SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
		SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];
		SpatialMatrix X_lambda = model.X_lambda[i];

		// note: X_lambda.inverse().spatial_adjoint() = X_lambda.transpose()
		model.IA[lambda] = model.IA[lambda] + X_lambda.transpose() * Ia * X_lambda;
		model.pA[lambda] = model.pA[lambda] + X_lambda.transpose() * pa;
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

	// !!!
	// model.a[0] = SpatialLinSolve (model.IA[0], model.pA[0]) * -1.;
	model.a[0].setZero();

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];
		SpatialMatrix X_lambda = model.X_lambda[i];

		model.a[i] = X_lambda * model.a[lambda] + model.c[i];
		model.qddot[i] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		QDDot[i - 1] = model.qddot[i];
	}

	LOG << "spatial_gravity = " << spatial_gravity << std::endl;
#ifndef RBDL_USE_SIMPLE_MATH
	LOG << "X_B * spatial_gravity = " << X_B * spatial_gravity << std::endl;
	model.a[0] = X_B * spatial_gravity;
#endif

	a_B = model.a[0];
}

void ComputeContactForces (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		std::vector<ContactInfo> &ContactData,
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

	MatrixNd Ae;
	Ae.resize(contact_count, contact_count);
	VectorNd C0 (contact_count);
	VectorNd a0 (contact_count);

	std::vector<SpatialVector> current_f_ext (model.f_ext);
	std::vector<SpatialVector> zero_f_ext (model.f_ext.size(), SpatialVector (0., 0., 0., 0., 0., 0.));
	Vector3d gravity_backup = model.gravity;

	model.f_ext = zero_f_ext;

	LOG << "-------- ZERO_EXT ------" << std::endl;
	VectorNd QDDot_zero_ext (QDot);
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
			point_accel = CalcPointAcceleration (model, Q, QDot, QDDot_zero_ext, contact_info.body_id, contact_info.point);
		}

		// evaluate a0 and C0
//		double a0i = cml::dot(contact_info.normal,point_accel);
		double a0i = contact_info.normal.dot(point_accel);

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
		Vector3d contact_point_position = model.CalcBodyToBaseCoordinates(contact_info.body_id, contact_info.point);

		test_forces[cj] = spatial_adjoint(Xtrans (contact_point_position)) * test_force;
		LOG << "body_id         = " << contact_info.body_id << std::endl;

		// apply the test force
		model.f_ext[contact_info.body_id] = test_forces[cj];
		VectorNd QDDot_test_ext (QDot);

		LOG << "-------- TEST_EXT -------" << std::endl;
		LOG << "test_force_body = " << spatial_adjoint(Xtrans (model.GetBodyOrigin(contact_info.body_id) - contact_point_position)) * test_forces[cj] << std::endl;
		LOG << "test_force_base = " << test_forces[cj] << std::endl;
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
				point_test_accel = CalcPointAcceleration (model, Q, QDot, QDDot_test_ext, test_contact_info.body_id, test_contact_info.point);
			}

			// acceleration due to the test force
//			double a1j_i = cml::dot(test_contact_info.normal, point_test_accel);
			double a1j_i = test_contact_info.normal.dot(point_test_accel);
			LOG << "test_accel a1j_i = " << a1j_i << std::endl;
			LOG << "a0[ci] = " << a0[ci] << std::endl;
			Ae(ci,cj) = a1j_i - a0[ci];
			LOG << "updating (" << ci << ", " << cj << ") = " << Ae(ci,cj) << std::endl;
		}

		// clear the test force
		model.f_ext[contact_info.body_id].setZero();
	}
	
	// solve the system!!!
	VectorNd u (contact_count);

	LOG << "Ae = " << std::endl << Ae << std::endl;
	LOG << "C0 = " << C0 << std::endl;
	LinSolveGaussElimPivot (Ae, C0, u);

	// !!!
	u[0] = 8.81;
//	test_forces[0].setZero(); 

	LOG << "u = " << u << std::endl;

	// compute and apply the constraint forces to the system
	model.f_ext = current_f_ext;

	Fext = zero_f_ext;

	for (ci = 0; ci < contact_count; ci++) {
		ContactData[ci].force = u[ci];

		test_forces[ci] = test_forces[ci] * u[ci];
		// it is important to *add* the constraint force as multiple forces
		// might act on the same body
		Fext[ContactData[ci].body_id] += test_forces[ci];
		LOG << "test_forces[" << ci << "] = " << test_forces[ci] << std::endl;
		LOG << "f_ext[" << ContactData[ci].body_id << "] = " << Fext[ContactData[ci].body_id] << std::endl;
	}
}

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

		if (i == body_id)
			d_p[i] -= spatial_adjoint(model.X_base[i]) * f_t;
		LOG << "i = " << i << " d_p[i] = " << d_p[i].transpose() << std::endl;
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
		LOG << "if = " << i << " d_a[i] = " << d_a[i].transpose() << std::endl;

		// we can skip further processing if the joint type is fixed
		if (model.mJoints[i].mJointType == JointTypeFixed) {
			model.qddot[i] = 0.;
			continue;
		}

		QDDot_t[i - 1] = (1./model.d[i]) * (d_u[i] - model.U[i].dot(d_a[i]));
		d_a[i] = d_a[i] + model.S[i] * QDDot_t[i - 1];
		LOG << "i = " << i << " d_a[i] = " << d_a[i].transpose() << std::endl;
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

	assert (ContactData.size() == 1);

	std::vector<SpatialVector> contact_f_ext (model.f_ext.size(), SpatialVector (0., 0., 0., 0., 0., 0.));

	VectorNd QDDot_0 = VectorNd::Zero(model.dof_count);
	VectorNd QDDot_t = VectorNd::Zero(model.dof_count);

	Vector3d point_accel_0, point_accel_t;
	double k;

	unsigned int body_id;
	Vector3d point, normal;
	double acceleration;

	body_id = ContactData[0].body_id;
	point = ContactData[0].point;
	normal = ContactData[0].normal;
	acceleration = ContactData[0].acceleration;

	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, QDDot_0);
		point_accel_0 = CalcPointAcceleration (model, Q, QDot, QDDot_0, body_id, point, true);
	}
	LOG << "point_accel_0 = " << point_accel_0.transpose() << std::endl;

	// assemble the test force
	// \TODO properly transform the force with respect to the orientation
	// of the body!
	assert (normal == Vector3d (0., 1., 0.));
	Vector3d contact_normal = normal;

	SpatialVector f_t;
	f_t.set (0., 0., 0., contact_normal[0], contact_normal[1], contact_normal[2]);
	f_t = spatial_adjoint(Xtrans(Vector3d (1., 0., 0.))) * f_t;

	ComputeAccelerationDeltas (model, body_id, f_t, QDDot_t);

	// \TODO here we need to update the kinematics ... but this should be
	// circumvented ... at some point	
	{
		SUPPRESS_LOGGING;
		point_accel_t = CalcPointAcceleration (model, Q, QDot, QDDot_t, body_id, point, true);
	}
	LOG << "point_accel_t = " << point_accel_t.transpose() << std::endl;

	k = (acceleration - normal.dot(point_accel_0)) / (normal.dot(point_accel_t) - normal.dot(point_accel_0));

	LOG << "k = " << k << std::endl;

	ContactData[0].force = k;

	model.f_ext[body_id] = f_t * k;
	{
		SUPPRESS_LOGGING;
		ForwardDynamics (model, Q, QDot, Tau, QDDot);
	}
}

} /* namespace Experimental */

} /* namespace RigidBodyDynamics */
