#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Dynamics.h"

using namespace std;
using namespace SpatialAlgebra;
using namespace RigidBodyDynamics;

const double TEST_PREC = 1.0e-14;

struct CompositeRigidBodyFixture {
	CompositeRigidBodyFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();
		model->gravity = Vector3d (0., -9.81, 0.);
	}
	~CompositeRigidBodyFixture () {
		delete model;
	}
	Model *model;
};

struct CompositeRigidBody12DofFloatingBase {
	CompositeRigidBody12DofFloatingBase () {
		ClearLogOutput();
		model = new Model;
		model->Init();

		model->gravity = Vector3d  (0., -9.81, 0.);

		/* 3 DoF (rot.) joint at base
		 * 3 DoF (rot.) joint child origin
		 *
		 *          X Contact point (ref child)
		 *          |
		 *    Base  |
		 *   / body |
		 *  O-------*
		 *           \
		 *             Child body
		 */

		base_rot_x = Body (
				1.,
				Vector3d (0.5, 0., 0.),
				Vector3d (1., 1., 1.)
				);
		base_rot_x_id = model->SetFloatingBaseBody(base_rot_x);

		// child body 1 (3 DoF)
		child_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		child_rot_z_id = model->AddBody (base_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_rot_z, child_rot_z);

		child_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		child_rot_y_id = model->AddBody (child_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_y, child_rot_y);

		child_rot_x = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		child_rot_x_id = model->AddBody (child_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_x, child_rot_x);

		// child body (3 DoF)
		child_2_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_2_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		child_2_rot_z_id = model->AddBody (child_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_2_rot_z, child_2_rot_z);

		child_2_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_2_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		child_2_rot_y_id = model->AddBody (child_2_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_2_rot_y, child_2_rot_y);

		child_2_rot_x = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_2_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		child_2_rot_x_id = model->AddBody (child_2_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_2_rot_x, child_2_rot_x);

		Q = VectorNd::Constant (model->dof_count, 0.);
		QDot = VectorNd::Constant (model->dof_count, 0.);
		QDDot = VectorNd::Constant (model->dof_count, 0.);
		Tau = VectorNd::Constant (model->dof_count, 0.);

		ClearLogOutput();
	}
	
	~CompositeRigidBody12DofFloatingBase () {
		delete model;
	}
	Model *model;

	unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id,
		child_rot_z_id, child_rot_y_id, child_rot_x_id,
		child_2_rot_z_id, child_2_rot_y_id,child_2_rot_x_id,
		base_body_id;

	Body base_rot_z, base_rot_y, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x,
		child_2_rot_z, child_2_rot_y, child_2_rot_x;

	Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x,
		joint_child_2_rot_z, joint_child_2_rot_y, joint_child_2_rot_x;

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;
};

TEST_FIXTURE(CompositeRigidBodyFixture, TestCompositeRigidBodyForwardDynamicsFloatingBase) {
	Body base_body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

	model->SetFloatingBaseBody(base_body);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd TauInv = VectorNd::Constant ((size_t) model->dof_count, 0.);

	MatrixNd H = MatrixNd::Constant ((size_t) model->dof_count, (size_t) model->dof_count, 0.);
	VectorNd C = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot_zero = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot_crba = VectorNd::Constant ((size_t) model->dof_count, 0.);

	Q[0] = 1.1;
	Q[1] = 1.2;
	Q[2] = 1.3;
	Q[3] = 0.1;
	Q[4] = 0.2;
	Q[5] = 0.3;

	QDot[0] = 1.1;
	QDot[1] = -1.2;
	QDot[2] = 1.3;
	QDot[3] = -0.1;
	QDot[4] = 0.2;
	QDot[5] = -0.3;

	Tau[0] = 2.1;
	Tau[1] = 2.2;
	Tau[2] = 2.3;
	Tau[3] = 1.1;
	Tau[4] = 1.2;
	Tau[5] = 1.3;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);
	CompositeRigidBodyAlgorithm (*model, Q, H);
	InverseDynamics (*model, Q, QDot, QDDot_zero, C);

	CHECK (LinSolveGaussElimPivot (H, C * -1. + Tau, QDDot_crba));

	CHECK_ARRAY_CLOSE (QDDot.data(), QDDot_crba.data(), QDDot.size(), TEST_PREC);
}

/*
TEST_FIXTURE(CompositeRigidBody12DofFloatingBase, TestCRBAFloatingBase12DoFInverseDynamics) {
	MatrixNd H_crba = MatrixNd::Zero ((size_t) model->dof_count, (size_t) model->dof_count);
	MatrixNd H_id = MatrixNd::Zero ((size_t) model->dof_count, (size_t) model->dof_count);

	Q[ 0] = 1.1;
	Q[ 1] = 1.2;
	Q[ 2] = 1.3;
	Q[ 3] = 0.1;
	Q[ 4] = 0.2;
	Q[ 5] = 0.3;
	Q[ 6] = -1.3;
	Q[ 7] = -1.4;
	Q[ 8] = -1.5;
	Q[ 9] = -0.3;
	Q[10] = -0.4;
	Q[11] = -0.5;

	QDot.setZero();

	assert (model->dof_count == 12);

	CompositeRigidBodyAlgorithm (*model, Q, H_crba);

	VectorNd H_col = VectorNd::Zero (model->dof_count);
	VectorNd QDDot_zero = VectorNd::Zero (model->dof_count);

	unsigned int i;
	for (i = 0; i < model->dof_count; i++) {
		// compute each column
		VectorNd delta_a = VectorNd::Zero (model->dof_count);
		delta_a[i] = 1.;
		cout << delta_a << endl;

		// compute ID (model, q, qdot, delta_a)
		VectorNd id_all = VectorNd::Zero (model->dof_count);
		InverseDynamics (*model, Q, QDot, delta_a, id_all);

		// compute ID (model, q, qdot, zero)
		VectorNd id_zero = VectorNd::Zero (model->dof_count);
		InverseDynamics (*model, Q, QDot, QDDot_zero, id_zero);
	
		H_col = id_all - id_zero;
		cout << "H_col = " << H_col << endl;
		H_id.block<12, 1>(0, i) = H_col;
	}

	cout << "H (crba) = " << endl << H_crba << endl;
	cout << "H (id) = " << endl << H_id << endl;
}
*/
