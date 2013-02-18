#include <UnitTest++.h>

#include <iostream>

#include "rbdl/Logging.h"
#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"

#include "Fixtures.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-12;

struct CompositeRigidBodyFixture {
	CompositeRigidBodyFixture () {
		ClearLogOutput();
		model = new Model;
		model->gravity = Vector3d (0., -9.81, 0.);
	}
	~CompositeRigidBodyFixture () {
		delete model;
	}
	Model *model;
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

	ClearLogOutput();
	CompositeRigidBodyAlgorithm (*model, Q, H);
	// cout << LogOutput.str() << endl;

	InverseDynamics (*model, Q, QDot, QDDot_zero, C);

	CHECK (LinSolveGaussElimPivot (H, C * -1. + Tau, QDDot_crba));

	CHECK_ARRAY_CLOSE (QDDot.data(), QDDot_crba.data(), QDDot.size(), TEST_PREC);
}

TEST_FIXTURE(FloatingBase12DoF, TestCRBAFloatingBase12DoF) {
	MatrixNd H = MatrixNd::Zero ((size_t) model->dof_count, (size_t) model->dof_count);
	
	VectorNd C = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot_zero = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot_crba = VectorNd::Constant ((size_t) model->dof_count, 0.);


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

	QDot[ 0] =  1.1;
	QDot[ 1] = -1.2;
	QDot[ 2] =  1.3;
	QDot[ 3] = -0.1;
	QDot[ 4] =  0.2;
	QDot[ 5] = -0.3;
	QDot[ 6] = -1.1;
	QDot[ 7] =  1.2;
	QDot[ 8] = -1.3;
	QDot[ 9] =  0.1;
	QDot[10] = -0.2;
	QDot[11] =  0.3;

	Tau[ 0] = -1.1;
	Tau[ 1] =  1.2;
	Tau[ 2] = -1.3;
	Tau[ 3] =  1.1;
	Tau[ 4] = -1.2;
	Tau[ 5] =  1.3;
	Tau[ 6] =  0.1;
	Tau[ 7] = -0.2;
	Tau[ 8] =  0.3;
	Tau[ 9] = -0.1;
	Tau[10] =  0.2;
	Tau[11] = -0.3;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);
	ClearLogOutput();
	CompositeRigidBodyAlgorithm (*model, Q, H);
	// cout << LogOutput.str() << endl;
	InverseDynamics (*model, Q, QDot, QDDot_zero, C);

	CHECK (LinSolveGaussElimPivot (H, C * -1. + Tau, QDDot_crba));

	CHECK_ARRAY_CLOSE (QDDot.data(), QDDot_crba.data(), QDDot.size(), TEST_PREC);
}

TEST_FIXTURE(FloatingBase12DoF, TestCRBAFloatingBase12DoFInverseDynamics) {
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

	UpdateKinematicsCustom (*model, &Q, NULL, NULL);
	CompositeRigidBodyAlgorithm (*model, Q, H_crba, false);

	VectorNd H_col = VectorNd::Zero (model->dof_count);
	VectorNd QDDot_zero = VectorNd::Zero (model->dof_count);

	unsigned int i;
	for (i = 0; i < model->dof_count; i++) {
		// compute each column
		VectorNd delta_a = VectorNd::Zero (model->dof_count);
		delta_a[i] = 1.;
		// cout << delta_a << endl;

		// compute ID (model, q, qdot, delta_a)
		VectorNd id_delta = VectorNd::Zero (model->dof_count);
		InverseDynamics (*model, Q, QDot, delta_a, id_delta);

		// compute ID (model, q, qdot, zero)
		VectorNd id_zero = VectorNd::Zero (model->dof_count);
		InverseDynamics (*model, Q, QDot, QDDot_zero, id_zero);
	
		H_col = id_delta - id_zero;
		// cout << "H_col = " << H_col << endl;
		H_id.block<12, 1>(0, i) = H_col;
	}

//	cout << "H (crba) = " << endl << H_crba << endl;
//	cout << "H (id) = " << endl << H_id << endl;

	CHECK_ARRAY_CLOSE (H_crba.data(), H_id.data(), model->dof_count * model->dof_count, TEST_PREC);
}

TEST_FIXTURE(FixedBase6DoF, TestCRBAFloatingBase12DoFInverseDynamics) {
	MatrixNd H_crba = MatrixNd::Zero ((size_t) model->dof_count, (size_t) model->dof_count);
	MatrixNd H_id = MatrixNd::Zero ((size_t) model->dof_count, (size_t) model->dof_count);

	Q[ 0] = 1.1;
	Q[ 1] = 1.2;
	Q[ 2] = 1.3;
	Q[ 3] = 0.1;
	Q[ 4] = 0.2;
	Q[ 5] = 0.3;

	QDot.setZero();

	assert (model->dof_count == 6);

	UpdateKinematicsCustom (*model, &Q, NULL, NULL);
	CompositeRigidBodyAlgorithm (*model, Q, H_crba, false);

	VectorNd H_col = VectorNd::Zero (model->dof_count);
	VectorNd QDDot_zero = VectorNd::Zero (model->dof_count);

	unsigned int i;
	for (i = 0; i < 6; i++) {
		// compute each column
		VectorNd delta_a = VectorNd::Zero (model->dof_count);
		delta_a[i] = 1.;

		ClearLogOutput();
		// compute ID (model, q, qdot, delta_a)
		VectorNd id_delta = VectorNd::Zero (model->dof_count);
		InverseDynamics (*model, Q, QDot, delta_a, id_delta);

		// compute ID (model, q, qdot, zero)
		VectorNd id_zero = VectorNd::Zero (model->dof_count);
		InverseDynamics (*model, Q, QDot, QDDot_zero, id_zero);

		H_col.setZero();
		H_col = id_delta - id_zero;

		H_id.block<6, 1>(0, i) = H_col;
	}

	CHECK_ARRAY_CLOSE (H_crba.data(), H_id.data(), model->dof_count * model->dof_count, TEST_PREC);
}
