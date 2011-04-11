#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Dynamics.h"
#include "Dynamics_experimental.h"

using namespace std;
using namespace SpatialAlgebra;
using namespace RigidBodyDynamics;

const double TEST_PREC = 1.0e-14;

struct InverseDynamicsFixture {
	InverseDynamicsFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();
		model->gravity.set (0., -9.81, 0.);
	}
	~InverseDynamicsFixture () {
		delete model;
	}
	Model *model;
};

TEST_FIXTURE(InverseDynamicsFixture, TestInverseForwardDynamicsFloatingBase) {
	Body base_body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

	model->SetFloatingBaseBody(base_body);

	// Initialization of the input vectors
	VectorNd Q ((size_t) model->dof_count, 0.);
	VectorNd QDot ((size_t) model->dof_count, 0.);
	VectorNd QDDot ((size_t) model->dof_count, 0.);
	VectorNd Tau ((size_t) model->dof_count, 0.);
	VectorNd TauInv ((size_t) model->dof_count, 0.);

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
	InverseDynamics(*model, Q, QDot, QDDot, TauInv);

	CHECK_ARRAY_CLOSE (Tau.data(), TauInv.data(), Tau.size(), TEST_PREC);
}

