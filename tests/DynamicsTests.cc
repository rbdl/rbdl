#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"
#include "Dynamics.h"

using namespace std;
using namespace SpatialAlgebra;
using namespace RigidBodyDynamics;

const double TEST_PREC = 1.0e-14;

TEST (TestForwardDynamicsLagrangian) {
	Model model;
	model.Init();
	Body base_body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

	model.SetFloatingBaseBody(base_body);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);
	
	VectorNd QDDot_aba = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDDot_lagrangian = VectorNd::Constant ((size_t) model.dof_count, 0.);

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

	ForwardDynamics(model, Q, QDot, Tau, QDDot_aba);
	ForwardDynamicsLagrangian(model, Q, QDot, Tau, QDDot_lagrangian);

	CHECK_ARRAY_CLOSE (QDDot_aba.data(), QDDot_lagrangian.data(), QDDot_aba.size(), TEST_PREC);
}

