#include <UnitTest++.h>

#include <iostream>

#include "Fixtures.h"
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/rbdl_utils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

TEST_FIXTURE(FloatingBase12DoF, TestKineticEnergy) {
	VectorNd q = VectorNd::Zero(model->q_size);
	VectorNd qdot = VectorNd::Zero(model->q_size);

	for (unsigned int i = 0; i < q.size(); i++) {
		q[i] = 0.1 * i;
		qdot[i] = 0.3 * i;
	}

	MatrixNd H = MatrixNd::Zero (model->q_size, model->q_size);
	CompositeRigidBodyAlgorithm (*model, q, H, true);

	double kinetic_energy_ref = 0.5 * qdot.transpose() * H * qdot;
	double kinetic_energy = Utils::CalcKineticEnergy (*model, q, qdot);

	CHECK_EQUAL (kinetic_energy_ref, kinetic_energy);
}

TEST(TestPotentialEnergy) {
	Model model;
	Matrix3d inertia = Matrix3d::Zero(3,3);
	Body body (0.5, Vector3d (0., 0., 0.), inertia);
	Joint joint (
			SpatialVector (0., 0., 0., 1., 0., 0.),
			SpatialVector (0., 0., 0., 0., 1., 0.),
			SpatialVector (0., 0., 0., 0., 0., 1.)
			);

	model.AppendBody (Xtrans (Vector3d::Zero()), joint, body);

	VectorNd q = VectorNd::Zero(model.q_size);
	double potential_energy_zero = Utils::CalcPotentialEnergy (model, q);
	CHECK_EQUAL (0., potential_energy_zero);

	q[1] = 1.;
	double potential_energy_lifted = Utils::CalcPotentialEnergy (model, q);
	CHECK_EQUAL (4.905, potential_energy_lifted);
}

TEST(TestCOMSimple) {
	Model model;
	Matrix3d inertia = Matrix3d::Zero(3,3);
	Body body (123., Vector3d (0., 0., 0.), inertia);
	Joint joint (
			SpatialVector (0., 0., 0., 1., 0., 0.),
			SpatialVector (0., 0., 0., 0., 1., 0.),
			SpatialVector (0., 0., 0., 0., 0., 1.)
			);

	model.AppendBody (Xtrans (Vector3d::Zero()), joint, body);

	VectorNd q = VectorNd::Zero(model.q_size);
	VectorNd qdot = VectorNd::Zero(model.qdot_size);

	double mass;
	Vector3d com;
	Vector3d com_velocity;
	Utils::CalcCenterOfMass (model, q, qdot, mass, com, &com_velocity);

	CHECK_EQUAL (123., mass);
	CHECK_EQUAL (Vector3d (0., 0., 0.), com);
	CHECK_EQUAL (Vector3d (0., 0., 0.), com_velocity);

	q[1] = 1.;
	Utils::CalcCenterOfMass (model, q, qdot, mass, com, &com_velocity);
	CHECK_EQUAL (Vector3d (0., 1., 0.), com);
	CHECK_EQUAL (Vector3d (0., 0., 0.), com_velocity);

	qdot[1] = 1.;
	Utils::CalcCenterOfMass (model, q, qdot, mass, com, &com_velocity);
	CHECK_EQUAL (Vector3d (0., 1., 0.), com);
	CHECK_EQUAL (Vector3d (0., 1., 0.), com_velocity);
}

TEST(TestAngularMomentumSimple) {
	Model model;
	Matrix3d inertia = Matrix3d::Zero(3,3);
	inertia(0,0) = 1.1;
	inertia(1,1) = 2.2;
	inertia(2,2) = 3.3;

	Body body (0.5, Vector3d (1., 0., 0.), inertia);
	Joint joint (
			SpatialVector (1., 0., 0., 0., 0., 0.),
			SpatialVector (0., 1., 0., 0., 0., 0.),
			SpatialVector (0., 0., 1., 0., 0., 0.)
			);

	model.AppendBody (Xtrans (Vector3d(0., 0., 0.)), joint, body);

	VectorNd q = VectorNd::Zero(model.q_size);
	VectorNd qdot = VectorNd::Zero(model.qdot_size);

	Vector3d angular_momentum;

	qdot << 1., 0., 0.;
	angular_momentum = Utils::CalcAngularMomentum (model, q, qdot);
	CHECK_EQUAL (Vector3d (1.1, 0., 0.), angular_momentum);

	qdot << 0., 1., 0.;
	angular_momentum = Utils::CalcAngularMomentum (model, q, qdot);
	CHECK_EQUAL (Vector3d (0., 2.2, 0.), angular_momentum);

	qdot << 0., 0., 1.;
	angular_momentum = Utils::CalcAngularMomentum (model, q, qdot);
	CHECK_EQUAL (Vector3d (0., 0., 3.3), angular_momentum);
}

TEST_FIXTURE (TwoArms12DoF, TestAngularMomentumSimple) {
	Vector3d angular_momentum;

	angular_momentum = Utils::CalcAngularMomentum (*model, q, qdot);

	CHECK_EQUAL (Vector3d (0., 0., 0.), angular_momentum);

	qdot[0] = 1.;
	qdot[1] = 2.;
	qdot[2] = 3.;

	angular_momentum = Utils::CalcAngularMomentum (*model, q, qdot);

	// only a rough guess from test calculation
	CHECK_ARRAY_CLOSE (Vector3d (3.3, 2.54, 1.5).data(), angular_momentum.data(), 3, 1.0e-1);

	qdot[3] = -qdot[0];
	qdot[4] = -qdot[1];
	qdot[5] = -qdot[2];

	ClearLogOutput();
	angular_momentum = Utils::CalcAngularMomentum (*model, q, qdot);

	CHECK (angular_momentum[0] == 0);
	CHECK (angular_momentum[1] < 0);
	CHECK (angular_momentum[2] == 0.);
}
