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

	for (int i = 0; i < q.size(); i++) {
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
	Body body (0.5, Vector3d (0., 0., 0.), Matrix3d::Zero());
	Joint joint (
			SpatialVector (0., 0., 0., 1., 0., 0.),
			SpatialVector (0., 0., 0., 0., 1., 0.),
			SpatialVector (0., 0., 0., 0., 0., 1.)
			);

	model.AppendBody (Xtrans (Vector3d::Zero()), joint, body);

	VectorNd q = VectorNd::Zero(model.q_size);

	ClearLogOutput();
	double potential_energy_zero = Utils::CalcPotentialEnergy (model, q);
//	cout << LogOutput.str() << endl;

	CHECK_EQUAL (0., potential_energy_zero);
	q[1] = 1.;

	ClearLogOutput();
	double potential_energy_lifted = Utils::CalcPotentialEnergy (model, q);
//	cout << LogOutput.str() << endl;

	CHECK_EQUAL (4.905, potential_energy_lifted);
}

TEST(TestCOMSimple) {
	Model model;
	Body body (123., Vector3d (0., 0., 0.), Matrix3d::Zero());
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

/*

TEST_FIXTURE (TwoArms12DoF, TestAngularMomentumSimple) {
	Vector3d momentum = Utils::CalcAngularMomentum (*model, q, qdot, true);

	CHECK_EQUAL (Vector3d (0., 0., 0.), momentum);

	qdot[0] = 1.;
	qdot[1] = 2.;
	qdot[2] = 3.;

	momentum = Utils::CalcAngularMomentum (*model, q, qdot, true);

	CHECK (momentum.norm() > 10);

	qdot[3] = -qdot[0];
	qdot[4] = -qdot[1];
	qdot[5] = -qdot[2];

	ClearLogOutput();
	momentum = Utils::CalcAngularMomentum (*model, q, qdot, true);
	cout << LogOutput.str() << endl;
	cout << "momentum: " << momentum.transpose() << endl;

	for (size_t i = 1; i < model->mBodies.size(); i++) {
		cout << "v[" << i << "]:  " << model->v[i].transpose() << endl;
	}

	for (size_t i = 1; i < model->mBodies.size(); i++) {
		cout << "hc[" << i << "]: " << model->hc[i].transpose() << endl;
	}

	for (size_t i = 1; i < model->mBodies.size(); i++) {
		cout << "X_T[" << i << "]: " << model->X_T[i].r.transpose() << endl;
	}

	for (size_t i = 1; i < model->mBodies.size(); i++) {
		cout << "X_lambda[" << i << "]: " << model->X_lambda[i].r.transpose() << endl;
	}
}

*/
