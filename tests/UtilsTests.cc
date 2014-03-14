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
	Body body (1., Vector3d (0., 0., 0.), Matrix3d::Zero());
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

	CHECK_EQUAL (9.81, potential_energy_lifted);
}
