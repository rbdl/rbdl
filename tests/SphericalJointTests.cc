#include <UnitTest++.h>

#include <iostream>

#include "Fixtures.h"
#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

struct SphericalJoint {
	SphericalJoint () {
		ClearLogOutput();

		emulated_model.gravity = Vector3d (0., 0., -9.81); 
		spherical_model.gravity = Vector3d (0., 0., -9.81); 

		Body body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

		Joint joint_rot_zyx (
				SpatialVector (0., 0., 1., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.)
				);
		Joint joint_spherical (JointTypeSpherical);

		Joint joint_rot_y (SpatialVector (0., 1., 0., 0., 0., 0.));

		emulated_model.AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_rot_zyx, body);
		emulated_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_rot_y, body);

		spherical_model.AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_spherical, body);
		spherical_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_rot_y, body);

		emuQ = VectorNd::Zero ((size_t) emulated_model.q_size);
		emuQDot = VectorNd::Zero ((size_t) emulated_model.qdot_size);
		emuQDDot = VectorNd::Zero ((size_t) emulated_model.qdot_size);
		emuTau = VectorNd::Zero ((size_t) emulated_model.qdot_size);

		sphQ = VectorNd::Zero ((size_t) spherical_model.q_size);
		sphQDot = VectorNd::Zero ((size_t) spherical_model.qdot_size);
		sphQDDot = VectorNd::Zero ((size_t) spherical_model.qdot_size);
		sphTau = VectorNd::Zero ((size_t) spherical_model.qdot_size);
	}

	Joint joint_rot_zyx;
	Joint joint_spherical;
	Body body;

	Model emulated_model;
	Model spherical_model;

	VectorNd emuQ;
	VectorNd emuQDot;
	VectorNd emuQDDot;
	VectorNd emuTau;

	VectorNd sphQ;
	VectorNd sphQDot;
	VectorNd sphQDDot;
	VectorNd sphTau;
};

TEST_FIXTURE(SphericalJoint, TestQIndices) {
	CHECK_EQUAL (0, spherical_model.mJoints[1].q_index);
	CHECK_EQUAL (3, spherical_model.mJoints[2].q_index);

	CHECK_EQUAL (4, emulated_model.q_size);
	CHECK_EQUAL (4, emulated_model.qdot_size);

	CHECK_EQUAL (4, spherical_model.q_size);
	CHECK_EQUAL (4, spherical_model.qdot_size);
}

TEST_FIXTURE(SphericalJoint, TestForwardDynamicsSimple) {
	for (unsigned int i = 0; i < emulated_model.q_size; i++) {
		emuQ[i] = 1.;
		sphQ[i] = 1.;
	}
	ForwardDynamics (emulated_model, emuQ, emuQDot, emuTau, emuQDDot);
	ForwardDynamics (spherical_model, sphQ, sphQDot, sphTau, sphQDDot);

	CHECK_ARRAY_CLOSE (emuQDDot.data(), sphQDDot.data(), emuQDDot.size(), TEST_PREC);
}
