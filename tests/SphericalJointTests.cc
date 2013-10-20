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

const double TEST_PREC = 1.0e-12;

struct SphericalJoint {
	SphericalJoint () {
		ClearLogOutput();

		emulated_model.gravity = Vector3d (0., 0., -9.81); 
		spherical_model.gravity = Vector3d (0., 0., -9.81); 

		body = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

		joint_rot_zyx = Joint (
				SpatialVector (0., 0., 1., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.)
				);
		joint_spherical = Joint (JointTypeSpherical);

		joint_rot_y = Joint (SpatialVector (0., 1., 0., 0., 0., 0.));

		emulated_model.AppendBody (Xtrans(Vector3d (0., 0., 0.)), joint_rot_y, body);
		emu_body_id = emulated_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_rot_zyx, body);
		emu_child_id = emulated_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_rot_y, body);

		spherical_model.AppendBody (Xtrans(Vector3d (0., 0., 0.)), joint_rot_y, body);
		sph_body_id = spherical_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_spherical, body);
		sph_child_id = spherical_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_rot_y, body);

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
	Joint joint_rot_y;
	Body body;

	unsigned int emu_body_id, emu_child_id, sph_body_id, sph_child_id;

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

void ConvertQAndQDotFromEmulated (
		const Model &emulated_model, const VectorNd &q_emulated, const VectorNd &qdot_emulated,
		const Model &spherical_model, VectorNd *q_spherical, VectorNd *qdot_spherical) {
	for (unsigned int i = 1; i < spherical_model.mJoints.size(); i++) {
		unsigned int q_index = spherical_model.mJoints[i].q_index;

		if (spherical_model.mJoints[i].mJointType == JointTypeSpherical) {
			Quaternion quat = Quaternion::fromZYXAngles ( Vector3d (
						q_emulated[q_index + 0], q_emulated[q_index + 1], q_emulated[q_index + 2]));
			spherical_model.SetQuaternion (i, quat, (*q_spherical));

			Vector3d omega = angular_velocity_from_angle_rates (
					Vector3d (q_emulated[q_index], q_emulated[q_index + 1], q_emulated[q_index + 2]),
					Vector3d (qdot_emulated[q_index], qdot_emulated[q_index + 1], qdot_emulated[q_index + 2])
					);

			(*qdot_spherical)[q_index] = omega[0];
			(*qdot_spherical)[q_index + 1] = omega[1];
			(*qdot_spherical)[q_index + 2] = omega[2];
		} else {
			(*q_spherical)[q_index] = q_emulated[q_index];
			(*qdot_spherical)[q_index] = qdot_emulated[q_index];
		}
	}
}

TEST(TestQuaternionIntegration ) {
	double timestep = 0.001;

	Vector3d zyx_angles_t0 (0.1, 0.2, 0.3);
	Vector3d zyx_rates (3., 5., 2.);
	Vector3d zyx_angles_t1 = zyx_angles_t0 + timestep * zyx_rates;
	Quaternion q_zyx_t1 = Quaternion::fromZYXAngles (zyx_angles_t1);

	Quaternion q_t0 = Quaternion::fromZYXAngles (zyx_angles_t0);
	Vector3d w_base = global_angular_velocity_from_rates (zyx_angles_t0, zyx_rates);
	Quaternion q_t1 = q_t0.timeStep (w_base, timestep);

	// Note: we test with a rather crude precision. My guess for the error is
	// that we compare two different things:
	//   A) integration under the assumption that the euler rates are
	//   constant
	//   B) integration under the assumption that the angular velocity is
	//   constant
	// However I am not entirely sure about this...
	CHECK_ARRAY_CLOSE (q_zyx_t1.data(), q_t1.data(), 4, 1.0e-5);
}

TEST_FIXTURE(SphericalJoint, TestQIndices) {
	CHECK_EQUAL (0, spherical_model.mJoints[1].q_index);
	CHECK_EQUAL (1, spherical_model.mJoints[2].q_index);
	CHECK_EQUAL (4, spherical_model.mJoints[3].q_index);

	CHECK_EQUAL (5, emulated_model.q_size);
	CHECK_EQUAL (5, emulated_model.qdot_size);

	CHECK_EQUAL (6, spherical_model.q_size);
	CHECK_EQUAL (5, spherical_model.qdot_size);
	CHECK_EQUAL (5, spherical_model.spherical_w_index[2]);
}

TEST_FIXTURE(SphericalJoint, TestGetQuaternion) {
	spherical_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_spherical, body);

	sphQ = VectorNd::Zero ((size_t) spherical_model.q_size);
	sphQDot = VectorNd::Zero ((size_t) spherical_model.qdot_size);
	sphQDDot = VectorNd::Zero ((size_t) spherical_model.qdot_size);
	sphTau = VectorNd::Zero ((size_t) spherical_model.qdot_size);

	CHECK_EQUAL (10, spherical_model.q_size);
	CHECK_EQUAL (8, spherical_model.qdot_size);

	CHECK_EQUAL (0, spherical_model.mJoints[1].q_index);
	CHECK_EQUAL (1, spherical_model.mJoints[2].q_index);
	CHECK_EQUAL (4, spherical_model.mJoints[3].q_index);
	CHECK_EQUAL (5, spherical_model.mJoints[4].q_index);

	CHECK_EQUAL (8, spherical_model.spherical_w_index[2]);
	CHECK_EQUAL (9, spherical_model.spherical_w_index[4]);

	sphQ[0] = 100.;
	sphQ[1] = 0.;
	sphQ[2] = 1.;
	sphQ[3] = 2.;
	sphQ[4] = 100.;
	sphQ[5] = -6.;
	sphQ[6] = -7.;
	sphQ[7] = -8;
	sphQ[8] = 4.;
	sphQ[9] = -9.;

	Quaternion reference_1 (0., 1., 2., 4.);
	Quaternion quat_1 = spherical_model.GetQuaternion (2, sphQ);
	CHECK_ARRAY_EQUAL (reference_1.data(), quat_1.data(), 4);

	Quaternion reference_3 (-6., -7., -8., -9.);
	Quaternion quat_3 = spherical_model.GetQuaternion (4, sphQ);
	CHECK_ARRAY_EQUAL (reference_3.data(), quat_3.data(), 4);
}

TEST_FIXTURE(SphericalJoint, TestSetQuaternion) {
	spherical_model.AppendBody (Xtrans (Vector3d (1., 0., 0.)), joint_spherical, body);

	sphQ = VectorNd::Zero ((size_t) spherical_model.q_size);
	sphQDot = VectorNd::Zero ((size_t) spherical_model.qdot_size);
	sphQDDot = VectorNd::Zero ((size_t) spherical_model.qdot_size);
	sphTau = VectorNd::Zero ((size_t) spherical_model.qdot_size);

	Quaternion reference_1 (0., 1., 2., 3.);
	spherical_model.SetQuaternion (2, reference_1, sphQ);
	Quaternion test = spherical_model.GetQuaternion (2, sphQ);
	CHECK_ARRAY_EQUAL (reference_1.data(), test.data(), 4);

	Quaternion reference_2 (11., 22., 33., 44.);
	spherical_model.SetQuaternion (4, reference_2, sphQ);
	test = spherical_model.GetQuaternion (4, sphQ);
	CHECK_ARRAY_EQUAL (reference_2.data(), test.data(), 4);
}

TEST_FIXTURE(SphericalJoint, TestOrientation) {
	emuQ[0] = 1.1;
	emuQ[1] = 1.1;
	emuQ[2] = 1.1;
	emuQ[3] = 1.1;

	for (unsigned int i = 0; i < emuQ.size(); i++) {
		sphQ[i] = emuQ[i];
	}

	Quaternion quat =  Quaternion::fromAxisAngle (Vector3d (1., 0., 0.), emuQ[2]) 
		* Quaternion::fromAxisAngle (Vector3d (0., 1., 0.), emuQ[1])
		* Quaternion::fromAxisAngle (Vector3d (0., 0., 1.), emuQ[0]);
	spherical_model.SetQuaternion (2, quat, sphQ);

	Matrix3d emu_orientation = CalcBodyWorldOrientation (emulated_model, emuQ, emu_child_id);
	Matrix3d sph_orientation = CalcBodyWorldOrientation (spherical_model, sphQ, sph_child_id);

	CHECK_ARRAY_CLOSE (emu_orientation.data(), sph_orientation.data(), 9, TEST_PREC);
}

TEST_FIXTURE(SphericalJoint, TestUpdateKinematics) {
	emuQ[0] = 1.;
	emuQ[1] = 1.;
	emuQ[2] = 1.;
	emuQ[3] = 1.;
	emuQ[4] = 1.;

	emuQDot[0] = 1.;
	emuQDot[1] = 1.;
	emuQDot[2] = 1.;
	emuQDot[3] = 1.;
	emuQDot[4] = 1.;

	emuQDDot[0] = 1.;
	emuQDDot[1] = 1.;
	emuQDDot[2] = 1.;
	emuQDDot[3] = 1.;
	emuQDDot[4] = 1.;

	ConvertQAndQDotFromEmulated (emulated_model, emuQ, emuQDot, spherical_model, &sphQ, &sphQDot);
	ConvertQAndQDotFromEmulated (emulated_model, emuQ, emuQDDot, spherical_model, &sphQ, &sphQDDot);

	Vector3d a = angular_acceleration_from_angle_rates (
			Vector3d (emuQ[3], emuQ[2], emuQ[1]),
			Vector3d (emuQDot[3], emuQDot[2], emuQDot[1]),
			Vector3d (emuQDDot[3], emuQDDot[2], emuQDDot[1])
			);

	sphQDDot[0] = emuQDDot[0];
	sphQDDot[1] = a[0];
	sphQDDot[2] = a[1];
	sphQDDot[3] = a[2];
	sphQDDot[4] = emuQDDot[4];

	UpdateKinematicsCustom (emulated_model, &emuQ, &emuQDot, &emuQDDot);
	UpdateKinematicsCustom (spherical_model, &sphQ, &sphQDot, &sphQDDot);

	CHECK_ARRAY_CLOSE (emulated_model.v[emu_body_id].data(), spherical_model.v[sph_body_id].data(), 6, TEST_PREC);
	CHECK_ARRAY_CLOSE (emulated_model.a[emu_body_id].data(), spherical_model.a[sph_body_id].data(), 6, TEST_PREC);

	UpdateKinematics (spherical_model, sphQ, sphQDot, sphQDDot);

	CHECK_ARRAY_CLOSE (emulated_model.v[emu_child_id].data(), spherical_model.v[sph_child_id].data(), 6, TEST_PREC);
	CHECK_ARRAY_CLOSE (emulated_model.a[emu_child_id].data(), spherical_model.a[sph_child_id].data(), 6, TEST_PREC);
}

TEST_FIXTURE(SphericalJoint, TestSpatialVelocities) {
	emuQ[0] = 1.;
	emuQ[1] = 2.;
	emuQ[2] = 3.;
	emuQ[3] = 4.;

	emuQDot[0] = 4.;
	emuQDot[1] = 2.;
	emuQDot[2] = 3.;
	emuQDot[3] = 6.;

	ConvertQAndQDotFromEmulated (emulated_model, emuQ, emuQDot, spherical_model, &sphQ, &sphQDot);

	UpdateKinematicsCustom (emulated_model, &emuQ, &emuQDot, NULL);
	UpdateKinematicsCustom (spherical_model, &sphQ, &sphQDot, NULL);

	CHECK_ARRAY_CLOSE (emulated_model.v[emu_child_id].data(), spherical_model.v[sph_child_id].data(), 6, TEST_PREC);
}

TEST_FIXTURE(SphericalJoint, TestForwardDynamicsQAndQDot) {
	emuQ[0] = 1.1;
	emuQ[1] = 1.2;
	emuQ[2] = 1.3;
	emuQ[3] = 1.4;

	emuQDot[0] = 2.2;
	emuQDot[1] = 2.3;
	emuQDot[2] = 2.4;
	emuQDot[3] = 2.5;

	ConvertQAndQDotFromEmulated (emulated_model, emuQ, emuQDot, spherical_model, &sphQ, &sphQDot);

	ForwardDynamics (emulated_model, emuQ, emuQDot, emuTau, emuQDDot);
	ForwardDynamics (spherical_model, sphQ, sphQDot, sphTau, sphQDDot);

	CHECK_ARRAY_CLOSE (emulated_model.a[emu_child_id].data(), spherical_model.a[sph_child_id].data(), 6, TEST_PREC);
}

TEST_FIXTURE(SphericalJoint, TestDynamicsConsistencyRNEA_ABA ) {
	emuQ[0] = 1.1;
	emuQ[1] = 1.2;
	emuQ[2] = 1.3;
	emuQ[3] = 1.4;
	emuQ[4] = 1.5;

	emuQDot[0] = 1.;
	emuQDot[1] = 2.;
	emuQDot[2] = 3.;
	emuQDot[3] = 4.;
	emuQDot[4] = 5.;

	sphTau[0] = 5.;
	sphTau[1] = 4.;
	sphTau[2] = 7.;
	sphTau[3] = 3.;
	sphTau[4] = 2.;

	ConvertQAndQDotFromEmulated (emulated_model, emuQ, emuQDot, spherical_model, &sphQ, &sphQDot);

	ForwardDynamics (spherical_model, sphQ, sphQDot, sphTau, sphQDDot);

	VectorNd tau_id (VectorNd::Zero (spherical_model.qdot_size));
	InverseDynamics (spherical_model, sphQ, sphQDot, sphQDDot, tau_id);

	CHECK_ARRAY_CLOSE (sphTau.data(), tau_id.data(), tau_id.size(), TEST_PREC);
}

TEST_FIXTURE(SphericalJoint, TestCRBA ) {
	emuQ[0] = 1.1;
	emuQ[1] = 1.2;
	emuQ[2] = 1.3;
	emuQ[3] = 1.4;
	emuQ[4] = 1.5;

	emuQDot[0] = 1.;
	emuQDot[1] = 2.;
	emuQDot[2] = 3.;
	emuQDot[3] = 4.;
	emuQDot[4] = 5.;

	sphTau[0] = 5.;
	sphTau[1] = 4.;
	sphTau[2] = 7.;
	sphTau[3] = 3.;
	sphTau[4] = 2.;

	ConvertQAndQDotFromEmulated (emulated_model, emuQ, emuQDot, spherical_model, &sphQ, &sphQDot);

	MatrixNd H_crba (MatrixNd::Zero (spherical_model.qdot_size, spherical_model.qdot_size));

	UpdateKinematicsCustom (spherical_model, &sphQ, NULL, NULL);
	CompositeRigidBodyAlgorithm (spherical_model, sphQ, H_crba, false);

	MatrixNd H_id (MatrixNd::Zero (spherical_model.qdot_size, spherical_model.qdot_size));
	VectorNd H_col = VectorNd::Zero (spherical_model.qdot_size);
	VectorNd QDDot_zero = VectorNd::Zero (spherical_model.qdot_size);

	for (unsigned int i = 0; i < spherical_model.qdot_size; i++) {
		// compute each column
		VectorNd delta_a = VectorNd::Zero (spherical_model.qdot_size);
		delta_a[i] = 1.;

		// compute ID (model, q, qdot, delta_a)
		VectorNd id_delta = VectorNd::Zero (spherical_model.qdot_size);
		InverseDynamics (spherical_model, sphQ, sphQDot, delta_a, id_delta);

		// compute ID (model, q, qdot, zero)
		VectorNd id_zero = VectorNd::Zero (spherical_model.qdot_size);
		InverseDynamics (spherical_model, sphQ, sphQDot, QDDot_zero, id_zero);
	
		H_col = id_delta - id_zero;
		H_id.block(0, i, spherical_model.qdot_size, 1) = H_col;
	}

	CHECK_ARRAY_CLOSE (H_id.data(), H_crba.data(), spherical_model.qdot_size, TEST_PREC);
}

TEST_FIXTURE(SphericalJoint, TestForwardDynamicsLagrangianVsABA ) {
	emuQ[0] = 1.1;
	emuQ[1] = 1.2;
	emuQ[2] = 1.3;
	emuQ[3] = 1.4;
	emuQ[4] = 1.5;

	emuQDot[0] = 1.;
	emuQDot[1] = 2.;
	emuQDot[2] = 3.;
	emuQDot[3] = 4.;
	emuQDot[4] = 5.;

	sphTau[0] = 5.;
	sphTau[1] = 4.;
	sphTau[2] = 7.;
	sphTau[3] = 3.;
	sphTau[4] = 2.;

	ConvertQAndQDotFromEmulated (emulated_model, emuQ, emuQDot, spherical_model, &sphQ, &sphQDot);

	VectorNd QDDot_aba = VectorNd::Zero (spherical_model.qdot_size);
	VectorNd QDDot_lag = VectorNd::Zero (spherical_model.qdot_size);

	ForwardDynamics (spherical_model, sphQ, sphQDot, sphTau, QDDot_aba);
	ForwardDynamicsLagrangian (spherical_model, sphQ, sphQDot, sphTau, QDDot_lag);

	CHECK_ARRAY_CLOSE (QDDot_lag.data(), QDDot_aba.data(), spherical_model.qdot_size, TEST_PREC);
}

TEST_FIXTURE(SphericalJoint, TestContactsLagrangian) {
	ConstraintSet constraint_set_emu;

	constraint_set_emu.AddConstraint (emu_child_id, Vector3d (0., 0., -1.), Vector3d (1., 0., 0.));
	constraint_set_emu.AddConstraint (emu_child_id, Vector3d (0., 0., -1.), Vector3d (0., 1., 0.));
	constraint_set_emu.AddConstraint (emu_child_id, Vector3d (0., 0., -1.), Vector3d (0., 0., 1.));

	constraint_set_emu.Bind(emulated_model);

	ConstraintSet constraint_set_sph;

	constraint_set_sph.AddConstraint (sph_child_id, Vector3d (0., 0., -1.), Vector3d (1., 0., 0.));
	constraint_set_sph.AddConstraint (sph_child_id, Vector3d (0., 0., -1.), Vector3d (0., 1., 0.));
	constraint_set_sph.AddConstraint (sph_child_id, Vector3d (0., 0., -1.), Vector3d (0., 0., 1.));

	constraint_set_sph.Bind(spherical_model);
	
	ForwardDynamicsContactsLagrangian (emulated_model, emuQ, emuQDot, emuTau, constraint_set_emu, emuQDDot);
	VectorNd emu_force_lagrangian = constraint_set_emu.force;
	ForwardDynamicsContactsLagrangian (spherical_model, sphQ, sphQDot, sphTau, constraint_set_sph, sphQDDot);
	VectorNd sph_force_lagrangian = constraint_set_sph.force;

	CHECK_ARRAY_CLOSE (emu_force_lagrangian.data(), sph_force_lagrangian.data(), 3, TEST_PREC);
}

TEST_FIXTURE(SphericalJoint, TestContacts) {
	ConstraintSet constraint_set_emu;

	constraint_set_emu.AddConstraint (emu_child_id, Vector3d (0., 0., -1.), Vector3d (1., 0., 0.));
	constraint_set_emu.AddConstraint (emu_child_id, Vector3d (0., 0., -1.), Vector3d (0., 1., 0.));
	constraint_set_emu.AddConstraint (emu_child_id, Vector3d (0., 0., -1.), Vector3d (0., 0., 1.));

	constraint_set_emu.Bind(emulated_model);

	ConstraintSet constraint_set_sph;

	constraint_set_sph.AddConstraint (sph_child_id, Vector3d (0., 0., -1.), Vector3d (1., 0., 0.));
	constraint_set_sph.AddConstraint (sph_child_id, Vector3d (0., 0., -1.), Vector3d (0., 1., 0.));
	constraint_set_sph.AddConstraint (sph_child_id, Vector3d (0., 0., -1.), Vector3d (0., 0., 1.));

	constraint_set_sph.Bind(spherical_model);
	
	ForwardDynamicsContacts(emulated_model, emuQ, emuQDot, emuTau, constraint_set_emu, emuQDDot);
	VectorNd emu_force_kokkevis = constraint_set_emu.force;
	ForwardDynamicsContacts(spherical_model, sphQ, sphQDot, sphTau, constraint_set_sph, sphQDDot);
	VectorNd sph_force_kokkevis = constraint_set_sph.force;

	CHECK_ARRAY_CLOSE (emu_force_kokkevis.data(), sph_force_kokkevis.data(), 3, TEST_PREC);
}
