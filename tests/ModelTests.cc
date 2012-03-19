#include <UnitTest++.h>

#include <iostream>

#include "rbdl_mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"
#include "Dynamics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

struct ModelFixture {
	ModelFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();
		model->gravity = Vector3d (0., -9.81, 0.);
	}
	~ModelFixture () {
		delete model;
	}
	Model *model;
};

TEST_FIXTURE(ModelFixture, TestInit) {
	CHECK_EQUAL (1u, model->lambda.size());
	CHECK_EQUAL (1u, model->mu.size());
	CHECK_EQUAL (0u, model->dof_count);

	CHECK_EQUAL (1u, model->v.size());
	CHECK_EQUAL (1u, model->a.size());
	
	CHECK_EQUAL (1u, model->mJoints.size());
	CHECK_EQUAL (1u, model->S.size());

	CHECK_EQUAL (1u, model->c.size());
	CHECK_EQUAL (1u, model->IA.size());
	CHECK_EQUAL (1u, model->pA.size());
	CHECK_EQUAL (1u, model->U.size());
	CHECK_EQUAL (1u, model->d.size());
	CHECK_EQUAL (1u, model->u.size());
	CHECK_EQUAL (1u, model->Ic.size());
	
	CHECK_EQUAL (1u, model->X_lambda.size());
	CHECK_EQUAL (1u, model->X_base.size());
	CHECK_EQUAL (1u, model->mBodies.size());
}

TEST_FIXTURE(ModelFixture, TestAddBodyDimensions) {
	Body body;
	Joint joint (
			JointTypeRevolute,
			Vector3d(0., 0., 1.)
			);

	unsigned int body_id = 0;
	body_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body); 

	CHECK_EQUAL (1u, body_id);
	CHECK_EQUAL (2u, model->lambda.size());
	CHECK_EQUAL (2u, model->mu.size());
	CHECK_EQUAL (1u, model->dof_count);

	CHECK_EQUAL (2u, model->v.size());
	CHECK_EQUAL (2u, model->a.size());
	
	CHECK_EQUAL (2u, model->mJoints.size());
	CHECK_EQUAL (2u, model->S.size());

	CHECK_EQUAL (2u, model->c.size());
	CHECK_EQUAL (2u, model->IA.size());
	CHECK_EQUAL (2u, model->pA.size());
	CHECK_EQUAL (2u, model->U.size());
	CHECK_EQUAL (2u, model->d.size());
	CHECK_EQUAL (2u, model->u.size());
	CHECK_EQUAL (2u, model->Ic.size());

	SpatialVector spatial_zero;
	spatial_zero.setZero();
	
	CHECK_EQUAL (2u, model->X_lambda.size());
	CHECK_EQUAL (2u, model->X_base.size());
	CHECK_EQUAL (2u, model->mBodies.size());
}

TEST_FIXTURE(ModelFixture, TestExperimentalFloatingBodyDimensions) {
	Body body;

	model->experimental_floating_base = true;
	model->SetFloatingBaseBody(body);

	CHECK_EQUAL (1u, model->lambda.size());
	CHECK_EQUAL (1u, model->mu.size());
	CHECK_EQUAL (6u, model->dof_count);

	CHECK_EQUAL (1u, model->v.size());
	CHECK_EQUAL (1u, model->a.size());
	
	CHECK_EQUAL (1u, model->mJoints.size());
	CHECK_EQUAL (1u, model->S.size());

	CHECK_EQUAL (1u, model->c.size());
	CHECK_EQUAL (1u, model->IA.size());
	CHECK_EQUAL (1u, model->pA.size());
	CHECK_EQUAL (1u, model->U.size());
	CHECK_EQUAL (1u, model->d.size());
	CHECK_EQUAL (1u, model->u.size());

	SpatialVector spatial_zero;
	spatial_zero.setZero();
	
	CHECK_EQUAL (1u, model->X_lambda.size());
	CHECK_EQUAL (1u, model->X_base.size());
	CHECK_EQUAL (1u, model->mBodies.size());
}

TEST_FIXTURE(ModelFixture, TestFloatingBodyDimensions) {
	Body body;

	model->SetFloatingBaseBody(body);

	CHECK_EQUAL (7u, model->lambda.size());
	CHECK_EQUAL (7u, model->mu.size());
	CHECK_EQUAL (6u, model->dof_count);

	CHECK_EQUAL (7u, model->v.size());
	CHECK_EQUAL (7u, model->a.size());
	
	CHECK_EQUAL (7u, model->mJoints.size());
	CHECK_EQUAL (7u, model->S.size());

	CHECK_EQUAL (7u, model->c.size());
	CHECK_EQUAL (7u, model->IA.size());
	CHECK_EQUAL (7u, model->pA.size());
	CHECK_EQUAL (7u, model->U.size());
	CHECK_EQUAL (7u, model->d.size());
	CHECK_EQUAL (7u, model->u.size());

	SpatialVector spatial_zero;
	spatial_zero.setZero();
	
	CHECK_EQUAL (7u, model->X_lambda.size());
	CHECK_EQUAL (7u, model->X_base.size());
	CHECK_EQUAL (7u, model->mBodies.size());
}

/** \brief Tests whether the joint and body information stored in the Model are computed correctly 
 */
TEST_FIXTURE(ModelFixture, TestAddBodySpatialValues) {
	Body body;
	Joint joint (
		JointTypeRevolute,
		Vector3d(0., 0., 1.)
		);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body); 

	SpatialVector spatial_joint_axis(0., 0., 1., 0., 0., 0.);
	CHECK_EQUAL (spatial_joint_axis, joint.mJointAxes[0]);

	// \Todo: Dynamic properties
}

TEST_FIXTURE(ModelFixture, TestAddBodyTestBodyName) {
	Body body;
	Joint joint (
		JointTypeRevolute,
		Vector3d(0., 0., 1.)
		);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body, "mybody"); 

	unsigned int body_id = model->GetBodyId("mybody");

	CHECK_EQUAL (1, body_id);
	CHECK_EQUAL (std::numeric_limits<unsigned int>::max(), model->GetBodyId("unknownbody"));
}

TEST_FIXTURE(ModelFixture, TestjcalcSimple) {
	Body body;
	Joint joint (
		JointTypeRevolute,
		Vector3d(0., 0., 1.)
		);

	model->AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint, body);

	SpatialTransform X_j;
	SpatialVector S;
	SpatialVector v_j;
	SpatialVector c;

	jcalc (*model, 1, X_j, S, v_j, c, 0., 1.);

	SpatialMatrix test_matrix (
			1.,  0.,  0.,  0.,  0.,  0.,
			0.,  1.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  0.,  0.,  1.,  0.,  0.,
			0.,  0.,  0.,  0.,  1.,  0.,
			0.,  0.,  0.,  0.,  0.,  1.
			);
	SpatialVector test_vector (
			0., 0., 1., 0., 0., 0.
			);
	SpatialVector test_joint_axis (
			0., 0., 1., 0., 0., 0.
			);

	CHECK (SpatialMatrixCompareEpsilon (test_matrix, X_j.toMatrix(), 1.0e-16));
	CHECK (SpatialVectorCompareEpsilon (test_vector, v_j, 1.0e-16));
	CHECK_EQUAL (test_joint_axis, S);

	jcalc (*model, 1, X_j, S, v_j, c, M_PI * 0.5, 1.);

	test_matrix.set (
			0.,  1.,  0.,  0.,  0.,  0.,
		 -1.,  0.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  0.,  0.,  0.,  1.,  0.,
			0.,  0.,  0., -1.,  0.,  0.,
			0.,  0.,  0.,  0.,  0.,  1.
			);

	CHECK (SpatialMatrixCompareEpsilon (test_matrix, X_j.toMatrix(), TEST_PREC));
	CHECK (SpatialVectorCompareEpsilon (test_vector, v_j, TEST_PREC));
	CHECK_EQUAL (test_joint_axis, S);
}

TEST_FIXTURE ( ModelFixture, TestTransformBaseToLocal ) {
	Body body;
	unsigned int body_id = model->SetFloatingBaseBody (body);

	VectorNd q = VectorNd::Zero (model->dof_count);
	VectorNd qdot = VectorNd::Zero (model->dof_count);
	VectorNd qddot = VectorNd::Zero (model->dof_count);
	VectorNd tau = VectorNd::Zero (model->dof_count);

	Vector3d base_coords (0., 0., 0.);
	Vector3d body_coords;
	Vector3d base_coords_back;

	UpdateKinematics (*model, q, qdot, qddot);
	body_coords = CalcBaseToBodyCoordinates (*model, q, body_id, base_coords, false);
	base_coords_back = CalcBodyToBaseCoordinates (*model, q, body_id, body_coords, false);

	CHECK_ARRAY_CLOSE (base_coords.data(), base_coords_back.data(), 3, TEST_PREC);

	q[0] = 1.;
	q[1] = 0.2;
	q[2] = -2.3;
	q[3] = -2.3;
	q[4] = 0.03;
	q[5] = -0.23;

	UpdateKinematics (*model, q, qdot, qddot);
	body_coords = CalcBaseToBodyCoordinates (*model, q, body_id, base_coords, false);
	base_coords_back = CalcBodyToBaseCoordinates (*model, q, body_id, body_coords, false);

	CHECK_ARRAY_CLOSE (base_coords.data(), base_coords_back.data(), 3, TEST_PREC);
}

TEST ( Model2DoFJoint ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Joint joint_rot_z (
			JointTypeRevolute,
			Vector3d(0., 0., 1.)
			);
	Joint joint_rot_x (
			JointTypeRevolute,
			Vector3d(1., 0., 0.)
			);

	Model model_std;
	model_std.Init();
	model_std.gravity = Vector3d (0., -9.81, 0.);

	model_std.AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body); 
	model_std.AppendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body); 

	// using a model with a 2 DoF joint
	Joint joint_rot_zx (
		SpatialVector (0., 0., 1., 0., 0., 0.),
		SpatialVector (1., 0., 0., 0., 0., 0.)
		);

	Model model_2;
	model_2.Init();
	model_2.gravity = Vector3d (0., -9.81, 0.);

	model_2.AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zx, body);

	VectorNd Q = VectorNd::Zero(model_std.dof_count);
	VectorNd QDot = VectorNd::Zero(model_std.dof_count);
	VectorNd Tau = VectorNd::Zero(model_std.dof_count);

	VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
	VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

	ForwardDynamics (model_std, Q, QDot, Tau, QDDot_std);
	ForwardDynamics (model_2, Q, QDot, Tau, QDDot_2);

	CHECK_ARRAY_CLOSE (QDDot_std.data(), QDDot_2.data(), model_std.dof_count, TEST_PREC);
}

TEST ( Model3DoFJoint ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Joint joint_rot_z (
			JointTypeRevolute,
			Vector3d(0., 0., 1.)
			);
	Joint joint_rot_y (
			JointTypeRevolute,
			Vector3d(0., 1., 0.)
			);
	Joint joint_rot_x (
			JointTypeRevolute,
			Vector3d(1., 0., 0.)
			);

	Model model_std;
	model_std.Init();
	model_std.gravity = Vector3d (0., -9.81, 0.);

	unsigned int body_id;

	// in total we add two bodies to make sure that the transformations are
	// correct.
	model_std.AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body); 
	model_std.AppendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body); 
	body_id = model_std.AppendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body); 

	model_std.AddBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body); 
	model_std.AppendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body); 
	body_id = model_std.AppendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body); 

	// using a model with a 2 DoF joint
	Joint joint_rot_zyx (
		SpatialVector (0., 0., 1., 0., 0., 0.),
		SpatialVector (0., 1., 0., 0., 0., 0.),
		SpatialVector (1., 0., 0., 0., 0., 0.)
		);

	Model model_2;
	model_2.Init();
	model_2.gravity = Vector3d (0., -9.81, 0.);

	// in total we add two bodies to make sure that the transformations are
	// correct.
	body_id = model_2.AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx, body);
	body_id = model_2.AddBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx, body);

	VectorNd Q = VectorNd::Zero(model_std.dof_count);
	VectorNd QDot = VectorNd::Zero(model_std.dof_count);
	VectorNd Tau = VectorNd::Zero(model_std.dof_count);

	VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
	VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

	ForwardDynamics (model_std, Q, QDot, Tau, QDDot_std);
	ForwardDynamics (model_2, Q, QDot, Tau, QDDot_2);

	CHECK_ARRAY_CLOSE (QDDot_std.data(), QDDot_2.data(), model_std.dof_count, TEST_PREC);
}

TEST ( Model6DoFJoint ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Joint joint_rot_z (
			JointTypeRevolute,
			Vector3d(0., 0., 1.)
			);
	Joint joint_rot_y (
			JointTypeRevolute,
			Vector3d(0., 1., 0.)
			);
	Joint joint_rot_x (
			JointTypeRevolute,
			Vector3d(1., 0., 0.)
			);

	Model model_std;
	model_std.Init();
	model_std.gravity = Vector3d (0., -9.81, 0.);

	unsigned int body_id;

	// in total we add two bodies to make sure that the transformations are
	// correct.
	body_id = model_std.SetFloatingBaseBody (body);

	model_std.AddBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body); 
	model_std.AppendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_y, null_body); 
	body_id = model_std.AppendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body); 

	// using a model with a 2 DoF joint
	Joint joint_floating_base (
		SpatialVector (0., 0., 0., 1., 0., 0.),
		SpatialVector (0., 0., 0., 0., 1., 0.),
		SpatialVector (0., 0., 0., 0., 0., 1.),
		SpatialVector (0., 0., 1., 0., 0., 0.),
		SpatialVector (0., 1., 0., 0., 0., 0.),
		SpatialVector (1., 0., 0., 0., 0., 0.)
		);

	Joint joint_rot_zyx (
		SpatialVector (0., 0., 1., 0., 0., 0.),
		SpatialVector (0., 1., 0., 0., 0., 0.),
		SpatialVector (1., 0., 0., 0., 0., 0.)
		);

	Model model_2;
	model_2.Init();
	model_2.gravity = Vector3d (0., -9.81, 0.);

	// in total we add two bodies to make sure that the transformations are
	// correct.
	body_id = model_2.AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_floating_base, body);
	body_id = model_2.AddBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx, body);

	VectorNd Q = VectorNd::Zero(model_std.dof_count);
	VectorNd QDot = VectorNd::Zero(model_std.dof_count);
	VectorNd Tau = VectorNd::Zero(model_std.dof_count);

	VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
	VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

	ForwardDynamics (model_std, Q, QDot, Tau, QDDot_std);
	ForwardDynamics (model_2, Q, QDot, Tau, QDDot_2);

	CHECK_ARRAY_CLOSE (QDDot_std.data(), QDDot_2.data(), model_std.dof_count, TEST_PREC);
}
