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

struct ModelFixture {
	ModelFixture () {
		ClearLogOutput();
		model = new Model;
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

	CHECK_EQUAL (0u, model->q_size);
	CHECK_EQUAL (0u, model->qdot_size);

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
	Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

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

	CHECK_EQUAL (0, model->mJoints[0].q_index);
	CHECK_EQUAL (0, model->mJoints[1].q_index);
	CHECK_EQUAL (1, model->mJoints[2].q_index);
	CHECK_EQUAL (2, model->mJoints[3].q_index);
	CHECK_EQUAL (3, model->mJoints[4].q_index);
	CHECK_EQUAL (4, model->mJoints[5].q_index);
	CHECK_EQUAL (5, model->mJoints[6].q_index);
}

/** \brief Tests whether the joint and body information stored in the Model are computed correctly 
 */
TEST_FIXTURE(ModelFixture, TestAddBodySpatialValues) {
	Body body;
	Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body); 

	SpatialVector spatial_joint_axis(0., 0., 1., 0., 0., 0.);
	CHECK_EQUAL (spatial_joint_axis, joint.mJointAxes[0]);

	// \Todo: Dynamic properties
}

TEST_FIXTURE(ModelFixture, TestAddBodyTestBodyName) {
	Body body;
	Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body, "mybody"); 

	unsigned int body_id = model->GetBodyId("mybody");

	CHECK_EQUAL (1u, body_id);
	CHECK_EQUAL (std::numeric_limits<unsigned int>::max(), model->GetBodyId("unknownbody"));
}

TEST_FIXTURE(ModelFixture, TestjcalcSimple) {
	Body body;
	Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

	model->AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint, body);

	VectorNd Q = VectorNd::Zero (model->q_size);
	VectorNd QDot = VectorNd::Zero (model->q_size);

	SpatialTransform X_j;
	SpatialVector S;
	SpatialVector v_j;
	SpatialVector c;

	QDot[0] = 1.;
	jcalc (*model, 1, X_j, v_j, c, Q, QDot);

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
	CHECK_EQUAL (test_joint_axis, model->S[1]);

	Q[0] = M_PI * 0.5;
	QDot[0] = 1.;

	jcalc (*model, 1, X_j, v_j, c, Q, QDot);

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
	CHECK_EQUAL (test_joint_axis, model->S[1]);
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
	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
	Joint joint_rot_x ( SpatialVector (1., 0., 0., 0., 0., 0.));

	Model model_std;
	model_std.gravity = Vector3d (0., -9.81, 0.);

	model_std.AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_rot_z, null_body); 
	model_std.AppendBody(Xtrans(Vector3d(0., 0., 0.)), joint_rot_x, body); 

	// using a model with a 2 DoF joint
	Joint joint_rot_zx (
		SpatialVector (0., 0., 1., 0., 0., 0.),
		SpatialVector (1., 0., 0., 0., 0., 0.)
		);

	Model model_2;
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

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
	Joint joint_rot_y ( SpatialVector (0., 1., 0., 0., 0., 0.));
	Joint joint_rot_x ( SpatialVector (1., 0., 0., 0., 0., 0.));

	Model model_std;
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

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
	Joint joint_rot_y ( SpatialVector (0., 1., 0., 0., 0., 0.));
	Joint joint_rot_x ( SpatialVector (1., 0., 0., 0., 0., 0.));

	Model model_std;
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

TEST ( ModelFixedJointQueryBodyId ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

	model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");

	CHECK_EQUAL (fixed_body_id, model.GetBodyId("fixed_body"));
}

/* 
 * Makes sure that when appending a body to a fixed joint the parent of the
 * newly added parent is actually the moving body that the fixed body is
 * attached to.
 */
TEST ( ModelAppendToFixedBody ) {
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

	unsigned int movable_body = model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");
	unsigned int appended_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), joint_rot_z, body, "appended_body");

	CHECK_EQUAL (movable_body + 1, appended_body_id);
	CHECK_EQUAL (movable_body, model.lambda[appended_body_id]);
}

// Adds a fixed body to another fixed body.
TEST ( ModelAppendFixedToFixedBody ) {
	Body null_body;

	double movable_mass = 1.1;
	Vector3d movable_com (1., 0.4, 0.4);

	double fixed_mass = 1.2;
	Vector3d fixed_com (1.1, 0.5, 0.5);

	Vector3d fixed_displacement (0., 1., 0.);

	Body body(movable_mass, movable_com, Vector3d (1., 1., 1.));
	Body fixed_body(fixed_mass, fixed_com, Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

	unsigned int movable_body = model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (Xtrans(fixed_displacement), Joint(JointTypeFixed), fixed_body, "fixed_body");
	unsigned int fixed_body_2_id = model.AppendBody (Xtrans(fixed_displacement), Joint(JointTypeFixed), fixed_body, "fixed_body_2");
	unsigned int appended_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), joint_rot_z, body, "appended_body");

	CHECK_EQUAL (movable_body + 1, appended_body_id);
	CHECK_EQUAL (movable_body, model.lambda[appended_body_id]);
	CHECK_EQUAL (movable_mass + fixed_mass * 2., model.mBodies[movable_body].mMass);

	CHECK_EQUAL (movable_body, model.mFixedBodies[fixed_body_id - model.fixed_body_discriminator].mMovableParent);
	CHECK_EQUAL (movable_body, model.mFixedBodies[fixed_body_2_id - model.fixed_body_discriminator].mMovableParent);

	double new_mass = 3.5;
	Vector3d new_com = (1. / new_mass) * (movable_mass * movable_com + fixed_mass * (fixed_com + fixed_displacement) + fixed_mass * (fixed_com + fixed_displacement * 2.));

	CHECK_ARRAY_CLOSE (new_com.data(), model.mBodies[movable_body].mCenterOfMass.data(), 3, TEST_PREC);
}

// Ensures that the transformations of the movable parent and fixed joint
// frame is in proper order
TEST ( ModelFixedJointRotationOrderTranslationRotation ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

	SpatialTransform trans_x = Xtrans (Vector3d (1., 0., 0.));
	SpatialTransform rot_z = Xrotz (45. * M_PI / 180.);

	model.AddBody (0, trans_x, joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (rot_z, Joint(JointTypeFixed), fixed_body, "fixed_body");
	unsigned int body_after_fixed = model.AppendBody (trans_x, joint_rot_z, body);

	VectorNd Q (VectorNd::Zero(model.dof_count));
	Q[0] = 45 * M_PI / 180.;
	Vector3d point = CalcBodyToBaseCoordinates (model, Q, body_after_fixed, Vector3d (0., 1., 0.));

	CHECK_ARRAY_CLOSE (Vector3d (0., 1., 0.).data(), point.data(), 3, TEST_PREC);
}

// Ensures that the transformations of the movable parent and fixed joint
// frame is in proper order
TEST ( ModelFixedJointRotationOrderRotationTranslation ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

	SpatialTransform rot_z = Xrotz (45. * M_PI / 180.);
	SpatialTransform trans_x = Xtrans (Vector3d (1., 0., 0.));

	model.AddBody (0, rot_z, joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (trans_x, Joint(JointTypeFixed), fixed_body, "fixed_body");
	unsigned int body_after_fixed = model.AppendBody (trans_x, joint_rot_z, body);

	VectorNd Q (VectorNd::Zero(model.dof_count));
	Q[0] = 45 * M_PI / 180.;
	Vector3d point = CalcBodyToBaseCoordinates (model, Q, body_after_fixed, Vector3d (0., 1., 0.));

	CHECK_ARRAY_CLOSE (Vector3d (-1., 2., 0.).data(), point.data(), 3, TEST_PREC);
}

TEST ( ModelGetBodyName ) {
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

	unsigned int movable_body = model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");
	unsigned int appended_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), joint_rot_z, body, "appended_body");

	CHECK_EQUAL (string("fixed_body"), model.GetBodyName(fixed_body_id));
	CHECK_EQUAL (string("appended_body"), model.GetBodyName(appended_body_id));
	CHECK_EQUAL (string(""), model.GetBodyName(123));
}

TEST_FIXTURE ( RotZRotZYXFixed, ModelGetParentBodyId ) {
	CHECK_EQUAL (0u, model->GetParentBodyId(0));
	CHECK_EQUAL (0u, model->GetParentBodyId(body_a_id));
	CHECK_EQUAL (body_a_id, model->GetParentBodyId(body_b_id));
}

TEST_FIXTURE(RotZRotZYXFixed, ModelGetParentIdFixed) {
	CHECK_EQUAL (body_b_id, model->GetParentBodyId(body_fixed_id));
}

TEST_FIXTURE(RotZRotZYXFixed, ModelGetJointFrame) {
	SpatialTransform transform_a = model->GetJointFrame (body_a_id);
	SpatialTransform transform_b = model->GetJointFrame (body_b_id);
	SpatialTransform transform_root = model->GetJointFrame (0);

	CHECK_ARRAY_EQUAL (fixture_transform_a.r.data(), transform_a.r.data(), 3);
	CHECK_ARRAY_EQUAL (fixture_transform_b.r.data(), transform_b.r.data(), 3);
	CHECK_ARRAY_EQUAL (Vector3d(0., 0., 0.).data(), transform_root.r.data(), 3);
}

TEST_FIXTURE(RotZRotZYXFixed, ModelGetJointFrameFixed) {
	SpatialTransform transform_fixed = model->GetJointFrame (body_fixed_id);

	CHECK_ARRAY_EQUAL (fixture_transform_fixed.r.data(), transform_fixed.r.data(), 3);
}

TEST_FIXTURE(RotZRotZYXFixed, ModelSetJointFrame) {
	SpatialTransform new_transform_a = Xtrans (Vector3d(-1., -2., -3.));
	SpatialTransform new_transform_b = Xtrans (Vector3d(-4., -5., -6.));
	SpatialTransform new_transform_root = Xtrans (Vector3d(-99, -99., -99.));

	model->SetJointFrame (body_a_id, new_transform_a);
	model->SetJointFrame (body_b_id, new_transform_b);
	model->SetJointFrame (0, new_transform_root);

	SpatialTransform transform_a = model->GetJointFrame (body_a_id);
	SpatialTransform transform_b = model->GetJointFrame (body_b_id);
	SpatialTransform transform_root = model->GetJointFrame (0);

	CHECK_ARRAY_EQUAL (new_transform_a.r.data(), transform_a.r.data(), 3);
	CHECK_ARRAY_EQUAL (new_transform_b.r.data(), transform_b.r.data(), 3);
	CHECK_ARRAY_EQUAL (Vector3d(0., 0., 0.).data(), transform_root.r.data(), 3);
}


