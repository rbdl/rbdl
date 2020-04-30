#include "rbdl_tests.h"

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

TEST_CASE_METHOD(ModelFixture, __FILE__"_TestInit", "") {
  REQUIRE (1u == model->lambda.size());
  REQUIRE (1u == model->mu.size());
  REQUIRE (0u == model->dof_count);

  REQUIRE (0u == model->q_size);
  REQUIRE (0u == model->qdot_size);

  REQUIRE (1u == model->v.size());
  REQUIRE (1u == model->a.size());

  REQUIRE (1u == model->mJoints.size());
  REQUIRE (1u == model->S.size());

  REQUIRE (1u == model->c.size());
  REQUIRE (1u == model->IA.size());
  REQUIRE (1u == model->pA.size());
  REQUIRE (1u == model->U.size());
  REQUIRE (1u == model->d.size());
  REQUIRE (1u == model->u.size());
  REQUIRE (1u == model->Ic.size());
  REQUIRE (1u == model->I.size());

  REQUIRE (1u == model->X_lambda.size());
  REQUIRE (1u == model->X_base.size());
  REQUIRE (1u == model->mBodies.size());
}

TEST_CASE_METHOD(ModelFixture, __FILE__"_TestAddBodyDimensions", "") {
  Body body;
  Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

  unsigned int body_id = 0;
  body_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

  REQUIRE (1u == body_id);
  REQUIRE (2u == model->lambda.size());
  REQUIRE (2u == model->mu.size());
  REQUIRE (1u == model->dof_count);

  REQUIRE (2u == model->v.size());
  REQUIRE (2u == model->a.size());

  REQUIRE (2u == model->mJoints.size());
  REQUIRE (2u == model->S.size());

  REQUIRE (2u == model->c.size());
  REQUIRE (2u == model->IA.size());
  REQUIRE (2u == model->pA.size());
  REQUIRE (2u == model->U.size());
  REQUIRE (2u == model->d.size());
  REQUIRE (2u == model->u.size());
  REQUIRE (2u == model->Ic.size());
  REQUIRE (2u == model->I.size());

  SpatialVector spatial_zero;
  spatial_zero.setZero();

  REQUIRE (2u == model->X_lambda.size());
  REQUIRE (2u == model->X_base.size());
  REQUIRE (2u == model->mBodies.size());
}

TEST_CASE_METHOD(ModelFixture, __FILE__"_TestFloatingBodyDimensions", "") {
  Body body;
  Joint float_base_joint (JointTypeFloatingBase);

  model->AppendBody (SpatialTransform(), float_base_joint, body);

  REQUIRE (3u == model->lambda.size());
  REQUIRE (3u == model->mu.size());
  REQUIRE (6u == model->dof_count);
  REQUIRE (7u == model->q_size);
  REQUIRE (6u == model->qdot_size);

  REQUIRE (3u == model->v.size());
  REQUIRE (3u == model->a.size());

  REQUIRE (3u == model->mJoints.size());
  REQUIRE (3u == model->S.size());

  REQUIRE (3u == model->c.size());
  REQUIRE (3u == model->IA.size());
  REQUIRE (3u == model->pA.size());
  REQUIRE (3u == model->U.size());
  REQUIRE (3u == model->d.size());
  REQUIRE (3u == model->u.size());

  SpatialVector spatial_zero;
  spatial_zero.setZero();

  REQUIRE (3u == model->X_lambda.size());
  REQUIRE (3u == model->X_base.size());
  REQUIRE (3u == model->mBodies.size());
}

/** \brief Tests whether the joint and body information stored in the Model are computed correctly
*/
TEST_CASE_METHOD(ModelFixture, __FILE__"_TestAddBodySpatialValues", "") {
  Body body;
  Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

  SpatialVector spatial_joint_axis(0., 0., 1., 0., 0., 0.);
  REQUIRE (spatial_joint_axis == joint.mJointAxes[0]);

  // \Todo: Dynamic properties
}

TEST_CASE_METHOD(ModelFixture, __FILE__"_TestAddBodyTestBodyName", "") {
  Body body;
  Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body, "mybody");

  unsigned int body_id = model->GetBodyId("mybody");

  REQUIRE (1u == body_id);
  REQUIRE (std::numeric_limits<unsigned int>::max() == model->GetBodyId("unknownbody"));
}

TEST_CASE_METHOD(ModelFixture, __FILE__"_TestjcalcSimple", "") {
  Body body;
  Joint joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model->AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint, body);

  VectorNd Q = VectorNd::Zero (model->q_size);
  VectorNd QDot = VectorNd::Zero (model->q_size);

  QDot[0] = 1.;
  jcalc (*model, 1, Q, QDot);

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

  REQUIRE_THAT (test_vector, AllCloseVector(model->v_J[1], 1.0e-16, 1.0e-16));
  REQUIRE (test_joint_axis == model->S[1]);

  Q[0] = M_PI * 0.5;
  QDot[0] = 1.;

  jcalc (*model, 1, Q, QDot);

  test_matrix <<
      0.,  1.,  0.,  0.,  0.,  0.,
      -1.,  0.,  0.,  0.,  0.,  0.,
      0.,  0.,  1.,  0.,  0.,  0.,
      0.,  0.,  0.,  0.,  1.,  0.,
      0.,  0.,  0., -1.,  0.,  0.,
      0.,  0.,  0.,  0.,  0.,  1.;

  REQUIRE_THAT (test_vector, AllCloseVector(model->v_J[1], 1.0e-16, 1.0e-16));
  REQUIRE (test_joint_axis == model->S[1]);
}

TEST_CASE_METHOD (ModelFixture, __FILE__"_TestTransformBaseToLocal", "") {
  Body body;

  unsigned int body_id = model->AddBody (0, SpatialTransform(),
      Joint (
        SpatialVector (0., 0., 0., 1., 0., 0.),
        SpatialVector (0., 0., 0., 0., 1., 0.),
        SpatialVector (0., 0., 0., 0., 0., 1.),
        SpatialVector (0., 0., 1., 0., 0., 0.),
        SpatialVector (0., 1., 0., 0., 0., 0.),
        SpatialVector (1., 0., 0., 0., 0., 0.)
        ),
      body);

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

  REQUIRE_THAT (base_coords, AllCloseVector(base_coords_back, TEST_PREC, TEST_PREC));

  q[0] = 1.;
  q[1] = 0.2;
  q[2] = -2.3;
  q[3] = -2.3;
  q[4] = 0.03;
  q[5] = -0.23;

  UpdateKinematics (*model, q, qdot, qddot);
  body_coords = CalcBaseToBodyCoordinates (*model, q, body_id, base_coords, false);
  base_coords_back = CalcBodyToBaseCoordinates (*model, q, body_id, body_coords, false);

  REQUIRE_THAT (base_coords, AllCloseVector(base_coords_back, TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_Model2DoFJoint", "") {
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

  REQUIRE_THAT (QDDot_std, AllCloseVector(QDDot_2, TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_Model3DoFJoint", "") {
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

  REQUIRE_THAT (QDDot_std, AllCloseVector(QDDot_2, TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_Model6DoFJoint", "") {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
  Joint joint_rot_y ( SpatialVector (0., 1., 0., 0., 0., 0.));
  Joint joint_rot_x ( SpatialVector (1., 0., 0., 0., 0., 0.));

  Model model_std;
  model_std.gravity = Vector3d (0., -9.81, 0.);

  unsigned int body_id;

  Joint joint_floating_base (
      SpatialVector (0., 0., 0., 1., 0., 0.),
      SpatialVector (0., 0., 0., 0., 1., 0.),
      SpatialVector (0., 0., 0., 0., 0., 1.),
      SpatialVector (0., 0., 1., 0., 0., 0.),
      SpatialVector (0., 1., 0., 0., 0., 0.),
      SpatialVector (1., 0., 0., 0., 0., 0.)
      );
  body_id = model_std.AddBody (0, SpatialTransform(), joint_floating_base, body);

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
  body_id = model_2.AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint_floating_base, body);
  body_id = model_2.AddBody(body_id, Xtrans(Vector3d(1., 0., 0.)), joint_rot_zyx, body);

  VectorNd Q = VectorNd::Zero(model_std.dof_count);
  VectorNd QDot = VectorNd::Zero(model_std.dof_count);
  VectorNd Tau = VectorNd::Zero(model_std.dof_count);

  VectorNd QDDot_2 = VectorNd::Zero(model_std.dof_count);
  VectorNd QDDot_std = VectorNd::Zero(model_std.dof_count);

  REQUIRE  (model_std.q_size == model_2.q_size);

  ForwardDynamics (model_std, Q, QDot, Tau, QDDot_std);
  ForwardDynamics (model_2, Q, QDot, Tau, QDDot_2);

  REQUIRE_THAT (QDDot_std, AllCloseVector(QDDot_2, TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_ModelFixedJointQueryBodyId", "" ) {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
  unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");

  REQUIRE (fixed_body_id == model.GetBodyId("fixed_body"));
}

/*
 * Makes sure that when appending a body to a fixed joint the parent of the
 * newly added parent is actually the moving body that the fixed body is
 * attached to.
 */
TEST_CASE (__FILE__"_ModelAppendToFixedBody", "") {
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

  unsigned int movable_body = model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
  //	unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");
  unsigned int appended_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), joint_rot_z, body, "appended_body");

  REQUIRE (movable_body + 1 == appended_body_id);
  REQUIRE (movable_body == model.lambda[appended_body_id]);
}

// Adds a fixed body to another fixed body.
TEST_CASE (__FILE__"_ModelAppendFixedToFixedBody", "") {
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

  REQUIRE (movable_body + 1 == appended_body_id);
  REQUIRE (movable_body == model.lambda[appended_body_id]);
  REQUIRE (movable_mass + fixed_mass * 2. == model.mBodies[movable_body].mMass);

  REQUIRE (movable_body == model.mFixedBodies[fixed_body_id - model.fixed_body_discriminator].mMovableParent);
  REQUIRE (movable_body == model.mFixedBodies[fixed_body_2_id - model.fixed_body_discriminator].mMovableParent);

  double new_mass = 3.5;
  Vector3d new_com = (1. / new_mass) * (movable_mass * movable_com + fixed_mass * (fixed_com + fixed_displacement) + fixed_mass * (fixed_com + fixed_displacement * 2.));

  REQUIRE_THAT (new_com, AllCloseVector(model.mBodies[movable_body].mCenterOfMass, TEST_PREC, TEST_PREC));
}

// Ensures that the transformations of the movable parent and fixed joint
// frame is in proper order
TEST_CASE (__FILE__"_ModelFixedJointRotationOrderTranslationRotation", "") {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

  SpatialTransform trans_x = Xtrans (Vector3d (1., 0., 0.));
  SpatialTransform rot_z = Xrotz (45. * M_PI / 180.);

  model.AddBody (0, trans_x, joint_rot_z, body);
  model.AppendBody (rot_z, Joint(JointTypeFixed), fixed_body, "fixed_body");
  unsigned int body_after_fixed = model.AppendBody (trans_x, joint_rot_z, body);

  VectorNd Q (VectorNd::Zero(model.dof_count));
  Q[0] = 45 * M_PI / 180.;
  Vector3d point = CalcBodyToBaseCoordinates (model, Q, body_after_fixed, Vector3d (0., 1., 0.));

  REQUIRE_THAT (Vector3d (0., 1., 0.), AllCloseVector(point, TEST_PREC, TEST_PREC));
}

// Ensures that the transformations of the movable parent and fixed joint
// frame is in proper order
TEST_CASE (__FILE__"_ModelFixedJointRotationOrderRotationTranslation", "") {
  // the standard modeling using a null body
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

  SpatialTransform rot_z = Xrotz (45. * M_PI / 180.);
  SpatialTransform trans_x = Xtrans (Vector3d (1., 0., 0.));

  model.AddBody (0, rot_z, joint_rot_z, body);
  model.AppendBody (trans_x, Joint(JointTypeFixed), fixed_body, "fixed_body");
  unsigned int body_after_fixed = model.AppendBody (trans_x, joint_rot_z, body);

  VectorNd Q (VectorNd::Zero(model.dof_count));
  Q[0] = 45 * M_PI / 180.;
  Vector3d point = CalcBodyToBaseCoordinates (model, Q, body_after_fixed, Vector3d (0., 1., 0.));

  REQUIRE_THAT (Vector3d (-1., 2., 0.), AllCloseVector(point, TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_ModelGetBodyName", "") {
  Body null_body;
  Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
  Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

  Model model;

  Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

  model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
  unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");
  unsigned int appended_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), joint_rot_z, body, "appended_body");

  REQUIRE (string("fixed_body") == model.GetBodyName(fixed_body_id));
  REQUIRE (string("appended_body") == model.GetBodyName(appended_body_id));
  REQUIRE (string("") == model.GetBodyName(123));
}

TEST_CASE_METHOD (RotZRotZYXFixed, __FILE__"_ModelGetParentBodyId", "") {
  REQUIRE (0u == model->GetParentBodyId(0));
  REQUIRE (0u == model->GetParentBodyId(body_a_id));
  REQUIRE (body_a_id == model->GetParentBodyId(body_b_id));
}

TEST_CASE_METHOD(RotZRotZYXFixed, __FILE__"_ModelGetParentIdFixed", "") {
  REQUIRE (body_b_id == model->GetParentBodyId(body_fixed_id));
}

TEST_CASE_METHOD(RotZRotZYXFixed, __FILE__"_ModelGetJointFrame", "") {
  SpatialTransform transform_a = model->GetJointFrame (body_a_id);
  SpatialTransform transform_b = model->GetJointFrame (body_b_id);
  SpatialTransform transform_root = model->GetJointFrame (0);

  REQUIRE_THAT (fixture_transform_a.r, AllCloseVector(transform_a.r, 0., 0.));
  REQUIRE_THAT (fixture_transform_b.r, AllCloseVector(transform_b.r, 0., 0.));
  REQUIRE_THAT (Vector3d(0., 0., 0.), AllCloseVector(transform_root.r, 0., 0.));
}

TEST_CASE_METHOD(RotZRotZYXFixed, __FILE__"_ModelGetJointFrameFixed", "") {
  SpatialTransform transform_fixed = model->GetJointFrame (body_fixed_id);

  REQUIRE_THAT (fixture_transform_fixed.r, AllCloseVector(transform_fixed.r, 0., 0.));
}

TEST_CASE_METHOD(RotZRotZYXFixed, __FILE__"_ModelSetJointFrame", "") {
  SpatialTransform new_transform_a = Xtrans (Vector3d(-1., -2., -3.));
  SpatialTransform new_transform_b = Xtrans (Vector3d(-4., -5., -6.));
  SpatialTransform new_transform_root = Xtrans (Vector3d(-99, -99., -99.));

  model->SetJointFrame (body_a_id, new_transform_a);
  model->SetJointFrame (body_b_id, new_transform_b);
  model->SetJointFrame (0, new_transform_root);

  SpatialTransform transform_a = model->GetJointFrame (body_a_id);
  SpatialTransform transform_b = model->GetJointFrame (body_b_id);
  SpatialTransform transform_root = model->GetJointFrame (0);

  REQUIRE_THAT (new_transform_a.r, AllCloseVector(transform_a.r, 0., 0.));
  REQUIRE_THAT (new_transform_b.r, AllCloseVector(transform_b.r, 0., 0.));
  REQUIRE_THAT (Vector3d(0., 0., 0.), AllCloseVector(transform_root.r, 0., 0.));
}

TEST_CASE (__FILE__"_CalcBodyWorldOrientationFixedJoint", "") {
  Model model_fixed;
  Model model_movable;

  Body body (1., Vector3d (1., 1., 1.), Vector3d (1., 1., 1.));
  Joint joint_fixed (JointTypeFixed);
  Joint joint_rot_x = (SpatialVector (1., 0., 0., 0., 0., 0.));

  model_fixed.AppendBody (Xrotx (45 * M_PI / 180), joint_rot_x, body);
  unsigned int body_id_fixed = model_fixed.AppendBody (Xroty (45 * M_PI / 180), joint_fixed, body);

  model_movable.AppendBody (Xrotx (45 * M_PI / 180), joint_rot_x, body);
  unsigned int body_id_movable = model_movable.AppendBody (Xroty (45 * M_PI / 180), joint_rot_x, body);

  VectorNd q_fixed (VectorNd::Zero (model_fixed.q_size));
  VectorNd q_movable (VectorNd::Zero (model_movable.q_size));

  Matrix3d E_fixed = CalcBodyWorldOrientation (model_fixed, q_fixed, body_id_fixed);
  Matrix3d E_movable = CalcBodyWorldOrientation (model_movable, q_movable, body_id_movable);

  REQUIRE_THAT (E_movable, AllCloseMatrix(E_fixed, TEST_PREC, TEST_PREC));
}

TEST_CASE (__FILE__"_TestAddFixedBodyToRoot", "") {
  Model model;

  Body body (1., Vector3d (1., 1., 1.), Vector3d (1., 1., 1.));
  Joint joint_fixed (JointTypeFixed);

  // Add a fixed body
  unsigned int body_id_fixed = model.AppendBody (Xtrans(Vector3d (1., 0., 0.)), joint_fixed, body, "FixedBody");

  // Add a second fixed body
  unsigned int body_id_fixed2 = model.AppendBody (Xtrans(Vector3d (1., 0., 0.)), joint_fixed, body, "FixedBody2");

  // Add a mobile boby
  unsigned int movable_body = model.AppendBody (Xrotx (45 * M_PI / 180), JointTypeRevoluteX, body, "MovableBody");

<<<<<<< HEAD
  CHECK_EQUAL ((unsigned int) 2, model.mBodies.size());
  CHECK_EQUAL ((unsigned int) 2, model.mFixedBodies.size());
=======
  REQUIRE (2 == model.mBodies.size());
  REQUIRE (2 == model.mFixedBodies.size());
>>>>>>> Moved all tests into Catch2 framework

  VectorNd q = VectorNd::Zero(model.q_size);

  Vector3d base_coords = CalcBodyToBaseCoordinates(model, q, body_id_fixed, Vector3d::Zero());
  REQUIRE_THAT (Vector3d (1., 0., 0.), AllCloseVector(base_coords, 0., 0.));

  base_coords = CalcBodyToBaseCoordinates(model, q, body_id_fixed2, Vector3d::Zero());
  REQUIRE_THAT (Vector3d (2., 0., 0.), AllCloseVector(base_coords, 0., 0.));

  base_coords = CalcBodyToBaseCoordinates(model, q, movable_body, Vector3d::Zero());
  REQUIRE_THAT (Vector3d (2., 0., 0.), AllCloseVector(base_coords, 0., 0.));
}

