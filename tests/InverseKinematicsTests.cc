#include <UnitTest++.h>

#include <iostream>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/rbdl_utils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"

#include "Human36Fixture.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-12;

void print_ik_set (const InverseKinematicsConstraintSet &cs) {
  int label_width = 18;
  cout.width (label_width);
  cout << "lambda: " << cs.lambda << endl;
  cout.width (label_width);
  cout << "num_steps: " << cs.num_steps << endl;
  cout.width (label_width);
  cout << "max_steps: " << cs.max_steps << endl;
  cout.width (label_width);
  cout << "step_tol: " << cs.step_tol << endl;
  cout.width (label_width);
  cout << "constraint_tol: " << cs.constraint_tol << endl;
  cout.width (label_width);
  cout << "error_norm: " << cs.error_norm << endl;
}

/// Checks whether a single point in a 3-link chain can be reached
TEST_FIXTURE ( Human36, ChainSinglePointConstraint ) {
  q[HipRightRY] = 0.3;
  q[HipRightRX] = 0.3;
  q[HipRightRZ] = 0.3;
  q[KneeRightRY] = 0.3;
  q[AnkleRightRY] = 0.3;
  q[AnkleRightRZ] = 0.3;

  Vector3d local_point (1., 0., 0.);

  UpdateKinematicsCustom (*model, &q, NULL, NULL);
  Vector3d target_position = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyFootRight], local_point);

  q.setZero();

  InverseKinematicsConstraintSet cs;
  cs.AddPointConstraint (body_id_emulated[BodyFootRight], local_point, target_position);

  VectorNd qres (q);

  bool result = InverseKinematics (*model, q, cs, qres);
  if (!result) {
    print_ik_set (cs);
  }

  CHECK (result);

  CHECK_CLOSE (0., cs.error_norm, TEST_PREC);
}


TEST_FIXTURE ( Human36, ManyPointConstraints ) {

  randomizeStates();

  Vector3d local_point1 (1., 0., 0.);
  Vector3d local_point2 (-1., 0., 0.);
  Vector3d local_point3 (0., 1., 0.);
  Vector3d local_point4 (1., 0., 1.);
  Vector3d local_point5 (0.,0.,-1.);

  UpdateKinematicsCustom (*model, &q, NULL, NULL);
  Vector3d target_position1 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyFootRight], local_point1);
  Vector3d target_position2 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyFootLeft], local_point2);
  Vector3d target_position3 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyHandRight], local_point3);
  Vector3d target_position4 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyHandLeft], local_point4);
  Vector3d target_position5 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyHead], local_point5);

  q.setZero();
  UpdateKinematicsCustom (*model, &q, NULL, NULL);

  InverseKinematicsConstraintSet cs;
  cs.AddPointConstraint (body_id_emulated[BodyFootRight], local_point1, target_position1);
  cs.AddPointConstraint (body_id_emulated[BodyFootLeft], local_point2, target_position2);
  cs.AddPointConstraint (body_id_emulated[BodyHandRight], local_point3, target_position3);
  cs.AddPointConstraint (body_id_emulated[BodyHandLeft], local_point4, target_position4);
  cs.AddPointConstraint (body_id_emulated[BodyHead], local_point5, target_position5);
  VectorNd qres (q);

  bool result = InverseKinematics (*model, q, cs, qres);

  CHECK (result);

  CHECK_CLOSE (0., cs.error_norm, TEST_PREC);

  UpdateKinematicsCustom (*model, &qres, NULL, NULL);  
  Vector3d result_position1 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyFootRight], local_point1);
  Vector3d result_position2 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyFootLeft], local_point2);
  Vector3d result_position3 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyHandRight], local_point3);
  Vector3d result_position4 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyHandLeft], local_point4);
  Vector3d result_position5 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyHead], local_point5);

  CHECK_ARRAY_CLOSE (target_position1.data(), result_position1.data(), 3, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_position2.data(), result_position2.data(), 3, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_position3.data(), result_position3.data(), 3, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_position4.data(), result_position4.data(), 3, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_position5.data(), result_position5.data(), 3, TEST_PREC); 
}

/// Checks whether the end of a 3-link chain can aligned with a given
// orientation.
TEST_FIXTURE ( Human36, ChainSingleBodyOrientation ) {
  q[HipRightRY] = 0.3;
  q[HipRightRX] = 0.3;
  q[HipRightRZ] = 0.3;
  q[KneeRightRY] = 0.3;
  q[AnkleRightRY] = 0.3;
  q[AnkleRightRZ] = 0.3;

  UpdateKinematicsCustom (*model, &q, NULL, NULL);
  Matrix3d target_orientation = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyFootRight], false);

  InverseKinematicsConstraintSet cs;
  cs.AddOrientationConstraint (body_id_emulated[BodyFootRight], target_orientation);

  VectorNd qres (q);
  q.setZero();

  bool result = InverseKinematics (*model, q, cs, qres);

  CHECK (result);
}

TEST_FIXTURE ( Human36, ManyBodyOrientations ) {

  randomizeStates();

  UpdateKinematicsCustom (*model, &q, NULL, NULL);
  Matrix3d target_orientation1 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyFootRight], false);
  Matrix3d target_orientation2 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyFootLeft], false);
  Matrix3d target_orientation3 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyHandRight], false);
  Matrix3d target_orientation4 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyHandLeft], false);
  Matrix3d target_orientation5 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyHead], false);

  q.setZero();

  InverseKinematicsConstraintSet cs;
  cs.AddOrientationConstraint (body_id_emulated[BodyFootRight], target_orientation1);
  cs.AddOrientationConstraint (body_id_emulated[BodyFootLeft], target_orientation2);
  cs.AddOrientationConstraint (body_id_emulated[BodyHandRight], target_orientation3);
  cs.AddOrientationConstraint (body_id_emulated[BodyHandLeft], target_orientation4);
  cs.AddOrientationConstraint (body_id_emulated[BodyHead], target_orientation5);

  VectorNd qres (q);

  bool result = InverseKinematics (*model, q, cs, qres);

  CHECK (result);

  CHECK_CLOSE (0., std::min( cs.error_norm, cs.delta_q_norm ), TEST_PREC);



  UpdateKinematicsCustom (*model, &qres, NULL, NULL);  
  Matrix3d result_orientation1 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyFootRight], false);
  Matrix3d result_orientation2 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyFootLeft], false);
  Matrix3d result_orientation3 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyHandRight], false);
  Matrix3d result_orientation4 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyHandLeft], false);
  Matrix3d result_orientation5 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyHead], false);

  CHECK_ARRAY_CLOSE (target_orientation1.data(), result_orientation1.data(), 9, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_orientation2.data(), result_orientation2.data(), 9, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_orientation3.data(), result_orientation3.data(), 9, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_orientation4.data(), result_orientation4.data(), 9, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_orientation5.data(), result_orientation5.data(), 9, TEST_PREC); 
}

TEST_FIXTURE ( Human36, ChainSingleBodyFullConstraint ) {
  q[HipRightRY] = 0.3;
  q[HipRightRX] = 0.3;
  q[HipRightRZ] = 0.3;
  q[KneeRightRY] = 0.3;
  q[AnkleRightRY] = 0.3;
  q[AnkleRightRZ] = 0.3;
  Vector3d local_point (1., 0., 0.);

  UpdateKinematicsCustom (*model, &q, NULL, NULL);
  Matrix3d target_orientation = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyFootRight], false);
  Vector3d target_position = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyFootRight], local_point);

  InverseKinematicsConstraintSet cs;
  cs.AddFullConstraint(body_id_emulated[BodyFootRight],local_point,target_position, target_orientation);

  VectorNd qres (q);
  q.setZero();

  bool result = InverseKinematics (*model, q, cs, qres);

  CHECK (result);
}

TEST_FIXTURE ( Human36, ManyBodyFullConstraints ) {

  randomizeStates();

  Vector3d local_point1 (1., 0., 0.);
  Vector3d local_point2 (-1., 0., 0.);
  Vector3d local_point3 (0., 1., 0.);
  Vector3d local_point4 (1., 0., 1.);
  Vector3d local_point5 (0.,0.,-1.);

  UpdateKinematicsCustom (*model, &q, NULL, NULL);

  Vector3d target_position1 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyFootRight], local_point1);
  Vector3d target_position2 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyFootLeft], local_point2);
  Vector3d target_position3 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyHandRight], local_point3);
  Vector3d target_position4 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyHandLeft], local_point4);
  Vector3d target_position5 = CalcBodyToBaseCoordinates (*model, q, body_id_emulated[BodyHead], local_point5);

  Matrix3d target_orientation1 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyFootRight], false);
  Matrix3d target_orientation2 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyFootLeft], false);
  Matrix3d target_orientation3 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyHandRight], false);
  Matrix3d target_orientation4 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyHandLeft], false);
  Matrix3d target_orientation5 = CalcBodyWorldOrientation (*model, q, body_id_emulated[BodyHead], false);

  InverseKinematicsConstraintSet cs;
  cs.AddFullConstraint (body_id_emulated[BodyFootRight], local_point1, target_position1, target_orientation1);
  cs.AddFullConstraint (body_id_emulated[BodyFootLeft],  local_point2, target_position2, target_orientation2);
  cs.AddFullConstraint (body_id_emulated[BodyHandRight], local_point3, target_position3, target_orientation3);
  cs.AddFullConstraint (body_id_emulated[BodyHandLeft],  local_point4, target_position4, target_orientation4);
  cs.AddFullConstraint (body_id_emulated[BodyHead],      local_point5, target_position5, target_orientation5);  
  cs.step_tol = 1e-12;

  q.setZero();

  VectorNd qres (q);

  bool result = InverseKinematics (*model, q, cs, qres);

  CHECK (result);

  CHECK_CLOSE (0., cs.error_norm, cs.step_tol);

  UpdateKinematicsCustom (*model, &qres, NULL, NULL);  
  Matrix3d result_orientation1 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyFootRight], false);
  Matrix3d result_orientation2 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyFootLeft], false);
  Matrix3d result_orientation3 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyHandRight], false);
  Matrix3d result_orientation4 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyHandLeft], false);
  Matrix3d result_orientation5 = CalcBodyWorldOrientation (*model, qres, body_id_emulated[BodyHead], false);

  Vector3d result_position1 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyFootRight], local_point1);
  Vector3d result_position2 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyFootLeft], local_point2);
  Vector3d result_position3 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyHandRight], local_point3);
  Vector3d result_position4 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyHandLeft], local_point4);
  Vector3d result_position5 = CalcBodyToBaseCoordinates (*model, qres, body_id_emulated[BodyHead], local_point5);

  CHECK_ARRAY_CLOSE (target_position1.data(), result_position1.data(), 3, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_position2.data(), result_position2.data(), 3, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_position3.data(), result_position3.data(), 3, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_position4.data(), result_position4.data(), 3, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_position5.data(), result_position5.data(), 3, TEST_PREC);

  CHECK_ARRAY_CLOSE (target_orientation1.data(), result_orientation1.data(), 9, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_orientation2.data(), result_orientation2.data(), 9, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_orientation3.data(), result_orientation3.data(), 9, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_orientation4.data(), result_orientation4.data(), 9, TEST_PREC); 
  CHECK_ARRAY_CLOSE (target_orientation5.data(), result_orientation5.data(), 9, TEST_PREC); 
}
