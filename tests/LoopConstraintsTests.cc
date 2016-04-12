#include <UnitTest++.h>

#include <iostream>
#include "rbdl/rbdl.h"
#include <cassert>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;

struct FiveBarLinkage {

  FiveBarLinkage()
    : model()
    , cs()
    , q()
    , qd()
    , qdd()
    , tau()
    , l1(2.)
    , l2(2.)
    , m1(2.)
    , m2(2.)
    , idB1(0)
    , idB2(0)
    , idB3(0)
    , idB4(0)
    , idB5(0)
    , X_p(Xtrans(Vector3d(l2, 0., 0.)))
    , X_s(Xtrans(Vector3d(0., 0., 0.))) {
    
    Body link1 = Body(
      m1,
      Vector3d(0.5 * l1, 0., 0.),
      Vector3d(0., 0., m1 * l1 * l1 / 3.));
    Body link2 = Body(
      m2,
      Vector3d(0.5 * l2, 0., 0.),
      Vector3d(0., 0., m2 * l2 * l2 / 3.));
    Body virtualBody = Body(0., Vector3d(), Vector3d());
    Joint jointRevZ = Joint(JointTypeRevoluteZ);

    idB1 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.))
      , jointRevZ, link1);
    idB2 = model.AddBody(idB1, Xtrans(Vector3d(l1, 0., 0.))
      , jointRevZ, link2);
    idB3 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.))
      , jointRevZ, link1);
    idB4 = model.AddBody(idB3, Xtrans(Vector3d(l1, 0., 0.))
      , jointRevZ, link2);
    idB5 = model.AddBody(idB4, Xtrans(Vector3d(l2, 0., 0.))
      , jointRevZ, virtualBody);

    cs.AddLoopConstraint(
      idB2, idB5,
      X_p, X_s,
      SpatialVector(0,0,0,1,0,0)
      , 0.1);
    cs.AddLoopConstraint(
      idB2, idB5,
      X_p, X_s,
      SpatialVector(0,0,0,0,1,0)
      , 0.1);
    cs.AddLoopConstraint(
      idB2, idB5,
      X_p, X_s,
      SpatialVector(0,0,1,0,0,0)
      , 0.1);

    cs.Bind(model);

    q = VectorNd::Zero(model.dof_count);
    qd = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);

  }

  Model model;
  ConstraintSet cs;

  VectorNd q;
  VectorNd qd;
  VectorNd qdd;
  VectorNd tau;

  double l1;
  double l2;
  double m1;
  double m2;

  unsigned int idB1;
  unsigned int idB2;
  unsigned int idB3;
  unsigned int idB4;
  unsigned int idB5;

  SpatialTransform X_p;
  SpatialTransform X_s;

};



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageConstraintErrors) {

  VectorNd err = VectorNd::Zero(cs.size());
  Vector3d pos1;
  Vector3d pos2;
  Vector3d posErr;
  Matrix3d rot_p;
  double angleErr;

  // Test in zero position.
  q[0] = 0.;
  q[1] = 0.;
  q[2] = 0.;
  q[3] = 0.;
  q[4] = 0.;

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_CLOSE(0., err[0], TEST_PREC);
  CHECK_CLOSE(0., err[1], TEST_PREC);
  CHECK_CLOSE(0., err[2], TEST_PREC);



  // Test in non-zero position.
  q[0] = M_PI * 3 / 4;
  q[1] = -M_PI;
  q[2] = M_PI - q[0];
  q[3] = -q[1];
  q[4] = 0.;
  angleErr = cos(M_PI);

  pos1 = CalcBodyToBaseCoordinates(model, q, idB2, X_p.r);
  pos2 = CalcBodyToBaseCoordinates(model, q, idB5, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, idB2, false) * X_p.E;
  posErr = rot_p.transpose() * (pos2 - pos1);

  assert(std::fabs(posErr[1]) < TEST_PREC);
  assert(std::fabs(posErr[2]) < TEST_PREC);

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_CLOSE(posErr[0], err[0], TEST_PREC);
  CHECK_CLOSE(0., err[1], TEST_PREC);
  CHECK_CLOSE(angleErr, err[2], TEST_PREC);



  // Test in non-zero position.
  q[0] = 0.;
  q[1] = 0.;
  q[2] = M_PI + 0.1;
  q[3] = 0.;
  q[4] = 0.;
  angleErr = cos(q[0] + q[1] - q[2] - q[3] - q[4] + 0.5 * M_PI);

  pos1 = CalcBodyToBaseCoordinates(model, q, idB2, X_p.r);
  pos2 = CalcBodyToBaseCoordinates(model, q, idB5, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, idB2, false) * X_p.E;
  posErr = rot_p.transpose() * (pos2 - pos1);

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_CLOSE(posErr[0], err[0], TEST_PREC);
  CHECK_CLOSE(posErr[1], err[1], TEST_PREC);
  CHECK_CLOSE(angleErr, err[2], TEST_PREC);



  // Test in non-zero position.
  q[0] = 0.8;
  q[1] = -0.4;
  q[2] = M_PI - q[0];
  q[3] = -q[1];
  q[4] = 0.;
  angleErr = cos(q[0] + q[1] - q[2] - q[3] - q[4] + 0.5 * M_PI);

  pos1 = CalcBodyToBaseCoordinates(model, q, idB2, X_p.r);
  pos2 = CalcBodyToBaseCoordinates(model, q, idB5, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, idB2, false) * X_p.E;
  posErr = rot_p.transpose() * (pos2 - pos1);

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_CLOSE(posErr[0], err[0], TEST_PREC);
  CHECK_CLOSE(posErr[1], err[1], TEST_PREC);
  CHECK_CLOSE(angleErr, err[2], TEST_PREC);

}



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageConstraintJacobian) {

  MatrixNd G = MatrixNd::Zero(cs.size(), q.size());
  VectorNd err = MatrixNd::Zero(cs.size(), 1);

 

  // Zero Q configuration, both arms of the 4-bar laying on the x-axis
  q[0] = 0.;
  q[1] = 0.;
  q[2] = 0.;
  q[3] = 0.;
  q[4] = 0.;
  assert(q[0] + q[1] - q[2] - q[3] - q[4] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = -1.;
  qd[1] = -1.;
  qd[2] = -1.;
  qd[3] = -1.;
  qd[4] = 0.;
  assert((CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity6D(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  CalcConstraintsJacobian(model, q, cs, G);

  err = G * qd;

  CHECK_ARRAY_CLOSE(VectorNd::Zero(cs.size()), err, cs.size(), TEST_PREC);



  // Both arms of the 4-bar laying on the y-axis
  q[0] = 0.5 * M_PI;
  q[1] = 0.;
  q[2] = 0.5 * M_PI;
  q[3] = 0.;
  q[4] = 0.;
  assert(q[0] + q[1] - q[2] - q[3] - q[4] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = -1.;
  qd[1] = -1.;
  qd[2] = -1.;
  qd[3] = -1.;
  qd[4] = 0.;
  assert((CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity6D(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  CalcConstraintsJacobian(model, q, cs, G);

  err = G * qd;

  CHECK_ARRAY_CLOSE(VectorNd::Zero(cs.size()), err, cs.size(), TEST_PREC);



  // Arms symmetric wrt y axis.
  q[0] = M_PI * 3 / 4;
  q[1] = -0.5 * M_PI;
  q[2] = M_PI - q[0];
  q[3] = -q[1];
  q[4] = q[0] + q[1] - q[2] - q[3];
  assert(q[0] + q[1] - q[2] - q[3] - q[4] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = -1.;
  qd[1] = -1.;
  qd[2] = -2.;
  qd[3] = +1.;
  qd[4] = -1.;
  assert((CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity6D(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  CalcConstraintsJacobian(model, q, cs, G);

  err = G * qd;

  CHECK_ARRAY_CLOSE(VectorNd::Zero(cs.size()), err, cs.size(), TEST_PREC);

}



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageQAssembly) {

  VectorNd weights(q.size());

  weights[0] = 1.;
  weights[1] = 0.;
  weights[2] = 1.;
  weights[3] = 0.;
  weights[4] = 0.;

  VectorNd qRef = VectorNd::Zero(q.size());
  qRef[0] = M_PI * 3 / 4;
  qRef[1] = -0.5 * M_PI;
  qRef[2] = M_PI - qRef[0];
  qRef[3] = -qRef[1];
  qRef[4] = qRef[0] + qRef[1] - qRef[2] - qRef[3];
  assert(qRef[0] + qRef[1] - qRef[2] - qRef[3] - qRef[4] == 0.);

  bool success;

  VectorNd qInit = VectorNd::Zero(q.size());
  qInit = qRef;
  
  success = ComputeAssemblyQ(model, qInit, cs, q, weights, 1.e-8);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(q[0] + q[1] , q[2] + q[3] + q[4], TEST_PREC);
  CHECK_CLOSE(qInit[0], q[0], TEST_PREC);
  CHECK_CLOSE(qInit[2], q[2], TEST_PREC);

  qInit[0] = qRef[0] + 0.01;
  qInit[1] = qRef[1] + 0.02;
  qInit[2] = qRef[2] - 0.03;
  qInit[3] = qRef[3] - 0.02;
  qInit[4] = qRef[4] + 0.01;

  success = ComputeAssemblyQ(model, qInit, cs, q, weights, 1.e-8);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(q[0] + q[1] , q[2] + q[3] + q[4], TEST_PREC);
  CHECK_CLOSE(qInit[0], q[0], TEST_PREC);
  CHECK_CLOSE(qInit[2], q[2], TEST_PREC);

}



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageQDotAssembly) {

  VectorNd weights(q.size());

  weights[0] = 1.;
  weights[1] = 0.;
  weights[2] = 1.;
  weights[3] = 0.;
  weights[4] = 0.;

  q[0] = M_PI * 3 / 4;
  q[1] = -0.5 * M_PI;
  q[2] = M_PI - q[0];
  q[3] = -q[1];
  q[4] = q[0] + q[1] - q[2] - q[3];
  assert(q[0] + q[1] - q[2] - q[3] - q[4] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  VectorNd qdInit = VectorNd::Zero(q.size());
  qdInit[0] = 0.01;
  qdInit[1] = 0.5;
  qdInit[2] = -0.7;
  qdInit[3] = -0.5;
  qdInit[4] = 0.3;

  ComputeAssemblyQDot(model, q, qdInit, cs, qd, weights);

  CHECK_CLOSE(qd[0] + qd[1], qd[2] + qd[3] + qd[4], TEST_PREC);

  MatrixNd G = MatrixNd::Zero(cs.size(), q.size());
  VectorNd err = MatrixNd::Zero(cs.size(), 1);
  CalcConstraintsJacobian(model, q, cs, G);
  err = G * qd;

  CHECK_ARRAY_CLOSE(CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    , CalcPointVelocity6D(model, q, qd, idB5, X_s.r), 6, TEST_PREC);
  CHECK_ARRAY_CLOSE(VectorNd::Zero(cs.size()), err, cs.size(), TEST_PREC);
  CHECK_CLOSE(qdInit[0], qd[0], TEST_PREC);
  CHECK_CLOSE(qdInit[2], qd[2], TEST_PREC);

}



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageDynamicsDirect) {

  VectorNd qddDirect;
  VectorNd qddRangeSparse;
  VectorNd qddNullSpace;

  // cs.SetSolver(LinearSolverPartialPivLU);
  // cs.SetSolver(LinearSolverColPivHouseholderQR);
  cs.SetSolver(LinearSolverHouseholderQR);

  // Configuration 1.

  q[0] = 0.;
  q[1] = 0.;
  q[2] = 0.;
  q[3] = 0.;
  q[4] = 0.;
  assert(q[0] + q[1] - q[2] - q[3] - q[4] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = 0.;
  qd[1] = 0.;
  qd[2] = 0.;
  qd[3] = 0.;
  qd[4] = 0.;
  assert(qd[0] + qd[1] - qd[2] - qd[3] - qd[4] == 0.);
  assert((CalcPointVelocity(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  tau[0] = 1.;
  tau[1] = -2.;
  tau[2] = 3.;
  tau[3] = -5.;
  tau[4] = 7.;



  qddDirect = VectorNd::Zero(q.size());
  ForwardDynamicsConstrainedDirect(model, q, qd, tau, cs, qddDirect);

  std::cout << "H" << std::endl << cs.H << std::endl << std::endl;
  std::cout << "G" << std::endl << cs.G << std::endl << std::endl;
  std::cout << "C" << std::endl << cs.C.transpose() << std::endl << std::endl;
  std::cout << "gamma" << std::endl << cs.gamma.transpose() << std::endl << std::endl;
  std::cout << "qd" << std::endl << qddDirect.transpose() << std::endl << std::endl;

  CHECK_ARRAY_CLOSE(
    CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r).data(),
    CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r).data(),
    6, TEST_PREC);



  // qddRangeSparse = VectorNd::Zero(q.size());
  // ForwardDynamicsConstrainedRangeSpaceSparse(model, q, qd, tau, cs, qddRangeSparse);

  // CHECK_ARRAY_CLOSE(
  //   CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r).data(),
  //   CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r).data(),
  //   6, TEST_PREC);



  // qddNullSpace = VectorNd::Zero(q.size());
  // ForwardDynamicsConstrainedNullSpace(model, q, qd, tau, cs, qddNullSpace);

  // CHECK_ARRAY_CLOSE(
  //   CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r).data(),
  //   CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r).data(),
  //   6, TEST_PREC);



  // CHECK_ARRAY_CLOSE(qddDirect.data(), qddRangeSparse.data(), q.size(), TEST_PREC);
  // CHECK_ARRAY_CLOSE(qddDirect.data(), qddNullSpace.data(), q.size(), TEST_PREC);



  // Configuration 2.

  q[0] = M_PI * 3 / 4;
  q[1] = -0.5 * M_PI;
  q[2] = M_PI - q[0];
  q[3] = -q[1];
  q[4] = q[0] + q[1] - q[2] - q[3];
  assert(q[0] + q[1] - q[2] - q[3] - q[4] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = -1.;
  qd[1] = -1.;
  qd[2] = -2.;
  qd[3] = +1.;
  qd[4] = -1.;
  assert(qd[0] + qd[1] - qd[2] - qd[3] - qd[4] == 0.);
  assert((CalcPointVelocity(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  tau[0] = 1.;
  tau[1] = -2.;
  tau[2] = 3.;
  tau[3] = -5.;
  tau[4] = 7.;



  qddDirect = VectorNd::Zero(q.size());
  ForwardDynamicsConstrainedDirect(model, q, qd, tau, cs, qddDirect);

  std::cout << "H" << std::endl << cs.H << std::endl << std::endl;
  std::cout << "G" << std::endl << cs.G << std::endl << std::endl;
  std::cout << "C" << std::endl << cs.C.transpose() << std::endl << std::endl;
  std::cout << "gamma" << std::endl << cs.gamma.transpose() << std::endl << std::endl;
  std::cout << "qd" << std::endl << qddDirect.transpose() << std::endl << std::endl;

  CHECK_ARRAY_CLOSE(
    CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r).data(),
    CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r).data(),
    6, TEST_PREC);



  qddRangeSparse = VectorNd::Zero(q.size());
  ForwardDynamicsConstrainedRangeSpaceSparse(model, q, qd, tau, cs, qddRangeSparse);

  std::cout << "H" << std::endl << cs.H << std::endl << std::endl;
  std::cout << "G" << std::endl << cs.G << std::endl << std::endl;
  std::cout << "C" << std::endl << cs.C.transpose() << std::endl << std::endl;
  std::cout << "gamma" << std::endl << cs.gamma.transpose() << std::endl << std::endl;
  std::cout << "qd" << std::endl << qddRangeSparse.transpose() << std::endl << std::endl;

  CHECK_ARRAY_CLOSE(
    CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r).data(),
    CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r).data(),
    6, TEST_PREC);



  qddNullSpace = VectorNd::Zero(q.size());
  ForwardDynamicsConstrainedNullSpace(model, q, qd, tau, cs, qddNullSpace);

  std::cout << "H" << std::endl << cs.H << std::endl << std::endl;
  std::cout << "G" << std::endl << cs.G << std::endl << std::endl;
  std::cout << "C" << std::endl << cs.C.transpose() << std::endl << std::endl;
  std::cout << "gamma" << std::endl << cs.gamma.transpose() << std::endl << std::endl;
  std::cout << "qd" << std::endl << qddNullSpace.transpose() << std::endl << std::endl;

  CHECK_ARRAY_CLOSE(
    CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r).data(),
    CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r).data(),
    6, TEST_PREC);



  CHECK_ARRAY_CLOSE(qddDirect.data(), qddRangeSparse.data(), q.size(), TEST_PREC);
  CHECK_ARRAY_CLOSE(qddDirect.data(), qddNullSpace.data(), q.size(), TEST_PREC);

}
