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
    
    Body link1 = Body(m1, Vector3d(0.5 * l1, 0., 0.)
      , Vector3d(0., 0., m1 * l1 * l1 / 3.));
    Body link2 = Body(m2, Vector3d(0.5 * l2, 0., 0.)
      , Vector3d(0., 0., m2 * l2 * l2 / 3.));
    Body virtualBody(0., Vector3d(), Vector3d());
    Joint jointRevZ(JointTypeRevoluteZ);

    idB1 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)), jointRevZ, link1);
    idB2 = model.AddBody(idB1, Xtrans(Vector3d(l1, 0., 0.)), jointRevZ, link2);
    idB3 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)), jointRevZ, link1);
    idB4 = model.AddBody(idB3, Xtrans(Vector3d(l1, 0., 0.)), jointRevZ, link2);
    idB5 = model.AddBody(idB4, Xtrans(Vector3d(l2, 0., 0.)), jointRevZ
      , virtualBody);

    cs.AddLoopConstraint(idB2, idB5, X_p, X_s, SpatialVector(0,0,0,1,0,0), 0.1);
    cs.AddLoopConstraint(idB2, idB5, X_p, X_s, SpatialVector(0,0,0,0,1,0), 0.1);
    cs.AddLoopConstraint(idB2, idB5, X_p, X_s, SpatialVector(0,0,1,0,0,0), 0.1);

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



struct SliderCrank3D {

  SliderCrank3D()
    : model()
    , cs()
    , q()
    , qd()
    , id_p(0)
    , id_s(0)
    , X_p()
    , X_s() {

    double sliderMass = 5.;
    double sliderHeight = 0.1;
    double crankLink1Mass = 3.;
    double crankLink1Length = 1.;
    double crankLink2Mass = 1.;
    double crankLink2Radius = 0.2;
    double crankLink2Length = 3.;
    double crankLink1Height = crankLink2Length - crankLink1Length 
      + sliderHeight;

    Body slider(sliderMass, Vector3d::Zero(), Vector3d(1., 1., 1.));
    Body crankLink1(crankLink1Mass
      , Vector3d(0.5 * crankLink1Length, 0., 0.)
      , Vector3d(0., 0.
      , crankLink1Mass * crankLink1Length * crankLink1Length / 3.));
    Body crankLink2(crankLink2Mass
      , Vector3d(0.5 * crankLink2Length, 0., 0.)
      , Vector3d(crankLink2Mass * crankLink2Radius * crankLink2Radius / 2.
      , crankLink2Mass * (3. * crankLink2Radius * crankLink2Radius 
      + crankLink2Length * crankLink2Length) / 12.
      , crankLink2Mass * (3. * crankLink2Radius * crankLink2Radius 
      + crankLink2Length * crankLink2Length) / 12.));

    Joint jointRevZ(JointTypeRevoluteZ);
    Joint jointSphere(JointTypeEulerZYX);
    Joint jointPrsX(SpatialVector(0.,0.,0.,1.,0.,0.));

    id_p = model.AddBody(0
      , SpatialTransform()
      , jointPrsX, slider);
    unsigned int id_b1 = model.AddBody(0
      , Xroty(-0.5*M_PI) * Xtrans(Vector3d(0., 0., crankLink1Height)) 
      , jointRevZ, crankLink1);
    id_s = model.AddBody(id_b1
      , Xroty(M_PI) * Xtrans(Vector3d(crankLink1Length, 0., 0.))
      , jointSphere, crankLink2);

    X_p = Xtrans(Vector3d(0., 0., sliderHeight));
    X_s = SpatialTransform(roty(-0.5 * M_PI), Vector3d(crankLink2Length, 0, 0));

    cs.AddLoopConstraint(id_p, id_s, X_p, X_s, SpatialVector(0,0,0,1,0,0), 10);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s, SpatialVector(0,0,0,0,1,0), 10);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s, SpatialVector(0,0,0,0,0,1), 10);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s, SpatialVector(0,0,1,0,0,0), 10);

    cs.Bind(model);

    q = VectorNd::Zero(model.dof_count);
    qd = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);

    Matrix3d rot_ps 
      = (CalcBodyWorldOrientation(model, q, id_p).transpose() * X_p.E).transpose()
      * CalcBodyWorldOrientation(model, q, id_s).transpose() * X_s.E;
    assert(rot_ps(0,0) - 1. < TEST_PREC);
    assert(rot_ps(1,1) - 1. < TEST_PREC);
    assert(rot_ps(2,2) - 1. < TEST_PREC);
    assert(rot_ps(0,1) < TEST_PREC);
    assert(rot_ps(0,2) < TEST_PREC);
    assert(rot_ps(1,0) < TEST_PREC);
    assert(rot_ps(1,2) < TEST_PREC);
    assert(rot_ps(2,0) < TEST_PREC);
    assert(rot_ps(2,1) < TEST_PREC);
    assert((CalcBodyToBaseCoordinates(model, q, id_p, X_p.r)
      - CalcBodyToBaseCoordinates(model, q, id_s, X_s.r)).norm() < TEST_PREC);

  }

  Model model;
  ConstraintSet cs;

  VectorNd q;
  VectorNd qd;
  VectorNd qdd;
  VectorNd tau;

  unsigned int id_p;
  unsigned int id_s;
  SpatialTransform X_p;
  SpatialTransform X_s;

};



TEST(RBDLfunctions) {

  Body stick(1, Vector3d(0,0,0), Vector3d(1,1,1));

}



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
  rot_p = CalcBodyWorldOrientation(model, q, idB2).transpose() * X_p.E;
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
  rot_p = CalcBodyWorldOrientation(model, q, idB2).transpose() * X_p.E;
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
  rot_p = CalcBodyWorldOrientation(model, q, idB2).transpose() * X_p.E;
  posErr = rot_p.transpose() * (pos2 - pos1);

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_CLOSE(posErr[0], err[0], TEST_PREC);
  CHECK_CLOSE(posErr[1], err[1], TEST_PREC);
  CHECK_CLOSE(angleErr, err[2], TEST_PREC);

}



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageConstraintJacobian) {

  MatrixNd G(MatrixNd::Zero(cs.size(), q.size()));
  VectorNd err(VectorNd::Zero(cs.size()));
  VectorNd errRef(VectorNd::Zero(cs.size()));


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

  CHECK_ARRAY_CLOSE(errRef.data(), err.data(), cs.size(), TEST_PREC);



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

  CHECK_ARRAY_CLOSE(errRef.data(), err.data(), cs.size(), TEST_PREC);



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

  CHECK_ARRAY_CLOSE(errRef.data(), err.data(), cs.size(), TEST_PREC);

}



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageConstraintsVelocityErrors) {

  VectorNd errd(VectorNd::Zero(cs.size()));
  VectorNd errdRef(VectorNd::Zero(cs.size()));
  SpatialVector vj;

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

  CalcConstraintsVelocityError(model, q, qd, cs, errd);

  CHECK_ARRAY_CLOSE(errdRef.data(), errd.data(), cs.size(), TEST_PREC);

  // Invalid velocities.
  qd[0] = -1.;
  qd[1] = -1.;
  qd[2] = 0.;
  qd[3] = 0.;
  qd[4] = 0.;

  CalcConstraintsVelocityError(model, q, qd, cs, errd);

  vj = CalcPointVelocity6D(model, q, qd, idB5, X_s.r) 
    - CalcPointVelocity6D(model, q, qd, idB2, X_p.r);
  errdRef[0] = vj[3];
  errdRef[1] = vj[4];
  errdRef[2] = vj[2];
  CHECK_ARRAY_CLOSE(errdRef.data(), errd.data(), cs.size(), TEST_PREC);

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



  // Feasible initial guess.
  VectorNd qInit = VectorNd::Zero(q.size());
  qInit = qRef;
  
  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-8);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r).data()
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r).data(), 3, TEST_PREC);
  CHECK_CLOSE(q[0] + q[1] , q[2] + q[3] + q[4], TEST_PREC);
  CHECK_CLOSE(qInit[0], q[0], TEST_PREC);
  CHECK_CLOSE(qInit[2], q[2], TEST_PREC);



  // Perturbed initial guess.
  qInit[0] = qRef[0] + 0.01;
  qInit[1] = qRef[1] + 0.02;
  qInit[2] = qRef[2] - 0.03;
  qInit[3] = qRef[3] - 0.02;
  qInit[4] = qRef[4] + 0.01;

  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-8);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r).data()
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r).data(), 3, TEST_PREC);
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

  CalcAssemblyQDot(model, q, qdInit, cs, qd, weights);
  MatrixNd G(MatrixNd::Zero(cs.size(), q.size()));
  VectorNd err(VectorNd::Zero(cs.size()));
  VectorNd errRef(VectorNd::Zero(cs.size()));
  CalcConstraintsJacobian(model, q, cs, G);
  err = G * qd;

  CHECK_ARRAY_CLOSE(CalcPointVelocity6D(model, q, qd, idB2, X_p.r).data()
    , CalcPointVelocity6D(model, q, qd, idB5, X_s.r).data(), 6, TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef.data(), err.data(), cs.size(), TEST_PREC);
  CHECK_CLOSE(qdInit[0], qd[0], TEST_PREC);
  CHECK_CLOSE(qdInit[2], qd[2], TEST_PREC);

}



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageForwardDynamics) {

  VectorNd qddDirect;
  VectorNd qddNullSpace;

  cs.SetSolver(LinearSolverColPivHouseholderQR);

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

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r).data()
    , CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r).data()
    , 6, TEST_PREC);



  qddNullSpace = VectorNd::Zero(q.size());
  ForwardDynamicsConstrainedNullSpace(model, q, qd, tau, cs, qddNullSpace);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB2, X_p.r).data()
    , CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB5, X_s.r).data()
    , 6, TEST_PREC);



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

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r).data()
    , CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r).data()
    , 6, TEST_PREC);



  qddNullSpace = VectorNd::Zero(q.size());
  ForwardDynamicsConstrainedNullSpace(model, q, qd, tau, cs, qddNullSpace);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB2, X_p.r).data()
    , CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB5, X_s.r).data()
    , 6, TEST_PREC);

  // Note:
  // The Range Space Sparse method can't be used because the H matrix has a 0 on
  // the diagonal and the LTL factorization tries to divide by 0.

  // Note:
  // LinearSolverPartialPivLU does not work because the A matrix in the dynamic
  // system is not invertible.

  // Note:
  // LinearSolverHouseholderQR sometimes does not work well when the system is
  // in a singular configuration.

}



TEST_FIXTURE(SliderCrank3D, TestSliderCrank3DConstraintErrors) {

  VectorNd err(VectorNd::Zero(cs.size()));
  VectorNd errRef(VectorNd::Zero(cs.size()));
  Vector3d pos_p;
  Vector3d pos_s;
  Matrix3d rot_p;
  Matrix3d rot_s;
  Matrix3d rot_ps;
  Vector3d rotationVec;

  // Test in zero position.

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_ARRAY_CLOSE(errRef.data(), err.data(), cs.size(), TEST_PREC);

  // Test in another configurations.

  q[0] = 0.4;
  q[1] = 0.25 * M_PI;
  q[2] = -0.25 * M_PI;
  q[3] = 0.01;
  q[4] = 0.01;

  CalcConstraintsPositionError(model, q, cs, err);

  pos_p = CalcBodyToBaseCoordinates(model, q, id_p, X_p.r);
  pos_s = CalcBodyToBaseCoordinates(model, q, id_s, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, id_p).transpose() * X_p.E;
  rot_s = CalcBodyWorldOrientation(model, q, id_s).transpose() * X_s.E;
  rot_ps = rot_s.transpose() * rot_p;
  rotationVec = 0.5 * Vector3d(rot_ps(2,1) - rot_ps(1,2)
    , rot_ps(0,2) - rot_ps(2,0), rot_ps(1,0) - rot_ps(0,1));
  errRef.block<3,1>(0,0) = pos_s - pos_p;
  errRef(3) = rotationVec(2);

  CHECK_ARRAY_CLOSE(errRef.data(), err.data(), cs.size(), TEST_PREC);

}



TEST_FIXTURE(SliderCrank3D, TestSliderCrank3DConstraintJacobian) {

  MatrixNd G(MatrixNd::Zero(cs.size(), q.size()));

  // Test in zero position.

  G.setZero();
  CalcConstraintsJacobian(model, q, cs, G);

  VectorNd errRef(VectorNd::Zero(cs.size()));
  VectorNd err = G * qd;

  CHECK_ARRAY_CLOSE(errRef.data(), err.data(), cs.size(), TEST_PREC);

}



TEST_FIXTURE(SliderCrank3D, TestSliderCrank3DAssemblyQ) {

  VectorNd weights(q.size());
  VectorNd qInit(q.size());

  Vector3d pos_p;
  Vector3d pos_s;
  Matrix3d rot_p;
  Matrix3d rot_s;
  Matrix3d rot_ps;

  bool success;

  weights[0] = 1.;
  weights[1] = 1.;
  weights[2] = 1.;
  weights[3] = 1.;
  weights[4] = 1.;

  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  qInit[2] = -0.25 * M_PI;
  qInit[3] = 0.1;
  qInit[4] = 0.1;

  success = CalcAssemblyQ(model, qInit, cs, q, weights, TEST_PREC);
  pos_p = CalcBodyToBaseCoordinates(model, q, id_p, X_p.r);
  pos_s = CalcBodyToBaseCoordinates(model, q, id_s, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, id_p).transpose() * X_p.E;
  rot_s = CalcBodyWorldOrientation(model, q, id_s).transpose() * X_s.E;
  rot_ps = rot_s.transpose() * rot_p;

  CHECK(success);
  CHECK_ARRAY_CLOSE(pos_p.data(), pos_s.data(), 3, TEST_PREC);
  CHECK_CLOSE(0., rot_ps(1,0) - rot_ps(0,1), TEST_PREC);

}



TEST_FIXTURE(SliderCrank3D, TestSliderCrank3DAssemblyQDot) {

  VectorNd qWeights(q.size());
  VectorNd qdWeights(q.size());
  VectorNd qInit(q.size());
  VectorNd qdInit(q.size());

  SpatialVector vel_p;
  SpatialVector vel_s;

  bool success;

  qWeights[0] = 1.;
  qWeights[1] = 1.;
  qWeights[2] = 1.;
  qWeights[3] = 1.;
  qWeights[4] = 1.;

  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  qInit[2] = -0.25 * M_PI;
  qInit[3] = 0.1;
  qInit[4] = 0.1;

  qdWeights[0] = 1.;
  qdWeights[1] = 0.;
  qdWeights[2] = 0.;
  qdWeights[3] = 0.;
  qdWeights[4] = 0.;

  qdInit[0] = -0.2;
  qdInit[1] = 0.1 * M_PI;
  qdInit[2] = -0.1 * M_PI;
  qdInit[3] = 0.;
  qdInit[4] = 0.1 * M_PI;

  success = CalcAssemblyQ(model, qInit, cs, q, qWeights, TEST_PREC);
  assert(success);

  CalcAssemblyQDot(model, q, qdInit, cs, qd, qdWeights);

  vel_p = CalcPointVelocity6D(model, q, qd, id_p, X_p.r);
  vel_s = CalcPointVelocity6D(model, q, qd, id_s, X_s.r);

  CHECK_ARRAY_CLOSE(vel_p.block(2,0,4,1).data(), vel_s.block(2,0,4,1).data(), 4
    , TEST_PREC);
  CHECK_CLOSE(qdInit[0], qd[0], TEST_PREC);

}



TEST_FIXTURE(SliderCrank3D, TestSliderCrank3DForwardDynamics) {

  VectorNd qWeights(q.size());
  VectorNd qdWeights(q.size());
  VectorNd qInit(q.size());
  VectorNd qdInit(q.size());

  SpatialVector acc_p;
  SpatialVector acc_s;

  bool success;



  // Test with zero q and qdot.

  tau[0] = 0.12;
  tau[1] = -0.3;
  tau[2] = 0.05;
  tau[3] = 0.7;
  tau[4] = -0.1;

  ForwardDynamicsConstrainedDirect(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  CHECK_ARRAY_CLOSE(acc_p.block(2,0,4,1).data(), acc_s.block(2,0,4,1).data(), 4
    , TEST_PREC);

  ForwardDynamicsConstrainedNullSpace(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  CHECK_ARRAY_CLOSE(acc_p.block(2,0,4,1).data(), acc_s.block(2,0,4,1).data(), 4
    , TEST_PREC);

  ForwardDynamicsConstrainedRangeSpaceSparse(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  CHECK_ARRAY_CLOSE(acc_p.block(2,0,4,1).data(), acc_s.block(2,0,4,1).data(), 4
    , TEST_PREC);



  // Compute non-zero assembly q and qdot;

  qWeights[0] = 1.;
  qWeights[1] = 1.;
  qWeights[2] = 1.;
  qWeights[3] = 1.;
  qWeights[4] = 1.;

  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  qInit[2] = -0.25 * M_PI;
  qInit[3] = 0.1;
  qInit[4] = 0.1;

  qdWeights[0] = 1.;
  qdWeights[1] = 0.;
  qdWeights[2] = 0.;
  qdWeights[3] = 0.;
  qdWeights[4] = 0.;

  qdInit[0] = -0.2;
  qdInit[1] = 0.1 * M_PI;
  qdInit[2] = -0.1 * M_PI;
  qdInit[3] = 0.;
  qdInit[4] = 0.1 * M_PI;

  qdInit.setZero();

  success = CalcAssemblyQ(model, qInit, cs, q, qWeights, TEST_PREC);
  assert(success);
  CalcAssemblyQDot(model, q, qdInit, cs, qd, qdWeights);

  Matrix3d rot_ps 
    = (CalcBodyWorldOrientation(model, q, id_s).transpose() * X_s.E).transpose()
    * CalcBodyWorldOrientation(model, q, id_p).transpose() * X_p.E;
  assert((CalcBodyToBaseCoordinates(model, q, id_p, X_p.r)
    - CalcBodyToBaseCoordinates(model, q, id_p, X_p.r)).norm() < TEST_PREC);
  assert(rot_ps(0,1) - rot_ps(0,1) < TEST_PREC);
  assert((CalcPointVelocity6D(model, q, qd, id_p, X_p.r)
    -CalcPointVelocity6D(model, q, qd, id_p, X_p.r)).norm() < TEST_PREC);



  // Test with non-zero q and qdot.

  ForwardDynamicsConstrainedDirect(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  CHECK_ARRAY_CLOSE(acc_p.block(2,0,4,1).data(), acc_s.block(2,0,4,1).data(), 4
    , TEST_PREC);

  ForwardDynamicsConstrainedNullSpace(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  CHECK_ARRAY_CLOSE(acc_p.block(2,0,4,1).data(), acc_s.block(2,0,4,1).data(), 4
    , TEST_PREC);

  ForwardDynamicsConstrainedRangeSpaceSparse(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  CHECK_ARRAY_CLOSE(acc_p.block(2,0,4,1).data(), acc_s.block(2,0,4,1).data(), 4
    , TEST_PREC);

}