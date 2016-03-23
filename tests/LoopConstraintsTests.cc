#include <UnitTest++.h>

#include <iostream>
#include "rbdl/rbdl.h"

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
    , l2(3.)
    , m1(2.)
    , m2(3.)
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

    idB1 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)), jointRevZ, link1);
    idB2 = model.AddBody(idB1, Xtrans(Vector3d(l1, 0., 0.)), jointRevZ, link2);
    idB3 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)), jointRevZ, link1);
    idB4 = model.AddBody(idB3, Xtrans(Vector3d(l1, 0., 0.)), jointRevZ, link2);
    idB5 = model.AddBody(idB4, Xtrans(Vector3d(l2, 0., 0.)), jointRevZ, virtualBody);

    cs.AddLoopPositionConstraint(
      idB2, idB5,
      X_p.r, X_s.r,
      Vector3d(1,0,0), 0.1);
    cs.AddLoopPositionConstraint(
      idB2, idB5,
      X_p.r, X_s.r,
      Vector3d(0,1,0), 0.1);
    cs.AddLoopOrientationConstraint(
      idB2, idB5,
      X_p.E, X_s.E,
      Vector3d(0,0,1), 0.1);

    std::cout << cs.size() << std::endl;
    std::cout << model.dof_count << std::endl;

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



TEST_FIXTURE(FiveBarLinkage, TestFiveBarLinkageConstraint) {
  
  CHECK_EQUAL(3, cs.size());

  VectorNd weights(q.size());

  weights[0] = 1.e5;
  weights[1] = 1.;
  weights[2] = 1.e5;
  weights[3] = 1.;
  weights[4] = 1.;

  VectorNd qInit = VectorNd::Zero(q.size());
  qInit[0] = 1.;
  qInit[3] = -1.;

  VectorNd qdInit = VectorNd::Zero(q.size());
  qdInit[0] = 0.5;
  qdInit[3] = -0.5;

  tau[0] = 1.;
  tau[1] = -2.;
  tau[2] = 3.;
  tau[3] = -5.;
  tau[4] = 7.;

  q = qInit;
  qd = qdInit;

  ComputeAssemblyQ(model, qInit, cs, q, weights);

  CHECK_CLOSE(q[0] + q[1], q[2] + q[3] + q[4], TEST_PREC);

  ComputeAssemblyQDot(model, q, qdInit, cs, qd, weights);

  CHECK_CLOSE(q[0] + q[1], q[2] + q[3] + q[4], TEST_PREC);



  CHECK_ARRAY_CLOSE(CalcBaseToBodyCoordinates(model, q, idB2, X_p.r).data(),
    CalcBaseToBodyCoordinates(model, q, idB5, X_s.r).data(),
    3, TEST_PREC);

  CHECK_ARRAY_CLOSE(CalcBodyWorldOrientation(model, q, idB2).data(),
    CalcBodyWorldOrientation(model, q, idB5).data(),
    9, TEST_PREC);

  CHECK_ARRAY_CLOSE(CalcPointVelocity6D(model, q, qd, idB2, X_p.r).data(),
    CalcPointVelocity6D(model, q, qd, idB5, X_s.r).data(),
    6, TEST_PREC);



  // VectorNd qddDirect = VectorNd::Zero(q.size());
  // ForwardDynamicsConstrainedDirect(model, q, qd, tau, cs, qddDirect);

  // CHECK_ARRAY_CLOSE(CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, constrPos_p).data(),
  //   CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, constrPos_s).data(),
  //   6, TEST_PREC);



  // VectorNd qddRangeSparse = VectorNd::Zero(q.size());
  // ForwardDynamicsConstrainedRangeSpaceSparse(model, q, qd, tau, cs, qddRangeSparse);

  // CHECK_ARRAY_CLOSE(CalcPointAcceleration6D(model, q, qd, qddRangeSparse, idB2, constrPos_p).data(),
  //   CalcPointAcceleration6D(model, q, qd, qddRangeSparse, idB5, constrPos_s).data(),
  //   6, TEST_PREC);



  // VectorNd qddNullSpace = VectorNd::Zero(q.size());
  // ForwardDynamicsConstrainedNullSpace(model, q, qd, tau, cs, qddNullSpace);

  // CHECK_ARRAY_CLOSE(CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB2, constrPos_p).data(),
  //   CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB5, constrPos_s).data(),
  //   6, TEST_PREC);



  // CHECK_ARRAY_CLOSE(qddDirect.data(), qddRangeSparse.data(), q.size(), TEST_PREC);
  // CHECK_ARRAY_CLOSE(qddDirect.data(), qddNullSpace.data(), q.size(), TEST_PREC);

}