#include <UnitTest++.h>
#include "rbdl/rbdl.h"
#include <cassert>

#include "PendulumModels.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;

// Reduce an angle to the (-pi, pi] range.
static double inRange(double angle) {
  while(angle > M_PI) {
    angle -= 2. * M_PI;
  }
  while(angle <= -M_PI) {
    angle += 2. * M_PI;
  }
  return angle;
}





struct FourBarLinkage {

  FourBarLinkage()
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
    Vector3d vector3d_zero = Vector3d::Zero();
    Body virtual_body(0., vector3d_zero, vector3d_zero);
    Joint joint_rev_z(JointTypeRevoluteZ);

    idB1 = model.AddBody(0   , Xtrans(Vector3d(0., 0., 0.)),
                         joint_rev_z, link1);
    idB2 = model.AddBody(idB1, Xtrans(Vector3d(l1, 0., 0.)),
                         joint_rev_z, link2);
    idB3 = model.AddBody(0   , Xtrans(Vector3d(0., 0., 0.)),
                         joint_rev_z, link1);
    idB4 = model.AddBody(idB3, Xtrans(Vector3d(l1, 0., 0.)),
                         joint_rev_z, link2);
    idB5 = model.AddBody(idB4, Xtrans(Vector3d(l2, 0., 0.)),
                         joint_rev_z
      , virtual_body);

    cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
                         SpatialVector(0,0,0,1,0,0), true, 0.1);
    cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
                         SpatialVector(0,0,0,0,1,0), true, 0.1);
    cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
                         SpatialVector(0,0,1,0,0,0), true, 0.1);

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

struct FloatingFourBarLinkage {

  FloatingFourBarLinkage()
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
    , idB0(0)
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
    Vector3d vector3d_zero = Vector3d::Zero();
    Body virtual_body(0., vector3d_zero, vector3d_zero);
    Joint joint_trans(JointTypeTranslationXYZ);
    Joint joint_rev_z(JointTypeRevoluteZ);

    idB0 = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_trans
      , virtual_body);
    idB1 = model.AddBody(idB0, Xtrans(Vector3d(0., 0., 0.)),
                         joint_rev_z, link1);
    idB2 = model.AddBody(idB1, Xtrans(Vector3d(l1, 0., 0.)),
                         joint_rev_z, link2);
    idB3 = model.AddBody(idB0, Xtrans(Vector3d(0., 0., 0.)),
                         joint_rev_z, link1);
    idB4 = model.AddBody(idB3, Xtrans(Vector3d(l1, 0., 0.)),
                         joint_rev_z, link2);
    idB5 = model.AddBody(idB4, Xtrans(Vector3d(l2, 0., 0.)),
                         joint_rev_z
      , virtual_body);

    cs.AddContactConstraint(idB0, Vector3d::Zero(), Vector3d(1.,0.,0.));
    cs.AddContactConstraint(idB0, Vector3d::Zero(), Vector3d(0.,1.,0.));
    cs.AddContactConstraint(idB0, Vector3d::Zero(), Vector3d(0.,0.,1.));
    cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
                         SpatialVector(0.,0.,0.,1.,0.,0.), true, 0.1);
    cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
                         SpatialVector(0.,0.,0.,0.,1.,0.), true, 0.1);
    cs.AddLoopConstraint(idB2, idB5, X_p, X_s,
                         SpatialVector(0.,0.,1.,0.,0.,0.), true, 0.1);

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

  unsigned int idB0;
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

    double slider_mass = 5.;
    double slider_height = 0.1;
    double crank_link1_mass = 3.;
    double crank_link1_length = 1.;
    double crank_link2_mass = 1.;
    double crank_link2_radius = 0.2;
    double crank_link2_length = 3.;
    double crank_link1_height = crank_link2_length - crank_link1_length 
      + slider_height;

    Body slider(slider_mass, Vector3d::Zero(), Vector3d(1., 1., 1.));
    Body crankLink1(crank_link1_mass
      , Vector3d(0.5 * crank_link1_length, 0., 0.)
      , Vector3d(0., 0.
      , crank_link1_mass * crank_link1_length * crank_link1_length / 3.));
    Body crankLink2(crank_link2_mass
      , Vector3d(0.5 * crank_link2_length, 0., 0.)
      , Vector3d(crank_link2_mass * crank_link2_radius * crank_link2_radius/2.
      , crank_link2_mass * (3. * crank_link2_radius * crank_link2_radius 
      + crank_link2_length * crank_link2_length) / 12.
      , crank_link2_mass * (3. * crank_link2_radius * crank_link2_radius 
      + crank_link2_length * crank_link2_length) / 12.));

    Joint joint_rev_z(JointTypeRevoluteZ);
    Joint joint_sphere(JointTypeEulerZYX);
    Joint joint_prs_x(SpatialVector(0.,0.,0.,1.,0.,0.));

    id_p = model.AddBody(0
      , SpatialTransform()
      , joint_prs_x, slider);
    unsigned int id_b1 = model.AddBody(0
      , Xroty(-0.5*M_PI) * Xtrans(Vector3d(0., 0., crank_link1_height)) 
      , joint_rev_z, crankLink1);
    id_s = model.AddBody(id_b1
      , Xroty(M_PI) * Xtrans(Vector3d(crank_link1_length, 0., 0.))
      , joint_sphere, crankLink2);

    X_p = Xtrans(Vector3d(0., 0., slider_height));
    X_s = SpatialTransform(roty(-0.5 * M_PI),
                           Vector3d(crank_link2_length, 0, 0));

    cs.AddLoopConstraint(id_p, id_s, X_p, X_s,
                         SpatialVector(0,0,0,1,0,0), true, 0.1);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s,
                         SpatialVector(0,0,0,0,1,0), true, 0.1);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s,
                         SpatialVector(0,0,0,0,0,1), true, 0.1);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s,
                         SpatialVector(0,0,1,0,0,0), true, 0.1);

    cs.Bind(model);

    q = VectorNd::Zero(model.dof_count);
    qd = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);

    Matrix3d rot_ps 
      = (CalcBodyWorldOrientation(model,q,id_p).transpose()*X_p.E).transpose()
      *  CalcBodyWorldOrientation(model,q,id_s).transpose()*X_s.E;
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

struct SliderCrank3DSphericalJoint {

  SliderCrank3DSphericalJoint()
    : model()
    , cs()
    , q()
    , qd()
    , id_p(0)
    , id_s(0)
    , X_p()
    , X_s() {

    double slider_mass = 5.;
    double slider_height = 0.1;
    double crank_link1_mass = 3.;
    double crank_link1_length = 1.;
    double crank_link2_mass = 1.;
    double crank_link2_radius = 0.2;
    double crank_link2_length = 3.;
    double crank_link1_height = crank_link2_length - crank_link1_length 
      + slider_height;

    Body slider(slider_mass, Vector3d::Zero(), Vector3d(1., 1., 1.));
    Body crankLink1(crank_link1_mass
      , Vector3d(0.5 * crank_link1_length, 0., 0.)
      , Vector3d(0., 0.
      , crank_link1_mass * crank_link1_length * crank_link1_length / 3.));
    Body crankLink2(crank_link2_mass
      , Vector3d(0.5 * crank_link2_length, 0., 0.)
      , Vector3d(crank_link2_mass * crank_link2_radius * crank_link2_radius/2.
      , crank_link2_mass * (3. * crank_link2_radius * crank_link2_radius 
      + crank_link2_length * crank_link2_length) / 12.
      , crank_link2_mass * (3. * crank_link2_radius * crank_link2_radius 
      + crank_link2_length * crank_link2_length) / 12.));

    Joint joint_rev_z(JointTypeRevoluteZ);
    Joint joint_sphere(JointTypeSpherical);
    Joint joint_prs_x(SpatialVector(0.,0.,0.,1.,0.,0.));

    id_p = model.AddBody(0
      , SpatialTransform()
      , joint_prs_x, slider);
    unsigned int id_b1 = model.AddBody(0
      , Xroty(-0.5*M_PI) * Xtrans(Vector3d(0., 0., crank_link1_height)) 
      , joint_rev_z, crankLink1);
    id_s = model.AddBody(id_b1
      , Xroty(M_PI) * Xtrans(Vector3d(crank_link1_length, 0., 0.))
      , joint_sphere, crankLink2);

    X_p = Xtrans(Vector3d(0., 0., slider_height));
    X_s = SpatialTransform(roty(-0.5 * M_PI),Vector3d(crank_link2_length,0,0));

    cs.AddLoopConstraint(id_p, id_s, X_p, X_s, 
        SpatialVector(0,0,0,1,0,0), true, 0.1);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s, 
        SpatialVector(0,0,0,0,1,0), true, 0.1);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s, 
        SpatialVector(0,0,0,0,0,1), true, 0.1);
    cs.AddLoopConstraint(id_p, id_s, X_p, X_s, 
        SpatialVector(0,0,1,0,0,0), true, 0.1);

    cs.Bind(model);

    q = VectorNd::Zero(model.q_size);
    qd = VectorNd::Zero(model.dof_count);
    qdd = VectorNd::Zero(model.dof_count);
    tau = VectorNd::Zero(model.dof_count);

    Matrix3d rot_ps 
      = (CalcBodyWorldOrientation(model,q,id_p).transpose()*X_p.E).transpose()
       * CalcBodyWorldOrientation(model,q,id_s).transpose()*X_s.E;
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

TEST_FIXTURE(FourBarLinkage, TestFourBarLinkageConstraintErrors) {
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
  angleErr = sin(-0.5 * M_PI);

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
  angleErr = sin(-q[0] - q[1] + q[2] + q[3] + q[4]);

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
  angleErr = sin(-q[0] - q[1] + q[2] + q[3] + q[4]);

  pos1 = CalcBodyToBaseCoordinates(model, q, idB2, X_p.r);
  pos2 = CalcBodyToBaseCoordinates(model, q, idB5, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, idB2).transpose() * X_p.E;
  posErr = rot_p.transpose() * (pos2 - pos1);

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_CLOSE(posErr[0], err[0], TEST_PREC);
  CHECK_CLOSE(posErr[1], err[1], TEST_PREC);
  CHECK_CLOSE(angleErr, err[2], TEST_PREC);
}

TEST_FIXTURE(FourBarLinkage, TestFourBarLinkageConstraintJacobian) {
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

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

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

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

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

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
}

TEST_FIXTURE(FourBarLinkage, TestFourBarLinkageConstraintsVelocityErrors) {
  VectorNd errd(VectorNd::Zero(cs.size()));
  VectorNd errdRef(VectorNd::Zero(cs.size()));
  MatrixNd G(cs.size(), model.dof_count);

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

  CHECK_ARRAY_CLOSE(errdRef, errd, cs.size(), TEST_PREC);

  // Invalid velocities.
  qd[0] = -1.;
  qd[1] = -1.;
  qd[2] = 0.;
  qd[3] = 0.;
  qd[4] = 0.;

  CalcConstraintsVelocityError(model, q, qd, cs, errd);
  CalcConstraintsJacobian(model, q, cs, G);
  errdRef = G * qd;
  CHECK_ARRAY_CLOSE(errdRef, errd, cs.size(), TEST_PREC);
}

TEST_FIXTURE(FourBarLinkage, TestFourBarLinkageQAssembly) {
  VectorNd weights(q.size());
  VectorNd err(cs.size());
  VectorNd errRef(VectorNd::Zero(cs.size()));

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
  
  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-12);
  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(inRange(q[0] + q[1]), inRange(q[2] + q[3] + q[4]), TEST_PREC);
  CHECK_CLOSE(qInit[0], q[0], TEST_PREC);
  CHECK_CLOSE(qInit[2], q[2], TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Perturbed initial guess.
  qInit[0] = qRef[0];
  qInit[1] = qRef[1];
  qInit[2] = qRef[2];
  qInit[3] = qRef[3];
  qInit[4] = qRef[4] + 0.05;

  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-12);
  CalcConstraintsPositionError(model, q, cs, err);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(inRange(q[0] + q[1]), inRange(q[2] + q[3] + q[4]), TEST_PREC);
  CHECK_CLOSE(qInit[0], q[0], TEST_PREC);
  CHECK_CLOSE(qInit[2], q[2], TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Perturbed initial guess.
  qInit[0] = qRef[0] - 0.2;
  qInit[1] = qRef[1] - 0.;
  qInit[2] = qRef[2] + 0.1;
  qInit[3] = qRef[3] - 0.03;
  qInit[4] = qRef[4] + 0.05;

  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-12);
  CalcConstraintsPositionError(model, q, cs, err);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(inRange(q[0] + q[1]), inRange(q[2] + q[3] + q[4]), TEST_PREC);
  CHECK_CLOSE(qInit[0], q[0], TEST_PREC);
  CHECK_CLOSE(qInit[2], q[2], TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Perturbed initial guess.
  qInit[0] = qRef[0] + 0.01;
  qInit[1] = qRef[1] + 0.02;
  qInit[2] = qRef[2] - 0.03;
  qInit[3] = qRef[3] - 0.02;
  qInit[4] = qRef[4] + 0.01;

  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-12);
  CalcConstraintsPositionError(model, q, cs, err);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(inRange(q[0] + q[1]), inRange(q[2] + q[3] + q[4]), TEST_PREC);
  CHECK_CLOSE(qInit[0], q[0], TEST_PREC);
  CHECK_CLOSE(qInit[2], q[2], TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
}

TEST_FIXTURE(FourBarLinkage, TestFourBarLinkageQDotAssembly) {
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

  CHECK_ARRAY_CLOSE(CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    , CalcPointVelocity6D(model, q, qd, idB5, X_s.r), 6, TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
  CHECK_CLOSE(qdInit[0], qd[0], TEST_PREC);
  CHECK_CLOSE(qdInit[2], qd[2], TEST_PREC);
}

TEST_FIXTURE(FourBarLinkage, TestFourBarLinkageForwardDynamics) {
  VectorNd qddDirect;
  VectorNd qddNullSpace;

  cs.SetSolver(LinearSolverColPivHouseholderQR);

#ifndef RBDL_USE_SIMPLE_MATH
  // SimpleMath has no solver that can solve the system in this configuration.
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
  ForwardDynamicsConstraintsDirect(model, q, qd, tau, cs, qddDirect);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r)
    , CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r)
    , 6, TEST_PREC);

  qddNullSpace = VectorNd::Zero(q.size());
  ForwardDynamicsConstraintsNullSpace(model, q, qd, tau, cs, qddNullSpace);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB2, X_p.r)
    , CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB5, X_s.r)
    , 6, TEST_PREC);
#endif

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
  ForwardDynamicsConstraintsDirect(model, q, qd, tau, cs, qddDirect);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r)
    , CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r)
    , 6, TEST_PREC);

  qddNullSpace = VectorNd::Zero(q.size());
  ForwardDynamicsConstraintsNullSpace(model, q, qd, tau, cs, qddNullSpace);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB2, X_p.r)
    , CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB5, X_s.r)
    , 6, TEST_PREC);

  // Note:
  // The Range Space Sparse method can't be used because the H matrix has a 0
  // on the diagonal and the LTL factorization tries to divide by 0.

  // Note:
  // LinearSolverPartialPivLU does not work because the A matrix in the dynamic
  // system is not invertible.

  // Note:
  // LinearSolverHouseholderQR sometimes does not work well when the system is
  // in a singular configuration.
}

TEST_FIXTURE(FourBarLinkage, FourBarLinkageImpulse) {
  VectorNd qdPlusDirect(qd.size());
  VectorNd qdPlusRangeSpaceSparse(qd.size());
  VectorNd qdPlusNullSpace(qd.size());
  VectorNd errd(cs.size());

  q[0] = M_PI * 3 / 4;
  q[1] = -0.5 * M_PI;
  q[2] = M_PI - q[0];
  q[3] = -q[1];
  q[4] = q[0] + q[1] - q[2] - q[3];
  assert(q[0] + q[1] - q[2] - q[3] - q[4] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  cs.v_plus[0] = 1.;
  cs.v_plus[1] = 2.;
  cs.v_plus[2] = 3.;

  ComputeConstraintImpulsesDirect(model, q, qd, cs, qdPlusDirect);
  CalcConstraintsVelocityError(model, q, qdPlusDirect, cs, errd);

  CHECK_ARRAY_CLOSE(cs.v_plus, errd, cs.size(), TEST_PREC);

  cs.v_plus[0] = 0.;
  cs.v_plus[1] = 0.;
  cs.v_plus[2] = 0.;

  qd[0] = 1.;
  qd[1] = 2.;
  qd[2] = 3.;

  ComputeConstraintImpulsesDirect(model, q, qd, cs, qdPlusDirect);
  CalcConstraintsVelocityError(model, q, qdPlusDirect, cs, errd);

  CHECK_ARRAY_CLOSE(cs.v_plus, errd, cs.size(), TEST_PREC);
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
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

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
  rot_ps = rot_p.transpose() * rot_s;
  rotationVec = - 0.5 * Vector3d
    ( rot_ps(1,2) - rot_ps(2,1)
    , rot_ps(2,0) - rot_ps(0,2)
    , rot_ps(0,1) - rot_ps(1,0));
  errRef.block<3,1>(0,0) = pos_s - pos_p;
  errRef[3] = rotationVec[2];

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
}

TEST_FIXTURE(SliderCrank3D, TestSliderCrank3DConstraintJacobian) {
  MatrixNd G(MatrixNd::Zero(cs.size(), q.size()));

  // Test in zero position.

  G.setZero();
  CalcConstraintsJacobian(model, q, cs, G);

  VectorNd errRef(VectorNd::Zero(cs.size()));
  VectorNd err = G * qd;

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
}

TEST_FIXTURE(SliderCrank3D, TestSliderCrank3DConstraintsVelocityErrors) {
  VectorNd errd(VectorNd::Zero(cs.size()));
  VectorNd errdRef(VectorNd::Zero(cs.size()));
  MatrixNd G(cs.size(), model.dof_count);
  VectorNd qWeights(q.size());
  VectorNd qInit(q.size());
  bool success;

  // Compute assembled configuration.
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

  success = CalcAssemblyQ(model, qInit, cs, q, qWeights, TEST_PREC);
  assert(success);

  // Some random velocity.
  qd[0] = -0.2;
  qd[1] = 0.1 * M_PI;
  qd[2] = -0.1 * M_PI;
  qd[3] = 0.;
  qd[4] = 0.1 * M_PI;

  CalcConstraintsVelocityError(model, q, qd, cs, errd);
  CalcConstraintsJacobian(model, q, cs, G);
  errdRef = G * qd;

  CHECK_ARRAY_CLOSE(errdRef, errd, cs.size(), TEST_PREC);
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
  rot_ps = rot_p.transpose() * rot_s;

  CHECK(success);
  CHECK_ARRAY_CLOSE(pos_p, pos_s, 3, TEST_PREC);
  CHECK_CLOSE(0., rot_ps(0,1) - rot_ps(1,0), TEST_PREC);
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

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(vel_p[i], vel_s[i], TEST_PREC);
  }
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

#ifndef RBDL_USE_SIMPLE_MATH
  // The SimpleMath solver cannot solve the system close to a singular
  // configuration.
  // Test with zero q and qdot.

  tau[0] = 0.12;
  tau[1] = -0.3;
  tau[2] = 0.05;
  tau[3] = 0.7;
  tau[4] = -0.1;

  ForwardDynamicsConstraintsDirect(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsNullSpace(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsRangeSpaceSparse(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }
#endif

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
    = (CalcBodyWorldOrientation(model, q, id_p).transpose()*X_p.E).transpose()
    * CalcBodyWorldOrientation(model, q, id_s).transpose() * X_s.E;
  assert((CalcBodyToBaseCoordinates(model, q, id_p, X_p.r)
    - CalcBodyToBaseCoordinates(model, q, id_p, X_p.r)).norm() < TEST_PREC);
  assert(rot_ps(0,1) - rot_ps(0,1) < TEST_PREC);
  assert((CalcPointVelocity6D(model, q, qd, id_p, X_p.r)
    -CalcPointVelocity6D(model, q, qd, id_p, X_p.r)).norm() < TEST_PREC);

  // Test with non-zero q and qdot.

  ForwardDynamicsConstraintsDirect(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsNullSpace(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsRangeSpaceSparse(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }
}

TEST_FIXTURE(SliderCrank3D, TestSliderCrank3DImpulse) {
  VectorNd qdPlusDirect(qd.size());
  VectorNd qdPlusRangeSpaceSparse(qd.size());
  VectorNd qdPlusNullSpace(qd.size());
  VectorNd errdDirect(cs.size());
  VectorNd errdSpaceSparse(cs.size());
  VectorNd errdNullSpace(cs.size());
  
  VectorNd qWeights(q.size());
  qWeights[0] = 1.;
  qWeights[1] = 1.;
  qWeights[2] = 1.;
  qWeights[3] = 1.;
  qWeights[4] = 1.;

  VectorNd qInit(q.size());
  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  qInit[2] = -0.25 * M_PI;
  qInit[3] = 0.1;
  qInit[4] = 0.1;

  bool success = CalcAssemblyQ(model, qInit, cs, q, qWeights, TEST_PREC);
  assert(success);

  cs.v_plus[0] = 1.;
  cs.v_plus[1] = 2.;
  cs.v_plus[2] = 3.;
  cs.v_plus[3] = 4.;
  
  ComputeConstraintImpulsesDirect(model, q, qd, cs, qdPlusDirect);
  CalcConstraintsVelocityError(model, q, qdPlusDirect, cs, errdDirect);
  
  CHECK_ARRAY_CLOSE(cs.v_plus, errdDirect, cs.size(), TEST_PREC);

  ComputeConstraintImpulsesRangeSpaceSparse(model, q, qd, cs
    , qdPlusRangeSpaceSparse);
  CalcConstraintsVelocityError(model, q, qdPlusRangeSpaceSparse, cs
    , errdSpaceSparse);
  
  CHECK_ARRAY_CLOSE(cs.v_plus, errdSpaceSparse, cs.size(), TEST_PREC);

  ComputeConstraintImpulsesNullSpace(model, q, qd, cs, qdPlusNullSpace);
  CalcConstraintsVelocityError(model, q, qdPlusNullSpace, cs, errdNullSpace);

  CHECK_ARRAY_CLOSE(cs.v_plus, errdNullSpace, cs.size(), TEST_PREC);

  cs.v_plus[0] = 0.;
  cs.v_plus[1] = 0.;
  cs.v_plus[2] = 0.;
  cs.v_plus[3] = 0.;

  qd[0] = 1.;
  qd[1] = 2.;
  qd[2] = 3.;

  ComputeConstraintImpulsesDirect(model, q, qd, cs, qdPlusDirect);
  CalcConstraintsVelocityError(model, q, qdPlusDirect, cs, errdDirect);
  
  CHECK_ARRAY_CLOSE(cs.v_plus, errdDirect, cs.size(), TEST_PREC);

  ComputeConstraintImpulsesRangeSpaceSparse(model, q, qd, cs
    , qdPlusRangeSpaceSparse);
  CalcConstraintsVelocityError(model, q, qdPlusRangeSpaceSparse, cs
    , errdSpaceSparse);
  
  CHECK_ARRAY_CLOSE(cs.v_plus, errdSpaceSparse, cs.size(), TEST_PREC);

  ComputeConstraintImpulsesNullSpace(model, q, qd, cs, qdPlusNullSpace);
  CalcConstraintsVelocityError(model, q, qdPlusNullSpace, cs, errdNullSpace);

  CHECK_ARRAY_CLOSE(cs.v_plus, errdNullSpace, cs.size(), TEST_PREC);
}

TEST_FIXTURE(FloatingFourBarLinkage
  , TestFloatingFourBarLinkageConstraintErrors) {

  VectorNd err = VectorNd::Zero(cs.size());
  Vector3d pos0;
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
  q[5] = 0.;
  q[6] = 0.;
  q[7] = 0.;

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_CLOSE(0., err[0], TEST_PREC);
  CHECK_CLOSE(0., err[1], TEST_PREC);
  CHECK_CLOSE(0., err[2], TEST_PREC);
  CHECK_CLOSE(0., err[3], TEST_PREC);
  CHECK_CLOSE(0., err[4], TEST_PREC);
  CHECK_CLOSE(0., err[5], TEST_PREC);

  // Test in non-zero position.
  q[0] = 1.;
  q[1] = 2.;
  q[2] = 3.;
  q[3] = M_PI * 3 / 4;
  q[4] = -M_PI;
  q[5] = M_PI - q[3];
  q[6] = -q[4];
  q[7] = 0.;
  angleErr = sin(-0.5 * M_PI);

  pos0 = CalcBodyToBaseCoordinates(model, q, idB0, Vector3d::Zero());
  pos1 = CalcBodyToBaseCoordinates(model, q, idB2, X_p.r);
  pos2 = CalcBodyToBaseCoordinates(model, q, idB5, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, idB2).transpose() * X_p.E;
  posErr = rot_p.transpose() * (pos2 - pos1);

  assert(std::fabs(posErr[1]) < TEST_PREC);
  assert(std::fabs(posErr[2]) < TEST_PREC);

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_ARRAY_CLOSE(Vector3d(1.,2.,3.), pos0, 3, TEST_PREC);
  CHECK_CLOSE(posErr[0], err[3], TEST_PREC);
  CHECK_CLOSE(0., err[4], TEST_PREC);
  CHECK_CLOSE(angleErr, err[5], TEST_PREC);

  // Test in non-zero position.
  q[0] = 1.;
  q[1] = 2.;
  q[2] = 3.;
  q[3] = 0.;
  q[4] = 0.;
  q[5] = M_PI + 0.1;
  q[6] = 0.;
  q[7] = 0.;
  angleErr = sin(-q[3] - q[4] + q[5] + q[6] + q[7]);

  pos0 = CalcBodyToBaseCoordinates(model, q, idB0, Vector3d::Zero());
  pos1 = CalcBodyToBaseCoordinates(model, q, idB2, X_p.r);
  pos2 = CalcBodyToBaseCoordinates(model, q, idB5, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, idB2).transpose() * X_p.E;
  posErr = rot_p.transpose() * (pos2 - pos1);

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_ARRAY_CLOSE(Vector3d(1.,2.,3.), pos0, 3, TEST_PREC);
  CHECK_CLOSE(posErr[0], err[3], TEST_PREC);
  CHECK_CLOSE(posErr[1], err[4], TEST_PREC);
  CHECK_CLOSE(angleErr, err[5], TEST_PREC);

  // Test in non-zero position.
  q[0] = 1.;
  q[1] = 2.;
  q[2] = 3.;
  q[3] = 0.8;
  q[4] = -0.4;
  q[5] = M_PI - q[3];
  q[6] = -q[4];
  q[7] = 0.;
  angleErr = sin(-q[3] - q[4] + q[5] + q[6] + q[7]);

  pos0 = CalcBodyToBaseCoordinates(model, q, idB0, Vector3d::Zero());
  pos1 = CalcBodyToBaseCoordinates(model, q, idB2, X_p.r);
  pos2 = CalcBodyToBaseCoordinates(model, q, idB5, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, idB2).transpose() * X_p.E;
  posErr = rot_p.transpose() * (pos2 - pos1);

  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_ARRAY_CLOSE(Vector3d(1.,2.,3.), pos0, 3, TEST_PREC);
  CHECK_CLOSE(posErr[0], err[3], TEST_PREC);
  CHECK_CLOSE(posErr[1], err[4], TEST_PREC);
  CHECK_CLOSE(angleErr, err[5], TEST_PREC);
}

TEST_FIXTURE(FloatingFourBarLinkage
  , TestFloatingFourBarLinkageConstraintJacobian) {

  MatrixNd G(MatrixNd::Zero(cs.size(), q.size()));
  VectorNd err(VectorNd::Zero(cs.size()));
  VectorNd errRef(VectorNd::Zero(cs.size()));

  // Zero Q configuration, both arms of the 4-bar laying on the x-axis
  q[0] = 0.;
  q[1] = 0.;
  q[2] = 0.;
  q[3] = 0.;
  q[4] = 0.;
  q[5] = 0.;
  q[6] = 0.;
  q[7] = 0.;
  assert(q[3] + q[4] - q[5] - q[6] - q[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = 0.;
  qd[1] = 0.;
  qd[2] = 0.;
  qd[3] = -1.;
  qd[4] = -1.;
  qd[5] = -1.;
  qd[6] = -1.;
  qd[7] = 0.;
  assert((CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity6D(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  CalcConstraintsJacobian(model, q, cs, G);

  err = G * qd;

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Both arms of the 4-bar laying on the y-axis
  q[0] = 0.;
  q[1] = 0.;
  q[2] = 0.;
  q[3] = 0.5 * M_PI;
  q[4] = 0.;
  q[5] = 0.5 * M_PI;
  q[6] = 0.;
  q[7] = 0.;
  assert(q[3] + q[4] - q[5] - q[6] - q[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = 0.;
  qd[1] = 0.;
  qd[2] = 0.;
  qd[3] = -1.;
  qd[4] = -1.;
  qd[5] = -1.;
  qd[6] = -1.;
  qd[7] = 0.;
  assert((CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity6D(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  CalcConstraintsJacobian(model, q, cs, G);

  err = G * qd;

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Arms symmetric wrt y axis.
  q[0] = 1.;
  q[1] = 2.;
  q[2] = 3.;
  q[3] = M_PI * 3 / 4;
  q[4] = -0.5 * M_PI;
  q[5] = M_PI - q[3];
  q[6] = -q[4];
  q[7] = q[3] + q[4] - q[5] - q[6];
  assert(q[3] + q[4] - q[5] - q[6] - q[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = 0.;
  qd[1] = 0.;
  qd[2] = 0.;
  qd[3] = -1.;
  qd[4] = -1.;
  qd[5] = -2.;
  qd[6] = +1.;
  qd[7] = -1.;
  assert((CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity6D(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  CalcConstraintsJacobian(model, q, cs, G);

  err = G * qd;

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
}

TEST_FIXTURE(FloatingFourBarLinkage
  , TestFloatingFourBarLinkageConstraintsVelocityErrors) {

  VectorNd errd(VectorNd::Zero(cs.size()));
  VectorNd errdRef(VectorNd::Zero(cs.size()));
  MatrixNd G(cs.size(), model.dof_count);

  // Arms symmetric wrt y axis.
  q[0] = 1.;
  q[1] = 2.;
  q[2] = 3.;
  q[3] = M_PI * 3 / 4;
  q[4] = -0.5 * M_PI;
  q[5] = M_PI - q[3];
  q[6] = -q[4];
  q[7] = q[3] + q[4] - q[5] - q[6];

  assert(q[3] + q[4] - q[5] - q[6] - q[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = 0.;
  qd[1] = 0.;
  qd[2] = 0.;
  qd[3] = -1.;
  qd[4] = -1.;
  qd[5] = -2.;
  qd[6] = +1.;
  qd[7] = -1.;

  CalcConstraintsVelocityError(model, q, qd, cs, errd);
  CHECK_ARRAY_CLOSE(errdRef, errd, cs.size(), TEST_PREC);

  // Invalid velocities.
  qd[0] = -1.;
  qd[1] = -1.;
  qd[2] = 0.;
  qd[3] = -1.;
  qd[4] = -1.;
  qd[5] = 0.;
  qd[6] = 0.;
  qd[7] = 0.;

  CalcConstraintsVelocityError(model, q, qd, cs, errd);
  CalcConstraintsJacobian(model, q, cs, G);
  errdRef = G * qd;
  CHECK_ARRAY_CLOSE(errdRef, errd, cs.size(), TEST_PREC);
}

TEST_FIXTURE(FloatingFourBarLinkage, TestFloatingFourBarLinkageQAssembly) {
  VectorNd weights(q.size());
  VectorNd err(cs.size());
  VectorNd errRef(VectorNd::Zero(cs.size()));

  weights[0] = 0.;
  weights[1] = 0.;
  weights[2] = 0.;
  weights[3] = 1.;
  weights[4] = 0.;
  weights[5] = 1.;
  weights[6] = 0.;
  weights[7] = 0.;

  VectorNd qRef = VectorNd::Zero(q.size());
  qRef[0] = 1.;
  qRef[1] = 2.;
  qRef[2] = 3.;
  qRef[3] = M_PI * 3 / 4;
  qRef[4] = -0.5 * M_PI;
  qRef[5] = M_PI - qRef[3];
  qRef[6] = -qRef[4];
  qRef[7] = qRef[3] + qRef[4] - qRef[5] - qRef[6];

  assert(qRef[3] + qRef[4] - qRef[5] - qRef[6] - qRef[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, qRef, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, qRef, idB5, X_s.r)).norm() < TEST_PREC);

  bool success;  

  // Feasible initial guess.
  VectorNd qInit = VectorNd::Zero(q.size());
  qInit = qRef;
  
  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-12);
  CalcConstraintsPositionError(model, q, cs, err);

  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(inRange(q[3] + q[4]), inRange(q[5] + q[6] + q[7]), TEST_PREC);
  CHECK_CLOSE(qInit[3], q[3], TEST_PREC);
  CHECK_CLOSE(qInit[5], q[5], TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Perturbed initial guess.
  qInit[3] = qRef[3];
  qInit[4] = qRef[4];
  qInit[5] = qRef[5];
  qInit[6] = qRef[6];
  qInit[7] = qRef[7] + 0.05;

  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-12);
  CalcConstraintsPositionError(model, q, cs, err);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(inRange(q[3] + q[4]), inRange(q[5] + q[6] + q[7]), TEST_PREC);
  CHECK_CLOSE(qInit[3], q[3], TEST_PREC);
  CHECK_CLOSE(qInit[5], q[5], TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Perturbed initial guess.
  qInit[3] = qRef[3] - 0.2;
  qInit[4] = qRef[4] - 0.;
  qInit[5] = qRef[5] + 0.1;
  qInit[6] = qRef[6] - 0.03;
  qInit[7] = qRef[7] + 0.05;

  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-12);
  CalcConstraintsPositionError(model, q, cs, err);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(inRange(q[3] + q[4]), inRange(q[5] + q[6] + q[7]), TEST_PREC);
  CHECK_CLOSE(qInit[3], q[3], TEST_PREC);
  CHECK_CLOSE(qInit[5], q[5], TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Perturbed initial guess.
  qInit[3] = qRef[3] + 0.01;
  qInit[4] = qRef[4] + 0.02;
  qInit[5] = qRef[5] - 0.03;
  qInit[6] = qRef[6] - 0.02;
  qInit[7] = qRef[7] + 0.01;

  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1.e-12);
  CalcConstraintsPositionError(model, q, cs, err);

  CHECK(success);
  CHECK_ARRAY_CLOSE(CalcBodyToBaseCoordinates(model, q, idB2, X_p.r)
    , CalcBodyToBaseCoordinates(model, q, idB5, X_s.r), 3, TEST_PREC);
  CHECK_CLOSE(inRange(q[3] + q[4]), inRange(q[5] + q[6] + q[7]), TEST_PREC);
  CHECK_CLOSE(qInit[3], q[3], TEST_PREC);
  CHECK_CLOSE(qInit[5], q[5], TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
}

TEST_FIXTURE(FloatingFourBarLinkage, TestFloatingFourBarLinkageQDotAssembly) {
  VectorNd weights(q.size());

  weights[0] = 0.;
  weights[1] = 0.;
  weights[2] = 0.;
  weights[3] = 1.;
  weights[4] = 0.;
  weights[5] = 1.;
  weights[6] = 0.;
  weights[7] = 0.;

  q[0] = 1.;
  q[1] = 2.;
  q[2] = 3.;
  q[3] = M_PI * 3 / 4;
  q[4] = -0.5 * M_PI;
  q[5] = M_PI - q[3];
  q[6] = -q[4];
  q[7] = q[3] + q[4] - q[5] - q[6];

  assert(q[3] + q[4] - q[5] - q[6] - q[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  VectorNd qdInit = VectorNd::Zero(q.size());
  qdInit[0] = 1.;
  qdInit[1] = 2.;
  qdInit[2] = 3.;
  qdInit[3] = 0.01;
  qdInit[4] = 0.5;
  qdInit[5] = -0.7;
  qdInit[6] = -0.5;
  qdInit[7] = 0.3;

  CalcAssemblyQDot(model, q, qdInit, cs, qd, weights);
  MatrixNd G(MatrixNd::Zero(cs.size(), q.size()));
  VectorNd err(VectorNd::Zero(cs.size()));
  VectorNd errRef(VectorNd::Zero(cs.size()));
  CalcConstraintsJacobian(model, q, cs, G);
  err = G * qd;

  CHECK_ARRAY_CLOSE(CalcPointVelocity6D(model, q, qd, idB2, X_p.r)
    , CalcPointVelocity6D(model, q, qd, idB5, X_s.r), 6, TEST_PREC);
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
  CHECK_CLOSE(qdInit[3], qd[3], TEST_PREC);
  CHECK_CLOSE(qdInit[5], qd[5], TEST_PREC);
}

TEST_FIXTURE(FloatingFourBarLinkage
  , TestFloatingFourBarLinkageForwardDynamics) {

  VectorNd qddDirect;
  VectorNd qddNullSpace;

  cs.SetSolver(LinearSolverColPivHouseholderQR);

#ifndef RBDL_USE_SIMPLE_MATH
  // The SimpleMath solver cannot solve the system close to a singular
  // configuration.
  // Configuration 1.
  q[0] = 0.;
  q[1] = 0.;
  q[2] = 0.;
  q[3] = 0.;
  q[4] = 0.;
  q[5] = 0.;
  q[6] = 0.;
  q[7] = 0.;
  assert(q[3] + q[4] - q[5] - q[6] - q[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = 0.;
  qd[1] = 0.;
  qd[2] = 0.;
  qd[3] = 0.;
  qd[4] = 0.;
  qd[5] = 0.;
  qd[6] = 0.;
  qd[7] = 0.;
  assert(qd[3] + qd[4] - qd[5] - qd[6] - qd[7] == 0.);
  assert((CalcPointVelocity(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  tau[0] = 0.;
  tau[1] = 0.;
  tau[2] = 0.;
  tau[3] = 1.;
  tau[4] = -2.;
  tau[5] = 3.;
  tau[6] = -5.;
  tau[7] = 7.;

  qddDirect = VectorNd::Zero(q.size());
  ForwardDynamicsConstraintsDirect(model, q, qd, tau, cs, qddDirect);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r)
    , CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r)
    , 6, TEST_PREC);

  qddNullSpace = VectorNd::Zero(q.size());
  ForwardDynamicsConstraintsNullSpace(model, q, qd, tau, cs, qddNullSpace);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB2, X_p.r)
    , CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB5, X_s.r)
    , 6, TEST_PREC);
#endif

  // Configuration 2.
  q[0] = 1.;
  q[1] = 2.;
  q[2] = 3.;
  q[3] = M_PI * 3 / 4;
  q[4] = -0.5 * M_PI;
  q[5] = M_PI - q[3];
  q[6] = -q[4];
  q[7] = q[3] + q[4] - q[5] - q[6];

  assert(q[3] + q[4] - q[5] - q[6] - q[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  qd[0] = 0.;
  qd[1] = 0.;
  qd[2] = 0.;
  qd[3] = -1.;
  qd[4] = -1.;
  qd[5] = -2.;
  qd[6] = +1.;
  qd[7] = -1.;
  assert(qd[3] + qd[4] - qd[5] - qd[6] - qd[7] == 0.);
  assert((CalcPointVelocity(model, q, qd, idB2, X_p.r)
    - CalcPointVelocity(model, q, qd, idB5, X_s.r)).norm() < TEST_PREC);

  tau[0] = 0.;
  tau[1] = 0.;
  tau[2] = 0.;
  tau[3] = 1.;
  tau[4] = -2.;
  tau[5] = 3.;
  tau[6] = -5.;
  tau[7] = 7.;

  qddDirect = VectorNd::Zero(q.size());
  ForwardDynamicsConstraintsDirect(model, q, qd, tau, cs, qddDirect);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddDirect, idB2, X_p.r)
    , CalcPointAcceleration6D(model, q, qd, qddDirect, idB5, X_s.r)
    , 6, TEST_PREC);

  qddNullSpace = VectorNd::Zero(q.size());
  ForwardDynamicsConstraintsNullSpace(model, q, qd, tau, cs, qddNullSpace);

  CHECK_ARRAY_CLOSE
    (CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB2, X_p.r)
    , CalcPointAcceleration6D(model, q, qd, qddNullSpace, idB5, X_s.r)
    , 6, TEST_PREC);
}

TEST_FIXTURE(FloatingFourBarLinkage, TestFloatingFourBarLinkageImpulse) {
  VectorNd qdPlusDirect(qd.size());
  VectorNd qdPlusRangeSpaceSparse(qd.size());
  VectorNd qdPlusNullSpace(qd.size());
  VectorNd errd(cs.size());

  q[0] = 1.;
  q[1] = 2.;
  q[2] = 3.;
  q[3] = M_PI * 3 / 4;
  q[4] = -0.5 * M_PI;
  q[5] = M_PI - q[3];
  q[6] = -q[4];
  q[7] = q[3] + q[4] - q[5] - q[6];

  assert(q[3] + q[4] - q[5] - q[6] - q[7] == 0.);
  assert((CalcBodyToBaseCoordinates(model, q, idB2, X_p.r) 
    - CalcBodyToBaseCoordinates(model, q, idB5, X_s.r)).norm() < TEST_PREC);

  cs.v_plus[0] = 1.;
  cs.v_plus[1] = 2.;
  cs.v_plus[2] = 3.;
  cs.v_plus[3] = 4.;
  cs.v_plus[4] = 5.;
  cs.v_plus[5] = 6.;

  ComputeConstraintImpulsesDirect(model, q, qd, cs, qdPlusDirect);
  CalcConstraintsVelocityError(model, q, qdPlusDirect, cs, errd);

  CHECK_ARRAY_CLOSE(cs.v_plus, errd, cs.size(), TEST_PREC);

  cs.v_plus[0] = 0.;
  cs.v_plus[1] = 0.;
  cs.v_plus[2] = 0.;
  cs.v_plus[3] = 0.;
  cs.v_plus[4] = 0.;
  cs.v_plus[5] = 0.;

  qd[0] = 1.;
  qd[2] = 2.;
  qd[4] = 3.;

  ComputeConstraintImpulsesDirect(model, q, qd, cs, qdPlusDirect);
  CalcConstraintsVelocityError(model, q, qdPlusDirect, cs, errd);

  CHECK_ARRAY_CLOSE(cs.v_plus, errd, cs.size(), TEST_PREC);
}

TEST_FIXTURE(SliderCrank3DSphericalJoint
  , TestSliderCrank3DSphericalJointConstraintErrors) {
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
  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);

  // Test in another configuration.
  Quaternion quat = Quaternion::fromZYXAngles(Vector3d(-0.25*M_PI,0.01,0.01));
  q[0] = 0.4;
  q[1] = 0.25 * M_PI;
  model.SetQuaternion(id_s, quat, q);

  CalcConstraintsPositionError(model, q, cs, err);

  pos_p = CalcBodyToBaseCoordinates(model, q, id_p, X_p.r);
  pos_s = CalcBodyToBaseCoordinates(model, q, id_s, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, id_p).transpose() * X_p.E;
  rot_s = CalcBodyWorldOrientation(model, q, id_s).transpose() * X_s.E;
  rot_ps = rot_p.transpose() * rot_s;
  rotationVec = - 0.5 * Vector3d
    ( rot_ps(1,2) - rot_ps(2,1)
    , rot_ps(2,0) - rot_ps(0,2)
    , rot_ps(0,1) - rot_ps(1,0));
  errRef.block<3,1>(0,0) = pos_s - pos_p;
  errRef[3] = rotationVec[2];

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
}

TEST_FIXTURE(SliderCrank3DSphericalJoint
  , TestSliderCrank3DSphericalJointConstraintJacobian) {
  MatrixNd G(MatrixNd::Zero(cs.size(), model.dof_count));

  // Test in zero position.

  G.setZero();
  CalcConstraintsJacobian(model, q, cs, G);

  VectorNd errRef(VectorNd::Zero(cs.size()));
  VectorNd err = G * qd;

  CHECK_ARRAY_CLOSE(errRef, err, cs.size(), TEST_PREC);
}

TEST_FIXTURE(SliderCrank3DSphericalJoint
  , TestSliderCrank3DSphericalJointConstraintsVelocityErrors) {
  VectorNd errd(VectorNd::Zero(cs.size()));
  VectorNd errdRef(VectorNd::Zero(cs.size()));
  MatrixNd G(cs.size(), model.dof_count);
  VectorNd qWeights(model.dof_count);
  VectorNd qInit(model.q_size);
  bool success;

  // Compute assembled configuration.
  qWeights[0] = 1.;
  qWeights[1] = 1.;
  qWeights[2] = 1.;
  qWeights[3] = 1.;
  qWeights[4] = 1.;

  Quaternion quat = Quaternion::fromZYXAngles(Vector3d(-0.25 * M_PI, 0.1,0.1));
  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  model.SetQuaternion(id_s, quat, qInit);

  success = CalcAssemblyQ(model, qInit, cs, q, qWeights, 1e-14, 800);
  assert(success);

  // Some random velocity.
  qd[0] = -0.2;
  qd[1] = 0.1 * M_PI;
  qd[2] = -0.1 * M_PI;
  qd[3] = 0.;
  qd[4] = 0.1 * M_PI;

  CalcConstraintsVelocityError(model, q, qd, cs, errd);
  CalcConstraintsJacobian(model, q, cs, G);
  errdRef = G * qd;

  CHECK_ARRAY_CLOSE(errdRef, errd, cs.size(), TEST_PREC);
}

TEST_FIXTURE(SliderCrank3DSphericalJoint
  , TestSliderCrank3DSphericalJointAssemblyQ) {
  VectorNd weights(model.dof_count);
  VectorNd qInit(model.q_size);

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

  Quaternion quat = Quaternion::fromZYXAngles(Vector3d(-0.25 * M_PI, 0.1,0.1));
  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  model.SetQuaternion(id_s, quat, qInit);

  success = CalcAssemblyQ(model, qInit, cs, q, weights, 1e-14, 800);
  pos_p = CalcBodyToBaseCoordinates(model, q, id_p, X_p.r);
  pos_s = CalcBodyToBaseCoordinates(model, q, id_s, X_s.r);
  rot_p = CalcBodyWorldOrientation(model, q, id_p).transpose() * X_p.E;
  rot_s = CalcBodyWorldOrientation(model, q, id_s).transpose() * X_s.E;
  rot_ps = rot_p.transpose() * rot_s;

  CHECK(success);
  CHECK_ARRAY_CLOSE(pos_p, pos_s, 3, TEST_PREC);
  CHECK_CLOSE(0., rot_ps(0,1) - rot_ps(1,0), TEST_PREC);
}

TEST_FIXTURE(SliderCrank3DSphericalJoint
  , TestSliderCrank3DSphericalJointAssemblyQDot) {
  VectorNd qWeights(model.dof_count);
  VectorNd qdWeights(model.dof_count);
  VectorNd qInit(model.q_size);
  VectorNd qdInit(model.dof_count);

  SpatialVector vel_p;
  SpatialVector vel_s;

  bool success;

  qWeights[0] = 1.;
  qWeights[1] = 1.;
  qWeights[2] = 1.;
  qWeights[3] = 1.;
  qWeights[4] = 1.;

  Quaternion quat = Quaternion::fromZYXAngles(Vector3d(-0.25 * M_PI, 0.1,0.1));
  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  model.SetQuaternion(id_s, quat, qInit);

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

  success = CalcAssemblyQ(model, qInit, cs, q, qWeights, 1e-14, 800);
  assert(success);

  CalcAssemblyQDot(model, q, qdInit, cs, qd, qdWeights);

  vel_p = CalcPointVelocity6D(model, q, qd, id_p, X_p.r);
  vel_s = CalcPointVelocity6D(model, q, qd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(vel_p[i], vel_s[i], TEST_PREC);
  }
  CHECK_CLOSE(qdInit[0], qd[0], TEST_PREC);
}

TEST_FIXTURE(SliderCrank3DSphericalJoint
  , TestSliderCrank3DSphericalJointForwardDynamics) {

  VectorNd qWeights(model.dof_count);
  VectorNd qdWeights(model.dof_count);
  VectorNd qInit(model.q_size);
  VectorNd qdInit(model.dof_count);

  SpatialVector acc_p;
  SpatialVector acc_s;

  bool success;

#ifndef RBDL_USE_SIMPLE_MATH
  // The SimpleMath solver cannot solve the system close to a singular
  // configuration.
  // Test with zero q and qdot.

  tau[0] = 0.12;
  tau[1] = -0.3;
  tau[2] = 0.05;
  tau[3] = 0.7;
  tau[4] = -0.1;

  ForwardDynamicsConstraintsDirect(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsNullSpace(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsRangeSpaceSparse(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }
#endif

  // Compute non-zero assembly q and qdot;

  qWeights[0] = 1.;
  qWeights[1] = 1.;
  qWeights[2] = 1.;
  qWeights[3] = 1.;
  qWeights[4] = 1.;

  Quaternion quat = Quaternion::fromZYXAngles(Vector3d(-0.25 * M_PI, 0.1,0.1));
  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  model.SetQuaternion(id_s, quat, qInit);

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

  success = CalcAssemblyQ(model, qInit, cs, q, qWeights, 1e-14, 800);
  assert(success);
  CalcAssemblyQDot(model, q, qdInit, cs, qd, qdWeights);

  Matrix3d rot_ps 
    = (CalcBodyWorldOrientation(model, q, id_p).transpose() * X_p.E).transpose()
    * CalcBodyWorldOrientation(model, q, id_s).transpose() * X_s.E;
  assert((CalcBodyToBaseCoordinates(model, q, id_p, X_p.r)
    - CalcBodyToBaseCoordinates(model, q, id_p, X_p.r)).norm() < TEST_PREC);
  assert(rot_ps(0,1) - rot_ps(0,1) < TEST_PREC);
  assert((CalcPointVelocity6D(model, q, qd, id_p, X_p.r)
    -CalcPointVelocity6D(model, q, qd, id_p, X_p.r)).norm() < TEST_PREC);

  // Test with non-zero q and qdot.

  ForwardDynamicsConstraintsDirect(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsNullSpace(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsRangeSpaceSparse(model, q, qd, tau, cs, qdd);

  acc_p = CalcPointAcceleration6D(model, q, qd, qdd, id_p, X_p.r);
  acc_s = CalcPointAcceleration6D(model, q, qd, qdd, id_s, X_s.r);

  for(size_t i = 2; i < 6; ++i) {
    CHECK_CLOSE(acc_p[i], acc_s[i], TEST_PREC);
  }
}

TEST_FIXTURE(SliderCrank3DSphericalJoint
  , TestSliderCrank3DSphericalJointImpulse) {

  VectorNd qdPlusDirect(model.dof_count);
  VectorNd qdPlusRangeSpaceSparse(model.dof_count);
  VectorNd qdPlusNullSpace(model.dof_count);
  VectorNd errdDirect(cs.size());
  VectorNd errdSpaceSparse(cs.size());
  VectorNd errdNullSpace(cs.size());
  VectorNd qWeights(model.dof_count);
  VectorNd qInit(model.q_size);

  qWeights[0] = 1.;
  qWeights[1] = 1.;
  qWeights[2] = 1.;
  qWeights[3] = 1.;
  qWeights[4] = 1.;

  Quaternion quat = Quaternion::fromZYXAngles(Vector3d(-0.25 * M_PI, 0.1,0.1));
  qInit[0] = 0.4;
  qInit[1] = 0.25 * M_PI;
  model.SetQuaternion(id_s, quat, qInit);

  bool success = CalcAssemblyQ(model, qInit, cs, q, qWeights, 1e-14, 800);
  assert(success);

  cs.v_plus[0] = 1.;
  cs.v_plus[1] = 2.;
  cs.v_plus[2] = 3.;
  cs.v_plus[3] = 4.;
  
  ComputeConstraintImpulsesDirect(model, q, qd, cs, qdPlusDirect);
  CalcConstraintsVelocityError(model, q, qdPlusDirect, cs, errdDirect);
  
  CHECK_ARRAY_CLOSE(cs.v_plus, errdDirect, cs.size(), TEST_PREC);

  ComputeConstraintImpulsesRangeSpaceSparse(model, q, qd, cs
    , qdPlusRangeSpaceSparse);
  CalcConstraintsVelocityError(model, q, qdPlusRangeSpaceSparse, cs
    , errdSpaceSparse);
  
  CHECK_ARRAY_CLOSE(cs.v_plus, errdSpaceSparse, cs.size(), TEST_PREC);

  ComputeConstraintImpulsesNullSpace(model, q, qd, cs, qdPlusNullSpace);
  CalcConstraintsVelocityError(model, q, qdPlusNullSpace, cs, errdNullSpace);

  CHECK_ARRAY_CLOSE(cs.v_plus, errdNullSpace, cs.size(), TEST_PREC);

  cs.v_plus[0] = 0.;
  cs.v_plus[1] = 0.;
  cs.v_plus[2] = 0.;
  cs.v_plus[3] = 0.;

  qd[0] = 1.;
  qd[1] = 2.;
  qd[2] = 3.;

  ComputeConstraintImpulsesDirect(model, q, qd, cs, qdPlusDirect);
  CalcConstraintsVelocityError(model, q, qdPlusDirect, cs, errdDirect);
  
  CHECK_ARRAY_CLOSE(cs.v_plus, errdDirect, cs.size(), TEST_PREC);

  ComputeConstraintImpulsesRangeSpaceSparse(model, q, qd, cs
    , qdPlusRangeSpaceSparse);
  CalcConstraintsVelocityError(model, q, qdPlusRangeSpaceSparse, cs
    , errdSpaceSparse);
  
  CHECK_ARRAY_CLOSE(cs.v_plus, errdSpaceSparse, cs.size(), TEST_PREC);

  ComputeConstraintImpulsesNullSpace(model, q, qd, cs, qdPlusNullSpace);
  CalcConstraintsVelocityError(model, q, qdPlusNullSpace, cs, errdNullSpace);

  CHECK_ARRAY_CLOSE(cs.v_plus, errdNullSpace, cs.size(), TEST_PREC);
}

TEST(ConstraintCorrectnessTest) {
  DoublePerpendicularPendulumAbsoluteCoordinates dba
    = DoublePerpendicularPendulumAbsoluteCoordinates();
  DoublePerpendicularPendulumJointCoordinates dbj
    = DoublePerpendicularPendulumJointCoordinates();

  //1. Set the pendulum modeled using joint coordinates to a specific
  //    state and then compute the spatial acceleration of the body.
  dbj.q[0]  = M_PI/3.0;       //About z0
  dbj.q[1]  = M_PI/6.0;       //About y1
  dbj.qd[0] = M_PI;           //About z0
  dbj.qd[1] = M_PI/2.0;       //About y1
  dbj.tau[0]= 0.;
  dbj.tau[1]= 0.;

  ForwardDynamics(dbj.model,dbj.q,dbj.qd,dbj.tau,dbj.qdd);

  Vector3d r010 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB1,
                    Vector3d(0.,0.,0.),true);
  Vector3d r020 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB2,
                    Vector3d(0.,0.,0.),true);
  Vector3d r030 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB2,
                    Vector3d(dbj.l2,0.,0.),true);

  SpatialVector v010 = CalcPointVelocity6D(
                        dbj.model,dbj.q,dbj.qd,dbj.idB1,
                        Vector3d(0.,0.,0.),true);
  SpatialVector v020 = CalcPointVelocity6D(
                        dbj.model,dbj.q,dbj.qd,dbj.idB2,
                        Vector3d(0.,0.,0.),true);
  SpatialVector v030 = CalcPointVelocity6D(
                        dbj.model,dbj.q,dbj.qd,dbj.idB2,
                        Vector3d(dbj.l2,0.,0.),true);

  SpatialVector a010 = CalcPointAcceleration6D(
                        dbj.model,dbj.q,dbj.qd,dbj.qdd,
                        dbj.idB1,Vector3d(0.,0.,0.),true);
  SpatialVector a020 = CalcPointAcceleration6D(
                        dbj.model,dbj.q,dbj.qd,dbj.qdd,
                        dbj.idB2,Vector3d(0.,0.,0.),true);
  SpatialVector a030 = CalcPointAcceleration6D(
                        dbj.model,dbj.q,dbj.qd,dbj.qdd,
                        dbj.idB2,Vector3d(dbj.l2,0.,0.),true);

  //2. Set the pendulum modelled using absolute coordinates to the
  //   equivalent state as the pendulum modelled using joint
  //   coordinates. Next

  /*
  dba.q[0]  = dbj.q[0];
  dba.q[1]  = r020[0];
  dba.q[2]  = r020[1];
  dba.q[3]  = r020[2];
  dba.q[4]  = 0;
  dba.q[5]  = 0;
  dba.q[6]  = dbj.q[0]+dbj.q[1];

  dba.qd[0]  = dbj.qd[0];
  dba.qd[1]  = v020[0];
  dba.qd[2]  = v020[1];
  dba.qd[3]  = v020[2];
  dba.qd[4]  = 0;
  dba.qd[5]  = 0;
  dba.qd[6]  = dbj.qd[0]+dbj.qd[1];
  */


  dba.q[0]  = r010[0];
  dba.q[1]  = r010[1];
  dba.q[2]  = r010[2];
  dba.q[3]  = dbj.q[0];
  dba.q[4]  = 0;
  dba.q[5]  = 0;
  dba.q[6]  = r020[0];
  dba.q[7]  = r020[1];
  dba.q[8]  = r020[2];
  dba.q[9]  = dbj.q[0];
  dba.q[10] = dbj.q[1];
  dba.q[11] = 0;

  dba.qd[0]  = v010[3];
  dba.qd[1]  = v010[4];
  dba.qd[2]  = v010[5];
  dba.qd[3]  = dbj.qd[0];
  dba.qd[4]  = 0;
  dba.qd[5]  = 0;
  dba.qd[6]  = v020[3];
  dba.qd[7]  = v020[4];
  dba.qd[8]  = v020[5];
  dba.qd[9]  = dbj.qd[0];
  dba.qd[10] = dbj.qd[1];
  dba.qd[11] = 0;

  VectorNd err(dba.cs.size());
  VectorNd errd(dba.cs.size());

  CalcConstraintsPositionError(dba.model,dba.q,dba.cs,err,true);
  CalcConstraintsVelocityError(dba.model,dba.q,dba.qd,dba.cs,errd,true);

  //The constraint errors at the position and velocity level
  //must be zero before the accelerations can be tested.
  for(unsigned int i=0; i<err.rows();++i){
    CHECK_CLOSE(0,err[i],TEST_PREC);
  }
  for(unsigned int i=0; i<errd.rows();++i){
    CHECK_CLOSE(0,errd[i],TEST_PREC);
  }

  //Evaluate the accelerations of the constrained pendulum and
  //compare those to the joint-coordinate pendulum
  for(unsigned int i=0; i<dba.tau.rows();++i){
    dba.tau[i] = 0.;
  }
  ForwardDynamicsConstraintsDirect(dba.model,dba.q,dba.qd,
                                   dba.tau,dba.cs,dba.qdd);
  SpatialVector a010c =
      CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB1,Vector3d(0.,0.,0.),true);
  SpatialVector a020c =
      CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(0.,0.,0.),true);
  SpatialVector a030c =
      CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(dba.l2,0.,0.),true);

  for(unsigned int i=0; i<6;++i){
    CHECK_CLOSE(a010[i],a010c[i],TEST_PREC);
    CHECK_CLOSE(a020[i],a020c[i],TEST_PREC);
    CHECK_CLOSE(a030[i],a030c[i],TEST_PREC);
  }

  /*
  ForwardDynamicsConstraintsNullSpace(dba.model,dba.q,dba.qd,
                                   dba.tau,dba.cs,dba.qdd);
  a010c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB1,Vector3d(0.,0.,0.),true);
  a020c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(0.,0.,0.),true);
  a030c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(dba.l2,0.,0.),true);

  for(unsigned int i=0; i<6;++i){
    CHECK_CLOSE(a010[i],a010c[i],TEST_PREC);
    CHECK_CLOSE(a020[i],a020c[i],TEST_PREC);
    CHECK_CLOSE(a030[i],a030c[i],TEST_PREC);
  }

  bool here=true;

  ForwardDynamicsConstraintsRangeSpaceSparse(dba.model,dba.q,dba.qd,
                                             dba.tau,dba.cs,dba.qdd);
  a010c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB1,Vector3d(0.,0.,0.),true);
  a020c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(0.,0.,0.),true);
  a030c = CalcPointAcceleration6D(dba.model,dba.q,dba.qd,dba.qdd,
                          dba.idB2,Vector3d(dba.l2,0.,0.),true);

  for(unsigned int i=0; i<6;++i){
    CHECK_CLOSE(a010[i],a010c[i],TEST_PREC);
    CHECK_CLOSE(a020[i],a020c[i],TEST_PREC);
    CHECK_CLOSE(a030[i],a030c[i],TEST_PREC);
  }
  */
}
