#include <UnitTest++.h>
#include "rbdl/rbdl.h"
#include "rbdl/Constraints.h"
#include <cassert>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;


//struct DoublePendulumAbsoluteCoordinatesCustomConstraint {

//}

/*
 A pin joint has 5 constraints between points p and
 q on bodies 1 and 2:

 r0P-r0Q = 0   : the points p and q are coincident.
 e1x'e2y   = 0 : the x axis of frame 1 is perp. to y axis of frame 2
 e1x'e2z   = 0 : the x axis of frame 1 is perp. to y axis of frame 2

 Taking the first time derivative of this position-level time invariant
 constraint yields (when expressed in the root frame)

     [J_r0P0_q             - J_r0Q0_q] dq = 0
     [J_e1x0_q'*e2y0 + e1x1'*J_e2y0_q] dq = 0
     [J_e1x0_q'*e2z0   e1x1'*J_e2z0_q] dq = 0


 Where the terms in the square brackets define the 5 x n Jacobian matrix
 for this constraint. The Jacobian matrix can be decomposed into

      G = Tu'*[X_JP - X_JQ]

 Where
  G     : constraint Jacobian
  Tu    : 6D directions of the constraints vectors in the root frame
          Note: Tu = X_P0*TuP
  X_JP  : Jacobian of the 6D vector (from the root frame to P) w.r.t. q
  X_JQ  : Jacobian of the 6D vector (from the root frame to Q) w.r.t. q

 Notation Conventions for 3d quantities
   r0P0 r: vector
        0: from frame 0 (root frame)
        P: to point 1
        0: in the coordinates of the root frame

   e1x0 e: direction unit vector
        1: from the 1 frame
        x: along the x-axis (of the 1 frame)
        0: resolved in the coordinates of frame 0.
   rmP0 rm: rotation matrix
        P : from frame P
        0 : to frame 0

*/
struct CustomXAxisPinJointConstraint : public RigidBodyDynamics::CustomConstraint
{

  CustomXAxisPinJointConstraint(){
    mConstraintCount = 5;
    TuP.resize(mConstraintCount);
    for(unsigned int i=0; i<TuP.size(); ++i){
      TuP[i].setZero();
    }

    /*A pin joint has constraints about these axis when expressed
      in the frame of the predecessor body:
          0   1   2   3   4   5
         wx  wy  wz  rx  ry  rz
     0: [ 0   1   0   0   0   0 ]
     1: [ 0   0   1   0   0   0 ]
     2: [ 0   0   0   1   0   0 ]
     3: [ 0   0   0   0   1   0 ]
     4: [ 0   0   0   0   0   1 ]
    */
    TuP[0](1) = 1;
    TuP[1](2) = 1;
    TuP[2](3) = 1;
    TuP[3](4) = 1;
    TuP[4](5) = 1;

  }


  virtual void CalcConstraintsJacobian(
                  Model &model,
                  unsigned int ccid,
                  const Math::VectorNd &Q,
                  ConstraintSet &CS,
                  Math::MatrixNd &Gblock)
  {


      //Set the working matrices for the previous/successor point Jacobians
      //to zero
      CS.GSpi.setZero();
      CS.GSsi.setZero();

      CalcPointJacobian6D(model,Q,CS.body_p[ccid],CS.X_p[ccid].r,CS.GSpi,false);
      CalcPointJacobian6D(model,Q,CS.body_s[ccid],CS.X_s[ccid].r,CS.GSsi,false);
      CS.GSJ = CS.GSsi-CS.GSpi;

      r0P0 = CalcBodyToBaseCoordinates (model, Q, CS.body_p[ccid],
                                                  CS.X_p[ccid].r, false);
      rmP0 = CalcBodyWorldOrientation (model, Q, CS.body_p[ccid], false
                                       ).transpose()* CS.X_p[ccid].E;
      xP0  = SpatialTransform (rmP0, r0P0);

      for(unsigned int i=0; i<TuP.size(); ++i){
        eT0 = xP0.apply(TuP[i]);
        CS.constraintAxis[ccid] = TuP[i]; //To keep it up to date.
        Gblock.block(i,0,1,model.dof_count) = eT0.transpose()*CS.GSJ;
      }
  }

  virtual void CalcGamma( Model &model,
                          unsigned int ccid,
                          const Math::VectorNd &Q,
                          const Math::VectorNd &QDot,
                          ConstraintSet &CS,
                          const MatrixNd &Gblock,
                          Math::VectorNd &gammaBlock)
  {
  /*
    Position-level

      phi(q) = 0
      r0P-r0S   = 0 : the points p and q are coincident.
      e1x'e2y   = 0 : the x axis of frame 1 is perp. to y axis of frame 2
      e1x'e2z   = 0 : the x axis of frame 1 is perp. to y axis of frame 2

    Velocity-level
      D_phi(q)_Dq * dq/dt = 0
      [J_r0P0_q             - J_r0Q0_q] dq/dt = 0
      [J_e1x0_q'*e2y0 + e1x1'*J_e2y0_q] dq/dt = 0
      [J_e1x0_q'*e2z0   e1x1'*J_e2z0_q] dq/dt = 0

      Or equivalently

      Tu[vP - vS] = 0

      where Tu are the directions the constraint is applied in and
      vP and vQ are the spatial velocities of points P and Q. This
      can be re-worked into:

      G(q)*dq/dt = 0

      Where G(q) is the Jacobian of the constraint phi w.r.t. q.

    Acceleration-level

      d/dt(Tu)[vP - vS] + Tu[aP - aS] = 0

      This is equivalent to

      G(q)*d^2q/dt^2 + [D_G(q)_Dq * dq/dt]*dq/dt = 0.

      Gamma is the term on the right. Note that it has no d^2q/dt^2 terms.
      Thus the gamma term can be computed by evaluating

      Gamma = - (vP x Tu)[vP - vS] - Tu[aP* - aS*]

      where aP* and aQ* are the spatial accelerations of points P and Q
      evaluated with d^2q/dt^2 = 0.
  */
    v0P0 = CalcPointVelocity6D(model, Q, QDot,
                               CS.body_p[ccid],
                               CS.X_p[ccid].r,false);

    v0S0 = CalcPointVelocity6D(model, Q, QDot,
                               CS.body_s[ccid],
                               CS.X_s[ccid].r,false);

    dv0P0nl = CalcPointAcceleration6D(model, Q, QDot,
                                   VectorNd::Zero(model.dof_count),
                                   CS.body_p[ccid],
                                   CS.X_p[ccid].r,false);

    dv0S0nl = CalcPointAcceleration6D(model, Q, QDot,
                                   VectorNd::Zero(model.dof_count),
                                   CS.body_s[ccid],
                                   CS.X_s[ccid].r,false);
    r0P0 = CalcBodyToBaseCoordinates (model, Q, CS.body_p[ccid], CS.X_p[ccid].r,
                                      false);
    rmP0 = CalcBodyWorldOrientation (model, Q, CS.body_p[ccid], false
                                     ).transpose() * CS.X_p[ccid].E;

    xP0 = SpatialTransform (rmP0, r0P0);

    for(unsigned int i=0; i<TuP.size(); ++i){
      eT0    = xP0.apply(TuP[i]);
      eT0Dot = crossm(v0P0,eT0);
      gammaBlock(i) = -eT0.dot(dv0P0nl-dv0S0nl)
                      -eT0Dot.dot(v0P0-v0S0);
    }

  }

  virtual void CalcPositionError( Model &model,
                                  unsigned int ccid,
                                  const Math::VectorNd &Q,
                                  ConstraintSet &CS,
                                  Math::VectorNd &errPosBlock)
  {
    r0P0 = CalcBodyToBaseCoordinates (model, Q, CS.body_p[ccid],
                                                CS.X_p[ccid].r, false);
    r0S0 = CalcBodyToBaseCoordinates (model, Q, CS.body_s[ccid],
                                                CS.X_s[ccid].r, false);
    rmP0 = CalcBodyWorldOrientation (model, Q, CS.body_p[ccid], false
                                     ).transpose()* CS.X_p[ccid].E;
    rmS0 = CalcBodyWorldOrientation (model, Q, CS.body_s[ccid], false
                                     ).transpose()* CS.X_s[ccid].E;
    rmPS = rmP0.transpose()*rmS0;

    //From Davide Corradi's nice expressions for the relative
    //orientation error. To do: CHECK THIS!
    err[0] = -0.5 * (rmPS(1,2) - rmPS(2,1));
    err[1] = -0.5 * (rmPS(2,0) - rmPS(0,2));
    err[2] = -0.5 * (rmPS(0,1) - rmPS(1,0));
    err.block<3,1>(3,0) = rmP0.transpose() * (r0S0 - r0P0);

    for(unsigned int i=0; i < TuP.size(); ++i){
      errPosBlock(i) = TuP[i].transpose()*err;
    }

  }

  virtual void CalcVelocityError( Model &model,
                                  unsigned int custom_constraint_id,
                                  const Math::VectorNd &Q,
                                  const Math::VectorNd &QDot,
                                  ConstraintSet &CS,
                                  const Math::MatrixNd &Gblock,
                                  Math::VectorNd &errVelBlock)
  {
    //Since this is a time-invariant constraint the expression for
    //the velocity error is quite straight forward:
    errVelBlock = Gblock*QDot;
  }

  std::vector < SpatialVector > TuP; //Constraint direction vectors resolved in
                                     //the frame that P is on.
  SpatialVector eT0,eT0Dot;
  SpatialVector err;

  Vector3d r0P0;
  Matrix3d rmP0;
  Vector3d r0S0;
  Matrix3d rmS0;
  Matrix3d rmPS;
  SpatialTransform xP0;

  SpatialVector v0P0,v0S0,dv0P0nl,dv0S0nl;

};

struct DoublePendulumAbsoluteCoordinatesLoopConstraint {
  DoublePendulumAbsoluteCoordinatesLoopConstraint()
    : model()
    , cs()
    , q()
    , qd()
    , qdd()
    , tau()
    , l1(1.)
    , l2(1.)
    , m1(1.)
    , m2(1.)
    , idB1(0)
    , idB2(0)
    , X_p1(Xtrans(Vector3d(0., 0., 0.)))
    , X_s1(Xtrans(Vector3d(0., 0., 0.)))
    , X_p2(Xtrans(Vector3d(0.,-l1, 0.)))
    , X_s2(Xtrans(Vector3d(0., 0., 0.))){

    model.gravity = Vector3d(0.,-9.81,0.);
    //Planar pendulum is at 0 when it is hanging down.
    //  x: points to the right
    //  y: points up
    //  z: out of the page
    Body body01 = Body(0, Vector3d( 0.,0.,0.),
                          Vector3d( 0.,0.,0.));

    Body body02 = Body(0, Vector3d( 0.,0.,0.),
                          Vector3d( 0.,0.,0.));

    Body link1 = Body(m1, Vector3d( 0., -l1*0.5,          0.),
                          Vector3d( 0.,      0., m1*l1*l1/3.));

    Body link2 = Body(m2, Vector3d( 0., -l2*0.5,          0.),
                          Vector3d( 0.,      0., m2*l2*l2/3.));
    //Joint joint_free(JointTypeFloatingBase);
    Joint joint_eulerXYZ(JointTypeEulerXYZ);
    Joint joint_transXYZ(JointTypeTranslationXYZ);
    Joint joint_rot_z = Joint(SpatialVector(0.,0.,1.,0.,0.,0.));

    idB01 = model.AddBody(    0, Xtrans(Vector3d(0., 0., 0.)),
                          joint_transXYZ, body01);
    idB1  = model.AddBody(idB01, Xtrans(Vector3d(0., 0., 0. )),
                         joint_eulerXYZ, link1);

    //idB1  = model.AddBody(idB01, Xtrans(Vector3d(0., 0., 0. )),
    //                      joint_rot_z, link1);

    idB02 = model.AddBody(    0, Xtrans(Vector3d(0., 0., 0.)),
                          joint_transXYZ, body02);
    idB2  = model.AddBody(idB02, Xtrans(Vector3d(0., 0., 0.)),
                         joint_eulerXYZ, link2);

    //Make the revolute joints about the y axis using 5 constraints
    //between the end points

    cs.AddLoopConstraint(0, idB1, X_p1, X_s1,
                         SpatialVector(0,0,0,1,0,0), false, 0.1);
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1,
                         SpatialVector(0,0,0,0,1,0), false, 0.1);
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1,
                         SpatialVector(0,0,0,0,0,1), false, 0.1);
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1,
                         SpatialVector(1,0,0,0,0,0), false, 0.1);
    cs.AddLoopConstraint(0, idB1, X_p1, X_s1,
                         SpatialVector(0,1,0,0,0,0), false, 0.1);


    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2,
                         SpatialVector(0,0,0,1,0,0), false, 0.1);
    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2,
                         SpatialVector(0,0,0,0,1,0), false, 0.1);
    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2,
                         SpatialVector(0,0,0,0,0,1), false, 0.1);
    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2,
                         SpatialVector(1,0,0,0,0,0), false, 0.1);
    cs.AddLoopConstraint(idB1, idB2, X_p2, X_s2,
                         SpatialVector(0,1,0,0,0,0), false, 0.1);

    cs.Bind(model);

    q   = VectorNd::Zero(model.dof_count);
    qd  = VectorNd::Zero(model.dof_count);
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
    unsigned int idB01;
    unsigned int idB02;

    SpatialTransform X_p1;
    SpatialTransform X_s1;
    SpatialTransform X_p2;
    SpatialTransform X_s2;

};

TEST(CustomConstraintCorrectnessTest) {
  DoublePendulumAbsoluteCoordinatesLoopConstraint dba 
    = DoublePendulumAbsoluteCoordinatesLoopConstraint();

  CHECK_CLOSE(1.0,1.0,TEST_PREC);


}
