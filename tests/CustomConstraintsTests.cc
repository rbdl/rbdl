#include <UnitTest++.h>
#include "rbdl/rbdl.h"
#include "rbdl/Constraints.h"
#include <cassert>

#include "PendulumModels.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;


struct PinJointCustomConstraint : public RigidBodyDynamics::CustomConstraint
{

  PinJointCustomConstraint():RigidBodyDynamics::CustomConstraint(5){
  }

  PinJointCustomConstraint(unsigned int x0y1z2):RigidBodyDynamics::CustomConstraint(5){    
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
    switch(x0y1z2){
      case 0:
      {
        TuP[3][1] = 1;
        TuP[4][2] = 1;
      }break;
      case 1:{
        TuP[3][0] = 1;
        TuP[4][2] = 1;
      }break;
      case 2:{
        TuP[3][0] = 1;
        TuP[4][1] = 1;
      }break;
      default: {
        cerr << "Invalid AxisOfRotation argument" << endl;
      }
    };

    TuP[0][3] = 1;
    TuP[1][4] = 1;
    TuP[2][5] = 1;

  }


  virtual void CalcConstraintsJacobianAndConstraintAxis(
                  Model &model,
                  unsigned int ccid,
                  const Math::VectorNd &Q,
                  ConstraintSet &CS,
                  Math::MatrixNd &G,
                  unsigned int GrowStart,
                  unsigned int GcolStart)
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
        CS.constraintAxis[ccid+i] = TuP[i]; //To keep it up to date.
        G.block(GrowStart+i,GcolStart,1,model.dof_count)
            = eT0.transpose()*CS.GSJ;
      }
  }

  virtual void CalcGamma( Model &model,
                          unsigned int ccid,
                          const Math::VectorNd &Q,
                          const Math::VectorNd &QDot,
                          ConstraintSet &CS,
                          const MatrixNd &Gblock,
                          Math::VectorNd &gamma,
                          unsigned int gammaStartIndex)
  {
  /*
    Position-level

      phi(q) = 0
      r0P-r0S   = 0 : the points p and q are coincident.
      e1x'e2y   = 0 : the x axis of frame 1 is perp. to y axis of frame 2
      e1x'e2z   = 0 : the x axis of frame 1 is perp. to z axis of frame 2

    Velocity-level
      D_phi(q)_Dq * dq/dt = 0
      [J_r0P0_q             - J_r0Q0_q] dq/dt = 0
      [J_e1x0_q'*e2y0 + e1x1'*J_e2y0_q] dq/dt = 0
      [J_e1x0_q'*e2z0 + e1x1'*J_e2z0_q] dq/dt = 0

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
      gamma[i+gammaStartIndex] = -eT0.dot(dv0S0nl-dv0P0nl)
                                 -eT0Dot.dot(v0S0-v0P0);
    }

  }

  virtual void CalcPositionError( Model &model,
                                  unsigned int ccid,
                                  const Math::VectorNd &Q,
                                  ConstraintSet &CS,
                                  Math::VectorNd &errPos,
                                  unsigned int errStartIndex)
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
      errPos[i+errStartIndex] = TuP[i].transpose()*err;
    }

  }

  virtual void CalcVelocityError( Model &model,
                                  unsigned int custom_constraint_id,
                                  const Math::VectorNd &Q,
                                  const Math::VectorNd &QDot,
                                  ConstraintSet &CS,
                                  const Math::MatrixNd &Gblock,
                                  Math::VectorNd &errVel,
                                  unsigned int errStartIndex)
  {
    //Since this is a time-invariant constraint the expression for
    //the velocity error is quite straight forward:
    errVelBlock = Gblock*QDot;
    for(unsigned int i=0; i < errVelBlock.rows();++i){
      errVel[errStartIndex+i] = errVelBlock[i];
    }
  }

  std::vector < SpatialVector > TuP; //Constraint direction vectors resolved in
                                     //the frame that P is on.
  SpatialVector err, eT0, eT0Dot, v0P0,  v0S0, dv0P0nl ,dv0S0nl;
  Vector3d r0P0, r0S0;
  Matrix3d rmP0, rmS0, rmPS;
  SpatialTransform xP0;

  VectorNd errVelBlock;

};



struct DoublePerpendicularPendulumCustomConstraint {
  DoublePerpendicularPendulumCustomConstraint()
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
    Body link1 = Body(m1, Vector3d(          0., -l1*0.5,          0.),
                      Matrix3d( m1*l1*l1/3.,          0.,           0.,
                                          0., m1*l1*l1/30.,           0.,
                                          0.,          0.,  m1*l1*l1/3.));


    Body link2 = Body(m2, Vector3d( l2*0.5,          0.,          0.),
                          Matrix3d( m2*l2*l2/30.,          0.,           0.,
                                              0., m2*l2*l2/3.,           0.,
                                              0.,          0.,  m2*l2*l2/3.));

    //Joint joint_free(JointTypeFloatingBase);
    Joint jointEA123T123(SpatialVector(0.,0.,0.,1.,0.,0.),
                         SpatialVector(0.,0.,0.,0.,1.,0.),
                         SpatialVector(0.,0.,0.,0.,0.,1.),
                         SpatialVector(0.,0.,1.,0.,0.,0.),
                         SpatialVector(0.,1.,0.,0.,0.,0.),
                         SpatialVector(1.,0.,0.,0.,0.,0.));

    idB1  = model.AddBody(0, Xtrans(Vector3d(0., 0., 0. )),
                         jointEA123T123, link1);

    idB2  = model.AddBody(0, Xtrans(Vector3d(0., 0., 0.)),
                         jointEA123T123, link2);

    //Make the revolute joints about the y axis using 5 constraints
    //between the end points

    ccPJZaxis = PinJointCustomConstraint(2);
    ccPJYaxis = PinJointCustomConstraint(1);

    cs.AddCustomConstraint(&ccPJZaxis,   0,idB1, X_p1, X_s1, false, 0.1);
    cs.AddCustomConstraint(&ccPJYaxis,idB1,idB2, X_p2, X_s2, false, 0.1);


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

    PinJointCustomConstraint ccPJZaxis;
    PinJointCustomConstraint ccPJYaxis;
};



TEST(CustomConstraintCorrectnessTest) {

  //Test to add:
  //  Jacobian vs. num Jacobian
  DoublePerpendicularPendulumCustomConstraint dbcc
    = DoublePerpendicularPendulumCustomConstraint();
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

  double qError = 1.0;
  double qDotError = 1.0;

  //Pefectly initialize the pendulum made with custom constraints and
  //perturb the initialization a bit.
  dbcc.q[0]  = r010[0];
  dbcc.q[1]  = r010[1]+qError;
  dbcc.q[2]  = r010[2];
  dbcc.q[3]  = dbj.q[0];
  dbcc.q[4]  = 0;
  dbcc.q[5]  = 0;
  dbcc.q[6]  = r020[0];
  dbcc.q[7]  = r020[1];
  dbcc.q[8]  = r020[2];
  dbcc.q[9]  = dbj.q[0];
  dbcc.q[10] = dbj.q[1];
  dbcc.q[11] = 0;

  dbcc.qd[0]  = v010[3];
  dbcc.qd[1]  = v010[4]+qDotError;
  dbcc.qd[2]  = v010[5];
  dbcc.qd[3]  = dbj.qd[0];
  dbcc.qd[4]  = 0;
  dbcc.qd[5]  = 0;
  dbcc.qd[6]  = v020[3];
  dbcc.qd[7]  = v020[4];
  dbcc.qd[8]  = v020[5];
  dbcc.qd[9]  = dbj.qd[0];
  dbcc.qd[10] = dbj.qd[1];
  dbcc.qd[11] = 0;

  VectorNd err(dbcc.cs.size());
  VectorNd errd(dbcc.cs.size());


  CalcConstraintsPositionError(dbcc.model,dbcc.q,dbcc.cs,err,true);
  CalcConstraintsVelocityError(dbcc.model,dbcc.q,dbcc.qd,dbcc.cs,errd,true);


  CHECK(err.norm()  >= qError);
  CHECK(errd.norm() >= qDotError);

  //Solve for the initial q and qdot terms that satisfy the constraints
  VectorNd qAsm,qDotAsm,w;
  qAsm.resize(dbcc.q.rows());
  qDotAsm.resize(dbcc.q.rows());
  w.resize(dbcc.q.rows());
  for(unsigned int i=0; i<w.rows();++i){
    w[i] = 1.0;
  }
  double tol = 1e-8;
  unsigned int maxIter = 100;

  CalcAssemblyQ(dbcc.model,dbcc.q,dbcc.cs,qAsm,w,tol,maxIter);
  for(unsigned int i=0; i<dbcc.q.rows();++i){
    dbcc.q[i] = qAsm[i];
  }

  CalcAssemblyQDot(dbcc.model,dbcc.q,dbcc.qd,dbcc.cs,qDotAsm,w);
  for(unsigned int i=0; i<dbcc.q.rows();++i){
    dbcc.qd[i] = qDotAsm[i];
  }

  CalcConstraintsPositionError(dbcc.model,dbcc.q,dbcc.cs,err,true);
  CalcConstraintsVelocityError(dbcc.model,dbcc.q,dbcc.qd,dbcc.cs,errd,true);


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
  dba.q = dbcc.q;
  dba.qd= dbcc.qd;

  for(unsigned int i=0; i<dbcc.tau.rows();++i){
    dbcc.tau[i] = 0.;
  }
  ForwardDynamicsConstraintsDirect(dbcc.model,dbcc.q,dbcc.qd,
                                   dbcc.tau,dbcc.cs,dbcc.qdd);

  ForwardDynamicsConstraintsDirect(dba.model, dba.q,  dba.qd,
                                   dba.tau,   dba.cs, dba.qdd);

  for(unsigned int i = 0; i < dba.cs.G.rows(); ++i){
    for(unsigned int j=0; j< dba.cs.G.cols();++j){
      CHECK_CLOSE(dba.cs.G(i,j),dbcc.cs.G(i,j),TEST_PREC);
    }
  }

  for(unsigned int i = 0; i < dba.cs.gamma.rows(); ++i){
    CHECK_CLOSE(dba.cs.gamma[i],dbcc.cs.gamma[i],TEST_PREC);
  }

  for(unsigned int i =0; i < dba.cs.constraintAxis.size(); ++i){
    for(unsigned int j=0; j< dba.cs.constraintAxis[0].rows();++j){
      CHECK_CLOSE(dba.cs.constraintAxis[i][j],
                  dbcc.cs.constraintAxis[i][j],TEST_PREC);
    }
  }
  

  SpatialVector a010c =
      CalcPointAcceleration6D(dbcc.model,dbcc.q,dbcc.qd,dbcc.qdd,
                          dbcc.idB1,Vector3d(0.,0.,0.),true);
  SpatialVector a020c =
      CalcPointAcceleration6D(dbcc.model,dbcc.q,dbcc.qd,dbcc.qdd,
                          dbcc.idB2,Vector3d(0.,0.,0.),true);
  SpatialVector a030c =
      CalcPointAcceleration6D(dbcc.model,dbcc.q,dbcc.qd,dbcc.qdd,
                          dbcc.idB2,Vector3d(dbcc.l2,0.,0.),true);

  for(unsigned int i=0; i<6;++i){
    CHECK_CLOSE(a010[i],a010c[i],TEST_PREC);
    CHECK_CLOSE(a020[i],a020c[i],TEST_PREC);
    CHECK_CLOSE(a030[i],a030c[i],TEST_PREC);
  }

}
