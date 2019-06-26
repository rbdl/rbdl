#include <UnitTest++.h>
#include "rbdl/rbdl.h"
#include "PendulumModels.h"
#include <cassert>

#include <chrono>
#include <thread>
#include <iostream>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;

const bool flag_printTimingData = false;




void calcCuboidInertia(double mass,
                       double xLength,
                       double yLength,
                       double zLength,
                       Vector3d& iner){
    iner[0] = (mass/12.)*(yLength*yLength + zLength*zLength);
    iner[1] = (mass/12.)*(xLength*xLength + zLength*zLength);
    iner[2] = (mass/12.)*(yLength*yLength + xLength*xLength);
}

struct PlanarBipedFloatingBase {
  PlanarBipedFloatingBase()
    : model()
      ,cs()
      ,q()
      ,qd()
      ,qdd()
      ,tau()
      ,pelvisWidth(0.2)
      ,segmentLength(0.5)
      ,pelvisMass(50.0)
      ,segmentMass(1.0)
      ,X_p(Xtrans(Vector3d(0.,0.,-segmentLength)))
      ,X_s(Xtrans(Vector3d(0.,0.,0.))) {

    model.gravity = Vector3d(0.,0.,-9.81);

    Body segment = Body(segmentMass,
                        Vector3d(0, 0., -0.5 * segmentLength),
                        Vector3d(0.,segmentMass*segmentLength*segmentLength/3.,0.));
    Body pelvis  = Body(pelvisMass, Vector3d(0, 0., 0.),
                        Vector3d(0.,pelvisMass*pelvisWidth*pelvisWidth/3.,0.));

    Joint joint_TxzRy = Joint(SpatialVector(0.,0.,0., 1.,0.,0.),
                              SpatialVector(0.,0.,0., 0.,0.,1.),
                              SpatialVector(0.,1.,0., 0.,0.,0.));

    Joint joint_Ry = Joint(SpatialVector(0.,1.,0., 0.,0.,0.));

    idxPelvis       = model.AddBody(0,
                                    Xtrans(Vector3d(0.,0.,0.)),
                                    joint_TxzRy, pelvis);
    idxLeftUpperLeg  = model.AddBody(idxPelvis,
                                    Xtrans(Vector3d(-pelvisWidth*0.5,0.,0.)),
                                    joint_Ry,segment);
    idxLeftLowerLeg = model.AddBody(idxLeftUpperLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Ry, segment);
    idxRightUpperLeg  = model.AddBody(idxPelvis,
                                    Xtrans(Vector3d( pelvisWidth*0.5,0.,0.)),
                                    joint_Ry,segment);
    idxRightLowerLeg = model.AddBody(idxRightUpperLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Ry, segment);

    cs.AddContactConstraint(idxLeftLowerLeg, Vector3d(0.,0.,-segmentLength),
                                             Vector3d(1.,0.,0.));
    cs.AddContactConstraint(idxLeftLowerLeg, Vector3d(0.,0.,-segmentLength),
                                             Vector3d(0.,0.,1.));
    cs.AddContactConstraint(idxRightLowerLeg, Vector3d(0.,0.,-segmentLength),
                                              Vector3d(1.,0.,0.));
    cs.AddContactConstraint(idxRightLowerLeg, Vector3d(0.,0.,-segmentLength),
                                              Vector3d(0.,0.,1.));

    cs.Bind(model);
    q     = VectorNd::Zero(model.dof_count);
    qd    = VectorNd::Zero(model.dof_count);
    qdd   = VectorNd::Zero(model.dof_count);
    tau   = VectorNd::Zero(model.dof_count);

  }

  Model model;
  ConstraintSet cs;

  VectorNd q;
  VectorNd qd;
  VectorNd qdd;
  VectorNd tau;

  double pelvisWidth;
  double segmentLength;
  double pelvisMass;
  double segmentMass;

  unsigned int idxPelvis ;
  unsigned int idxLeftUpperLeg  ;
  unsigned int idxLeftLowerLeg  ;
  unsigned int idxRightUpperLeg ;
  unsigned int idxRightLowerLeg ;

  SpatialTransform X_p;
  SpatialTransform X_s;

};


struct SpatialBipedFloatingBase {
  SpatialBipedFloatingBase()
    : model()
      ,cs()
      ,q()
      ,qd()
      ,qdd()
      ,tau()
      ,pelvisWidth(0.2)
      ,segmentLength(0.5)
      ,footLength(0.3)
      ,pelvisMass(50.0)
      ,segmentMass(5.0)
      ,footMass(1.0)
      ,X_p(Xtrans(Vector3d(0.,0.,-segmentLength)))
      ,X_s(Xtrans(Vector3d(0.,0.,0.))) {


    //From the biped's perspective
    //x: forward
    //z: up
    //y: left

    model.gravity = Vector3d(0.,0.,-9.81);

    Vector3d segmentInertia, pelvisInertia, footInertia;
    calcCuboidInertia(segmentMass,
                      0.1*segmentLength,
                      0.1*segmentLength,
                      segmentLength,
                      segmentInertia);
    calcCuboidInertia(pelvisMass,
                      0.75*pelvisWidth,
                           pelvisWidth,
                      0.50*pelvisWidth,
                      pelvisInertia);
    calcCuboidInertia(footMass,
                          footLength,
                      0.1*footLength,
                      0.1*footLength,
                      footInertia);

    Body segment = Body(segmentMass,
                        Vector3d(0, 0., -0.5 * segmentLength),
                        segmentInertia);
    Body pelvis  = Body(pelvisMass,
                        Vector3d(0, 0., 0.),
                        pelvisInertia);

    Body foot    = Body(footMass,
                        Vector3d(0.5*footLength,0.,0.),
                        footInertia);

    Joint joint_TxyzRxyz = Joint(SpatialVector(0.,0.,0., 1.,0.,0.),
                                 SpatialVector(0.,0.,0., 0.,1.,0.),
                                 SpatialVector(0.,0.,0., 0.,0.,1.),
                                 SpatialVector(1.,0.,0., 0.,0.,0.),
                                 SpatialVector(0.,1.,0., 0.,0.,0.),
                                 SpatialVector(0.,0.,1., 0.,0.,0.));

    Joint joint_Ry  = Joint(SpatialVector(0.,1.,0., 0.,0.,0.));

    Joint joint_Rxy = Joint(SpatialVector(1.,0.,0., 0.,0.,0.),
                            SpatialVector(0.,1.,0., 0.,0.,0.));
    Joint joint_Rxyz = Joint(SpatialVector(1.,0.,0., 0.,0.,0.),
                             SpatialVector(0.,1.,0., 0.,0.,0.),
                             SpatialVector(0.,0.,1., 0.,0.,0.));


    idxPelvis       = model.AddBody(0,
                                    Xtrans(Vector3d(0.,0.,0.)),
                                    joint_TxyzRxyz, pelvis);

    idxLeftUpperLeg  = model.AddBody(idxPelvis,
                                    Xtrans(Vector3d(-pelvisWidth*0.5,0.,0.)),
                                    joint_Rxyz,segment);
    idxLeftLowerLeg = model.AddBody(idxLeftUpperLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Ry, segment);
    idxLeftFoot     = model.AddBody(idxLeftLowerLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Rxy,
                                    foot);

    idxRightUpperLeg  = model.AddBody(idxPelvis,
                                    Xtrans(Vector3d( pelvisWidth*0.5,0.,0.)),
                                    joint_Rxyz,segment);
    idxRightLowerLeg = model.AddBody(idxRightUpperLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Ry, segment);
    idxRightFoot     = model.AddBody(idxRightLowerLeg,
                                    Xtrans(Vector3d(0.,0.,-segmentLength)),
                                    joint_Rxy,
                                    foot);

    SpatialTransform X_zero = Xtrans(Vector3d(0.,0.,0.));
    bool baumgarteEnabled = false;
    double timeStabilityInverse = 0.1;

    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,1,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,0,1,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,0,0,1),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(1,0,0,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,1,0,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxLeftFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,1,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);

    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,1,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,0,1,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,0,0,0,1),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(1,0,0,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,1,0,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);
    cs.AddLoopConstraint(idxRightFoot, 0,X_zero,X_zero,
                                      SpatialVector(0,0,1,0,0,0),
                                      baumgarteEnabled,timeStabilityInverse);

    cs.Bind(model);
    q     = VectorNd::Zero(model.dof_count);
    qd    = VectorNd::Zero(model.dof_count);
    qdd   = VectorNd::Zero(model.dof_count);
    tau   = VectorNd::Zero(model.dof_count);

  }

  Model model;
  ConstraintSet cs;

  VectorNd q;
  VectorNd qd;
  VectorNd qdd;
  VectorNd tau;

  double pelvisWidth;
  double segmentLength;
  double footLength;
  double pelvisMass;
  double segmentMass;
  double footMass;

  unsigned int idxPelvis ;
  unsigned int idxLeftUpperLeg  ;
  unsigned int idxLeftLowerLeg  ;
  unsigned int idxLeftFoot;
  unsigned int idxRightUpperLeg ;
  unsigned int idxRightLowerLeg ;
  unsigned int idxRightFoot;

  SpatialTransform X_p;
  SpatialTransform X_s;

};


TEST_FIXTURE(PlanarBipedFloatingBase, TestCorrectness) {

  //1. Make the simple biped
  //2. Assemble it to a specific q and qdot.

  VectorNd q0, qd0, weights, qddTarget, lambda, qddFwd,qddErr,
           lambdaIdc, lambdaFwd, lambdaErr;
  MatrixNd K;
  std::vector<bool> dofActuated;

  unsigned int n  = unsigned (int (q.rows()));
  //unsigned int na = unsigned (int(q.rows()))-3;
  //unsigned int nc = unsigned (int(cs.name.size()));

  q0.resize(n);
  qd0.resize(n);
  weights.resize(n);
  qddTarget.resize(n);
  qddFwd.resize(n);
  qddErr.resize(n);
  dofActuated.resize(n);



  for(unsigned int i=0; i<n;++i){
    if(i>=3){
      dofActuated[i] = true;
    }else{
      dofActuated[i] = false;
    }
  }

  //A statically stable standing pose
  q0[0] = 0;
  q0[1] = 1.0;
  q0[2] = 0;
  q0[3] = M_PI*0.25;
  q0[4] =-M_PI*0.25;
  q0[5] =-M_PI*0.25;
  q0[6] = M_PI*0.25;

  qd0[0] = 0.;
  qd0[1] = 0.;
  qd0[2] = 0.;
  qd0[3] = 0.;
  qd0[4] = 0.;
  qd0[5] = 0.;
  qd0[6] = 0.;

  for(unsigned int i =0; i< q.rows();++i){
    weights[i]    = 1;
  }

  qddTarget[0]  = 0.;
  qddTarget[1]  = 0.;
  qddTarget[2]  = 0.;
  qddTarget[3]  = 0.;
  qddTarget[4]  = 0.;
  qddTarget[5]  = 0.;
  qddTarget[6]  = 0.;


  bool qasm = CalcAssemblyQ(model,q0,cs,q,weights);
  CHECK(qasm);
  CalcAssemblyQDot(model,q,qd0,cs,qd,weights);


  //6. Call InverseDynamicsConstraints & repeat setps 4 & 5
  VectorNd tauIDC = VectorNd::Zero(tau.size());
  VectorNd qddIDC = VectorNd::Zero(qdd.size());

  cs.SetActuationMap(model, dofActuated);
  InverseDynamicsConstraints(model,
                             q,qd,qddTarget,
                             cs, qddIDC,tauIDC);

  //In this case the target qdd should be reachable exactly.
  for(unsigned int i=0; i<qddIDC.rows();++i){
    CHECK_CLOSE(qddIDC[i], qddTarget[i],TEST_PREC);
  }


  lambdaIdc = cs.force;
  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDC,cs,qddFwd);
  lambdaFwd = cs.force;


  qddErr = qddIDC-qddFwd;
  lambdaErr=lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_CLOSE(qddIDC[i], qddFwd[i], TEST_PREC);
  }
  for(unsigned int i=0; i<lambdaFwd.rows();++i){
    CHECK_CLOSE(lambdaIdc[i], lambdaFwd[i], TEST_PREC);
  }

  //Check the relaxed method
  VectorNd tauIDCR(tauIDC.rows());
  VectorNd qddIDCR(tauIDC.rows());
  InverseDynamicsConstraintsRelaxed(model,
                             q,qd,qddTarget,
                             cs, qddIDCR,tauIDCR);
  lambdaIdc = cs.force;

  //In this case the acceleration constraint is relaxed and so there
  //is always some error between what is asked for and what is received.
  for(unsigned int i=0; i<qddIDC.rows();++i){
    CHECK_CLOSE(qddIDCR[i], qddTarget[i],0.01);
  }

  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDCR,cs,qddFwd);
  lambdaFwd = cs.force;

  lambdaErr=lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_CLOSE(qddIDCR[i], qddFwd[i], TEST_PREC);
  }
  for(unsigned int i=0; i<lambdaFwd.rows();++i){
    CHECK_CLOSE(lambdaIdc[i], lambdaFwd[i], TEST_PREC);
  }

  //Timing comparision
  if(flag_printTimingData){
    unsigned int iterations = 100;

    auto t1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      InverseDynamicsConstraints(model,
                                 q,qd,qddTarget,
                                 cs,qddIDC,tauIDC);
    }
    auto t2   = std::chrono::high_resolution_clock::now();
    auto tidc = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);

    t1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      ForwardDynamicsConstraintsDirect(model,q,qd,tauIDC,cs,qddFwd);
    }
    t2 = std::chrono::high_resolution_clock::now();
    auto tfd = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);

    std::cout << "Planar Biped Dof: " << model.dof_count << std::endl;
    std::cout << "Cost per evaluation : us, xfd " << std::endl;
    std::cout << "IDCns: " << double(tidc.count()) / double(iterations) << " "
                        << double(tidc.count())/double(tfd.count()) << std::endl;
    std::cout << "FD:    " << double(tfd.count()) / double(iterations) << " "
                        << double(tfd.count())/double(tfd.count()) << std::endl;
  }

}


TEST_FIXTURE(SpatialBipedFloatingBase, TestCorrectness) {

  //1. Make the simple spatial biped
  //2. Assemble it to a specific q and qdot.

  VectorNd q0, qd0, weights, qddTarget, lambda, qddFwd,qddErr,
           lambdaFwd,lambdaIdc,lambdaErr;
  MatrixNd K;
  std::vector<bool> dofActuated;

  unsigned int n  = unsigned( int( q.rows()));
  //unsigned int na = unsigned( int( q.rows()))-6;
  unsigned int nc = unsigned( int( cs.name.size()));

  q0.resize(n);
  qd0.resize(n);
  weights.resize(n);
  qddTarget.resize(n);
  qddFwd.resize(n);
  qddErr.resize(n);
  lambda.resize(nc);
  dofActuated.resize(n);

  for(unsigned int i=0; i<n;++i){
    if(i>=6){
      dofActuated[i] = true;
    }else{
      dofActuated[i] = false;
    }
  }

  //All of the ugly numbers below are pseudo random numbers generated
  //from Matlab to ensure that the method works, and that the tests are
  //not passing due to some quirk of symmetry in the state of the model.

  q0[0]  = 0.;         //tx Floating base
  q0[1]  = 0.;         //ty
  q0[2]  = 0.75;       //tx
  q0[3]  = 0.;         //rx
  q0[4]  = 0.;         //ry
  q0[5]  = 0.;         //rz
  q0[6]  = 0.;         //rx Left hip
  q0[7]  =  M_PI*0.25; //ry
  q0[8]  = 0.;         //rz
  q0[9]  = -M_PI*0.25; //ry Left knee
  q0[10] = 0.;         //rx Left ankle
  q0[11] = 0.;         //ry
  q0[12] = 0.;         //rx Right hip
  q0[13] = -M_PI*0.25; //ry
  q0[14] = 0.;         //rz
  q0[15] =  M_PI*0.25; //ry Right knee
  q0[16] = 0.;         //rx Right ankle
  q0[17] = 0.;         //ry

  qd0[0]  = 0.;
  qd0[1]  = 0.;
  qd0[2]  = 0.;
  qd0[3]  = 0.;
  qd0[4]  = 0.;
  qd0[5]  = 0.;
  qd0[6]  = 0.;
  qd0[7]  = 0.;
  qd0[8]  = 0.;
  qd0[9]  = 0.;
  qd0[10] = 0.;
  qd0[11] = 0.;
  qd0[12] = 0.;
  qd0[13] = 0.;
  qd0[14] = 0.;
  qd0[15] = 0.;
  qd0[16] = 0.;
  qd0[17] = 0.;

  for(unsigned int i =0; i< q.rows();++i){
    weights[i]    = 1.;
  }

  qddTarget[0]  = 0.;
  qddTarget[1]  = 0.;
  qddTarget[2]  = 0.;
  qddTarget[3]  = 0.;
  qddTarget[4]  = 0.;
  qddTarget[5]  = 0.;
  qddTarget[6]  = 0.;
  qddTarget[7]  = 0.;
  qddTarget[8]  = 0.;
  qddTarget[9]  = 0.;
  qddTarget[10] = 0.;
  qddTarget[11] = 0.;
  qddTarget[12] = 0.;
  qddTarget[13] = 0.;
  qddTarget[14] = 0.;
  qddTarget[15] = 0.;
  qddTarget[16] = 0.;
  qddTarget[17] = 0.;


  bool qAsm=CalcAssemblyQ(model,q0,cs,q,weights);
  CHECK(qAsm==true);

  CalcAssemblyQDot(model,q,qd0,cs,qd,weights);


  //6. Call InverseDynamicsConstraints

  VectorNd tauIDC = VectorNd::Zero(tau.size());
  VectorNd qddIDC = VectorNd::Zero(qdd.size());
  cs.SetActuationMap(model,dofActuated);
  InverseDynamicsConstraints(model,
                             q,qd,qddTarget,
                             cs, qddIDC,tauIDC);
  lambdaIdc = cs.force;

  //In this case the target qdd should be reachable exactly.
  for(unsigned int i=0; i<qddIDC.rows();++i){
    CHECK_CLOSE(qddIDC[i], qddTarget[i],TEST_PREC);
  }


  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDC,cs,qddFwd);
  lambdaFwd = cs.force;

  //Check that the solution is physically consistent
  qddErr = qddIDC-qddFwd;
  lambdaErr= lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_CLOSE(qddIDC[i], qddFwd[i], TEST_PREC);
  }
  for(unsigned int i=0; i<lambdaIdc.rows();++i){
    CHECK_CLOSE(lambdaIdc[i], lambdaFwd[i], TEST_PREC);
  }

  //Check the relaxed method
  VectorNd tauIDCR(tauIDC.rows());
  VectorNd qddIDCR(qddIDC.rows());
  InverseDynamicsConstraintsRelaxed(model,
                             q,qd,qddTarget,
                             cs, qddIDCR,tauIDCR);
  lambdaIdc = cs.force;

  //In this case the acceleration constraint is relaxed and so there
  //is always some error between what is asked for and what is received.
  for(unsigned int i=0; i<qddIDCR.rows();++i){
    CHECK_CLOSE(qddIDCR[i], qddIDC[i],0.01);
  }


  ForwardDynamicsConstraintsDirect(model,q,qd,tauIDCR,cs,qddFwd);
  lambdaFwd = cs.force;

  //Check that the solution is physically consistent
  qddErr = qddIDCR-qddFwd;
  lambdaErr= lambdaIdc-lambdaFwd;
  for(unsigned int i=0; i<q.rows();++i){
    CHECK_CLOSE(qddIDC[i], qddFwd[i], TEST_PREC);
  }
  for(unsigned int i=0; i<lambdaIdc.rows();++i){
    CHECK_CLOSE(lambdaIdc[i], lambdaFwd[i], TEST_PREC);
  }


  //Timing comparision
  if(flag_printTimingData){
    unsigned int iterations = 100;

    auto t1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      InverseDynamicsConstraints(model,
                                 q,qd,qddTarget,
                                 cs, qddIDC,tauIDC);
    }
    auto t2     = std::chrono::high_resolution_clock::now();
    auto tidc = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);

    t1 = std::chrono::high_resolution_clock::now();
    for(unsigned int i=0; i<iterations; ++i){
      ForwardDynamicsConstraintsDirect(model,q,qd,tauIDC,cs,qddFwd);
    }
    t2 = std::chrono::high_resolution_clock::now();
    auto tfd = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1);

    std::cout << "Spatial Biped Dof: " << model.dof_count << std::endl;
    std::cout << "Cost per evaluation : us, xfd" << std::endl;
    std::cout << "IDCns: " << double(tidc.count())/double(iterations) << " "
                        << double(tidc.count())/double(tfd.count()) << std::endl;
    std::cout << "FD:    " << double(tfd.count())/double(iterations) << " "
                        << double(tfd.count())/double(tfd.count()) << std::endl;
  }
}


TEST(CorrectnessTestWithSinglePlanarPendulum){

  //With loop constraints
  SinglePendulumAbsoluteCoordinates spa
    = SinglePendulumAbsoluteCoordinates();

  //Without any joint coordinates
  SinglePendulumJointCoordinates spj
    = SinglePendulumJointCoordinates();


  //1. Set the pendulum modeled using joint coordinates to a specific
  //    state and then compute the spatial acceleration of the body.
  spj.q[0]  = M_PI/3.0;   //About z0
  spj.qd.setZero();
  spj.qdd.setZero();
  spj.tau.setZero();


  InverseDynamics(spj.model,spj.q,spj.qd,spj.qdd,spj.tau);

  Vector3d r010 = CalcBodyToBaseCoordinates(
                    spj.model,spj.q,spj.idB1,
                    Vector3d(0.,0.,0.),true);

  //2. Set the pendulum modelled using absolute coordinates to the
  //   equivalent state as the pendulum modelled using joint
  //   coordinates.

  spa.q[0]  = r010[0];
  spa.q[1]  = r010[1];
  spa.q[2]  = spj.q[0];


  spa.qd.setZero();
  spa.qdd.setZero();

  spa.tau[0]  = 0.;//tx
  spa.tau[1]  = 0.;//ty
  spa.tau[2]  = spj.tau[0];//rz


  //Test
  //  calcPositionError
  //  calcVelocityError

  VectorNd err(spa.cs.size());
  VectorNd errd(spa.cs.size());

  CalcConstraintsPositionError(spa.model,spa.q,spa.cs,err,true);
  CalcConstraintsVelocityError(spa.model,spa.q,spa.qd,spa.cs,errd,true);

  for(unsigned int i=0; i<err.rows();++i){
    CHECK_CLOSE(0, err[i], TEST_PREC);
  }
  for(unsigned int i=0; i<errd.rows();++i){
    CHECK_CLOSE(0, errd[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsDirect(spa.model,spa.q, spa.qd, spa.tau,spa.cs,
                                   spa.qdd);
  for(unsigned int i=0; i<spa.qdd.rows();++i){
    CHECK_CLOSE(0., spa.qdd[i], TEST_PREC);
  }

  std::vector< bool > actuationMap;
  actuationMap.resize(spa.tau.rows());
  for(unsigned int i=0; i<actuationMap.size();++i){
    actuationMap[i]=false;
  }
  actuationMap[2] = true;

  VectorNd qddDesired = VectorNd::Zero(spa.qdd.rows());
  VectorNd qddIdc = VectorNd::Zero(spa.qdd.rows());
  VectorNd tauIdc = VectorNd::Zero(spa.qdd.rows());

  //The IDC operator should be able to statisfy qdd=0 and return a tau
  //vector that matches the hand solution produced above.
  spa.cs.SetActuationMap(spa.model,actuationMap);
  InverseDynamicsConstraints(spa.model,spa.q,spa.qd,qddDesired,
                                          spa.cs, qddIdc,tauIdc);

  for(unsigned int i=0; i<qddIdc.rows();++i){
    CHECK_CLOSE(0.,qddIdc[i],TEST_PREC);
  }
  for(unsigned int i=0; i<tauIdc.rows();++i){
    CHECK_CLOSE(spa.tau[i],tauIdc[i],TEST_PREC);
  }


  InverseDynamicsConstraintsRelaxed(spa.model,spa.q,spa.qd,qddDesired,
                                    spa.cs, qddIdc,tauIdc);

  //For this simple system with only point-ground constraints the
  //desired qdd's should be met exactly (thanks to the small updates I've made
  for(unsigned int i=0; i<qddIdc.rows();++i){
    CHECK_CLOSE(0.,qddIdc[i],TEST_PREC);
  }
  for(unsigned int i=0; i<tauIdc.rows();++i){
    CHECK_CLOSE(spa.tau[i],tauIdc[i],TEST_PREC);
  }
}



TEST(CorrectnessTestWithDoublePerpendicularPendulum){

  //With loop constraints
  DoublePerpendicularPendulumAbsoluteCoordinates dba
    = DoublePerpendicularPendulumAbsoluteCoordinates();

  //Without any joint coordinates
  DoublePerpendicularPendulumJointCoordinates dbj
    = DoublePerpendicularPendulumJointCoordinates();


  //1. Set the pendulum modeled using joint coordinates to a specific
  //    state and then compute the spatial acceleration of the body.
  dbj.q[0]  = M_PI/3.0;   //About z0
  dbj.q[1]  = M_PI/6.0;         //About y1
  dbj.qd.setZero();
  dbj.qdd.setZero();
  dbj.tau.setZero();


  InverseDynamics(dbj.model,dbj.q,dbj.qd,dbj.qdd,dbj.tau);
  //ForwardDynamics(dbj.model,dbj.q,dbj.qd,dbj.tau,dbj.qdd);

  Vector3d r010 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB1,
                    Vector3d(0.,0.,0.),true);
  Vector3d r020 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB2,
                    Vector3d(0.,0.,0.),true);
  Vector3d r030 = CalcBodyToBaseCoordinates(
                    dbj.model,dbj.q,dbj.idB2,
                    Vector3d(dbj.l2,0.,0.),true);

  //2. Set the pendulum modelled using absolute coordinates to the
  //   equivalent state as the pendulum modelled using joint
  //   coordinates. Next


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

  dba.qd.setZero();
  dba.qdd.setZero();

  dba.tau[0]  = 0.;//tx
  dba.tau[1]  = 0.;//ty
  dba.tau[2]  = 0.;//tz
  dba.tau[3]  = dbj.tau[0];//rz
  dba.tau[4]  = 0.;//ry
  dba.tau[5]  = 0.;//rx
  dba.tau[6]  = 0.;//tx
  dba.tau[7]  = 0.;//ty
  dba.tau[8]  = 0.;//tz
  dba.tau[9]  = 0.;//rz
  dba.tau[10] = dbj.tau[1];//ry
  dba.tau[11] = 0.;//rx



  VectorNd err(dba.cs.size());
  VectorNd errd(dba.cs.size());

  CalcConstraintsPositionError(dba.model,dba.q,dba.cs,err,true);
  CalcConstraintsVelocityError(dba.model,dba.q,dba.qd,dba.cs,errd,true);

  for(unsigned int i=0; i<err.rows();++i){
    CHECK_CLOSE(0, err[i], TEST_PREC);
  }
  for(unsigned int i=0; i<errd.rows();++i){
    CHECK_CLOSE(0, errd[i], TEST_PREC);
  }

  ForwardDynamicsConstraintsDirect(dba.model,dba.q, dba.qd, dba.tau,dba.cs,
                                   dba.qdd);
  for(unsigned int i=0; i<dba.qdd.rows();++i){
    CHECK_CLOSE(0., dba.qdd[i], TEST_PREC);
  }

  std::vector< bool > actuationMap;
  actuationMap.resize(dba.tau.rows());
  for(unsigned int i=0; i<actuationMap.size();++i){
    actuationMap[i]=false;
  }
  actuationMap[3] = true;
  actuationMap[10] = true;

  VectorNd qddDesired = VectorNd::Zero(dba.qdd.rows());
  VectorNd qddIdc = VectorNd::Zero(dba.qdd.rows());
  VectorNd tauIdc = VectorNd::Zero(dba.qdd.rows());

  //The IDC operator should be able to statisfy qdd=0 and return a tau
  //vector that matches the hand solution produced above.
  dba.cs.SetActuationMap(dba.model,actuationMap);
  InverseDynamicsConstraints(dba.model,dba.q,dba.qd,qddDesired,
                                          dba.cs, qddIdc,tauIdc);

  for(unsigned int i=0; i<qddIdc.rows();++i){
    CHECK_CLOSE(0.,qddIdc[i],TEST_PREC);
  }
  for(unsigned int i=0; i<tauIdc.rows();++i){
    CHECK_CLOSE(dba.tau[i],tauIdc[i],TEST_PREC);
  }

  InverseDynamicsConstraintsRelaxed(dba.model,dba.q,dba.qd,qddDesired,
                                          dba.cs, qddIdc,tauIdc);

  //Check if the result is physically consistent.
  VectorNd qddCheck(qddIdc.rows());
  ForwardDynamicsConstraintsDirect(dba.model,dba.q,dba.qd, tauIdc, dba.cs, qddCheck);

  for(unsigned int i=0; i<qddIdc.rows();++i){
    CHECK_CLOSE(qddCheck[i],qddIdc[i],TEST_PREC);
  }

  //Unfortunately because this system contains loop constraints the
  //IDC relaxed operator is not able to exactly satisfy the desired accelerations

  //for(unsigned int i=0; i<qddIdc.rows();++i){
  //  CHECK_CLOSE(0.,qddIdc[i],TEST_PREC);
  //}
  //for(unsigned int i=0; i<tauIdc.rows();++i){
  //  CHECK_CLOSE(dba.tau[i],tauIdc[i],TEST_PREC);
  //}
}


