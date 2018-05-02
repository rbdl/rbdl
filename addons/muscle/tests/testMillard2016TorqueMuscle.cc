/*                                                                             *
 *
 * Copyright (c) 2016 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */


//==============================================================================
// INCLUDES
//==============================================================================



#include "../Millard2016TorqueMuscle.h"
#include "../csvtools.h"
#include "../../geometry/tests/numericalTestFunctions.h"
#include <UnitTest++.h>
#include <rbdl/rbdl_math.h>
#include <ctime>
#include <string>
#include <ostream>
#include <sstream>
#include <stdio.h>
#include <exception>
#include <cassert>
#include <vector>


using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;
using namespace std;
/*
   Constructor tests:
   1. Coefficients are copied over correctly.
   2. Curves are made correctly

   calcTorqueMuscleInfo test
   stiffness calculation
   power calculation

*/

TEST(ConstructorRegularCallCheck)
{


    //Check that the 3 constructors when called properly
    //do not abort
    Millard2016TorqueMuscle test0 = Millard2016TorqueMuscle();
    

    SubjectInformation subjectInfo;
      subjectInfo.gender          = GenderSet::Male;
      subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
      subjectInfo.heightInMeters  =  1.732;
      subjectInfo.massInKg        = 69.0;


    Millard2016TorqueMuscle test2 =
    Millard2016TorqueMuscle(
          DataSet::Anderson2007,
          subjectInfo,
          Anderson2007::HipExtension,
          0.0,
          1.0,
          1.0,
          "test_easyConstructor");

    CHECK(abs( test2.getPassiveTorqueScale()-1.0) < TOL);

}



TEST(calcJointTorqueCorrectnessTests){


    double jointAngleOffset     = 0;    
    double signOfJointAngle     = 1;
    double signOfJointTorque    = 1;
    double err = 0.0;

    std::string name("test");

    SubjectInformation subjectInfo;
      subjectInfo.gender          = GenderSet::Male;
      subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
      subjectInfo.heightInMeters  =  1.732;
      subjectInfo.massInKg        = 69.0;

    Millard2016TorqueMuscle tq =
            Millard2016TorqueMuscle(DataSet::Anderson2007,
                                     subjectInfo,
                                     Anderson2007::HipExtension,
                                     jointAngleOffset,
                                     signOfJointAngle,
                                     signOfJointTorque,
                                     name);

    bool flagMakeTestVector = false;
    if(flagMakeTestVector){
      Millard2016TorqueMuscle tqG =
              Millard2016TorqueMuscle(DataSet::Gymnast,
                                       subjectInfo,
                                       Gymnast::HipExtension,
                                       jointAngleOffset,
                                       signOfJointAngle,
                                       signOfJointTorque,
                                       name);
      TorqueMuscleInfo tmiG;
      tqG.calcTorqueMuscleInfo(M_PI/3.0,0.1,0.77,tmiG);

      printf("%f\n",tmiG.fiberAngle);
      printf("%f\n",tmiG.fiberAngularVelocity);
      printf("%f\n",tmiG.activation);
      printf("%f\n",tmiG.fiberTorque);
      printf("%f\n",tmiG.fiberStiffness);
      printf("%f\n",tmiG.fiberPassiveTorqueAngleMultiplier);
      printf("%f\n",tmiG.fiberActiveTorqueAngleMultiplier);
      printf("%f\n",tmiG.fiberActiveTorqueAngularVelocityMultiplier);
      printf("%f\n",tmiG.fiberPassiveTorque);
      printf("%f\n",tmiG.fiberActiveTorque);
      printf("%f\n",tmiG.fiberDampingTorque);
      printf("%f\n",tmiG.fiberNormDampingTorque);
      printf("%f\n",tmiG.fiberActivePower);
      printf("%f\n",tmiG.fiberPassivePower);
      printf("%f\n",tmiG.fiberPower);
      printf("%f\n",tmiG.DjointTorqueDjointAngle);
      printf("%f\n",tmiG.DjointTorqueDjointAngularVelocity);
      printf("%f\n",tmiG.DjointTorqueDactivation);

    }


    //Zero out the passive forces so that calcMuscleTorque reports
    //just the active force - this allows us to test its correctness.
    tq.setPassiveTorqueScale(0.0);
    double tmp = tq.calcJointTorque(0,0,1.0);

    //Test that the get and set functions work for
    //maximum isometric torque
    double tauMaxOld = tq.getMaximumActiveIsometricTorque();
    double tauMax = tauMaxOld*10.0;
    tq.setMaximumActiveIsometricTorque(tauMax);
    tmp = tq.calcJointTorque(0,0,1.0);
    //ensures updateTorqueMuscleCurves is called
    CHECK(abs( tq.getMaximumActiveIsometricTorque()-tauMax)
          < TOL );

    double omegaMaxOld = tq.getMaximumConcentricJointAngularVelocity();
    double omegaMax    = 2.0*fabs(omegaMaxOld);
    tq.setMaximumConcentricJointAngularVelocity(omegaMax);
    tmp = tq.calcJointTorque(0,0,1.0);
    //ensures updateTorqueMuscleCurves is called
    CHECK(abs( abs(tq.getMaximumConcentricJointAngularVelocity())-omegaMax)
          < TOL );

    //getParametersC1C2C3C4C5C6() has been removed and so this
    //test is no longer possible. It is the responsibility of
    //the underlying active-torque-angle curve to ensure that
    //it peaks at 1.0
    /*
    RigidBodyDynamics::Math::VectorNd c1c2c3c4c5c6 =
            tq.getParametersC1C2C3C4C5C6();
    double thetaAtTauMax = c1c2c3c4c5c6[2];
    */
    //TorqueMuscleInfo tqInfo;

    //getParametersC1C2C3C4C5C6() has been removed. It is the
    //responsibility of the underlying curve test code to
    //check for correctness.
    //double torque = tq.calcJointTorque(thetaAtTauMax,0.0,1.0);
    //err = abs(torque -tauMax);
    //CHECK(err< TOL);

    //These tests have been updated because Anderson
    //torque velocity curve had to be replaced because it
    //can behave badly (e.g. the concentric curve of the ankle
    //extensors of a young man does not interest the x axis.)
    //
    //The new curves do not pass exactly through the points
    //0.5*tauMax and 0.75*tauMax, but within 5% of this
    //value.

    //getParametersC1C2C3C4C5C6() has been removed. It is the
    //responsibility of the underlying curve test code to
    //check for correctness.
    //double omegaAt75TauMax = c1c2c3c4c5c6[3];
    //torque = tq.calcJointTorque(thetaAtTauMax,omegaAt75TauMax,1.0);
    //err = abs(torque - 0.75*tauMax)/tauMax;
    //CHECK( err < 0.05);

    //double omegaAt50TauMax = c1c2c3c4c5c6[4];
    //torque = tq.calcJointTorque(thetaAtTauMax,omegaAt50TauMax,1.0);
    //err = abs(torque -0.50*tauMax)/tauMax;
    //CHECK(err < 0.05);


}

TEST(dampingTermTests){

  double err = 0.;
  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;
  double signOfJointVelocity  = signOfJointTorque;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Female;
    subjectInfo.ageGroup        = AgeGroupSet::SeniorOver65;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  Millard2016TorqueMuscle tq =
          Millard2016TorqueMuscle(DataSet::Anderson2007,
                                   subjectInfo,
                                   Anderson2007::HipExtension,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   signOfJointTorque,
                                   name);
  TorqueMuscleInfo tmi0;
  //Test the damping term
  double beta = tq.getNormalizedDampingCoefficient();
  tq.setNormalizedDampingCoefficient(beta+0.1);
  CHECK(abs(beta+0.1-tq.getNormalizedDampingCoefficient())<SQRTEPSILON);

  double omegaMax = tq.getMaximumConcentricJointAngularVelocity();
  double tau = tq.calcJointTorque(-M_PI/3.0, omegaMax,0);
  CHECK(abs(tau) < SQRTEPSILON );

  tq.calcTorqueMuscleInfo(-M_PI/3.0,omegaMax,0.1,tmi0);
  err = abs(tmi0.activation
            *tmi0.fiberActiveTorqueAngleMultiplier
            *tmi0.fiberActiveTorqueAngularVelocityMultiplier
            +tmi0.fiberPassiveTorqueAngleMultiplier
            *tq.getPassiveTorqueScale()
            +tmi0.fiberNormDampingTorque);
  CHECK( err < SQRTEPSILON);

  beta    = tq.getNormalizedDampingCoefficient();
  double tauMax  = tq.getMaximumActiveIsometricTorque();
  tq.calcTorqueMuscleInfo(tq.getJointAngleAtOneNormalizedPassiveIsometricTorque(),
                          -omegaMax,
                          0,
                          tmi0);
  CHECK( abs(tmi0.fiberDampingTorque
             - 1.0*beta*1.0*tauMax) < SQRTEPSILON );
  CHECK( abs(tmi0.fiberNormDampingTorque -beta*1) < SQRTEPSILON );

}

TEST(fittingFunctionTests){
  double err = 0.;

  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;
  double signOfJointVelocity  = signOfJointTorque;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Female;
    subjectInfo.ageGroup        = AgeGroupSet::SeniorOver65;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  Millard2016TorqueMuscle tq =
          Millard2016TorqueMuscle(DataSet::Anderson2007,
                                   subjectInfo,
                                   Anderson2007::HipExtension,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   signOfJointTorque,
                                   name);

  TorqueMuscleInfo tmi0, tmi1;

  tq.setPassiveTorqueScale(1.0);
  tq.setPassiveCurveAngleOffset(0.0);
  double jointAngleAtPassiveTauMax =
      tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  double activation = 0.1;
  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.-SQRTEPSILON,
                          activation,
                          tmi0);

  tq.setPassiveCurveAngleOffset(M_PI/3.0);
  double updJointAngleAtPassiveTauMax =
    tq.getJointAngleAtOneNormalizedPassiveIsometricTorque() ;

  CHECK( abs(updJointAngleAtPassiveTauMax-jointAngleAtPassiveTauMax-M_PI/3.0)
        < SQRTEPSILON);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax+M_PI/3.0,
                          0.-SQRTEPSILON,
                          activation,
                          tmi1);

  CHECK( abs(tmi0.fiberPassiveTorqueAngleMultiplier
            -tmi1.fiberPassiveTorqueAngleMultiplier) < SQRTEPSILON);

  //fitPassiveCurveAngleOffset: Extension test
  double tauMax = tq.getMaximumActiveIsometricTorque();
  jointAngleAtPassiveTauMax
      = tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();
  tq.fitPassiveCurveAngleOffset(1.0,
                                tauMax);

  tq.calcTorqueMuscleInfo(1.0,
                          0.,
                          0.,
                          tmi0);

  CHECK(abs(tmi0.fiberPassiveTorque - tauMax) < SQRTEPSILON);

  //fitPassiveCurveAngleOffset: flexion test
  Millard2016TorqueMuscle tqF =
          Millard2016TorqueMuscle(DataSet::Anderson2007,
                                   subjectInfo,
                                   Anderson2007::HipFlexion,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   -1*signOfJointTorque,
                                   "flexion");
  tauMax = tqF.getMaximumActiveIsometricTorque();
  jointAngleAtPassiveTauMax =
      tqF.getJointAngleAtOneNormalizedPassiveIsometricTorque();
  tqF.fitPassiveCurveAngleOffset(-1.0, tauMax);

  tqF.calcTorqueMuscleInfo(-1.0,
                          0.,
                          0.,
                          tmi0);

  CHECK(abs(tmi0.fiberPassiveTorque - tauMax) < SQRTEPSILON);

  tauMax = tq.getMaximumActiveIsometricTorque();
  jointAngleAtPassiveTauMax
      = tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  tq.fitPassiveTorqueScale(jointAngleAtPassiveTauMax, tauMax*0.5);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          0.,
                          tmi0);

  CHECK(abs(tmi0.fiberPassiveTorque - tauMax*0.5) < SQRTEPSILON);

  //Now for the flexor ...
  tauMax = tqF.getMaximumActiveIsometricTorque();
  jointAngleAtPassiveTauMax
      = tqF.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  tqF.fitPassiveTorqueScale(jointAngleAtPassiveTauMax, tauMax*0.5);

  tqF.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          0.,
                          tmi0);

  CHECK(abs(tmi0.fiberPassiveTorque - tauMax*0.5) < SQRTEPSILON);

  //Now test the calcMaximumActiveIsometricTorqueScalingFactor and
  //calcActivation functions

  double jointTorque =
      tq.calcJointTorque(M_PI/3.0, M_PI/5.0, 0.5);
  TorqueMuscleSummary tms;
  tq.calcActivation(M_PI/3.0, M_PI/5.0, jointTorque,tms);
  activation = tms.activation;
  CHECK(abs(activation-0.5) < SQRTEPSILON );

  double scaling = tq.calcMaximumActiveIsometricTorqueScalingFactor(
        M_PI/3.0,M_PI/5.0,0.5, jointTorque*1.1);
  CHECK(abs(1.1-scaling) < SQRTEPSILON);


}

TEST(calcTorqueMuscleInfoCorrectnessTests){

  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;
  double signOfJointVelocity  = signOfJointTorque;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Female;
    subjectInfo.ageGroup        = AgeGroupSet::SeniorOver65;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  Millard2016TorqueMuscle tq =
          Millard2016TorqueMuscle(DataSet::Anderson2007,
                                   subjectInfo,
                                   Anderson2007::HipExtension,
                                   jointAngleOffset,
                                   signOfJointAngle,
                                   signOfJointTorque,
                                   name);
  double jointAngle       = 0.;
  double jointVelocity    = 0.;
  double activation       = 1.0;

  tq.setPassiveTorqueScale(0.5);
  double tmp = tq.calcJointTorque(0,0,1.0);
  tq.setPassiveTorqueScale(1.0);
  tmp = tq.calcJointTorque(0,0,1.0);

  double tauMax = tq.getMaximumActiveIsometricTorque();
  //RigidBodyDynamics::Math::VectorNd c1c2c3c4c5c6 =
  //        tq.getParametersC1C2C3C4C5C6();

  //RigidBodyDynamics::Math::VectorNd b1k1b2k2 =
  //        tq.getParametersB1K1B2K2();

  /*
  int idx = -1;
  if(b1k1b2k2[0] > 0){
      idx = 0;
  }else if(b1k1b2k2[2] > 0){
      idx = 2;
  }
  */

  /*
  double b = b1k1b2k2[idx];
  double k = b1k1b2k2[idx+1];
  double thetaAtPassiveTauMax = log(tauMax/b)/k;

  double thetaAtTauMax    = c1c2c3c4c5c6[2];
  double omegaAt75TauMax  = c1c2c3c4c5c6[3];
  double omegaAt50TauMax  = c1c2c3c4c5c6[4];

  double jointAngleAtTauMax =
          signOfJointAngle*thetaAtTauMax+jointAngleOffset;
  */

  double jointAngleAtTauMax = tq.getJointAngleAtMaximumActiveIsometricTorque();
  TorqueMuscleInfo tmi;
  tq.calcTorqueMuscleInfo(jointAngleAtTauMax,
                              0.,
                              activation,
                              tmi);

  //Keypoint check: active force components + fiber kinematics

  CHECK(abs(tmi.activation-1)                 < EPSILON);
  CHECK(abs(tmi.jointAngle-jointAngleAtTauMax)< TOL);
  CHECK(abs(tmi.jointAngularVelocity-0.)      < TOL);

  CHECK(abs(tmi.activation-1)                 < EPSILON);
  //CHECK(abs(tmi.fiberAngle-thetaAtTauMax)     < TOL);
  CHECK(abs(tmi.fiberAngularVelocity-0.)      < TOL);

  CHECK(abs(tmi.fiberActiveTorque - tauMax)   < TOL);
  CHECK(abs(tmi.fiberActiveTorqueAngleMultiplier-1.0) < TOL);
  CHECK(abs(tmi.fiberActiveTorqueAngularVelocityMultiplier-1.0)<TOL);

  //Total force check
  double torque = tq.calcJointTorque(jointAngleAtTauMax,0,activation);
  double err    = abs(torque 
                      - signOfJointTorque*(
                          tmi.fiberActiveTorque+tmi.fiberPassiveTorque));
  CHECK(abs(torque 
            - signOfJointTorque*(
                tmi.fiberActiveTorque+tmi.fiberPassiveTorque)) < TOL);

  //Total active force scales with activation
  tq.calcTorqueMuscleInfo( jointAngleAtTauMax,
                           0.,
                           activation*0.5,
                           tmi);

  CHECK(abs(tmi.fiberActiveTorque - tauMax*0.5)   < TOL);

  //Keypoint check - at omega at 50% tau max
  //tq.calcTorqueMuscleInfo(jointAngleAtTauMax,
  //                        signOfJointVelocity*omegaAt50TauMax,
  //                        activation,
  //                        tmi);

  //CHECK(abs(tmi.jointAngularVelocity
  //          -signOfJointVelocity*omegaAt50TauMax)      < TOL);

  //CHECK(abs(tmi.fiberAngularVelocity-omegaAt50TauMax)  < TOL);

  //This test was updated because Anderson's curve is no longer
  //used exactly as it is written in the paper because it
  //has very odd behavior: the parameters used for ankle extension
  //of the young men is such that the force velocity curve on
  //the concentric side never goes to 0.
  //CHECK(abs(tmi.fiberActiveTorque - tauMax*0.5)   < 0.05*tauMax);

  //Keypoint check - power
  CHECK(abs(tmi.jointPower-tmi.fiberPower) < TOL);
  //CHECK(abs(tmi.jointPower-tmi.fiberTorque*omegaAt50TauMax) < TOL);

  //Keypoint check - where passive curve hits tau max.
  double jointAngleAtPassiveTauMax =
          tq.getJointAngleAtOneNormalizedPassiveIsometricTorque();

  //tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
  //                       signOfJointVelocity*omegaAt50TauMax,
  //                        activation,
  //                        tmi);

  //CHECK(abs(tmi.fiberPassiveTorqueAngleMultiplier-1) < TOL);

  //Numerically check the active and passive fiber stiffnesses
  double h = sqrt(EPSILON);
  tq.calcTorqueMuscleInfo(jointAngleAtTauMax*0.75,
                          0.,
                          activation,
                          tmi);


  TorqueMuscleInfo tmiL;
  TorqueMuscleInfo tmiR;

  tq.calcTorqueMuscleInfo(jointAngleAtTauMax*0.75-h,
                          0.,
                          activation,
                          tmiL);

  tq.calcTorqueMuscleInfo(jointAngleAtTauMax*0.75+h,
                          0.,
                          activation,
                          tmiR);

  double jointK = (tmiR.jointTorque-tmiL.jointTorque)/(2*h);
  err = tmi.jointStiffness - jointK;

  CHECK(abs(tmi.jointStiffness-jointK) < 1e-5);

  double fiberK = signOfJointAngle*(tmiR.fiberTorque-tmiL.fiberTorque)/(2*h);
  err = tmi.fiberStiffness - fiberK;
  CHECK(abs(tmi.fiberStiffness - fiberK) < 1e-5);

  tq.setPassiveTorqueScale(1.5);
  tmp = tq.calcJointTorque(0,0,1.0);

  CHECK(abs(tq.getPassiveTorqueScale()-1.5)<TOL);

  tq.setPassiveTorqueScale(1.0);
  tmp = tq.calcJointTorque(0,0,1.0);

  TorqueMuscleInfo tmi0;
  TorqueMuscleInfo tmi1;
  TorqueMuscleInfo tmi2;

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                         0.,
                         activation,
                         tmi0);

  tq.setPassiveTorqueScale(2.0);
  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          activation,
                          tmi1);

  CHECK(abs(tmi0.fiberPassiveTorqueAngleMultiplier -
            0.5*tmi1.fiberPassiveTorqueAngleMultiplier) < TOL);

  CHECK(abs(tmi0.fiberPassiveTorque -
            0.5*tmi1.fiberPassiveTorque) < TOL);

  double jtq = tq.calcJointTorque(jointAngleAtPassiveTauMax,
                                  0.,
                                  activation);
  err = jtq-tmi1.jointTorque;
  CHECK(abs(jtq-tmi1.jointTorque) < TOL );


  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          activation-SQRTEPSILON,
                          tmi0);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          activation,
                          tmi1);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.,
                          activation+SQRTEPSILON,
                          tmi2);

  double DtqDa = tmi1.DjointTorqueDactivation;
  double DtqDa_NUM = (tmi2.jointTorque-tmi0.jointTorque)/(2*SQRTEPSILON);
  err = abs(DtqDa-DtqDa_NUM);
  CHECK(abs(DtqDa-DtqDa_NUM) < abs(DtqDa)*1e-5 );


  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax-SQRTEPSILON,
                          0.,
                          activation,
                          tmi0);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax+SQRTEPSILON,
                          0.,
                          activation,
                          tmi2);

  double DtqDq = tmi1.DjointTorqueDjointAngle;
  double DtqDq_NUM = (tmi2.jointTorque-tmi0.jointTorque)/(2*SQRTEPSILON);
  err = abs(DtqDq-DtqDq_NUM);
  CHECK(abs(DtqDq-DtqDq_NUM) < abs(DtqDq)*1e-5 );

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.-SQRTEPSILON,
                          activation,
                          tmi0);

  tq.calcTorqueMuscleInfo(jointAngleAtPassiveTauMax,
                          0.+SQRTEPSILON,
                          activation,
                          tmi2);

  double DtqDqdot = tmi1.DjointTorqueDjointAngularVelocity;
  double DtqDqdot_NUM = (tmi2.jointTorque-tmi0.jointTorque)/(2*SQRTEPSILON);
  err = abs(DtqDqdot-DtqDqdot_NUM);
  CHECK(abs(DtqDqdot-DtqDqdot_NUM) < abs(DtqDqdot)*1e-5 );


}

TEST(exampleUsage){


  bool printCurves = false;
  bool printAllCurves = false;

  //int dataSet = DataSetAnderson2007;

  //int gender  = 0; //male
  //int age     = 0; //young

  double jointAngleOffset     = 0;
  double signOfJointAngle     = 1;
  double signOfJointTorque    = 1;

  std::string name("test");

  SubjectInformation subjectInfo;
    subjectInfo.gender          = GenderSet::Male;
    subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
    subjectInfo.heightInMeters  =  1.732;
    subjectInfo.massInKg        = 69.0;

  std::vector< Millard2016TorqueMuscle > muscleVector;

  bool exception = false;

  double angleTorqueSigns[][2] = {{-1, 1},
                                  {-1,-1},
                                  { 1,-1},
                                  { 1, 1},
                                  {-1, 1},
                                  {-1,-1}};

  Millard2016TorqueMuscle tqMuscle;
  std::stringstream tqName;
  int tqIdx;

  for(int i=0; i < Anderson2007::LastJointTorque; ++i){

      tqName.str("");
      tqName << DataSet.names[0]
             <<Anderson2007::JointTorqueNames[i];

      tqMuscle = Millard2016TorqueMuscle(
                    DataSet::Anderson2007,
                    subjectInfo,
                    Anderson2007::JointTorque(i),
                    0.0,
                    1.0,
                    1.0,
                    tqName.str() );


      if(printCurves)
          tqMuscle.printJointTorqueProfileToFile("",tqMuscle.getName() ,100);
  }

  for(int i=0; i < Gymnast::LastJointTorque; ++i){

      tqName.str("");
      tqName << DataSet.names[1]
             << Gymnast::JointTorqueNames[i];

      tqMuscle = Millard2016TorqueMuscle(
                    DataSet::Gymnast,
                    subjectInfo,
                    Gymnast::JointTorque(i),
                    0.0,
                    1.0,
                    1.0,
                    tqName.str() );


      if(printCurves)
          tqMuscle.printJointTorqueProfileToFile("",tqMuscle.getName() ,100);
  }

  tqIdx = -1;


  if(printAllCurves){
      std::stringstream muscleName;

      Millard2016TorqueMuscle muscle;
      int genderIdx,ageIdx,tqIdx;

      for(int age =0; age < Anderson2007::LastAgeGroup; ++age){
          for(int gender=0; gender < Anderson2007::LastGender; ++gender){
            for( int tqDir = 0; tqDir < Anderson2007::LastJointTorque; ++tqDir){
              //for(int joint=0; joint < 3; ++joint){
              //   for(int dir = 0; dir < 2; ++dir){
                      muscleName.str(std::string());

                      genderIdx = Anderson2007::Gender(gender);
                      ageIdx    = Anderson2007::AgeGroup(age);
                      tqIdx     = Anderson2007::JointTorque(tqDir);


                      muscleName  << "Anderson2007_"
                                  << AgeGroupSet::names[ageIdx]
                                  << "_"
                                  << GenderSet::names[genderIdx]
                                  << "_"
                                  << JointTorqueSet::names[tqIdx];

                      subjectInfo.ageGroup = AgeGroupSet::item(age);
                      subjectInfo.gender   = GenderSet::item(gender);

                       muscle = Millard2016TorqueMuscle(
                                 DataSet::Anderson2007,
                                 subjectInfo,
                                 Anderson2007::JointTorque(tqDir),
                                 0,
                                 1.0,
                                 1.0,
                                 muscleName.str());

                      const SmoothSegmentedFunction &tp
                          = muscle.getPassiveTorqueAngleCurve();
                      const SmoothSegmentedFunction &ta
                          = muscle.getActiveTorqueAngleCurve();
                      const SmoothSegmentedFunction &tv
                          = muscle.getTorqueAngularVelocityCurve();

                      RigidBodyDynamics::Math::VectorNd tpDomain
                          = tp.getCurveDomain();
                      RigidBodyDynamics::Math::VectorNd tvDomain
                          = tv.getCurveDomain();
                      RigidBodyDynamics::Math::VectorNd taDomain
                          = ta.getCurveDomain();



                      tp.printCurveToCSVFile(
                        "",
                        tp.getName(),
                        tpDomain[0]-0.1*(tpDomain[1]-tpDomain[0]),
                        tpDomain[1]+0.1*(tpDomain[1]-tpDomain[0]));
                      tv.printCurveToCSVFile(
                        "",
                        tv.getName(),
                        tvDomain[0]-0.1*(tvDomain[1]-tvDomain[0]),
                        tvDomain[1]+0.1*(tvDomain[1]-tvDomain[0]));
                      ta.printCurveToCSVFile(
                        "",
                        ta.getName(),
                        taDomain[0]-0.1*(taDomain[1]-taDomain[0]),
                        taDomain[1]+0.1*(taDomain[1]-taDomain[0]));

                  //}
              //}
            }
          }
      }


  }

   //catch(...){
   //     exceptionThrown = true;
   // }
   CHECK(true);




}

int main (int argc, char *argv[])
{
    return UnitTest::RunAllTests ();
}

