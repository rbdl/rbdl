/* -------------------------------------------------------------------------- *
 *        OpenSim:  testSmoothSegmentedFunctionFactory.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
/*
    Update:
     This is a port of the original code so that it will work with
     the multibody code RBDL written by Martin Felis.
    
     This also includes additional curves (the Anderson2007 curves)
     which are not presently in OpenSim.

    Author:
     Matthew Millard
    
    Date:
     Nov 2015

*/
/* 
Below is a basic bench mark simulation for the SmoothSegmentedFunctionFactory
class, a class that enables the easy generation of C2 continuous curves 
that define the various characteristic curves required in a muscle model
 */

// Author:  Matthew Millard

//==============================================================================
// INCLUDES
//==============================================================================

#include "../TorqueMuscleFunctionFactory.h"
#include "../../geometry/geometry.h"
#include "../../geometry/tests/numericalTestFunctions.h"


#include <UnitTest++.h>
#include <rbdl/rbdl_math.h>
#include <ctime>
#include <string>
#include <stdio.h>
#include <exception>
#include <cassert>

using namespace RigidBodyDynamics::Addons::Geometry;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace std;

/*
static double EPSILON = numeric_limits<double>::epsilon();

static bool FLAG_PLOT_CURVES    = false;
static string FILE_PATH         = "";
static double TOL_DX            = 5e-3;
static double TOL_DX_BIG        = 1e-2;
static double TOL_BIG           = 1e-6;
static double TOL_SMALL         = 1e-12;
*/

TEST(Anderson2007ActiveTorqueAngleCurve)
{
    double subjectWeight    = 75.0*9.81;
    double subjectHeight    = 1.75;
    double scale      = subjectHeight*subjectWeight; 

    //These parameters are taken from table 3 for hip extension for
    //men between the ages of 18-25
    double c1      = 0.161; //normalized maximum hip joint torque
    double c2      = 0.958; // pi/(theta_max - theta_min)
    double c3      = 0.932; //theta_max_iso_torque
    double c4      = 1.578; //omega_1: angular velocity at 75% tq_iso_max
    double c5      = 3.190; //omega_2: angular velocity at 50% tq_iso_max
    double c6      = 0.242; //E, where eccentric slope = (1+E)*concentricSlope
                //Passive torque angle curve parameters
    double b1      =-1.210; // torque_passive = b1*exp(k1*theta) 
    double k1      =-6.351; //        +b2*exp(k2*theta)
    double b2      = 0.476;            
    double k2      = 5.910;



    //cout <<endl;      
    //cout <<endl;
    //cout <<"**************************************************"<<endl;
    //cout <<"ANDERSON 2007 ACTIVE TORQUE ANGLE CURVE TESTING   "<<endl;

    SmoothSegmentedFunction andersonTaCurve = SmoothSegmentedFunction();
    TorqueMuscleFunctionFactory::
     createAnderson2007ActiveTorqueAngleCurve(c2,c3,
          "test_Anderson2007TorqueAngleCurve",
          andersonTaCurve);      

    double angleRange    = (M_PI/c4); 
    double angleActiveMin   = c3 - angleRange*0.75;
    double angleActiveMax   = c3 + angleRange*0.75;

    RigidBodyDynamics::Math::MatrixNd andersonTaCurveSample 
        = andersonTaCurve.calcSampledCurve( 6,
                       angleActiveMin,
                       angleActiveMax);    

    //cout << "   Keypoint Testing" << endl;

    CHECK(abs(andersonTaCurve.calcValue(c3) - 1.0) < TOL_SMALL);  
    CHECK(abs(andersonTaCurve.calcDerivative(c3,1))     < TOL_BIG);
    CHECK(abs(andersonTaCurve.calcDerivative(c3,2))     < TOL_BIG);

    RigidBodyDynamics::Math::VectorNd curveDomain 
     = andersonTaCurve.getCurveDomain();

    CHECK(abs(andersonTaCurve.calcValue(curveDomain[0]))     < TOL_SMALL);
    CHECK(abs(andersonTaCurve.calcDerivative(curveDomain[0],1))    < TOL_BIG);
    CHECK(abs(andersonTaCurve.calcValue(curveDomain[1]))     < TOL_SMALL);
    CHECK(abs(andersonTaCurve.calcDerivative(curveDomain[1],1))    < TOL_BIG);
    //cout << "   passed " << endl;

    //cout << "    Continuity and Smoothness Testing" << endl;
    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      andersonTaCurve,  
      andersonTaCurveSample, 
      TOL_DX);
    CHECK(areCurveDerivativesGood);

    bool curveIsContinuous = isCurveC2Continuous( andersonTaCurve, 
                                                  andersonTaCurveSample,
                                                  TOL_BIG);
    CHECK(curveIsContinuous);

    if(FLAG_PLOT_CURVES){
     andersonTaCurve.printCurveToCSVFile(
        FILE_PATH,
        "anderson2007ActiveTorqueAngleCurve",
        angleActiveMin,
        angleActiveMax);
    }


}   

TEST(Millard2016PassiveTorqueAngleCurve)
{
    double subjectWeight    = 75.0*9.81;
    double subjectHeight    = 1.75;
    double scale      = subjectHeight*subjectWeight; 

    //These parameters are taken from table 3 for hip extension for
    //men between the ages of 18-25
    double c1      = 0.161; //normalized maximum hip joint torque
    double c2      = 0.958; // pi/(theta_max - theta_min)
    double c3      = 0.932; //theta_max_iso_torque
    double c4      = 1.578; //omega_1: angular velocity at 75% tq_iso_max
    double c5      = 3.190; //omega_2: angular velocity at 50% tq_iso_max
    double c6      = 0.242; //E, where eccentric slope = (1+E)*concentricSlope
                //Passive torque angle curve parameters
    double b1      =-1.210; // torque_passive = b1*exp(k1*theta) 
    double k1      =-6.351; //        +b2*exp(k2*theta)
    double b2      = 0.476;            
    double k2      = 5.910;

    
    //cout <<endl;
    //cout <<endl;
    //cout <<"**************************************************"<<endl;
    //cout <<"ANDERSON 2007 PASSIVE TORQUE ANGLE CURVE TESTING   "<<endl;

    double curveSign = 1.0;



    for(int z = 0; z<2; ++z){

     if(z == 0){
      curveSign = 1.0;
      //cout <<"    TESTING SIDE 1"<<endl;
     }else{
      curveSign = -1.0;
      //cout <<"    TESTING SIDE 2"<<endl;

     }
     SmoothSegmentedFunction andersonTpCurve = SmoothSegmentedFunction();
     TorqueMuscleFunctionFactory::
       createAnderson2007PassiveTorqueAngleCurve(
            scale,
            c1,
            b1*curveSign,
            k1,
            b2*curveSign,
            k2,
            "test_passiveTorqueAngleCurve",
            andersonTpCurve);

     RigidBodyDynamics::Math::VectorNd curveDomain 
      = andersonTpCurve.getCurveDomain();

     double angleMin = curveDomain[0];
     double angleMax = curveDomain[1];

     RigidBodyDynamics::Math::MatrixNd andersonTpCurveSample
            = andersonTpCurve.calcSampledCurve( 6,
                        angleMin-0.1,
                        angleMax+0.1);

     //cout << "   Keypoint Testing" << endl;

     double tauMin = andersonTpCurve.calcValue(angleMin);
     double tauMax = andersonTpCurve.calcValue(angleMax);
     double tauMinAngle = angleMin;

     if(tauMin > tauMax){
      double tmp  = tauMin;
      tauMin   = tauMax;
      tauMax   = tmp;
      tauMinAngle = angleMax;
     }

     CHECK( abs(tauMin) < TOL_SMALL);
     CHECK( abs(tauMax - 1.0) < TOL_SMALL);
     CHECK( abs(andersonTpCurve.calcDerivative(tauMinAngle,1)) < TOL_SMALL);

     //cout << "   passed " << endl;

     //cout << "   Continuity and Smoothness Testing " << endl;
     bool areCurveDerivativesGood = 
      areCurveDerivativesCloseToNumericDerivatives( 
       andersonTpCurve,  
       andersonTpCurveSample, 
       TOL_DX);
     CHECK(areCurveDerivativesGood);

     bool curveIsContinuous = isCurveC2Continuous(andersonTpCurve,  
                                                  andersonTpCurveSample,
                                                  TOL_BIG);
     CHECK(curveIsContinuous);

     bool curveIsMonotonic = isCurveMontonic(andersonTpCurveSample);
     CHECK(curveIsMonotonic);
     //cout << "   passed " << endl;


    }

    SmoothSegmentedFunction andersonTpCurve = SmoothSegmentedFunction();
    TorqueMuscleFunctionFactory::
      createAnderson2007PassiveTorqueAngleCurve(
        scale,
        c1,
        b1,
        k1,
        b2,
        k2,
        "test_passiveTorqueAngleCurve",
        andersonTpCurve);

    if(FLAG_PLOT_CURVES){
     andersonTpCurve.printCurveToCSVFile(
        FILE_PATH,
        "anderson2007PassiveTorqueAngleCurve",
        andersonTpCurve.getCurveDomain()[0]-0.1,
        andersonTpCurve.getCurveDomain()[1]+0.1);
    }

}

TEST(Millard2016ActiveTorqueVelocityCurve)
{
    double subjectWeight    = 75.0*9.81;
    double subjectHeight    = 1.75;
    double scale      = subjectHeight*subjectWeight; 

    //These parameters are taken from table 3 for hip extension for
    //men between the ages of 18-25
    double c1      = 0.161; //normalized maximum hip joint torque
    double c2      = 0.958; // pi/(theta_max - theta_min)
    double c3      = 0.932; //theta_max_iso_torque
    double c4      = 1.578; //omega_1: angular velocity at 75% tq_iso_max
    double c5      = 3.190; //omega_2: angular velocity at 50% tq_iso_max
    double c6      = 0.242; //E, where eccentric slope = (1+E)*concentricSlope
                //Passive torque angle curve parameters
    double b1      =-1.210; // torque_passive = b1*exp(k1*theta) 
    double k1      =-6.351; //        +b2*exp(k2*theta)
    double b2      = 0.476;            
    double k2      = 5.910;

    //cout <<endl;
    //cout <<endl;
    //cout <<"**************************************************"<<endl;
    //cout <<"ANDERSON 2007 ACTIVE TORQUE VELOCITY CURVE TESTING"<<endl;

    double minEccentricMultiplier = 1.1;
    double maxEccentricMultiplier = 1.4;

    SmoothSegmentedFunction andersonTvCurve = SmoothSegmentedFunction();
    TorqueMuscleFunctionFactory::
    createAnderson2007ActiveTorqueVelocityCurve(
     c4,c5,c6,minEccentricMultiplier,maxEccentricMultiplier,
     "test_anderson2007ActiveTorqueVelocityCurve",
     andersonTvCurve);

    RigidBodyDynamics::Math::VectorNd curveDomain 
     = andersonTvCurve.getCurveDomain();

    double angularVelocityMin = curveDomain[0];
    double angularVelocityMax = curveDomain[1];


    RigidBodyDynamics::Math::MatrixNd andersonTvCurveSample
        = andersonTvCurve.calcSampledCurve( 6,
                       angularVelocityMin-0.1,
                       angularVelocityMax+0.1);

    //cout << "   Keypoint Testing" << endl;

    CHECK(abs(andersonTvCurve.calcValue(0) - 1.0)        < TOL_SMALL);
    CHECK(abs(andersonTvCurve.calcValue(c4) - 0.75)    < TOL_BIG);
    CHECK(abs(andersonTvCurve.calcValue(c5) - 0.5)     < TOL_BIG);
    CHECK(abs(andersonTvCurve.calcValue(angularVelocityMax))   < TOL_BIG);

    double maxTv = andersonTvCurve.calcValue(angularVelocityMin);
    CHECK(maxTv >= minEccentricMultiplier);
    CHECK(maxTv <= maxEccentricMultiplier);

    CHECK(abs(andersonTvCurve.calcDerivative(angularVelocityMin,1))<TOL_SMALL);
    CHECK(abs(andersonTvCurve.calcDerivative(angularVelocityMax,1))<TOL_SMALL);

    //cout << "   passed " << endl;
    //cout << "    Continuity and Smoothness Testing" << endl;

    bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      andersonTvCurve,  
      andersonTvCurveSample, 
      TOL_DX);

    CHECK(areCurveDerivativesGood);

    bool curveIsContinuous = isCurveC2Continuous( andersonTvCurve,
                                                  andersonTvCurveSample,
                                                  TOL_BIG);
    CHECK(curveIsContinuous);

    bool curveIsMonotonic = isCurveMontonic(andersonTvCurveSample);
    CHECK(curveIsMonotonic);


    if(FLAG_PLOT_CURVES){
     andersonTvCurve.printCurveToCSVFile(
        FILE_PATH,
        "anderson2007ActiveTorqueVelocityCurve",
        angularVelocityMin,
        angularVelocityMax);
    }

}


