/*
 * 
 * Copyright (c) 2020 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

//==============================================================================
// INCLUDES
//==============================================================================

#define CATCH_CONFIG_MAIN
#include "rbdl/rbdl.h"

#include "../balance.h"
#include "rbdl_tests.h"

#include <ctime>
#include <string>
#include <stdio.h>
#include <exception>
#include <cassert>
#include <fstream>

using namespace RigidBodyDynamics::Addons::Balance;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;
using namespace std;


static double   TEST_PREC_NEAR_EPS  = std::numeric_limits<double>::epsilon()*1e1;

static double   TOLERANCE        = (M_PI*0.25)*pow(2,-15);
static double   TOLERANCE_MEDIUM = TOLERANCE*10;
static double   TOLERANCE_BIG    = TOLERANCE*1000;

// @author Millard 
// @date 13 January 2020
//
// This test compares the output from the RBDL implementation of the 3DFPE to
// the output produced by a Matlab implementation that has been used in the 
// publication of Sloot et al. As tests go this is weak test, because after all
// the matlab implementation could also have errors in it. However, presently
// I don't have the time to write a test that evaluates the invariant 
// mathematical properties of the FPE solution to a random set of data.
//
// Sloot LH, Millard M, Werner C, Mombaur K. Slow but Steady: Similar 
// Sit-to-Stand Balance at Seat-Off in Older vs. Younger Adults. Frontiers in 
// sports and active living. 2020;2.
TEST_CASE(__FILE__"_NumericalTestFootPlacementEstimator",""){



  //This data comes from 
  // 
  // Sloot LH, Millard M, Werner C, Mombaur K. Slow but Steady: Similar 
  // Sit-to-Stand Balance at Seat-Off in Older vs. Younger Adults. Frontiers in 
  // sports and active living. 2020;2.
  //
  //  participant E01, 
  //  trial sts_0002_Chest.c3d, 
  //  time 1.466666666666667e+00
  //
  // The scripts that write this data to file appear in Millard's RhodeCode
  // repository for
  //   FrontiersBalance2020/code/matlab
  //      main_ProcessBalanceData.m
  //      process3DFootPlacementEstimator.m (lines 189-208)
  //
  //the first data point at which all inputs to the FPE are non-zero and the
  //center of mass has a speed that is greater than 30 cm/s 
  //


  //Not a typo: E01's mass really is 100kg even
  double mass =  1.000000000000000e+02; 

  Vector3d r0C0 = Vector3d(
    6.003900000000000e-01,
    2.167700000000000e-01,
    5.957600000000000e-01);
  Vector3d v0C0 = Vector3d(
    3.020800000000000e-01,
    2.088000000000000e-02,
    2.757000000000000e-02);
  Matrix3d JC0 = Matrix3d(
    5.165410000000000e+00,  5.842000000000000e-02,  4.450500000000000e-01, 
    5.842000000000000e-02,  6.110220000000000e+00,  -1.848600000000000e-01, 
    4.450500000000000e-01,  -1.848600000000000e-01,  2.099090000000000e+00);

  Vector3d r0P0 = Vector3d(
    6.003900000000000e-01, 
    2.167700000000000e-01, 
    0.000000000000000e+00);
  Vector3d v0P0 = Vector3d(
    -3.212179353090099e-02, 
     5.833672380300754e-01, 
     1.784919742991831e-02);
  Vector3d HP0 = Vector3d(
     -1.263996880000000e+00, 
     2.426477808000000e+01, 
     -8.467000000000000e-02);


  double f    = -8.670397733112623e-12;
  double phi  = 1.551340426396078e-01;
  double projectionError = 3.484674002384912e-03;

  Vector3d HC0 = Vector3d(
    -2.005000000000000e-02, 
     6.268060000000000e+00, 
    -8.467000000000000e-02);
  Vector3d w0C0 = Vector3d(
    -2.019572919578354e-02,
     1.027672539346732e+00,
     5.444914458275742e-02);    

  Vector3d n = Vector3d(
   -5.202130399910231e-02, 
   9.986459752736367e-01, 
   0.000000000000000e+00); 
  Vector3d u = Vector3d(
   9.986459752736367e-01, 
   5.202130399910231e-02, 
   0.000000000000000e+00);
  Vector3d k = Vector3d(
   -0.000000000000000e+00, 
   -0.000000000000000e+00, 
   1.000000000000000e+00);  

  Vector3d r0F0 = Vector3d(
    6.934351408607322e-01, 
    2.216168923704711e-01, 
    0.000000000000000e+00);

  double h            =  5.957600000000000e-01;
  double v0C0u        =  3.027571810381615e-01;
  double v0C0k        =  2.757000000000000e-02;
  double J            =  6.101593200827201e+00;
  double Df_Dphi      = -1.824211348591032e+02; 
  double Df_Dw0C0n    =  6.890339351758533e+00; 
  double Df_Dh        = -2.847069951699541e+01; 
  double Df_Dv0C0u    =  6.727732310385989e+01; 
  double Df_Dv0C0k    =  1.052154468083766e+01; 
  double Df_DJ        =  8.335241002082351e-01; 
  double Df_Dm        = -5.085824982564849e-02; 
  double Df_Dg        = -1.413732690973843e+00; 
  double Ds_Dl        = -9.411122195708749e-02; 
  double Ds_DJ        =  2.788743191588045e-03; 
  double Ds_DE        =  6.597315882691825e-03; 
  double Ds_Dv0C0u    =  2.250914841062020e-01; 
  double Ds_Dv0C0k    =  3.520220481488790e-02; 
  double Ds_Dw0C0n    =  2.305318700460767e-02; 
  double Dphi_Dl      = -1.541969905085013e-01; 
  double Dphi_DJ      =  4.569229880364602e-03; 
  double Dphi_DE      =  1.080940437697085e-02; 
  double Dphi_Dv0C0u  =  3.688022396956522e-01; 
  double Dphi_Dv0C0k  =  5.767722412736988e-02; 
  double Dphi_Dw0C0n  =  3.777160665665429e-02; 
  double E            =  5.915445196560693e+02;

  RigidBodyDynamics::Model model;
  double g = 9.81;
  model.gravity = Vector3d(0.,0.,-g);


  Body body1 = Body( mass,Vector3d(0., 0., 0.),JC0);

  //Q order: Tx, Ty, Tz, Rx, Ry, Rz
  Joint jointFloatingBase = Joint(
    SpatialVector( 0., 0., 0., 1., 0., 0.),
    SpatialVector( 0., 0., 0., 0., 1., 0.),
    SpatialVector( 0., 0., 0., 0., 0., 1.),
    SpatialVector( 1., 0., 0., 0., 0., 0.),
    SpatialVector( 0., 1., 0., 0., 0., 0.),
    SpatialVector( 0., 0., 1., 0., 0., 0.));

  unsigned int idB1 = model.AddBody(0,Xtrans(Vector3d(0., 0, 0. )),
                                    jointFloatingBase, body1, "body1");

  VectorNd q  = VectorNd::Zero(model.dof_count);
  VectorNd qd = VectorNd::Zero(model.dof_count);
   
  q[0] = r0C0[0];
  q[1] = r0C0[1]; 
  q[2] = r0C0[2]; 
  q[3] = 0.;
  q[4] = 0.; 
  q[5] = 0.; 

  qd[0] = v0C0[0]; 
  qd[1] = v0C0[1]; 
  qd[2] = v0C0[2]; 
  qd[3] = w0C0[0];
  qd[4] = w0C0[1]; 
  qd[5] = w0C0[2]; 

  Vector3d pointOnGroundPlane = Vector3d(0.,0.,0.);
  Vector3d groundPlaneNormal  = Vector3d(0.,0.,1.);

  bool evaluate_derivatives = true;
  bool update_kinematics = true;
  FootPlacementEstimatorInfo fpeInfo;
  double omegaSmall = 1e-6;

  BalanceToolkit::CalculateFootPlacementEstimator(model,q,qd,pointOnGroundPlane,
    groundPlaneNormal,fpeInfo,omegaSmall,evaluate_derivatives,update_kinematics);
    
  //Compare the numerical solution to the one produced from the matlab 
  //implementation made for Sloot et al. (calc3DFootPlacementEstimatorInfo.m)
  CHECK_THAT(fpeInfo.k,    AllCloseVector(    k,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT(fpeInfo.r0C0, AllCloseVector( r0C0,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT(fpeInfo.v0C0, AllCloseVector( v0C0,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT( fpeInfo.HC0, AllCloseVector(  HC0,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT( fpeInfo.JC0, AllCloseMatrix(  JC0,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT(fpeInfo.w0C0, AllCloseVector( w0C0,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT(fpeInfo.r0P0, AllCloseVector( r0P0,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT( fpeInfo.HP0, AllCloseVector(  HP0,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT(   fpeInfo.n, AllCloseVector(    n,  TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));

  REQUIRE( fabs(fpeInfo.h - h )         <= TEST_PREC_NEAR_EPS );
  REQUIRE( fabs(fpeInfo.v0C0u - v0C0u ) <= TEST_PREC_NEAR_EPS );
  REQUIRE( fabs(fpeInfo.v0C0k - v0C0k ) <= TEST_PREC_NEAR_EPS );
  REQUIRE( fabs(fpeInfo.nJC0n - J     ) <= TEST_PREC_NEAR_EPS );


  REQUIRE( fabs(fpeInfo.f)        <= TOLERANCE*(mass*g)   );
  REQUIRE( fabs(fpeInfo.phi-phi ) <= TOLERANCE            );
  REQUIRE( fabs(fpeInfo.projectionError-projectionError) <= TEST_PREC_NEAR_EPS );
  REQUIRE( fabs(fpeInfo.E - E) < TOLERANCE*(mass*g));

  CHECK_THAT(fpeInfo.r0F0, AllCloseVector(r0F0, TOLERANCE_MEDIUM, TOLERANCE_MEDIUM) );

  //Derivative check
  REQUIRE( fabs( fpeInfo.Df_Dphi     - Df_Dphi    ) <= TOLERANCE_BIG);
  REQUIRE( fabs( fpeInfo.Df_Dw0C0n   - Df_Dw0C0n  ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Df_Dh       - Df_Dh      ) <= TOLERANCE_BIG);
  REQUIRE( fabs( fpeInfo.Df_Dv0C0u   - Df_Dv0C0u  ) <= TOLERANCE_BIG);
  REQUIRE( fabs( fpeInfo.Df_Dv0C0k   - Df_Dv0C0k  ) <= TOLERANCE_BIG);
  REQUIRE( fabs( fpeInfo.Df_DnJC0n   - Df_DJ      ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Df_Dm       - Df_Dm      ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Df_Dg       - Df_Dg      ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Ds_Dl       - Ds_Dl      ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Ds_DnJC0n   - Ds_DJ      ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Ds_DE       - Ds_DE      ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Ds_Dv0C0u   - Ds_Dv0C0u  ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Ds_Dv0C0k   - Ds_Dv0C0k  ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Ds_Dw0C0n   - Ds_Dw0C0n  ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Dphi_Dl     - Dphi_Dl    ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Dphi_DnJC0n - Dphi_DJ    ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Dphi_DE     - Dphi_DE    ) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Dphi_Dv0C0u - Dphi_Dv0C0u) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Dphi_Dv0C0k - Dphi_Dv0C0k) <= TOLERANCE_MEDIUM);
  REQUIRE( fabs( fpeInfo.Dphi_Dw0C0n - Dphi_Dw0C0n) <= TOLERANCE_MEDIUM);


}

TEST_CASE(__FILE__"_InertiaCalculations","")
{

  //Approach:
  //Make a system that consists of two bodies with the mass and inertia
  //consistent with two identical cylinders. Place and orient them in
  //space, end-to-end. The inertia JC0 returned by CalcFootPlacementEstimator
  //should be consistent with the equivalent single cylinder.

  double Tx = 23.0/22.;
  double Ty = 1.0/9.0 ;
  double Tz = 6.0/11.0;

  double Rx = M_PI/6.;
  double Ry = M_PI/12.;
  double Rz = M_PI/24.;

  double mass0 = M_PI;
  double mass1 = mass0*0.5;
  double h0 = 3.0;
  double h1 = h0*0.5;
  double r  = 0.75;

  double Ixx0 = (1./12.)*mass0*(3.*r*r +h0*h0);
  double Iyy0 = (1./12.)*mass0*(3.*r*r +h0*h0);
  double Izz0 = (1./2. )*mass0*(r*r);

  double Ixx1 = (1./12.)*(mass1)*(3.*r*r +h1*h1);
  double Iyy1 = (1./12.)*(mass1)*(3.*r*r +h1*h1);
  double Izz1 = (1./2. )*(mass1)*(r*r);


  Matrix3d I0 = Matrix3d(Ixx0,    0,    0,
                            0, Iyy0,    0,
                            0,    0, Izz0);

  Matrix3d I1 = Matrix3d(Ixx1,    0,    0,
                            0, Iyy1,    0,
                            0,    0, Izz1);

  Body cylinderUpper = Body( mass1,Vector3d(0., 0., h1*0.5),I1);
  Body cylinderLower = Body( mass1,Vector3d(0., 0.,-h1*0.5),I1);

  RigidBodyDynamics::Model model;
  model.gravity = Vector3d(0.,0.,-9.81);
  Joint jointFloatingBase = Joint(
    SpatialVector( 0., 0., 0., 1., 0., 0.),
    SpatialVector( 0., 0., 0., 0., 1., 0.),
    SpatialVector( 0., 0., 0., 0., 0., 1.),
    SpatialVector( 1., 0., 0., 0., 0., 0.),
    SpatialVector( 0., 1., 0., 0., 0., 0.),
    SpatialVector( 0., 0., 1., 0., 0., 0.));

  unsigned int idB1 = model.AddBody(0,Xtrans(Vector3d(0., 0, 0. )),
                        jointFloatingBase, cylinderUpper, "cylinderUpper");
  unsigned int idB2 = model.AddBody(0,Xtrans(Vector3d(0., 0, 0. )),
                        jointFloatingBase, cylinderLower, "cylinderLower");

  VectorNd q  = VectorNd::Zero(model.dof_count);
  VectorNd qd = VectorNd::Zero(model.dof_count);


  q[0]  = Tx;
  q[1]  = Ty;
  q[2]  = Tz;
  q[3]  = Rx;
  q[4]  = Ry;
  q[5]  = Rz;

  q[6]  = Tx;
  q[7]  = Ty;
  q[8]  = Tz;
  q[9]  = Rx;
  q[10] = Ry;
  q[11] = Rz;

  //By construction this is the center of mass of the two cylinder system
  Vector3d r0C0 = Vector3d(Tx,Ty,Tz);

  Vector3d pointOnGroundPlane  = Vector3d(0.,0.,0.);
  Vector3d groundPlaneNormal   = Vector3d(0.,0.,1.);


  bool evaluate_derivatives = true;
  bool update_kinematics    = true;

  FootPlacementEstimatorInfo fpeInfo;
  double omegaSmall = 1e-6;
  BalanceToolkit::CalculateFootPlacementEstimator(model,q,qd,pointOnGroundPlane,
    groundPlaneNormal,fpeInfo,omegaSmall,evaluate_derivatives,update_kinematics);

  Matrix3d I0Test =
      model.X_base[idB1].E*fpeInfo.JC0*model.X_base[idB1].E.transpose();

  CHECK_THAT(I0Test, AllCloseMatrix(          I0, TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));
  CHECK_THAT(  r0C0, AllCloseVector(fpeInfo.r0C0, TEST_PREC_NEAR_EPS, TEST_PREC_NEAR_EPS));


}


