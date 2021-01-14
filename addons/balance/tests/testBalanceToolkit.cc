/*
 * 
 * Copyright (c) 2020 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

//==============================================================================
// INCLUDES
//==============================================================================

#include <UnitTest++.h>
#include "rbdl/rbdl.h"
#include "balance.h"

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

static double   TEST_PREC_BIG    = std::numeric_limits<double>::epsilon()*1e6;
static double   TEST_PREC_MEDIUM = std::numeric_limits<double>::epsilon()*1e4;
static double   TEST_PREC_SMALL  = std::numeric_limits<double>::epsilon()*1e1;
static double   TEST_TOLERANCE   = std::numeric_limits<double>::epsilon()*1e2;

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
TEST(NumericalTestFootPlacementEstimator){



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
  model.gravity = Vector3d(0.,0.,-9.81);


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
  BalanceToolkit::FootPlacementEstimatorInfo fpeInfo;

  BalanceToolkit::CalculateFootPlacementEstimator(model,q,qd,pointOnGroundPlane,
    groundPlaneNormal,fpeInfo,evaluate_derivatives,update_kinematics);
    
  //Compare the numerical solution to the one produced from the matlab 
  //implementation made for Sloot et al. (calc3DFootPlacementEstimatorInfo.m)
  CHECK_ARRAY_CLOSE(fpeInfo.k.data(),      k.data(),  3, TEST_PREC_SMALL);
  CHECK_ARRAY_CLOSE(fpeInfo.r0C0.data(),r0C0.data(),  3, TEST_PREC_SMALL);
  CHECK_ARRAY_CLOSE(fpeInfo.v0C0.data(),v0C0.data(),  3, TEST_PREC_SMALL);
  CHECK_ARRAY_CLOSE(fpeInfo.HC0.data(),  HC0.data(),  3, TEST_PREC_SMALL);
  CHECK_ARRAY_CLOSE(fpeInfo.JC0.data(),  JC0.data(),  9, TEST_PREC_SMALL);
  CHECK_ARRAY_CLOSE(fpeInfo.w0C0.data(),w0C0.data(),  3, TEST_PREC_SMALL);
  CHECK_ARRAY_CLOSE(fpeInfo.r0P0.data(),r0P0.data(),  3, TEST_PREC_SMALL);
  CHECK_ARRAY_CLOSE(fpeInfo.HP0.data(),  HP0.data(),  3, TEST_PREC_SMALL);
  CHECK_ARRAY_CLOSE(fpeInfo.n.data(),      n.data(),  3, TEST_PREC_SMALL);

  CHECK( fabs(fpeInfo.h - h ) <= TEST_PREC_SMALL );
  CHECK( fabs(fpeInfo.v0C0u - v0C0u ) <= TEST_PREC_SMALL );
  CHECK( fabs(fpeInfo.v0C0k - v0C0k ) <= TEST_PREC_SMALL );
  CHECK( fabs(fpeInfo.nJC0n - J     ) <= TEST_PREC_SMALL );


  CHECK( fabs(fpeInfo.f)        <= TEST_TOLERANCE       );
  CHECK( fabs(fpeInfo.phi-phi ) <= TEST_TOLERANCE*10.0  );
  CHECK( fabs(fpeInfo.projectionError-projectionError) <= TEST_PREC_SMALL );
  CHECK( fabs(fpeInfo.E - E) < TEST_TOLERANCE*10.0);

  CHECK_ARRAY_CLOSE(fpeInfo.r0F0.data(), r0F0.data(), 3, TEST_PREC_MEDIUM );


  //Derivative check

  CHECK( fabs( fpeInfo.Df_Dphi     - Df_Dphi    ) <= TEST_PREC_BIG);
  CHECK( fabs( fpeInfo.Df_Dw0C0n   - Df_Dw0C0n  ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Df_Dh       - Df_Dh      ) <= TEST_PREC_BIG);
  CHECK( fabs( fpeInfo.Df_Dv0C0u   - Df_Dv0C0u  ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Df_Dv0C0k   - Df_Dv0C0k  ) <= TEST_PREC_BIG);
  CHECK( fabs( fpeInfo.Df_DnJC0n   - Df_DJ      ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Df_Dm       - Df_Dm      ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Df_Dg       - Df_Dg      ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Ds_Dl       - Ds_Dl      ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Ds_DnJC0n   - Ds_DJ      ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Ds_DE       - Ds_DE      ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Ds_Dv0C0u   - Ds_Dv0C0u  ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Ds_Dv0C0k   - Ds_Dv0C0k  ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Ds_Dw0C0n   - Ds_Dw0C0n  ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Dphi_Dl     - Dphi_Dl    ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Dphi_DnJC0n - Dphi_DJ    ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Dphi_DE     - Dphi_DE    ) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Dphi_Dv0C0u - Dphi_Dv0C0u) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Dphi_Dv0C0k - Dphi_Dv0C0k) <= TEST_PREC_MEDIUM);
  CHECK( fabs( fpeInfo.Dphi_Dw0C0n - Dphi_Dw0C0n) <= TEST_PREC_MEDIUM);


}

int main (int argc, char *argv[])
{
    return UnitTest::RunAllTests ();
}
