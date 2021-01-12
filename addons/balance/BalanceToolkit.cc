/*
 * 
 * Copyright (c) 2020 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */


#include "BalanceToolkit.h"
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>
#include <sstream>
#include <rbdl/rbdl_errors.h>
#include <rbdl/rbdl_utils.h>

//#include <cassert>
using namespace RigidBodyDynamics::Addons::Balance;

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Utils;

static double   TOLERANCE  = std::numeric_limits<double>::epsilon()*1e2;
static int      MAXITER       = 12;


//=============================================================================
// FUNCTIONS
//=============================================================================
void BalanceToolkit::CalculateFootPlacementEstimator(
      Model &model,
      Math::VectorNd& q,
      Math::VectorNd& qdot,
      Math::Vector3d& pointOnGroundPlane,
      Math::Vector3d& groundPlaneNormal,
      FootPlacementEstimatorInfo& fpeInfo,      
      bool evaluate_derivatives,
      bool update_kinematics)
{

  Vector3d r0C0;
  Vector3d v0C0;
  Vector3d HC0;

  //Get the normal vector
  double gNorm  = model.gravity.norm();
  fpeInfo.k     = model.gravity * (-1.0 / model.gravity.norm());

  //Make sure the plane normal is parallel to gravity. While it is possible
  //to derive an FPE for a tilted plane, this implementation assumes a plane
  //that is perpendicular to gravity.
  assert((fpeInfo.k.dot(groundPlaneNormal)-1.0)
         <= std::numeric_limits<double>::epsilon()*10);

  //Evaluate the position and velocity of the COM and the whole body angular
  //velocity about the COM
  double mass;
  CalcCenterOfMass( model,q,qdot,NULL,mass,r0C0,&v0C0,NULL,&HC0,NULL,
                    update_kinematics);
  //Compute the moment of inertia about the center of mass
  SpatialRigidBodyInertia Ic (0., Vector3d (0., 0., 0.),
                                 Matrix3d::Zero(3,3));
  for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int lambda = model.lambda[i];
    if (lambda != 0) {
      model.Ic[lambda] = model.Ic[lambda]
        + model.X_lambda[i].applyTranspose (model.Ic[i]);
    } else {
      Ic = Ic + model.X_lambda[i].applyTranspose (model.Ic[i]);
    }
  }
  fpeInfo.IC0  = Ic.toMatrix();
  fpeInfo.w0C0 = fpeInfo.IC0.householderQr().solve (HC0);

  //Evaluate the ground projection of the COM: this is one point that is
  //guaranteed to be in the u-k plane
  fpeInfo.r0G0 = r0C0 - ((r0C0-pointOnGroundPlane).dot(fpeInfo.k))*fpeInfo.k;


  //Solve for the whole body inertia, angular momentum, and average angular
  //velocity about the center of mass ground contact point.
  Vector3d rGC0 = r0C0 - r0G0;
  Matrix3d rGC0x = VectorCrossMatrix(rGC0);
  fpeInfo.IG0 = fpeInfo.IC0 - mass*(rGC0x*rGC0x);
  fpeInfo.HG0 = fpeInfo.HC0 + rGC0x * (mass * v0C0);
  fpeInfo.w0G0 = fpeInfo.IG0.householderQr().solve (HG0);

}
