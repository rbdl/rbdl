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
      Math::VectorNd &q,
      Math::VectorNd &qdot,
      Math::Vector3d &pointOnGroundPlane,
      Math::Vector3d &groundPlaneNormal,
      FootPlacementEstimatorInfo &fpeInfo,      
      bool evaluate_derivatives,
      bool update_kinematics)
{

  //Allocate local memory
  double gNorm;
  double mass;

  Vector3d rPC0;
  Matrix3d rPC0x;
  Vector3d riC0;
  Matrix3d riC0x;

  SpatialRigidBodyInertia IC0 (0., Vector3d (0.,0.,0.), Matrix3d::Zero(3,3));

  //Get the normal vector
  gNorm      = model.gravity.norm();
  fpeInfo.k  = model.gravity * (-1.0 / model.gravity.norm());

  //Make sure the plane normal is parallel to gravity. While it is possible
  //to derive an FPE for a tilted plane, this implementation assumes a plane
  //that is perpendicular to gravity.
  assert((fpeInfo.k.dot(groundPlaneNormal)-1.0)
         <= std::numeric_limits<double>::epsilon()*10);

  //Evaluate the position and velocity of the COM and the whole body angular
  //velocity about the COM

  CalcCenterOfMass( model,q,qdot,NULL,mass,fpeInfo.r0C0,&fpeInfo.v0C0,NULL,
                    &fpeInfo.HC0,NULL,update_kinematics);

  //Evaluate the inertia matrix about the whole-body-com resolved in the
  //coordinates of the base frame. Note: the numerical test code does not
  //test this code adequately. It is possible there is a mistake in here.
  fpeInfo.JC0.setZero();
  for (size_t i = 1; i < model.mBodies.size(); i++) {
    //Vector from the COM of body i to the COM of the entire system
    if(model.mBodies[i].mIsVirtual == false){
      riC0 = fpeInfo.r0C0
           - model.X_base[i].r
           - model.X_base[i].E.transpose()*model.mBodies[i].mCenterOfMass;
      riC0x =VectorCrossMatrix(riC0);

      fpeInfo.JC0 = fpeInfo.JC0
          + (model.X_base[i].E.transpose()
             * model.mBodies[i].mInertia
             * model.X_base[i].E)
          + model.mBodies[i].mMass*(riC0x*riC0x.transpose());
    }
  }

  //Solve for the average whole-body angular velocity about the COM.
  fpeInfo.w0C0 = fpeInfo.JC0.householderQr().solve (fpeInfo.HC0);

  //Evaluate the ground projection of the COM: this is one point that is
  //guaranteed to be in the u-k plane. Note r0P0 corresponds to point 'P'
  //in Fig. 6 of Millard et al.
  fpeInfo.r0P0 = fpeInfo.r0C0 - 
               ((fpeInfo.r0C0-pointOnGroundPlane).dot(fpeInfo.k))*fpeInfo.k;

  //Solve for the whole body inertia, angular momentum, and average angular
  //velocity about the center of mass ground contact point.
  rPC0          = fpeInfo.r0C0 - fpeInfo.r0P0;
  rPC0x         = VectorCrossMatrix(rPC0);
  fpeInfo.JP0   = fpeInfo.JC0 + mass*(rPC0x*rPC0x.transpose());
  fpeInfo.HP0   = fpeInfo.HC0 + rPC0x * (mass * fpeInfo.v0C0);
  fpeInfo.w0P0  = fpeInfo.JP0.householderQr().solve (fpeInfo.HP0);

  //Form the remaining direction vectors of the u-v-k frame
  fpeInfo.n = fpeInfo.HP0 - (fpeInfo.HP0.dot(fpeInfo.k))*fpeInfo.k;
  fpeInfo.n.normalize();
  fpeInfo.u = fpeInfo.n.cross(fpeInfo.k);

  //Project the 3D state onto the u-k plane
  fpeInfo.nJC0n = fpeInfo.n.transpose()*fpeInfo.JC0*fpeInfo.n;
  fpeInfo.v0C0u = fpeInfo.u.transpose()*fpeInfo.v0C0;
  fpeInfo.v0C0k = fpeInfo.k.transpose()*fpeInfo.v0C0;
  fpeInfo.w0C0n = fpeInfo.n*fpeInfo.w0C0;
  fpeInfo.h = fpeInfo.k.dot(r0C0-pointOnGroundPlane);
  fpeInfo.projectionError = fpeInfo.k.dot(fpeInfo.w0C0)
                    /max(fpeInfo.w0C0.norm(),TOLERANCE);


}
