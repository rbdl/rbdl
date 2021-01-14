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

static double       TOLERANCE  = std::numeric_limits<double>::epsilon()*1e2;
static unsigned int MAX_BISECTION_ITERATIONS    = 5;
static unsigned int MAX_NEWTON_ITERATIONS       = 12;
static double MAX_NEWTON_STEP_SIZE = M_PI*0.125;


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
  double g; //gravity
  double m; //whole body mass

  //Variables needed to calculate the single body equivalent model and
  //project its state onto the u-v plane.
  Vector3d rPC0;
  Matrix3d rPC0x;
  Vector3d riC0;
  Matrix3d riC0x;
  SpatialRigidBodyInertia IC0 (0., Vector3d (0.,0.,0.), Matrix3d::Zero(3,3));

  //Variables needed for the root solving method
  double cosphi;
  double cos2phi;
  double sinphi;
  double tanphi;
  double h2;

  double t0; //temporary working variable for squared quantities
  double t1;
  double fLeft;
  double phiLeft;

  double fRight;
  double phiRight;

  double phiBest;
  double fBest;
  double delta;

  //Get the normal vector
  g          = model.gravity.norm();
  fpeInfo.k  = model.gravity * (-1.0 / g);

  //Make sure the plane normal is parallel to gravity. While it is possible
  //to derive an FPE for a tilted plane, this implementation assumes a plane
  //that is perpendicular to gravity.
  assert((fpeInfo.k.dot(groundPlaneNormal)-1.0)
         <= std::numeric_limits<double>::epsilon()*10);

  //Evaluate the position and velocity of the COM and the whole body angular
  //velocity about the COM

  CalcCenterOfMass( model,q,qdot,NULL,m,fpeInfo.r0C0,&fpeInfo.v0C0,NULL,
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
  fpeInfo.JP0   = fpeInfo.JC0 + m*(rPC0x*rPC0x.transpose());
  fpeInfo.HP0   = fpeInfo.HC0 + rPC0x * (m * fpeInfo.v0C0);
  fpeInfo.w0P0  = fpeInfo.JP0.householderQr().solve (fpeInfo.HP0);

  //Form the remaining direction vectors of the u-v-k frame
  fpeInfo.n = fpeInfo.HP0 - (fpeInfo.HP0.dot(fpeInfo.k))*fpeInfo.k;
  fpeInfo.n.normalize();
  fpeInfo.u = fpeInfo.n.cross(fpeInfo.k);

  assert( fabs(fpeInfo.u.norm()-1) < TOLERANCE);

  //Project the 3D state onto the u-k plane
  fpeInfo.nJC0n = fpeInfo.n.dot(fpeInfo.JC0*fpeInfo.n);
  fpeInfo.v0C0u = fpeInfo.u.dot(fpeInfo.v0C0);
  fpeInfo.v0C0k = fpeInfo.k.dot(fpeInfo.v0C0);
  fpeInfo.w0C0n = fpeInfo.n.dot(fpeInfo.w0C0);  
  fpeInfo.h     = fpeInfo.k.dot(fpeInfo.r0C0-pointOnGroundPlane);

  fpeInfo.projectionError = fabs(fpeInfo.k.dot(fpeInfo.HP0))
                            /max(fpeInfo.HP0.norm(),TOLERANCE);

  fpeInfo.f = TOLERANCE*10;
  fpeInfo.iterations = 0;

  //----------------------------------------------------------------------------
  //I. Bisection method initialization
  //----------------------------------------------------------------------------
  //Do not touch: these values will allow the bisection method to find solutions
  //              of phi within [0,M_PI*0.5].
    phiBest     = M_PI * 0.25;

    //Use the bisection method to get close to the solution
    cosphi    = cos(phiBest);
    cos2phi   = cosphi*cosphi;
    sinphi    = sin(phiBest);
    h2        = fpeInfo.h*fpeInfo.h;

    //numerator of Eqn. 45 of Millard et al.
    t0        =(cos2phi*fpeInfo.w0C0n*fpeInfo.nJC0n
                + cosphi*fpeInfo.h*m*(
                    sinphi*fpeInfo.v0C0k + cosphi*fpeInfo.v0C0u));

    //Eqn. 45 of Millard et al., but note here the residual is assigned to f
    // and will not be 0 until the final solution.
    fBest     = (t0*t0) / (cos2phi*fpeInfo.nJC0n+h2*m)
                +2*(cosphi-1)*cosphi*g*fpeInfo.h*m;

    delta = phiBest*0.5;

  for(size_t i=0; i<MAX_BISECTION_ITERATIONS; ++i){
    //Evaluate to the left of the current solution
    phiLeft   = phiBest - delta;
    cosphi    = cos(phiLeft);
    cos2phi   = cosphi*cosphi;
    sinphi    = sin(phiLeft);
    h2        = fpeInfo.h*fpeInfo.h;
    t0        =(cos2phi*fpeInfo.w0C0n*fpeInfo.nJC0n
                + cosphi*fpeInfo.h*m*(
                    sinphi*fpeInfo.v0C0k + cosphi*fpeInfo.v0C0u));
    fLeft     = (t0*t0) / (cos2phi*fpeInfo.nJC0n+h2*m)
                +2*(cosphi-1)*cosphi*g*fpeInfo.h*m;

    //Evaluate to the right of the current solution
    phiRight  = phiBest + delta;
    cosphi    = cos(phiRight);
    cos2phi   = cosphi*cosphi;
    sinphi    = sin(phiRight);
    h2        = fpeInfo.h*fpeInfo.h;
    t0        =(cos2phi*fpeInfo.w0C0n*fpeInfo.nJC0n
                + cosphi*fpeInfo.h*m*(
                    sinphi*fpeInfo.v0C0k + cosphi*fpeInfo.v0C0u));
    fRight     = (t0*t0) / (cos2phi*fpeInfo.nJC0n+h2*m)
                +2*(cosphi-1)*cosphi*g*fpeInfo.h*m;

    //Update the current solution if the left/right offers a lower error
    if( fabs(fLeft) < fabs(fBest) && fabs(fLeft) <= fabs(fRight)){
      phiBest = phiLeft;
      fBest   = fLeft;
    }
    if( fabs(fRight) < fabs(fBest) && fabs(fRight) < fabs(fLeft)){
      phiBest = phiRight;
      fBest   = fRight;
    }
    delta = delta*0.5;
  }

   fpeInfo.f = fBest;
   fpeInfo.phi = phiBest;

  //----------------------------------------------------------------------------
  //II. Newton method: polish the root to tolerance
  //----------------------------------------------------------------------------
  fpeInfo.iterations = 0;

  while( (fabs(fpeInfo.f) > TOLERANCE)
        && (fpeInfo.iterations < MAX_NEWTON_ITERATIONS) ){

    //Use the bisection method to get close to the solution
    cosphi    = cos(fpeInfo.phi);
    cos2phi   = cosphi*cosphi;
    sinphi    = sin(fpeInfo.phi);
    h2        = fpeInfo.h*fpeInfo.h;

    //numerator of Eqn. 45 of Millard et al.
    t0        =(cos2phi*fpeInfo.w0C0n*fpeInfo.nJC0n
               + cosphi*fpeInfo.h*m*(
                   sinphi*fpeInfo.v0C0k + cosphi*fpeInfo.v0C0u));

    //Eqn. 45 of Millard et al.
    fpeInfo.f = (t0*t0) / (cos2phi*fpeInfo.nJC0n+h2*m)
               +2*(cosphi-1)*cosphi*g*fpeInfo.h*m;

    //Partial derivative of f w.r.t. phi (derived and simplified using Maxima)
    t0 = (cos2phi*fpeInfo.w0C0n*fpeInfo.nJC0n + cosphi*fpeInfo.h*m*(
           sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u));
    t1 = (cos2phi*fpeInfo.nJC0n+h2*m);

    fpeInfo.Df_Dphi =  (2*cosphi*sinphi*fpeInfo.nJC0n*(t0*t0))/(t1*t1)
       + ( 2*(cos2phi*fpeInfo.w0C0n*fpeInfo.nJC0n
              + cosphi*fpeInfo.h*m*(
                  sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
             )*((-2*cosphi*fpeInfo.w0C0n*sinphi*fpeInfo.nJC0n)
                 -fpeInfo.h*m*sinphi*(
                  sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
               + cosphi*fpeInfo.h*m*(
                  cosphi*fpeInfo.v0C0k-sinphi*fpeInfo.v0C0u)
                )
         )/(cos2phi*fpeInfo.nJC0n+h2*m)
       -2*cosphi*g*fpeInfo.h*m*sinphi
       -2*(cosphi-1)*g*fpeInfo.h*m*sinphi;

    //Only update the solution if we haven't met the desired tolerance. Why?
    //This keeps the values for phi, f, and Df_Dphi consistent
    if(fabs(fpeInfo.f) > TOLERANCE ){
      delta = -fpeInfo.f / fpeInfo.Df_Dphi;

      //If the step size is too large, limit it. This is necessary to keep
      //the Newton method in a region that is not wildly nonlinear.
      if( fabs(delta) > MAX_NEWTON_STEP_SIZE){
        delta = copysign(MAX_NEWTON_STEP_SIZE,delta);
      }

      fpeInfo.phi = fpeInfo.phi + delta;
      fpeInfo.iterations = fpeInfo.iterations+1;
    }
  }

  assert( fabs(fpeInfo.f) <= TOLERANCE);
  assert( fabs(fpeInfo.phi) >= -TOLERANCE);
  tanphi       = tan(fpeInfo.phi);
  fpeInfo.r0F0 = fpeInfo.r0P0 + (fpeInfo.h * tanphi)*fpeInfo.u;
  fpeInfo.l    = fpeInfo.h/cos(fpeInfo.phi);
  fpeInfo.E    = m*g*fpeInfo.l;

  fpeInfo.w0F0nPlus =  (cosphi*fpeInfo.h*m*(
                          sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
                        + fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n
                        )/(h2*m+fpeInfo.nJC0n*cos2phi);

  // MMillard 14 January 2020
  // The expressions are big, and have been derived and simplified using Maxima.
  // To make these run a bit faster I'm using memory that exists in the struct
  // fpeInfo wherever possible. In addition I've replaced all occurances of
  // pow(x,2.0) with simply x*x, which is faster. These things don't make the
  // huge equations any easier to read, but I think that's fine: these equations
  // are already so big that I have to use a symbolic mathematics package.
  if(evaluate_derivatives){
    double dh_dl;
    double Ds_Dphi;
    double Dl_DE;
    double Dl_Dphi;
    double Dphi_Dl;


    t0 = (cosphi*fpeInfo.h*m*
          (sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
          +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n);

    t1 = (h2*m+fpeInfo.nJC0n*cos2phi);

    fpeInfo.Df_Dphi    = (2*fpeInfo.nJC0n*cosphi*sinphi*t0*t0)/(t1*t1)
        +(2*(cosphi*fpeInfo.h*m*(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
             +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n)
            *((-fpeInfo.h*m*sinphi*(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u))
              +cosphi*fpeInfo.h*m*(cosphi*fpeInfo.v0C0k-sinphi*fpeInfo.v0C0u)
              -2*fpeInfo.nJC0n*cosphi*fpeInfo.w0C0n*sinphi))
        /(h2*m+fpeInfo.nJC0n*cos2phi)
        -2*cosphi*g*fpeInfo.h*m*sinphi
        -2*(cosphi-1)*g*fpeInfo.h*m*sinphi;

    fpeInfo.Df_Dw0C0n  = (2*fpeInfo.nJC0n*cos2phi
                          *(cosphi*fpeInfo.h*m
                            *(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
                            +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n))
                          /(h2*m+fpeInfo.nJC0n*cos2phi);

    t0 = cosphi*fpeInfo.h*m
        *(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
        +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n;
    t1 = h2*m+fpeInfo.nJC0n*cos2phi;

    fpeInfo.Df_Dh      = (-(2*fpeInfo.h*m*(t0*t0)))/(t1*t1)
        +(2*cosphi*m*(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
          *(cosphi*fpeInfo.h*m*(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
            +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n))
            /(h2*m+fpeInfo.nJC0n*cos2phi) + 2*(cosphi-1)*cosphi*g*m;



    fpeInfo.Df_Dv0C0u     =
        (2*cos2phi*fpeInfo.h*m*(cosphi*fpeInfo.h*m
          *(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
            +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n))
          /(h2*m+fpeInfo.nJC0n*cos2phi);

    fpeInfo.Df_Dv0C0k  =
        (2*cosphi*fpeInfo.h*m*sinphi*(cosphi*fpeInfo.h*m
          *(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
            +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n))
        /(h2*m+fpeInfo.nJC0n*cos2phi);

    t0 = cosphi*fpeInfo.h*m
        *(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
          +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n;

    t1 = h2*m+fpeInfo.nJC0n*cos2phi;

    fpeInfo.Df_DnJC0n  =
        (2*cos2phi*fpeInfo.w0C0n*(cosphi*fpeInfo.h*m
          *(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
            +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n))
        /(h2*m+fpeInfo.nJC0n*cos2phi)
        -(cos2phi*(t0*t0))/(t1*t1);

    t0=cosphi*fpeInfo.h*m
        *(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
          +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n;
    t1=h2*m+fpeInfo.nJC0n*cos2phi;

    fpeInfo.Df_Dm  = (-(h2*(t0*t0))/(t1*t1))
        +(2*cosphi*fpeInfo.h*(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
          *(cosphi*fpeInfo.h*m*(sinphi*fpeInfo.v0C0k+cosphi*fpeInfo.v0C0u)
            +fpeInfo.nJC0n*cos2phi*fpeInfo.w0C0n))
        /(h2*m+fpeInfo.nJC0n*cos2phi)
        +2*(cosphi-1)*cosphi*g*fpeInfo.h;

    fpeInfo.Df_Dg  = 2*(cosphi-1)*cosphi*fpeInfo.h*m;

    Ds_Dphi = fpeInfo.h*(1+tanphi*tanphi);

    //l = fpeInfo.h/cos(phi)
    //fpeInfo.h = l*cos(phi)
    //dh/dl = cos(phi)
    dh_dl = cosphi;

    // f = fo + (Df/Dphi)Dphi + (Df/Dh * dh/dl)*Dl.
    // f = fo = 0 at the solution
    // Dphi/Dl = -(Df/Dh dh/dl)/(Df/Dphi)
    fpeInfo.Dphi_Dl = (-1/fpeInfo.Df_Dphi)*(fpeInfo.Df_Dh)*dh_dl;
    fpeInfo.Ds_Dl   = (Ds_Dphi)*fpeInfo.Dphi_Dl;

    // f = fo + (Df/Dphi)Dphi + (Df/DJ)*dJ.
    // f = fo = 0
    // Dphi/DJ = -(Df/DJ)/(Df/Dphi)
    fpeInfo.Dphi_DnJC0n = (-1/fpeInfo.Df_Dphi)*(fpeInfo.Df_DnJC0n);
    fpeInfo.Ds_DnJC0n   = (Ds_Dphi)*fpeInfo.Dphi_DnJC0n;

    //  l = fpeInfo.h/cosphi;
    Dl_Dphi = fpeInfo.h*sinphi/(cos2phi);
    Dphi_Dl = (1/Dl_Dphi);
    //cosphi=fpeInfo.h/l

    //E = mgl
    Dl_DE = 1/(m*g);
    fpeInfo.Dphi_DE = (Dphi_Dl*Dl_DE);
    fpeInfo.Ds_DE = (Ds_Dphi)*(Dphi_Dl*Dl_DE);


    //Angular momentum = r x mv + Jw
    //The sensitivity of the foot placement to the input momentum under
    //the assumption that m, r, and fpeInfo.nJC0n are constant is just the
    //sensitivity of the foot placement location to the input velocities
    //fpeInfo.v0C0u, vz and w

    fpeInfo.Dphi_Dv0C0u = (-1/fpeInfo.Df_Dphi)*fpeInfo.Df_Dv0C0u;
    fpeInfo.Ds_Dv0C0u = (Ds_Dphi)*fpeInfo.Dphi_Dv0C0u;

    fpeInfo.Dphi_Dv0C0k = (-1/fpeInfo.Df_Dphi)*fpeInfo.Df_Dv0C0k;
    fpeInfo.Ds_Dv0C0k = (Ds_Dphi)*fpeInfo.Dphi_Dv0C0k;

    fpeInfo.Dphi_Dw0C0n = (-1/fpeInfo.Df_Dphi)*fpeInfo.Df_Dw0C0n;
    fpeInfo.Ds_Dw0C0n = (Ds_Dphi)*fpeInfo.Dphi_Dw0C0n;

  }
 }

