/*
 * Muscle addon for RBDL
 * Copyright (c) 2016 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */
#include "TorqueMuscleFunctionFactory.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;
//=============================================================================
// Anderson 2007 Active Torque Angle Curve
//=============================================================================


void TorqueMuscleFunctionFactory::
  createAnderson2007ActiveTorqueAngleCurve(
    double c2, 
    double c3,
    const std::string& curveName,
    SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Check the input arguments
  if( !(c2 > 0) ){ 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAnderson2007ActiveTorqueAngleCurve " 
          << curveName
          << ": c2 must be greater than 0"
          << endl;
    assert(0);
    abort();          
  }


  std::string name=curveName;
  name.append(".createAnderson2007ActiveTorqueAngleCurve");

  //For now these advanced paramters are hidden. They will only be
  //uncovered if absolutely necessary.
  double minValueAtShoulders = 0;
  double minShoulderSlopeMagnitude = 0;

  double curviness = 0.5;
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);

  //Translate the user parameters to quintic Bezier points
  double x0 = c3 - 1.05*(0.5*(M_PI/c2));
  double x1 = c3 - 0.95*(0.5*(M_PI/c2));
  double x2 = c3;
  double x3 = c3 + 0.95*(0.5*(M_PI/c2));
  double x4 = c3 + 1.05*(0.5*(M_PI/c2));

  double y0 = minValueAtShoulders;
  double y1 = cos(c2*(x1-c3));
  double y2 = cos(c2*(x2-c3));
  double y3 = cos(c2*(x3-c3));
  double y4 = minValueAtShoulders;

  double dydx0 =  minShoulderSlopeMagnitude;
  double dydx1 = -sin(c2*(x1-c3))*c2;
  double dydx2 = -sin(c2*(x2-c3))*c2;
  double dydx3 = -sin(c2*(x3-c3))*c2;
  double dydx4 = -minShoulderSlopeMagnitude;


  //Compute the Quintic Bezier control points
  RigidBodyDynamics::Math::MatrixNd p0 = 
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x0,y0,dydx0,
                          x1,y1,dydx1,c);

  RigidBodyDynamics::Math::MatrixNd p1 = 
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                          x2,y2,dydx2,c);

  RigidBodyDynamics::Math::MatrixNd p2 = 
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                          x3,y3,dydx3,c);

  RigidBodyDynamics::Math::MatrixNd p3 = 
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x3,y3,dydx3,
                          x4,y4,dydx4,c);                                

  RigidBodyDynamics::Math::MatrixNd mX(6,4);
  RigidBodyDynamics::Math::MatrixNd mY(6,4);

  mX.col(0) = p0.col(0);
  mY.col(0) = p0.col(1);
  mX.col(1) = p1.col(0);
  mY.col(1) = p1.col(1);
  mX.col(2) = p2.col(0);
  mY.col(2) = p2.col(1);
  mX.col(3) = p3.col(0);
  mY.col(3) = p3.col(1);


  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
              mX, mY, x0, x4, y0, y4, dydx0, dydx4, curveName);
}

//=============================================================================
// ANDERSON 2007 Active Torque Angular Velocity Curve
//=============================================================================
void TorqueMuscleFunctionFactory::
  createAnderson2007ActiveTorqueVelocityCurve(
    double c4, 
    double c5,
    double c6,
    double minEccentricMultiplier,
    double maxEccentricMultiplier,
    const std::string& curveName,
    SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{
  //Check the input arguments
  if( !(c4 < c5) ){ 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAndersonActiveTorqueVelocityCurve " 
          << curveName
          << ": c4 must be greater than c5"
          << endl;
    assert(0);
    abort();          
  }

  if( !((c4 > 0)) ){ 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAndersonActiveTorqueVelocityCurve " 
          << curveName
          << ": c4 must be greater than 0"
          << endl;
    assert(0);
    abort();          
  }

  if( !(c6 > 0.0) ){ 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAndersonActiveTorqueVelocityCurve " 
          << curveName
          << ": c6 must be greater than 1.0"
          << endl;
    assert(0);
    abort();          
  }

  if( !(minEccentricMultiplier > 1.0) ){ 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAndersonActiveTorqueVelocityCurve " 
          << curveName
          << ": minEccentricMultiplier must be greater than 1.0"
          << endl;
    assert(0);
    abort(); 
  }

  if( !(maxEccentricMultiplier > minEccentricMultiplier) ){ 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAndersonActiveTorqueVelocityCurve " 
          << curveName
          << ": maxEccentricMultiplier must be greater than "
          << " minEccentricMultiplier"
          << endl;
    assert(0);
    abort();
  }

  //Advanced settings that we'll hide for now
  double minShoulderSlopeMagnitude  = 0;
  double curviness = 0.75;
  double c         = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);

  //Go and get the value of the curve that is closest to 
  //the maximum contraction velocity by setting rhs of Eqn. 9
  //to 0 and solving
  double dthMaxConc = fabs( 2.0*c4*c5/(c5-3.0*c4) );

  //Depending on the selection of parameters, the original Anderson 
  //torque-velocity curves won't actually go to 0 on the concentric
  //side. This only show up in the case of the plantar flexors.

  //To get the maximum concentric contraction velocity we'll back 
  //up from the maximum calculated above and linearly extrapolate
  //to zero.
  double x1  =   dthMaxConc*0.80;

  double y1den =  (2*c4*c5 + x1*(2*c5-4*c4));            
  double y1  =   (2*c4*c5 + x1*(c5-3*c4))/y1den;

  double dydx1 =   (c5-3*c4)/(2*c4*c5 + x1*(2*c5-4*c4)) 
          -(2*c4*c5 + x1*(c5-3*c4))*(2*c5-4*c4)
          /( y1den*y1den );

  if( !(dydx1 < 0.0) ){ 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAndersonActiveTorqueVelocityCurve " 
          << curveName
          << ": choice of c3,c5, and c6 is such that "
          << " the curve will never cross the x axis."
          << endl;
    assert(0);
    abort();   
  }

  //Linearly extrapolate to get the maximum velocity, and then go just a
  //little futher - so that curve stays smooth
  double x0 = (x1-y1/dydx1)*1.1;

  double x2 = c5; // 50% iso
  double x3 = c4; // 75% iso
  double x4 = 0; 
  double x5 = -x0;//-60*pi/180; 
  //Anderson et al. 2007: pg 3107, column 2, paragraph 2.
  //they made 1 measurement at 60 degree's sec.

  double y0 = 0;

  double y2 =  (2*c4*c5 + x2*(c5-3*c4))
        /(2*c4*c5 + x2*(2*c5-4*c4));

  double y3 =  (2*c4*c5 + x3*(c5-3*c4))
        /(2*c4*c5 + x3*(2*c5-4*c4));

  double y4 = 1;

  double y5 = minEccentricMultiplier; 
  double x60  = -60*M_PI/180;
  double y5at60 = (2*c4*c5 - x60*(  c5-3*c4))*(1.0-c6*x60)
           /(2*c4*c5 - x60*(2*c5-4*c4));

  //This expression to evaluate y5 given the value of the
  //eccentric curve at an angular velocity of 60/s
  //(where Anderson actually took a measurement) is a
  //hueristic that fits most of Anderson's data that I have
  //observed
  y5 = 1+3*(y5at60-1);

  if(y5 < minEccentricMultiplier){
     y5 = minEccentricMultiplier;
  }
  if(y5 > maxEccentricMultiplier){
     y5 = maxEccentricMultiplier; 
  }



  double dydx0 = -minShoulderSlopeMagnitude;

  double tmpD =   (2*c4*c5 + x2*(2*c5-4*c4));
  double dydx2 =  (c5-3*c4)/(tmpD)
          -(2*c4*c5 + x2*(c5-3*c4))*(2*c5-4*c4)/(tmpD*tmpD);
      
  tmpD     =  (2*c4*c5 + x3*(2*c5-4*c4));      
  double dydx3 =  (c5-3*c4)/(tmpD)
          -(2*c4*c5 + x3*(c5-3*c4))*(2*c5-4*c4)/(tmpD*tmpD);
      
  tmpD     =  (2*c4*c5 + x4*(2*c5-4*c4));     
  double dydx4 =  (c5-3*c4)/(tmpD)
          -(2*c4*c5 + x4*(c5-3*c4))*(2*c5-4*c4)
          /(tmpD*tmpD);

  double dydx5 = -minShoulderSlopeMagnitude;


  RigidBodyDynamics::Math::MatrixNd p0 =
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x5,y5,dydx5,
                          x4,y4,dydx4,
                          c);
  RigidBodyDynamics::Math::MatrixNd p1 =
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x4,y4,dydx4,
                          x3,y3,dydx3,
                          c);
  RigidBodyDynamics::Math::MatrixNd p2 =
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x3,y3,dydx3,
                          x2,y2,dydx2,c);
  RigidBodyDynamics::Math::MatrixNd p3 =
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                          x1,y1,dydx1,c);
  RigidBodyDynamics::Math::MatrixNd p4 =
    SegmentedQuinticBezierToolkit::
       calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                          x0,y0,dydx0,c);


  RigidBodyDynamics::Math::MatrixNd mX(6,5);
  RigidBodyDynamics::Math::MatrixNd mY(6,5);

  mX.col(0) = p0.col(0);
  mY.col(0) = p0.col(1);
  mX.col(1) = p1.col(0);
  mY.col(1) = p1.col(1);
  mX.col(2) = p2.col(0);
  mY.col(2) = p2.col(1);
  mX.col(3) = p3.col(0);
  mY.col(3) = p3.col(1);
  mX.col(4) = p4.col(0);
  mY.col(4) = p4.col(1);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
              mX, mY, x5, x0, y5, y0, dydx5, dydx0, curveName);
}
//=============================================================================
// ANDERSON 2007 Passive Torque Angle Curve
//=============================================================================
void TorqueMuscleFunctionFactory:: 
  createAnderson2007PassiveTorqueAngleCurve(
    double scale,
    double c1,
    double b1,
    double k1,
    double b2,
    double k2,                    
    const std::string& curveName,
    SmoothSegmentedFunction& smoothSegmentedFunctionToUpdate)
{

  if( !(scale > 0) ){ 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAnderson2007PassiveTorqueAngleCurve " 
          << curveName
          << ": scale must be greater than 0"
          << endl;
    assert(0);
    abort();         
  }

  if( !(c1 > 0) ) { 
    cerr  << "TorqueMuscleFunctionFactory::"
          << "createAnderson2007PassiveTorqueAngleCurve " 
          << curveName
          << ": c1 must be greater than 0"
          << endl;
    assert(0);
    abort();          
  }

  //Advanced settings that we'll hide for now
  double curviness = 0.75;
  double c = SegmentedQuinticBezierToolkit::scaleCurviness(curviness);
  double minShoulderSlopeMagnitude = 0;
  
  //Zero out the coefficients associated with a 
  //the side of the curve that goes negative.
  bool flag_oneSided = true;

  if(flag_oneSided){
    if(fabs(b1) > 0){
      if(b1 > 0){
        b2 = 0;
        k2 = 0;
      }else{
        b1 = 0; 
        k1 = 0;
      }

    }else if(fabs(b2) > 0){
      if(b2 > 0){
        b1 = 0; 
        k1 = 0;   
      }else{
        b2 = 0;
        k2 = 0;
      }
    }   
  }
  //Divide up the curve into a left half
  //and a right half, rather than 1 and 2. 
  //Why? These two different halves require different
  //   Bezier curves.

  double c1Scale = c1*scale;
  double thL    = 0.; //left
  double thR    = 0.; //right
  double DtauDthL = 0.;
  double DtauDthR = 0.;
  double bL     = 0.;
  double kL     = 0.;
  double bR     = 0.;
  double kR     = 0.;

  int curveType   = 0; //flat curve
  int flag_thL  = 0;
  int flag_thR  = 0;

  if(fabs(k1)>0 && fabs(b1)>0){
    //The value of theta where the passive force generated by the
    //muscle is equal to 1 maximum isometric contraction.
    thL     = (1/k1)*log(fabs( c1Scale/b1 ));
    DtauDthL  = b1*k1*exp(thL*k1);      
    bL      = b1;
    kL      = k1;
    flag_thL  = 1;
  }
  
  if(fabs(k2)>0 && fabs(b2)>0){
    //The value of theta where the passive force generated by the
    //muscle is equal to 1 maximum isometric contraction.
    thR     = (1/k2)*log(fabs( c1Scale/b2 ));
    DtauDthR  = b2*k2*exp(thR*k2);  
    bR      = b2;
    kR      = k2;   
    flag_thR  = 1;
  }

  //A 'left' curve has a negative slope,
  //A 'right' curve has a positive slope.
  if(DtauDthL > DtauDthR){
    double tmpD = thL;
    thL   = thR;
    thR   = tmpD;

    tmpD  =  bR;
    bR    = bL;
    bL    = tmpD;

    tmpD  = kR;
    kR    = kL;
    kL    = tmpD;

    tmpD    = DtauDthL;
    DtauDthL  = DtauDthR;
    DtauDthR  = tmpD;

    int tmpI = flag_thL;
    flag_thL = flag_thR;
    flag_thR = tmpI;    
  }


  if(flag_thL){
    curveType   = curveType + 1;
  }
  if(flag_thR){
    curveType   = curveType + 2;
  }

  RigidBodyDynamics::Math::MatrixNd mX(6,1);
  RigidBodyDynamics::Math::MatrixNd mY(6,1);

  double xStart  = 0;
  double xEnd    = 0;
  double yStart  = 0;
  double yEnd    = 0;
  double dydxStart = 0;
  double dydxEnd   = 0;


  switch(curveType){

    case 0:
      {//No curve - it's a flat line
        RigidBodyDynamics::Math::MatrixNd p0 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(0.,0.,0.,
                                1.,0.,0.,c); 

        mX.col(0) = p0.col(0);
        mY.col(0) = p0.col(1);

      }break;
    case 1:
      {        
        //Get a point on the curve that is close to 0.
        double x1  = (1/kL)*log(fabs(0.01*c1Scale/bL) );
        double y1  = bL*exp(kL*x1);
        double dydx1 = bL*kL*exp(kL*x1);

        //Get a point that is at 1 maximum isometric torque
        double x3  = thL;
        double y3  = bL*exp(kL*x3);
        double dydx3 = bL*kL*exp(kL*x3);
           
        //Get a mid-point         
        double x2  = 0.5*(x1+x3);
        double y2  = bL*exp(kL*x2);
        double dydx2 = bL*kL*exp(kL*x2);
        
        //Past the crossing point of the linear extrapolation
        double x0   = x1 - 2*y1/dydx1;
        double y0   = 0;
        double dydx0  = minShoulderSlopeMagnitude*copysign(1.0,dydx1);
        
        xStart    = x3;
        xEnd    = x0;
        yStart    = y3;
        yEnd    = y0;
        dydxStart   = dydx3;
        dydxEnd   = dydx0;

        RigidBodyDynamics::Math::MatrixNd p0 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x3,y3,dydx3,
                                x2,y2,dydx2,c); 
        RigidBodyDynamics::Math::MatrixNd p1 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                                x1,y1,dydx1,c); 
        RigidBodyDynamics::Math::MatrixNd p2 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                                x0,y0,dydx0,c);

        mX.resize(6,3);
        mY.resize(6,3);

        mX.col(0) = p0.col(0);
        mY.col(0) = p0.col(1);     
        mX.col(1) = p1.col(0);
        mY.col(1) = p1.col(1); 
        mX.col(2) = p2.col(0);
        mY.col(2) = p2.col(1); 

      }break;
    case 2:
      {
        //Get a point on the curve that is close to 0.
        double x1  = (1/kR)*log(fabs(0.01*c1Scale/bR) );
        double y1  = bR*exp(kR*x1);
        double dydx1 = bR*kR*exp(kR*x1);

        //Go just past the crossing point of the linear extrapolation
        double x0   = x1 - 2*y1/dydx1;
        double y0   = 0;
        double dydx0 = minShoulderSlopeMagnitude*copysign(1.0,dydx1);

        //Get a point close to 1 maximum isometric torque
        double x3  = thR;
        double y3  = bR*exp(kR*x3);
        double dydx3 = bR*kR*exp(kR*x3);

        //Get a mid point.
        double x2   = 0.5*(x1+x3);
        double y2   = bR*exp(kR*x2);
        double dydx2  = bR*kR*exp(kR*x2);    

        xStart    = x0;
        xEnd    = x3;
        yStart    = y0;
        yEnd    = y3;
        dydxStart   = dydx0;
        dydxEnd   = dydx3;

        RigidBodyDynamics::Math::MatrixNd p0 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x0,y0,dydx0,
                                x1,y1,dydx1,c); 
        RigidBodyDynamics::Math::MatrixNd p1 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                                x2,y2,dydx2,c); 
        RigidBodyDynamics::Math::MatrixNd p2 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                                x3,y3,dydx3,c);
        mX.resize(6,3);
        mY.resize(6,3);

        mX.col(0) = p0.col(0);
        mY.col(0) = p0.col(1);     
        mX.col(1) = p1.col(0);
        mY.col(1) = p1.col(1); 
        mX.col(2) = p2.col(0);
        mY.col(2) = p2.col(1); 

      }break;  
    case 3:
      {
        //Only when flag_oneSided = false;
        double x0 = thL;
        double x4 = thR;

        double x2 = 0.5*(x0 + x4);
        double x1 = 0.5*(x0 + x2);
        double x3 = 0.5*(x2 + x4);

        double y0 = b1*exp(k1*x0)
              + b2*exp(k2*x0);
        double y1 = b1*exp(k1*x1)
              + b2*exp(k2*x1);        
        double y2 = b1*exp(k1*x2)
              + b2*exp(k2*x2);
        double y3 = b1*exp(k1*x3)
              + b2*exp(k2*x3);
        double y4 = b1*exp(k1*x4)
              + b2*exp(k2*x4);    

        double dydx0 =   b1*k1*exp(k1*x0)
                 + b2*k2*exp(k2*x0);
        double dydx1 =   b1*k1*exp(k1*x1)
                 + b2*k2*exp(k2*x1);
        double dydx2 =   b1*k1*exp(k1*x2)
                 + b2*k2*exp(k2*x2);
        double dydx3 =   b1*k1*exp(k1*x3)
                 + b2*k2*exp(k2*x3);
        double dydx4 =   b1*k1*exp(k1*x4)
                 + b2*k2*exp(k2*x4);  

        xStart    = x0;
        xEnd    = x4;
        yStart    = y0;
        yEnd    = y4;
        dydxStart   = dydx0;
        dydxEnd   = dydx4;                   

        RigidBodyDynamics::Math::MatrixNd p0 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x0,y0,dydx0,
                                x1,y1,dydx1,c); 
        RigidBodyDynamics::Math::MatrixNd p1 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x1,y1,dydx1,
                                x2,y2,dydx2,c); 
        RigidBodyDynamics::Math::MatrixNd p2 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x2,y2,dydx2,
                                x3,y3,dydx3,c);
        RigidBodyDynamics::Math::MatrixNd p3 = 
          SegmentedQuinticBezierToolkit::
             calcQuinticBezierCornerControlPoints(x3,y3,dydx3,
                                x4,y4,dydx4,c);

        mX.resize(6,4);
        mY.resize(6,4);

        mX.col(0) = p0.col(0);
        mY.col(0) = p0.col(1);     
        mX.col(1) = p1.col(0);
        mY.col(1) = p1.col(1); 
        mX.col(2) = p2.col(0);
        mY.col(2) = p2.col(1);    
        mX.col(3) = p3.col(0);
        mY.col(3) = p3.col(1); 

      }break;
    default:
    {
      cerr  << "TorqueMuscleFunctionFactory::"
            << "createAnderson2007PassiveTorqueAngleCurve " 
            << curveName
            << ": undefined curveType"
            << endl;
    assert(0);
    abort();   
    }

  };

  //Normalize the y values.
  mY = mY*(1/c1Scale);

  smoothSegmentedFunctionToUpdate.updSmoothSegmentedFunction(
    mX,                mY, 
    xStart,            xEnd, 
    yStart/c1Scale,    yEnd/c1Scale,
    dydxStart/c1Scale, dydxEnd/c1Scale,
    curveName);
}


