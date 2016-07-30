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

#include "geometry.h"

#include <UnitTest++.h>
#include <rbdl/rbdl_math.h>
#include <ctime>
#include <string>
#include <stdio.h>
#include <exception>
#include <cassert>
#include <fstream>

using namespace RigidBodyDynamics::Addons::Geometry;

using namespace std;

static double EPSILON = numeric_limits<double>::epsilon();

static bool FLAG_PLOT_CURVES    = false;
static string FILE_PATH         = "";
static double TOL_DX            = 5e-3;
static double TOL_DX_BIG        = 1e-2;
static double TOL_BIG           = 1e-6;
static double TOL_SMALL         = 1e-12;



/**
This function will print cvs file of the matrix 
 data

@params data: A matrix of data
@params filename: The name of the file to print
*/
void printMatrixToFile( 
    const RigidBodyDynamics::Math::MatrixNd& data, 
    string& filename)
{
    ofstream datafile;
    datafile.open(filename.c_str());

    for(int i = 0; i < data.rows(); i++){
     for(int j = 0; j < data.cols(); j++){
      if(j<data.cols()-1)
       datafile << data(i,j) << ",";
      else
       datafile << data(i,j) << "\n";
     }   
    }
    datafile.close();
}

/**
    This function computes a standard central difference dy/dx. If 
    extrap_endpoints is set to 1, then the derivative at the end points is 
    estimated by linearly extrapolating the dy/dx values beside the end points

 @param x domain vector
 @param y range vector
 @param extrap_endpoints: (false)   Endpoints of the returned vector will be 
               zero, because a central difference
               is undefined at these endpoints
             (true)  Endpoints are computed by linearly 
               extrapolating using a first difference from 
               the neighboring 2 points
 @returns dy/dx computed using central differences
*/
RigidBodyDynamics::Math::VectorNd 
    calcCentralDifference(  
             RigidBodyDynamics::Math::VectorNd& x, 
             RigidBodyDynamics::Math::VectorNd& y, 
             bool extrap_endpoints){
 
    RigidBodyDynamics::Math::VectorNd dy(x.size());
    double dx1,dx2;
    double dy1,dy2;
    int size = x.size();
    for(int i=1; i<size-1; i++){
     dx1 = x[i]   - x[i-1];
     dx2 = x[i+1] - x[i];
     dy1 = y[i]   - y[i-1];
     dy2 = y[i+1] - y[i];
     dy[i]= 0.5*dy1/dx1 + 0.5*dy2/dx2;
    }

    if(extrap_endpoints == true){
     dy1 = dy[2] - dy[1];
     dx1 = x[2]  - x[1];
     dy[0] = dy[1] + (dy1/dx1)*(x[0]-x[1]);

     dy2 = dy[size-2] - dy[size-3];
     dx2 = x[size-2]  - x[size-3];
     dy[size-1] = dy[size-2] + (dy2/dx2)*(x[size-1]-x[size-2]);
    }
    return dy;
}

/**
    This function computes a standard central difference dy/dx at each point in
    a vector x, for a SmoothSegmentedFunction mcf, to a desired tolerance. This 
    function will take the best step size at each point to minimize the 
    error caused by taking a numerical derivative, and the error caused by
    numerical rounding error:

    For a step size of h/2 to the left and to the right of the point of 
    interest the error is
    error = 1/4*h^2*c3 + r*f(x)/h,         (1)
      
    Where c3 is the coefficient of the 3rd order Taylor series expansion
    about point x. Thus c3 can be computed if the order + 2 derivative is
    known
     
     c3 = (d^3f(x)/dx^3)/(6)            (2)
     
    And r*f(x)/h is the rounding error that occurs due to the central 
    difference.

    Taking a first derivative of 1 and solving for h yields

    h = (r*f(x)*2/c3)^(1/3)

    Where r is EPSILON

  @param x domain vector
  @param mcf the SmoothSegmentedFunction of interest
  @param order the order of the numerical derivative
  @param tolerance desired tolerance on the returned result
  @returns dy/dx computed using central differences
*/
RigidBodyDynamics::Math::VectorNd 
calcCentralDifference(  RigidBodyDynamics::Math::VectorNd& x, 
            SmoothSegmentedFunction& mcf,
            double tol, int order){
 

    RigidBodyDynamics::Math::VectorNd dyV(x.size());
    RigidBodyDynamics::Math::VectorNd yV(x.size());

    double y = 0;
    double dy = 0;
    double dyNUM = 0;
    double err= 0;
    double h = 0;
    double xL = 0;
    double xR = 0;

    double c3 = 0;
    double fL = 0;
    double fR = 0;
    double rootEPS = sqrt(EPSILON);

    double y_C3min = 1e-10;
    double y_C3max = 1e1;


    for(int i=0; i<x.size(); i++){
     yV[i] = mcf.calcDerivative(x[i],order-1);
    }
   

    for(int i=0; i< x.size(); i++){
     
     c3 = abs(mcf.calcDerivative(x[i],order+2));
     
     //singularity prevention
     if(abs(c3) < y_C3min)
      c3 = y_C3min;
     //Compute h
     y  = abs(mcf.calcDerivative(x[i], order-1));
     //preventing 0 from being assigned to y
     if(y < y_C3min)
      y = y_C3min;

     //Dumb check
     if(y/c3 < y_C3min){
      c3 = 1;
      y = y_C3min;
     }
     if(y/c3 > y_C3max){
      c3 = 1;
      y = y_C3max;
     }

     h  = pow( ( (EPSILON*y*2.0)/(c3) ) , 1.0/3.0);
    
     //Now check that h to the left and right are at least similar
     //If not, take the smallest one.
     xL = x[i]-h/2;
     xR = x[i]+h/2;

     fL = mcf.calcDerivative(xL, order-1);
     fR = mcf.calcDerivative(xR, order-1);

     //Just for convenience checking ...
     dyNUM = (fR-fL)/h;
     dy    = mcf.calcDerivative(x[i],order);
     err   = abs(dy-dyNUM);

     /*if(err > tol && abs(dy) > rootEPS && order <= 2){
      err = err/abs(dy);
      if(err > tol)
       cout << "rel tol exceeded" << endl;     
     }*/

     dyV[i] = dyNUM;

    }


    return dyV;
}


/**
    This function tests numerically for continuity of a curve. The test is 
    performed by taking a point on the curve, and then two points (called the 
    shoulder points) to the left and right of the point in question. The value
    of the functions derivative is evaluated at each of the shoulder points and
    used to linearly extrapolate from the shoulder points back to the original 
    point. If the original point and the linear extrapolations of each of the 
    shoulder points agree within tol, then the curve is assumed to be 
    continuous.


    @param x     Values to test for continuity
    @param yx    The SmoothSegmentedFunction to test
    @param order    The order of the curve of SmoothSegmentedFunction to test
        for continuity
    @param minTol   The minimum error allowed - this prevents the second order
        error term from going to zero
    @param taylorErrorMult  This scales the error tolerance. The default error
             tolerance is the the 2nd order Taylor series
             term.
*/
bool isFunctionContinuous(  RigidBodyDynamics::Math::VectorNd& xV, 
             SmoothSegmentedFunction& yV, 
             int order, 
             double minTol,
             double taylorErrorMult)
{
    bool flag_continuous = true;

    double xL = 0;   // left shoulder point
    double xR = 0;   // right shoulder point
    double yL = 0;   // left shoulder point function value
    double yR = 0;   // right shoulder point function value
    double dydxL = 0;   // left shoulder point derivative value
    double dydxR = 0;   // right shoulder point derivative value

    double xVal = 0;    //x value to test
    double yVal = 0;    //Y(x) value to test

    double yValEL = 0;  //Extrapolation to yVal from the left
    double yValER = 0;  //Extrapolation to yVal from the right

    double errL = 0;
    double errR = 0;

    double errLMX = 0;
    double errRMX = 0;


    for(int i =1; i < xV.size()-1; i++){
     xVal = xV[i];
     yVal = yV.calcDerivative(xVal, order);

     xL = 0.5*(xV[i]+xV[i-1]);
     xR = 0.5*(xV[i]+xV[i+1]);

     yL = yV.calcDerivative(xL,order);
     yR = yV.calcDerivative(xR,order);

     dydxL = yV.calcDerivative(xL,order+1);
     dydxR = yV.calcDerivative(xR,order+1);

     
     yValEL = yL + dydxL*(xVal-xL);
     yValER = yR - dydxR*(xR-xVal);

     errL = abs(yValEL-yVal);
     errR = abs(yValER-yVal);

     errLMX = abs(yV.calcDerivative(xL,order+2)*0.5*(xVal-xL)*(xVal-xL));
     errRMX = abs(yV.calcDerivative(xR,order+2)*0.5*(xR-xVal)*(xR-xVal));

     errLMX*=taylorErrorMult;
     errRMX*=taylorErrorMult;

     if(errLMX < minTol)
      errLMX = minTol;

     if(errRMX < minTol)
      errRMX = minTol; // to accomodate numerical
              //error in errL

     if(errL > errLMX || errR > errRMX){      
      flag_continuous = false;
     }
    }

    return flag_continuous;
}


/**
This function will scan through a vector and determine if it is monotonic or
not

@param y the vector of interest
@param multEPS The tolerance on the monotoncity check, expressed as a scaling of
       EPSILON
@return true if the vector is monotonic, false if it is not
*/
bool isVectorMonotonic( RigidBodyDynamics::Math::VectorNd& y, 
            int multEPS)
{
    double dir = y[y.size()-1]-y[0];
    bool isMonotonic = true;

    if(dir < 0){
     for(int i =1; i <y.size(); i++){
      if(y[i] > y[i-1]+EPSILON*multEPS){
       isMonotonic = false;
      //printf("Monotonicity broken at idx %i, since %fe-16 > %fe-16\n",
       //     i,y(i)*1e16,y(i-1)*1e16);
       printf("Monotonicity broken at idx %i, since "
        "y(i)-y(i-1) < tol, (%f*EPSILON < EPSILON*%i) \n",
            i,((y[i]-y[i-1])/EPSILON), multEPS);
      }
     }
    }
    if(dir > 0){
     for(int i =1; i <y.size(); i++){
      if(y[i] < y[i-1]-EPSILON*multEPS){
       isMonotonic = false;
       printf("Monotonicity broken at idx %i, since "
        "y(i)-y(i-1) < -tol, (%f*EPSILON < -EPSILON*%i) \n",
            i,((y[i]-y[i-1])/EPSILON), multEPS);
      }
     }
    }
    if(dir == 0){
     isMonotonic = false;
    }

    return isMonotonic;
}


/*
 3. The MuscleCurveFunctions function value, first and second derivative curves
    will be numerically tested for continuity.
*/ 
bool isCurveC2Continuous(SmoothSegmentedFunction& mcf,
              RigidBodyDynamics::Math::MatrixNd& mcfSample)
{
    //cout << "   TEST: C2 Continuity " << endl;

    int multC0 = 5;
    int multC1 = 50;
    int multC2 = 100;

    RigidBodyDynamics::Math::VectorNd fcnSample = 
     RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());

    for(int i=0; i < mcfSample.rows(); i++){
     fcnSample[i] = mcfSample(i,0);
    }


    bool c0 = isFunctionContinuous(fcnSample, mcf, 0, 1e-6, multC0);
    bool c1 = isFunctionContinuous(fcnSample, mcf, 1, 1e-6, multC1);
    bool c2 = isFunctionContinuous(fcnSample, mcf, 2, 1e-6, multC2);



    return (c0 && c1 && c2);
    //printf( "   passed: C2 continuity established to a multiple\n"
    //     "     of the next Taylor series error term.\n "
    //     "     C0,C1, and C2 multiples: %i,%i and %i\n",
    //        multC0,multC1,multC2);
    //cout << endl;
}

/*
 4. The MuscleCurveFunctions which are supposed to be monotonic will be
    tested for monotonicity.
*/
bool isCurveMontonic(RigidBodyDynamics::Math::MatrixNd mcfSample)
{
    //cout << "   TEST: Monotonicity " << endl;
    int multEps = 10;

    RigidBodyDynamics::Math::VectorNd fcnSample = 
     RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());

    for(int i=0; i < mcfSample.rows(); i++){
     fcnSample[i] = mcfSample(i,1);
    }

    bool monotonic = isVectorMonotonic(fcnSample,10);
    return monotonic;
    //printf("   passed: curve is monotonic to %i*EPSILON",multEps);
    //cout << endl;
}

/**
 1. The SmoothSegmentedFunction's derivatives will be compared against 
    numerically calculated derivatives to ensure that the errors between the 
    2 are small

*/
bool areCurveDerivativesCloseToNumericDerivatives(
     SmoothSegmentedFunction& mcf,
     RigidBodyDynamics::Math::MatrixNd& mcfSample,
     double tol)
{
    //cout << "   TEST: Derivative correctness " << endl;
    int maxDer = 4;//mcf.getMaxDerivativeOrder() - 2;
   
    RigidBodyDynamics::Math::MatrixNd numSample(mcfSample.rows(),maxDer);  
    RigidBodyDynamics::Math::MatrixNd relError(mcfSample.rows(),maxDer);
    

    RigidBodyDynamics::Math::VectorNd domainX = 
     RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());

    for(int j=0; j<mcfSample.rows(); j++)
     domainX[j] = mcfSample(j,0);

    for(int i=0; i < maxDer; i++){
      //Compute the relative error
      numSample.col(i)=calcCentralDifference(domainX,mcf,tol,i+1);
      for(int j=0; j<mcfSample.rows();++j ){
        relError(j,i)= mcfSample(j,i+2)-numSample(j,i);  
      }
     //compute a relative error where possible
     for(int j=0; j < relError.rows(); j++){
      if(abs(mcfSample(j,i+2)) > tol){
       relError(j,i) = relError(j,i)/mcfSample(j,i+2);
      }
     }

    }

    RigidBodyDynamics::Math::VectorNd errRelMax =
     RigidBodyDynamics::Math::VectorNd::Zero(6);
    RigidBodyDynamics::Math::VectorNd errAbsMax = 
     RigidBodyDynamics::Math::VectorNd::Zero(6);

    double absTol = 5*tol;

    bool flagError12=false;
    RigidBodyDynamics::Math::VectorNd tolExceeded12V =
     RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());
    
    int tolExceeded12 = 0;
    int tolExceeded34 = 0;
    for(int j=0;j<maxDer;j++){
     
     for(int i=0; i<mcfSample.rows(); i++){
      if(relError(i,j) > tol && mcfSample(i,j+2) > tol){
       if(j <= 1){
        tolExceeded12++;
        tolExceeded12V[i]=1;
        flagError12=true;
       }
       if(j>=2)
        tolExceeded34++;       
      }
      if(mcfSample(i,j+2) > tol)
      if(errRelMax[j] < abs(relError(i,j)))
        errRelMax[j] = abs(relError(i,j));

      //This is a harder test: here we're comparing absolute error
      //so the tolerance margin is a little higher
      if(relError(i,j) > absTol && mcfSample(i,j+2) <= tol){
       if(j <= 1){
        tolExceeded12++;
        tolExceeded12V[i]=1;
        flagError12=true;
       }
       if(j>=2)
        tolExceeded34++;            
      }

      if(mcfSample(i,j+2) < tol)
      if(errAbsMax[j] < abs(relError(i,j)))
        errAbsMax[j] = abs(relError(i,j));
      
     
     }

     /*
     if(flagError12 == true){
      printf("Derivative %i Rel Error Exceeded:\n",j);
      printf("x     dx_relErr dx_calcVal dx_sample"
        " dx2_relErr dx2_calcVal dx2_sample\n");
      for(int i=0; i<mcfSample.rows(); i++){
       if(tolExceeded12V(i) == 1){
          printf("%f %f %f  %f %f   %f    %f",
        mcfSample(i,0),relError(i,0),mcfSample(i,2),numSample(i,0),
                relError(i,1),mcfSample(i,3),numSample(i,1));
        
       }
      }
     }
     flagError12=false;*/
     //tolExceeded12V = 
     //RigidBodyDynamics::Math::VectorNd::Zero(mcfSample.rows());
    }
    




    //CHECK(tolExceeded12 == 0);

  //   printMatrixToFile(mcfSample,"analyticDerivatives.csv");
  //   printMatrixToFile(numSample,"numericDerivatives.csv");
  //   printMatrixToFile(numError,"numAnalyticError.csv");
  //   cout << "Matricies Printed" << endl;
    /*
   printf("passed: A tolerance of %fe-3 reached with a maximum relative error\n"
       "     of %fe-3 and %fe-3 for the first two derivatives\n"
       "     at points where it is possible to compute the relative\n"
       "     error.                     \n\n"
       "     At points where the relative error couldn't be computed,\n"
       "     due to divide by 0, the first two derivatives met an \n"
       "     absolute tolerance of %fe-3, with a maximum value of \n"
       "     %fe-3 and %fe-3. \n\n"
       "     Derivatives of order 3 through %i exceeded the \n"
       "     relative or absolute tolerance (as appropriate) %i times.\n",
       tol*1e3, errRelMax(0)*1e3, errRelMax(1)*1e3, absTol*1e3,
       errAbsMax(0)*1e3, errAbsMax(1)*1e3, maxDer, tolExceeded34);
  */
  //cout << endl;

   return (tolExceeded12 == 0);
    
}

/**
    This function will create a quintic Bezier curve y(x) and sample it, its 
    first derivative w.r.t. U (dx(u)/du and dy(u)/du), and its first derivative
    w.r.t. to X and print it to the screen.
*/
void testSegmentedQuinticBezierDerivatives(
      int maximumNumberOfToleranceViolations)
{
    //cout <<"**************************************************"<<endl;
    //cout << "   TEST: Bezier Curve Derivative DU" << endl;
    string name  = "testSegmentedQuinticBezierDerivatives()";
     RigidBodyDynamics::Math::VectorNd xPts(6);
     RigidBodyDynamics::Math::VectorNd yPts(6);
     xPts[0] = 0;
     xPts[1] = 0.5;
     xPts[2] = 0.5;
     xPts[3] = 0.75;
     xPts[4] = 0.75;
     xPts[5] = 1;

     yPts[0] = 0;
     yPts[1] = 0.125;
     yPts[2] = 0.125;
     yPts[3] = 0.5;
     yPts[4] = 0.5;
     yPts[5] = 1;

     double val = 0;
     double d1 = 0;
     double d2 = 0;
     double d3 = 0;
     double d4 = 0;
     double d5 = 0;
     double d6 = 0;

     double u = 0;

     int steps = 100;

     RigidBodyDynamics::Math::MatrixNd analyticDerXU(steps,8);
     RigidBodyDynamics::Math::MatrixNd analyticDerYU(steps,8);
     RigidBodyDynamics::Math::VectorNd uV(steps);
     for(int i = 0; i<steps; i++){
      //int i = 10;
      u = (double)i/(steps-1);
      uV[i] = u;

      val= SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveVal(u,xPts);
      d1 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,1);
      d2 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,2);
      d3 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,3);
      d4 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,4);
      d5 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,5);
      d6 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,xPts,6);

      analyticDerXU(i,0) = u;
      analyticDerXU(i,1) = val;
      analyticDerXU(i,2) = d1;
      analyticDerXU(i,3) = d2;
      analyticDerXU(i,4) = d3;
      analyticDerXU(i,5) = d4;
      analyticDerXU(i,6) = d5;
      analyticDerXU(i,7) = d6;

      val= SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveVal(u,yPts);
      d1 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,1);
      d2 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,2);
      d3 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,3);
      d4 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,4);
      d5 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,5);
      d6 = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivU(u,yPts,6);

      analyticDerYU(i,0) = u;
      analyticDerYU(i,1) = val;
      analyticDerYU(i,2) = d1;
      analyticDerYU(i,3) = d2;
      analyticDerYU(i,4) = d3;
      analyticDerYU(i,5) = d4;
      analyticDerYU(i,6) = d5;
      analyticDerYU(i,7) = d6;

     }

     int mxDU = 6-1;
     RigidBodyDynamics::Math::MatrixNd numericDer(analyticDerXU.rows(), mxDU);
     RigidBodyDynamics::Math::MatrixNd errorDer(analyticDerXU.rows(), mxDU);

     double tol = (double)(1.0/steps);
     tol = tol*tol*50; 
     //Numerical error in a central difference increases with the 
     //square of h. 
     //http://en.wikipedia.org/wiki/Finite_difference

     RigidBodyDynamics::Math::VectorNd domainX = 
      RigidBodyDynamics::Math::VectorNd::Zero(analyticDerXU.rows());
     RigidBodyDynamics::Math::VectorNd rangeY = 
      RigidBodyDynamics::Math::VectorNd::Zero(analyticDerXU.rows());

     RigidBodyDynamics::Math::VectorNd analyticDerYX = 
      RigidBodyDynamics::Math::VectorNd::Zero(analyticDerXU.rows());
     for(int j=0; j<analyticDerXU.rows(); j++){
      domainX[j] = analyticDerXU(j,0);
     }


     for(int i=0;i<mxDU;i++){

      for(int j=0; j<analyticDerXU.rows(); j++){
       rangeY[j]        = analyticDerXU(j,i+1);
       analyticDerYX[j] = analyticDerXU(j,i+2);
      }

      numericDer.col(i) = calcCentralDifference(domainX,
                                                rangeY,true); 
      for(int j=0; j<analyticDerYX.rows();++j){
        errorDer(j,i)  =  analyticDerYX(j,i)-numericDer(j,i);
      }

      //errorDer(i)= abs( errorDer(i).elementwiseDivide(numericDer(i)) );
      //The end points can't be tested because a central difference
      //cannot be accurately calculated at these locations
      for(int j=1; j<analyticDerXU.rows()-1; j++){
       assert( abs(errorDer(j,i))<tol );
       //if(errorDer(j,i)>tol)
       //printf("Error > Tol: (%i,%i): %f > %f\n",j,i,errorDer(j,i),tol);
      }
     }     
     //errorDer.cwiseAbs();
     //cout << errorDer << endl;

    //printf("...absolute tolerance of %f met\n", tol);

     //cout << "   TEST: Bezier Curve Derivative DYDX to d6y/dx6" << endl;
     RigidBodyDynamics::Math::MatrixNd numericDerXY(analyticDerXU.rows(), 6);
     RigidBodyDynamics::Math::MatrixNd analyticDerXY(analyticDerXU.rows(),6);

     for(int i=0; i< analyticDerXU.rows(); i++)
     {
      analyticDerXY(i,0) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,1);
      analyticDerXY(i,1) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,2);
      analyticDerXY(i,2) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,3);
      analyticDerXY(i,3) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,4);
      analyticDerXY(i,4) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,5);
      analyticDerXY(i,5) = SegmentedQuinticBezierToolkit::
       calcQuinticBezierCurveDerivDYDX(uV[i],xPts,yPts,6);
     }


     for(int j=0; j<numericDerXY.cols();j++){

      for(int k=0; k<numericDerXY.rows(); k++){
       domainX[k] = analyticDerXU(k,1);
       if(j == 0){
        rangeY[k]  = analyticDerYU(k,1);
       }else{
        rangeY[k]  = analyticDerXY(k,j-1);
       }
      }
      numericDerXY.col(j) = calcCentralDifference(domainX,
                       rangeY,true);
     
     }


     //Generate numerical derivative curves for the first 3 derivatives
     /*
     numericDerXY.col(0) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerYU.col(1),true);
     numericDerXY.col(1) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(0),true);
     numericDerXY.col(2) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(1),true);
     numericDerXY.col(3) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(2),true);
     numericDerXY.col(4) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(3),true);
     numericDerXY.col(5) = calcCentralDifference(analyticDerXU.col(1),
                     analyticDerXY.col(4),true);
     */

     //Create the matrix of errors
     RigidBodyDynamics::Math::MatrixNd errorDerXYNum(analyticDerXU.rows(), 6);
     RigidBodyDynamics::Math::MatrixNd errorDerXYDen(analyticDerXU.rows(), 6);
     RigidBodyDynamics::Math::MatrixNd errorDerXY(analyticDerXU.rows(), 6);

     for(int i = 0; i < errorDerXYNum.rows(); ++i){
        for(int j = 0; j < errorDerXYNum.cols(); ++j){
            errorDerXYNum(i,j) = abs( analyticDerXY(i,j)-numericDerXY(i,j));
            errorDerXYDen(i,j) = abs( analyticDerXY(i,j)+numericDerXY(i,j));
            errorDerXY(i,j)    = errorDerXYNum(i,j)/errorDerXYDen(i,j);
        }
     }

     double relTol = 5e-2;

     int relTolExceeded = 0;

     for(int j=0;j<6;j++){
      //can't test the first and last entries because a central diff.
      //cannot calculate these values accurately.
      for(int i=1;i<analyticDerXU.rows()-1;i++){
       if(errorDerXY(i,j)>relTol){
        //printf("Error > Tol: (%i,%i): %f > %f\n",i,j,
        //          errorDerXY(i,j),relTol);
        relTolExceeded++;
       }
      }
     }
     //cout << relTolExceeded << endl;

     //The relative tolerance gets exceeded occasionally in locations of
     //rapid change in the curve. Provided there are only a few locations
     //where the relative tolerance of 5% is broken, the curves should be
     //regarded as being good. Ten errors out of a possible 100*6 data points
     //seems relatively small.
     CHECK(relTolExceeded < maximumNumberOfToleranceViolations);


     //std::string fname = "analyticDerXY.csv";
     //printMatrixToFile(analyticDerXY,fname);
     //fname = "numericDerXY.csv";
     //printMatrixToFile(numericDerXY,fname);
     //fname = "errorDerXY.csv";
     //printMatrixToFile(errorDerXY,fname);
     //printf("   ...relative tolerance of %f not exceeded more than %i times\n"
     //    "   across all 6 derivatives, with 100 samples each\n",
     //        relTol, 10);
     //cout <<"**************************************************"<<endl;


}


   
TEST(QuinticBezierToolKitDerivatives)
{
    int maximumNumberOfToleranceViolations = 10;
    testSegmentedQuinticBezierDerivatives(10);
}

TEST(SmoothSegmentedFunctionProperties)
{
  //1. Make a fake monotonic curve
  RigidBodyDynamics::Math::VectorNd x(5);
  RigidBodyDynamics::Math::VectorNd y(5);
  RigidBodyDynamics::Math::VectorNd dydx(5);
  for(int i=0; i<x.size();++i){
    x[i]      = i*0.5*M_PI/(x.size()-1);  
    y[i]      = sin(x[i]) + x[i];
    dydx[i]   = cos(x[i]) + 1.0; 
  }
  double c = 0.5;
  
  RigidBodyDynamics::Math::MatrixNd mX(6,4), mY(6,4);
  RigidBodyDynamics::Math::MatrixNd p0(6,2);

  for(int i=0; i < 4; ++i){
    p0 = SegmentedQuinticBezierToolkit::
          calcQuinticBezierCornerControlPoints(  x[i],  y[i],  dydx[i],
                                               x[i+1],y[i+1],dydx[i+1],c);             
    mX.col(i)  = p0.col(0);
    mY.col(i)  = p0.col(1);
  }
  SmoothSegmentedFunction testCurve = SmoothSegmentedFunction();
  testCurve.updSmoothSegmentedFunction(   mX,     mY, 
                                        x[0],   x[4],
                                        y[0],   y[4],
                                     dydx[0],dydx[4],
                                     "testCurve");


  //2. Test key points.
  RigidBodyDynamics::Math::VectorNd yErr(5);
  RigidBodyDynamics::Math::VectorNd dydxErr(5);
  for(int i=0; i<x.size(); ++i){
    yErr[i]    = testCurve.calcValue(x[i]) - y[i];
    dydxErr[i] = testCurve.calcDerivative(x[i],1) - dydx[i];

    CHECK( abs(yErr[i])    < TOL_SMALL );
    CHECK( abs(dydxErr[i]) < TOL_SMALL );    
  }  

  //3. Test derivatives for numerical consistency
  RigidBodyDynamics::Math::MatrixNd testCurveSample
    = testCurve.calcSampledCurve( 6, x[0]-0.1, x[4]+0.1);

  bool areCurveDerivativesGood = 
     areCurveDerivativesCloseToNumericDerivatives(
      testCurve,  
      testCurveSample, 
      TOL_DX);

  CHECK(areCurveDerivativesGood);

  //4. Test C2 continuity
  bool curveIsContinuous = isCurveC2Continuous(testCurve,
                                               testCurveSample);
  CHECK(curveIsContinuous);

  //5. Test monotonicity
  bool curveIsMonotonic = isCurveMontonic(testCurveSample);
  CHECK(curveIsMonotonic);

}

int main (int argc, char *argv[])
{
    return UnitTest::RunAllTests ();
}
