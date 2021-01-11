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

//#include <cassert>
using namespace RigidBodyDynamics::Addons::Balance;

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace RigidBodyDynamics::Math;
static double   FPETOLERANCE     = std::numeric_limits<double>::epsilon()*1e2;
static int      MAXITER = 12;


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
      bool flag_evaluateDerivatives,
      bool flag_verbose)
{

}