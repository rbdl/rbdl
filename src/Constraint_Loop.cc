/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <sstream>
#include <limits>
#include <assert.h>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Constraint_Loop.h"
#include "rbdl/Kinematics.h"

using namespace RigidBodyDynamics;
using namespace Math;



//==============================================================================
LoopConstraint::LoopConstraint():
  Constraint("",ConstraintTypeLoopNew,
              std::numeric_limits<unsigned int>::max(),1){}

//==============================================================================
LoopConstraint::LoopConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyIdPredecessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const Math::SpatialVector &constraintAxis,
      bool positionLevelConstraint,
      bool velocityLevelConstraint,            
      const char *name):
        Constraint(name,
                   ConstraintTypeLoopNew,
                   indexOfConstraintInG,
                   unsigned(int(1)))
{
  T.push_back(constraintAxis);
  dblA = std::numeric_limits<double>::epsilon()*10.;
  assert(std::fabs(T[0].norm()-1.0)<= dblA);

  positionConstraint[0]=positionLevelConstraint;
  velocityConstraint[0]=velocityLevelConstraint;

  bodyIds.push_back(bodyIdPredecessor);
  bodyIds.push_back(bodyIdSuccessor);

  bodyFrames.push_back(bodyFramePredecessor);
  bodyFrames.push_back(bodyFrameSuccessor);

}
//==============================================================================
LoopConstraint::LoopConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyIdPredecessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const std::vector< Math::SpatialVector > &constraintAxes,
      bool positionLevelConstraint,
      bool velocityLevelConstraint,            
      const char *name):
      Constraint(name,
                 ConstraintTypeLoopNew,
                 indexOfConstraintInG,
                 unsigned (constraintAxes.size())),
      T(constraintAxes)
{

  assert( sizeOfConstraint <= 6 && sizeOfConstraint > 0);
  dblA = std::numeric_limits<double>::epsilon()*10.;

  for(unsigned int i = 0; i<sizeOfConstraint;++i){

    //Check that all vectors in T are orthonormal    
    assert(std::fabs(T[i].norm()-1.0) <= dblA);
    if(i > 0){
      for(unsigned int j=0; j< i;++j){
        assert(std::fabs(T[i].dot(T[j])) <= dblA);
        }
    }

    positionConstraint[i] = positionLevelConstraint;
    velocityConstraint[i] = velocityLevelConstraint;
  }

  bodyIds.push_back(bodyIdPredecessor);
  bodyIds.push_back(bodyIdSuccessor);

  bodyFrames.push_back(bodyFramePredecessor);
  bodyFrames.push_back(bodyFrameSuccessor);


}

//==============================================================================
LoopConstraint::LoopConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyIdPredecessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const std::vector< Math::SpatialVector > &constraintAxes,
      const std::vector< bool > &positionLevelConstraint,
      const std::vector< bool > &velocityLevelConstraint,      
      const char *name):
      Constraint(name,
                 ConstraintTypeLoopNew,
                 indexOfConstraintInG,
                 unsigned(constraintAxes.size()) ),
      T(constraintAxes)
{

  assert( sizeOfConstraint <= 6 && sizeOfConstraint > 0);
  dblA = std::numeric_limits<double>::epsilon()*10.;

  for(unsigned int i = 0; i<sizeOfConstraint;++i){

    //Check that all vectors in T are orthonormal    
    assert(std::fabs(T[i].norm()-1.0) <= dblA);
    if(i > 0){
      for(unsigned int j=0; j< i;++j){
        assert(std::fabs(T[i].dot(T[j])) <= dblA);
        }
    }

    positionConstraint[i] = positionLevelConstraint[i];
    velocityConstraint[i] = velocityLevelConstraint[i];
  }

  bodyIds.push_back(bodyIdPredecessor);
  bodyIds.push_back(bodyIdSuccessor);

  bodyFrames.push_back(bodyFramePredecessor);
  bodyFrames.push_back(bodyFrameSuccessor);


}

//==============================================================================


//==============================================================================

void LoopConstraint::bind(const Model &model)
{
  //The ConstraintCache input to each function contains all of the working
  //memory necessary for this constraint. So nothing appears here.
}


//==============================================================================

void LoopConstraint::calcConstraintJacobian( Model &model,
                              const double time,
                              const Math::VectorNd &Q,
                              const Math::VectorNd &QDot,
                              Math::MatrixNd &GSysUpd,
                              ConstraintCache &cache,
                              bool updKin)
{
    //Compute the spatial Jacobians of the predecessor point Gp and the
    //successor point Gs and evaluate Gs-Gp
    cache.mat6NA.setZero();
    cache.mat6NB.setZero();
    CalcPointJacobian6D(model,Q,bodyIds[0],bodyFrames[0].r,cache.mat6NA,updKin);
    CalcPointJacobian6D(model,Q,bodyIds[1],bodyFrames[1].r,cache.mat6NB,updKin);
    cache.mat6NA = cache.mat6NB-cache.mat6NA;

    //Evaluate the transform from the world frame into the constraint frame
    //that is attached to the precessor body
    cache.stA.r =
        CalcBaseToBodyCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,updKin);
    cache.stA.E = CalcBodyWorldOrientation(model,Q,bodyIds[0],updKin
                                           ).transpose()*bodyFrames[0].E;

    for(unsigned int i=0; i<sizeOfConstraint;++i){
      //Resolve each constraint axis into the global frame
      cache.svecA =cache.stA.apply(T[i]);

      //Take the dot product of the constraint axis with Gs-Gp
      GSysUpd.block(indexOfConstraintInG+i,0,1,GSysUpd.cols())
          = cache.svecA.transpose()*cache.mat6NA;
    }

}

//==============================================================================

void LoopConstraint::calcGamma(  Model &model,
                  const double time,
                  const Math::VectorNd &Q,
                  const Math::VectorNd &QDot,
                  const Math::MatrixNd &GSys,
                  Math::VectorNd &gammaSysUpd,
                  ConstraintCache &cache,
                  bool updateKinematics)
{

  // Express the constraint axis in the base frame.

  //pos_p = CalcBodyToBaseCoordinates (model, Q, CS.body_p[c],
  //                                  CS.X_p[c].r,false);
  //rot_p = CalcBodyWorldOrientation (model, Q, CS.body_p[c], false
  //                                    ).transpose() * CS.X_p[c].E;
  //axis = SpatialTransform (rot_p, pos_p).apply(CS.constraintAxis[c]);

  // Compute the spatial velocities of the two constrained bodies.

  //vel_p = CalcPointVelocity6D (model, Q, QDot, CS.body_p[c],
  //                              CS.X_p[c].r, false);
  //vel_s = CalcPointVelocity6D (model, Q, QDot, CS.body_s[c],
  //                              CS.X_s[c].r, false);

  // Compute the derivative of the axis wrt the base frame.

  //SpatialVector axis_dot = crossm(vel_p, axis);

  // Compute the velocity product accelerations. These correspond to the
  // accelerations that the bodies would have if q ddot were 0.

  //SpatialVector acc_p = CalcPointAcceleration6D (model, Q, QDot
  //  , VectorNd::Zero(model.dof_count), CS.body_p[c], CS.X_p[c].r, false);
  //SpatialVector acc_s = CalcPointAcceleration6D (model, Q, QDot
  //  , VectorNd::Zero(model.dof_count), CS.body_s[c], CS.X_s[c].r, false);

  // Problem here if one of the bodies is fixed...
  // Compute the value of gamma.

  //CS.gamma[c]
  //  = - axis.dot(acc_s - acc_p) - axis_dot.dot(vel_s - vel_p)

}


//==============================================================================


void LoopConstraint::calcPositionError(Model &model,
                                                      const double time,
                                                      const Math::VectorNd &Q,
                                                      Math::VectorNd &errSysUpd,
                                                      ConstraintCache &cache,
                                                      bool updateKinematics)
{

}

//==============================================================================

void LoopConstraint::calcVelocityError(  Model &model,
                            const double time,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::MatrixNd &GSys,
                            Math::VectorNd &derrSysUpd,
                            ConstraintCache &cache,
                            bool updateKinematics)
{

}

//==============================================================================

void LoopConstraint::calcConstraintForces( 
              Model &model,
              const double time,
              const Math::VectorNd &Q,
              const Math::VectorNd &QDot,
              const Math::MatrixNd &GSys,
              const Math::VectorNd &LagrangeMultipliersSys,
              std::vector< unsigned int > &constraintBodiesUpd,
              std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
              std::vector< Math::SpatialVector > &constraintForcesUpd,
              ConstraintCache &cache,
              bool resolveAllInRootFrame,
              bool updateKinematics)
{


}
//==============================================================================
void LoopConstraint::
        appendConstraintAxisconst Math::SpatialVector &constraintAxis,
                          bool positionLevelConstraint = false,
                          bool velocityLevelConstraint = true)
{

}

