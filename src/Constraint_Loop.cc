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
  Constraint("",ConstraintTypeLoopNew,unsigned(int(0))){}

//==============================================================================
LoopConstraint::LoopConstraint(
      //const unsigned int indexOfConstraintInG,
      const unsigned int bodyIdPredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const Math::SpatialVector &constraintAxis,
      bool positionLevelConstraint,
      bool velocityLevelConstraint,            
      const char *name):
        Constraint(name,
                   ConstraintTypeLoopNew,
                   unsigned(int(1)))
{
  //connectToConstraintSet(indexOfConstraintInG, unsigned(int(1)));

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
      //const unsigned int indexOfConstraintInG,
      const unsigned int bodyIdPredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const std::vector< Math::SpatialVector > &constraintAxes,
      bool positionLevelConstraint,
      bool velocityLevelConstraint,            
      const char *name):
      Constraint( name,
                  ConstraintTypeLoopNew,
                  unsigned(constraintAxes.size())),
      T(constraintAxes)
{

  //connectToConstraintSet(indexOfConstraintInG,
  //                       unsigned(constraintAxes.size()));

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
      //const unsigned int indexOfConstraintInG,
      const unsigned int bodyIdPredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const std::vector< Math::SpatialVector > &constraintAxes,
      const std::vector< bool > &positionLevelConstraint,
      const std::vector< bool > &velocityLevelConstraint,      
      const char *name):
      Constraint(name,
                 ConstraintTypeLoopNew,
                 unsigned(constraintAxes.size())),
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
  //There are no dynamically-sized local matrices or vectors that
  //need to be adjusted for this Constraint - the dynamic members in
  //the ConstraintCache are enough.
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
    //Please refer to Ch. 8 of Featherstone's Rigid Body Dynamics for details

    //Compute the spatial Jacobians of the predecessor point Gp and the
    //successor point Gs and evaluate Gs-Gp
    cache.mat6NA.setZero();
    cache.mat6NB.setZero();
    CalcPointJacobian6D(model,Q,bodyIds[0],bodyFrames[0].r,cache.mat6NA,updKin);
    CalcPointJacobian6D(model,Q,bodyIds[1],bodyFrames[1].r,cache.mat6NB,updKin);
    cache.mat6NA = cache.mat6NB-cache.mat6NA;

    //Evaluate the transform from the world frame into the constraint frame
    //that is attached to the precessor body
    cache.stA.r = CalcBodyToBaseCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,
                                            updKin);
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
                  bool updKin)
{
  //Please refer to Ch. 8 of Featherstone's Rigid Body Dynamics text for details

  // Express the constraint axis in the base frame.
  cache.stA.r = CalcBodyToBaseCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,
                                          updKin);
  cache.stA.E = CalcBodyWorldOrientation(model,Q,bodyIds[0],updKin
                                         ).transpose()*bodyFrames[0].E;

  // Compute the spatial velocities of the two constrained bodies.
  //vel_p
  cache.svecA = CalcPointVelocity6D(model,Q,QDot,bodyIds[0],bodyFrames[0].r,
                                    updKin);

  //vel_s
  cache.svecB = CalcPointVelocity6D(model,Q,QDot,bodyIds[1],bodyFrames[1].r,
                                    updKin);

  // Compute the velocity product accelerations. These correspond to the
  // accelerations that the bodies would have if q ddot were 0. If this
  // confuses you please see Sec. 8.2 of Featherstone's Rigid Body Dynamics text

  //acc_p
  cache.svecC = CalcPointAcceleration6D(model,Q,QDot,cache.vecNZeros,
                                        bodyIds[0],bodyFrames[0].r,updKin);
  //acc_s
  cache.svecD = CalcPointAcceleration6D(model,Q,QDot,cache.vecNZeros,
                                        bodyIds[1],bodyFrames[1].r,updKin);

  for(unsigned int i=0; i<sizeOfConstraint;++i){

    axis = cache.stA.apply(T[i]);

    axisDot = crossm(cache.svecA, axis);

    gammaSysUpd[indexOfConstraintInG+i] =
        -axis.dot(cache.svecD-cache.svecC)
        -axisDot.dot(cache.svecB-cache.svecA);
  }

}


//==============================================================================


void LoopConstraint::calcPositionError(Model &model,
                                                      const double time,
                                                      const Math::VectorNd &Q,
                                                      Math::VectorNd &errSysUpd,
                                                      ConstraintCache &cache,
                                                      bool updKin)
{

  // Constraints computed in the predecessor body frame.


  // Compute the position of the two contact points.
  //pos_p = CalcBodyToBaseCoordinates (model, Q, CS.body_p[lci], CS.X_p[lci].r
  //  , false);
  //pos_s = CalcBodyToBaseCoordinates (model, Q, CS.body_s[lci], CS.X_s[lci].r
  //  , false);

  // Compute the orientation of the two constraint frames.
  //rot_p = CalcBodyWorldOrientation (model, Q, CS.body_p[lci], false).transpose()
  //  * CS.X_p[lci].E;
  //rot_s = CalcBodyWorldOrientation (model, Q, CS.body_s[lci], false).transpose()
  //  * CS.X_s[lci].E;

  //Kp: predecessor frame
  cache.stA.r = CalcBodyToBaseCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,
                                          updKin);
  cache.stA.E = CalcBodyWorldOrientation(model,Q,bodyIds[0],updKin
                                         ).transpose()*bodyFrames[0].E;

  //Ks: successor frame
  cache.stB.r = CalcBodyToBaseCoordinates(model,Q,bodyIds[1],bodyFrames[1].r,
                                          updKin);
  cache.stB.E = CalcBodyWorldOrientation(model,Q,bodyIds[1],updKin
                                         ).transpose()*bodyFrames[1].E;


  // Compute the orientation from the predecessor to the successor frame.

  cache.mat3A = cache.stA.E.transpose()*cache.stB.E;
  //rot_ps = rot_p.transpose() * rot_s;


  // The first three elements represent the rotation error.
  // This formulation is equivalent to u * sin(theta), where u and theta are
  // the angle-axis of rotation from the predecessor to the successor frame.
  // These quantities are expressed in the predecessor frame. This is also
  // identical to the rotation error calculation that appears in Table 8.1 of
  // Featherstone.
  cache.svecA[0] = -0.5*(cache.mat3A(1,2)-cache.mat3A(2,1));
  cache.svecA[1] = -0.5*(cache.mat3A(2,0)-cache.mat3A(0,2));
  cache.svecA[2] = -0.5*(cache.mat3A(0,1)-cache.mat3A(1,0));

  // The last three elements represent the position error.
  // It is equivalent to the difference in the position of the two
  // constraint points. The distance is projected on the predecessor frame
  // to be consistent with the rotation.

  //Qn: Should this be multiplied by -0.5 to be consistent with table 8.1?
  //For now I'm leaving this as is: this is equivalent to the functioning
  //original loop constraint code.
  cache.svecA.block(3,0,3,1)=cache.stA.E.transpose()*(cache.stB.r-cache.stA.r);

  //d.block<3,1>(3,0) = rot_p.transpose() * (pos_s - pos_p);
  // Project the error on the constraint axis to find the actual error.
  //err[lci] = CS.constraintAxis[lci].transpose() * d;

  for(unsigned int i=0; i<sizeOfConstraint;++i){
    if(positionConstraint[i]){
      errSysUpd[indexOfConstraintInG+i] = T[i].transpose()*cache.svecA;
    }else{
      errSysUpd[indexOfConstraintInG+i] = 0.;
    }
  }

}

//==============================================================================

void LoopConstraint::calcVelocityError(  Model &model,
                            const double time,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::MatrixNd &GSys,
                            Math::VectorNd &derrSysUpd,
                            ConstraintCache &cache,
                            bool updKin)
{
  //SimpleMath cannot handle multiplying a block matrix by a vector
  //Using a for loop here to maintain backwards compatibility.
  //Rant: all of this ugliness for a dot product! Does anyone even use
  //      SimpleMath?

  //derrSysUpd.block(indexOfConstraintInG,0,sizeOfConstraint,1) =
  //    GSys.block(indexOfConstraintInG,0,sizeOfConstraint,GSys.cols())*QDot;

  for(unsigned int i=0; i<sizeOfConstraint;++i){
    derrSysUpd[indexOfConstraintInG+i] = 0;
    for(unsigned int j=0; j<GSys.cols();++j){
      derrSysUpd[indexOfConstraintInG+i] +=
           GSys(indexOfConstraintInG+i,j)*QDot[j];
    }
  }

}

//==============================================================================

void LoopConstraint::calcConstraintForces( 
              Model &model,
              const double time,
              const Math::VectorNd &Q,
              const Math::VectorNd &QDot,
              const Math::MatrixNd &GSys,
              const Math::VectorNd &LagMultSys,
              std::vector< unsigned int > &constraintBodiesUpd,
              std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
              std::vector< Math::SpatialVector > &constraintForcesUpd,
              ConstraintCache &cache,
              bool resolveAllInRootFrame,
              bool updKin)
{
  assert(bodyIds.size()==2);
  assert(bodyFrames.size() == 2);

  cache.stA.r = CalcBaseToBodyCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,
                                          updKin);
  cache.stA.E = CalcBodyWorldOrientation(model,Q,bodyIds[0],updKin
                                         ).transpose()*bodyFrames[0].E;

  cache.stB.r = CalcBaseToBodyCoordinates(model,Q,bodyIds[1],bodyFrames[1].r,
                                          updKin);
  cache.stB.E = CalcBodyWorldOrientation(model,Q,bodyIds[1],updKin
                                         ).transpose()*bodyFrames[1].E;

  constraintForcesUpd.resize(2);
  constraintForcesUpd[0].setZero();
  constraintForcesUpd[1].setZero();

  //Using Eqn. 8.30 of Featherstone. Note that this force is resolved in the
  //predecessor frame
  for(unsigned int i=0; i<sizeOfConstraint;++i){
    constraintForcesUpd[0] += T[i]*LagMultSys[indexOfConstraintInG+i];
  }

  constraintBodiesUpd.resize(2);
  constraintBodyFramesUpd.resize(2);

  if(resolveAllInRootFrame){
    constraintBodiesUpd[0] = 0;
    constraintBodiesUpd[1] = 0;

    //These forces are returned in the coordinates of the
    //root frame but w.r.t. the respective points of the constaint
    constraintBodyFramesUpd[0].r = -cache.stA.r;
    constraintBodyFramesUpd[0].E.Identity();
    //constraintBodyFramesUpd[0].E.setIdentity();
    constraintBodyFramesUpd[1].r = -cache.stB.r;
    constraintBodyFramesUpd[1].E.Identity();
    //constraintBodyFramesUpd[1].E.setIdentity();

    //Rotate the forces from the predecessor body frame to the
    //root frame.
    constraintForcesUpd[0].block(0,0,3,1) =
        cache.stA.E*constraintForcesUpd[0].block(0,0,3,1);
    constraintForcesUpd[0].block(3,0,3,1) =
        cache.stA.E*constraintForcesUpd[0].block(3,0,3,1);

    //The forces applied to the successor body are equal and opposite
    constraintForcesUpd[1] = -constraintForcesUpd[0];

  }else{
    constraintBodiesUpd     = bodyIds;
    constraintBodyFramesUpd = bodyFrames;

    //The forces applied to the predecessor frame are already in
    //the correct coordinates. The forces of applied to the successor
    //frame are equal and opposite, but in the successor frame.
    cache.mat3A = cache.stB.E.transpose()*cache.stA.E;
    constraintForcesUpd[1].block(0,0,3,1) =
        -cache.mat3A*constraintForcesUpd[0].block(0,0,3,1);
    constraintForcesUpd[1].block(3,0,3,1) =
        -cache.mat3A*constraintForcesUpd[0].block(3,0,3,1);

  }



}
//==============================================================================
void LoopConstraint::
        appendConstraintAxis( const Math::SpatialVector &constraintAxis,
                              bool positionLevelConstraint,
                              bool velocityLevelConstraint)
{

  dblA = 10.0*std::numeric_limits<double>::epsilon();

  //Make sure the normal is valid
  assert( std::fabs(constraintAxis.norm()-1.) < dblA);
  for(unsigned int i=0; i<sizeOfConstraint;++i){
    assert(std::fabs(T[i].dot(constraintAxis)) <= dblA);
  }

  T.push_back(constraintAxis);
  positionConstraint.push_back(positionLevelConstraint);
  velocityConstraint.push_back(velocityLevelConstraint);
  sizeOfConstraint++;

  assert(sizeOfConstraint <= 6 && sizeOfConstraint > 0);
}

