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
#include "rbdl/Constraint_Contact.h"
#include "rbdl/Kinematics.h"

using namespace RigidBodyDynamics;
using namespace Math;



//==============================================================================
ContactConstraint::ContactConstraint():
  Constraint("",ConstraintTypeContact,
              std::numeric_limits<unsigned int>::max(),1){}

//==============================================================================
ContactConstraint::ContactConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundConstraintUnitVector,
      bool positionLevelConstraint,
      bool velocityLevelConstraint,         
      const char *name):
        Constraint(name,
                   ConstraintTypeContact,
                   indexOfConstraintInG,
                   unsigned(int(1)))
{

  

  T.push_back(groundConstraintUnitVector); 
  dblA = std::numeric_limits<double>::epsilon()*10.;
  assert(std::fabs(T[0].norm()-1.0)<= dblA);

  groundPoint = Math::Vector3dZero;
  positionConstraint[0]=positionLevelConstraint;
  velocityConstraint[0]=velocityLevelConstraint;

  bodyIds.push_back(bodyId);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, bodyPoint));

  bodyIds.push_back(0);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, groundPoint));
}
ContactConstraint::ContactConstraint(
    const unsigned int indexOfConstraintInG,
    const unsigned int bodyId,
    const Math::Vector3d &bodyPoint,
    const std::vector< Math::Vector3d > &groundConstraintUnitVectors,
    bool positionLevelConstraint,
    bool velocityLevelConstraint,             
    const char *name):
      Constraint(name,
                 ConstraintTypeContact,
                 indexOfConstraintInG,
                 groundConstraintUnitVectors.size()),
      T(groundConstraintUnitVectors)
{
  assert( sizeOfConstraint <= 3 && sizeOfConstraint > 0);

  dblA = 10.0*std::numeric_limits<double>::epsilon();

  for(unsigned int i=0; i<sizeOfConstraint;++i){

    //Check that all vectors in T are orthonormal
    assert(std::fabs(T[i].norm()-1.0) <= dblA);
    if(i > 0){
      for(unsigned int j=0; j< i;++j){
        assert(std::fabs(T[i].dot(T[j])) <= dblA);
        }
    }

    //To make this consistent with the RBDL implementation of
    //ContactConstraints
    positionConstraint[i]=positionLevelConstraint;
    velocityConstraint[i]=velocityLevelConstraint;
  }

  bodyIds.push_back(bodyId);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, bodyPoint));
  bodyIds.push_back(0);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, groundPoint));

}

//==============================================================================

ContactConstraint::ContactConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundPoint,
      const std::vector< Math::Vector3d > &groundConstraintUnitVectors,
      const std::vector< bool > &positionLevelConstraint,
      const std::vector< bool > &velocityLevelConstraint,
      const char *name):
        Constraint(name,
                   ConstraintTypeContact,
                   indexOfConstraintInG,
                   groundConstraintUnitVectors.size()),
        T(groundConstraintUnitVectors)
{
  assert( sizeOfConstraint <= 3 && sizeOfConstraint > 0);

  dblA = 10.0*std::numeric_limits<double>::epsilon();

  for(unsigned int i=0; i<sizeOfConstraint;++i){

    //Check that all vectors in T are orthonormal
    assert(std::fabs(T[i].norm()-1.0) <= dblA);
    if(i > 0){
      for(unsigned int j=0; j< i;++j){
        assert(std::fabs(T[i].dot(T[j])) <= dblA);
        }
    }

    //To make this consistent with the RBDL implementation of
    //ContactConstraints
    positionConstraint[i]=positionLevelConstraint[i];
    velocityConstraint[i]=velocityLevelConstraint[i];
  }

  bodyIds.push_back(bodyId);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, bodyPoint));
  bodyIds.push_back(0);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, groundPoint));

}

//==============================================================================

void ContactConstraint::bind(const Model &model)
{

  //There are no dynamically-sized local matrices or vectors that
  //need to be adjusted for this constraint

}


//==============================================================================

void ContactConstraint::calcConstraintJacobian( Model &model,
                              const double time,
                              const Math::VectorNd &Q,
                              const Math::VectorNd &QDot,
                              Math::MatrixNd &GSysUpd,
                              ConstraintCache &cache,
                              bool updateKinematics)
{
  cache.mat3NA.setZero();
  CalcPointJacobian(model, Q,bodyIds[0],bodyFrames[0].r,cache.mat3NA,
                    updateKinematics);

  for(unsigned int i=0; i < sizeOfConstraint; ++i){
    GSysUpd.block(indexOfConstraintInG+i,0,1,GSysUpd.cols()) =
        T[i].transpose()*cache.mat3NA;
  }
}

//==============================================================================

void ContactConstraint::calcGamma(  Model &model,
                  const double time,
                  const Math::VectorNd &Q,
                  const Math::VectorNd &QDot,
                  const Math::MatrixNd &GSys,
                  Math::VectorNd &gammaSysUpd,
                  ConstraintCache &cache,
                  bool updateKinematics)
{


  cache.vec3A = CalcPointAcceleration (model, Q, QDot, cache.vecNZeros,
                                       bodyIds[0], bodyFrames[0].r,
                                       updateKinematics);

  for(unsigned int i=0; i < sizeOfConstraint; ++i){
    gammaSysUpd.block(indexOfConstraintInG+i,0,1,1) =
          -T[i].transpose()*cache.vec3A;
  }
}


//==============================================================================


void ContactConstraint::calcPositionError(Model &model,
                                                      const double time,
                                                      const Math::VectorNd &Q,
                                                      Math::VectorNd &errSysUpd,
                                                      ConstraintCache &cache,
                                                      bool updateKinematics)
{
  cache.vec3A = CalcBodyToBaseCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,
                                          updateKinematics)  - groundPoint;
  for(unsigned int i = 0; i < sizeOfConstraint; ++i){
    if(positionConstraint[i]){
      errSysUpd[indexOfConstraintInG+i] = cache.vec3A.dot( T[i] );
    }else{
      errSysUpd[indexOfConstraintInG+i] = 0.;
    }
  }
}

//==============================================================================

void ContactConstraint::calcVelocityError(  Model &model,
                            const double time,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::MatrixNd &GSys,
                            Math::VectorNd &derrSysUpd,
                            ConstraintCache &cache,
                            bool updateKinematics)
{
  cache.vec3A =  CalcPointVelocity(model,Q,QDot,bodyIds[0],bodyFrames[0].r,
                                    updateKinematics);
  for(unsigned int i = 0; i < sizeOfConstraint; ++i){
    if(velocityConstraint[i]){
      derrSysUpd[indexOfConstraintInG+i] = cache.vec3A.dot( T[i] );
    }else{
      derrSysUpd[indexOfConstraintInG+i] = 0.;
    }
  }
}

//==============================================================================

void ContactConstraint::calcConstraintForces( 
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

  //Size the vectors of bodies, local frames, and spatial vectors
  constraintBodiesUpd       = bodyIds;
  constraintBodyFramesUpd   = bodyFrames;

  cache.vec3A = CalcBodyToBaseCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,
                                    updateKinematics);

  if(resolveAllInRootFrame){
    constraintBodiesUpd[0] = constraintBodiesUpd[1];
    constraintBodyFramesUpd[0].r = cache.vec3A;
    constraintBodyFramesUpd[0].E = constraintBodyFramesUpd[1].E;
  }

  for(unsigned int i=0; i < sizeOfConstraint; ++i){
    //If this constraint direction is not enforced at the position level
    //update the reference position of the ground point
    if(positionConstraint[i]==false){
      constraintBodyFramesUpd[1].r +=
          (cache.vec3A-bodyFrames[1].r).dot(T[i])*T[i];
    }
  }

  constraintForcesUpd.resize(bodyIds.size());
  for(unsigned int i=0; i< bodyIds.size(); ++i){
    constraintForcesUpd[i].setZero();
  }
  //Evaluate the total force the constraint applies to the contact point
  cache.vec3A.setZero();
  for(unsigned int i=0; i < sizeOfConstraint; ++i){
    cache.vec3A += T[i]*LagrangeMultipliersSys[indexOfConstraintInG+i];
  }

  //Update the forces applied to the body in the frame of the body
  if(resolveAllInRootFrame){
    constraintForcesUpd[0].block(3,0,3,1) = cache.vec3A;
  }else{
    cache.mat3A = CalcBodyWorldOrientation(model,Q,bodyIds[0],false);
    constraintForcesUpd[0].block(3,0,3,1) = cache.mat3A*cache.vec3A;
  }

  //Update the forces applied to the ground in the frame of the ground
  constraintForcesUpd[1].block(3,0,3,1) = -cache.vec3A;
}
//==============================================================================
void ContactConstraint::
        appendNormalVector(const Math::Vector3d& normal,
                           bool posLevel, bool velLevel)
{
  dblA = 10.0*std::numeric_limits<double>::epsilon();

  //Make sure the normal is valid
  assert( std::fabs(normal.norm()-1.) < dblA);
  for(unsigned int i=0; i<sizeOfConstraint;++i){
    assert(std::fabs(T[i].dot(normal)) <= dblA);
  }

  T.push_back(normal);
  positionConstraint.push_back(posLevel);
  velocityConstraint.push_back(velLevel);
  sizeOfConstraint++;

  assert( sizeOfConstraint <= 3 && sizeOfConstraint > 0);

}

//==============================================================================

void ContactConstraint::
      calcPointAccelerations(Model &model,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::VectorNd &QDDot,
                            std::vector<Math::Vector3d> &pointAccelerationsSysUpd,
                            bool updateKinematics)
{
  pointAccelerationsSysUpd[indexOfConstraintInG] =
      CalcPointAcceleration (model, Q, QDot, QDDot, bodyIds[0],
                                      bodyFrames[0].r, updateKinematics);
  for(unsigned int i=1; i<sizeOfConstraint;++i){
    pointAccelerationsSysUpd[indexOfConstraintInG+i] =
        pointAccelerationsSysUpd[indexOfConstraintInG];
  }
}

//==============================================================================

void ContactConstraint::
      calcPointAccelerations(Model &model,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::VectorNd &QDDot,
                            Math::Vector3d &pointAccelerationsUpd,
                            bool updateKinematics)
{
  pointAccelerationsUpd = CalcPointAcceleration (model, Q, QDot, QDDot,
                                                 bodyIds[0], bodyFrames[0].r,
                                                 updateKinematics);
}

//==============================================================================

void ContactConstraint::
      calcPointAccelerationError(
                    const std::vector<Math::Vector3d> &pointAccelerationsSys,
                    Math::VectorNd &ddErrSysUpd)
{
  for(unsigned int i=0; i<sizeOfConstraint;++i){
    ddErrSysUpd[indexOfConstraintInG+i] =
        T[i].dot(pointAccelerationsSys[indexOfConstraintInG+i]);
  }
}

void ContactConstraint::
      calcPointForceJacobian(
        Model &model,
        const Math::VectorNd &Q,
        ConstraintCache &cache,
        std::vector<Math::SpatialVector> &fExtSysUpd,
        bool updateKinematics)
{
  cache.vec3A = CalcBodyToBaseCoordinates(
                  model,Q,bodyIds[0],bodyFrames[0].r,updateKinematics);
  cache.stA.E.Identity();
  cache.stA.r = -cache.vec3A;
  cache.svecA[0]=0.;
  cache.svecA[1]=0.;
  cache.svecA[2]=0.;
  for(unsigned int i=0; i<sizeOfConstraint;++i){
    cache.svecA[3] = -T[i][0];
    cache.svecA[4] = -T[i][1];
    cache.svecA[5] = -T[i][2];
    fExtSysUpd[indexOfConstraintInG+i] = cache.stA.applyAdjoint( cache.svecA );
  }
}



