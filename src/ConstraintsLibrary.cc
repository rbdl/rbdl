/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
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
#include "rbdl/ConstraintsLibrary.h"
#include "rbdl/Kinematics.h"

using namespace RigidBodyDynamics;
using namespace Math;



//==============================================================================
BodyToGroundPositionConstraint::BodyToGroundPositionConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundConstraintUnitVector,
      const char *name):
        Constraint(name,
                   ConstraintTypeBodyToGroundPosition,
                   indexOfConstraintInG,
                   unsigned(int(1)))
{

  assert( sizeOfConstraint <= 3 && sizeOfConstraint > 0);

  T.resize(sizeOfConstraint);  
  groundPoint = Math::Vector3dZero;
  dblA = 10.0*std::numeric_limits<double>::epsilon();

  for(unsigned int i=0; i<sizeOfConstraint;++i){
    //Check that all vectors in T are orthonormal
    T[i]=groundConstraintUnitVector;
    assert(std::fabs(T[i].norm()-1.0) <= dblA);
    if(i > 0){
      for(unsigned int j=0; j< i;++j){
        assert(std::fabs(T[i].dot(T[j])) <= dblA);
        }
    }
    //To make this consistent with the RBDL's ContactConstraints
    positionConstraint[i]=false;
    velocityConstraint[i]=true;
  }

  bodyIds.push_back(bodyId);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, bodyPoint));

  bodyIds.push_back(0);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, groundPoint));
}
BodyToGroundPositionConstraint::BodyToGroundPositionConstraint(
    const unsigned int indexOfConstraintInG,
    const unsigned int bodyId,
    const Math::Vector3d &bodyPoint,
    const std::vector< Math::Vector3d > &groundConstraintUnitVectors,
    const char *name):
      Constraint(name,
                 ConstraintTypeBodyToGroundPosition,
                 indexOfConstraintInG,
                 groundConstraintUnitVectors.size()),
      T(groundConstraintUnitVectors){

  assert( sizeOfConstraint <= 3 && sizeOfConstraint > 0);

  groundPoint = Math::Vector3dZero;
  dblA = 10.0*std::numeric_limits<double>::epsilon();

  for(unsigned int i=0; i<sizeOfConstraint;++i){
    //Check that all vectors in T are orthonormal
    assert(std::fabs(T[i].norm()-1.0) <= dblA);
    if(i > 0){
      for(unsigned int j=0; j< i;++j){
        assert(std::fabs(T[i].dot(T[j])) <= dblA);
        }
    }
    //To make this consistent with the RBDL's ContactConstraints
    positionConstraint[i]=false;
    velocityConstraint[i]=true;
  }

  bodyIds.push_back(bodyId);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, bodyPoint));

  bodyIds.push_back(0);
  bodyFrames.push_back(
        Math::SpatialTransform(Math::Matrix3dIdentity, groundPoint));
}

BodyToGroundPositionConstraint::BodyToGroundPositionConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundPoint,
      const std::vector< Math::Vector3d > &groundConstraintUnitVectors,
      const std::vector< bool > &positionLevelConstraint,
      const std::vector< bool > &velocityLevelConstraint,
      const char *name):
        Constraint(name,
                   ConstraintTypeBodyToGroundPosition,
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

void BodyToGroundPositionConstraint::bind(const Model &model)
{
    XpJacobian3D.resize(3, model.qdot_size);
}

void BodyToGroundPositionConstraint::calcConstraintJacobian( Model &model,
                              const Math::VectorNd &Q,
                              Math::MatrixNd &GSysUpd)
{

  CalcPointJacobian(model,Q,bodyIds[0],bodyFrames[0].r,XpJacobian3D,false);

  for(unsigned int i=0; i < sizeOfConstraint; ++i){
    GSysUpd.block(indexOfConstraintInG+i,0,1,GSysUpd.cols()) =
        T[i].transpose()*XpJacobian3D;
  }
}

void BodyToGroundPositionConstraint::calcGamma(  Model &model,
                  const Math::VectorNd &Q,
                  const Math::VectorNd &QDot,
                  const Math::MatrixNd &GSys,
                  Math::VectorNd &gammaSysUpd)
{
  gammaSysUpd.block(indexOfConstraintInG,0,
                    sizeOfConstraint,1).setZero();
}


void BodyToGroundPositionConstraint::calcPositionError(  Model &model,
                            const Math::VectorNd &Q,
                            Math::VectorNd &errSysUpd)
{
  vecA = CalcBodyToBaseCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,false)
          -groundPoint;
  for(unsigned int i = 0; i < sizeOfConstraint; ++i){
    if(positionConstraint[i]){
      errSysUpd[indexOfConstraintInG+i] = vecA.dot( T[i] );
    }else{
      errSysUpd[indexOfConstraintInG+i] = 0.;
    }
  }
}

void BodyToGroundPositionConstraint::calcVelocityError(  Model &model,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::MatrixNd &GSys,
                            Math::VectorNd &derrSysUpd)
{
  vecA =  CalcPointVelocity(model,Q,QDot,bodyIds[0],bodyFrames[0].r,false);
  for(unsigned int i = 0; i < sizeOfConstraint; ++i){
    if(velocityConstraint[i]){
      derrSysUpd[indexOfConstraintInG+i] = vecA.dot( T[i] );
    }else{
      derrSysUpd[indexOfConstraintInG+i] = 0.;
    }
  }
}

void BodyToGroundPositionConstraint::calcConstraintForces( 
              Model &model,
              const Math::VectorNd &Q,
              const Math::VectorNd &QDot,
              const Math::MatrixNd &GSys,
              const Math::VectorNd &LagrangeMultipliersSys,
              std::vector< unsigned int > &constraintBodiesUpd,
              std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
              std::vector< Math::SpatialVector > &constraintForcesUpd,
              bool resolveAllInRootFrame)
{

  //Size the vectors of bodies, local frames, and spatial vectors
  constraintBodiesUpd       = bodyIds;
  constraintBodyFramesUpd   = bodyFrames;

  vecA = CalcBodyToBaseCoordinates(model,Q,bodyIds[0],bodyFrames[0].r,false);

  if(resolveAllInRootFrame){
    constraintBodiesUpd[0] = constraintBodiesUpd[1];
    constraintBodyFramesUpd[0].r = vecA;
    constraintBodyFramesUpd[0].E = constraintBodyFramesUpd[1].E;
  }

  for(unsigned int i=0; i < sizeOfConstraint; ++i){
    //If this constraint direction is not enforced at the position level
    //update the reference position of the ground point
    if(positionConstraint[i]==false){
      constraintBodyFramesUpd[1].r += (vecA-bodyFrames[1].r).dot(T[i])*T[i];
    }
  }

  constraintForcesUpd.resize(bodyIds.size());
  for(unsigned int i=0; i< bodyIds.size(); ++i){
    constraintForcesUpd[i].setZero();
  }

  //Evaluate the total force the constraint applies to the contact point
  vecA.setZero();
  for(unsigned int i=0; i < sizeOfConstraint; ++i){
    vecA += T[i]*LagrangeMultipliersSys[indexOfConstraintInG+i];
  }

  //Update the forces applied to the body in the frame of the body
  if(resolveAllInRootFrame){
    constraintForcesUpd[0].block(3,0,3,1) = vecA;
  }else{
    matA = CalcBodyWorldOrientation(model,Q,bodyIds[0],false);
    constraintForcesUpd[0].block(3,0,3,1) = matA*vecA;
  }

  //Update the forces applied to the ground in the frame of the ground
  constraintForcesUpd[1].block(3,0,3,1) = -vecA;
}
//==============================================================================





