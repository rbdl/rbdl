/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_BODY_TO_GROUND_POSITION_H
#define RBDL_BODY_TO_GROUND_POSITION_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Constraint.h>
#include <assert.h>

namespace RigidBodyDynamics {




class RBDL_DLLAPI BodyToGroundPositionConstraint : public Constraint {

public:

  //~BodyToGroundPositionConstraint(){};  
  BodyToGroundPositionConstraint();

  BodyToGroundPositionConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundConstraintUnitVector,
      const char *name = NULL);

  BodyToGroundPositionConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const std::vector< Math::Vector3d > &groundConstraintUnitVectors,
      const char *name = NULL);

  BodyToGroundPositionConstraint(
      const unsigned int indexOfConstraintInG,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundPoint,
      const std::vector< Math::Vector3d > &groundConstraintUnitVectors,
      const std::vector< bool > &positionLevelConstraint,
      const std::vector< bool > &velocityLevelConstraint,
      const char *name = NULL);

  void bind( const Model &model);

  void calcConstraintJacobian(  Model &model,
                                const Math::VectorNd &Q,
                                Math::MatrixNd &GSysUpd);

  void calcGamma( Model &model,
                  const Math::VectorNd &Q,
                  const Math::VectorNd &QDot,
                  const Math::MatrixNd &GSys,
                  Math::VectorNd &gammaSysUpd);


  void calcPositionError( Model &model,
                          const Math::VectorNd &Q,
                          Math::VectorNd &errSysUpd);

  void calcVelocityError( Model &model,
                          const Math::VectorNd &Q,
                          const Math::VectorNd &QDot,
                          const Math::MatrixNd &GSys,
                          Math::VectorNd &derrSysUpd);

  void calcConstraintForces(
        Model &model,
        const Math::VectorNd &Q,
        const Math::VectorNd &QDot,
        const Math::MatrixNd &GSys,
        const Math::VectorNd &lagrangeMultipliersSys,
        std::vector< unsigned int > &constraintBodiesUpd,
        std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
        std::vector< Math::SpatialVector > &constraintForcesUpd,
        bool resolveAllInRootFrame = false);


private:
  std::vector< Math::Vector3d > T;
  Math::MatrixNd XpJacobian3D;
  Math::Vector3d groundPoint;
  Math::Matrix3d matA;
  Math::Vector3d vecA;
  double dblA;
};





/** @} */

} 

/* namespace RigidBodyDynamics */

/* RBDL_BODY_TO_GROUND_POSITION_H */
#endif
