/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <millard.matthew@gmail.com>
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_LOOP_CONSTRAINT_H
#define RBDL_LOOP_CONSTRAINT_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Constraint.h>
#include <assert.h>

namespace RigidBodyDynamics {




class RBDL_DLLAPI LoopConstraint : public Constraint {

public:


  LoopConstraint();

  LoopConstraint(
      //const unsigned int rowInSystem,
      const unsigned int bodyIdPredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const Math::SpatialVector &constraintAxis,
      bool positionLevelConstraint=true,
      bool velocityLevelConstraint=true,      
      const char *name = NULL);

  LoopConstraint(
      //const unsigned int rowInSystem,
      const unsigned int bodyIdPredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const std::vector< Math::SpatialVector > &constraintAxes,
      bool positionLevelConstraint=true,
      bool velocityLevelConstraint=true,            
      const char *name = NULL);

  LoopConstraint(
      //const unsigned int rowInSystem,
      const unsigned int bodyIdPredecessor,
      const unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &bodyFramePredecessor,
      const Math::SpatialTransform &bodyFrameSuccessor,
      const std::vector< Math::SpatialVector > &constraintAxes,
      const std::vector< bool > &positionLevelConstraint,
      const std::vector< bool > &velocityLevelConstraint,
      const char *name = NULL);

  void bind( const Model &model) override;

  void calcConstraintJacobian(  Model &model,
                                const double time,
                                const Math::VectorNd &Q,
                                const Math::VectorNd &QDot,
                                Math::MatrixNd &GSysUpd,
                                ConstraintCache &cache,
                                bool updateKinematics=false) override;

  void calcGamma( Model &model,
                  const double time,
                  const Math::VectorNd &Q,
                  const Math::VectorNd &QDot,
                  const Math::MatrixNd &GSys,
                  Math::VectorNd &gammaSysUpd,
                  ConstraintCache &cache,
                  bool updateKinematics=false) override;

  void calcPositionError( Model &model,
                          const double time,
                          const Math::VectorNd &Q,
                          Math::VectorNd &errSysUpd,
                          ConstraintCache &cache,
                          bool updateKinematics=false) override;

  void calcVelocityError( Model &model,
                          const double time,
                          const Math::VectorNd &Q,
                          const Math::VectorNd &QDot,
                          const Math::MatrixNd &GSys,
                          Math::VectorNd &derrSysUpd,
                          ConstraintCache &cache,
                          bool updateKinematics=false) override;

  void calcConstraintForces(
        Model &model,
        const double time,
        const Math::VectorNd &Q,
        const Math::VectorNd &QDot,
        const Math::MatrixNd &GSys,
        const Math::VectorNd &lagrangeMultipliersSys,
        std::vector< unsigned int > &constraintBodiesUpd,
        std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
        std::vector< Math::SpatialVector > &constraintForcesUpd,
        ConstraintCache &cache,
        bool resolveAllInRootFrame = false,
        bool updateKinematics=false) override;

  //==========================================================

  const std::vector< Math::SpatialVector >& getConstraintAxes(){
    return T;
  }

  void appendConstraintAxis(const Math::SpatialVector &constraintAxis,
                          bool positionLevelConstraint = false,
                          bool velocityLevelConstraint = true);


private:
  std::vector< Math::SpatialVector > T;
  double dblA;
};

} 

/* namespace RigidBodyDynamics */

/* RBDL_LOOP_CONSTRAINT_H */
#endif
