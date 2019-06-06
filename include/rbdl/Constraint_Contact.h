/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <millard.matthew@gmail.com>
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_CONTACT_CONSTRAINT_H
#define RBDL_CONTACT_CONSTRAINT_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Constraint.h>
#include <assert.h>

namespace RigidBodyDynamics {




class RBDL_DLLAPI ContactConstraint : public Constraint {

public:


  ContactConstraint();

  ContactConstraint(
      //const unsigned int rowInGSys,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundConstraintNormalVectors,
      bool positionLevelConstraint=false,
      bool velocityLevelConstraint=true,         
      const char *name = NULL);

  ContactConstraint(
      //const unsigned int rowInGSys,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const std::vector< Math::Vector3d > &groundConstraintNormalVectors,
      bool positionLevelConstraint=false,
      bool velocityLevelConstraint=true,               
      const char *name = NULL);

  ContactConstraint(
      //const unsigned int rowInGSys,
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundPoint,
      const std::vector< Math::Vector3d > &groundConstraintUnitVectors,
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

  const std::vector< Math::Vector3d >& getConstraintNormalVectors(){
    return T;
  }

  void appendNormalVector(const Math::Vector3d &normal,
                          bool positionLevelConstraint = false,
                          bool velocityLevelConstraint = true);

  //To support ForwardDynamicsKokkevis
  void calcPointAccelerations(Model &model,
                              const Math::VectorNd &Q,
                              const Math::VectorNd &QDot,
                              const Math::VectorNd &QDDot,
                              std::vector<Math::Vector3d> &pointAccelerationSysUpd,
                              bool updateKinematics=false);

  //To support ForwardDynamicsKokkevis
  void calcPointAccelerations(Model &model,
                              const Math::VectorNd &Q,
                              const Math::VectorNd &QDot,
                              const Math::VectorNd &QDDot,
                              Math::Vector3d &pointAccelerationUpd,
                              bool updateKinematics=false);


  //To support ForwardDynamicsKokkevis
  void calcPointAccelerationError(
                    const std::vector<Math::Vector3d> &pointAccelerationsSys,
                    Math::VectorNd &ddErrSysUpd);

  //To support ForwardDynamicsKokkevis
  void calcPointForceJacobian(
          Model &model,
          const Math::VectorNd &Q,
          ConstraintCache &cache,
          std::vector<Math::SpatialVector> &fExtSysUpd,
          bool updateKinematics=false);

private:
  std::vector< Math::Vector3d > T;
  Math::Vector3d groundPoint;
  double dblA;

};





/** @} */

} 

/* namespace RigidBodyDynamics */

/* RBDL_CONTACT_CONSTRAINT_H */
#endif
