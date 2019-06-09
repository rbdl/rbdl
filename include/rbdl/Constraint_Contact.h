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

  /**

    @param bodyId the body which is affected directly by the constraint

    @param bodyPoint the point that is constrained relative to the
            contact body

    @param groundConstraintNormalVectors the normal direction in which to apply 
            the constraint

     @param name a human readable name (optional, default: NULL).
            Set this field to a unique name (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.

     @param userDefinedId a user defined id (optional, defaults to max()).
            Set this field to a unique number (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.

    @param enableBaumgarteStabilization (optional, default false) setting this
            flag to true will modify the right hand side of the acceleration
            equation with a penaltiy term that is proportional to the constraint
            error scaled by a constant.

    @param stabilizationTimeConstant (optional, defaults to 0.1 sec) this
            value scales the strength of Baumgarte stabilization so that the
            settling time of the error is proportional the value given here.

    @param velocityLevelConstraint (advanced, optional, defaults to true) :
                This flag controls whether or not velocity errors are computed
                for this constraint. When velocity errors are computed
                they are used by CalcAssemblyQDot (to assemble this constraint
                at the velocity level) and by Baumgarte stabilization (if it is
                enabled) to modify the right hand side of the acceleration
                equation with a penalty term proportional to error. To be
                consistent with the original RBDL implementation position level
                errors are not computed (all 0's) for this constraint type.
  */

  ContactConstraint(
      const unsigned int bodyId,
      const Math::Vector3d &bodyPoint,
      const Math::Vector3d &groundConstraintNormalVectors,
      const char *name = NULL,
      unsigned int userDefinedId = std::numeric_limits<unsigned int>::max(),
      bool enableBaumgarteStabilization=false,
      double stabilizationParameter=0.1,      
      bool velocityLevelConstraint=true);


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
                          bool velocityLevelConstraint=true);

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
