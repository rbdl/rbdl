/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_CONSTRAINTS_LIBRARY_H
#define RBDL_CONSTRAINTS_LIBRARY_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Kinematics.h>
#include <assert.h>

namespace RigidBodyDynamics {

// Enum to describe the type of a constraint.
enum ConstraintType {
  ConstraintTypeContact=0,
  ConstraintTypeLoop,
  ConstraintTypeCustom,
  ConstraintTypeBodyToGroundPosition,
  ConstraintTypeLast,
};


class RBDL_DLLAPI Constraint {
  public:

    virtual ~Constraint(){};    

    Constraint(const char* name,
               const unsigned int typeOfConstraint,
               const unsigned int indexOfConstraintInG,
               const unsigned int sizeOfConstraint):
                name(name),
                typeOfConstraint(typeOfConstraint),
                indexOfConstraintInG(indexOfConstraintInG),
                sizeOfConstraint(sizeOfConstraint),
                baumgarteParameters(1./0.1,1./0.1),
                baumgarteEnabled(false)
    {
      positionConstraint.resize(sizeOfConstraint);
      velocityConstraint.resize(sizeOfConstraint);
      for(unsigned int i=0; i<sizeOfConstraint;++i){
        positionConstraint[i]=false;
        velocityConstraint[i]=false;
      }
    }

    virtual void bind(const Model &model)=0;

    virtual void calcConstraintJacobian(  Model &model,
                                          const Math::VectorNd &Q,
                                          Math::MatrixNd &GSysUpd) = 0;    

    Math::MatrixNd getConstraintJacobian(Math::MatrixNd &GSys){
      return GSys.block(indexOfConstraintInG,0,sizeOfConstraint,GSys.cols());
    }

    virtual void calcGamma( Model &model,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::MatrixNd &GSys,
                            Math::VectorNd &gammaSysUpd) = 0;   

    Math::VectorNd getGamma(Math::VectorNd &gammaSys){
      return gammaSys.block(indexOfConstraintInG,0,sizeOfConstraint,1);
    }

    virtual void calcConstraintForces(
                 Model &model,
                 const Math::VectorNd &Q,
                 const Math::VectorNd &QDot,
                 const Math::MatrixNd &GSys,
                 const Math::VectorNd &LagrangeMultipliersSys,
                 std::vector< unsigned int > &constraintBodiesUpd,
                 std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
                 std::vector< Math::SpatialVector > &constraintForcesUpd,
                 bool resolveAllInRootFrame = false) = 0;

    virtual void calcPositionError( Model &model,
                                    const Math::VectorNd &Q,
                                    Math::VectorNd &errSysUpd) = 0;

    Math::VectorNd getPositionError(Math::VectorNd &errSys){
      return errSys.block(indexOfConstraintInG,0,sizeOfConstraint,1);
    }

    virtual void calcVelocityError( Model &model,
                                    const Math::VectorNd &Q,
                                    const Math::VectorNd &QDot,
                                    const Math::MatrixNd &GSys,
                                    Math::VectorNd &derrSysUpd) = 0;

    Math::VectorNd getVelocityError(Math::VectorNd &derrSys){
      return derrSys.block(indexOfConstraintInG,0,sizeOfConstraint,1);
    }


    Math::VectorNd getBaumgarteStabilizationForces(const Math::VectorNd &errPos,
                                                    const Math::VectorNd &errVel)
    {
        return Math::VectorNd(baumgarteParameters[0]*errPos 
                            + baumgarteParameters[1]*errVel);
    }

    void addInBaumgarteStabilizationForces(const Math::VectorNd &errSys,
                                           const Math::VectorNd &derrSys,
                                           Math::VectorNd &gammaSysUpd)
    {
      gammaSysUpd.block(indexOfConstraintInG,0,sizeOfConstraint,1) +=  
          getBaumgarteStabilizationForces(
            errSys.block(indexOfConstraintInG,0,sizeOfConstraint,1),
           derrSys.block(indexOfConstraintInG,0,sizeOfConstraint,1));
    }

    unsigned int getConstraintSize(){
      return sizeOfConstraint;
    }

    void setBaumgarteTimeConstant(double tStab){
      assert(tStab > 0);
      baumgarteParameters[0] = 1./tStab;
      baumgarteParameters[1] = 1./tStab;
    }

    void setBaumgarteStabilization(bool enableBaumgarteStabilization){
      baumgarteEnabled = enableBaumgarteStabilization;
    }

    bool getBaumgarteStabilization(){
      return baumgarteEnabled;
    }

    const char* getName(){
      return name;
    }



  protected:
    ///A unique name
    const char* name;

    ///The type of this constraint
    const unsigned int typeOfConstraint;

    ///The number of rows that this constraint adds to G.
    const unsigned int sizeOfConstraint;

    ///The first row in G that corresponds to this constraint.
    const unsigned int indexOfConstraintInG;

    ///The index of the predecessor body in the vector of bodies in Model
    std::vector< unsigned int > bodyIds;

    ///Transform from the frame of the predecessor body to the constraint frame
    std::vector< Math::SpatialTransform > bodyFrames;

    ///The Baumgarte stabilization coefficients at the position and velocity
    /// level
    Math::Vector2d baumgarteParameters;

    ///A flag which enables or disables Baumgarte stabilization
    bool baumgarteEnabled;

    ///A mask that is used to selectively enable/disable the calculation of
    /// position-level and velocity-level errors. These errors are used to
    /// functions that assemble the system at the position and velocity levels,
    /// and also by functions that stablize the constraint error at the
    /// position and velocity level.
    std::vector< bool > positionConstraint;
    std::vector< bool > velocityConstraint;
};


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

/* RBDL_CONSTRAINTS_LIBRARY_H */
#endif
