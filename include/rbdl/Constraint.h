/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_CONSTRAINT_H
#define RBDL_CONSTRAINT_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
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
                            const Math::VectorNd &QDDot0,
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
        return (-2*baumgarteParameters[0]*errPos
                  -baumgarteParameters[1]*baumgarteParameters[1]*errVel);
    }

    void addInBaumgarteStabilizationForces(const Math::VectorNd &errSys,
                                           const Math::VectorNd &derrSys,
                                           Math::VectorNd &gammaSysUpd)
    {

      gammaSysUpd.block(indexOfConstraintInG,0,sizeOfConstraint,1) +=  
            -2*baumgarteParameters[0]*errSys.block(
                indexOfConstraintInG,0,sizeOfConstraint,1),
           -(baumgarteParameters[1]*baumgarteParameters[1])*derrSys.block(
                indexOfConstraintInG,0,sizeOfConstraint,1);
    }

    unsigned int getConstraintType(){
      return typeOfConstraint;
    }

    unsigned int getConstraintSize(){
      return sizeOfConstraint;
    }

    unsigned int getConstraintIndex(){
      return indexOfConstraintInG;
    }

    void setBaumgarteTimeConstant(double tStab){
      assert(tStab > 0);
      baumgarteParameters[0] = 1./tStab;
      baumgarteParameters[1] = 1./tStab;
    }

    void enableBaumgarteStabilization(){
      baumgarteEnabled = true;
    }
    void disableBaumgarteStabilization(){
      baumgarteEnabled = false;
    }

    bool isBaumgarteStabilizationEnabled(){
      return baumgarteEnabled;
    }


    const char* getName(){
      return name;
    }

    const std::vector< unsigned int >& getBodyIds(){
      return bodyIds;
    }

    const std::vector< Math::SpatialTransform >& getBodyFrames(){
      return bodyFrames;
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




/** @} */

} 

/* namespace RigidBodyDynamics */

/* RBDL_CONSTRAINT_H */
#endif
