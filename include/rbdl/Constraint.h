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
  ConstraintTypeLast,
};

struct ConstraintCache {

  ///Here N is taken to mean the number of elements in QDot.
  Math::VectorNd vecNZeros;
  Math::VectorNd vecNA, vecNB, vecNC, vecND;

  Math::Vector3d  vec3A, vec3B, vec3C, vec3D, vec3E, vec3F;
  Math::Matrix3d  mat3A, mat3B, mat3C, mat3D, mat3E, mat3F;

  Math::MatrixNd mat3NA, mat3NB, mat3NC, mat3ND;
  Math::MatrixNd mat6NA, mat6NB, mat6NC, mat6ND;
  Math::SpatialVector  svecA, svecB, svecC, svecD, svecE, svecF;
  Math::SpatialTransform stA, stB, stC, stD;

  ConstraintCache(){}
};

class RBDL_DLLAPI Constraint {
  public:

    virtual ~Constraint(){};    

    Constraint();

    Constraint(const char* name,
               const unsigned int typeOfConstraint,
               const unsigned int sizeOfConstraint,
               const unsigned int userDefinedIdNumber):
               name(name),
               typeOfConstraint(typeOfConstraint),
               sizeOfConstraint(sizeOfConstraint),
               id(userDefinedIdNumber),
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

    void addToConstraintSet(
        const unsigned int rowIndex)
    {
      rowInSystem = rowIndex;
    }


    virtual void bind(const Model &model)=0;

    virtual void calcConstraintJacobian(  Model &model,
                                          const double time,
                                          const Math::VectorNd &Q,
                                          const Math::VectorNd &QDot,
                                          Math::MatrixNd &GSysUpd,
                                          ConstraintCache &cache,
                                          bool updateKinematics=false) = 0;



    virtual void calcGamma( Model &model,
                            const double time,
                            const Math::VectorNd &Q,
                            const Math::VectorNd &QDot,
                            const Math::MatrixNd &GSys,
                            Math::VectorNd &gammaSysUpd,
                            ConstraintCache &cache,
                            bool updateKinematics=false) = 0;

    virtual void calcPositionError( Model &model,
                                    const double time,
                                    const Math::VectorNd &Q,
                                    Math::VectorNd &errSysUpd,
                                    ConstraintCache &cache,
                                    bool updateKinematics=false) = 0;


    virtual void calcVelocityError( Model &model,
                                    const double time,
                                    const Math::VectorNd &Q,
                                    const Math::VectorNd &QDot,
                                    const Math::MatrixNd &GSys,
                                    Math::VectorNd &derrSysUpd,
                                    ConstraintCache &cache,
                                    bool updateKinematics=false) = 0;

    virtual void calcConstraintForces(
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
                 bool resolveAllInRootFrame = false,
                 bool updateKinematics=false) = 0;

    unsigned int getUserDefinedId(){
      return id;
    }

    void setUserDefinedId(unsigned int userDefinedId){
      id = userDefinedId;
    }

    Math::MatrixNd getConstraintJacobian(Math::MatrixNd &GSys){
      return GSys.block(rowInSystem,0,sizeOfConstraint,GSys.cols());
    }

    Math::VectorNd getGamma(Math::VectorNd &gammaSys){
      return gammaSys.block(rowInSystem,0,sizeOfConstraint,1);
    }

    Math::VectorNd getPositionError(Math::VectorNd &errSys){
      return errSys.block(rowInSystem,0,sizeOfConstraint,1);
    }

    Math::VectorNd getVelocityError(Math::VectorNd &derrSys){
      return derrSys.block(rowInSystem,0,sizeOfConstraint,1);
    }



    Math::VectorNd getBaumgarteStabilizationForces(const Math::VectorNd &errPos,
                                                   const Math::VectorNd &errVel)
    {

        return (-2*baumgarteParameters[0]*errVel
                  -baumgarteParameters[1]*baumgarteParameters[1]*errPos);
    }

    void addInBaumgarteStabilizationForces(const Math::VectorNd &errPosSys,
                                           const Math::VectorNd &errVelSys,
                                           Math::VectorNd &gammaSysUpd)
    {

      //Here a for loop is used rather than a block operation
      //to be compatible with SimpleMath.
      for(unsigned int i=0; i<sizeOfConstraint;++i){
        gammaSysUpd[rowInSystem+i] +=
              -2.*baumgarteParameters[0]*errVelSys[rowInSystem+i]
             -(baumgarteParameters[1]*baumgarteParameters[1]
              )*errPosSys[rowInSystem+i];
      }
    }

    unsigned int getConstraintType(){
      return typeOfConstraint;
    }

    unsigned int getConstraintSize(){
      return sizeOfConstraint;
    }

    unsigned int getConstraintIndex(){
      return rowInSystem;
    }

    void setBaumgarteTimeConstant(double tStab){
      assert(tStab > 0);
      baumgarteParameters[0] = 1./tStab;
      baumgarteParameters[1] = 1./tStab;
    }

    void setEnableBaumgarteStabilization(bool flagEnableBaumgarteStabilization){
      baumgarteEnabled = flagEnableBaumgarteStabilization;
    }

    bool isBaumgarteStabilizationEnabled(){
      return baumgarteEnabled;
    }



    void enableConstraintErrorFromPositionLevel(unsigned int constraintSubIndex)
    {
      assert(constraintSubIndex < sizeOfConstraint);
      positionConstraint[constraintSubIndex] = true;
      velocityConstraint[constraintSubIndex] = true;      
    }

    void enableConstraintErrorFromVelocityLevel(unsigned int constraintSubIndex)
    {
      assert(constraintSubIndex < sizeOfConstraint);
      positionConstraint[constraintSubIndex] = false;
      velocityConstraint[constraintSubIndex] = true;      
    }

    void enableConstraintErrorFromAccelerationLevel(unsigned int constraintSubIndex)
    {
      assert(constraintSubIndex < sizeOfConstraint);
      positionConstraint[constraintSubIndex] = false;
      velocityConstraint[constraintSubIndex] = false;      
    }

    void enableConstraintErrorFromPositionLevel()
    {
        for(unsigned int i=0; i<sizeOfConstraint;++i){
          positionConstraint[i] = true;
          velocityConstraint[i] = true;                
        }
    }

    void enableConstraintErrorFromVelocityLevel()
    {
        for(unsigned int i=0; i<sizeOfConstraint;++i){
          positionConstraint[i] = false;
          velocityConstraint[i] = true;                
        }
    }

    void enableConstraintErrorFromAccelerationLevel()
    {
        for(unsigned int i=0; i<sizeOfConstraint;++i){
          positionConstraint[i] = false;
          velocityConstraint[i] = false;                
        }
    }



    bool getPositionLevelError(unsigned int constraintSubIndex){
      assert(constraintSubIndex < sizeOfConstraint);
      return positionConstraint[constraintSubIndex];      
    }

    bool getVelocityLevelError(unsigned int constraintSubIndex){
      assert(constraintSubIndex < sizeOfConstraint);
      return velocityConstraint[constraintSubIndex];      
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
    ///A user defined name which is unique to this constraint set
    const char* name;

    ///A user defined id which is unique to this constraint set
    unsigned int id;
    ///The type of this constraint
    const unsigned int typeOfConstraint;

    ///The number of rows that this constraint adds to G.
    unsigned int sizeOfConstraint;

    ///The first row in G that corresponds to this constraint.
    unsigned int rowInSystem;

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
