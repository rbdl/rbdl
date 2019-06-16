/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *               2019 Matthew Millard <mjhmilla@protonmail.com>
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_CONSTRAINTSETS_H
#define RBDL_CONSTRAINTSETS_H

#include <memory>

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Constraint.h>
#include <rbdl/Constraint_Contact.h>
#include <rbdl/Constraint_Loop.h>
#include <string.h>
#include <assert.h>
#include <map>

namespace RigidBodyDynamics {




/** \page constraints_page Constraints
 *
 * All functions related to constraints are specified in the \ref
 * constraints_group "Constraints Module".

 * \defgroup constraints_group Constraints
 *
 * Constraints are handled by specification of a \link
 * RigidBodyDynamics::ConstraintSet
 * ConstraintSet \endlink which contains all information about the
 * current constraints and workspace memory.
 *
 * Separate constraints can be specified by calling
 * ConstraintSet::AddContactConstraint(), ConstraintSet::AddLoopConstraint(),
 * and ConstraintSet::AddCustomConstraint().
 * After all constraints have been specified, this \link
 * RigidBodyDynamics::ConstraintSet
 * ConstraintSet \endlink has to be bound to the model via
 * ConstraintSet::Bind(). This initializes workspace memory that is
 * later used when calling one of the contact functions, such as
 * ForwardDynamicsContactsDirects().
 *
 * The values in the vectors ConstraintSet::force and
 * ConstraintSet::impulse contain the Lagrange multipliers or
 * change in Lagrange multipliers for each constraint when returning from one 
 * of the contact functions.
 *
 * \section solution_constraint_system Solution of the Constraint System
 *
 * \subsection constraint_system Linear System of the Constrained Dynamics
 *
 * In the presence of constraints, to compute the
 * acceleration one has to solve a linear system of the form: \f[
 \left(
   \begin{array}{cc}
     H & G^T \\
     G & 0
   \end{array}
 \right)
 \left(
   \begin{array}{c}
     \ddot{q} \\
     - \lambda
   \end{array}
 \right)
 =
 \left(
   \begin{array}{c}
     -C + \tau \\
     \gamma
   \end{array}
 \right)
 * \f] where \f$H\f$ is the joint space inertia matrix computed with the
 * CompositeRigidBodyAlgorithm(), \f$G\f$ is the constraint Jacobian,
 * \f$C\f$ the bias force (sometimes called "non-linear
 * effects"), and \f$\gamma\f$ the generalized acceleration independent
 * part of the constraints.
 *
 * \subsection collision_system Linear System of the Constraint Impacts
 *
 * Similarly to compute the response of the model to a constraint gain one has
 * to solve a system of the following form: \f[
 \left(
   \begin{array}{cc}
     H & G^T \\
     G & 0
   \end{array}
 \right)
 \left(
   \begin{array}{c}
     \dot{q}^{+} \\
     \Lambda
   \end{array}
 \right)
 =
 \left(
   \begin{array}{c}
     H \dot{q}^{-} \\
    v^{+} 
   \end{array}
 \right)
 * \f] where \f$H\f$ is the joint space inertia matrix computed with the
 * CompositeRigidBodyAlgorithm(), \f$G\f$ are the Jacobians of the
 * constraints, \f$\dot{q}^{+}\f$ the generalized velocity after the
 * impact, \f$\Lambda\f$ the impulses at each constraint, \f$\dot{q}^{-}\f$
 * the generalized velocity before the impact, and \f$v^{+}\f$ the desired
 * velocity of each constraint after the impact (known beforehand, usually
 * 0). The value of \f$v^{+}\f$ can is specified via the variable
 * ConstraintSet::v_plus and defaults to 0.
 *
 * \subsection solution_methods Solution Methods
 *
 * There are essentially three different approaches to solve these systems:
 * -# \b Direct: solve the full system to simultaneously compute
 *  \f$\ddot{q}\f$ and \f$\lambda\f$. This may be slow for large systems
 *  and many constraints.
 * -# \b Range-Space: solve first for \f$\lambda\f$ and then for
 *  \f$\ddot{q}\f$.
 * -# \b Null-Space: solve furst for \f$\ddot{q}\f$ and then for
 *  \f$\lambda\f$
 * The methods are the same for the contact gains just with different
 * variables on the right-hand-side.
 *
 * RBDL provides methods for all approaches. The implementation for the
 * range-space method also exploits sparsities in the joint space inertia
 * matrix using a sparse structure preserving \f$L^TL\f$ decomposition as
 * described in Chapter 8.5 of "Rigid Body Dynamics Algorithms".
 *
 * None of the methods is generally superior to the others and each has
 * different trade-offs as factors such as model topology, number of
 * constraints, constrained bodies, numerical stability, and performance
 * vary and evaluation has to be made on a case-by-case basis.
 * 
 * \subsection solving_constraints_dynamics Methods for Solving Constrained 
 * Dynamics
 * 
 * RBDL provides the following methods to compute the acceleration of a
 * constrained system:
 *
 * - ForwardDynamicsConstraintsDirect()
 * - ForwardDynamicsConstraintsRangeSpaceSparse()
 * - ForwardDynamicsConstraintsNullSpace()
 *
 * \subsection solving_constraints_collisions Methods for Computing Collisions
 *
 * RBDL provides the following methods to compute the collision response on
 * new contact gains:
 *
 * - ComputeConstraintImpulsesDirect()
 * - ComputeConstraintImpulsesRangeSpaceSparse()
 * - ComputeConstraintImpulsesNullSpace()
 *
 * \subsection assembly_q_qdot Computing generalized joint positions and velocities
 * satisfying the constraint equations.
 *
 * When considering a model subject position level constraints expressed by the
 * equation \f$\phi (q) = 0\f$, it is often necessary to compute generalized joint
 * position and velocities which satisfy the constraints. Even velocity-level
 * constraints may have position-level assembly constraints:  a rolling-without-slipping
 * constraint is a velocity-level constraint, but during assembly it might be desireable
 * to put the rolling surfaces in contact with eachother.
 *
 * In order to compute a vector of generalized joint positions that satisfy
 * the constraints it is necessary to solve the following optimization problem:
 * \f{eqnarray*}{
 * \text{minimize} && \sum_{i = 0}^{n} (q - q_{0})^T W (q - q_{0}) \\
 * \text{over} && q \\
 * \text{subject to} && \phi (q) = 0
 * \f}
 * 
 * In order to compute a vector of generalized joint velocities that satisfy
 * the constraints it is necessary to solve the following optimization problem:
 * \f{eqnarray*}{
 * \text{minimize} && \sum_{i = 0}^{n} (\dot{q} - \dot{q}_{0})^T W (\dot{q} - \dot{q}_{0}) \\
 * \text{over} && \dot{q} \\
 * \text{subject to} && \dot{\phi} (q) = \phi _{q}(q) \dot{q} + \phi _{t}(q) = 0
 * \f}
 *
 * \f$q_{0}\f$ and \f$\dot{q}_{0}\f$ are initial guesses, \f$\phi _{q}\f$ is the
 * constraints Jacobian (partial derivative of \f$\phi\f$ with respect to \f$q\f$),
 * and \f$\phi _{t}(q)\f$ is the partial derivative of \f$\phi\f$ with respect
 * to time.  \f$W\f$ is a diagonal weighting matrix, which can be exploited
 * to prioritize changes in the position/ velocity of specific joints.
 * With a solver capable of handling singular matrices, it is possible to set to
 * 1 the weight of the joint positions/ velocities that should not be changed
 * from the initial guesses, and to 0 those corresponding to the values that
 * can be changed.
 *
 * These problems are solved using the Lagrange multipliers method. For the
 * velocity problem the solution is exact. For the position problem the
 * constraints are linearized in the form 
 * \f$ \phi (q_{0}) + \phi _{t}(q0) + \phi _{q_0}(q) (q - q_{0}) \f$
 * and the linearized problem is solved iteratively until the constraint position
 * errors are smaller than a given threshold.
 *
 * RBDL provides two functions to compute feasible joint position and velocities:
 * - CalcAssemblyQ()
 * - CalcAssemblyQDot()
 *
 * \subsection baumgarte_stabilization Baumgarte Stabilization
 *
 * The constrained dynamic equations are correct at the acceleration level
 * but will drift at the velocity and position level during numerical 
 * integration. RBDL implements Baumgarte stabilization to avoid
 * the accumulation of position and velocity errors for loop constraints and
 * custom constraints. Contact constraints do not have Baumgarte stabilization
 * because they are a special case which does not typically suffer from drift. 
 * The stabilization term can be enabled/disabled using the appropriate 
 * ConstraintSet::AddLoopConstraint and ConstraintSet::AddCustomConstraint 
 * functions. 
 *
 * The dynamic equations are changed to the following form: \f[
 \left(
   \begin{array}{cc}
     H & G^T \\
     G & 0
   \end{array}
 \right)
 \left(
   \begin{array}{c}
     \ddot{q} \\
     - \lambda
   \end{array}
 \right)
 =
 \left(
   \begin{array}{c}
     -C + \tau \\
     \gamma + \gamma _{stab}
   \end{array}
 \right)
 * \f] A term \f$\gamma _{stab}\f$ is added to the right hand side of the
 * equation, defined in the following way: \f[
   \gamma _{stab} = - 2 \alpha \dot{\phi} (q) - \beta^2 \phi (q)
 * \f] where \f$\phi (q)\f$ are the position level constraint errors and 
 * \f$\alpha\f$ and \f$\beta\f$ are tuning coefficients. There is
 * no clear and simple rule on how to choose them as good values also
 * depend on the used integration method and time step. If the values are
 * too small the constrained dynamics equation becomes stiff, too big
 * values result in errors not being reduced.
 *
 * A good starting point is to parameterize the coefficients as:
 * \f[
 * \alpha = \beta = 1 / T_\textit{stab}
 * \f]
 * with \f$T_\textit{stab}\f$ specifies a time constant for errors in position
 * and velocity errors to reduce. Featherstone suggests in his book "Rigid
 * Body Dynamics Algorithms" that for a big industrial robot a value of 0.1
 * is reasonable. When testing different values best is to try different
 * orders of magnitude as e.g. doubling a value only has little effect.
 *
 * FBaumgarte stabilization is disabled by default, and uses the default
 * the stabilization parameter \f$T_\textit{stab} = 0.1\f$.
 *
 * @{
 */

struct Model;



//class RBDL_DLLAPI Constraint;


/** \brief Structure that contains both constraint information and workspace memory.
 *
 * This structure is used to reduce the amount of memory allocations that
 * are needed when computing constraint forces.
 *
 * The ConstraintSet has to be bound to a model using ConstraintSet::Bind()
 * before it can be used in \link RigidBodyDynamics::ForwardDynamicsContacts
 * ForwardDynamicsContacts \endlink.
 */
struct RBDL_DLLAPI ConstraintSet {
  ConstraintSet() :
    linear_solver (Math::LinearSolverColPivHouseholderQR),
    bound (false) {}



  /**
      @brief getGroupIndex returns the index to a constraints that have been
             grouped because they are of the same type, apply to the same
             bodies, and apply to the same local frames on each body.

      @note Internally this function makes a temporary string. If speed is 
            a concern then do not call this function in a loop: either save
            the index locally, use the userDefinedId (which is an integer),
            or use the assigned id (also an integer).

      @param userDefinedName : the optional name that the constraint was
              assigned when it was added to the constraint set.

      @return: the group index of the constraint
  */

  unsigned int getGroupIndexByName(const char* userDefinedName){
    std::string conName(userDefinedName);
    return nameGroupMap.at(conName);
  }

  /**
      @brief getGroupIndex returns the index to a constraints that have been
             grouped because they are of the same type, apply to the same
             bodies, and apply to the same local frames on each body.

      @param userDefinedId : the optional integer id that was assigned to this
                constraint when it was created.

      @return: the group index of the constraint
  */

  unsigned int getGroupIndexById(unsigned int userDefinedId){
    return userDefinedIdGroupMap.at(userDefinedId);
  }

  /**
      @brief getGroupIndex returns the index to a constraints that have been
         grouped because they are of the same type, apply to the same
         bodies, and apply to the same local frames on each body.

      @param assignedId :
            the integer id that was returned when the constraint was added
            to the constraint set by the functions: AddContactConstraint,
            AddLoopConstraint, AddCustomConstraint, etc.

      @return: the group index of the constraint
  */

  unsigned int getGroupIndexByAssignedId(unsigned int assignedId){
    return idGroupMap.at(assignedId);
  }


  /**
      @brief getGroupIndexMax returns the maximum valid constraint
      group index (the min. is zero) so that constraint groups can
      be iterated over if desired.
      @return the largest group index
   */
  unsigned int getGroupIndexMax(){
    return unsigned(constraints.size()-1);
  }

  /**
      @brief Returns the name of the constraint group, which may differ from
             the names entered by the user if the constraint is one in which
             grouping is done automatically (e.g. contact and loop constraints)

      @param groupIndex the index number of a constraint group.

      @return the name of the constraint group. For constraints that are grouped
             automatically (e.g. contact and loop constraints) the group name is
             the name of the first constraint added to the group. For
             constraints which are not automatically grouped (e.g.
             CustomConstraints) the group name is identical to the optional
             name used when the constraint was added to the constraint
             set (e.g. when AddCustomConstraint was called).

   */
  const char* getGroupName(unsigned int groupIndex){
    return constraints[groupIndex]->getName();
  }

  /**
      @brief Returns the user-defined-id of the constraint group, which may
             differ from the names entered by the user if the constraint is one
             in which grouping is done automatically (e.g. contact and loop
             constraints).

      @param groupIndex the index number of a constraint group.

      @return the user-defined-id of the constraint group. For constraints that
              are grouped automatically (e.g. contact and loop constraints) the
              user-defined-id will be the user-defined-id fo the of the first
              constraint added to the group. For constraints which are not
              automatically grouped (e.g. CustomConstraints) the user-defined-id
              is identical to the one the user assigned (optionally) when the
              constraint was added to the constraint set (e.g. when
              AddCustomConstraint was called).
   */
  unsigned int getGroupId(unsigned int groupIndex){  
    return constraints[groupIndex]->getUserDefinedId();
  }

  /**
      @brief Returns assigned id of the constraint group which will be the first
             id assigned to an entry in a group (if the grouping was done
             automatically - as is done for contact and loop constraints).

      @param groupIndex the index number of a constraint group.

      @return the assigned of the constraint group. For constraints that
              are grouped automatically (e.g. contact and loop constraints) the
              assigned id will be the assigned id fo the of the first
              constraint added to the group. For constraints which are not
              automatically grouped (e.g. CustomConstraints) the assigned id
              is identical to the one returned when the constraint was added
              to the constraint set (e.g. when AddCustomConstraint was called).
   */

  unsigned int getGroupAssignedId(unsigned int groupIndex){
    unsigned int assignedId=std::numeric_limits<unsigned int>::max();
    auto it = idGroupMap.begin();
    bool found = false;
    while(it != idGroupMap.end() && found == false){
      if(it->second == groupIndex){
        assignedId = it->first;
        found = true;
      }
      it++;
    }
    if(found==false){
      std::cerr << "Error: a groupIndex of " << groupIndex
                << " could not be found. Valid groupIndex range between 0 and "
                << unsigned(constraints.size()-1);
      assert(0);
      abort();
    }
    return assignedId;
  }

  /**
      @brief  calcForces resolves the generalized forces generated by
            this constraint into equivalent spatial forces (resolved in the
            local or the base frame) that are applied between the bodies and
            frames that this constraint applies to.

      @note <b>Important:</b> The values returned by this function are only
            valid <b>after</b> one of the dynamics methods
            (ForwardDynamicsConstraintsDirect,
            ForwardDynamicsConstraintsNullSpace, InverseDynamicsWithConstraints,
            etc) have been evaluated. Why? This function makes use of the
            Lagrange multipliers which are only evaluated when these
            dynamics-level functions are called.

      @param groupIndex: the index number of this constraint (see getGroupIndex
            index functions)

      @param model: the multibody model

      @param Q: the generalized positions of the model

      @param QDot: the generalized velocities of the model

      @param constraintBodyIdsUpd: a vector of body ids in which the spatial
              forces are resolved into. The ordering for the standard constraints
              are as follows :
              - ContactConstraint
                - [0] : body 
                - [1] : ground
              - Loop Constraint
                - [0] : predecessor body
                - [1] : successor

      @param constraintBodyFramesUpd: a vector of frames in which
              the spatial forces are resolved. The \f$i^{th}\f$ frame is
              located in the \f$i^{th}\f$ body id listed in 
              constraintBodyIdsUpd.
 
      @param constraintForcesUpd: a vector of spatial forces generated by this
              constraint. The \f$i^{th}\f$ spatial force is resolved into the
              \f$i^{th}\f$ frame listed in constraintBodyFramesUpd.

      @param resolveAllInRootFrame: 
              - false: spatial forces are resolved into the local frames attached to the bodies involved in this constraint
              - true: spatial forces are resolved into the base frame. Note that this means that all entires in constraintBodyIdsUpd will be 0, the frames in constraintFramesUpd will have their origins at the contact frames but with directions that match the base frame.

      @param updateKinematics: setting this flag to true will trigger the
            kinematic transforms of the model to be updated.
   */
  void calcForces(
      unsigned int groupIndex,
      Model& model,
      const Math::VectorNd &Q,
      const Math::VectorNd &QDot,
      std::vector< unsigned int > &constraintBodyIdsUpd,
      std::vector< Math::SpatialTransform > &constraintBodyFramesUpd,
      std::vector< Math::SpatialVector > &constraintForcesUpd,
      bool resolveAllInRootFrame = false,
      bool updateKinematics = false);

  /**
      @brief calcPositionError calculates the vector of position
              errors associated with this constraint. Note that if the
              constraint group, or parts of it, are not defined at the position
              level then 0's will be returned.

      @param groupIndex: the index number of this constraint (see getGroupIndex
            index functions)

      @param model: the multibody model

      @param Q: the generalized positions of the model

      @param positionErrorUpd: the vector of constraint errors

      @param updateKinematics: setting this flag to true will trigger the
            kinematic transforms of the model to be updated.
   */
  void calcPositionError(
      unsigned int groupIndex,
      Model& model,
      const Math::VectorNd &Q,
      Math::VectorNd &positionErrorUpd,
      bool updateKinematics = false);


  /**
      @brief calcVelocityError calculates the vector of position
              errors associated with this constraint. Note that if the
              constraint group, or parts of it, are not defined at the position
              level then 0's will be returned.

      @param groupIndex: the index number of this constraint (see getGroupIndex
            index functions)

      @param model: the multibody model

      @param Q: the generalized positions of the model

      @param QDot: the generalized velocities of the model

      @param velocityErrorUpd: the vector of constraint errors at the velocity
              level

      @param updateKinematics: setting this flag to true will trigger the
            kinematic transforms of the model to be updated.
   */
  void calcVelocityError(
      unsigned int groupIndex,
      Model& model,
      const Math::VectorNd &Q,
      const Math::VectorNd &QDot,
      Math::VectorNd &velocityErrorUpd,
      bool updateKinematics = false);


  /**

      @param groupIndex: the index number of this constraint (see getGroupIndex
            index functions)

      @param model: the multibody model

      @param positionError: the position errors associated with this constraint
                            computed using the calcConstraintPositionError
                            function

      @param velocityError: the velocity errors associated with this constraint
                            computed using the calcConstraintVelocityError
                            function

      @param baumgarteForcesUpd: the baumgarte stabilization forces applied to this
                              this constraint.
   */
  void calcBaumgarteStabilizationForces(
      unsigned int groupIndex,
      Model& model,
      const Math::VectorNd &positionError,
      const Math::VectorNd &velocityError,
      Math::VectorNd &baumgarteForcesUpd);

  /**
     @param groupIndex: the index number of this constraint (see getGroupIndex
            index functions)
     @return true if Baumgarte stabilization is enabled
  */
  bool isBaumgarteStabilizationEnabled(
      unsigned int groupIndex){
    return constraints[groupIndex]->isBaumgarteStabilizationEnabled();
  }

  void getBaumgarteStabilizationCoefficients(
      unsigned int groupIndex,
      Math::Vector2d& baumgartePositionVelocityCoefficientsUpd){
      constraints[groupIndex]->getBaumgarteStabilizationParameters(
                                baumgartePositionVelocityCoefficientsUpd);
  }

  /** @brief Adds a single contact constraint (point-ground) to the
      constraint set.

      This type of constraints ensures that the velocity and acceleration of a
      specified body point along a specified axis are null.

     @param bodyId the body which is affected directly by the constraint

     @param bodyPoint the point that is constrained relative to the
            contact body

     @param worldNormal the normal direction in which to apply the constraint

     @param constraintName a human readable name (optional, default: NULL).
            Set this field to a unique name (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.

     @param userDefinedId a user defined id (optional, defaults to max()).
            Set this field to a unique number (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.
   */
  unsigned int AddContactConstraint (
    unsigned int bodyId,
    const Math::Vector3d &bodyPoint,
    const Math::Vector3d &worldNormal,
    const char *constraintName = NULL,
    unsigned int userDefinedId = std::numeric_limits<unsigned int>::max());

  /** \brief Adds a loop constraint to the constraint set.
   
    This type of constraints ensures that the relative orientation and position,
    spatial velocity, and spatial acceleration between two frames in two bodies
    are null along a specified spatial constraint axis.
   
    @param bodyIdPredecessor the identifier of the predecessor body

    @param bodyIdSuccessor the identifier of the successor body

    @param XPredecessor a spatial transform localizing the constrained
            frames on the predecessor body, expressed with respect to the 
            predecessor body frame. Note the position vector should be
            the r_BPB (from the body's origin, to the predessor frame, in the
            coordinates of the body's frame) and E_BP (the rotation matrix that
            will rotate vectors from the coordinates of the P frame to the 
            coordinates of the body's frame).
    
    @param XSuccessor a spatial transform localizing the constrained
          frames on the successor body, expressed with respect to the successor
          body frame. Note the position vector should be
            the r_BSB (from the body's origin, to the successor frame, in the
            coordinates of the body's frame) and E_BS (the rotation matrix that
            will rotate vectors from the coordinates of the S frame to the 
            coordinates of the body's frame).
    
    @param constraintAxisInPredecessor a spatial vector, resolved in the frame of
            the predecessor frame, indicating the axis along which the 
            constraint acts
    
     @param enableBaumgarteStabilization (optional, default false) setting this
            flag to true will modify the right hand side of the acceleration
            equation with a penaltiy term that is proportional to the constraint
            error scaled by a constant.

     @param stabilizationTimeConstant (optional, defaults to 0.1 sec) this
            value scales the strength of Baumgarte stabilization so that the
            settling time of the error is proportional the value given here.

     @param constraintName a human readable name (optional, default: NULL).
            Set this field to a unique name (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.

     @param userDefinedId a user defined id (optional, defaults to max()).
            Set this field to a unique number (within this ConstraintSet) so that
            the function GetConstraintIndex can find it.


   */
    unsigned int AddLoopConstraint(
      unsigned int bodyIdPredecessor,
      unsigned int bodyIdSuccessor,
      const Math::SpatialTransform &XPredecessor,
      const Math::SpatialTransform &XSuccessor,
      const Math::SpatialVector &constraintAxisInPredecessor,
      bool enableBaumgarteStabilization = false,
      double stabilizationTimeConstant = 0.1,
      const char *constraintName = NULL,
      unsigned int userDefinedId = std::numeric_limits<unsigned int>::max());




  /** \brief Adds a custom constraint to the constraint set.
   *
   * \param customConstraint The CustomConstraint to be added to the
   * ConstraintSet
   *
   */
  unsigned int AddCustomConstraint(
      std::shared_ptr<Constraint> customConstraint);

  /** \brief Copies the constraints and resets its ConstraintSet::bound
   * flag.
   */
  ConstraintSet Copy() {
    ConstraintSet result (*this);
    result.bound = false;

    return result;
  }

  /** \brief Specifies which method should be used for solving undelying linear systems.
   */
  void SetSolver (Math::LinearSolver solver) {
    linear_solver = solver;
  }

  /** \brief Initializes and allocates memory for the constraint set.
   *
   * This function allocates memory for temporary values and matrices that
   * are required for contact force computation. Both model and constraint
   * set must not be changed after a binding as the required memory is
   * dependent on the model size (i.e. the number of bodies and degrees of
   * freedom) and the number of constraints in the Constraint set.
   *
   * The values of ConstraintSet::acceleration may still be
   * modified after the set is bound to the model.
   *
   */
  bool Bind (const Model &model);

  /** \brief Returns the number of constraints. */
  size_t size() const {
    return constraintType.size();
  }

  /** \brief Clears all variables in the constraint set. */
  void clear ();

  /// Method that should be used to solve internal linear systems.
  Math::LinearSolver linear_solver;
  /// Whether the constraint set was bound to a model (mandatory!).
  bool bound;

  // Common constraints variables.
  std::vector<ConstraintType> constraintType;

  std::vector<std::string> name;

  std::map < std::string, unsigned int > nameGroupMap;
  std::map < unsigned int, unsigned int> userDefinedIdGroupMap;
  std::map < unsigned int, unsigned int> idGroupMap;

  //A shared_ptr (MM 28 May 2019)
  //  :is 2x as expensive (with the optimize flags turned on) as a regular
  //   pointer (https://www.modernescpp.com/index.php/memory-and-performance-overhead-of-smart-pointer)
  //  :But
  //    -The memory is deleted automatically when all references are deleted.
  //    -A shared_ptr works with the existing Copy function
  //    -... although the Copy is a shallow copy. A deep copy would be better
  //
  //  :A unique_ptr would be faster, but will require copy constructors to
  //   be defined for all constraint objects. In addition unique_ptr is only
  //   included in C++14, while shared_ptr is available in C++11. While this
  //   is clearly the better option, this will require me to make more
  //   edits to ConstraintSet: I don't have the time for this at the moment
  std::vector< std::shared_ptr<Constraint> > constraints;

  std::vector< std::shared_ptr<ContactConstraint> > contactConstraints;

  std::vector< std::shared_ptr<LoopConstraint> > loopConstraints;

  
  /** Position error for the Baumgarte stabilization */
  Math::VectorNd err;
  /** Velocity error for the Baumgarte stabilization */
  Math::VectorNd errd;

  /// The Lagrange multipliers, which due to some careful normalization
  /// in the formulation of constraints are the constraint forces.
  Math::VectorNd force;
  /// Constraint impulses along the constraint directions. 
  Math::VectorNd impulse;
  /// The velocities we want to have along the constraint directions 
  Math::VectorNd v_plus;

  // Variables used by the Lagrangian methods

  /// Workspace for the joint space inertia matrix.
  Math::MatrixNd H;
  /// Workspace for the coriolis forces.
  Math::VectorNd C;
  /// Workspace of the right hand side of the acceleration equation.
  Math::VectorNd gamma;
  /// Workspace for the constraint Jacobian
  Math::MatrixNd G;

  /// Workspace for the Lagrangian left-hand-side matrix.
  Math::MatrixNd A;
  /// Workspace for the Lagrangian right-hand-side.
  Math::VectorNd b;
  /// Workspace for the Lagrangian solution.
  Math::VectorNd x;

  /// Workspace for the QR decomposition of the null-space method
#ifdef RBDL_USE_SIMPLE_MATH
  SimpleMath::HouseholderQR<Math::MatrixNd> GT_qr;
#else
  Eigen::HouseholderQR<Math::MatrixNd> GT_qr;
#endif

  Math::MatrixNd GT_qr_Q;
  Math::MatrixNd Y;
  Math::MatrixNd Z;
  Math::VectorNd qddot_y;
  Math::VectorNd qddot_z;

  // Variables used by the IABI methods

  /// Workspace for the Inverse Articulated-Body Inertia.
  Math::MatrixNd K;
  /// Workspace for the accelerations of due to the test forces
  Math::VectorNd a;
  /// Workspace for the test accelerations.
  Math::VectorNd QDDot_t;
  /// Workspace for the default accelerations.
  Math::VectorNd QDDot_0;
  /// Workspace for the test forces.
  std::vector<Math::SpatialVector> f_t;
  /// Workspace for the actual spatial forces.
  std::vector<Math::SpatialVector> f_ext_constraints;
  /// Workspace for the default point accelerations.
  std::vector<Math::Vector3d> point_accel_0;

  /// Workspace for the bias force due to the test force
  std::vector<Math::SpatialVector> d_pA;
  /// Workspace for the acceleration due to the test force
  std::vector<Math::SpatialVector> d_a;
  Math::VectorNd d_u;

  /// Workspace for the inertia when applying constraint forces
  std::vector<Math::SpatialMatrix> d_IA;
  /// Workspace when applying constraint forces
  std::vector<Math::SpatialVector> d_U;
  /// Workspace when applying constraint forces
  Math::VectorNd d_d;

  std::vector<Math::Vector3d> d_multdof3_u;

  ConstraintCache cache;



  
};

/** \brief Computes the position errors for the given ConstraintSet.
  *
  * \param model the model
  * \param Q the generalized positions of the joints
  * \param CS the constraint set for which the error should be computed
  * \param err (output) vector where the error will be stored in (should have
  * the size of CS).
  * \param update_kinematics whether the kinematics of the model should be
  * updated from Q.
  *
  * \note the position error is always 0 for contact constraints.
  *
  */
RBDL_DLLAPI
void CalcConstraintsPositionError(
  Model& model,
  const Math::VectorNd &Q,
  ConstraintSet &CS,
  Math::VectorNd& err,
  bool update_kinematics = true
);

/** \brief Computes the Jacobian for the given ConstraintSet
 *
 * \param model the model
 * \param Q     the generalized positions of the joints
 * \param CS    the constraint set for which the Jacobian should be computed
 * \param G     (output) matrix where the output will be stored in
 * \param update_kinematics whether the kinematics of the model should be 
 * updated from Q
 *
 */
RBDL_DLLAPI
void CalcConstraintsJacobian(
  Model &model,
  const Math::VectorNd &Q,
  ConstraintSet &CS,
  Math::MatrixNd &G,
  bool update_kinematics = true
);

/** \brief Computes the velocity errors for the given ConstraintSet.
  *
  *
  * \param model the model
  * \param Q the generalized positions of the joints
  * \param QDot the generalized velocities of the joints
  * \param CS the constraint set for which the error should be computed
  * \param err (output) vector where the error will be stored in (should have
  * the size of CS).
  * \param update_kinematics whether the kinematics of the model should be
  * updated from Q.
  *
  * \note this is equivalent to multiplying the constraint Jacobian by the
  * generalized velocities of the joints.
  *
  */
RBDL_DLLAPI
void CalcConstraintsVelocityError(
  Model& model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  ConstraintSet &CS,
  Math::VectorNd& err,
  bool update_kinematics = true
);

/** \brief Computes the terms \f$H\f$, \f$G\f$, and \f$\gamma\f$ of the 
  * constrained dynamic problem and stores them in the ConstraintSet.
  *
  *
  * \param model the model
  * \param Q the generalized positions of the joints
  * \param QDot the generalized velocities of the joints
  * \param Tau the generalized forces of the joints
  * \param CS the constraint set for which the error should be computed
  * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
  *
  * \note This function is normally called automatically in the various
  * constrained dynamics functions, the user normally does not have to call it.
  *
  */
RBDL_DLLAPI
void CalcConstrainedSystemVariables (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  std::vector<Math::SpatialVector> *f_ext = NULL
);

/** \brief Computes a feasible initial value of the generalized joint positions.
  * 
  * \param model the model
  * \param QInit initial guess for the generalized positions of the joints
  * \param CS the constraint set for which the error should be computed
  * \param Q (output) vector of the generalized joint positions.
  * \param weights weighting coefficients for the different joint positions.
  * \param tolerance the function will return successfully if the constraint
  * position error norm is lower than this value.
  * \param max_iter the funciton will return unsuccessfully after performing
  * this number of iterations.
  * 
  * \return true if the generalized joint positions were computed successfully,
  * false otherwise.
  *
  */
RBDL_DLLAPI
bool CalcAssemblyQ(
  Model &model,
  Math::VectorNd QInit,
  ConstraintSet &CS,
  Math::VectorNd &Q,
  const Math::VectorNd &weights,
  double tolerance = 1e-12,
  unsigned int max_iter = 100
);

/** \brief Computes a feasible initial value of the generalized joint velocities.
  * 
  * \param model the model
  * \param Q the generalized joint position of the joints. It is assumed that
  * this vector satisfies the position level assemblt constraints.
  * \param QDotInit initial guess for the generalized velocities of the joints
  * \param CS the constraint set for which the error should be computed
  * \param QDot (output) vector of the generalized joint velocities.
  * \param weights weighting coefficients for the different joint positions.
  *
  */
RBDL_DLLAPI
void CalcAssemblyQDot(
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotInit,
  ConstraintSet &CS,
  Math::VectorNd &QDot,
  const Math::VectorNd &weights
);

/** \brief Computes forward dynamics with contact by constructing and solving 
 *  the full lagrangian equation
 *
 * This method builds and solves the linear system \f[
 \left(
   \begin{array}{cc}
     H & G^T \\
     G & 0
   \end{array}
 \right)
 \left(
   \begin{array}{c}
     \ddot{q} \\
     -\lambda
   \end{array}
 \right)
 =
 \left(
   \begin{array}{c}
     -C + \tau \\
     \gamma
   \end{array}
 \right)
 * \f] where \f$H\f$ is the joint space inertia matrix computed with the
 * CompositeRigidBodyAlgorithm(), \f$G\f$ are the point Jacobians of the
 * contact points, \f$C\f$ the bias force (sometimes called "non-linear
 * effects"), and \f$\gamma\f$ the generalized acceleration independent
 * part of the contact point accelerations.
 *
 * \note This function works with ContactConstraints, LoopConstraints and
 * Custom Constraints. Nonetheless, this method will not tolerate redundant
 * constraints.
 * 
 * \par 
 *
 * \note To increase performance group constraints body and pointwise such
 * that constraints acting on the same body point are sequentially in
 * ConstraintSet. This can save computation of point Jacobians \f$G\f$.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param CS    the description of all acting constraints
 * \param QDDot accelerations of the internals joints (output)
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 * \note During execution of this function values such as
 * ConstraintSet::force get modified and will contain the value
 * of the force acting along the normal.
 *
 */
RBDL_DLLAPI
void ForwardDynamicsConstraintsDirect (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  Math::VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext = NULL
);

RBDL_DLLAPI
void ForwardDynamicsConstraintsRangeSpaceSparse (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  Math::VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext = NULL
);

RBDL_DLLAPI
void ForwardDynamicsConstraintsNullSpace (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  Math::VectorNd &QDDot,
  std::vector<Math::SpatialVector> *f_ext = NULL
);

/** \brief Computes forward dynamics that accounts for active contacts in 
 *  ConstraintSet.
 *
 * The method used here is the one described by Kokkevis and Metaxas in the
 * Paper "Practical Physics for Articulated Characters", Game Developers
 * Conference, 2004.
 *
 * It does this by recursively computing the inverse articulated-body inertia (IABI)
 * \f$\Phi_{i,j}\f$ which is then used to build and solve a system of the form:
 \f[
 \left(
   \begin{array}{c}
     \dot{v}_1 \\
     \dot{v}_2 \\
     \vdots \\
     \dot{v}_n
   \end{array}
 \right)
 =
 \left(
   \begin{array}{cccc}
     \Phi_{1,1} & \Phi_{1,2} & \cdots & \Phi{1,n} \\
     \Phi_{2,1} & \Phi_{2,2} & \cdots & \Phi{2,n} \\
     \cdots & \cdots & \cdots & \vdots \\
     \Phi_{n,1} & \Phi_{n,2} & \cdots & \Phi{n,n} 
   \end{array}
 \right)
 \left(
   \begin{array}{c}
     f_1 \\
     f_2 \\
     \vdots \\
     f_n
   \end{array}
 \right)
 + 
 \left(
   \begin{array}{c}
   \phi_1 \\
   \phi_2 \\
   \vdots \\
   \phi_n
   \end{array}
 \right).
 \f]
 Here \f$n\f$ is the number of constraints and the method for building the system
 uses the Articulated Body Algorithm to efficiently compute entries of the system. The
 values \f$\dot{v}_i\f$ are the constraint accelerations, \f$f_i\f$ the constraint forces,
 and \f$\phi_i\f$ are the constraint bias forces.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param CS a list of all contact points
 * \param QDDot accelerations of the internals joints (output)
 *
 * \note During execution of this function values such as
 * ConstraintSet::force get modified and will contain the value
 * of the force acting along the normal.
 *
 * \note This function supports only contact constraints.
 *
 * \todo Allow for external forces
 */
RBDL_DLLAPI
void ForwardDynamicsContactsKokkevis (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDot,
  const Math::VectorNd &Tau,
  ConstraintSet &CS,
  Math::VectorNd &QDDot
);

/** \brief Computes contact gain by constructing and solving the full lagrangian 
 *  equation
 *
 * This method builds and solves the linear system \f[
 \left(
   \begin{array}{cc}
     H & G^T \\
     G & 0
   \end{array}
 \right)
 \left(
   \begin{array}{c}
     \dot{q}^{+} \\
     \Lambda
   \end{array}
 \right)
 =
 \left(
   \begin{array}{c}
     H \dot{q}^{-} \\
    v^{+} 
   \end{array}
 \right)
 * \f] where \f$H\f$ is the joint space inertia matrix computed with the
 * CompositeRigidBodyAlgorithm(), \f$G\f$ are the point Jacobians of the
 * contact points, \f$\dot{q}^{+}\f$ the generalized velocity after the
 * impact, \f$\Lambda\f$ the impulses at each constraint, \f$\dot{q}^{-}\f$
 * the generalized velocity before the impact, and \f$v^{+}\f$ the desired
 * velocity of each constraint after the impact (known beforehand, usually
 * 0). The value of \f$v^{+}\f$ can is specified via the variable
 * ConstraintSet::v_plus and defaults to 0.
 *
 * \note So far, only constraints acting along cartesian coordinate axes
 * are allowed (i.e. (1, 0, 0), (0, 1, 0), and (0, 0, 1)). Also, one must
 * not specify redundant constraints!
 * 
 * \par 
 *
 * \note To increase performance group constraints body and pointwise such
 * that constraints acting on the same body point are sequentially in
 * ConstraintSet. This can save computation of point Jacobians \f$G\f$.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDotMinus  velocity vector of the internal joints before the impact
 * \param CS the set of active constraints
 * \param QDotPlus velocities of the internals joints after the impact (output)
 */
RBDL_DLLAPI
void ComputeConstraintImpulsesDirect (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
);

/** \brief Resolves contact gain using SolveContactSystemRangeSpaceSparse()
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDotMinus  velocity vector of the internal joints before the impact
 * \param CS the set of active constraints
 * \param QDotPlus velocities of the internals joints after the impact (output)
 */
RBDL_DLLAPI
void ComputeConstraintImpulsesRangeSpaceSparse (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
);

/** \brief Resolves contact gain using SolveContactSystemNullSpace()
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDotMinus  velocity vector of the internal joints before the impact
 * \param CS the set of active constraints
 * \param QDotPlus velocities of the internals joints after the impact (output)
 */
RBDL_DLLAPI
void ComputeConstraintImpulsesNullSpace (
  Model &model,
  const Math::VectorNd &Q,
  const Math::VectorNd &QDotMinus,
  ConstraintSet &CS,
  Math::VectorNd &QDotPlus
);

/** \brief Solves the full contact system directly, i.e. simultaneously for 
 *  contact forces and joint accelerations.
 *
 * This solves a \f$ (n_\textit{dof} +
 * n_c) \times (n_\textit{dof} + n_c\f$ linear system.
 *
 * \param H the joint space inertia matrix
 * \param G the constraint Jacobian
 * \param c the \f$ \mathbb{R}^{n_\textit{dof}}\f$ vector of the upper part of 
 * the right hand side of the system
 * \param gamma the \f$ \mathbb{R}^{n_c}\f$ vector of the lower part of the 
 * right hand side of the system
 * \param lambda result: constraint forces
 * \param A work-space for the matrix of the linear system 
 * \param b work-space for the right-hand-side of the linear system
 * \param x work-space for the solution of the linear system
 * \param linear_solver type of solver that should be used to solve the system
 */
RBDL_DLLAPI
void SolveConstrainedSystemDirect (
  Math::MatrixNd &H, 
  const Math::MatrixNd &G, 
  const Math::VectorNd &c, 
  const Math::VectorNd &gamma, 
  Math::VectorNd &lambda, 
  Math::MatrixNd &A, 
  Math::VectorNd &b,
  Math::VectorNd &x,
  Math::LinearSolver &linear_solver
);

/** \brief Solves the contact system by first solving for the the joint 
 *  accelerations and then the contact forces using a sparse matrix 
 *  decomposition of the joint space inertia matrix.
 *
 * This method exploits the branch-induced sparsity by the structure
 * preserving \f$L^TL \f$ decomposition described in RBDL, Section 6.5.
 *
 * \param model rigid body model
 * \param H the joint space inertia matrix
 * \param G the constraint Jacobian
 * \param c the \f$ \mathbb{R}^{n_\textit{dof}}\f$ vector of the upper part of 
 * the right hand side of the system
 * \param gamma the \f$ \mathbb{R}^{n_c}\f$ vector of the lower part of the 
 * right hand side of the system
 * \param qddot result: joint accelerations
 * \param lambda result: constraint forces
 * \param K work-space for the matrix of the constraint force linear system
 * \param a work-space for the right-hand-side of the constraint force linear 
 * system
 * \param linear_solver type of solver that should be used to solve the 
 * constraint force system
 */
RBDL_DLLAPI
void SolveConstrainedSystemRangeSpaceSparse (
  Model &model, 
  Math::MatrixNd &H, 
  const Math::MatrixNd &G, 
  const Math::VectorNd &c, 
  const Math::VectorNd &gamma, 
  Math::VectorNd &qddot, 
  Math::VectorNd &lambda, 
  Math::MatrixNd &K, 
  Math::VectorNd &a,
  Math::LinearSolver linear_solver
);

/** \brief Solves the contact system by first solving for the joint 
 *  accelerations and then for the constraint forces.
 *
 * This methods requires a \f$n_\textit{dof} \times n_\textit{dof}\f$
 * matrix of the form \f$\left[ \ Y \ | Z \ \right]\f$ with the property
 * \f$GZ = 0\f$ that can be computed using a QR decomposition (e.g. see
 * code for ForwardDynamicsContactsNullSpace()).
 *
 * \param H the joint space inertia matrix
 * \param G the constraint Jacobian
 * \param c the \f$ \mathbb{R}^{n_\textit{dof}}\f$ vector of the upper part of 
 * the right hand side of the system
 * \param gamma the \f$ \mathbb{R}^{n_c}\f$ vector of the lower part of the 
 * right hand side of the system
 * \param qddot result: joint accelerations
 * \param lambda result: constraint forces
 * \param Y basis for the range-space of the constraints
 * \param Z basis for the null-space of the constraints
 * \param qddot_y work-space of size \f$\mathbb{R}^{n_\textit{dof}}\f$
 * \param qddot_z work-space of size \f$\mathbb{R}^{n_\textit{dof}}\f$
 * \param linear_solver type of solver that should be used to solve the system
 */
RBDL_DLLAPI
void SolveConstrainedSystemNullSpace (
  Math::MatrixNd &H, 
  const Math::MatrixNd &G, 
  const Math::VectorNd &c, 
  const Math::VectorNd &gamma, 
  Math::VectorNd &qddot, 
  Math::VectorNd &lambda,
  Math::MatrixNd &Y,
  Math::MatrixNd &Z,
  Math::VectorNd &qddot_y,
  Math::VectorNd &qddot_z,
  Math::LinearSolver &linear_solver
);



} 

/* namespace RigidBodyDynamics */

/* RBDL_CONSTRAINTS_H */
#endif
