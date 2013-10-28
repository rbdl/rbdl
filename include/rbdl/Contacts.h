/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _CONTACTS_H
#define _CONTACTS_H

#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>

namespace RigidBodyDynamics {

/** \page contacts_page External Contacts
 *
 * All functions related to contacts are specified in the \ref
 * contacts_group "Contacts Module".

 * \defgroup contacts_group Contacts
 *
 * External contacts are handled by specification of a \link
 * RigidBodyDynamics::ConstraintSet
 * ConstraintSet \endlink which contains all informations about the
 * current contacts and workspace memory.
 *
 * Separate contacts can be specified by calling
 * ConstraintSet::AddConstraint(). After all constraints have been
 * specified, this \link
 * RigidBodyDynamics::ConstraintSet
 * ConstraintSet \endlink has to be bound to the model via
 * ConstraintSet::Bind(). This initializes workspace memory that is
 * later used when calling one of the contact functions, such as
 * ForwardDynamicsContacts() or ForwardDynamicsContactsLagrangian().
 *
 * The values in the vectors ConstraintSet::force and
 * ConstraintSet::impulse contain the computed force or
 * impulse values for each constraint when returning from one of the
 * contact functions.
 *
*
 * @{
 */

struct Model;

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
		bound (false)
	{}

	/** \brief Adds a constraint to the constraint set.
	 *
	 * \param body_id the body which is affected directly by the constraint
	 * \param body_point the point that is constrained relative to the
	 * contact body
	 * \param world_normal the normal along the constraint acts (in base
	 * coordinates)
	 * \param constraint_name a human readable name (optional, default: NULL)
	 * \param normal_acceleration the acceleration of the contact along the normal
	 * (optional, default: 0.)
	 */
	unsigned int AddConstraint (
			unsigned int body_id,
			const Math::Vector3d &body_point,
			const Math::Vector3d &world_normal,
			const char *constraint_name = NULL,
			double normal_acceleration = 0.);

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
	 */
	bool Bind (const Model &model);

	/** \brief Returns the number of constraints. */
	unsigned int size() {
		return acceleration.size();
	}

	/** \brief Clears all variables in the constraint set. */
	void clear ();

	/// Method that should be used to solve internal linear systems.
	Math::LinearSolver linear_solver;
	/// Whether the constraint set was bound to a model (mandatory!).
	bool bound;

	std::vector<std::string> name;
	std::vector<unsigned int> body;
	std::vector<Math::Vector3d> point;
	std::vector<Math::Vector3d> normal;

	/** Enforced accelerations of the contact points along the contact
	 * normal. */
	Math::VectorNd acceleration;
	/** Actual constraint forces along the contact normals. */
	Math::VectorNd force;
	/** Actual constraint impulses along the contact normals. */
	Math::VectorNd impulse;
	/** The velocities we want to have along the contact normals after
	 * calling ComputeContactImpulsesLagrangian */
	Math::VectorNd v_plus;

	// Variables used by the Lagrangian methods

	/// Workspace for the joint space inertia matrix.
	Math::MatrixNd H;
	/// Workspace for the coriolis forces.
	Math::VectorNd C;
	/// Workspace of the lower part of b.
	Math::VectorNd gamma;
	Math::MatrixNd G;
	/// Workspace for the Lagrangian left-hand-side matrix.
	Math::MatrixNd A;
	/// Workspace for the Lagrangian right-hand-side.
	Math::VectorNd b;
	/// Workspace for the Lagrangian solution.
	Math::VectorNd x;

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

	std::vector<Math::Matrix63> d_multdof3_U;
	std::vector<Math::Matrix3d> d_multdof3_Dinv;
	std::vector<Math::Vector3d> d_multdof3_u;
};

/** \brief Computes forward dynamics with contact by constructing and solving the full lagrangian equation
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
		 \lambda
   \end{array}
 \right)
 =
 \left(
   \begin{array}{c}
	   -C + \tau \\
		 -\gamma
   \end{array}
 \right)
 * \f] where \f$H\f$ is the joint space inertia matrix computed with the
 * CompositeRigidBodyAlgorithm(), \f$G\f$ are the point jacobians of the
 * contact points, \f$C\f$ the bias force (sometimes called "non-linear
 * effects"), and \f$\gamma\f$ the generalized acceleration independent
 * part of the contact point accelerations.
 *
 * \note So far, only constraints acting along cartesian coordinate axes
 * are allowed (i.e. (1, 0, 0), (0, 1, 0), and (0, 0, 1)). Also, one must
 * not specify redundant constraints!
 * 
 * \par 
 *
 * \note To increase performance group constraints body and pointwise such
 * that constraints acting on the same body point are sequentially in
 * ConstraintSet. This can save computation of point jacobians \f$G\f$.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param CS    the description of all acting constraints
 * \param QDDot accelerations of the internals joints (output)
 *
 * \note During execution of this function values such as
 * ConstraintSet::force get modified and will contain the value
 * of the force acting along the normal.
 *
 */
RBDL_DLLAPI
void ForwardDynamicsContactsLagrangian (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDot,
		const Math::VectorNd &Tau,
		ConstraintSet &CS,
		Math::VectorNd &QDDot
		);

/** \brief Computes forward dynamics with contact by constructing and solving the full lagrangian equation
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
 * CompositeRigidBodyAlgorithm(), \f$G\f$ are the point jacobians of the
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
 * ConstraintSet. This can save computation of point jacobians \f$G\f$.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDotMinus  velocity vector of the internal joints before the impact
 * \param CS the set of active constraints
 * \param QDotPlus velocities of the internals joints after the impact (output)
 */
RBDL_DLLAPI
void ComputeContactImpulsesLagrangian (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDotMinus,
		ConstraintSet &CS,
		Math::VectorNd &QDotPlus
		);

/** \brief Computes forward dynamics that accounts for active contacts in ConstraintSet.
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
 * \todo Allow for external forces
 */
RBDL_DLLAPI
void ForwardDynamicsContacts (
		Model &model,
		const Math::VectorNd &Q,
		const Math::VectorNd &QDot,
		const Math::VectorNd &Tau,
		ConstraintSet &CS,
		Math::VectorNd &QDDot
		);

/** @} */

} /* namespace RigidBodyDynamics */

#endif /* _CONTACTS_H */
