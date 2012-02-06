/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _CONTACTS_H
#define _CONTACTS_H

#include "mathwrapper.h"

namespace RigidBodyDynamics {

struct Model;

/** \defgroup contacts_group External Contacts
 *
 * Here you find information about the functions related to contact
 * handling.
 *
 * @{
 */

/** Structure that contains both constraint information and memory workspace.
 *
 */
struct ConstraintSet {
	ConstraintSet() :
		linear_solver (LinearSolverColPivHouseholderQR),
		bound (false)
	{}

	enum LinearSolver {
		LinearSolverUnknown = 0,
		LinearSolverPartialPivLU,
		LinearSolverColPivHouseholderQR,
		LinearSolverLast,
	};

	unsigned int AddConstraint (
			unsigned int body_id,
			const Vector3d &body_point,
			const Vector3d &world_normal,
			const char *constraint_name = NULL,
			double acceleration = 0.);

	void SetSolver (LinearSolver solver) {
		linear_solver = solver;
	}

	bool Bind (const Model &model);

	unsigned int size() {
		return constraint_acceleration.size();
	}

	void clear ();

	/// Method that should be used to solve internal linear systems.
	LinearSolver linear_solver;
	/// Whether the constraint set was bound to a model (mandatory!).
	bool bound;

	std::vector<std::string> name;
	std::vector<unsigned int> body;
	std::vector<Vector3d> point;
	std::vector<Vector3d> normal;

	VectorNd constraint_acceleration;
	VectorNd constraint_force;

	// Variables used by the Lagrangian methods

	/// Workspace for the joint space inertia matrix.
	MatrixNd H;
	/// Workspace for the coriolis forces.
	VectorNd C;
	/// Workspace of the lower part of b.
	VectorNd gamma;
	MatrixNd G;
	/// Workspace for the Lagrangian left-hand-side matrix.
	MatrixNd A;
	/// Workspace for the Lagrangian right-hand-side.
	VectorNd b;
	/// Workspace for the Lagrangian solution.
	VectorNd x;

	// Variables used by the IABI methods

	/// Workspace for the Inverse Articulated-Body Inertia.
	MatrixNd K;
	/// Workspace for the accelerations of due to the test forces
	VectorNd a;
	/// Workspace for the test accelerations.
	VectorNd QDDot_t;
	/// Workspace for the default accelerations.
	VectorNd QDDot_0;
	/// Workspace for the test forces.
	std::vector<SpatialAlgebra::SpatialVector> f_t;
	/// Workspace for the actual spatial forces.
	std::vector<SpatialAlgebra::SpatialVector> f_ext_constraints;
	/// Workspace for the default point accelerations.
	std::vector<Vector3d> point_accel_0;

	/// Workspace for the bias force due to the test force
	std::vector<SpatialAlgebra::SpatialVector> d_pA;
	/// Workspace for the acceleration due to the test force
	std::vector<SpatialAlgebra::SpatialVector> d_a;
	VectorNd d_u;

	/// Workspace for the inertia when applying constraint forces
	std::vector<SpatialAlgebra::SpatialMatrix> d_IA;
	/// Workspace when applying constraint forces
	std::vector<SpatialAlgebra::SpatialVector> d_U;
	/// Workspace when applying constraint forces
	VectorNd d_d;
};

/** \brief Structure that contains information about a one-dimensional
 *  \brief contact constraint
 *
 *  This structure is also used to describe contact points that undergo an
 *  impulse, see alse ComputeContactImpulses().
 */
struct ContactInfo {
	ContactInfo() :
		body_id (0),
		point (0., 0., 0.),
		normal (0., 0., 0.),
		acceleration (0.),
		force (0.)
	{	}
	ContactInfo (const ContactInfo &contact_info) :
		body_id (contact_info.body_id),
		point (contact_info.point),
		normal (contact_info.normal),
		acceleration (contact_info.acceleration),
		force (contact_info.force)
	{}
	ContactInfo& operator= (const ContactInfo &contact_info) {
		if (this != &contact_info) {
			body_id = contact_info.body_id;
			point = contact_info.point;
			normal = contact_info.normal;
			acceleration = contact_info.acceleration;
			force = contact_info.force;
		}

		return *this;
	}
	~ContactInfo() {};

	ContactInfo (unsigned int body, const Vector3d &contact_point, const Vector3d &contact_normal):
		body_id (body),
		point (contact_point),
		normal (contact_normal),
		acceleration (0.),
		force (0.)
	{	}

	ContactInfo (unsigned int body, const Vector3d &contact_point, const Vector3d &contact_normal, const double accel):
		body_id (body),
		point (contact_point),
		normal (contact_normal),
		acceleration (accel),
		force (force)
	{	}

	/// \brief The id of the body of which the motion is constrained
	unsigned int body_id;
	/// \brief Coordinate of the contact point in base coordinates
	Vector3d point;
	/// \brief Normal of the contact point in base coordinates
	Vector3d normal;
	/// \brief Acceleration value of the constraint along the normal
	double acceleration;
	/// \brief Force acting along the normal
	double force;
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
 * ContactData. This can save computation of point jacobians \f$G\f$.
  *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param ContactData	a list of all contact points
 * \param QDDot accelerations of the internals joints (output)
 *
 * \note During execution of this function the values ContactData[i].force
 * 	get modified and will contain the value of the force acting along
 * 	the normal.
 */
void ForwardDynamicsContactsLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		ConstraintSet &CS,
		VectorNd &QDDot
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
 * 0).
 *
 * The desired velocity can be specified by ContactInfo::acceleration.
 * 
 * \note So far, only constraints acting along cartesian coordinate axes
 * are allowed (i.e. (1, 0, 0), (0, 1, 0), and (0, 0, 1)). Also, one must
 * not specify redundant constraints!
 * 
 * \par 
 *
 * \note To increase performance group constraints body and pointwise such
 * that constraints acting on the same body point are sequentially in
 * ContactData. This can save computation of point jacobians \f$G\f$.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDotMinus  velocity vector of the internal joints before the impact
 * \param ContactData	a list of all contact points
 * \param QDotPlus velocities of the internals joints after the impact (output)
 *
 * \note During execution of this function the values ContactInfo::force
 * 	get modified and will contain the value of the impulse acting along
 * 	the normal.
 */
void ComputeContactImpulsesLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDotMinus,
		std::vector<ContactInfo> &ContactData,
		VectorNd &QDotPlus
		);

/** \brief Computes forward dynamics that accounts for active contacts in mContactInfoMap
 *
 * The method used here is the one described by Kokkevis and Metaxas in the
 * Paper "Efficient Dynamic Constraints for Animating Articulated Figures,
 * Multibody System Dynamics 2, 89-114, 1998.
 *
 * This function is superseeded by \ref RigidBodyDynamics::ForwardDynamicsContacts
 * the same approach but faster.
 * 
 * \todo Allow for external forces
 */
void ForwardDynamicsContactsOld (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		std::vector<ContactInfo> &ContactData,
		VectorNd &QDDot
		);

/** \brief Computes forward dynamics that accounts for active contacts in ContactData
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
 * \param ContactData	a list of all contact points
 * \param QDDot accelerations of the internals joints (output)
 *
 * \note During execution of this function the values ContactData[i].force
 * 	get modified and will contain the value of the force acting along
 * 	the normal.
 *
 * \todo Allow for external forces
 */void ForwardDynamicsContacts (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		ConstraintSet &CS,
		VectorNd &QDDot
		);

/** @} */

} /* namespace RigidBodyDynamics */

#endif /* _CONTACTS_H */
