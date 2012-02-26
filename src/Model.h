/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MODEL_H
#define _MODEL_H

#include <rbdl_math.h>
#include <map>
#include <list>
#include <assert.h>
#include <iostream>
#include <limits>

#include "Logging.h"
#include "Joint.h"
#include "Body.h"

// std::vectors containing any objectst that have Eigen matrices or vectors
// as members need to have a special allocater. This can be achieved with
// the following macro.

#ifdef EIGEN_CORE_H
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Joint);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Body);
#endif

/** \brief Namespace for all structures of the RigidBodyDynamics library
 */
namespace RigidBodyDynamics {

/** \defgroup model_group Modelling
 * @{
 *
 * All model related values are stored in the model structure \link
 * RigidBodyDynamics::Model\endlink. The functions 
 * \link RigidBodyDynamics::Model::Init Model::Init()\endlink,
 * \link RigidBodyDynamics::Model::AddBody Model::AddBody(...)\endlink,
 * \link RigidBodyDynamics::Model::AppendBody Model::AppendBody(...)\endlink, and
 * \link RigidBodyDynamics::Model::GetBodyId Model::GetBodyId(...)\endlink,
 * are used to initialize and construct the \ref model_structure.
 *
 * Once this is done, the model structure can be used with the functions of \ref
 * kinematics_group, \ref dynamics_group, \ref contacts_group, to perform
 * computations.
 *
 * \section model_structure Model Structure
 *
 * The model structure contains all the parameters of the rigid multi-body
 * model such as joint informations, mass and inertial parameters of the
 * rigid bodies, etc.
 *
 * Furthermore the current state of the model, such as the
 * generalized positions, velocities and accelerations is also stored
 * within the model. Angles are always stored as radians. This internal
 * state is used when functions such as \link
 * RigidBodyDynamics::Kinematics::CalcBodyToBaseCoordinates
 * Kinematics::CalcBodyToBaseCoordinates(...)\endlink, \link
 * RigidBodyDynamics::Kinematics::CalcPointJacobian
 * Kinematics::CalcPointJacobian\endlink are called.
 *
 * \note Please note that in the Rigid %Body Dynamics Library all angles
 * are specified in radians.
 */

/** \brief Contains all information about the rigid body model
 *
 * This class contains all information required to perform the forward
 * dynamics calculation. The variables in this class are also used for
 * storage of temporary values. It is designed for use of the Articulated
 * Rigid Body Algorithm (which is implemented in ForwardDynamics()) and
 * follows the numbering as described in Featherstones book.
 *
 * An important note is that body 0 is the root body and the moving bodies
 * start at index 1. Additionally the vectors for the states q, qdot, etc.
 * have \#Model::dof_count + 1 entries where always the first entry (e.g. q[0])
 * contains the value for the base (or "root" body). Thus the numbering might be
 * confusing as q[1] holds the position variable of the first degree of
 * freedom. This numbering scheme is very beneficial in terms of readability
 * of the code as the resulting code is very similar to the pseudo-code in
 * the RBDA book.
 */
struct Model {
	// Structural information

	/// \brief The id of the parents body
	std::vector<unsigned int> lambda;
	/// \brief Contains the ids of all the children of a given body
	std::vector<std::vector<unsigned int> >mu;

	/** \brief Use floating base extension as described in RBDA chapter 9.4
	 *
	 * \warning This function is experimental and produces wrong results. Do
	 * \warning not use!
	 */
	bool experimental_floating_base;

	/** \brief number of degrees of freedoms of the model
	 *
	 * This value contains the number of entries in the generalized state (q)
	 * velocity (qdot), acceleration (qddot), and force (tau) vector.
	 */
	unsigned int dof_count;

	/// \brief the cartesian vector of the gravity
	Math::Vector3d gravity;

	// State information

	/** \brief The joint position
	 * 
	 * Warning: to have an easier numbering in the algorithm the state vector
	 * has NDOF + 1 elements. However element with index 0 is not used!
	 * 
	 * q[0] - unused <br>
	 * q[1] - joint 1 <br>
	 * q[2] - joint 2 <br>
	 * ... <br>
	 * q[NDOF] - joint NDOF <br>
	 *
	 */
	Math::VectorNd q;
	/// \brief The joint velocity
	Math::VectorNd qdot;
	/// \brief The joint acceleration
	Math::VectorNd qddot;
	/// \brief The force / torque applied at joint i
	Math::VectorNd tau;
	/// \brief The spatial velocity of body i
	std::vector<Math::SpatialVector> v;
	/// \brief The spatial acceleration of body i
	std::vector<Math::SpatialVector> a;

	////////////////////////////////////
	// Joints

	/// \brief All joints
	
	std::vector<Joint> mJoints;
	/// \brief The joint axis for joint i
	std::vector<Math::SpatialVector> S;
	/// \brief Transformations from the parent body to the frame of the joint
	std::vector<Math::SpatialTransform> X_T;

	////////////////////////////////////
	// Dynamics variables

	/// \brief The velocity dependent spatial acceleration
	std::vector<Math::SpatialVector> c;
	/// \brief The spatial inertia of body i
	std::vector<Math::SpatialMatrix> IA;
	/// \brief The spatial bias force
	std::vector<Math::SpatialVector> pA;
	/// \brief Temporary variable U_i (RBDA p. 130)
	std::vector<Math::SpatialVector> U;
	/// \brief Temporary variable D_i (RBDA p. 130)
	Math::VectorNd d;
	/// \brief Temporary variable u (RBDA p. 130)
	Math::VectorNd u;
	/// \brief Internal forces on the body (used only InverseDynamics())
	std::vector<Math::SpatialVector> f;
	/// \brief The spatial inertia of body i (used only in CompositeRigidBodyAlgorithm())
	std::vector<Math::SpatialMatrix> Ic;

	////////////////////////////////////
	// Bodies

	/// \brief Transformation from the parent body to the current body
	std::vector<Math::SpatialTransform> X_lambda;
	/// \brief Transformation from the base to bodies reference frame
	std::vector<Math::SpatialTransform> X_base;

	/** \brief All bodies 0 ... N_B, including the base
	 *
	 * mBodies[0] - base body <br>
	 * mBodies[1] - 1st moveable body <br>
	 * ... <br>
	 * mBodies[N_B] - N_Bth moveable body <br>
	 */
	std::vector<Body> mBodies;

	/// \brief Human readable names for the bodies
	std::vector<std::string> mBodyNames;

	/** \brief Connects a given body to the model
	 *
	 * When adding a body there are basically informations required:
	 * - what kind of body will be added?
	 * - where is the new body to be added?
	 * - by what kind of joint should the body be added?
	 *
	 * The first information "what kind of body will be added" is contained
	 * in the Body class that is given as a parameter.
	 *
	 * The question "where is the new body to be added?" is split up in two
	 * parts: first the parent (or successor) body to which it is added and
	 * second the transformation to the origin of the joint that connects the
	 * two bodies. With these two informations one specifies the relative
	 * positions of the bodies when the joint is in neutral position.gk
	 *
	 * The last question "by what kind of joint should the body be added?" is
	 * again simply contained in the Joint class.
	 *
	 * \param parent_id   id of the parent body
	 * \param joint_frame the transformation from the parent frame to the origin
	 *                    of the joint frame (represents X_T in RBDA)
	 * \param joint       specification for the joint that describes the connection
	 * \param body        specification of the body itself
	 * \param body_name   human readable name for the body (can be used to retrieve its id
	 *                    with GetBodyId())
	 *
	 * \returns id of the added body
	 */
	unsigned int AddBody (
			const unsigned int parent_id,
			const Math::SpatialTransform &joint_frame,
			const Joint &joint,
			const Body &body,
			std::string body_name = "" 
			);

	/** \brief Adds a Body to the model such that the previously added Body is the Parent.
	 *
	 * This function is basically the same as Model::AddBody() however the
	 * most recently added body (or body 0) is taken as parent.
	 */
	unsigned int AppendBody (
			const Math::SpatialTransform &joint_frame,
			const Joint &joint,
			const Body &body,
			std::string body_name = "" 
			);

	/** \brief Specifies the dynamical parameters of the first body and
	 *  \brief assigns it a 6 DoF joint.
	 *
	 * The 6 DoF joint is simulated by adding 5 massless bodies at the base
	 * which are connected with joints. The body that is specified as a
	 * parameter of this function is then added by a 6th joint to the model.
	 *
	 * The floating base has the following order of degrees of freedom:
	 * 
	 * \li translation X
	 * \li translation Y
	 * \li translation Z
	 * \li rotation Z
	 * \li rotation Y
	 * \li rotation X
	 *
	 * \param body Properties of the floating base body.
	 *
	 *  \returns id of the body with 6 DoF
	 */
	unsigned int SetFloatingBaseBody (
			const Body &body
			);

	/** \brief Returns the id of a body that was passed to AddBody()
	 *
	 * Bodies can be given a human readable name. This function allows to
	 * resolve its name to the numeric id.
	 *
	 * \note Instead of querying this function repeatedly, it might be
	 * advisable to query it once and reuse the returned id.
	 *
	 * \returns the id of the body or \c std::numeric_limits<unsigned int>::max() if the id was not found.
	 */
	unsigned int GetBodyId (const char *id);

	/// \brief Initializes the helper values for the dynamics algorithm
	void Init ();
};

/** @} */

}

#endif /* _MODEL_H */
