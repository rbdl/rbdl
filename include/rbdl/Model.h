/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _MODEL_H
#define _MODEL_H

#include "rbdl/rbdl_math.h"
#include <map>
#include <list>
#include <assert.h>
#include <iostream>
#include <limits>
#include <cstring>

#include "rbdl/Logging.h"
#include "rbdl/Joint.h"
#include "rbdl/Body.h"

// std::vectors containing any objectst that have Eigen matrices or vectors
// as members need to have a special allocater. This can be achieved with
// the following macro.

#ifdef EIGEN_CORE_H
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Joint);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Body);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::FixedBody);
#endif

/** \brief Namespace for all structures of the RigidBodyDynamics library
 */
namespace RigidBodyDynamics {

/** \page modeling_page Model 
 *
 * There are two ways of creating \link RigidBodyDynamics::Model Models\endlink for RBDL:
 *
 *   \li Using \ref luamodel_introduction that uses Lua files or
 *   \li using the C++ interface.
 *
 * The first approach requires the addon \ref luamodel_introduction to be
 * activated which is done by enabling BUILD_ADDON_LUAMODEL in CMake and is
 * recommended when one is not interested in the details of RBDL and simply
 * wants to create a model.
 *
 * \section modeling_lua Modeling using Lua
 *
 * For this see the documentation of \ref luamodel_introduction.
 *
 * \section modeling_cpp Modeling using C++
 *
 * The API for the RBDL Models can be found at \ref RigidBodyDynamics::Model.
 *
 * Using the C++ interface is more advanced but gives some overview about the
 * internals of RBDL.
 *
 * \subsection modeling_overview Overview
 *
 * All model related values are stored in the model structure \link
 * RigidBodyDynamics::Model\endlink. The functions 
 * \link RigidBodyDynamics::Model::AddBody Model::AddBody(...)\endlink,
 * \link RigidBodyDynamics::Model::AppendBody Model::AppendBody(...)\endlink, and
 * \link RigidBodyDynamics::Model::GetBodyId Model::GetBodyId(...)\endlink,
 * are used to construct the \ref model_structure.
 *
 * \section model_construction Model Construction
 *
 * The construction of \link RigidBodyDynamics::Model Model Structures \endlink makes
 * use of carefully designed constructors of the classes \link
 * RigidBodyDynamics::Body Body \endlink and \link RigidBodyDynamics::Joint
 * Joint \endlink to ease the process of articulated models. Adding bodies to
 * the model is done by specifying the parent body by its id, the
 * transformation from the parent origin to the joint origin, the joint
 * specification as an object, and the body itself. These parameters are
 * then fed to the function RigidBodyDynamics::Model::AddBody().
 *
 * To create a model with a floating base (a.k.a a model with a free-flyer
 * joint) it is recommended to use \link
 * RigidBodyDynamics::Model::SetFloatingBaseBody
 * Model::SetFloatingBaseBody(...)\endlink.
 *
 * Once this is done, the model structure can be used with the functions of \ref
 * kinematics_group, \ref dynamics_group, \ref contacts_page, to perform
 * computations.
 *
 * A simple example can be found \ref SimpleExample "here".
 *
 * \subsection model_structure Model Structure
 
 * The \link RigidBodyDynamics::Model Model Structure \endlink contains all
 * the parameters of the rigid multi-body model such as joint informations,
 * mass and inertial parameters of the rigid bodies, etc. It also contains
 * storage for the transformations and current state, such as velocity and
 * acceleration of each body.
 
 * See also \ref joint_description for information about Joint modeling.
 */

/** \brief Contains all information about the rigid body model
 *
 * This class contains all information required to perform the forward
 * dynamics calculation. The variables in this class are also used for
 * storage of temporary values. It is designed for use of the Articulated
 * Rigid Body Algorithm (which is implemented in ForwardDynamics()) and
 * follows the numbering as described in Featherstones book.
 * 
 * Please note that body 0 is the root body and the moving bodies start at
 * index 1. This numbering scheme is very beneficial in terms of
 * readability of the code as the resulting code is very similar to the
 * pseudo-code in the RBDA book. The generalized variables q, qdot, qddot
 * and tau however start at 0 such that the first entry (e.g. q[0]) always
 * specifies the value for the first moving body.
 *
 * \note To query the number of degrees of freedom use Model::dof_count.
 */
struct RBDL_DLLAPI Model {
	Model();

	// Structural information

	/// \brief The id of the parents body
	std::vector<unsigned int> lambda;
	/// \brief Contains the ids of all the children of a given body
	std::vector<std::vector<unsigned int> >mu;

	/** \brief number of degrees of freedoms of the model
	 *
	 * This value contains the number of entries in the generalized state (q)
	 * velocity (qdot), acceleration (qddot), and force (tau) vector.
	 */
	unsigned int dof_count;

	/** \brief The size of the \f$\mathbf{q}\f$-vector. 
	 * For models without spherical joints the value is the same as
	 * Model::dof_count, otherwise additional values for the w-component of the
	 * Quaternion is stored at the end of \f$\mathbf{q}\f$. 
	 *
	 * \sa \ref joint_description for more details.
	 */
	unsigned int q_size;
	/** \brief The size of the \f$\mathbf{\dot{q}}, \mathbf{\ddot{q}}\f$, and \f$\mathbf{\tau}\f$-vector. 
	 *
	 * \sa \ref joint_description for more details.
	 */
	unsigned int qdot_size;

	/// \brief Id of the previously added body, required for Model::AppendBody()
	unsigned int previously_added_body_id;

	/// \brief the cartesian vector of the gravity
	Math::Vector3d gravity;

	// State information
	/// \brief The spatial velocity of the bodies
	std::vector<Math::SpatialVector> v;
	/// \brief The spatial acceleration of the bodies
	std::vector<Math::SpatialVector> a;

	////////////////////////////////////
	// Joints

	/// \brief All joints
	
	std::vector<Joint> mJoints;
	/// \brief The joint axis for joint i
	std::vector<Math::SpatialVector> S;
	/// \brief Transformations from the parent body to the frame of the joint.
	// It is expressed in the coordinate frame of the parent.
	std::vector<Math::SpatialTransform> X_T;
	/// \brief The number of fixed joints that have been declared before each joint.
	std::vector<unsigned int> mFixedJointCount;

	////////////////////////////////////
	// Special variables for joints with 3 degrees of freedom
	/// \brief Motion subspace for joints with 3 degrees of freedom
	std::vector<Math::Matrix63> multdof3_S;
	std::vector<Math::Matrix63> multdof3_U;
	std::vector<Math::Matrix3d> multdof3_Dinv;
	std::vector<Math::Vector3d> multdof3_u;
	std::vector<unsigned int> multdof3_w_index;

	////////////////////////////////////
	// Dynamics variables

	/// \brief The velocity dependent spatial acceleration
	std::vector<Math::SpatialVector> c;
	/// \brief The spatial inertia of the bodies 
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
	std::vector<Math::SpatialRigidBodyInertia> Ic;
	std::vector<Math::SpatialVector> hc;

	////////////////////////////////////
	// Bodies

	/** \brief Transformation from the parent body to the current body
	 * \f[
	 *	X_{\lambda(i)} = {}^{i} X_{\lambda(i)}
	 * \f]
	 */
	std::vector<Math::SpatialTransform> X_lambda;
	/// \brief Transformation from the base to bodies reference frame
	std::vector<Math::SpatialTransform> X_base;

	/// \brief All bodies that are attached to a body via a fixed joint.
	std::vector<FixedBody> mFixedBodies;
	/** \brief Value that is used to discriminate between fixed and movable
	 * bodies.
	 *
	 * Bodies with id 1 .. (fixed_body_discriminator - 1) are moving bodies
	 * while bodies with id fixed_body_discriminator .. max (unsigned int)
	 * are fixed to a moving body. The value of max(unsigned int) is
	 * determined via std::numeric_limits<unsigned int>::max() and the
	 * default value of fixed_body_discriminator is max (unsigned int) / 2.
	 * 
	 * On normal systems max (unsigned int) is 4294967294 which means there
	 * could be a total of 2147483646 movable and / or fixed bodies.
	 */
	unsigned int fixed_body_discriminator;

	/** \brief All bodies 0 ... N_B, including the base
	 *
	 * mBodies[0] - base body <br>
	 * mBodies[1] - 1st moveable body <br>
	 * ... <br>
	 * mBodies[N_B] - N_Bth moveable body <br>
	 */
	std::vector<Body> mBodies;

	/// \brief Human readable names for the bodies
	std::map<std::string, unsigned int> mBodyNameMap;

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

	unsigned int AddBodySphericalJoint (
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
	 * To specify a different ordering, it is recommended to create a 6 DoF
	 * joint. See \link RigidBodyDynamics::Joint Joint\endlink for more
	 * information.
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
	unsigned int GetBodyId (const char *body_name) const {
		if (mBodyNameMap.count(body_name) == 0) {
			return std::numeric_limits<unsigned int>::max();
		}

		return mBodyNameMap.find(body_name)->second;
	}

	/** \brief Returns the name of a body for a given body id */
	std::string GetBodyName (unsigned int body_id) const {
		std::map<std::string, unsigned int>::const_iterator iter = mBodyNameMap.begin();

		while (iter != mBodyNameMap.end()) {
			if (iter->second == body_id)
				return iter->first;

			iter++;
		}

		return "";
	}

	/** \brief Checks whether the body is rigidly attached to another body.
	 */
	bool IsFixedBodyId (unsigned int body_id) {
		if (body_id >= fixed_body_discriminator 
				&& body_id < std::numeric_limits<unsigned int>::max() 
				&& body_id - fixed_body_discriminator < mFixedBodies.size()) {
			return true;
		}
		return false;
	}

	bool IsBodyId (unsigned int id) {
		if (id > 0 && id < mBodies.size())
			return true;
		if (id >= fixed_body_discriminator && id < std::numeric_limits<unsigned int>::max()) {
			if (id - fixed_body_discriminator < mFixedBodies.size())
				return true;
		}
		return false;
	}

	/** Determines id the actual parent body.
	 *
	 * When adding bodies using joints with multiple degrees of
	 * freedom, additional virtual bodies are added for each degree of
	 * freedom. This function returns the id of the actual
	 * non-virtual parent body.
	 */
	unsigned int GetParentBodyId (unsigned int id) {
		if (id >= fixed_body_discriminator) {
			return mFixedBodies[id - fixed_body_discriminator].mMovableParent;
		}

		unsigned int parent_id = lambda[id]; 
	
		while (mBodies[parent_id].mIsVirtual) {
			parent_id = lambda[parent_id];
		}

		return parent_id;
	}

	/** Returns the joint frame transformtion, i.e. the second argument to Model::AddBody().
	 */
	Math::SpatialTransform GetJointFrame (unsigned int id) {
		if (id >= fixed_body_discriminator) {
			return mFixedBodies[id - fixed_body_discriminator].mParentTransform;
		}

		unsigned int child_id = id;
		unsigned int parent_id = lambda[id];
		if (mBodies[parent_id].mIsVirtual) {
			while (mBodies[parent_id].mIsVirtual) {
				child_id = parent_id;
				parent_id = lambda[child_id];
			}
			return X_T[child_id];
		} else
			return X_T[id];	
	}

	/** Sets the joint frame transformtion, i.e. the second argument to Model::AddBody().
	 */
	void SetJointFrame (unsigned int id, const Math::SpatialTransform &transform) {
		if (id >= fixed_body_discriminator) {
			std::cerr << "Error: setting of parent transform not supported for fixed bodies!" << std::endl;
			abort();
		}

		unsigned int child_id = id;
		unsigned int parent_id = lambda[id];
		if (mBodies[parent_id].mIsVirtual) {
			while (mBodies[parent_id].mIsVirtual) {
				child_id = parent_id;
				parent_id = lambda[child_id];
			}
			X_T[child_id] = transform;
		} else if (id > 0) {
			X_T[id] = transform;
		}
	}

	Math::Quaternion GetQuaternion (unsigned int i, const Math::VectorNd &Q) const {
		assert (mJoints[i].mJointType == JointTypeSpherical);
		unsigned int q_index = mJoints[i].q_index;
		return Math::Quaternion (Q[q_index], Q[q_index + 1], Q[q_index + 2], Q[multdof3_w_index[i]]);
	}

	void SetQuaternion (unsigned int i, const Math::Quaternion &quat, Math::VectorNd &Q) const {
		assert (mJoints[i].mJointType == JointTypeSpherical);
		unsigned int q_index = mJoints[i].q_index;
		
		Q[q_index] = quat[0];
		Q[q_index + 1] = quat[1];
		Q[q_index + 2] = quat[2];
		Q[multdof3_w_index[i]] = quat[3];
	}
};

/** @} */
}

#endif /* _MODEL_H */
