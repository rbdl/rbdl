#ifndef _MODEL_H
#define _MODEL_H

#include <cmlwrapper.h>
#include <vector>
#include <assert.h>
#include <iostream>
#include "Logging.h"

#include "Joint.h"
#include "Body.h"

/** \brief Contains all information of the model
 *
 * This class contains all information required to perform the forward
 * dynamics calculation. The variables in this class are also used for storage
 * of temporary values. It is designed for use of the Composite Rigid Body
 * Algorithm and follows the numbering as described in Featherstones book.
 *
 * An important note is that body 0 is the root body and the moving bodies
 * start at index 1. Additionally the vectors for the states q, qdot, etc.
 * have #bodies + 1 entries where always the first entry (e.g. q[0]) contains
 * the value for the root body. Thus the numbering might be confusing as q[1]
 * holds the position variable of the first degree of freedom. This numbering
 * scheme is very benefial in terms of readability of the code as the
 * resulting code is very similar to the pseudo-code in the RBDA book.
 */
struct Model {
	// Structural information

	/// \brief The id of the parents body
	std::vector<unsigned int> lambda;

	/// \brief true if the body has a floating base
	bool floating_base;
	/// \brief the cartestian translation of the base
	Vector3d base_translation;
	/// \brief the rotation of the base in ZYX-Euler angles
	Vector3d base_rotation;

	/// \brief the cartesian vector of the gravity
	Vector3d gravity;

	// State information

	/** \brief The joint position
	 * 
	 * Warning: to have an easier numbering in the algorithm the state vector
	 * has NDOF + 1 elements. However element with index 0 is not used!
	 * 
	 * q[0] - unused
	 * q[1] - joint 1
	 * q[2] - joint 2
	 * ...
	 * q[NDOF] - joint NDOF
	 *
	 */
	std::vector<double> q;
	/// \brief The joint velocity
	std::vector<double> qdot;
	/// \brief The joint acceleration
	std::vector<double> qddot;
	/// \brief The force / torque applied at joint i
	std::vector<double> tau;
	/// \brief The spatial velocity of body i
	std::vector<SpatialAlgebra::SpatialVector> v;
	/// \brief The spatial acceleration of body i
	std::vector<SpatialAlgebra::SpatialVector> a;

	////////////////////////////////////
	// Joints\t

	/// \brief All joints
	std::vector<Joint> mJoints;
	/// \brief The joint axis for joint i
	std::vector<SpatialAlgebra::SpatialVector> S;
	/// \brief Transformations from the parent body to the frame of the joint
	std::vector<SpatialAlgebra::SpatialMatrix> X_T;

	////////////////////////////////////
	// Dynamics variables

	/// \brief The velocity dependent spatial acceleration
	std::vector<SpatialAlgebra::SpatialVector> c;
	/// \brief The spatial inertia of body i
	std::vector<SpatialAlgebra::SpatialMatrix> IA;
	/// \brief The spatial bias force
	std::vector<SpatialAlgebra::SpatialVector> pA;
	/// \brief Temporary variable U_i (RBDA p. 130)
	std::vector<SpatialAlgebra::SpatialVector> U;
	/// \brief Temporary variable D_i (RBDA p. 130)
	std::vector<double> d;
	/// \brief Temporary variable u (RBDA p. 130)
	std::vector<double> u;

	////////////////////////////////////
	// Bodies

	/// \brief Transformation from the parent body to the current body
	std::vector<SpatialAlgebra::SpatialMatrix> X_lambda;
	/// \brief Transformation from the base to bodies reference frame
	std::vector<SpatialAlgebra::SpatialMatrix> X_base;

	/** \brief All bodies 0 ... N_B, including the base
	 * mBodies[0] - base body
	 * mBodies[1] - 1st movable body
	 * ...
	 * mBodies[N_B] - N_Bth movable body
	 */
	std::vector<Body> mBodies;
	std::vector<Matrix3d> mBodyOrientation;

	/// \brief Initializes the helper values for the dynamics algorithm
	void Init ();
	/** \brief Connects a given body to the model
	 *
	 * \param parent_id   id of the parend body
	 * \param joint_frame the transformation from the parent frame to the origin
	 *                    of the joint frame (represents X_T in RBDA)
	 * \param joint       specification for the joint that describes the connection
	 * \param body        specification of the body itself
	 *
	 * \returns id of the added body
	 */
	unsigned int AddBody (
			const unsigned int parent_id,
			const SpatialAlgebra::SpatialMatrix &joint_frame,
			const Joint &joint,
			const Body &body
			);
	void SetFloatingBody (
			const Body &body
			);
	unsigned int AddContact (
			const unsigned int body_id,
			const Vector3d &contact_point
			);
};

/** \brief Computes the joint variables 
 *
 * \param model    the rigid body model
 * \param joint_id the id of the joint we are interested in (output)
 * \param XJ       the joint transformation (output)
 * \param S        motion subspace of the joint (output)
 * \param v_J      joint velocity (output)
 * \param c_J      joint acceleration for rhenomic joints (output)
 * \param q        joint state variable
 * \param qdot     joint velocity variable
 */
void jcalc (
		const Model &model,
		const unsigned int &joint_id,
		SpatialAlgebra::SpatialMatrix &XJ,
		SpatialAlgebra::SpatialVector &S,
		SpatialAlgebra::SpatialVector &v_J,
		SpatialAlgebra::SpatialVector &c_J,
		const double &q,
		const double &qdot
		);

/** \brief Computes forward dynamics for models with a fixed base
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internals joints (output)
 */
void ForwardDynamics (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &Tau,
		std::vector<double> &QDDot
		);

/** \brief Computes forward dynamics for models with a floating base
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param X_B   transformation into base coordinates
 * \param v_B   velocity of the base (in base coordinates)
 * \param f_B   forces acting on the base (in base coordinates)
 * \param a_B   accelerations of the base (output, in base coordinates)
 * \param QDDot accelerations of the internals joints (output)
 */
void ForwardDynamicsFloatingBase (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &Tau,
		const SpatialAlgebra::SpatialMatrix &X_B,
		const SpatialAlgebra::SpatialVector &v_B,
		const SpatialAlgebra::SpatialVector &f_B,
		SpatialAlgebra::SpatialVector &a_B,
		std::vector<double> &QDDot
		);

/** \brief Computes the velocity of a point on a body 
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param point_velocity cartesian velocity of the point in global frame (output)
 */
void CalcPointVelocity (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_velocity
		);

/** \brief Computes the acceleration of a point on a body 
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param QDDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param point_acceleration cartesian velocity of the point in global frame (output)
 */
void CalcPointAcceleration (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_acceleration
		);

#endif /* _MODEL_H */
