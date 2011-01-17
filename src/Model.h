#ifndef _MODEL_H
#define _MODEL_H

#include <cmlwrapper.h>
#include <vector>
#include <map>
#include <assert.h>
#include <iostream>
#include "Logging.h"

#include "Joint.h"
#include "Body.h"
#include "Contacts.h"

/** \brief Contains all information of the model
 *
 * This class contains all information required to perform the forward
 * dynamics calculation. The variables in this class are also used for
 * storage of temporary values. It is designed for use of the Composite
 * Rigid Body Algorithm and follows the numbering as described in
 * Featherstones book.
 *
 * An important note is that body 0 is the root body and the moving bodies
 * start at index 1. Additionally the vectors for the states q, qdot, etc.
 * have #bodies + 1 entries where always the first entry (e.g. q[0])
 * contains the value for the root body. Thus the numbering might be
 * confusing as q[1] holds the position variable of the first degree of
 * freedom. This numbering scheme is very benefial in terms of readability
 * of the code as the resulting code is very similar to the pseudo-code in
 * the RBDA book.
 */
struct Model {
	// Structural information

	/// \brief The id of the parents body
	std::vector<unsigned int> lambda;
	/// \brief Contains the ids of all the children of a given body
	std::vector<std::vector<unsigned int> >mu;

	/// \brief true if the body has a floating base
	bool floating_base;
	/// \brief true if contact information in mContactInfoMap should be used
	bool use_contacts;

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
	 * q[0] - unused q[1] - joint 1 q[2] - joint 2 ...  q[NDOF] - joint NDOF
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
	/// \brief Forces acting on the body (in base coordinates)
	std::vector<SpatialAlgebra::SpatialVector> f_ext;

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

	////////////////////////////////////
	// Contact Data
	
	/// \brief Contains for each body all the contact constraint information
	typedef std::map<unsigned int, std::vector<ContactInfo> > ContactMap;
	typedef std::map<unsigned int, std::vector<ContactInfo> >::iterator ContactMapIter;
	ContactMap mContactInfoMap;

	/// \brief Initializes the helper values for the dynamics algorithm
	void Init ();
	/** \brief Connects a given body to the model
	 *
	 * \param parent_id   id of the parent body
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
			const Vector3d &contact_point,
			const Vector3d &contact_normal
			) {
		assert (mBodies.size() > body_id);
		ContactMapIter contact_iter = mContactInfoMap.find (body_id);
		if (contact_iter == mContactInfoMap.end()) {
			mContactInfoMap[body_id] = std::vector<ContactInfo> ();
			contact_iter = mContactInfoMap.find(body_id);
		}
	
		ContactInfo contact_info (body_id, contact_point, contact_normal);

		contact_iter->second.push_back (contact_info);
	}

	/** \brief Returns the 3D coordinate vector of the origin of a given body
	 *  \brief in base coordinates
	 *
	 *  \param body_id id of the body of intrest
	 *  
	 *  \returns 3D coordinate vector of the origin of the body in base
	 *  \returns coordinates
	 */
	Vector3d GetBodyOrigin (const unsigned int body_id);
	/** \brief Returns the orientation of a given body as 3x3 matrix
	 *
	 *  \param body_id id of the body of intrest
	 *
	 *  \returns A 3x3 matrix that contains the rotation from base
	 *  \returns orientation to body orientation
	 */
	Matrix3d GetBodyWorldOrientation (const unsigned int body_id);
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

#endif /* _MODEL_H */
