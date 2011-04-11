#ifndef _CONTACTS_H
#define _CONTACTS_H

#include "cmlwrapper.h"

namespace RigidBodyDynamics {

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

/** \brief Computes forces acting on the model due to contact
 *
 * The method used here is the one described by Kokkevis and Metaxas in the
 * Paper "Efficient Dynamic Constraints for Animating Articulated Figures",
 * published in Multibody System Dynamics Vol.2, 1998.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param ContactData	a list of all contact points and their desired accelerations
 * \param Fext  constraint forces that enforce desired acceleration on the constraints
 *
 * \note During execution of this function the values ContactData[i].force
 * 	get modified and will contain the value of the force acting along
 * 	the normal.
 */
void ComputeContactForces (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		std::vector<ContactInfo> &ContactData,
		const std::vector<SpatialAlgebra::SpatialVector> &Fext
		);

/** \brief Computes forward dynamics that accounts for active contacts in mContactInfoMap
 *
 * The method used here is the one described by Kokkevis and Metaxas in the
 * Paper "Efficient Dynamic Constraints for Animating Articulated Figures",
 * published in Multibody System Dynamics Vol.2, 1998.
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
void ForwardDynamicsContacts (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		std::vector<ContactInfo> &ContactData,
		VectorNd &QDDot
		);

/** \brief Computes the change of the generalized velocity due to collisions
 *
 * The method used here is the one described by Kokkevis and Metaxas in the
 * Paper "Efficient Dynamic Constraints for Animating Articulated Figures",
 * published in Multibody System Dynamics Vol.2, 1998.
 *
 * This function computes the change of the generalized velocity vector
 * QDot such that the points defined in ContactData have zero velocity.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDotPre  generalized velocity before the collision
 * \param ContactData	a list of all contact points
 * \param QDotPost generalized velocity after the collision
 */
void ComputeContactImpulses (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDotPre,
		const std::vector<ContactInfo> &ContactData,
		VectorNd &QDotPost
		);


}

#endif /* _CONTACTS_H */
