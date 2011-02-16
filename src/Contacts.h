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
		acceleration (0.)
	{	}
	ContactInfo (const ContactInfo &contact_info) :
		body_id (contact_info.body_id),
		point (contact_info.point),
		normal (contact_info.normal),
		acceleration (contact_info.acceleration)
	{}
	ContactInfo& operator= (const ContactInfo &contact_info) {
		if (this != &contact_info) {
			body_id = contact_info.body_id;
			point = contact_info.point;
			normal = contact_info.normal;
			acceleration = contact_info.acceleration;
		}

		return *this;
	}
	~ContactInfo() {};

	ContactInfo (unsigned int body, const Vector3d &contact_point, const Vector3d &contact_normal):
		body_id (body),
		point (contact_point),
		normal (contact_normal),
		acceleration (0.)
	{	}

	ContactInfo (unsigned int body, const Vector3d &contact_point, const Vector3d &contact_normal, const double accel):
		body_id (body),
		point (contact_point),
		normal (contact_normal),
		acceleration (accel)
	{	}

	/// \brief The id of the body of which the motion is constrained
	unsigned int body_id;
	/// \brief Coordinate of the contact point in base coordinates
	Vector3d point;
	/// \brief Normal of the contact point in base coordinates
	Vector3d normal;
	/// \brief Acceleration value of the constraint along the normal
	double acceleration;
};

}

#endif /* _CONTACTS_H */
