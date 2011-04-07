#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <cmlwrapper.h>
#include <vector>
#include <assert.h>
#include <iostream>
#include "Logging.h"

namespace RigidBodyDynamics {

/** \brief Computes the velocity of a point on a body 
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 *
 * \returns The cartesian velocity of the point in global frame (output)
 */
Vector3d CalcPointVelocity (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		unsigned int body_id,
		const Vector3d &point_position
		);

/** \brief Computes the acceleration of a point on a body 
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param QDDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data

 * \returns The cartesian acceleration of the point in global frame (output)
 */
Vector3d CalcPointAcceleration (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &QDDot,
		unsigned int body_id,
		const Vector3d &point_position
		);

}

#endif /* _KINEMATICS_H */
