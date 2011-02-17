#ifndef _KINEMATICS_STDVEC_H
#define _KINEMATICS_STDVEC_H

#include <cmlwrapper.h>
#include <vector>
#include <assert.h>
#include <iostream>
#include "Logging.h"

namespace RigidBodyDynamics {

/** \brief [Deprecated] Computes the velocity of a point on a body 
 *
 * \deprecated Use alternative function that uses cmlVectors as arguments.
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

/** \brief [Deprecated] Computes the acceleration of a point on a body 
 *
 * \deprecated Use alternative function that uses cmlVectors as arguments.
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
}

#endif /* _KINEMATICS_STDVEC_H */
