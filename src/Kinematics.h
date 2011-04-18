#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <mathwrapper.h>
#include <assert.h>
#include <iostream>
#include "Logging.h"

namespace RigidBodyDynamics {

/** \brief Updates the and computes velocities and accelerations of the bodies
 *
 * This function updates the kinematic variables such as body velocities
 * and accelerations in the model to reflect the variables passed to this function.
 *
 * \param model the model
 * \param Q     the positional variables of the model
 * \param QDot  the generalized velocities of the joints
 * \param QDDot the generalized accelerations of the joints
 */
void ForwardKinematics (Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot
		);

/** \brief Computes the point jacobian for a point on a body
 *
 * If a position of a point is computed by a function \f$g(q(t))\f$ for which its
 * time derivative is \f$\frac{d}{dt} g(q(t)) = G(q)\dot{q}\f$ then this
 * function computes the jacobian matrix \f$G(q)\f$.
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param update_kinematics whether ForwardKinematics() should be called or not (default: true)
 *
 * \returns A 3 x \#dof_count matrix of the point jacobian
 */
MatrixNd CalcPointJacobian (Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_position,
		bool update_kinematics = true
		);

/** \brief Computes the velocity of a point on a body 
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param update_kinematics whether ForwardKinematics() should be called or not (default: true)
 *
 * \returns The cartesian velocity of the point in global frame (output)
 */
Vector3d CalcPointVelocity (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		unsigned int body_id,
		const Vector3d &point_position,
		bool update_kinematics = true
		);

/** \brief Computes the acceleration of a point on a body 
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param QDDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param update_kinematics whether ForwardKinematics() should be called or not (default: true)
 *
 * \returns The cartesian acceleration of the point in global frame (output)
 */
Vector3d CalcPointAcceleration (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		bool update_kinematics = true
	);

}

#endif /* _KINEMATICS_H */
