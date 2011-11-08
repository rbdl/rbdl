/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <mathwrapper.h>
#include <assert.h>
#include <iostream>
#include "Logging.h"

namespace RigidBodyDynamics {

/** \brief Updates and computes velocities and accelerations of the bodies
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

/** \brief Selectively updates and computes velocities and accelerations of the bodies
 *
 * This function updates the kinematic variables such as body velocities and
 * accelerations in the model to reflect the variables passed to this function.
 *
 * \param model the model
 * \param Q     the positional variables of the model
 * \param QDot  the generalized velocities of the joints
 * \param QDDot the generalized accelerations of the joints
 */
void ForwardKinematicsCustom (Model &model,
		const VectorNd *Q,
		const VectorNd *QDot,
		const VectorNd *QDDot
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
 * \param G       a matrix where the result will be stored in
 * \param update_kinematics whether ForwardKinematics() should be called or not (default: true)
 *
 * \returns A 3 x \#dof_count matrix of the point jacobian
 */
void CalcPointJacobian (Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_position,
		MatrixNd &G,
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

/** \brief Computes the inverse kinematics iteratively using a damped Levenberg-Marquardt method
 *
 * \param model rigid body model
 * \param Qinit initial guess for the state
 * \param body_id a vector of all bodies for which we we have kinematic target positions
 * \param body_point a vector of points in body local coordinates that are
 * to be matched to target positions
 * \param target_pos a vector of target positions
 * \param Qres output of the computed inverse kinematics
 * \param step_tol tolerance used for convergence detection
 * \param lambda damping factor for the least squares function
 * \param max_iter maximum number of steps that should be performed
 * \returns true on success, false otherwise
 *
 * This function repeatedly computes
 *   \f[ Qres = Qres + \Delta \theta\f]
 *   \f[ \Delta \theta = G^T (G^T G + \lambda^2 I)^{-1} e \f]
 * where \f$G = G(q) = \frac{d}{dt} g(q(t))\f$ and \f$e\f$ is the
 * correction of the body points so that they coincide with the target
 * positions. The function returns true when \f$||\Delta \theta||_2 \le\f$
 * step_tol or if the error between body points and target gets smaller
 * than step_tol. Otherwise it returns false.
 *
 * The parameter \f$\lambda\f$ is the damping factor that has to
 * be chosen carefully. In case of unreachable positions higher values (e.g
 * 0.9) can be helpful. Otherwise values of 0.0001, 0.001, 0.01, 0.1 might
 * yield good results. See the literature for best practices.
 *
 * \warning The actual accuracy might be rather low (~1.0e-2)! Use this function with a
 * grain of suspicion.
 */
bool InverseKinematics (
		Model &model,
		const VectorNd &Qinit,
		const std::vector<unsigned int>& body_id,
		const std::vector<Vector3d>& body_point,
		const std::vector<Vector3d>& target_pos,
		VectorNd &Qres,
		double step_tol = 1.0e-12,
		double lambda = 0.01,
		unsigned int max_iter = 50
		);

}

#endif /* _KINEMATICS_H */
