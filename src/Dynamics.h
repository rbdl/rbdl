#ifndef _DYNAMICS_H
#define _DYNAMICS_H

#include <mathwrapper.h>
#include <assert.h>
#include <iostream>
#include "Logging.h"

namespace RigidBodyDynamics {

class Model;

/** \brief Computes forward dynamics with the Articulated Body Algorithm
 *
 * This function computes the generalized accelerations from given
 * generalized states, velocities and forces:
 *   \f$ \ddot{q} = M(q)^{-1} ( -N(q, \dot{q}) + \tau)\f$
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internal joints (output)
 */
void ForwardDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot
		);

/** \brief Computes forward dynamics by building and solving the full Lagrangian equation
 *
 * This method builds and solves the linear system
 * \f[ 	H \ddot{q} = -C + \tau	\f]
 * for \f$\ddot{q}\f$ where \f$H\f$ is the joint space inertia matrix
 * computed with the CompositeRigidBodyAlgorithm(), \f$C\f$ the bias
 * force (sometimes called "non-linear effects").
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internal joints (output)
 */
void ForwardDynamicsLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot
		);

/** \brief Computes inverse dynamics with the Newton-Euler Algorithm
 *
 * This function computes the generalized forces from given generalized
 * states, velocities, and accelerations:
 *   \f$ \tau = M(q) \ddot{q} + N(q, \dot{q}) \f$
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param QDDot accelerations of the internals joints
 * \param Tau   actuations of the internal joints (output)
  */
void InverseDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot,
		VectorNd &Tau
		);

/** \brief Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
 *
 * This function computes the joint space inertia matrix from a given model and
 * the generalized state vector:
 *   \f$ M(q) \f$
 *
 * \param model rigid body model
 * \param Q     state vector of the model
 * \param H     a matrix where the result will be stored in
 */
void CompositeRigidBodyAlgorithm (
		Model& model,
		const VectorNd &Q,
		MatrixNd &H
		);

}

#endif /* _DYNAMICS_H */
