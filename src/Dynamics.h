/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

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
 * It does this by using the recursive Articulated Body Algorithm that runs
 * in \f$O(n_{dof})\f$ with \f$n_{dof}\f$ being the number of joints.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internal joints (output)
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 *
 * \todo [high] Create a test for this function.
 */
void ForwardDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot,
		std::vector<SpatialAlgebra::SpatialVector> *f_ext = NULL
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
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 */
void ForwardDynamicsLagrangian (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		VectorNd &QDDot,
		std::vector<SpatialAlgebra::SpatialVector> *f_ext = NULL
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
 * \param f_ext External forces acting on the body in base coordinates (optional, defaults to NULL)
 */
void InverseDynamics (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot,
		VectorNd &Tau,
		std::vector<SpatialAlgebra::SpatialVector> *f_ext = NULL
		);

/** \brief Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
 *
 * This function computes the joint space inertia matrix from a given model and
 * the generalized state vector:
 *   \f$ M(q) \f$
 *
 * \warning This function does not update joint axis and body transformations,
 * \warning hence one has to call ForwardKinematics() first!
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
