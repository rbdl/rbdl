/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _DYNAMICS_EXPERIMENTAL_H
#define _DYNAMICS_EXPERIMENTAL_H

#include <mathwrapper.h>
#include <assert.h>
#include <iostream>
#include "Logging.h"

#include "Model.h"

namespace RigidBodyDynamics {

/** \brief Namespace for experimental code that is not thoroughly tested
 */
namespace Experimental {

/** \brief Computes forward dynamics for models with a floating base
 *
 * This method uses explicit information about the state of the floating
 * base body, i.e., base transformation and velocities are specified as
 * parameters to this function. This function will be called by
 * ForwardDynamics() when the flag Model::floating_base was set to true
 * (e.g. as done when calling Model::SetFloatingBaseBody()).
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
void ForwardDynamicsFloatingBaseExpl (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &Tau,
		const SpatialAlgebra::SpatialMatrix &X_B,
		const SpatialAlgebra::SpatialVector &v_B,
		const SpatialAlgebra::SpatialVector &f_B,
		SpatialAlgebra::SpatialVector &a_B,
		VectorNd &QDDot
		);

}

}

#endif /* _DYNAMICS_EXPERIMENTAL_H */
