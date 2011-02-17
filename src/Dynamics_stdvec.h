#ifndef _DYNAMICS_STDVEC_H
#define _DYNAMICS_STDVEC_H

#include <cmlwrapper.h>
#include <vector>
#include <assert.h>
#include <iostream>
#include "Logging.h"

#include "Model.h"

namespace RigidBodyDynamics {

/** \brief [Deprecated] Computes forward dynamics for models with a fixed base
 *
 * \deprecated Use alternative function that uses cmlVectors as arguments.
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internals joints (output)
 */
void ForwardDynamics (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &Tau,
		std::vector<double> &QDDot
		);

/** \brief [Deprecated] Computes forward dynamics for models with a floating base
 *
 * \deprecated Use alternative function that uses cmlVectors as arguments.
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
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &Tau,
		const SpatialAlgebra::SpatialMatrix &X_B,
		const SpatialAlgebra::SpatialVector &v_B,
		const SpatialAlgebra::SpatialVector &f_B,
		SpatialAlgebra::SpatialVector &a_B,
		std::vector<double> &QDDot
		);
}

#endif /* _DYNAMICS_STDVEC_H */
