#ifndef _DYNAMICS_H
#define _DYNAMICS_H

#include <cmlwrapper.h>
#include <vector>
#include <assert.h>
#include <iostream>
#include "Logging.h"

#include "Model.h"

namespace RigidBodyDynamics {

/** \brief Computes forward dynamics for a given model
 *
 * When the function Model::SetFloatingBaseBody() was called on a model,
 * this function uses the function ForwardDynamicsFloatingBaseExpl() to
 * compute the accelerations. In this case the first 6 entries of Q, QDot,
 * Tau, and QDDot are first the translations and then the rotations of the
 * floating base. The 6 DoF are ordered in the following way:
 *   0: x translation
 *   1: y translation
 *   2: z translation
 *   3: z rotation
 *   4: y rotation
 *   5: x rotation
 *
 * \param model rigid body model
 * \param Q     state vector of the internal joints
 * \param QDot  velocity vector of the internal joints
 * \param Tau   actuations of the internal joints
 * \param QDDot accelerations of the internals joints (output)
 */
void ForwardDynamics (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		cmlVector &QDDot
		);

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
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		const SpatialAlgebra::SpatialMatrix &X_B,
		const SpatialAlgebra::SpatialVector &v_B,
		const SpatialAlgebra::SpatialVector &f_B,
		SpatialAlgebra::SpatialVector &a_B,
		cmlVector &QDDot
		);

struct ContactInfo;

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
 */
void ComputeContactForces (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		const std::vector<ContactInfo> &ContactData,
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
 */
void ForwardDynamicsContacts (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &Tau,
		const std::vector<ContactInfo> &ContactData,
		cmlVector &QDDot
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
		const cmlVector &Q,
		const cmlVector &QDotPre,
		const std::vector<ContactInfo> &ContactData,
		cmlVector &QDotPost
		);
}

#endif /* _DYNAMICS_H */
