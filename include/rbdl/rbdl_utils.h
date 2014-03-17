#ifndef _RBDL_UTILS_H
#define _RBDL_UTILS_H

#include <string>
#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl_math.h>

namespace RigidBodyDynamics {

struct Model;

/** \brief Namespace that contains optional helper functions */
namespace Utils {
	/** \brief Creates a human readable overview of the model. */
	RBDL_DLLAPI std::string GetModelHierarchy (const Model &model);
	/** \brief Creates a human readable overview of the Degrees of Freedom. */
	RBDL_DLLAPI std::string GetModelDOFOverview (const Model &model);
	/** \brief Creates a human readable overview of the locations of all bodies that have names. */
	RBDL_DLLAPI std::string GetNamedBodyOriginsOverview (Model &model);

	/** \brief Computes the Center of Mass (COM) and optionally its linear velocity.
	 *
	 * When only interested in computing the location of the COM you can use
	 * NULL as value for com_velocity.
	 *
	 * \param model The model for which we want to compute the COM
	 * \param q The current joint positions
	 * \param qdot The current joint velocities
	 * \param mass (output) total mass of the model
	 * \param com (output) location of the Center of Mass of the model in base coordinates
	 * \param com_velocity (optional output) linear velocity of the COM in base coordinates 
	 * \param update_kinematics (optional input) whether the kinematics should be updated (defaults to true)
	 */
	RBDL_DLLAPI void CalcCenterOfMass (Model &model, const Math::VectorNd &q, const Math::VectorNd &qdot, double &mass, Math::Vector3d &com, Math::Vector3d *com_velocity = NULL, bool update_kinematics = true);

	/** \brief Computes the potential energy of the full model. */
	RBDL_DLLAPI double CalcPotentialEnergy (Model &model, const Math::VectorNd &q, bool update_kinematics = true);

	/** \brief Computes the kinetic energy of the full model. */
	RBDL_DLLAPI double CalcKineticEnergy (Model &model, const Math::VectorNd &q, const Math::VectorNd &qdot, bool update_kinematics = true);

	/** \brief Computes the angular momentum of the model about the model's Center of Mass */
	RBDL_DLLAPI Math::Vector3d CalcAngularMomentum (Model &model, const Math::VectorNd &q, const Math::VectorNd &qdot, bool update_kinematics = true);
}

}

/* _RBDL_UTILS_H */
#endif
