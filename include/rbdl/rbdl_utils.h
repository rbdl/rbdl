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

	RBDL_DLLAPI double CalcKineticEnergy (Model &model, const Math::VectorNd &q, const Math::VectorNd &qdot, bool update_kinematics = true);

	RBDL_DLLAPI double CalcPotentialEnergy (Model &model, const Math::VectorNd &q, bool update_kinematics = true);
}
}

/* _RBDL_UTILS_H */
#endif
