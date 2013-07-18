#ifndef _RBDL_UTILS_H
#define _RBDL_UTILS_H

#include <string>

namespace RigidBodyDynamics {

struct Model;

/** \brief Namespace that contains optional helper functions */
namespace Utils {

	/** \brief Creates a human readable overview of the model. */
	std::string GetModelHierarchy (const Model &model);
	/** \brief Creates a human readable overview of the Degrees of Freedom. */
	std::string GetModelDOFOverview (const Model &model);
	/** \brief Creates a human readable overview of the locations of all bodies that have names. */
	std::string GetNamedBodyOriginsOverview (Model &model);
}
}

/* _RBDL_UTILS_H */
#endif
