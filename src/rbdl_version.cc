/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <rbdl_config.h>

#include <iostream>
#include <string>

namespace RigidBodyDynamics {

void rbdl_print_version() {
	std::cout << "RigidBodyDynamicsLibrary version:" << std::endl
		<< "  revision     : " << RBDL_BUILD_REVISION
		<< " (branch: " << RBDL_BUILD_BRANCH << ")" << std::endl
		<< "  build type   : " << RBDL_BUILD_TYPE << std::endl
#ifdef RBDL_ENABLE_LOGGING
		<< "  logging      : on (warning: reduces performance!)" << std::endl
#else
		<< "  logging      : off" << std::endl
#endif
#ifdef RBDL_USE_SIMPLE_MATH
		<< "  simplemath   : on (warning: reduces performance!)" << std::endl
#else
		<< "  simplemath   : off" << std::endl
#endif
#ifdef BUILD_ADDON_LUAMODEL
		<< "  luamodel     : on" << std::endl;
#else
	  << "  luamodel     : off" << std::endl;
#endif
		;

	std::string build_revision (RBDL_BUILD_REVISION);
	if (build_revision == "unknown") {
		std::cout << std::endl << "Version information incomplete: to enable version information re-build" << std::endl << "library from valid repository and enable RBDL_STORE_VERSION." << std::endl;
	}
}

}
