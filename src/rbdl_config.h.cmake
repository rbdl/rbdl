/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _RBDLCONFIG_H
#define _RBDLCONFIG_H

#cmakedefine RBDL_USE_SIMPLE_MATH
#cmakedefine RBDL_ENABLE_LOGGING
#cmakedefine RBDL_BUILD_REVISION "@RBDL_BUILD_REVISION@"
#cmakedefine RBDL_BUILD_TYPE "@RBDL_BUILD_TYPE@"
#cmakedefine RBDL_BUILD_BRANCH "@RBDL_BUILD_BRANCH@"
#cmakedefine BUILD_ADDON_LUAMODEL

/* compatibility defines */
#ifdef _WIN32
	#define __func__ __FUNCTION__
#endif

#endif
