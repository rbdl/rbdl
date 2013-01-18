/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _RBDL_H
#define _RBDL_H

#include "rbdl/rbdl_math.h"
#include "rbdl/rbdl_mathutils.h"

#include "rbdl/Body.h"
#include "rbdl/Model.h"

#include "rbdl/Dynamics.h"
#include "rbdl/Joint.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Logging.h"

#include "rbdl/Contacts.h"

namespace RigidBodyDynamics {
	/** Prints version information to standard output */
	void rbdl_print_version();
}

#endif /* _RBDL_H */
