/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _RBDL_H
#define _RBDL_H

#include "rbdl_math.h"
#include "rbdl_mathutils.h"

#include "Body.h"
#include "Model.h"

#include "Dynamics.h"
#include "Joint.h"
#include "Kinematics.h"
#include "Logging.h"

#include "Contacts.h"

namespace RigidBodyDynamics {
	/** Prints version information to standard output */
	void rbdl_print_version();
}

#endif /* _RBDL_H */
