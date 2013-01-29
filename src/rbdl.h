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

/** \defgroup api_version_checking API Version Checking
 * @{
 *
 * This documentation was created for API version 1.0.0.
 */

/** Returns the API version at compile time of the library. */
int rbdl_get_api_version();

/** Ensures whether the RBDL library we are linking against is compatible
 * with the the version we have from rbdl.h.
 *
 * To perform the check run:
 * \code
 *   rbdl_check_api_version(API_VERSION);
 * \endcode
 *
 * This function will abort if compatibility is not met or warn if you run
 * a version that might not be entirely compatible.
 *
 * In most cases you want to specify a specific version to ensure you are
 * using a compatible version. To do so replace API_VERSION by a
 * value of the form 0xAABBCC where AA is the major, BB the minor, and CC
 * the patch version in hex-format, e.g:
 *
 * \code
 *   rbdl_check_api_version(0x020A0C);
 * \endcode
 * 
 * Would abort if the API major version is not 2 (= 0x02), warn if the
 * linked minor version is not 10 (= 0x0A). The patch version 12 (= 0x12)
 * does not have an influence on compatibility.
 */
void rbdl_check_api_version(int version);

/** Prints version information to standard output */
void rbdl_print_version();

/** @} */

#endif /* _RBDL_H */
