/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "Logging.h"

std::ostringstream LogOutput;

void ClearLogOutput() {
	LogOutput.str("");
}
