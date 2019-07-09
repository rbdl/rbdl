/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Felix Richter <judge@felixrichter.tech>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <string>
#include <exception>

#ifndef RBDL_ERRORS_H
#define RBDL_ERRORS_H

namespace RigidBodyDynamics {

/** \brief Namespace that contains error classes thrown by RBDL at runtime */
namespace Errors {

/** \brief Base class for all RBDL exceptions */
class RBDLError : public std::exception {
	protected: 
		std::string text;
	public:
		RBDLError(std::string text);
		virtual const char* what() const noexcept;
};

class RBDLFileParseError : public RBDLError {
	public:
		RBDLFileParseError(std::string text);
};
}

}

#endif

