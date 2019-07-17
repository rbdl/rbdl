/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2019 Felix Richter <judge@felixrichter.tech>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */
#ifndef RBDL_ERRORS_H
#define RBDL_ERRORS_H

#include <string>
#include <exception>


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

class RBDLInvalidParameterError : public RBDLError {
	public:
		RBDLInvalidParameterError(std::string text);
};


class RBDLSizeMismatchError : public RBDLError {
	public:
		RBDLSizeMismatchError(std::string text);
};

class RBDLDofMismatchError : public RBDLError {
	public:
		RBDLDofMismatchError(std::string text);
};


class RBDLMissingImplementationError: public RBDLError {
	public:
		RBDLMissingImplementationError(std::string text);
};

class RBDLInvalidFileError : public RBDLError {
	public:
		RBDLInvalidFileError(std::string text);
};

class RBDLFileParseError : public RBDLError {
	public:
		RBDLFileParseError(std::string text);
};

}
}

#endif

