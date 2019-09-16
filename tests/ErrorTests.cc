#include <UnitTest++.h>

#include "rbdl/Logging.h"
#include "rbdl/rbdl_errors.h"
#include <iostream>
#include <exception>

using namespace RigidBodyDynamics::Errors;

// Test if each error type can be catched by Base Class
TEST ( BaseClassCatch )
{
  //RBDLError
  try {
    throw RBDLError("Test RBDLError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLInvalidParameterError
  try {
    throw RBDLInvalidParameterError("Test RBDLInvalidParameterError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLSizeMismatchError
  try {
    throw RBDLSizeMismatchError("Test RBDLSizeMismatchError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLDofMismatchError
  try {
    throw RBDLDofMismatchError("Test RBDLDofMismatchError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLMissingImplementationError
  try {
    throw RBDLMissingImplementationError("Test RBDLMissingImplementationError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLInvalidFileError
  try {
    throw RBDLInvalidFileError("Test RBDLInvalidFileError");
  } catch (RBDLError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }

  //RBDLFileParseError
  try {
    throw RBDLFileParseError("Test RBDLFileParseError");
  } catch (RBDLFileParseError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

// Tests to check catching each error type

TEST ( RBDLInvalidParameterErrorTest )
{
  try {
    throw RBDLInvalidParameterError("Test RBDLInvalidParameterError");
  } catch (RBDLInvalidParameterError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST ( RBDLSizeMismatchErrorTest )
{
  try {
    throw RBDLSizeMismatchError("Test RBDLSizeMismatchError");
  } catch (RBDLSizeMismatchError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST ( RBDLDofMismatchErrorTest )
{
  try {
    throw RBDLDofMismatchError("Test RBDLDofMismatchError");
  } catch (RBDLDofMismatchError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST ( RBDLMissingImplementationErrorTest )
{
  try {
    throw RBDLMissingImplementationError("Test RBDLMissingImplementationError");
  } catch (RBDLMissingImplementationError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST ( RBDLInvalidFileErrorTest )
{
  try {
    throw RBDLInvalidFileError("Test RBDLInvalidFileError");
  } catch (RBDLInvalidFileError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}

TEST ( RBDLFileParseErrorTest )
{
  try {
    throw RBDLFileParseError("Test RBDLFileParseError");
  } catch (RBDLFileParseError &err) {
    CHECK(true);
  } catch (std::exception &e) {
    CHECK(false);
  }
}
