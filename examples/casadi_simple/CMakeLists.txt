PROJECT (RBDLCasadiExample CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )


SET(CUSTOM_RBDL_PATH "" CACHE PATH "Path to specific RBDL Installation")
# Search for the RBDL include directory and library
FIND_PACKAGE (Casadi REQUIRED)
FIND_PACKAGE (RBDLCasadi REQUIRED)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES (
  ${RBDLCasadi_INCLUDE_DIR}
  ${Casadi_INCLUDE_DIR}
  ${Casadi_INCLUDE_DIR}/..
)

# Create an executable
ADD_EXECUTABLE (casadi_example example.cc)

# And link the library against the executable
TARGET_LINK_LIBRARIES (casadi_example
  ${Casadi_LIBRARY}
	${RBDLCasadi_LIBRARY}
	)
