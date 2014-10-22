PROJECT (RBDLEXAMPLE CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

# Search for the RBDL include directory and library
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)
FIND_PACKAGE (Eigen3 REQUIRED)
FIND_PACKAGE (URDF REQUIRED)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${URDF_INCLUDE_DIRS})

# Create an executable
ADD_EXECUTABLE (example_urdfreader example_urdfreader.cc)

# And link the library against the executable
TARGET_LINK_LIBRARIES ( example_urdfreader
	${RBDL_LIBRARY}
	${RBDL_URDFReader_LIBRARY}
	${URDF_LIBRARIES}
	)
