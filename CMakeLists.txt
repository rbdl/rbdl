PROJECT (RBDL)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

INCLUDE_DIRECTORIES ( 
	${CMAKE_CURRENT_BINARY_DIR}/
	src/
	include/rbdl
)

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
		LINKER_LANGUAGE CXX
	)

# Perform the proper linking
SET (CMAKE_SKIP_BUILD_RPATH FALSE)
SET (CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
SET (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Options
OPTION (BUILD_STATIC "Build the statically linked library" OFF)
OPTION (BUILD_GUI "Build the (rudimentary) graphical user interface" OFF)
OPTION (BUILD_TESTS "Build the test executables" OFF)
OPTION (RBDL_ENABLE_LOGGING "Enable logging (warning: major impact on performance!)" OFF)
OPTION (RBDL_USE_SIMPLE_MATH "Use slow math instead of the fast Eigen3 library (faster compilation)" OFF)

SET ( RBDL_SOURCES 
	src/mathutils.cc
	src/Logging.cc
	src/Body.cc
	src/Joint.cc
	src/Model.cc
	src/Kinematics.cc
	src/Dynamics.cc
	)

ADD_LIBRARY ( rbdl SHARED ${RBDL_SOURCES} )

IF (BUILD_STATIC)
  ADD_LIBRARY ( rbdl-static STATIC ${RBDL_SOURCES} )
  SET_TARGET_PROPERTIES ( rbdl-static PROPERTIES PREFIX "lib")
  SET_TARGET_PROPERTIES ( rbdl-static PROPERTIES OUTPUT_NAME "rbdl")

	INSTALL (TARGETS rbdl-static
	  LIBRARY DESTINATION lib
  	ARCHIVE DESTINATION lib
	)
ENDIF (BUILD_STATIC)

IF (BUILD_TESTS)
  ADD_SUBDIRECTORY ( tests )
ENDIF (BUILD_TESTS)

IF (BUILD_GUI)
  ADD_SUBDIRECTORY ( gui )
ENDIF (BUILD_GUI)

# Installing
INSTALL (TARGETS rbdl
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)

CONFIGURE_FILE (
	"${CMAKE_CURRENT_SOURCE_DIR}/src/rbdlconfig.h.cmake" 
	"${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdlconfig.h"
	)

FILE ( GLOB headers 
	"${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
	"${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdlconfig.h"
	)

INSTALL ( FILES ${headers} DESTINATION include/rbdl )
