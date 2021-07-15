CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

FIND_PACKAGE (Casadi REQUIRED)

IF (RBDL_STORE_VERSION)
	# Set versioning information that can be queried during runtime
	EXECUTE_PROCESS(COMMAND "git" "rev-parse" "HEAD"
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			OUTPUT_VARIABLE RBDL_BUILD_COMMIT)
	STRING(REGEX REPLACE "\n$" "" RBDL_BUILD_COMMIT "${RBDL_BUILD_COMMIT}")
	EXECUTE_PROCESS(COMMAND "git" "describe" "--all" "--dirty" "--long"
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			OUTPUT_VARIABLE RBDL_BUILD_BRANCH)
	STRING(REGEX REPLACE "\n$" "" RBDL_BUILD_BRANCH "${RBDL_BUILD_BRANCH}")
	SET (RBDL_BUILD_TYPE ${CMAKE_BUILD_TYPE})
ELSE (RBDL_STORE_VERSION)
	SET (RBDL_BUILD_COMMIT "unknown")
	SET (RBDL_BUILD_BRANCH "unknown")
	SET (RBDL_BUILD_TYPE "unknown")
ENDIF (RBDL_STORE_VERSION)

CONFIGURE_FILE (
	"${CMAKE_SOURCE_DIR}/include/rbdl/rbdl_casadi_config.h.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdl_config.h"
	)

SET ( RBDL_CASADI_SOURCES
	${CMAKE_SOURCE_DIR}/src/rbdl_version.cc
	${CMAKE_SOURCE_DIR}/src/rbdl_mathutils.cc
	${CMAKE_SOURCE_DIR}/src/rbdl_utils.cc
	${CMAKE_SOURCE_DIR}/src/rbdl_errors.cc
	${CMAKE_SOURCE_DIR}/src/Constraints.cc
  ${CMAKE_SOURCE_DIR}/src/Constraint_Contact.cc
  ${CMAKE_SOURCE_DIR}/src/Constraint_Loop.cc
	${CMAKE_SOURCE_DIR}/src/Dynamics.cc
	${CMAKE_SOURCE_DIR}/src/Logging.cc
	${CMAKE_SOURCE_DIR}/src/Joint.cc
	${CMAKE_SOURCE_DIR}/src/Model.cc
	${CMAKE_SOURCE_DIR}/src/Kinematics.cc
)

IF (RBDL_BUILD_STATIC)
  ADD_LIBRARY ( rbdl-casadi-static STATIC ${RBDL_CASADI_SOURCES} )

  TARGET_INCLUDE_DIRECTORIES (rbdl-casadi-static PUBLIC BEFORE
    ${Casadi_INCLUDE_DIR}
    ${Casadi_INCLUDE_DIR}/..
	  ${CMAKE_SOURCE_DIR}/include
	  ${CMAKE_CURRENT_BINARY_DIR}/include
  )

  IF (NOT MSVC)
    SET_TARGET_PROPERTIES ( rbdl-casadi-static PROPERTIES PREFIX "lib")
  ENDIF (NOT MSVC)
  SET_TARGET_PROPERTIES ( rbdl-casadi-static PROPERTIES OUTPUT_NAME "rbdl-casadi")

  TARGET_LINK_LIBRARIES ( rbdl-casadi-static
      ${Casadi_LIBRARY}
      )

	INSTALL (TARGETS rbdl-casadi-static
	  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
ELSE (RBDL_BUILD_STATIC)
	ADD_LIBRARY ( rbdl-casadi SHARED ${RBDL_CASADI_SOURCES} )
  TARGET_INCLUDE_DIRECTORIES (rbdl-casadi PUBLIC BEFORE
    ${Casadi_INCLUDE_DIR}
    ${Casadi_INCLUDE_DIR}/..
	  ${CMAKE_SOURCE_DIR}/include
	  ${CMAKE_CURRENT_BINARY_DIR}/include
  )

	SET_TARGET_PROPERTIES ( rbdl-casadi PROPERTIES
		VERSION ${RBDL_VERSION}
		SOVERSION ${RBDL_SO_VERSION}
	)

  TARGET_LINK_LIBRARIES ( rbdl-casadi
    ${Casadi_LIBRARY}
  )

	INSTALL (TARGETS rbdl-casadi
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		)
ENDIF (RBDL_BUILD_STATIC)

# Setup of CasADi install settings
FILE ( GLOB headers
	${CMAKE_SOURCE_DIR}/include/rbdl/*.h
	${CMAKE_CURRENT_BINARY_DIR}/include/rbdl/rbdl_config.h
	)

INSTALL ( FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbdl-casadi/rbdl )
INSTALL ( DIRECTORY "${CMAKE_SOURCE_DIR}/include/rbdl/CasadiMath"
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbdl-casadi/rbdl
)

# pkg-config
CONFIGURE_FILE (
  ${CMAKE_SOURCE_DIR}/share/rbdl-casadi.pc.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/rbdl-casadi.pc @ONLY
  )
INSTALL (
  FILES  ${CMAKE_CURRENT_BINARY_DIR}/rbdl-casadi.pc
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
  )

# Prepare share
if(WIN32 AND NOT CYGWIN)
  set(CMAKE_SHARE_DIR cmake/RBDLCasadi)
else()
  set(CMAKE_SHARE_DIR lib/cmake/RBDLCasadi)
endif()
include(CMakePackageConfigHelpers)
configure_package_config_file(
  ${CMAKE_SOURCE_DIR}/share/RBDLCasadiConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/RBDLCasadiConfig.cmake
  INSTALL_DESTINATION ${CMAKE_SHARE_DIR}
)
INSTALL (
  FILES  ${CMAKE_CURRENT_BINARY_DIR}/RBDLCasadiConfig.cmake
  DESTINATION ${CMAKE_SHARE_DIR}
  )
