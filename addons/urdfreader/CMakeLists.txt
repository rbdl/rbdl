PROJECT (RBDL_ADDON_URDFREADER)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

INCLUDE ( CMake/ros.cmake)
INCLUDE ( CMake/pkg-config.cmake)

ADD_ROSPACK_DEPENDENCY ("urdf")
ADD_ROSPACK_DEPENDENCY ("urdf_interface")
ADD_ROSPACK_DEPENDENCY ("urdf_parser")

# We additionally link against urdf explicitly
FIND_PACKAGE (URDF REQUIRED)
INCLUDE_DIRECTORIES (${URDF_INCLUDE_DIR})

INCLUDE_DIRECTORIES ( 
	${CMAKE_CURRENT_BINARY_DIR}/include/rbdl
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
SET ( URDFREADER_SOURCES 
	rbdl_urdfreader.cc
	)

ADD_LIBRARY ( rbdl_urdfreader SHARED ${URDFREADER_SOURCES} )

ADD_EXECUTABLE (rbdl_urdf_test rbdl_urdf_test.cc)

ROSPACK_USE_DEPENDENCY (rbdl_urdfreader urdf)
ROSPACK_USE_DEPENDENCY (rbdl_urdfreader urdf_interface)

ROSPACK_USE_DEPENDENCY (rbdl_urdf_test urdf)
ROSPACK_USE_DEPENDENCY (rbdl_urdf_test urdf_interface)

IF (BUILD_STATIC)
	ADD_LIBRARY ( rbdl_urdfreader-static STATIC ${URDFREADER_SOURCES} )

	ROSPACK_USE_DEPENDENCY (rbdl_urdfreader-static urdf)
	ROSPACK_USE_DEPENDENCY (rbdl_urdfreader-static urdf_interface)

  SET_TARGET_PROPERTIES ( rbdl_urdfreader-static PROPERTIES PREFIX "lib")
  SET_TARGET_PROPERTIES ( rbdl_urdfreader-static PROPERTIES OUTPUT_NAME "rbdl_urdfreader")

	INSTALL (TARGETS rbdl_urdfreader-static
	  LIBRARY DESTINATION lib
  	ARCHIVE DESTINATION lib
	)
ENDIF (BUILD_STATIC)

TARGET_LINK_LIBRARIES (rbdl_urdfreader
	rbdl
	${URDF_LIBRARY}
	)

TARGET_LINK_LIBRARIES (rbdl_urdf_test
	rbdl_urdfreader
	)

# Installing
INSTALL (TARGETS rbdl_urdfreader
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	)

FILE ( GLOB headers 
	"${CMAKE_CURRENT_SOURCE_DIR}/*.h"
	)

INSTALL ( FILES ${headers} DESTINATION include/rbdl/addons/urdfreader )
