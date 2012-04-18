PROJECT (RBDL_ADDON_URDFREADER)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

INCLUDE_DIRECTORIES ( 
	${CMAKE_CURRENT_BINARY_DIR}/include/rbdl
	${CMAKE_CURRENT_SOURCE_DIR}/tinyxml/
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
	tinyxml/tinystr.cpp
	tinyxml/tinyxml.cpp
	tinyxml/tinyxmlerror.cpp
	tinyxml/tinyxmlparser.cpp
	)

ADD_LIBRARY ( rbdl_urdfreader SHARED ${URDFREADER_SOURCES} )

ADD_DEFINITIONS (-DTIXML_USE_STL)

IF (BUILD_STATIC)
  ADD_LIBRARY ( rbdl_urdfreader-static STATIC ${RBDL_SOURCES} )
  SET_TARGET_PROPERTIES ( rbdl_urdfreader-static PROPERTIES PREFIX "lib")
  SET_TARGET_PROPERTIES ( rbdl_urdfreader-static PROPERTIES OUTPUT_NAME "rbdl_urdfreader")

	INSTALL (TARGETS rbdl_urdfreader-static
	  LIBRARY DESTINATION lib
  	ARCHIVE DESTINATION lib
	)
ENDIF (BUILD_STATIC)

ADD_EXECUTABLE (rbdl_urdf_test rbdl_urdf_test.cc)

TARGET_LINK_LIBRARIES (rbdl_urdfreader
	rbdl
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
