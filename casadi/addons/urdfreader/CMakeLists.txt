CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
	LINKER_LANGUAGE CXX
)


IF (RBDL_USE_ROS_URDF_LIBRARY)
	SET (URDFREADER_CASADI_DEPENDENCIES
		${urdf_LIBRARIES}
		)
ELSE()
	SET (URDFREADER_CASADI_DEPENDENCIES
    urdfparser
		)
ENDIF()

SET(CMAKE_CXX_STANDARD 17)
SET ( URDFREADER_CASADI_SOURCES
	${CMAKE_SOURCE_DIR}/addons/urdfreader/urdfreader.cc
)

IF (RBDL_BUILD_STATIC)
	ADD_LIBRARY ( rbdl_urdfreader-casadi-static STATIC ${URDFREADER_CASADI_SOURCES} )
  TARGET_INCLUDE_DIRECTORIES( rbdl_urdfreader-casadi-static PUBLIC
    ${RBDL_CASADI_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/addons/urdfreader/thirdparty/urdfparser/include
  )

	IF (NOT WIN32)
		SET_TARGET_PROPERTIES ( rbdl_urdfreader-casadi-static PROPERTIES PREFIX "lib")
	ENDIF (NOT WIN32)
	SET_TARGET_PROPERTIES ( rbdl_urdfreader-casadi-static PROPERTIES OUTPUT_NAME "rbdl_urdfreader-casadi")

	TARGET_LINK_LIBRARIES (rbdl_urdfreader-casadi-static
		rbdl-casadi-static
		${URDFREADER_CASADI_DEPENDENCIES}
		)

	INSTALL (TARGETS rbdl_urdfreader-casadi-static
		  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	)
ELSE (RBDL_BUILD_STATIC)
	ADD_LIBRARY ( rbdl_urdfreader-casadi SHARED ${URDFREADER_CASADI_SOURCES} )
  TARGET_INCLUDE_DIRECTORIES( rbdl_urdfreader-casadi PUBLIC
    ${RBDL_CASADI_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/addons/urdfreader/thirdparty/urdfparser/include
  )
	SET_TARGET_PROPERTIES ( rbdl_urdfreader-casadi PROPERTIES
		VERSION ${RBDL_VERSION}
		SOVERSION ${RBDL_SO_VERSION}
  )

	TARGET_LINK_LIBRARIES (rbdl_urdfreader-casadi
    rbdl-casadi
		${URDFREADER_CASADI_DEPENDENCIES}
		)

	INSTALL (TARGETS rbdl_urdfreader-casadi
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		)
ENDIF (RBDL_BUILD_STATIC)


FILE ( GLOB urdf_casadi_headers
	${CMAKE_SOURCE_DIR}/addons/urdfreader/*.h
	)

INSTALL ( FILES ${urdf_casadi_headers}
	DESTINATION
	${CMAKE_INSTALL_INCLUDEDIR}/rbdl-casadi/rbdl/addons/urdfreader
	)
