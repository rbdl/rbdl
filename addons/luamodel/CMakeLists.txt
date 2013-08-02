PROJECT (RBDL_ADDON_LUAMODEL)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

INCLUDE(GNUInstallDirs)

SET (PROJECT_VERSION 2.0.0)

INCLUDE_DIRECTORIES ( 
	${CMAKE_CURRENT_BINARY_DIR}/include/rbdl
	${CMAKE_CURRENT_SOURCE_DIR}/luatables
#	${CMAKE_CURRENT_SOURCE_DIR}/lua-5.2.1/src
)

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
		LINKER_LANGUAGE CXX
	)

# Perform the proper linking
#SET (CMAKE_SKIP_BUILD_RPATH FALSE)
#SET (CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
#SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#SET (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Look for Lua
FIND_PACKAGE (PkgConfig)
PKG_CHECK_MODULES (LUA REQUIRED lua5.2)
INCLUDE_DIRECTORIES (SYSTEM ${LUA_INCLUDE_DIRS})
LINK_DIRECTORIES (${LUA_LIBARY_DIRS})


SET ( LUA_SOURCES
	lua-5.2.1/src/lapi.c
	lua-5.2.1/src/lauxlib.c
	lua-5.2.1/src/lbaselib.c
	lua-5.2.1/src/lbitlib.c
	lua-5.2.1/src/lcode.c
	lua-5.2.1/src/lcorolib.c
	lua-5.2.1/src/lctype.c
	lua-5.2.1/src/ldblib.c
	lua-5.2.1/src/ldebug.c
	lua-5.2.1/src/ldo.c
	lua-5.2.1/src/ldump.c
	lua-5.2.1/src/lfunc.c
	lua-5.2.1/src/lgc.c
	lua-5.2.1/src/linit.c
	lua-5.2.1/src/liolib.c
	lua-5.2.1/src/llex.c
	lua-5.2.1/src/lmathlib.c
	lua-5.2.1/src/lmem.c
	lua-5.2.1/src/loadlib.c
	lua-5.2.1/src/lobject.c
	lua-5.2.1/src/lopcodes.c
	lua-5.2.1/src/loslib.c
	lua-5.2.1/src/lparser.c
	lua-5.2.1/src/lstate.c
	lua-5.2.1/src/lstring.c
	lua-5.2.1/src/lstrlib.c
	lua-5.2.1/src/ltable.c
	lua-5.2.1/src/ltablib.c
	lua-5.2.1/src/ltm.c
	lua-5.2.1/src/lua.c
	lua-5.2.1/src/lundump.c
	lua-5.2.1/src/lvm.c
	lua-5.2.1/src/lzio.c	
	)

# Options
SET ( LUAMODEL_SOURCES 
	luamodel.cc
	luatables.cc
	)

ADD_DEFINITIONS (-DLUA_COMPAT_ALL)

ADD_LIBRARY ( rbdl_luamodel SHARED ${LUAMODEL_SOURCES} )
SET_TARGET_PROPERTIES ( rbdl_luamodel PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 2.0
  )

IF (BUILD_STATIC)
	ADD_LIBRARY ( rbdl_luamodel-static STATIC ${LUAMODEL_SOURCES} )
	SET_TARGET_PROPERTIES ( rbdl_luamodel-static PROPERTIES PREFIX "lib")
	SET_TARGET_PROPERTIES ( rbdl_luamodel-static PROPERTIES OUTPUT_NAME "rbdl_luamodel")

	TARGET_LINK_LIBRARIES (rbdl_luamodel-static
		rbdl
		lua5.2
	)

	INSTALL (TARGETS rbdl_luamodel-static
	  LIBRARY DESTINATION lib
  	ARCHIVE DESTINATION lib
	)
ENDIF (BUILD_STATIC)

ADD_EXECUTABLE (rbdl_luamodel_util rbdl_luamodel_util.cc)

TARGET_LINK_LIBRARIES (rbdl_luamodel
	rbdl
	lua5.2
	)

TARGET_LINK_LIBRARIES (rbdl_luamodel_util
	rbdl_luamodel
	rbdl
	)

# Installing
INSTALL (TARGETS rbdl_luamodel_util rbdl_luamodel 
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	)

FILE ( GLOB headers 
	"${CMAKE_CURRENT_SOURCE_DIR}/*.h"
	)

INSTALL ( FILES ${headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/rbdl/addons/luamodel )
