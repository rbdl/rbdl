# Searches for RBDL includes and library files, including Addons.
#
# Sets the variables
#   RBDL_FOUND
#   RBDL_INCLUDE_DIR
#   RBDL_LIBRARY
#
# You can use the following components:
#   LuaModel
#   URDFReader
# and then link to them e.g. using RBDL_LuaModel_LIBRARY.

SET (RBDL_FOUND FALSE)
SET (RBDL_LuaModel_FOUND FALSE)
SET (RBDL_URDFReader_FOUND FALSE)

FIND_PATH (RBDL_INCLUDE_DIR rbdl/rbdl.h
	/usr/include
	/usr/local/include
	$ENV{HOME}/local/include
	$ENV{RBDL_PATH}/src
	$ENV{RBDL_PATH}/include
	$ENV{RBDL_INCLUDE_PATH}
	)

FIND_LIBRARY (RBDL_LIBRARY NAMES rbdl	PATHS
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{RBDL_PATH}
	$ENV{RBDL_LIBRARY_PATH}
	)

FIND_PATH (RBDL_LuaModel_INCLUDE_DIR rbdl/addons/luamodel/luamodel.h
	/usr/include
	/usr/local/include
	$ENV{HOME}/local/include
	$ENV{RBDL_PATH}/src
	$ENV{RBDL_PATH}/include
	$ENV{RBDL_INCLUDE_PATH}
	)

FIND_LIBRARY (RBDL_LuaModel_LIBRARY NAMES rbdl_luamodel	PATHS
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{RBDL_PATH}
	$ENV{RBDL_LIBRARY_PATH}
	)

FIND_PATH (RBDL_URDFReader_INCLUDE_DIR rbdl/addons/urdfreader/urdfreader.h
	/usr/include
	/usr/local/include
	$ENV{HOME}/local/include
	$ENV{RBDL_PATH}/src
	$ENV{RBDL_PATH}/include
	$ENV{RBDL_INCLUDE_PATH}
	)

FIND_LIBRARY (RBDL_URDFReader_LIBRARY NAMES rbdl_urdfreader PATHS
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{RBDL_PATH}
	$ENV{RBDL_LIBRARY_PATH}
	)

IF (NOT RBDL_LIBRARY)
	MESSAGE (ERROR "Could not find RBDL")
ENDIF (NOT RBDL_LIBRARY)

IF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)
	SET (RBDL_FOUND TRUE)
ENDIF (RBDL_INCLUDE_DIR AND RBDL_LIBRARY)

IF (RBDL_LuaModel_INCLUDE_DIR AND RBDL_LuaModel_LIBRARY)
	SET (RBDL_LuaModel_FOUND TRUE)
ENDIF (RBDL_LuaModel_INCLUDE_DIR AND RBDL_LuaModel_LIBRARY)

IF (RBDL_URDFReader_INCLUDE_DIR AND RBDL_URDFReader_LIBRARY)
	SET (RBDL_URDFReader_FOUND TRUE)
ENDIF (RBDL_URDFReader_INCLUDE_DIR AND RBDL_URDFReader_LIBRARY)

IF (RBDL_FOUND)
   IF (NOT RBDL_FIND_QUIETLY)
      MESSAGE(STATUS "Found RBDL: ${RBDL_LIBRARY}")
   ENDIF (NOT RBDL_FIND_QUIETLY)

	 foreach ( COMPONENT ${RBDL_FIND_COMPONENTS} )
		 IF (RBDL_${COMPONENT}_FOUND)
			 IF (NOT RBDL_FIND_QUIETLY)
				 MESSAGE(STATUS "Found RBDL ${COMPONENT}: ${RBDL_${COMPONENT}_LIBRARY}")
			 ENDIF (NOT RBDL_FIND_QUIETLY)
		 ELSE (RBDL_${COMPONENT}_FOUND)
			 MESSAGE(SEND_ERROR "Could not find RBDL ${COMPONENT}")
		 ENDIF (RBDL_${COMPONENT}_FOUND)
	 endforeach ( COMPONENT )
ELSE (RBDL_FOUND)
   IF (RBDL_FIND_REQUIRED)
		 MESSAGE(SEND_ERROR "Could not find RBDL")
   ENDIF (RBDL_FIND_REQUIRED)
ENDIF (RBDL_FOUND)

MARK_AS_ADVANCED (
	RBDL_INCLUDE_DIR
	RBDL_LIBRARY
	RBDL_LuaModel_INCLUDE_DIR
	RBDL_LuaModel_LIBRARY
	RBDL_URDFReader_INCLUDE_DIR
	RBDL_URDFReader_LIBRARY
	)
