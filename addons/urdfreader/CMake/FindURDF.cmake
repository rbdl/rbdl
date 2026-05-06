# - Try to find URDF dependencies
#

find_package (PkgConfig QUIET)
if (PKG_CONFIG_FOUND)
  pkg_check_modules (CONSOLE_BRIDGE_PC QUIET console_bridge)
  pkg_check_modules (URDFDOM_HEADERS_PC QUIET urdfdom_headers)
  pkg_check_modules (URDFDOM_PC QUIET urdfdom)
endif()

find_path (CONSOLE_BRIDGE_DIR console_bridge/console.h
  HINTS
  ${CONSOLE_BRIDGE_PC_INCLUDE_DIRS}
  PATHS
  /usr/local/include
  /usr/include
  )

find_library (CONSOLE_BRIDGE_LIBRARY NAMES console_bridge
  HINTS
  ${CONSOLE_BRIDGE_PC_LIBRARY_DIRS}
  PATHS
  /usr/local/lib
  /usr/lib
  )

find_path (URDFDOM_HEADERS_DIR urdf_model/model.h
  HINTS
  ${URDFDOM_HEADERS_PC_INCLUDE_DIRS}
  ${URDFDOM_PC_INCLUDE_DIRS}
  PATHS
  /usr/local/include
  /usr/include
  )

find_path (URDFDOM_DIR urdf_parser/urdf_parser.h
  HINTS
  ${URDFDOM_PC_INCLUDE_DIRS}
  PATHS
  /usr/local/include
  /usr/include
  )

find_library (URDFDOM_MODEL_LIBRARY NAMES urdfdom_model
  HINTS
  ${URDFDOM_PC_LIBRARY_DIRS}
  PATHS
  /usr/local/lib
  /usr/lib
  )

find_library (URDFDOM_WORLD_LIBRARY NAMES urdfdom_world
  HINTS
  ${URDFDOM_PC_LIBRARY_DIRS}
  PATHS
  /usr/local/lib
  /usr/lib
  )

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (URDF DEFAULT_MSG
  CONSOLE_BRIDGE_DIR
  CONSOLE_BRIDGE_LIBRARY
  URDFDOM_HEADERS_DIR
  URDFDOM_DIR
  URDFDOM_MODEL_LIBRARY
  URDFDOM_WORLD_LIBRARY
  )

if (URDF_FOUND)
  set (CONSOLE_BRIDGE_FOUND TRUE)
  set (URDFDOM_HEADERS_FOUND TRUE)
  set (URDFDOM_FOUND TRUE)
  set (URDF_INCLUDE_DIRS
    ${CONSOLE_BRIDGE_DIR}
    ${URDFDOM_HEADERS_DIR}
    ${URDFDOM_DIR}
    )
  set (URDF_LIBRARIES
    ${CONSOLE_BRIDGE_LIBRARY}
    ${URDFDOM_WORLD_LIBRARY}
    ${URDFDOM_MODEL_LIBRARY}
    )
endif()

mark_as_advanced (
  CONSOLE_BRIDGE_DIR
  CONSOLE_BRIDGE_LIBRARY
  URDFDOM_HEADERS_DIR
  URDFDOM_DIR
  URDFDOM_MODEL_LIBRARY
  URDFDOM_WORLD_LIBRARY
  )
