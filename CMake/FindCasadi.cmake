# - Find Casadi
# Find the native Casadi includes and libraries
#
#  Casadi_INCLUDE_DIR - where to find casadi.hpp, etc.
#  Casadi_LIBRARIES   - List of libraries when using Casadi.
#  Casadi_FOUND       - True if Casadi is found.

if (Casadi_INCLUDE_DIR)
  # Already in cache, be silent
  set (Casadi_FIND_QUIETLY TRUE)
endif (Casadi_INCLUDE_DIR)

find_package (Casadi CONFIG QUIET)
if (NOT Casadi_FOUND)
  find_package (casadi CONFIG QUIET)
  if (casadi_FOUND)
    set (Casadi_FOUND TRUE)
  endif()
endif()

foreach (_casadi_target Casadi::casadi casadi::casadi casadi)
  if (TARGET ${_casadi_target})
    set (Casadi_LIBRARY ${_casadi_target})
    if (NOT Casadi_INCLUDE_DIR)
      get_target_property (Casadi_INCLUDE_DIR ${_casadi_target} INTERFACE_INCLUDE_DIRECTORIES)
    endif()
  endif()
endforeach()

if (NOT Casadi_INCLUDE_DIR AND Casadi_INCLUDE_DIRS)
  list (GET Casadi_INCLUDE_DIRS 0 Casadi_INCLUDE_DIR)
endif()
if (NOT Casadi_INCLUDE_DIR AND casadi_INCLUDE_DIRS)
  list (GET casadi_INCLUDE_DIRS 0 Casadi_INCLUDE_DIR)
endif()
if (NOT Casadi_LIBRARY AND Casadi_LIBRARIES)
  list (GET Casadi_LIBRARIES 0 Casadi_LIBRARY)
endif()
if (NOT Casadi_LIBRARY AND casadi_LIBRARIES)
  list (GET casadi_LIBRARIES 0 Casadi_LIBRARY)
endif()

find_path (Casadi_INCLUDE_DIR "casadi.hpp"
    PATHS
    ${CMAKE_INSTALL_PREFIX}/include/casadi
    ${CMAKE_INSTALL_PREFIX}/Library/include/casadi
    /usr/local/include/casadi
    /usr/include/casadi
)
find_library (Casadi_LIBRARY NAMES casadi 
    PATHS ${CMAKE_INSTALL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/Library/lib
    /usr/lib
)

# handle the QUIETLY and REQUIRED arguments and set Casadi_FOUND to TRUE if
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (Casadi DEFAULT_MSG 
    Casadi_LIBRARY
    Casadi_INCLUDE_DIR
)


