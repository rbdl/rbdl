# - Find Casadi
# Find the native Casadi includes and libraries
#
#  Casadi_INCLUDE_DIR - where to find casadi.hpp, etc.
#  Casadi_LIBRARIES   - List of libraries when using Casadi.
#  Casadi_FOUND       - True if Casadi is found.

set(Casadi_DIR "" CACHE PATH "Path to Casadi installation")

if (Casadi_INCLUDE_DIR)
  # Already in cache, be silent
  set (Casadi_FIND_QUIETLY TRUE)
endif (Casadi_INCLUDE_DIR)

find_path (Casadi_INCLUDE_DIR "casadi.hpp" 
    PATHS
    ${Casadi_DIR}/casadi
    ${Casadi_DIR}/include/casadi
    ${CMAKE_INSTALL_PREFIX}/include/casadi
    ${CMAKE_INSTALL_PREFIX}/Library/include/casadi
    /usr/local/include/casadi
    /usr/include/casadi
)
find_library (Casadi_LIBRARY NAMES casadi 
    PATHS 
    ${Casadi_DIR}
    ${Casadi_DIR}/lib
    ${Casadi_DIR}/Library/lib
    ${CMAKE_INSTALL_PREFIX}/lib 
    ${CMAKE_INSTALL_PREFIX}/Library/lib
    /usr/lib
)

# handle the QUIETLY and REQUIRED arguments and set Casadi_FOUND to TRUE if
# all listed variables are TRUE
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (Casadi DEFAULT_MSG 
    Casadi_LIBRARY
    Casadi_INCLUDE_DIR
)
