find_package (Ipopt CONFIG QUIET)

if (Ipopt_FOUND AND NOT IPOPT_FOUND)
  foreach (_ipopt_target Ipopt::ipopt ipopt::ipopt ipopt)
    if (TARGET ${_ipopt_target})
      set (IPOPT_LIBRARY ${_ipopt_target})
      if (NOT IPOPT_INCLUDE_DIR)
        get_target_property (IPOPT_INCLUDE_DIR ${_ipopt_target} INTERFACE_INCLUDE_DIRECTORIES)
      endif()
    endif()
  endforeach()

  if (NOT IPOPT_INCLUDE_DIR AND Ipopt_INCLUDE_DIRS)
    list (GET Ipopt_INCLUDE_DIRS 0 IPOPT_INCLUDE_DIR)
  endif()
  if (NOT IPOPT_LIBRARY AND Ipopt_LIBRARIES)
    list (GET Ipopt_LIBRARIES 0 IPOPT_LIBRARY)
  endif()
endif()

find_package (PkgConfig QUIET)
if (PKG_CONFIG_FOUND)
  pkg_check_modules (IPOPT_PC QUIET ipopt)
endif()

find_path (IPOPT_INCLUDE_DIR IpTNLP.hpp
  HINTS
  ${IPOPT_PC_INCLUDE_DIRS}
  PATHS
  ${CUSTOM_IPOPT_PATH}/include
  PATH_SUFFIXES
  coin
  coin-or
  )

find_library (IPOPT_LIBRARY ipopt
  HINTS
  ${IPOPT_PC_LIBRARY_DIRS}
  PATHS
  ${CUSTOM_IPOPT_PATH}/lib
  )

include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (IPOPT DEFAULT_MSG
  IPOPT_LIBRARY
  IPOPT_INCLUDE_DIR
  )

if (IPOPT_FOUND)
  set (IPOPT_LIBRARIES ${IPOPT_LIBRARY})
  set (IPOPT_INCLUDE_DIRS ${IPOPT_INCLUDE_DIR})
endif()

mark_as_advanced (
  IPOPT_INCLUDE_DIR
  IPOPT_LIBRARY
  )
