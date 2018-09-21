FIND_PROGRAM ( PYTHON "python3" )

SET (Python_ADDITIONAL_VERSIONS 3.7)

INCLUDE ( UseCython )
INCLUDE ( FindNumPy3 )

MESSAGE(${PYTHON_NUMPY_3_INCLUDE_DIR})

FILE( COPY "crbdl.pxd" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

CONFIGURE_FILE (
  ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/setup.py
  )

# Process the rbdl-wrapper.pyx to generate rbdl.pyx BEFORE build time
# else cython_add_module will complain on configuration
EXECUTE_PROCESS (
  COMMAND python3
  ${CMAKE_CURRENT_SOURCE_DIR}/wrappergen.py
  ${CMAKE_CURRENT_SOURCE_DIR}/rbdl-wrapper.pyx
  ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pyx
  OUTPUT_QUIET
)

# Enable C++11 (or C++0x for older compilers)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# If the pyx file is a C++ file, we should specify that here.
set_source_files_properties( ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pyx
  PROPERTIES CYTHON_IS_CXX TRUE )

# Multi-file cython modules do not appear to be working at the moment.
# NOTE there is some directory property problem in UseCython when build
#      from CMAKE_CURRENT_BINARY_DIR
cython_add_module( rbdl-python3 ${CMAKE_CURRENT_BINARY_DIR}/rbdl.pyx )

#SET_TARGET_PROPERTIES ( rbdl-python3 PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES ( rbdl-python3 PROPERTIES OUTPUT_NAME "rbdl")

INCLUDE_DIRECTORIES (
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/python3
  ${PROJECT_SOURCE_DIR}
  ${PYTHON_NUMPY_3_INCLUDE_DIR}
  )

TARGET_LINK_LIBRARIES (rbdl-python3 rbdl )

IF (RBDL_BUILD_ADDON_LUAMODEL)
  TARGET_LINK_LIBRARIES (rbdl-python3 rbdl_luamodel )
ENDIF()

IF (RBDL_BUILD_ADDON_URDFREADER)
  TARGET_LINK_LIBRARIES (rbdl-python3 rbdl_urdfreader )
ENDIF()

INSTALL ( CODE "EXECUTE_PROCESS(WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMAND python3 setup.py install)")
