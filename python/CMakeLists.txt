FIND_PROGRAM ( PYTHON "python" )

SET (Python_ADDITIONAL_VERSIONS 2.7)

INCLUDE ( UseCython )

CONFIGURE_FILE (
  ${CMAKE_CURRENT_SOURCE_DIR}/setup.py.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/setup.py
  )
  
  
execute_process(COMMAND ${PYTHON}
                  ${CMAKE_CURRENT_SOURCE_DIR}/wrappergen.py
                  ${CMAKE_CURRENT_SOURCE_DIR}/rbdl-wrapper.pyx
                  ${CMAKE_CURRENT_SOURCE_DIR}/rbdl.pyx
  )
  
execute_process(COMMAND ${PYTHON} 
                  ${CMAKE_CURRENT_SOURCE_DIR}/wrappergen.py 
                  ${CMAKE_CURRENT_SOURCE_DIR}/rbdlmuscle-wrapper.pyx 
                  ${CMAKE_CURRENT_SOURCE_DIR}/rbdlmuscle.pyx
                )

FILE( COPY "rbdl.pyx" "rbdl.pxd" "crbdl.pxd" "ICustomJoint.cpp" 
            "rbdlmuscle.pyx" "crbdlmuscle.pxd" DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")

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
set_source_files_properties( ${CMAKE_CURRENT_SOURCE_DIR}/rbdl.pyx
  PROPERTIES CYTHON_IS_CXX TRUE )

# Multi-file cython modules do not appear to be working at the moment.
cython_add_module( rbdl-python ${CMAKE_CURRENT_SOURCE_DIR}/rbdl.pyx ICustomJoint.cpp )

#SET_TARGET_PROPERTIES ( rbdl-python PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES ( rbdl-python PROPERTIES OUTPUT_NAME "rbdl")

IF (RBDL_BUILD_ADDON_MUSCLE)
  set_source_files_properties( ${CMAKE_CURRENT_SOURCE_DIR}/rbdlmuscle.pyx
  PROPERTIES CYTHON_IS_CXX TRUE ) 
  cython_add_module( rbdlmuscle-python ${CMAKE_CURRENT_SOURCE_DIR}/rbdlmuscle.pyx )
  SET_TARGET_PROPERTIES ( rbdlmuscle-python PROPERTIES OUTPUT_NAME "rbdlmuscle")
ENDIF()


INCLUDE_DIRECTORIES (
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/python
  ${PROJECT_SOURCE_DIR}
  )
  
IF (RBDL_BUILD_ADDON_MUSCLE_FITTING)
  #FIND_PACKAGE (IPOPT REQUIRED)
  INCLUDE_DIRECTORIES (
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/python
  ${PROJECT_SOURCE_DIR}
  ${IPOPT_INCLUDE_DIR}
  )
ENDIF()


TARGET_LINK_LIBRARIES (rbdl-python rbdl )

IF (RBDL_BUILD_ADDON_LUAMODEL) 
  TARGET_LINK_LIBRARIES (rbdl-python rbdl_luamodel )
ENDIF()

IF (RBDL_BUILD_ADDON_URDFREADER) 
  TARGET_LINK_LIBRARIES (rbdl-python rbdl_urdfreader )
ENDIF()

IF (RBDL_BUILD_ADDON_MUSCLE) 
  TARGET_LINK_LIBRARIES (rbdlmuscle-python rbdl rbdl_geometry rbdl_muscle )
  IF (RBDL_BUILD_ADDON_LUAMODEL) 
    TARGET_LINK_LIBRARIES (rbdlmuscle-python rbdl_luamodel )
  ENDIF()
  IF (RBDL_BUILD_ADDON_URDFREADER) 
    TARGET_LINK_LIBRARIES (rbdlmuscle-python rbdl_urdfreader )
  ENDIF()
ENDIF()

INSTALL ( TARGETS rbdl-python
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/
)

IF (RBDL_BUILD_ADDON_MUSCLE)
INSTALL ( TARGETS rbdlmuscle-python
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python2.7/site-packages/
)
ENDIF()
