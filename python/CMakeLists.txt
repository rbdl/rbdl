# Process the rbdl.template.pyx to generate rbdl.pyx
ADD_CUSTOM_COMMAND ( OUTPUT ${PROJECT_SOURCE_DIR}/python/rbdl.pyx
	COMMAND "/usr/bin/python"
	ARGS ${PROJECT_SOURCE_DIR}/python/wrappergen.py ${PROJECT_SOURCE_DIR}/python/rbdl.template.pyx ${PROJECT_SOURCE_DIR}/python/rbdl.pyx
	COMMENT "Generatying rbdl.pyx from rbdl.template.pyx"
	DEPENDS ${PROJECT_SOURCE_DIR}/python/rbdl.template.pyx ${PROJECT_SOURCE_DIR}/python/wrappergen.py
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
set_source_files_properties(
	${PROJECT_SOURCE_DIR}/python/rbdl.pyx
  PROPERTIES CYTHON_IS_CXX TRUE )

# Multi-file cython modules do not appear to be working at the moment.
cython_add_module( rbdl-python rbdl.pyx )

SET_TARGET_PROPERTIES ( rbdl-python PROPERTIES PREFIX "")
SET_TARGET_PROPERTIES ( rbdl-python PROPERTIES OUTPUT_NAME "rbdl")

INCLUDE_DIRECTORIES (
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/python
	${PROJECT_SOURCE_DIR}
	)

TARGET_LINK_LIBRARIES (rbdl-python rbdl_luamodel rbdl_urdfreader )

INSTALL ( TARGETS rbdl-python
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python-2.7/rbdl
	)
