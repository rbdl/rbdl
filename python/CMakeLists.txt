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
	)

TARGET_LINK_LIBRARIES (rbdl-python rbdl)

INSTALL ( TARGETS rbdl-python
	LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/python-2.7/rbdl
	)
