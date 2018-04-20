CMAKE_MINIMUM_REQUIRED (VERSION 3.0)

SET ( RBDL_ADDON_MUSCLE_TESTS_VERSION_MAJOR 1 )
SET ( RBDL_ADDON_MUSCLE_TESTS_VERSION_MINOR 0 )
SET ( RBDL_ADDON_MUSCLE_TESTS_VERSION_PATCH 0 )

SET ( RBDL_ADDON_MUSCLE_VERSION 
	${RBDL_ADDON_MUSCLE_TESTS_VERSION_MAJOR}.${RBDL_ADDON_MUSCLE_TESTS_VERSION_MINOR}.${RBDL_ADDON_MUSCLE_TESTS_VERSION_PATCH}
)


PROJECT (RBDL_MUSCLE_TESTS VERSION ${RBDL_ADDON_MUSCLE_VERSION})
#SET (PROJECT_VERSION ${RBDL_ADDON_MUSCLE_TESTS_VERSION})

# Needed for UnitTest++
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMake )

# Look for unittest++
FIND_PACKAGE (UnitTest++ REQUIRED)
INCLUDE_DIRECTORIES (${UNITTEST++_INCLUDE_DIR})

SET ( MUSCLE_TESTS_SRCS
	testMillard2016TorqueMuscle.cc
	testMuscleFunctionFactory.cc
	testTorqueMuscleFunctionFactory.cc
    ../muscle.h
    ../MuscleFunctionFactory.h
    ../MuscleFunctionFactory.cc
    ../TorqueMuscleFunctionFactory.h
		../TorqueMuscleFunctionFactory.cc    
    ../Millard2016TorqueMuscle.h
    ../Millard2016TorqueMuscle.cc
    ../csvtools.h
    ../csvtools.cc
    ../../geometry/geometry.h
    ../../geometry/SegmentedQuinticBezierToolkit.h
    ../../geometry/SmoothSegmentedFunction.h
    ../../geometry/SegmentedQuinticBezierToolkit.cc
    ../../geometry/SmoothSegmentedFunction.cc
    ../../geometry/tests/numericalTestFunctions.cc
		../../geometry/tests/numericalTestFunctions.h
	)

INCLUDE_DIRECTORIES ( ../../../geometry )

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
)

ADD_EXECUTABLE ( rbdl_muscle_tests ${MUSCLE_TESTS_SRCS} )

SET_TARGET_PROPERTIES ( rbdl_muscle_tests PROPERTIES
	LINKER_LANGUAGE CXX
	OUTPUT_NAME runMuscleTests
	)

SET (RBDL_LIBRARY rbdl)
IF (RBDL_BUILD_STATIC)
	SET (RBDL_LIBRARY rbdl-static)
ENDIF (RBDL_BUILD_STATIC)

TARGET_LINK_LIBRARIES ( rbdl_muscle_tests
		${UNITTEST++_LIBRARY}
		${RBDL_LIBRARY}
	)
	
OPTION (RUN_AUTOMATIC_TESTS "Perform automatic tests after compilation?" OFF)

IF (RUN_AUTOMATIC_TESTS)
ADD_CUSTOM_COMMAND (TARGET rbdl_muscle_tests
	POST_BUILD
	COMMAND ./runMuscleTests
	COMMENT "Running automated addon muscle tests..."
	)
ENDIF (RUN_AUTOMATIC_TESTS)
