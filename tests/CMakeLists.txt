PROJECT (ABA_TESTS)

CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# Needed for UnitTest++
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMake )

SET ( TESTS_SRCS
	main.cc
	SpatialAlgebraTests.cc
	MathTests.cc
	BodyTests.cc
	ArticulatedFigureTests.cc
	FloatingBaseTests.cc
	CalcVelocitiesTests.cc
	CalcAccelerationsTests.cc
	KinematicsTests.cc
	# ContactsTests.cc
	# ImpulsesTests.cc
	)

FIND_PACKAGE (UnitTest++)

INCLUDE_DIRECTORIES ( ../src/ )

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
)

IF ( UNITTEST++_FOUND )
	ADD_EXECUTABLE ( aba_tests ${TESTS_SRCS} )

  INCLUDE_DIRECTORIES ( ${UNITTEST++_INCLUDE_DIR} )

	SET_TARGET_PROPERTIES ( aba_tests PROPERTIES
		LINKER_LANGUAGE CXX
		OUTPUT_NAME runtests
		)

	TARGET_LINK_LIBRARIES ( aba_tests
			${UNITTEST++_LIBRARY}
			aba	
		)
		
	OPTION (RUN_AUTOMATIC_TESTS "Perform automatic tests after compilation?" OFF)

	IF (RUN_AUTOMATIC_TESTS)
	ADD_CUSTOM_COMMAND (TARGET runtests
		POST_BUILD
		COMMAND ./runtests
		COMMENT "Running automated tests..."
		)
	ENDIF (RUN_AUTOMATIC_TESTS)

ENDIF ( UNITTEST++_FOUND )

