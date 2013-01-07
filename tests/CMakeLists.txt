PROJECT (RBDL_TESTS)

CMAKE_MINIMUM_REQUIRED (VERSION 2.6)

# Needed for UnitTest++
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMake )

SET ( TESTS_SRCS
	main.cc
	MathTests.cc
	SpatialAlgebraTests.cc
	KinematicsTests.cc
	BodyTests.cc
	ModelTests.cc
	FloatingBaseTests.cc
	CalcVelocitiesTests.cc
	CalcAccelerationsTests.cc
	DynamicsTests.cc
	InverseDynamicsTests.cc
	CompositeRigidBodyTests.cc
	ImpulsesTests.cc
	TwolegModelTests.cc
	ContactsTests.cc
	)

INCLUDE_DIRECTORIES ( ../src/ )

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
)

ADD_EXECUTABLE ( rbdl_tests ${TESTS_SRCS} )

SET_TARGET_PROPERTIES ( rbdl_tests PROPERTIES
	LINKER_LANGUAGE CXX
	OUTPUT_NAME runtests
	)

TARGET_LINK_LIBRARIES ( rbdl_tests
		unittest++
		rbdl	
	)
	
OPTION (RUN_AUTOMATIC_TESTS "Perform automatic tests after compilation?" OFF)

IF (RUN_AUTOMATIC_TESTS)
ADD_CUSTOM_COMMAND (TARGET runtests
	POST_BUILD
	COMMAND ./runtests
	COMMENT "Running automated tests..."
	)
ENDIF (RUN_AUTOMATIC_TESTS)
