CMAKE_MINIMUM_REQUIRED (VERSION 3.0)

# Needed for UnitTest++
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../CMake )

# Look for unittest++
FIND_PACKAGE (UnitTest++ REQUIRED)
INCLUDE_DIRECTORIES (${UNITTEST++_INCLUDE_DIR})

SET ( TESTS_SRCS
  main.cc
  MathTests.cc
  ErrorTests.cc
  SpatialAlgebraTests.cc
  MultiDofTests.cc
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
  UtilsTests.cc
  SparseFactorizationTests.cc
  CustomJointSingleBodyTests.cc
  CustomJointMultiBodyTests.cc
  CustomConstraintsTests.cc
  InverseKinematicsTests.cc
  LoopConstraintsTests.cc
  ScrewJointTests.cc
  ForwardDynamicsConstraintsExternalForces.cc
  InverseDynamicsWithConstraintsTests.cc  
  )

INCLUDE_DIRECTORIES ( ../src/ )

SET_TARGET_PROPERTIES ( ${PROJECT_EXECUTABLES} PROPERTIES
  LINKER_LANGUAGE CXX
  OUTPUT_NAME runtests
  )

ADD_EXECUTABLE ( rbdl_tests ${TESTS_SRCS} )

SET_TARGET_PROPERTIES ( rbdl_tests PROPERTIES
  LINKER_LANGUAGE CXX
  OUTPUT_NAME runtests
  )

SET (RBDL_LIBRARY rbdl)
IF (RBDL_BUILD_STATIC)
  SET (RBDL_LIBRARY rbdl-static)
ENDIF (RBDL_BUILD_STATIC)

TARGET_LINK_LIBRARIES ( rbdl_tests
  ${UNITTEST++_LIBRARY}
  ${RBDL_LIBRARY}
  )

OPTION (RUN_AUTOMATIC_TESTS "Perform automatic tests after compilation?" OFF)

IF (RUN_AUTOMATIC_TESTS)
  ADD_CUSTOM_COMMAND (TARGET rbdl_tests
    POST_BUILD
    COMMAND ./runtests
    COMMENT "Running automated tests..."
    )
ENDIF (RUN_AUTOMATIC_TESTS)
