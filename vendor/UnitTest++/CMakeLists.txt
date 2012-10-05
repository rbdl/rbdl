PROJECT (UNITTESTPP)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET (UNITTESTPP_SOURCES
	src/AssertException.cpp 
	src/Test.cpp 
	src/Checks.cpp 
	src/TestRunner.cpp 
	src/TestResults.cpp 
	src/TestReporter.cpp 
	src/TestReporterStdout.cpp 
	src/ReportAssert.cpp 
	src/TestList.cpp 
	src/TimeConstraint.cpp 
	src/TestDetails.cpp 
	src/MemoryOutStream.cpp 
	src/DeferredTestReporter.cpp 
	src/DeferredTestResult.cpp 
	src/XmlTestReporter.cpp 
	src/CurrentTest.cpp
  src/Posix/SignalTranslator.cpp
	src/Posix/TimeHelpers.cpp
)

ADD_LIBRARY ( unittest++ SHARED ${UNITTESTPP_SOURCES})
