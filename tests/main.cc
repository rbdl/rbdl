#include <UnitTest++.h>
#include <iostream>
#include <string>

#include <rbdl/rbdl.h>

int main (int argc, char *argv[])
{
	if (argc > 1) {
		std::string arg (argv[1]);
	
		if (arg == "-v" || arg == "--version")
			RigidBodyDynamics::rbdl_print_version();
	}

	return UnitTest::RunAllTests ();
}
