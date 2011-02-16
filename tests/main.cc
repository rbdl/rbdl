#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Contacts.h"
#include "Dynamics.h"
#include "Kinematics.h"

using namespace std;
using namespace SpatialAlgebra;
using namespace RigidBodyDynamics;

int main (int argc, char *argv[])
{
	/*
	Model *float_model = new Model();
	std::vector<ContactInfo> contact_data;

	float_model->Init();
	float_model->gravity.set (0., -9.81, 0.);

	Body base_body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));

	float_model->SetFloatingBaseBody(base_body);

	cmlVector Q (6);
	cmlVector QDot (6);
	cmlVector QDDot (6);
	cmlVector Tau (6);

	ContactInfo ground_x (0, Vector3d (0., -1., 0.), Vector3d (1., 0., 0.));
	ContactInfo ground_y (0, Vector3d (0., -1., 0.), Vector3d (0., 1., 0.));
	ContactInfo ground_z (0, Vector3d (0., -1., 0.), Vector3d (0., 0., 1.));

	contact_data.push_back (ground_y);

	ForwardDynamicsContacts (*float_model, Q, QDot, Tau, contact_data, QDDot);

	cout << QDDot << std::endl;
	cout << LogOutput.str() << endl;

	cmlVector qddot_test (6);
	qddot_test.zero();
	*/
	
	return UnitTest::RunAllTests ();
}
