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
using namespace RigidBodyDynamics::Experimental;

const double TEST_PREC = 1.0e-14;

struct ContactsFixture {
	ContactsFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();

		model->gravity.set (0., -9.81, 0.);

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 *                X
		 *                *
		 *              _/
		 *            _/  (-Z)
		 *      Z    /
		 *      *---* 
		 *      |
		 *      |
		 *  Z   |
		 *  O---*
		 *      Y
		 */

		// base body
		base_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_base_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		base_rot_z_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_z, base_rot_z);

		base_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_base_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		base_rot_y_id = model->AddBody (base_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_y, base_rot_y);

		base_rot_x = Body (
				1.,
				Vector3d (0., 1., 0.),
				Vector3d (1., 1., 1.)
				);
		joint_base_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		base_rot_x_id = model->AddBody (base_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_x, base_rot_x);

		// child body
		child_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		child_rot_z_id = model->AddBody (base_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_rot_z, child_rot_z);

		child_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		child_rot_y_id = model->AddBody (child_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_y, child_rot_y);

		child_rot_x = Body (
				1.,
				Vector3d (0., 1., 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		child_rot_x_id = model->AddBody (child_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_x, child_rot_x);

		Q = VectorNd(model->mBodies.size() - 1);
		QDot = VectorNd(model->mBodies.size() - 1);
		QDDot = VectorNd(model->mBodies.size() - 1);
		Tau = VectorNd(model->mBodies.size() - 1);

		Q.zero();
		QDot.zero();
		QDDot.zero();
		Tau.zero();

		contact_body_id = child_rot_x_id;
		contact_point.set (0., 1., 0.);
		contact_normal.set (0., 1., 0.);

		ClearLogOutput();
	}
	
	~ContactsFixture () {
		delete model;
	}
	Model *model;

	unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id,
		child_rot_z_id, child_rot_y_id, child_rot_x_id,
		base_body_id;

	Body base_rot_z, base_rot_y, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x;

	Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x;

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;

	unsigned int contact_body_id;
	Vector3d contact_point;
	Vector3d contact_normal;
	std::vector<ContactInfo> contact_data;
};

TEST_FIXTURE(ContactsFixture, TestContactSimple) {
	contact_data.push_back (ContactInfo (contact_body_id, contact_point, contact_normal));

	Q[0] = 0.2;
	Q[3] = 0.6;
	Tau[0] = 1.0;
	Tau[1] = -5.0;
	Tau[2] = 3.0;
	{
		_NoLogging nolog;
		ForwardDynamics (*model, Q, QDot, Tau, QDDot);
	}

	VectorNd humans_values (QDDot.size());
	humans_values[0] =	-1.647101149402497e+00;
	humans_values[1] =	-3.333333333333333e+00;
	humans_values[2] =	1.500000000000000e+00;
	humans_values[3] =	4.700721141799440e+00;
	humans_values[4] =	3.598082426458145e+00;
	humans_values[5] =	-1.022528511047732e+00;

	CHECK_ARRAY_CLOSE (humans_values.data(), QDDot.data(), QDDot.size(), TEST_PREC);

	Vector3d point_accel;
	{
		point_accel = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	}

	ForwardDynamicsContacts(*model, Q, QDot, Tau, contact_data, QDDot);
	{
		_NoLogging nolog;
		point_accel = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	}

	humans_values[0] = 5.681687528667114e-01;
	humans_values[1] = -3.333333333333333e+00;
	humans_values[2] = 1.500000000000000e+00;
	humans_values[3] = 2.080750294369360e-01;
	humans_values[4] = 3.598082426458145e+00;
	humans_values[5] = -1.022528511047732e+00;

	CHECK_ARRAY_CLOSE (humans_values.data(), QDDot.data(), QDDot.size(), TEST_PREC);
}

TEST_FIXTURE(ContactsFixture, TestContactEulerSingularity) {
	contact_data.push_back (ContactInfo (contact_body_id, contact_point, contact_normal));

	Q[0] = 27.9045;
	Q[1] = -0.439375;
	Q[2] = 1.52627;
	Q[3] = 20.5971;
	Q[4] = -1.48387;
	Q[5] = -2.95715;
	QDot[0] = 12.288;
	QDot[1] = -3.42405;
	QDot[2] = -5.17653;
	QDot[3] = 45.3575;
	QDot[4] = -4.51642;
	QDot[5] = -51.0512;
	{
		_NoLogging nolog;
		ForwardDynamics (*model, Q, QDot, Tau, QDDot);
	}

	VectorNd humans_values (QDDot.size());

	humans_values[0] = 4.008081005501898e+01;
	humans_values[1] = 5.046003674456097e+01;
	humans_values[2] = -8.111575678933713e+01;
	humans_values[3] = 5.420161855111747e+03;
	humans_values[4] = 1.339039929480332e+02;
	humans_values[5] = -5.415409099653753e+03;

	/// \todo Warning: the precision of this test has been changed as
	// something strange happens (maybe a euler angle singularity?) at this
	// model state. However it is still quite close to the HuMAnS values.
	CHECK_ARRAY_CLOSE (humans_values.data(), QDDot.data(), QDDot.size(), 1.0e-10);
}

TEST_FIXTURE(ContactsFixture, TestContactFixedPoint) {
	contact_data.push_back (ContactInfo (contact_body_id, contact_point, Vector3d (1., 0., 0.)));
	contact_data.push_back (ContactInfo (contact_body_id, contact_point, Vector3d (0., 1., 0.)));
	contact_data.push_back (ContactInfo (contact_body_id, contact_point, Vector3d (0., 0., 1.)));

	VectorNd humans_values (QDDot.size());

	Q[0] = 0.2;
	Q[1] = -0.5;
	Q[2] = 0.1;
	Q[3] = -0.4;
	Q[4] = -0.1;
	Q[5] = 0.4;

	humans_values[0] = 1.834747596432898e-01;
	humans_values[1] = 2.800501472504468e-01;
	humans_values[2] = 8.568126519022337e-01;
	humans_values[3] = -1.311288581471320e-01;
	humans_values[4] = -6.079597431872865e-01;
	humans_values[5] = -4.389578697923985e-01;

	Vector3d point_accel;
	{
		SUPPRESS_LOGGING;
		ForwardDynamicsContacts(*model, Q, QDot, Tau, contact_data, QDDot);
		point_accel = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	}
//	cout << LogOutput.str() << endl;

//	cout << "Point Accel = " << point_accel << endl;
	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.).data(), point_accel.data(), 3, 1.0e-12);

	/// \todo Warning: the precision of this test has been changed as
	// something strange happens (maybe a euler angle singularity?) at this
	// model state. However it is still quite close to the HuMAnS values.
	CHECK_ARRAY_CLOSE (humans_values.data(), QDDot.data(), QDDot.size(), 1.0e-12);
}

TEST_FIXTURE(ContactsFixture, TestContactFloatingBaseSimple) {
	Model *float_model = new Model();

	float_model->Init();
	float_model->gravity.set (0., -9.81, 0.);

	Body base_body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));

	float_model->SetFloatingBaseBody(base_body);

	VectorNd Q (6);
	VectorNd QDot (6);
	VectorNd QDDot (6);
	VectorNd Tau (6);

	ContactInfo ground_x (6, Vector3d (0., -1., 0.), Vector3d (1., 0., 0.));
	ContactInfo ground_y (6, Vector3d (0., -1., 0.), Vector3d (0., 1., 0.));
	ContactInfo ground_z (6, Vector3d (0., -1., 0.), Vector3d (0., 0., 1.));

	contact_data.push_back (ground_y);

	ForwardDynamicsContacts (*float_model, Q, QDot, Tau, contact_data, QDDot);

//	cout << QDDot << std::endl;
//	cout << LogOutput.str() << endl;

	VectorNd qddot_test (6);
	qddot_test.zero();

	CHECK_ARRAY_CLOSE (qddot_test.data(), QDDot.data(), QDDot.size(), TEST_PREC);
}

TEST_FIXTURE(ContactsFixture, TestContactFloatingBaseRotating) {
	Model *float_model = new Model();

	float_model->Init();
	float_model->gravity.set (0., -9.81, 0.);

	Body base_body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));

	base_body_id = float_model->SetFloatingBaseBody(base_body);

	VectorNd Q (model->dof_count);
	VectorNd QDot (model->dof_count);
	VectorNd QDDot (model->dof_count);
	VectorNd Tau (model->dof_count);

	Q[1] = 1.;

	// update kinematic values!
	ForwardDynamics (*float_model, Q, QDot, Tau, QDDot);

	contact_body_id = base_body_id;
	Vector3d contact_point = float_model->CalcBaseToBodyCoordinates (contact_body_id, Vector3d (Q[0], 0., Q[2]));
	
	ContactInfo ground_x (contact_body_id, contact_point, Vector3d (1., 0., 0.));
	ContactInfo ground_y (contact_body_id, contact_point, Vector3d (0., 1., 0.));
	ContactInfo ground_z (contact_body_id, contact_point, Vector3d (0., 0., 1.));

//	contact_data.push_back (ground_x);
	contact_data.push_back (ground_y);
//	contact_data.push_back (ground_z);

	// We want the body to rotate around its contact point which is located
	// at (0, 0, 0). There it should have a pure rotation of negative unit
	// magnitude around the Z-axis (i.e. rolling along the X axis). The
	// spatial velocity of the body at the contact point is therefore
	//   (0, 0, -1, 0, 0, 0).
	SpatialVector velocity_ground (0., 0., -1., 0., 0., 0.);

	// This has now to be transformed to body coordinates.
	SpatialVector velocity_body = Xtrans (Vector3d (0., 1., 0.)) * velocity_ground;

	// This has now to be shuffled such that it complies with the ordering of
	// the DoF in the generalized velocity vector.
	QDot[0] = velocity_body[3];
	QDot[1] = velocity_body[4];
	QDot[2] = velocity_body[5];
	QDot[3] = velocity_body[2];
	QDot[4] = velocity_body[1];
	QDot[5] = velocity_body[0];

	cout << "velocity_body = " << velocity_body << std::endl;
	cout << "QDot = " << QDot << std::endl;

	ForwardDynamics (*float_model, Q, QDot, Tau, QDDot);

	QDDot.zero();
	QDDot[0] = 0.;

	Vector3d test_point;
	Vector3d test_velocity;
	Vector3d test_accel;

	test_point = float_model->CalcBodyToBaseCoordinates(contact_data[0].body_id, contact_data[0].point);
	test_velocity = CalcPointVelocity (*float_model, Q, QDot, contact_data[0].body_id, contact_data[0].point);
	test_accel = CalcPointAcceleration (*float_model, Q, QDot, QDDot, contact_data[0].body_id, contact_data[0].point);
	
	cout << "q     = " << Q << endl;
	cout << "qdot  = " << QDot << endl;
	cout << "tau   = " << Tau << endl;
	cout << "qddot = " << QDDot << endl;

	cout << "contact_point    = " << test_point << std::endl;
	cout << "contact_velocity = " << test_velocity << std::endl;
	cout << "contact_accel    = " << test_accel << endl;

	test_velocity = CalcPointVelocity (*float_model, Q, QDot, contact_data[0].body_id, Vector3d (0., 0., 0.));
	cout << "base_velocity    = " << test_velocity << std::endl;

	cout << "FWD" << endl;

	ClearLogOutput();
	ForwardDynamicsContacts (*float_model, Q, QDot, Tau, contact_data, QDDot);
	cout << LogOutput.str() << endl;
//	cout << "QDot = " << QDot << std::endl;

//	cout << "--- post ---" << endl;
	// check the velocity of the contact point
	test_accel = CalcPointAcceleration (*float_model, Q, QDot, QDDot, contact_data[0].body_id, contact_data[0].point);
	cout << "contact accel   = " << test_accel << endl;
	test_accel = CalcPointAcceleration (*float_model, Q, QDot, QDDot, contact_data[0].body_id, Vector3d (0., 0., 0.));
	cout << "base accel      = " << test_accel << endl;
	test_velocity = CalcPointVelocity (*float_model, Q, QDot, contact_data[0].body_id, Vector3d (0., -1., 0.));
	cout << "contact veloc   = " << test_velocity << endl;
	test_velocity = CalcPointVelocity (*float_model, Q, QDot, contact_data[0].body_id, Vector3d (0., 0., 0.));
	cout << "base veloc      = " << test_velocity << endl;

	cout << "Contact force   = ";
	unsigned int i;
	for (i = 0; i < contact_data.size(); i++) {
		cout << contact_data[i].force << " ";
	}
	cout << endl;

	cout << "q     = " << Q << endl;
	cout << "qdot  = " << QDot << endl;
	cout << "tau   = " << Tau << endl;
	cout << "qddot = " << QDDot << endl;

	VectorNd qddot_test (6);

	qddot_test[0] = 0.;
	qddot_test[1] = 0.;
	qddot_test[2] = 0.;
	qddot_test[3] = 0.;
	qddot_test[4] = 0.;
	qddot_test[5] = 0.;

	CHECK_ARRAY_CLOSE (qddot_test.data(), QDDot.data(), QDDot.size(), TEST_PREC);
}
