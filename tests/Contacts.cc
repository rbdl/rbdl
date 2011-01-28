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

		Q = cmlVector(model->mBodies.size() - 1);
		QDot = cmlVector(model->mBodies.size() - 1);
		QDDot = cmlVector(model->mBodies.size() - 1);
		Tau = cmlVector(model->mBodies.size() - 1);

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
		child_rot_z_id, child_rot_y_id, child_rot_x_id;

	Body base_rot_z, base_rot_y, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x;

	Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x;

	cmlVector Q;
	cmlVector QDot;
	cmlVector QDDot;
	cmlVector Tau;

	unsigned int contact_body_id;
	Vector3d contact_point;
	Vector3d contact_normal;
};

TEST_FIXTURE(ContactsFixture, TestContactSimple) {
	model->AddContact(contact_body_id, contact_point, contact_normal);

	/*
	Q[0] = -0.2;
	Q[3] = 0.2;
	QDot[0] = 1.;
	*/

	Q[0] = 0.1;

	{
		_NoLogging nolog;
		ForwardDynamics (*model, Q, QDot, Tau, QDDot);
	}

	/*
	SpatialVector ext_force_body (
			0., 0., 0.,
			0., 1.164439e+01, 0.
			);
			*/
//	ext_force_body = Xtrans (

	cout << "FDab no contact = " << QDDot << endl;

	Vector3d point_accel;
	{
//		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point, point_accel);
	}
	cout << LogOutput.str() << endl;
	LogOutput.str("");
		
	cout << "point accel pre  = " << point_accel << endl;

	ForwardDynamicsContacts(*model, Q, QDot, Tau, QDDot);
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point, point_accel);
	}
	cout << LogOutput.str() << endl;
	cout  << "point accel post = " << point_accel << endl;

	cout << "QDDot Contact = " << QDDot << endl;

//	CHECK_CLOSE(0., cml::dot(point_accel, contact_normal), TEST_PREC);
}


/*
TEST_FIXTURE(ContactsFixture, TestContactSimple) {
	model->AddContact(body_id, contact_point, contact_normal);

	Vector3d point_accel;
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel);
	}
	LOG << "point accel pre  = " << point_accel << endl;

	ForwardDynamicsContacts(*model, Q, QDot, Tau, QDDot);
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel);
	}
//	cout << LogOutput.str() << endl;
	LOG << "point accel post = " << point_accel << endl;

	CHECK_CLOSE(0., cml::dot(point_accel, contact_normal), TEST_PREC);
}

TEST_FIXTURE(ContactsFixture, TestContactSimpleMoving) {
	model->AddContact(body_id, contact_point, contact_normal);
	QDot[0] = -0.3;
	QDot[1] = 0.4;

	Vector3d point_accel_pre;
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel_pre);
	}
	LOG << "point accel pre  = " << point_accel_pre << endl;

	Vector3d point_accel_post;
	ForwardDynamicsContacts(*model, Q, QDot, Tau, QDDot);
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel_post);
	}
//	cout << LogOutput.str() << endl;
	LOG << "point accel post = " << point_accel_post << endl;

	double accel_value = cml::dot(point_accel_post, contact_normal);
	LOG << scientific << "Accel value = " << accel_value << endl;

	CHECK_CLOSE(0., accel_value, TEST_PREC);
}

TEST_FIXTURE(ContactsFixture, TestContactSimpleAccelerating) {
	model->AddContact(body_id, contact_point, contact_normal);

	QDot[0] = -0.3;
	QDot[1] = 0.4;
	QDDot[0] = 2.;
	QDDot[1] = -2.;
	QDDot[2] = -1.5;

	Vector3d point_accel_pre;
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel_pre);
	}
	LOG << "point accel pre  = " << point_accel_pre << endl;

	Vector3d point_accel_post;
	ForwardDynamicsContacts(*model, Q, QDot, Tau, QDDot);
	
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel_post);
	}

	double accel_value = cml::dot(point_accel_post, contact_normal);

	LOG << "point accel post = " << point_accel_post << endl;
	LOG << scientific << "Accel value = " << accel_value << endl;

//	cout << LogOutput.str();

	CHECK_CLOSE(0., accel_value, TEST_PREC);
}

TEST_FIXTURE(ContactsFixture, TestContactSimpleAcceleratingRotated) {
	model->AddContact(body_id, contact_point, contact_normal);

	Q[0] = 0.7;
	Q[1] = 0.2;
	QDot[0] = -0.3;
	QDot[1] = 0.4;
	QDDot[0] = 2.;
	QDDot[1] = -2.;
	QDDot[2] = -1.5;

	Vector3d point_accel_pre;
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel_pre);
	}
	LOG << "point accel pre  = " << point_accel_pre << endl;

	Vector3d point_accel_post;
	ForwardDynamicsContacts(*model, Q, QDot, Tau, QDDot);
	
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel_post);
	}

	double accel_value = cml::dot(point_accel_post, contact_normal);

	LOG << "point accel post = " << point_accel_post << endl;
	LOG << scientific << "Accel value = " << accel_value << endl;
	
	// cout << LogOutput.str();

	CHECK_CLOSE(0., accel_value, TEST_PREC);
}

TEST_FIXTURE(ContactsFixture, TestContactDataFromSimulation) {
	model->AddContact(body_id, contact_point, contact_normal);

	Q[0] = -0.00618019;
	Q[1] = -0.00205865;
	Q[2] = 0.000296316;
	QDot[0] = -0.309134;
	QDot[1] = -0.102556;
	QDot[2] = 0.0154681;

	Vector3d point_accel_pre;
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel_pre);
	}
	LOG << "point accel pre  = " << point_accel_pre << endl;

	Vector3d point_accel_post;
	ForwardDynamicsContacts(*model, Q, QDot, Tau, QDDot);
	
	{
		_NoLogging nolog;
		CalcPointAcceleration (*model, Q, QDot, QDDot, body_id, contact_point, point_accel_post);
	}

	double accel_value = cml::dot(point_accel_post, contact_normal);

	LOG << "point accel post = " << point_accel_post << endl;
	LOG << scientific << "Accel value = " << accel_value << endl;
	
	// cout << LogOutput.str();

	CHECK_CLOSE(0., accel_value, TEST_PREC);
}
*/
