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
using namespace SpatialAlgebra::Operators;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Experimental;

const double TEST_PREC = 1.0e-14;

struct ContactsFixture {
	ContactsFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();

		model->gravity = Vector3d  (0., -9.81, 0.);

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

		Q = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		Tau = VectorNd::Constant (model->mBodies.size() - 1, 0.);

		contact_body_id = child_rot_x_id;
		contact_point = Vector3d  (0., 1., 0.);
		contact_normal = Vector3d  (0., 1., 0.);

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

TEST ( TestForwardDynamicsContactsLagrangianSimple ) {
	Model model;
	model.Init();
	model.gravity = Vector3d  (0., -9.81, 0.);
	Body base_body (1., Vector3d (0., 0., 0.), Vector3d (1., 1., 1.));
	unsigned int base_body_id = model.SetFloatingBaseBody(base_body);

	VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant  ((size_t) model.dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);

	Q[1] = 1.;
	QDot[0] = 1.;
	QDot[3] = -1.;

	unsigned int contact_body_id = base_body_id;
	Vector3d contact_point ( 0., -1., 0.);

	ContactInfo ground_x (contact_body_id, contact_point, Vector3d (1., 0., 0.));
	ContactInfo ground_y (contact_body_id, contact_point, Vector3d (0., 1., 0.));
	ContactInfo ground_z (contact_body_id, contact_point, Vector3d (0., 0., 1.));

	std::vector<ContactInfo> contact_data;

	contact_data.push_back (ground_x);
	contact_data.push_back (ground_y);
	contact_data.push_back (ground_z);

	ClearLogOutput();

	ForwardDynamicsContactsLagrangian (model, Q, QDot, Tau, contact_data, QDDot);

	Vector3d point_acceleration = CalcPointAcceleration (model, Q, QDot, QDDot, contact_body_id, contact_point);

	CHECK_ARRAY_CLOSE (
			Vector3d (0., 0., 0.).data(),
			point_acceleration.data(),
			3,
			TEST_PREC
			);

	// cout << "LagrangianSimple Logoutput Start" << endl;
	// cout << LogOutput.str() << endl;
	// cout << "LagrangianSimple Logoutput End" << endl;

	/*
	unsigned int i;
	for (i = 0; i < contact_data.size(); i++) {
		cout << "cf[" << i << "] = " << contact_data[i].force << endl;
	}

	cout << QDDot << endl;
	*/
}

TEST ( TestForwardDynamicsContactsLagrangianMoving ) {
	Model model;
	model.Init();
	model.gravity = Vector3d  (0., -9.81, 0.);
	Body base_body (1., Vector3d (0., 0., 0.), Vector3d (1., 1., 1.));
	unsigned int base_body_id = model.SetFloatingBaseBody(base_body);

	VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant  ((size_t) model.dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);

	Q[0] = 0.1;
	Q[1] = 0.2;
	Q[2] = 0.3;
	Q[3] = 0.4;
	Q[4] = 0.5;
	Q[5] = 0.6;
	QDot[0] = 1.1;
	QDot[1] = 1.2;
	QDot[2] = 1.3;
	QDot[3] = -1.4;
	QDot[4] = -1.5;
	QDot[5] = -1.6;

	unsigned int contact_body_id = base_body_id;
	Vector3d contact_point ( 0., -1., 0.);

	ContactInfo ground_x (contact_body_id, contact_point, Vector3d (1., 0., 0.));
	ContactInfo ground_y (contact_body_id, contact_point, Vector3d (0., 1., 0.));
	ContactInfo ground_z (contact_body_id, contact_point, Vector3d (0., 0., 1.));

	std::vector<ContactInfo> contact_data;

	contact_data.push_back (ground_x);
	contact_data.push_back (ground_y);
	contact_data.push_back (ground_z);

	ClearLogOutput();

	ForwardDynamicsContactsLagrangian (model, Q, QDot, Tau, contact_data, QDDot);

	Vector3d point_acceleration = CalcPointAcceleration (model, Q, QDot, QDDot, contact_body_id, contact_point);

	CHECK_ARRAY_CLOSE (
			Vector3d (0., 0., 0.).data(),
			point_acceleration.data(),
			3,
			TEST_PREC
			);

	// cout << "LagrangianSimple Logoutput Start" << endl;
	// cout << LogOutput.str() << endl;
	// cout << "LagrangianSimple Logoutput End" << endl;

	/*
	unsigned int i;
	for (i = 0; i < contact_data.size(); i++) {
		cout << "cf[" << i << "] = " << contact_data[i].force << endl;
	}

	cout << QDDot << endl;
	*/
}

TEST ( TestComputeAccelerationDeltas ) {
	Model *model;

	unsigned int body_a_id, body_b_id, base_rot_x_id,
		child_rot_z_id, child_rot_y_id, child_rot_x_id,
		base_body_id;

	Body body_a, body_b, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x;

	Joint joint_body_a, joint_body_b, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x;

	unsigned int contact_body_id;
	Vector3d contact_point;
	Vector3d contact_normal;
	std::vector<ContactInfo> contact_data;

	model = new Model;
	model->Init();

	model->gravity = Vector3d  (0., -9.81, 0.);

	/* A simple model that is located at the origin and has a rotational
	 * joint around the Z-axis.
	 *
	 *  Z   
	 *  O---*
	 *      
	 */

	// base body
	body_a = Body (
			1.,
			Vector3d (1., 0., 0.),
			Vector3d (1., 1., 1.)
			);
	joint_body_a = Joint (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);
	body_a_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_body_a, body_a);

	body_b = Body (
			1.,
			Vector3d (1., 0., 0.),
			Vector3d (1., 1., 1.)
			);
	joint_body_b = Joint (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);
	body_b_id = model->AddBody (body_a_id, Xtrans (Vector3d (1., 0., 0.)), joint_body_b, body_b);
	
	VectorNd Q = VectorNd::Zero (model->dof_count);
	VectorNd QDot = VectorNd::Zero (model->dof_count);
	VectorNd QDDot = VectorNd::Zero (model->dof_count);
	VectorNd Tau = VectorNd::Zero (model->dof_count);

	contact_point = Vector3d  (0., 1., 0.);
	contact_normal = Vector3d  (0., 1., 0.);

	Vector3d point_accel_0;	

	double fm = 1.;
	contact_body_id = body_b_id;
	contact_point.set (5., 0., 0.);

	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
	cout << "qddot = " << QDDot.transpose() << endl;
	point_accel_0 = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	cout << "point_accel_0 = " << point_accel_0.transpose() << endl;

	ClearLogOutput();

	// this is used here
	VectorNd QDDot_t = VectorNd::Zero (model->dof_count);

	SpatialVector f_t;

	f_t.set (0., 0., 0., 0., fm, 0.);
	f_t = spatial_adjoint(Xtrans(Vector3d (1., 0., 0.))) * f_t;
	cout << "f_t = " << f_t.transpose() << endl;
	
	ClearLogOutput();
	ComputeAccelerationDeltas (*model, contact_body_id, f_t, QDDot_t);

	cout << "qddot_t = " << QDDot_t.transpose() << endl;

	// compute the actual accelerations
	Vector3d point_accel_t;	
	point_accel_t = CalcPointAcceleration (*model, Q, QDot, QDDot_t, contact_body_id, contact_point);
	cout << "point_accel_t = " << point_accel_t.transpose() << endl;

	double C0 = point_accel_0[1];
	cout << "C0 = " << C0 << endl;

	double k_1 = point_accel_t[1];
	// k = f * (qdd_d - qdd_0) / (qdd_t - qdd_0)
	k_1 = (point_accel_0[1]) / (point_accel_t[1] - point_accel_0[1]);
	cout << "k_1 = " << k_1 << endl;

	// compute constrained acceleration
	SpatialVector f_ext = f_t * k_1 * -1.;
	model->f_ext[contact_body_id] = f_ext;

	Vector3d point_accel_c;
	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
	model->f_ext[contact_body_id].setZero();
	point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	cout << "point_accel_c = " << point_accel_c.transpose() << endl;

	cout << "NOOOOOOW" << endl;
	contact_data.push_back (ContactInfo(contact_body_id, contact_point, contact_normal, 0.));
	ClearLogOutput();
	ForwardDynamicsContacts (*model, Q, QDot, Tau, contact_data, QDDot);
	cout << LogOutput.str() << endl;

	point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	cout << "point_accel_c neeu = " << point_accel_c.transpose() << endl;

	delete model;
}
