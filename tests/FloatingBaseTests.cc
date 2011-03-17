#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"
#include "Dynamics_stdvec.h"
#include "Dynamics.h"

using namespace std;
using namespace SpatialAlgebra;
using namespace RigidBodyDynamics;

const double TEST_PREC = 1.0e-14;

struct FloatingBaseFixture {
	FloatingBaseFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();
		model->gravity.set (0., -9.81, 0.);

		base = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

	}
	~FloatingBaseFixture () {
		delete model;
	}
	Model *model;
	Body base;
	unsigned int base_body_id;

	cmlVector q, qdot, qddot, tau;
};

TEST_FIXTURE ( FloatingBaseFixture, TestCalcPointTransformation ) {
	base_body_id = model->SetFloatingBaseBody(base);

	q.resize(model->dof_count);
	qdot.resize(model->dof_count);
	qddot.resize(model->dof_count);
	tau.resize(model->dof_count);

	q.zero();
	qdot.zero();
	qddot.zero();
	tau.zero();

	q[1] = 1.;
	ForwardDynamics (*model, q, qdot, tau, qddot);

	Vector3d test_point;

	test_point = model->CalcBaseToBodyCoordinates(base_body_id, Vector3d (0., 0., 0.));
	CHECK_ARRAY_CLOSE (Vector3d (0., -1., 0.).data(), test_point.data(), 3, TEST_PREC);
}

TEST_FIXTURE(FloatingBaseFixture, TestCalcDynamicFloatingBaseSimple) {
	model->experimental_floating_base = true;
	base_body_id = model->SetFloatingBaseBody(base);
	CHECK_EQUAL (0u, base_body_id);

	std::vector<double> Q (0, 0.);
	std::vector<double> QDot (0, 0.);
	std::vector<double> QDDot (0, 0.);
	std::vector<double> Tau (0, 0.);

	Vector3d pos_B(0., 0., 0.);
	Vector3d rot_B(0., 0., 0.);

	SpatialMatrix X_B (XtransRotZYXEuler(pos_B, rot_B));
	SpatialVector v_B(0., 0., 0., 0., 0., 0.);
	SpatialVector f_B(0., 0., 0., 0., 0., 0.);
	SpatialVector a_B(0., 0., 0., 0., 0., 0.);

	ForwardDynamicsFloatingBaseExpl(*model, Q, QDot, Tau, X_B, v_B, f_B, a_B, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, a_B[0], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_B[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_B[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_B[3], TEST_PREC);
	CHECK_CLOSE (-9.8100, a_B[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_B[5], TEST_PREC);

	// We rotate the base... let's see what happens...
	rot_B[0] = 0.8;
	X_B = XtransRotZYXEuler(pos_B, rot_B);
	ForwardDynamicsFloatingBaseExpl (*model, Q, QDot, Tau, X_B, v_B, f_B, a_B, QDDot);
	SpatialVector a_world = X_B.inverse() * a_B;

	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, a_world[0], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[3], TEST_PREC);
	CHECK_CLOSE (-9.8100, a_world[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[5], TEST_PREC);
}

TEST_FIXTURE(FloatingBaseFixture, TestCalcDynamicFloatingBaseDouble) {
	// floating base
	model->experimental_floating_base = true;
	model->SetFloatingBaseBody(base);

	// body_a
	Body body_a (1., Vector3d (1., 0., 0), Vector3d (1., 1., 1.));
	Joint joint_a (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(0, Xtrans(Vector3d(2., 0., 0.)), joint_a, body_a);

	std::vector<double> Q (1, 0.);
	std::vector<double> QDot (1, 0.);
	std::vector<double> QDDot (1, 0.);
	std::vector<double> Tau (1, 0.);

	Vector3d pos_B(0., 0., 0.);
	Vector3d rot_B(0., 0., 0.);

	SpatialMatrix X_B (XtransRotZYXEuler(pos_B, rot_B));
	SpatialVector v_B(0., 0., 0., 0., 0., 0.);
	SpatialVector f_B(0., 0., 0., 0., 0., 0.);
	SpatialVector a_B(0., 0., 0., 0., 0., 0.);
	SpatialVector a_world(0., 0., 0., 0., 0., 0.);

	ForwardDynamicsFloatingBaseExpl(*model, Q, QDot, Tau, X_B, v_B, f_B, a_B, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, a_B[0], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_B[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_B[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_B[3], TEST_PREC);
	CHECK_CLOSE (-9.8100, a_B[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_B[5], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[0], TEST_PREC);

	// We rotate the base... let's see what happens...
	rot_B[0] = 0.8;
	X_B = XtransRotZYXEuler(pos_B, rot_B);

	ForwardDynamicsFloatingBaseExpl (*model, Q, QDot, Tau, X_B, v_B, f_B, a_B, QDDot);
	a_world = X_B.inverse() * a_B;

	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, a_world[0], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[3], TEST_PREC);
	CHECK_CLOSE (-9.8100, a_world[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[5], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[0], TEST_PREC);

	// We apply a torqe let's see what happens...
	rot_B[0] = 0.0;
	X_B = XtransRotZYXEuler(pos_B, rot_B);

	Tau[0] = 1.;

	ForwardDynamicsFloatingBaseExpl (*model, Q, QDot, Tau, X_B, v_B, f_B, a_B, QDDot);
	a_world = X_B.inverse() * a_B;

	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;


	CHECK_CLOSE ( 0.0000, a_world[0], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[1], TEST_PREC);
	CHECK_CLOSE (-1.0000, a_world[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[3], TEST_PREC);
	CHECK_CLOSE (-8.8100, a_world[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, a_world[5], TEST_PREC);
	CHECK_CLOSE ( 2.0000, QDDot[0],   TEST_PREC);
}

TEST_FIXTURE(FloatingBaseFixture, TestCalcDynamicFloatingBaseDoubleImplicit) {
	// floating base
	base_body_id = model->SetFloatingBaseBody(base);

	// body_a
	Body body_a (1., Vector3d (1., 0., 0), Vector3d (1., 1., 1.));
	Joint joint_a (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(base_body_id, Xtrans(Vector3d(2., 0., 0.)), joint_a, body_a);

	std::vector<double> Q (7, 0.);
	std::vector<double> QDot (7, 0.);
	std::vector<double> QDDot (7, 0.);
	std::vector<double> Tau (7, 0.);

	Vector3d pos_B(0., 0., 0.);
	Vector3d rot_B(0., 0., 0.);

	SpatialMatrix X_B (XtransRotZYXEuler(pos_B, rot_B));
	SpatialVector v_B(0., 0., 0., 0., 0., 0.);
	SpatialVector f_B(0., 0., 0., 0., 0., 0.);
	SpatialVector a_B(0., 0., 0., 0., 0., 0.);
	SpatialVector a_world(0., 0., 0., 0., 0., 0.);

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, QDDot[0], TEST_PREC);
	CHECK_CLOSE (-9.8100, QDDot[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[3], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[5], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[6], TEST_PREC);

	// We rotate the base... let's see what happens...
	Q[3] = 0.8;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, QDDot[0], TEST_PREC);
	CHECK_CLOSE (-9.8100, QDDot[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[3], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[5], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[6], TEST_PREC);

	// We apply a torqe let's see what happens...
	Q[3] = 0.;
/*
	rot_B[0] = 0.0;
	X_B = XtransRotZYXEuler(pos_B, rot_B);
	*/

	Tau[6] = 1.;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, QDDot[0], TEST_PREC);
	CHECK_CLOSE (-8.8100, QDDot[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[3], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[4], TEST_PREC);
	CHECK_CLOSE (-1.0000, QDDot[5], TEST_PREC);
	CHECK_CLOSE ( 2.0000, QDDot[6], TEST_PREC);
}

TEST_FIXTURE(FloatingBaseFixture, TestCalcPointVelocityFloatingBaseSimple) {
	// floating base
	base_body_id = model->SetFloatingBaseBody(base);

	cmlVector Q;
	cmlVector QDot;
	cmlVector QDDot;
	cmlVector Tau;

	Q.resize(6);
	QDot.resize(6);
	QDDot.resize(6);
	Tau.resize(6);

	Q.zero();
	QDot.zero();
	QDDot.zero();
	Tau.zero();

	unsigned int ref_body_id = base_body_id;

	// first we calculate the velocity when moving along the X axis
	QDot[0] = 1.;
	Vector3d point_position(1., 0., 0.);
	Vector3d point_velocity;

	CalcPointVelocity(*model, Q, QDot, ref_body_id, point_position, point_velocity);

	CHECK_CLOSE(1., point_velocity[0], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[2], TEST_PREC);

	LOG << "Point velocity = " << point_velocity << endl;
//	cout << LogOutput.str() << endl;

	ClearLogOutput();

	// Now we calculate the velocity when rotating around the Z axis
	QDot[0] = 0.;
	QDot[3] = 1.;

	CalcPointVelocity(*model, Q, QDot, ref_body_id, point_position, point_velocity);

	CHECK_CLOSE(0., point_velocity[0], TEST_PREC);
	CHECK_CLOSE(1., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[2], TEST_PREC);

	LOG << "Point velocity = " << point_velocity << endl;
//	cout << LogOutput.str() << endl;
	
	// Now we calculate the velocity when rotating around the Z axis and the
	// base is rotated around the z axis by 90 degrees 
	ClearLogOutput();
	Q[3] = M_PI * 0.5;
	QDot[3] = 1.;

	CalcPointVelocity(*model, Q, QDot, ref_body_id, point_position, point_velocity);

	CHECK_CLOSE(-1., point_velocity[0], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[2], TEST_PREC);

	LOG << "Point velocity = " << point_velocity << endl;
//	cout << LogOutput.str() << endl;
}

TEST_FIXTURE(FloatingBaseFixture, TestCalcPointAccelerationFloatingBaseSimple) {
	// floating base
	base_body_id = model->SetFloatingBaseBody(base);

	cmlVector Q;
	cmlVector QDot;
	cmlVector QDDot;
	cmlVector Tau;

	Q.resize(6);
	QDot.resize(6);
	QDDot.resize(6);
	Tau.resize(6);

	Q.zero();
	QDot.zero();
	QDDot.zero();
	Tau.zero();

	unsigned int ref_body_id = base_body_id;

	// first we calculate the velocity when moving along the X axis
	QDDot[0] = 1.;
	Vector3d point_position(1., 0., 0.);
	Vector3d point_acceleration;

	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	CHECK_CLOSE(1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[2], TEST_PREC);

	LOG << "Point acceleration = " << point_acceleration << endl;
//	cout << LogOutput.str() << endl;

	ClearLogOutput();

	// Now we calculate the acceleration when rotating around the Z axis
	QDDot[0] = 0.;
	QDDot[3] = 1.;

	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	CHECK_CLOSE(0., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(1., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[2], TEST_PREC);

	LOG << "Point acceleration = " << point_acceleration << endl;
//	cout << LogOutput.str() << endl;
	
	// Now we calculate the acceleration when rotating around the Z axis and the
	// base is rotated around the z axis by 90 degrees 
	ClearLogOutput();
	Q[3] = M_PI * 0.5;
	QDDot[3] = 1.;

	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	CHECK_CLOSE(-1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[2], TEST_PREC);

	LOG << "Point acceleration = " << point_acceleration << endl;
//	cout << LogOutput.str() << endl;

	// Now there is only a rotation around the Y axis so the point should
	// accelerate towards the center
	ClearLogOutput();
	Q.zero();
	QDot.zero();
	QDDot.zero();

	QDDot[4] = 1.;

	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	CHECK_CLOSE(0., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(-1., point_acceleration[2], TEST_PREC);

	LOG << "Point acceleration = " << point_acceleration << endl;
//	cout << LogOutput.str() << endl;
}

TEST_FIXTURE(FloatingBaseFixture, TestCalcPointAccelerationFloatingBaseRotation) {
	// floating base
	base_body_id = model->SetFloatingBaseBody(base);

	cmlVector Q;
	cmlVector QDot;
	cmlVector QDDot;
	cmlVector Tau;

	Q.resize(6);
	QDot.resize(6);
	QDDot.resize(6);
	Tau.resize(6);

	Q.zero();
	QDot.zero();
	QDDot.zero();
	Tau.zero();

	unsigned int ref_body_id = base_body_id;

	// first we calculate the velocity when rotating around the Z axis
	QDot[3] = 1.;

	Vector3d point_position(0., -1., 0.);
	Vector3d point_acceleration;
	Vector3d expected_acceleration (0., 1., 0.);
	Vector3d point_velocity;
	Vector3d expected_velocity;

	expected_velocity.set (1., 0., 0.);
	CalcPointVelocity (*model, Q, QDot, ref_body_id, point_position, point_velocity);
	CHECK_ARRAY_CLOSE (expected_velocity.data(), point_velocity.data(), 3, TEST_PREC);

	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);
	LOG << "Point acceleration = " << point_acceleration << endl;

	CHECK_ARRAY_CLOSE (expected_acceleration.data(), point_acceleration.data(), 3, TEST_PREC);

//	cout << LogOutput.str() << endl;

	ClearLogOutput();

	//  check for the point on the upper side
	point_position.set (0., 1., 0.);
	expected_acceleration.set (0., -1., 0.);

	expected_velocity.set (-1., 0., 0.);
	CalcPointVelocity (*model, Q, QDot, ref_body_id, point_position, point_velocity);
	CHECK_ARRAY_CLOSE (expected_velocity.data(), point_velocity.data(), 3, TEST_PREC);

	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);
	LOG << "Point acceleration = " << point_acceleration << endl;

	CHECK_ARRAY_CLOSE (expected_acceleration.data(), point_acceleration.data(), 3, TEST_PREC);

//	cout << LogOutput.str() << endl;
}

/*
TEST_FIXTURE(FloatingBaseFixture, TestDynamicsManualFloatBase) {
	// floating base
	base = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
	model->experimental_floating_base = true;
	model->SetFloatingBaseBody(base);

	cmlVector Q;
	cmlVector QDot;
	cmlVector QDDot;
	cmlVector QDDot_manual;
	cmlVector Tau;

	Q.resize(6);
	QDot.resize(6);
	QDDot.resize(6);
	QDDot_manual.resize(6);
	Tau.resize(6);

	Q.zero();
	QDot.zero();
	QDDot.zero();
	QDDot_manual.zero();
	Tau.zero();

	// build the manual model
	Model *float_model_manual = new Model;
	float_model_manual->Init();
	float_model_manual->gravity.set (0., -9.81, 0.);

	// Order: tx ty tz rz ry rx
	unsigned body_tx_id;
	Body body_tx (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_tx (JointTypePrismatic, Vector3d (1., 0., 0.));
	body_tx_id = float_model_manual->AddBody(0, Xtrans (Vector3d (0., 0., 0.)), joint_tx, body_tx);

	unsigned body_ty_id;
	Body body_ty (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_ty (JointTypePrismatic, Vector3d (0., 1., 0.));
	body_ty_id = float_model_manual->AddBody(body_tx_id, Xtrans (Vector3d (0., 0., 0.)), joint_ty, body_ty);

	unsigned body_tz_id;
	Body body_tz (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_tz (JointTypePrismatic, Vector3d (0., 0., 1.));
	body_tz_id = float_model_manual->AddBody(body_ty_id, Xtrans (Vector3d (0., 0., 0.)), joint_tz, body_tz);

	unsigned body_rz_id;
	Body body_rz (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_rz (JointTypeRevolute, Vector3d (0., 0., 1.));
	body_rz_id = float_model_manual->AddBody(body_tz_id, Xtrans (Vector3d (0., 0., 0.)), joint_rz, body_rz);

	unsigned body_ry_id;
	Body body_ry (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_ry (JointTypeRevolute, Vector3d (0., 1., 0.));
	body_ry_id = float_model_manual->AddBody(body_rz_id, Xtrans (Vector3d (0., 0., 0.)), joint_ry, body_ry);

	unsigned body_rx_id;
	Body body_rx (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
	Joint joint_rx (JointTypeRevolute, Vector3d (1., 0., 0.));
	body_rx_id = float_model_manual->AddBody(body_ry_id, Xtrans (Vector3d (0., 0., 0.)), joint_rx, body_rx);

	Q[0] = 0.;
	Q[1] = 1.;
	Q[2] = 0.;
	QDot[0] = 1.;
	QDot[3] = -1.;

	cout << "--------------- manual -------------" << endl;
	ForwardDynamics	(*float_model_manual, Q, QDot, Tau, QDDot_manual);
	cout << "IA[6] = " << float_model_manual->IA[6] << endl;
	cout << "pA[6] = " << float_model_manual->pA[6] << endl;
//	cout << LogOutput.str() << endl;
	cout << "manual qddot = " << QDDot_manual << endl;

	ClearLogOutput();
	cout << "---------------- explicit -----------------" << endl;
	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
	cout << LogOutput.str() << endl;
	cout << "explicit qddot = " << QDDot << endl;

	delete float_model_manual;
}
*/

/*
TEST_FIXTURE(FloatingBaseFixture, TestDynamicsManualFloatBaseOtherOrder) {
	cmlVector Q;
	cmlVector QDot;
	cmlVector QDDot;
	cmlVector QDDot_manual;
	cmlVector Tau;

	Q.resize(6);
	QDot.resize(6);
	QDDot.resize(6);
	QDDot_manual.resize(6);
	Tau.resize(6);

	Q.zero();
	QDot.zero();
	QDDot.zero();
	QDDot_manual.zero();
	Tau.zero();

	// build the manual model
	Model *float_model_manual = new Model;
	float_model_manual->Init();
	float_model_manual->gravity.set (0., -9.81, 0.);

	// Order: rx ry rz tx ty tz
	unsigned body_rx_id;
	Body body_rx (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_rx (JointTypeRevolute, Vector3d (1., 0., 0.));
	body_rx_id = float_model_manual->AddBody(0, Xtrans (Vector3d (0., 0., 0.)), joint_rx, body_rx);

	unsigned body_ry_id;
	Body body_ry (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_ry (JointTypeRevolute, Vector3d (0., 1., 0.));
	body_ry_id = float_model_manual->AddBody(body_rx_id, Xtrans (Vector3d (0., 0., 0.)), joint_ry, body_ry);

	unsigned body_rz_id;
	Body body_rz (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_rz (JointTypeRevolute, Vector3d (0., 0., 1.));
	body_rz_id = float_model_manual->AddBody(body_ry_id, Xtrans (Vector3d (0., 0., 0.)), joint_rz, body_rz);

	unsigned body_tx_id;
	Body body_tx (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_tx (JointTypePrismatic, Vector3d (1., 0., 0.));
	body_tx_id = float_model_manual->AddBody(body_rz_id, Xtrans (Vector3d (0., 0., 0.)), joint_tx, body_tx);

	unsigned body_ty_id;
	Body body_ty (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));
	Joint joint_ty (JointTypePrismatic, Vector3d (0., 1., 0.));
	body_ty_id = float_model_manual->AddBody(body_tx_id, Xtrans (Vector3d (0., 0., 0.)), joint_ty, body_ty);

	unsigned body_tz_id;
	Body body_tz (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
	Joint joint_tz (JointTypePrismatic, Vector3d (0., 0., 1.));
	body_tz_id = float_model_manual->AddBody(body_ty_id, Xtrans (Vector3d (0., 0., 0.)), joint_tz, body_tz);

	Q[0] = 0.;
	Q[1] = 0.;
	Q[2] = 0.;
	Q[3] = 0.;
	Q[4] = 1.;
	Q[5] = 0.;
	QDot[0] = -1.;
	QDot[3] = 1.;

	cout << "--------------- manual other order -------------" << endl;
	ForwardDynamics	(*float_model_manual, Q, QDot, Tau, QDDot_manual);
	cout << "IA[6] = " << float_model_manual->IA[6] << endl;
	cout << "pA[6] = " << float_model_manual->pA[6] << endl;
//	cout << LogOutput.str() << endl;
	cout << "manual other order qddot = " << QDDot_manual << endl;

	delete float_model_manual;
}
*/

/*
TEST_FIXTURE(FloatingBaseFixture, TestDynamicsPointAcceleration) {
	// floating base
	base = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
	model->SetFloatingBaseBody(base);

	cmlVector Q;
	cmlVector QDot;
	cmlVector QDDot;
	cmlVector expected_qddot;
	cmlVector Tau;

	Q.resize(6);
	QDot.resize(6);
	QDDot.resize(6);
	expected_qddot.resize(6);
	Tau.resize(6);

	Q.zero();
	QDot.zero();
	QDDot.zero();
	Tau.zero();

	unsigned int ref_body_id = 0;

	Vector3d point_position(0., -1., 0.);

	Vector3d point_velocity;
	Vector3d expected_velocity;

	Vector3d point_acceleration;
	Vector3d expected_acceleration;

	// set the state
	Q[0] = 0.2;
	Q[1] = 1.1; // translation to (0., 1., 0.)
	Q[2] = -0.4;

	QDot[0] = 1.; // moving along the x-axis
	QDot[3] = -1.; // rotating cw around the z-axis

	CalcPointVelocity (*model, Q, QDot, ref_body_id, point_position, point_velocity);

	expected_velocity.set (0., 0., 0.);
	CHECK_ARRAY_CLOSE (expected_velocity.data(), point_velocity.data(), 3, TEST_PREC);

	ClearLogOutput();

	cout << "sd = " << Q << " " << QDot << endl;

	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
	expected_qddot[0] = 0.;
	expected_qddot[1] = -8.81;
	expected_qddot[2] = 0.;
	expected_qddot[3] = 0.;
	expected_qddot[4] = 0.;
	expected_qddot[5] = 0.;

	cout << LogOutput.str() << endl;

	CHECK_ARRAY_CLOSE (expected_qddot.data(), QDDot.data(), 6, TEST_PREC);

//	cout << LogOutput.str() << endl;

	ClearLogOutput();
}
*/
