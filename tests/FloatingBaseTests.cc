#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"
#include "Dynamics_stdvec.h"

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
};

TEST_FIXTURE(FloatingBaseFixture, TestCalcDynamicFloatingBaseSimple) {
	model->floating_base = true;

	model->SetFloatingBaseBody(base);

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
	model->floating_base = true;

	// floating base
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
	model->SetFloatingBaseBody(base);

	// body_a
	Body body_a (1., Vector3d (1., 0., 0), Vector3d (1., 1., 1.));
	Joint joint_a (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(0, Xtrans(Vector3d(2., 0., 0.)), joint_a, body_a);

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
	model->SetFloatingBaseBody(base);

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

	unsigned int ref_body_id = 0;

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
	model->SetFloatingBaseBody(base);

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

	unsigned int ref_body_id = 0;

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
	Q[0] = 0.;
	QDDot[4] = 1.;

	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	CHECK_CLOSE(-1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[2], TEST_PREC);

	LOG << "Point acceleration = " << point_acceleration << endl;
//	cout << LogOutput.str() << endl;
}


