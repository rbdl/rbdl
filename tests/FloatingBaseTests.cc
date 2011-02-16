#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Dynamics_stdvec.h"

using namespace std;
using namespace SpatialAlgebra;

const double TEST_PREC = 1.0e-14;

struct FloatingBaseFixture {
	FloatingBaseFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();
		model->gravity.set (0., -9.81, 0.);
	}
	~FloatingBaseFixture () {
		delete model;
	}
	Model *model;
};

TEST_FIXTURE(FloatingBaseFixture, TestCalcDynamicFloatingBaseSimple) {
	model->floating_base = true;

	Body base(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

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

	int i;
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
	Body base(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
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

	int i;
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
	model->floating_base = true;

	// floating base
	Body base(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
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

	int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot.at(i) << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, QDDot[0], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[3], TEST_PREC);
	CHECK_CLOSE (-9.8100, QDDot[4], TEST_PREC);
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
	CHECK_CLOSE ( 0.0000, QDDot[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[3], TEST_PREC);
	CHECK_CLOSE (-9.8100, QDDot[4], TEST_PREC);
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
	CHECK_CLOSE ( 0.0000, QDDot[1], TEST_PREC);
	CHECK_CLOSE (-1.0000, QDDot[2], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[3], TEST_PREC);
	CHECK_CLOSE (-8.8100, QDDot[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[5], TEST_PREC);
	CHECK_CLOSE ( 2.0000, QDDot[6], TEST_PREC);
}
