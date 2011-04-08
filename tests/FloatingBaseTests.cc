#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"
#include "Dynamics.h"
#include "Dynamics_experimental.h"

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

	VectorNd q, qdot, qddot, tau;
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

	// Initialization of the input vectors
	VectorNd Q ((size_t) 0, 0.);
	VectorNd QDot ((size_t) 0, 0.);
	VectorNd QDDot ((size_t) 0, 0.);
	VectorNd Tau ((size_t) 0, 0.);

	Vector3d pos_B(0., 0., 0.);
	Vector3d rot_B(0., 0., 0.);

	SpatialMatrix X_B (XtransRotZYXEuler(pos_B, rot_B));
	SpatialVector v_B(0., 0., 0., 0., 0., 0.);
	SpatialVector f_B(0., 0., 0., 0., 0., 0.);
	SpatialVector a_B(0., 0., 0., 0., 0., 0.);

	ForwardDynamicsFloatingBaseExpl(*model, Q, QDot, Tau, X_B, v_B, f_B, a_B, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
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
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
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

	// Initialization of the input vectors
	VectorNd Q ((size_t) 1, 0.);
	VectorNd QDot ((size_t) 1, 0.);
	VectorNd QDDot ((size_t) 1, 0.);
	VectorNd Tau ((size_t) 1, 0.);

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
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
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
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
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
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
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

	// Initialization of the input vectors
	VectorNd Q ((size_t) model->dof_count, 0.);
	VectorNd QDot ((size_t) model->dof_count, 0.);
	VectorNd QDDot ((size_t) model->dof_count, 0.);
	VectorNd Tau ((size_t) model->dof_count, 0.);

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
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
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
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
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a.at(i) << endl;
	}

//	std::cout << LogOutput.str() << std::endl;

	CHECK_CLOSE ( 0.0000, QDDot[0], TEST_PREC);
	CHECK_CLOSE (-8.8100, QDDot[1], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[2], TEST_PREC);
	CHECK_CLOSE (-1.0000, QDDot[3], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[4], TEST_PREC);
	CHECK_CLOSE ( 0.0000, QDDot[5], TEST_PREC);
	CHECK_CLOSE ( 2.0000, QDDot[6], TEST_PREC);
}

TEST_FIXTURE(FloatingBaseFixture, TestCalcPointVelocityFloatingBaseSimple) {
	// floating base
	base_body_id = model->SetFloatingBaseBody(base);

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;

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

	point_velocity = CalcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

	CHECK_CLOSE(1., point_velocity[0], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[2], TEST_PREC);

	LOG << "Point velocity = " << point_velocity << endl;
//	cout << LogOutput.str() << endl;

	ClearLogOutput();

	// Now we calculate the velocity when rotating around the Z axis
	QDot[0] = 0.;
	QDot[3] = 1.;

	point_velocity = CalcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

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

	point_velocity = CalcPointVelocity(*model, Q, QDot, ref_body_id, point_position);

	CHECK_CLOSE(-1., point_velocity[0], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[2], TEST_PREC);

	LOG << "Point velocity = " << point_velocity << endl;
//	cout << LogOutput.str() << endl;
}

TEST_FIXTURE(FloatingBaseFixture, TestCalcPointVelocityCustom) {
	// floating base
	base = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));	
	base_body_id = model->SetFloatingBaseBody(base);

	q.resize (model->dof_count);
	qdot.resize (model->dof_count);
	qddot.resize (model->dof_count);
	tau.resize (model->dof_count);

	q.zero ();
	qdot.zero ();
	qddot.zero ();
	tau.zero ();

	unsigned int ref_body_id = base_body_id;

	q[0] = 0.1;
	q[1] = 1.1;
	q[2] = 1.2;
	q[3] = 1.3;
	q[4] = 1.5;
	q[5] = 1.7;

	qdot[0] = 0.1;
	qdot[1] = 1.1;
	qdot[2] = 1.2;
	qdot[3] = 1.3;
	qdot[4] = 1.5;
	qdot[5] = 1.7;

	// first we calculate the velocity when rotating around the Z axis
	Vector3d point_body_position (1., 0., 0.);
	Vector3d point_base_position;
	Vector3d point_base_velocity;
	Vector3d point_base_velocity_reference;

	ForwardDynamics(*model, q, qdot, tau, qddot);

	point_base_velocity = CalcPointVelocity (*model, q, qdot, ref_body_id, point_body_position);

	point_base_velocity_reference.set (
			 -3.888503432977729e-01,
			 -3.171179347202455e-01,
			 1.093894197498446e+00
			);

	CHECK_ARRAY_CLOSE (point_base_velocity_reference.data(), point_base_velocity.data(), 3, TEST_PREC);
}

/** \brief Compares computation of acceleration values for zero qddot
 *
 * Ensures that computation of position, velocity, and acceleration of a
 * point produce the same values as in an equivalent model that was
 * created with the HuMAnS toolbox
 *    http://www.inrialpes.fr/bipop/software/humans/ .
 * Here we omit the term of the generalized acceleration by setting qddot
 * to zero.
 */
TEST_FIXTURE(FloatingBaseFixture, TestCalcPointAccelerationNoQDDot) {
	// floating base
	base = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));	
	base_body_id = model->SetFloatingBaseBody(base);

	q.resize (model->dof_count);
	qdot.resize (model->dof_count);
	qddot.resize (model->dof_count);
	tau.resize (model->dof_count);

	q.zero ();
	qdot.zero ();
	qddot.zero ();
	tau.zero ();

	unsigned int ref_body_id = base_body_id;

	q[0] = 0.1;
	q[1] = 1.1;
	q[2] = 1.2;
	q[3] = 1.3;
	q[4] = 1.5;
	q[5] = 1.7;

	qdot[0] = 0.1;
	qdot[1] = 1.1;
	qdot[2] = 1.2;
	qdot[3] = 1.3;
	qdot[4] = 1.5;
	qdot[5] = 1.7;

	// first we calculate the velocity when rotating around the Z axis
	Vector3d point_body_position (-1.9, -1.8, 0.);
	Vector3d point_world_position;
	Vector3d point_world_velocity;
	Vector3d point_world_acceleration;

	// call ForwardDynamics to update the model
	ForwardDynamics(*model, q, qdot, tau, qddot);
	qddot.zero();

	qdot = qdot;

	point_world_position = model->CalcBodyToBaseCoordinates(ref_body_id, point_body_position);
	point_world_velocity = CalcPointVelocity (*model, q, qdot, ref_body_id, point_body_position);

	// we set the generalized acceleration to zero

	ClearLogOutput();

	point_world_acceleration = CalcPointAcceleration (*model, q, qdot, qddot, ref_body_id, point_body_position);

	Vector3d humans_point_position (
			-6.357089363622626e-01, -6.831041744630977e-01, 2.968974805916970e+00
			);
	Vector3d humans_point_velocity (
			 3.091226260907569e-01, 3.891012095550828e+00, 4.100277995030419e+00
			);
	Vector3d humans_point_acceleration (
			-5.302760158847160e+00, 6.541369639625232e+00, -4.795115077652286e+00
			);

//	cout << LogOutput.str() << endl;
//
//	cout << "q     = " << q << endl;
//	cout << "qdot  = " << qdot << endl;
//	cout << "qddot = " << qddot << endl;
//
//	cout << "body_coords = " << point_body_position << endl;
//	cout << "world_pos   = " << point_world_position << endl;
//	cout << "world_vel   = " << point_world_velocity << endl;
//	cout << "world_accel = " << point_world_acceleration << endl;


	CHECK_ARRAY_CLOSE (humans_point_position.data(), point_world_position.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (humans_point_velocity.data(), point_world_velocity.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (humans_point_acceleration.data(), point_world_acceleration.data(), 3, TEST_PREC);
}

/** \brief Compares computation of acceleration values for zero q and qdot 
 *
 * Ensures that computation of position, velocity, and acceleration of a
 * point produce the same values as in an equivalent model that was
 * created with the HuMAnS toolbox
 *    http://www.inrialpes.fr/bipop/software/humans/ .
 * 
 * Here we set q and qdot to zero and only take into account values that
 * are dependent on qddot.
 */
TEST_FIXTURE(FloatingBaseFixture, TestCalcPointAccelerationOnlyQDDot) {
	// floating base
	base = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));	
	base_body_id = model->SetFloatingBaseBody(base);

	q.resize (model->dof_count);
	qdot.resize (model->dof_count);
	qddot.resize (model->dof_count);
	tau.resize (model->dof_count);

	q.zero ();
	qdot.zero ();
	qddot.zero ();
	tau.zero ();

	unsigned int ref_body_id = base_body_id;

	// first we calculate the velocity when rotating around the Z axis
	Vector3d point_body_position (-1.9, -1.8, 0.);
	Vector3d point_world_position;
	Vector3d point_world_velocity;
	Vector3d point_world_acceleration;

	ForwardDynamics(*model, q, qdot, tau, qddot);

	qddot.zero();

	qddot[0] = 0.1;
	qddot[1] = 1.1;
	qddot[2] = 1.2;
	qddot[3] = 1.3;
	qddot[4] = 1.5;
	qddot[5] = 1.7;

//	cout << "ref_body_id = " << ref_body_id << endl;
//	cout << "point_body_position = " << point_body_position << endl;
	point_world_position = model->CalcBodyToBaseCoordinates(ref_body_id, point_body_position);
	point_world_velocity = CalcPointVelocity (*model, q, qdot, ref_body_id, point_body_position);

	ClearLogOutput();

	point_world_acceleration = CalcPointAcceleration (*model, q, qdot, qddot, ref_body_id, point_body_position);

	Vector3d humans_point_position (
			-1.900000000000000e+00, -1.800000000000000e+00, 0.000000000000000e+00
			);
	Vector3d humans_point_velocity (
			0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00
			);
	Vector3d humans_point_acceleration (
			2.440000000000000e+00, -1.370000000000000e+00, 9.899999999999999e-01
			);

//	cout << LogOutput.str() << endl;
//
//	cout << "q     = " << q << endl;
//	cout << "qdot  = " << qdot << endl;
//	cout << "qddot = " << qddot << endl;
//
//	cout << "body_coords = " << point_body_position << endl;
//	cout << "world_pos   = " << point_world_position << endl;
//	cout << "world_vel   = " << point_world_velocity << endl;
//	cout << "world_accel = " << point_world_acceleration << endl;

	CHECK_ARRAY_CLOSE (humans_point_position.data(), point_world_position.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (humans_point_velocity.data(), point_world_velocity.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (humans_point_acceleration.data(), point_world_acceleration.data(), 3, TEST_PREC);
}

/** \brief Compares computation of acceleration values for zero q and qdot 
 *
 * Ensures that computation of position, velocity, and acceleration of a
 * point produce the same values as in an equivalent model that was
 * created with the HuMAnS toolbox
 *    http://www.inrialpes.fr/bipop/software/humans/ .
 * 
 * Here we set q and qdot to zero and only take into account values that
 * are dependent on qddot.
 */
TEST_FIXTURE(FloatingBaseFixture, TestCalcPointAccelerationFull) {
	// floating base
	base = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));	
	base_body_id = model->SetFloatingBaseBody(base);

	q.resize (model->dof_count);
	qdot.resize (model->dof_count);
	qddot.resize (model->dof_count);
	tau.resize (model->dof_count);

	q.zero ();
	qdot.zero ();
	qddot.zero ();
	tau.zero ();

	unsigned int ref_body_id = base_body_id;

	// first we calculate the velocity when rotating around the Z axis
	Vector3d point_body_position (-1.9, -1.8, 0.);
	Vector3d point_world_position;
	Vector3d point_world_velocity;
	Vector3d point_world_acceleration;

	q[0] = 0.1;
	q[1] = 1.1;
	q[2] = 1.2;
	q[3] = 1.3;
	q[4] = 1.5;
	q[5] = 1.7;

	qdot[0] = 0.1;
	qdot[1] = 1.1;
	qdot[2] = 1.2;
	qdot[3] = 1.3;
	qdot[4] = 1.5;
	qdot[5] = 1.7;

	ForwardDynamics(*model, q, qdot, tau, qddot);

	qddot[0] = 0.1;
	qddot[1] = 1.1;
	qddot[2] = 1.2;
	qddot[3] = 1.3;
	qddot[4] = 1.5;
	qddot[5] = 1.7;

//	cout << "ref_body_id = " << ref_body_id << endl;
//	cout << "point_body_position = " << point_body_position << endl;
	point_world_position = model->CalcBodyToBaseCoordinates(ref_body_id, point_body_position);
	point_world_velocity = CalcPointVelocity (*model, q, qdot, ref_body_id, point_body_position);

	ClearLogOutput();

	point_world_acceleration = CalcPointAcceleration (*model, q, qdot, qddot, ref_body_id, point_body_position);

	Vector3d humans_point_position (
			-6.357089363622626e-01, -6.831041744630977e-01, 2.968974805916970e+00
			);
	Vector3d humans_point_velocity (
			3.091226260907569e-01, 3.891012095550828e+00, 4.100277995030419e+00	
			);
	Vector3d humans_point_acceleration (
			-4.993637532756404e+00, 1.043238173517606e+01, -6.948370826218673e-01
			);

//	cout << LogOutput.str() << endl;
//
//	cout << "q     = " << q << endl;
//	cout << "qdot  = " << qdot << endl;
//	cout << "qddot = " << qddot << endl;
//
//	cout << "body_coords = " << point_body_position << endl;
//	cout << "world_pos   = " << point_world_position << endl;
//	cout << "world_vel   = " << point_world_velocity << endl;
//	cout << "world_accel = " << point_world_acceleration << endl;

	CHECK_ARRAY_CLOSE (humans_point_position.data(), point_world_position.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (humans_point_velocity.data(), point_world_velocity.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (humans_point_acceleration.data(), point_world_acceleration.data(), 3, TEST_PREC);
}


