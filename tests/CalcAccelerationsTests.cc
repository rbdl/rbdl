#include <UnitTest++.h>

#include <iostream>

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"

#include "Fixtures.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

TEST_FIXTURE(FixedBase3DoF, TestCalcPointSimple) {
	QDDot[0] = 1.;
	ref_body_id = body_a_id;
	point_position = Vector3d (1., 0., 0.);
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE(0., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(1., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[2], TEST_PREC);

	// LOG << "Point accel = " << point_acceleration << endl;
}

TEST_FIXTURE(FixedBase3DoF, TestCalcPointSimpleRotated) {
	Q[0] = 0.5 * M_PI;

	ref_body_id = body_a_id;
	QDDot[0] = 1.;
	point_position = Vector3d (1., 0., 0.);
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE(-1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);

	// LOG << "Point accel = " << point_acceleration << endl;
}

TEST_FIXTURE(FixedBase3DoF, TestCalcPointRotation) {
	ref_body_id = 1;
	QDot[0] = 1.;
	point_position = Vector3d (1., 0., 0.);
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE(-1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);

	ClearLogOutput();

	// if we are on the other side we should have the opposite value
	point_position = Vector3d (-1., 0., 0.);
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE( 1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);
}

TEST_FIXTURE(FixedBase3DoF, TestCalcPointRotatedBaseSimple) {
	// rotated first joint

	ref_body_id = 1;
	Q[0] = M_PI * 0.5;
	QDot[0] = 1.;
	point_position = Vector3d (1., 0., 0.);
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

	CHECK_CLOSE( 0., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(-1., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);

	point_position = Vector3d (-1., 0., 0.);
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

	CHECK_CLOSE( 0., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE( 1., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);
//	cout << LogOutput.str() << endl;
}

TEST_FIXTURE(FixedBase3DoF, TestCalcPointRotatingBodyB) {
	// rotating second joint, point at third body
	
	ref_body_id = 3;
	QDot[1] = 1.;
	point_position = Vector3d (1., 0., 0.); 
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE( -1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[2], TEST_PREC);

	// move it a bit further up (acceleration should stay the same)
	point_position = Vector3d (1., 1., 0.); 
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE( -1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[2], TEST_PREC);
}

TEST_FIXTURE(FixedBase3DoF, TestCalcPointBodyOrigin) {
	// rotating second joint, point at third body
	
	QDot[0] = 1.;

	ref_body_id = body_b_id;
	point_position = Vector3d (0., 0., 0.); 
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position);

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE( -1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[2], TEST_PREC);
}

TEST_FIXTURE(FixedBase3DoF, TestAccelerationLinearFuncOfQddot) {
	// rotating second joint, point at third body
	
	QDot[0] = 1.1;
	QDot[1] = 1.3;
	QDot[2] = 1.5;

	ref_body_id = body_c_id;
	point_position = Vector3d (1., 1., 1.); 

	VectorNd qddot_1 = VectorNd::Zero (model->dof_count);
	VectorNd qddot_2 = VectorNd::Zero (model->dof_count);
	VectorNd qddot_0 = VectorNd::Zero (model->dof_count);

	qddot_1[0] = 0.1;
	qddot_1[1] = 0.2;
	qddot_1[2] = 0.3;

	qddot_2[0] = 0.32;
	qddot_2[1] = -0.1;
	qddot_2[2] = 0.53;

	Vector3d acc_1 = CalcPointAcceleration(*model, Q, QDot, qddot_1, ref_body_id, point_position);
	Vector3d acc_2 = CalcPointAcceleration(*model, Q, QDot, qddot_2, ref_body_id, point_position);

	MatrixNd G = MatrixNd::Zero (3, model->dof_count);
	CalcPointJacobian (*model, Q, ref_body_id, point_position, G, true);

	VectorNd net_acc = G * (qddot_1 - qddot_2);

	Vector3d acc_new = acc_1 - acc_2;

	CHECK_ARRAY_CLOSE (net_acc.data(), acc_new.data(), 3, TEST_PREC);
}

TEST_FIXTURE (FloatingBase12DoF, TestAccelerationFloatingBaseWithUpdateKinematics ) {
	ForwardDynamics (*model, Q, QDot, Tau, QDDot);

	ClearLogOutput();
	Vector3d accel = CalcPointAcceleration (*model, Q, QDot, QDDot, child_2_rot_x_id, Vector3d (0., 0., 0.), true);

	CHECK_ARRAY_CLOSE (Vector3d (0., -9.81, 0.), accel.data(), 3, TEST_PREC);
}

TEST_FIXTURE (FloatingBase12DoF, TestAccelerationFloatingBaseWithoutUpdateKinematics ) {
	ForwardDynamics (*model, Q, QDot, Tau, QDDot);

	//ClearLogOutput();
	Vector3d accel = CalcPointAcceleration (*model, Q, QDot, QDDot, child_2_rot_x_id, Vector3d (0., 0., 0.), false);

	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.), accel.data(), 3, TEST_PREC);
//	cout << LogOutput.str() << endl;
//	cout << accel.transpose() << endl;
}

TEST_FIXTURE(FixedBase3DoF, TestCalcPointRotationFixedJoint) {
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	unsigned int fixed_body_id = model->AddBody (body_c_id, Xtrans (Vector3d (1., -1., 0.)), Joint(JointTypeFixed), fixed_body, "fixed_body");

	QDot[0] = 1.;
	point_position = Vector3d (0., 0., 0.);
	Vector3d point_acceleration_reference = CalcPointAcceleration (*model, Q, QDot, QDDot, body_c_id, Vector3d (1., -1., 0.));

	ClearLogOutput();
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, fixed_body_id, point_position);
//	cout << LogOutput.str() << endl;

	CHECK_ARRAY_CLOSE (point_acceleration_reference.data(),
			point_acceleration.data(),
			3, 
			TEST_PREC);
}

TEST_FIXTURE(FixedBase3DoF, TestCalcPointRotationFixedJointRotatedTransform) {
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	SpatialTransform fixed_transform = Xtrans (Vector3d (1., -1., 0.)) * Xrotz(M_PI * 0.5);
	unsigned int fixed_body_id = model->AddBody (body_c_id, fixed_transform, Joint(JointTypeFixed), fixed_body, "fixed_body");

	QDot[0] = 1.;
	point_position = Vector3d (0., 0., 0.);
	ClearLogOutput();
	Vector3d point_acceleration_reference = CalcPointAcceleration (*model, Q, QDot, QDDot, body_c_id, Vector3d (1., 1., 0.));
	// cout << LogOutput.str() << endl;

	// cout << "Point position = " << CalcBodyToBaseCoordinates (*model, Q, fixed_body_id, Vector3d (0., 0., 0.)).transpose() << endl;
	// cout << "Point position_ref = " << CalcBodyToBaseCoordinates (*model, Q, body_c_id, Vector3d (1., 1., 0.)).transpose() << endl;

	ClearLogOutput();
	point_acceleration = CalcPointAcceleration(*model, Q, QDot, QDDot, fixed_body_id, point_position);
	// cout << LogOutput.str() << endl;

	CHECK_ARRAY_CLOSE (point_acceleration_reference.data(),
			point_acceleration.data(),
			3, 
			TEST_PREC);
}

