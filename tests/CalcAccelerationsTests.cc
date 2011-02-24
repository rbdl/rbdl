#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics_stdvec.h"

using namespace std;
using namespace SpatialAlgebra;
using namespace RigidBodyDynamics;

const double TEST_PREC = 1.0e-14;

struct ModelAccelerationsFixture {
	ModelAccelerationsFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 *      Z
		 *      *---* 
		 *      |
		 *      |
		 *  Z   |
		 *  O---*
		 *      Y
		 */

		body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_a = Joint(
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

		body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
		joint_b = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);

		body_b_id = model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

		body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
		joint_c = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_c_id = model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q = std::vector<double> (3, 0.);
		QDot = std::vector<double> (3, 0.);
		QDDot = std::vector<double> (3, 0.);

		point_position.zero();
		point_acceleration.zero();

		ref_body_id = 0;

		ClearLogOutput();
	}
	~ModelAccelerationsFixture () {
		delete model;
	}
	Model *model;

	unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
	Body body_a, body_b, body_c;
	Joint joint_a, joint_b, joint_c;

	std::vector<double> Q;
	std::vector<double> QDot;
	std::vector<double> QDDot;

	Vector3d point_position, point_acceleration;
};

TEST_FIXTURE(ModelAccelerationsFixture, TestCalcPointSimple) {
	QDDot[0] = 1.;
	ref_body_id = body_a_id;
	point_position.set(1., 0., 0.);
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE(0., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(1., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(0., point_acceleration[2], TEST_PREC);

	// LOG << "Point accel = " << point_acceleration << endl;
}

TEST_FIXTURE(ModelAccelerationsFixture, TestCalcPointSimpleRotated) {
	Q[0] = 0.5 * M_PI;

	ref_body_id = body_a_id;
	QDDot[0] = 1.;
	point_position.set(1., 0., 0.);
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE(-1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);

	// LOG << "Point accel = " << point_acceleration << endl;
}

TEST_FIXTURE(ModelAccelerationsFixture, TestCalcPointRotation) {
	ref_body_id = 1;
	QDot[0] = 1.;
	point_position.set(1., 0., 0.);
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE(-1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);

	ClearLogOutput();

	// if we are on the other side we should have the opposite value
	point_position.set(-1., 0., 0.);
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE( 1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);
}

TEST_FIXTURE(ModelAccelerationsFixture, TestCalcPointRotatedBaseSimple) {
	// rotated first joint

	ref_body_id = 1;
	Q[0] = M_PI * 0.5;
	QDot[0] = 1.;
	point_position.set(1., 0., 0.);
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	CHECK_CLOSE( 0., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(-1., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);

	point_position.set(-1., 0., 0.);
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	CHECK_CLOSE( 0., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE( 1., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE( 0., point_acceleration[2], TEST_PREC);
//	cout << LogOutput.str() << endl;
}

TEST_FIXTURE(ModelAccelerationsFixture, TestCalcPointRotatingBodyB) {
	// rotating second joint, point at third body
	
	ref_body_id = 3;
	QDot[1] = 1.;
	point_position.set(1., 0., 0.); 
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE( -1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[2], TEST_PREC);

	// move it a bit further up (acceleration should stay the same)
	point_position.set(1., 1., 0.); 
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE( -1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[2], TEST_PREC);
}

TEST_FIXTURE(ModelAccelerationsFixture, TestCalcPointBodyOrigin) {
	// rotating second joint, point at third body
	
	QDot[0] = 1.;

	ref_body_id = body_b_id;
	point_position.set(0., 0., 0.); 
	CalcPointAcceleration(*model, Q, QDot, QDDot, ref_body_id, point_position, point_acceleration);

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE( -1., point_acceleration[0], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[1], TEST_PREC);
	CHECK_CLOSE(  0., point_acceleration[2], TEST_PREC);
}
