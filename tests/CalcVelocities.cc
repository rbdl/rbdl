#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Body.h"
#include "ArticulatedFigure.h"
#include "Logging.h"

using namespace std;
using namespace SpatialAlgebra;

const double TEST_PREC = 1.0e-14;

struct ModelVelocitiesFixture {
	ModelVelocitiesFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();

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

		body_c = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_c = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);

		body_c_id = model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q = std::vector<double> (3, 0.);
		QDot = std::vector<double> (3, 0.);
		QDDot = std::vector<double> (3, 0.);
		Tau = std::vector<double> (3, 0.);

		point_position.zero();
		point_velocity.zero();

		ref_body_id = 0;

		ClearLogOutput();
	}
	~ModelVelocitiesFixture () {
		delete model;
	}
	Model *model;

	unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
	Body body_a, body_b, body_c;
	Joint joint_a, joint_b, joint_c;

	std::vector<double> Q;
	std::vector<double> QDot;
	std::vector<double> QDDot;
	std::vector<double> Tau;

	Vector3d point_position, point_velocity;
};

TEST_FIXTURE(ModelVelocitiesFixture, TestCalcPointSimple) {
	ref_body_id = 1;
	QDot[0] = 1.;
	point_position.set(1., 0., 0.);
	CalcPointVelocity(*model, Q, QDot, Tau, QDDot, ref_body_id, point_position, point_velocity);

	CHECK_CLOSE(0., point_velocity[0], TEST_PREC);
	CHECK_CLOSE(1., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(0., point_velocity[2], TEST_PREC);

	LOG << "Point velocity = " << point_velocity << endl;
//	cout << LogOutput.str() << endl;
}

TEST_FIXTURE(ModelVelocitiesFixture, TestCalcPointRotatedBaseSimple) {
	// rotated first joint

	ref_body_id = 1;
	Q[0] = M_PI * 0.5;
	QDot[0] = 1.;
	point_position.set(1., 0., 0.);
	CalcPointVelocity(*model, Q, QDot, Tau, QDDot, ref_body_id, point_position, point_velocity);

	CHECK_CLOSE(-1., point_velocity[0], TEST_PREC);
	CHECK_CLOSE( 0., point_velocity[1], TEST_PREC);
	CHECK_CLOSE( 0., point_velocity[2], TEST_PREC);

//	cout << LogOutput.str() << endl;
}

TEST_FIXTURE(ModelVelocitiesFixture, TestCalcPointRotatingBodyB) {
	// rotating second joint, point at third body
	
	ref_body_id = 3;
	QDot[1] = 1.;
	point_position.set(1., 0., 0.);
	CalcPointVelocity(*model, Q, QDot, Tau, QDDot, ref_body_id, point_position, point_velocity);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE( 0., point_velocity[0], TEST_PREC);
	CHECK_CLOSE( 0., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(-1., point_velocity[2], TEST_PREC);
}

TEST_FIXTURE(ModelVelocitiesFixture, TestCalcPointRotatingBaseXAxis) {
	// also rotate the first joint and take a point that is
	// on the X direction

	ref_body_id = 3;
	QDot[0] = 1.;
	QDot[1] = 1.;
	point_position.set(1., -1., 0.);
	CalcPointVelocity(*model, Q, QDot, Tau, QDDot, ref_body_id, point_position, point_velocity);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE( 0., point_velocity[0], TEST_PREC);
	CHECK_CLOSE( 2., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(-1., point_velocity[2], TEST_PREC);
}

TEST_FIXTURE(ModelVelocitiesFixture, TestCalcPointRotatedBaseXAxis) {
	// perform the previous test with the first joint rotated by pi/2
	// upwards 
	ClearLogOutput();

	ref_body_id = 3;
	point_position.set(1., -1., 0.);

	Q[0] = M_PI * 0.5;
	QDot[0] = 1.;
	QDot[1] = 1.;
	CalcPointVelocity(*model, Q, QDot, Tau, QDDot, ref_body_id, point_position, point_velocity);

//	cout << LogOutput.str() << endl;

	CHECK_CLOSE(-2., point_velocity[0], TEST_PREC);
	CHECK_CLOSE( 0., point_velocity[1], TEST_PREC);
	CHECK_CLOSE(-1., point_velocity[2], TEST_PREC);
}
