#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"

using namespace std;
using namespace SpatialAlgebra;

const double TEST_PREC = 1.0e-14;

struct KinematicsFixture {
	KinematicsFixture () {
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
		Tau = std::vector<double> (3, 0.);

		ClearLogOutput();
	}
	
	~KinematicsFixture () {
		delete model;
	}
	Model *model;

	unsigned int body_a_id, body_b_id, body_c_id;
	Body body_a, body_b, body_c;
	Joint joint_a, joint_b, joint_c;

	std::vector<double> Q;
	std::vector<double> QDot;
	std::vector<double> QDDot;
	std::vector<double> Tau;
};

TEST_FIXTURE(KinematicsFixture, TestPositionNeutral) {
	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	Vector3d body_position;

	body_position.set (0., 0., 0.);
	CHECK_EQUAL (body_position, model->GetBodyOrigin(body_a_id));

	body_position.set (1., 0., 0.);
	CHECK_EQUAL (body_position, model->GetBodyOrigin(body_b_id));

	body_position.set (1., 1., 0.);
	CHECK_EQUAL (body_position, model->GetBodyOrigin(body_c_id));
}

TEST_FIXTURE(KinematicsFixture, TestPositionBaseRotated) {
	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices

	Q[0] = 0.5 * M_PI;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	Vector3d body_position;

	cout << LogOutput.str() << endl;
	CHECK_EQUAL (Vector3d (0., 0., 0.), model->GetBodyOrigin(body_a_id) );
	CHECK_EQUAL (Vector3d (0., 1., 0.), model->GetBodyOrigin(body_b_id) );
	CHECK_EQUAL (Vector3d (-1., 1., 0.),model->GetBodyOrigin(body_c_id) );

	cout << "X_base[body_b] = " << model->X_base[body_b_id] << endl;
	cout << "gettranslation = " << model->X_base[body_b_id].get_translation() << endl;
}
