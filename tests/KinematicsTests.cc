#include <UnitTest++.h>

#include <iostream>

#include "rbdl/rbdl_mathutils.h"
#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Kinematics.h"
#include "rbdl/Dynamics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-12;

struct KinematicsFixture {
	KinematicsFixture () {
		ClearLogOutput();
		model = new Model;

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

		body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_a = Joint( SpatialVector (0., 0., 1., 0., 0., 0.));

		body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

		body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
		joint_b = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));

		body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

		body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
		joint_c = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

		body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		body_d = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_c = Joint ( SpatialVector (1., 0., 0., 0., 0., 0.));

		body_d_id = model->AddBody(body_c_id, Xtrans(Vector3d(0., 0., -1.)), joint_c, body_d);

		Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

		ClearLogOutput();
	}
	
	~KinematicsFixture () {
		delete model;
	}
	Model *model;

	unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
	Body body_a, body_b, body_c, body_d;
	Joint joint_a, joint_b, joint_c, joint_d;

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;
};

struct KinematicsFixture6DoF {
	KinematicsFixture6DoF () {
		ClearLogOutput();
		model = new Model;

		model->gravity = Vector3d  (0., -9.81, 0.);

		/* 
		 *
		 *          X Contact point (ref child)
		 *          |
		 *    Base  |
		 *   / body |
		 *  O-------*
		 *           \
		 *             Child body
		 */

		// base body (3 DoF)
		base = Body (
				1.,
				Vector3d (0.5, 0., 0.),
				Vector3d (1., 1., 1.)
				);
		joint_rotzyx = Joint (
				SpatialVector (0., 0., 1., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.)
				);
		base_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_rotzyx, base);

		// child body (3 DoF)
		child = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		child_id = model->AddBody (base_id, Xtrans (Vector3d (1., 0., 0.)), joint_rotzyx, child);

		Q = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		Tau = VectorNd::Constant (model->mBodies.size() - 1, 0.);

		ClearLogOutput();
	}
	
	~KinematicsFixture6DoF () {
		delete model;
	}
	Model *model;

	unsigned int base_id, child_id;
	Body base, child;
	Joint joint_rotzyx;

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;
};



TEST_FIXTURE(KinematicsFixture, TestPositionNeutral) {
	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	Vector3d body_position;

	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.), CalcBodyToBaseCoordinates(*model, Q, body_a_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (1., 0., 0.), CalcBodyToBaseCoordinates(*model, Q, body_b_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (1., 1., 0.), CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (1., 1., -1.), CalcBodyToBaseCoordinates(*model, Q, body_d_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
}

TEST_FIXTURE(KinematicsFixture, TestPositionBaseRotated90Deg) {
	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices

	Q[0] = 0.5 * M_PI;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	Vector3d body_position;

//	cout << LogOutput.str() << endl;
	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.), CalcBodyToBaseCoordinates(*model, Q, body_a_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (0., 1., 0.), CalcBodyToBaseCoordinates(*model, Q, body_b_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (-1., 1., 0.),CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (-1., 1., -1.), CalcBodyToBaseCoordinates(*model, Q, body_d_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
}

TEST_FIXTURE(KinematicsFixture, TestPositionBaseRotatedNeg45Deg) {
	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices

	Q[0] = -0.25 * M_PI;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	Vector3d body_position;

//	cout << LogOutput.str() << endl;
	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.), CalcBodyToBaseCoordinates(*model, Q, body_a_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (0.707106781186547, -0.707106781186547, 0.), CalcBodyToBaseCoordinates(*model, Q, body_b_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (sqrt(2.0), 0., 0.),CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (sqrt(2.0), 0., -1.), CalcBodyToBaseCoordinates(*model, Q, body_d_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
}

TEST_FIXTURE(KinematicsFixture, TestPositionBodyBRotated90Deg) {
	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices
	Q[1] = 0.5 * M_PI;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	Vector3d body_position;

	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.), CalcBodyToBaseCoordinates(*model, Q, body_a_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (1., 0., 0.), CalcBodyToBaseCoordinates(*model, Q, body_b_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (1., 1., 0.),CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (0., 1., 0.),CalcBodyToBaseCoordinates(*model, Q, body_d_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
}

TEST_FIXTURE(KinematicsFixture, TestPositionBodyBRotatedNeg45Deg) {
	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices
	Q[1] = -0.25 * M_PI;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	Vector3d body_position;

	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.), CalcBodyToBaseCoordinates(*model, Q, body_a_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (1., 0., 0.), CalcBodyToBaseCoordinates(*model, Q, body_b_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (1., 1., 0.),CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
	CHECK_ARRAY_CLOSE (Vector3d (1 + 0.707106781186547, 1., -0.707106781186547), CalcBodyToBaseCoordinates(*model, Q, body_d_id, Vector3d (0., 0., 0.), true), 3, TEST_PREC );
}

TEST_FIXTURE(KinematicsFixture, TestCalcBodyToBaseCoordinates) {
	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	CHECK_ARRAY_CLOSE (
			Vector3d (1., 2., 0.),
			CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 1., 0.)),
			3, TEST_PREC
			);
}

TEST_FIXTURE(KinematicsFixture, TestCalcBodyToBaseCoordinatesRotated) {
	Q[2] = 0.5 * M_PI;

	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	CHECK_ARRAY_CLOSE (
			Vector3d (1., 1., 0.).data(),
			CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 0., 0.), false).data(),
			3, TEST_PREC
			);

	CHECK_ARRAY_CLOSE (
			Vector3d (0., 1., 0.).data(),
			CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 1., 0.), false).data(),
			3, TEST_PREC
			);

	// Rotate the other way round
	Q[2] = -0.5 * M_PI;

	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	CHECK_ARRAY_CLOSE (
			Vector3d (1., 1., 0.),
			CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 0., 0.), false),
			3, TEST_PREC
			);

	CHECK_ARRAY_CLOSE (
			Vector3d (2., 1., 0.),
			CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 1., 0.), false),
			3, TEST_PREC
			);

	// Rotate around the base
	Q[0] = 0.5 * M_PI;
	Q[2] = 0.;

	// We call ForwardDynamics() as it updates the spatial transformation
	// matrices
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	CHECK_ARRAY_CLOSE (
			Vector3d (-1., 1., 0.),
			CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 0., 0.), false),
			3, TEST_PREC
			);

	CHECK_ARRAY_CLOSE (
			Vector3d (-2., 1., 0.),
			CalcBodyToBaseCoordinates(*model, Q, body_c_id, Vector3d (0., 1., 0.), false),
			3, TEST_PREC
			);

//	cout << LogOutput.str() << endl;
}

TEST(TestCalcPointJacobian) {
	Model model;
	Body base_body (1., Vector3d (0., 0., 0.), Vector3d (1., 1., 1.));
	unsigned int base_body_id = model.SetFloatingBaseBody(base_body);

	VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
	MatrixNd G = MatrixNd::Constant (3, model.dof_count, 0.);
	Vector3d point_position (1.1, 1.2, 2.1);
	Vector3d point_velocity_ref;
	Vector3d point_velocity;

	Q[0] = 1.1;
	Q[1] = 1.2;
	Q[2] = 1.3;
	Q[3] = 0.7;
	Q[4] = 0.8;
	Q[5] = 0.9;

	QDot[0] = -1.1;
	QDot[1] = 2.2;
	QDot[2] = 1.3;
	QDot[3] = -2.7;
	QDot[4] = 1.8;
	QDot[5] = -2.9;

	// Compute the reference velocity
	point_velocity_ref = CalcPointVelocity (model, Q, QDot, base_body_id, point_position);

	CalcPointJacobian (model, Q, base_body_id, point_position, G);

	point_velocity = G * QDot;

	CHECK_ARRAY_CLOSE (
			point_velocity_ref.data(),
			point_velocity.data(),
			3, TEST_PREC
			);
}

TEST_FIXTURE(KinematicsFixture, TestInverseKinematicSimple) {
	std::vector<unsigned int> body_ids;
	std::vector<Vector3d> body_points;
	std::vector<Vector3d> target_pos;

	Q[0] = 0.2;
	Q[1] = 0.1;
	Q[2] = 0.1;

	VectorNd Qres = VectorNd::Zero ((size_t) model->dof_count);

	unsigned int body_id = body_d_id;
	Vector3d body_point = Vector3d (1., 0., 0.);
	Vector3d target (1.3, 0., 0.);

	body_ids.push_back (body_d_id);
	body_points.push_back (body_point);
	target_pos.push_back (target);

	ClearLogOutput();
	bool res = InverseKinematics (*model, Q, body_ids, body_points, target_pos, Qres);
	//	cout << LogOutput.str() << endl;
	CHECK_EQUAL (true, res);

	UpdateKinematicsCustom (*model, &Qres, NULL, NULL);

	Vector3d effector;
	effector = CalcBodyToBaseCoordinates(*model, Qres, body_id, body_point, false);

	CHECK_ARRAY_CLOSE (target.data(), effector.data(), 3, TEST_PREC);	
}

TEST_FIXTURE(KinematicsFixture6DoF, TestInverseKinematicUnreachable) {
	std::vector<unsigned int> body_ids;
	std::vector<Vector3d> body_points;
	std::vector<Vector3d> target_pos;

	Q[0] = 0.2;
	Q[1] = 0.1;
	Q[2] = 0.1;

	VectorNd Qres = VectorNd::Zero ((size_t) model->dof_count);

	unsigned int body_id = child_id;
	Vector3d body_point = Vector3d (1., 0., 0.);
	Vector3d target (2.2, 0., 0.);

	body_ids.push_back (body_id);
	body_points.push_back (body_point);
	target_pos.push_back (target);

	ClearLogOutput();
	bool res = InverseKinematics (*model, Q, body_ids, body_points, target_pos, Qres, 1.0e-8, 0.9, 1000);
//	cout << LogOutput.str() << endl;
	CHECK_EQUAL (true, res);

	UpdateKinematicsCustom (*model, &Qres, NULL, NULL);

	Vector3d effector;
	effector = CalcBodyToBaseCoordinates(*model, Qres, body_id, body_point, false);

	CHECK_ARRAY_CLOSE (Vector3d (2.0, 0., 0.).data(), effector.data(), 3, 1.0e-7);	
}

TEST_FIXTURE(KinematicsFixture6DoF, TestInverseKinematicTwoPoints) {
	std::vector<unsigned int> body_ids;
	std::vector<Vector3d> body_points;
	std::vector<Vector3d> target_pos;

	Q[0] = 0.2;
	Q[1] = 0.1;
	Q[2] = 0.1;

	VectorNd Qres = VectorNd::Zero ((size_t) model->dof_count);

	unsigned int body_id = child_id;
	Vector3d body_point = Vector3d (1., 0., 0.);
	Vector3d target (2., 0., 0.);

	body_ids.push_back (body_id);
	body_points.push_back (body_point);
	target_pos.push_back (target);

	body_ids.push_back (base_id);
	body_points.push_back (Vector3d (0.6, 1.0, 0.));
	target_pos.push_back (Vector3d (0.5, 1.1, 0.));

	ClearLogOutput();
	bool res = InverseKinematics (*model, Q, body_ids, body_points, target_pos, Qres, 1.0e-3, 0.9, 200);
	CHECK_EQUAL (true, res);

//	cout << LogOutput.str() << endl;
	UpdateKinematicsCustom (*model, &Qres, NULL, NULL);

	Vector3d effector;

	// testing with very low precision
	effector = CalcBodyToBaseCoordinates(*model, Qres, body_ids[0], body_points[0], false);
	CHECK_ARRAY_CLOSE (target_pos[0].data(), effector.data(), 3, 1.0e-1);	

	effector = CalcBodyToBaseCoordinates(*model, Qres, body_ids[1], body_points[1], false);
	CHECK_ARRAY_CLOSE (target_pos[1].data(), effector.data(), 3, 1.0e-1);	
}

TEST ( FixedJointBodyCalcBodyToBase ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
	model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body);

	VectorNd Q_zero = VectorNd::Zero (model.dof_count);
	Vector3d base_coords = CalcBodyToBaseCoordinates (model, Q_zero, fixed_body_id, Vector3d (1., 1., 0.1));

	CHECK_ARRAY_CLOSE (Vector3d (1., 2., 0.1).data(), base_coords.data(), 3, TEST_PREC);
}

TEST ( FixedJointBodyCalcBodyToBaseRotated ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector(0., 0., 1., 0., 0., 0.));
	model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(1., 0., 0.)), Joint(JointTypeFixed), fixed_body);

	VectorNd Q = VectorNd::Zero (model.dof_count);

	ClearLogOutput();
	Q[0] = M_PI * 0.5;
	Vector3d base_coords = CalcBodyToBaseCoordinates (model, Q, fixed_body_id, Vector3d (1., 0., 0.));
//	cout << LogOutput.str() << endl;	

	CHECK_ARRAY_CLOSE (Vector3d (0., 2., 0.).data(), base_coords.data(), 3, TEST_PREC);
}

TEST ( FixedJointBodyCalcBaseToBody ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
	model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(0., 1., 0.)), Joint(JointTypeFixed), fixed_body);

	VectorNd Q_zero = VectorNd::Zero (model.dof_count);
	Vector3d base_coords = CalcBaseToBodyCoordinates (model, Q_zero, fixed_body_id, Vector3d (1., 2., 0.1));

	CHECK_ARRAY_CLOSE (Vector3d (1., 1., 0.1).data(), base_coords.data(), 3, TEST_PREC);
}

TEST ( FixedJointBodyCalcBaseToBodyRotated ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
	model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);
	unsigned int fixed_body_id = model.AppendBody (Xtrans(Vector3d(1., 0., 0.)), Joint(JointTypeFixed), fixed_body);

	VectorNd Q = VectorNd::Zero (model.dof_count);

	ClearLogOutput();
	Q[0] = M_PI * 0.5;
	Vector3d base_coords = CalcBaseToBodyCoordinates (model, Q, fixed_body_id, Vector3d (0., 2., 0.));
	// cout << LogOutput.str() << endl;	

	CHECK_ARRAY_CLOSE (Vector3d (1., 0., 0.).data(), base_coords.data(), 3, TEST_PREC);
}

TEST ( FixedJointBodyWorldOrientation ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
	model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

	SpatialTransform transform = Xrotz(0.25) * Xtrans (Vector3d (1., 2., 3.));
	unsigned int fixed_body_id = model.AppendBody (transform, Joint(JointTypeFixed), fixed_body);

	VectorNd Q_zero = VectorNd::Zero (model.dof_count);
	Matrix3d orientation = CalcBodyWorldOrientation (model, Q_zero, fixed_body_id);

	Matrix3d reference = transform.E;

	CHECK_ARRAY_CLOSE (reference.data(), orientation.data(), 9, TEST_PREC);
}

TEST ( FixedJointCalcPointJacobian ) {
	// the standard modeling using a null body
	Body null_body;
	Body body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));
	Body fixed_body(1., Vector3d (1., 0.4, 0.4), Vector3d (1., 1., 1.));

	Model model;

	Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));
	model.AddBody (0, Xtrans(Vector3d(0., 0., 0.)), joint_rot_z, body);

	SpatialTransform transform = Xrotz(0.25) * Xtrans (Vector3d (1., 2., 3.));
	unsigned int fixed_body_id = model.AppendBody (transform, Joint(JointTypeFixed), fixed_body);

	VectorNd Q = VectorNd::Zero (model.dof_count);
	VectorNd QDot = VectorNd::Zero (model.dof_count);

	Q[0] = 1.1;
	QDot[0] = 1.2;

	Vector3d point_position (1., 0., 0.);

	MatrixNd G = MatrixNd (3, model.dof_count);	
	CalcPointJacobian (model, Q, fixed_body_id, point_position, G);
	Vector3d point_velocity_jacobian = G * QDot;
	Vector3d point_velocity_reference = CalcPointVelocity (model, Q, QDot, fixed_body_id, point_position);

	CHECK_ARRAY_CLOSE (point_velocity_reference.data(), point_velocity_jacobian.data(), 3, TEST_PREC);
}
