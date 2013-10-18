#include <UnitTest++.h>

#include <iostream>

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Contacts.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"

#include "Fixtures.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-11;

struct FixedBase6DoF9DoF {
	FixedBase6DoF9DoF () {
		ClearLogOutput();
		model = new Model;

		model->gravity = Vector3d  (0., -9.81, 0.);

		/* 3 DoF (rot.) joint at base
		 * 3 DoF (rot.) joint child origin
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

		// child body 1 (3 DoF)
		child = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		child_id = model->AddBody (base_id, Xtrans (Vector3d (0., 0., 0.)), joint_rotzyx, child);

		// child body (3 DoF)
		child_2 = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		child_2_id = model->AddBody (child_id, Xtrans (Vector3d (0., 0., 0.)), joint_rotzyx, child_2);

		Q = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		Tau = VectorNd::Constant (model->mBodies.size() - 1, 0.);

		contact_body_id = child_id;
		contact_point = Vector3d  (0.5, 0.5, 0.);
		contact_normal = Vector3d  (0., 1., 0.);

		ClearLogOutput();
	}
	
	~FixedBase6DoF9DoF () {
		delete model;
	}
	Model *model;

	unsigned int base_id, child_id, child_2_id;

	Body base, child, child_2;

	Joint joint_rotzyx;

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;

	unsigned int contact_body_id;
	Vector3d contact_point;
	Vector3d contact_normal;
	ConstraintSet constraint_set;
};

struct FixedBase6DoF12DoFFloatingBase {
	FixedBase6DoF12DoFFloatingBase () {
		ClearLogOutput();
		model = new Model;

		model->gravity = Vector3d  (0., -9.81, 0.);

		/* 3 DoF (rot.) joint at base
		 * 3 DoF (rot.) joint child origin
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
		base_id = model->SetFloatingBaseBody (base);

		// child body 1 (3 DoF)
		child = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_rotzyx = Joint (
				SpatialVector (0., 0., 1., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.)
				);
		child_id = model->AddBody (base_id, Xtrans (Vector3d (1., 0., 0.)), joint_rotzyx, child);

		// child body (3 DoF)
		child_2 = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		child_2_id = model->AddBody (child_id, Xtrans (Vector3d (1., 0., 0.)), joint_rotzyx, child_2);

		Q = VectorNd::Constant (model->dof_count, 0.);
		QDot = VectorNd::Constant (model->dof_count, 0.);
		QDDot = VectorNd::Constant (model->dof_count, 0.);
		Tau = VectorNd::Constant (model->dof_count, 0.);

		contact_body_id = child_id;
		contact_point = Vector3d  (0.5, 0.5, 0.);
		contact_normal = Vector3d  (0., 1., 0.);

		ClearLogOutput();
	}
	
	~FixedBase6DoF12DoFFloatingBase () {
		delete model;
	}
	Model *model;

	unsigned int base_id, child_id, child_2_id;

	Body base, child, child_2;

	Joint joint_rotzyx;

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;

	unsigned int contact_body_id;
	Vector3d contact_point;
	Vector3d contact_normal;
	ConstraintSet constraint_set;
};

// 
// ForwardDynamicsContactsLagrangian 
// 
TEST ( TestForwardDynamicsContactsLagrangianSimple ) {
	Model model;
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

	ConstraintSet constraint_set;

	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), "ground_x");
	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (0., 1., 0.), "ground_y");
	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (0., 0., 1.), "ground_z");

	constraint_set.Bind (model);

	ClearLogOutput();

//	cout << constraint_set.acceleration.transpose() << endl;
	ForwardDynamicsContactsLagrangian (model, Q, QDot, Tau, constraint_set, QDDot);

//	cout << "A = " << endl << constraint_set.A << endl << endl;
//	cout << "H = " << endl << constraint_set.H << endl << endl;
//	cout << "b = " << endl << constraint_set.b << endl << endl;
//	cout << "x = " << endl << constraint_set.x << endl << endl;
//	cout << constraint_set.b << endl;
//	cout << "QDDot = " << QDDot.transpose() << endl;

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
}

TEST ( TestForwardDynamicsContactsLagrangianMoving ) {
	Model model;
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

	ConstraintSet constraint_set;

	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), "ground_x");
	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (0., 1., 0.), "ground_y");
	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (0., 0., 1.), "ground_z");

	constraint_set.Bind (model);

	ClearLogOutput();

	ForwardDynamicsContactsLagrangian (model, Q, QDot, Tau, constraint_set, QDDot);

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
}
 
// 
// ForwardDynamicsContacts
// 
TEST_FIXTURE (FixedBase6DoF, ForwardDynamicsContactsSingleContact) {
	contact_normal.set (0., 1., 0.);
	constraint_set.AddConstraint (contact_body_id, contact_point, contact_normal);
	ConstraintSet constraint_set_lagrangian = constraint_set.Copy();

	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);
	
	Vector3d point_accel_lagrangian, point_accel_contacts;
	
	ClearLogOutput();

	VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	
	ClearLogOutput();
	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
	ClearLogOutput();
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot_contacts);
//	cout << LogOutput.str() << endl;
	ClearLogOutput();

	point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
	point_accel_contacts = CalcPointAcceleration (*model, Q, QDot, QDDot_contacts, contact_body_id, contact_point, true);

	CHECK_CLOSE (constraint_set_lagrangian.force[0], constraint_set.force[0], TEST_PREC);
	CHECK_CLOSE (contact_normal.dot(point_accel_lagrangian), contact_normal.dot(point_accel_contacts), TEST_PREC);
	CHECK_ARRAY_CLOSE (point_accel_lagrangian.data(), point_accel_contacts.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot_contacts.data(), QDDot_lagrangian.size(), TEST_PREC);
}

TEST_FIXTURE (FixedBase6DoF, ForwardDynamicsContactsSingleContactRotated) {
	Q[0] = 0.6;
	Q[3] =   M_PI * 0.6;
	Q[4] = 0.1;

	contact_normal.set (0., 1., 0.);
	
	constraint_set.AddConstraint (contact_body_id, contact_point, contact_normal);
	ConstraintSet constraint_set_lagrangian = constraint_set.Copy();
	
	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);
	
	Vector3d point_accel_lagrangian, point_accel_contacts, point_accel_contacts_opt;
	
	ClearLogOutput();

	VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	VectorNd QDDot_contacts_opt = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	
	ClearLogOutput();
	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot_contacts_opt);

	point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
	point_accel_contacts_opt = CalcPointAcceleration (*model, Q, QDot, QDDot_contacts_opt, contact_body_id, contact_point, true);

	CHECK_CLOSE (constraint_set_lagrangian.force[0], constraint_set.force[0], TEST_PREC);
	CHECK_CLOSE (contact_normal.dot(point_accel_lagrangian), contact_normal.dot(point_accel_contacts_opt), TEST_PREC);
	CHECK_ARRAY_CLOSE (point_accel_lagrangian.data(), point_accel_contacts_opt.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot_contacts_opt.data(), QDDot_lagrangian.size(), TEST_PREC);
}

// 
// Similiar to the previous test, this test compares the results of 
//   - ForwardDynamicsContactsLagrangian
//   - ForwardDynamcsContactsOpt
// for the example model in FixedBase6DoF and a moving state (i.e. a
// nonzero QDot)
// 
TEST_FIXTURE (FixedBase6DoF, ForwardDynamicsContactsSingleContactRotatedMoving) {
	Q[0] = 0.6;
	Q[3] =   M_PI * 0.6;
	Q[4] = 0.1;

	QDot[0] = -0.3;
	QDot[1] = 0.1;
	QDot[2] = -0.5;
	QDot[3] = 0.8;

	contact_normal.set (0., 1., 0.);
	constraint_set.AddConstraint (contact_body_id, contact_point, contact_normal);
	ConstraintSet constraint_set_lagrangian = constraint_set.Copy();
	
	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);
	
	Vector3d point_accel_lagrangian, point_accel_contacts;
	
	VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	
	ClearLogOutput();
	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
//	cout << LogOutput.str() << endl;
	ClearLogOutput();
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot_contacts);
//	cout << LogOutput.str() << endl;

	point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
	point_accel_contacts = CalcPointAcceleration (*model, Q, QDot, QDDot_contacts, contact_body_id, contact_point, true);

	// check whether FDContactsLagrangian and FDContactsOld match
	CHECK_CLOSE (constraint_set_lagrangian.force[0], constraint_set.force[0], TEST_PREC);

	CHECK_CLOSE (contact_normal.dot(point_accel_lagrangian), contact_normal.dot(point_accel_contacts), TEST_PREC);
	CHECK_ARRAY_CLOSE (point_accel_lagrangian.data(), point_accel_contacts.data(), 3, TEST_PREC);
	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot_contacts.data(), QDDot_lagrangian.size(), TEST_PREC);
}

TEST_FIXTURE (FixedBase6DoF, ForwardDynamicsContactsOptDoubleContact) {
	ConstraintSet constraint_set_lagrangian;

	constraint_set.AddConstraint (contact_body_id, Vector3d (1., 0., 0.), contact_normal);
	constraint_set.AddConstraint (contact_body_id, Vector3d (0., 1., 0.), contact_normal);
	
	constraint_set_lagrangian = constraint_set.Copy();
	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);
	
	Vector3d point_accel_lagrangian, point_accel_contacts;
	
	ClearLogOutput();

	VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	
	ClearLogOutput();

	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot_contacts);

	point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
	point_accel_contacts = CalcPointAcceleration (*model, Q, QDot, QDDot_contacts, contact_body_id, contact_point, true);

	// check whether FDContactsLagrangian and FDContacts match
	CHECK_ARRAY_CLOSE (
			constraint_set_lagrangian.force.data(),
			constraint_set.force.data(),
			constraint_set.size(), TEST_PREC
			);

	// check whether the point accelerations match
	CHECK_ARRAY_CLOSE (point_accel_lagrangian.data(), point_accel_contacts.data(), 3, TEST_PREC);

	// check whether the generalized accelerations match
	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot_contacts.data(), QDDot_lagrangian.size(), TEST_PREC);
}

TEST_FIXTURE (FixedBase6DoF, ForwardDynamicsContactsOptDoubleContactRepeated) {
	// makes sure that all variables in the constraint set gets reset
	// properly when making repeated calls to ForwardDynamicsContacts.
	ConstraintSet constraint_set_lagrangian;

	constraint_set.AddConstraint (contact_body_id, Vector3d (1., 0., 0.), contact_normal);
	constraint_set.AddConstraint (contact_body_id, Vector3d (0., 1., 0.), contact_normal);
	
	constraint_set_lagrangian = constraint_set.Copy();
	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);
	
	Vector3d point_accel_lagrangian, point_accel_contacts;
	
	ClearLogOutput();

	VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	
	ClearLogOutput();

	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
	// Call ForwardDynamicsContacts multiple times such that old values might
	// be re-used and thus cause erroneus values.
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot_contacts);
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot_contacts);
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot_contacts);

	point_accel_lagrangian = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point, true);
	point_accel_contacts = CalcPointAcceleration (*model, Q, QDot, QDDot_contacts, contact_body_id, contact_point, true);

	// check whether FDContactsLagrangian and FDContacts match
	CHECK_ARRAY_CLOSE (
			constraint_set_lagrangian.force.data(),
			constraint_set.force.data(),
			constraint_set.size(), TEST_PREC
			);

	// check whether the point accelerations match
	CHECK_ARRAY_CLOSE (point_accel_lagrangian.data(), point_accel_contacts.data(), 3, TEST_PREC);

	// check whether the generalized accelerations match
	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot_contacts.data(), QDDot_lagrangian.size(), TEST_PREC);
}

TEST_FIXTURE (FixedBase6DoF, ForwardDynamicsContactsOptMultipleContact) {
	ConstraintSet constraint_set_lagrangian;

	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (1., 0., 0.));
	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (0., 1., 0.));
	
	constraint_set_lagrangian = constraint_set.Copy();
	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);

	// we rotate the joints so that we have full mobility at the contact
	// point:
	//
	//  O       X (contact point)
	//   \     /
	//    \   /
	//     \ /
	//      *      
	//

	Q[0] = M_PI * 0.25;
	Q[1] = 0.2;
	Q[3] = M_PI * 0.5;

	VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	VectorNd QDDot_contacts = VectorNd::Constant (model->mBodies.size() - 1, 0.);
	
	ClearLogOutput();
	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot_contacts);

//	cout << LogOutput.str() << endl;

	Vector3d point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
//	cout << "point_accel_c = " << point_accel_c.transpose() << endl;

//	cout << "Lagrangian contact force " << contact_data_lagrangian[0].force << ", " << contact_data_lagrangian[1].force << endl;

	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot_contacts.data(), QDDot_lagrangian.size(), TEST_PREC);

	CHECK_ARRAY_CLOSE (
			constraint_set_lagrangian.force.data(),
			constraint_set.force.data(),
			constraint_set.size(), TEST_PREC
			);

	CHECK_CLOSE (0., point_accel_c[0], TEST_PREC);
	CHECK_CLOSE (0., point_accel_c[1], TEST_PREC);
}

TEST_FIXTURE (FixedBase6DoF9DoF, ForwardDynamicsContactsOptMultipleContactsMultipleBodiesMoving) {
	ConstraintSet constraint_set_lagrangian;

	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (1., 0., 0.));
	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (0., 1., 0.));
	constraint_set.AddConstraint (child_2_id, contact_point, Vector3d (0., 1., 0.));
	
	constraint_set_lagrangian = constraint_set.Copy();
	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);
	
	Q[0] = 0.1;
	Q[1] = -0.1;
	Q[2] = 0.1;
	Q[3] = -0.1;
	Q[4] = -0.1;
	Q[5] = 0.1;

	QDot[0] =  1.; 
	QDot[1] = -1.;
	QDot[2] =  1; 
	QDot[3] = -1.5; 
	QDot[4] =  1.5; 
	QDot[5] = -1.5; 

	VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);

	ClearLogOutput();
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot);
//	cout << LogOutput.str() << endl;

	Vector3d point_accel_c, point_accel_2_c;

	point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot, child_2_id, contact_point);

//	cout << "point_accel_c = " << point_accel_c.transpose() << endl;

	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
//	cout << "Lagrangian contact force " << contact_data_lagrangian[0].force << ", " << contact_data_lagrangian[1].force << ", " << contact_data_lagrangian[2].force << endl;

	CHECK_ARRAY_CLOSE (
			constraint_set_lagrangian.force.data(),
			constraint_set.force.data(),
			constraint_set.size(), TEST_PREC
			);

	CHECK_CLOSE (0., point_accel_c[0], TEST_PREC);
	CHECK_CLOSE (0., point_accel_c[1], TEST_PREC);
	CHECK_CLOSE (0., point_accel_2_c[1], TEST_PREC);

	point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point);
	point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, child_2_id, contact_point);

	CHECK_CLOSE (0., point_accel_c[0], TEST_PREC);
	CHECK_CLOSE (0., point_accel_c[1], TEST_PREC);
	CHECK_CLOSE (0., point_accel_2_c[1], TEST_PREC);

	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot.data(), QDDot.size(), TEST_PREC);
}

TEST_FIXTURE (FixedBase6DoF9DoF, ForwardDynamicsContactsOptMultipleContactsMultipleBodiesMovingAlternate) {
	ConstraintSet constraint_set_lagrangian;

	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (1., 0., 0.));
	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (0., 1., 0.));
	constraint_set.AddConstraint (child_2_id, contact_point, Vector3d (0., 1., 0.));
	
	constraint_set_lagrangian = constraint_set.Copy();
	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);

	Q[0] = 0.1;
	Q[1] = -0.3;
	Q[2] = 0.15;
	Q[3] = -0.21;
	Q[4] = -0.81;
	Q[5] = 0.11;
	Q[6] = 0.31;
	Q[7] = -0.91;
	Q[8] = 0.61;

	QDot[0] =  1.3; 
	QDot[1] = -1.7;
	QDot[2] =  3; 
	QDot[3] = -2.5; 
	QDot[4] =  1.5; 
	QDot[5] = -5.5; 
	QDot[6] =  2.5; 
	QDot[7] = -1.5; 
	QDot[8] = -3.5; 

	VectorNd QDDot_lagrangian = VectorNd::Constant (model->mBodies.size() - 1, 0.);

	ClearLogOutput();
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot);
//	cout << LogOutput.str() << endl;

	Vector3d point_accel_c, point_accel_2_c;

	point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot, child_2_id, contact_point);

//	cout << "point_accel_c = " << point_accel_c.transpose() << endl;

	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
//	cout << "Lagrangian contact force " << contact_data_lagrangian[0].force << ", " << contact_data_lagrangian[1].force << ", " << contact_data_lagrangian[2].force << endl;

	CHECK_ARRAY_CLOSE (
			constraint_set_lagrangian.force.data(),
			constraint_set.force.data(),
			constraint_set.size(), TEST_PREC
			);

	CHECK_CLOSE (0., point_accel_c[0], TEST_PREC);
	CHECK_CLOSE (0., point_accel_c[1], TEST_PREC);
	CHECK_CLOSE (0., point_accel_2_c[1], TEST_PREC);

	point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point);
	point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, child_2_id, contact_point);

	CHECK_CLOSE (0., point_accel_c[0], TEST_PREC);
	CHECK_CLOSE (0., point_accel_c[1], TEST_PREC);
	CHECK_CLOSE (0., point_accel_2_c[1], TEST_PREC);

	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot.data(), QDDot.size(), TEST_PREC);
}

TEST_FIXTURE (FixedBase6DoF12DoFFloatingBase, ForwardDynamicsContactsMultipleContactsFloatingBase) {
	ConstraintSet constraint_set_lagrangian;

	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (1., 0., 0.));
	constraint_set.AddConstraint (contact_body_id, contact_point, Vector3d (0., 1., 0.));
	constraint_set.AddConstraint (child_2_id, contact_point, Vector3d (0., 1., 0.));
	
	constraint_set_lagrangian = constraint_set.Copy();
	constraint_set_lagrangian.Bind (*model);
	constraint_set.Bind (*model);

	VectorNd QDDot_lagrangian = VectorNd::Constant (model->dof_count, 0.);

	Q[0] = 0.1;
	Q[1] = -0.3;
	Q[2] = 0.15;
	Q[3] = -0.21;
	Q[4] = -0.81;
	Q[5] = 0.11;
	Q[6] = 0.31;
	Q[7] = -0.91;
	Q[8] = 0.61;

	QDot[0] =  1.3; 
	QDot[1] = -1.7;
	QDot[2] =  3; 
	QDot[3] = -2.5; 
	QDot[4] =  1.5; 
	QDot[5] = -5.5; 
	QDot[6] =  2.5; 
	QDot[7] = -1.5; 
	QDot[8] = -3.5; 

	ClearLogOutput();
	ForwardDynamicsContacts (*model, Q, QDot, Tau, constraint_set, QDDot);
//	cout << LogOutput.str() << endl;

	Vector3d point_accel_c, point_accel_2_c;

	point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point);
	point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot, child_2_id, contact_point);

//	cout << "point_accel_c = " << point_accel_c.transpose() << endl;

	ClearLogOutput();
	ForwardDynamicsContactsLagrangian (*model, Q, QDot, Tau, constraint_set_lagrangian, QDDot_lagrangian);
//	cout << "Lagrangian contact force " << contact_data_lagrangian[0].force << ", " << contact_data_lagrangian[1].force << ", " << contact_data_lagrangian[2].force << endl;
//	cout << LogOutput.str() << endl;

	CHECK_ARRAY_CLOSE (
			constraint_set_lagrangian.force.data(),
			constraint_set.force.data(),
			constraint_set.size(), TEST_PREC
			);

	CHECK_CLOSE (0., point_accel_c[0], TEST_PREC);
	CHECK_CLOSE (0., point_accel_c[1], TEST_PREC);
	CHECK_CLOSE (0., point_accel_2_c[1], TEST_PREC);

	point_accel_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, contact_body_id, contact_point);
	point_accel_2_c = CalcPointAcceleration (*model, Q, QDot, QDDot_lagrangian, child_2_id, contact_point);

	CHECK_CLOSE (0., point_accel_c[0], TEST_PREC);
	CHECK_CLOSE (0., point_accel_c[1], TEST_PREC);
	CHECK_CLOSE (0., point_accel_2_c[1], TEST_PREC);

	CHECK_ARRAY_CLOSE (QDDot_lagrangian.data(), QDDot.data(), QDDot.size(), TEST_PREC);
}
