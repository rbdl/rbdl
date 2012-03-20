#include <UnitTest++.h>

#include <iostream>

#include "Logging.h"

#include "Model.h"
#include "Contacts.h"
#include "Dynamics.h"
#include "Kinematics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

struct ImpulsesFixture {
	ImpulsesFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();

		model->gravity = Vector3d (0., -9.81, 0.);

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

		// base body
		base_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_base_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		base_rot_z_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_z, base_rot_z);

		base_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_base_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		base_rot_y_id = model->AddBody (base_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_y, base_rot_y);

		base_rot_x = Body (
				1.,
				Vector3d (0., 1., 0.),
				Vector3d (1., 1., 1.)
				);
		joint_base_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		base_rot_x_id = model->AddBody (base_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_x, base_rot_x);

		// child body
		child_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_z = Joint (
				JointTypeRevolute,
				Vector3d (0., 0., 1.)
				);
		child_rot_z_id = model->AddBody (base_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_rot_z, child_rot_z);

		child_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_y = Joint (
				JointTypeRevolute,
				Vector3d (0., 1., 0.)
				);
		child_rot_y_id = model->AddBody (child_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_y, child_rot_y);

		child_rot_x = Body (
				1.,
				Vector3d (0., 1., 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_rot_x = Joint (
				JointTypeRevolute,
				Vector3d (1., 0., 0.)
				);
		child_rot_x_id = model->AddBody (child_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_x, child_rot_x);

		Q = VectorNd::Zero(model->dof_count);
		QDot = VectorNd::Zero(model->dof_count);
		QDDot = VectorNd::Zero(model->dof_count);
		Tau = VectorNd::Zero(model->dof_count);

		contact_body_id = child_rot_x_id;
		contact_point = Vector3d (0., 1., 0.);
		contact_normal = Vector3d (0., 1., 0.);

		ClearLogOutput();
	}
	
	~ImpulsesFixture () {
		delete model;
	}
	Model *model;

	unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id,
		child_rot_z_id, child_rot_y_id, child_rot_x_id;

	Body base_rot_z, base_rot_y, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x;

	Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x;

	VectorNd Q;
	VectorNd QDot;
	VectorNd QDDot;
	VectorNd Tau;

	unsigned int contact_body_id;
	Vector3d contact_point;
	Vector3d contact_normal;
	ConstraintSet constraint_set;
};

TEST_FIXTURE(ImpulsesFixture, TestContactImpulse) {
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), NULL, 0.); 
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 1., 0.), NULL, 0.); 
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 0., 1.), NULL, 0.); 

	constraint_set.Bind (*model);

	QDot[0] = 0.1;
	QDot[1] = -0.2;
	QDot[2] = 0.1;

	Vector3d point_velocity;
	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, QDot, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;

	VectorNd qdot_post (QDot.size());
	ComputeContactImpulsesLagrangian (*model, Q, QDot, constraint_set, qdot_post);
	// cout << LogOutput.str() << endl;
	// cout << "QdotPost = " << qdot_post << endl;

	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, qdot_post, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;
	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.).data(), point_velocity.data(), 3, TEST_PREC);
}

TEST_FIXTURE(ImpulsesFixture, TestContactImpulseRotated) {
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), NULL, 0.); 
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 1., 0.), NULL, 0.); 
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 0., 1.), NULL, 0.); 

	constraint_set.Bind (*model);

	Q[0] = 0.2;
	Q[1] = -0.5;
	Q[2] = 0.1;
	Q[3] = -0.4;
	Q[4] = -0.1;
	Q[5] = 0.4;

	QDot[0] = 0.1;
	QDot[1] = -0.2;
	QDot[2] = 0.1;

	Vector3d point_velocity;
	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, QDot, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;
	VectorNd qdot_post (QDot.size());
	ComputeContactImpulsesLagrangian (*model, Q, QDot, constraint_set, qdot_post);
	// cout << LogOutput.str() << endl;
	// cout << "QdotPost = " << qdot_post << endl;

	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, qdot_post, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;
	CHECK_ARRAY_CLOSE (Vector3d (0., 0., 0.).data(), point_velocity.data(), 3, TEST_PREC);
}

TEST_FIXTURE(ImpulsesFixture, TestContactImpulseRotatedCollisionVelocity) {
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (1., 0., 0.), NULL, 1.); 
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 1., 0.), NULL, 2.); 
	constraint_set.AddConstraint(contact_body_id, contact_point, Vector3d (0., 0., 1.), NULL, 3.); 

	constraint_set.Bind (*model);

	Q[0] = 0.2;
	Q[1] = -0.5;
	Q[2] = 0.1;
	Q[3] = -0.4;
	Q[4] = -0.1;
	Q[5] = 0.4;

	QDot[0] = 0.1;
	QDot[1] = -0.2;
	QDot[2] = 0.1;

	Vector3d point_velocity;
	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, QDot, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;

	VectorNd qdot_post (QDot.size());
	ComputeContactImpulsesLagrangian (*model, Q, QDot, constraint_set, qdot_post);
	
	// cout << LogOutput.str() << endl;
	// cout << "QdotPost = " << qdot_post << endl;

	{
		SUPPRESS_LOGGING;
		point_velocity = CalcPointVelocity (*model, Q, qdot_post, contact_body_id, contact_point, true);
	}

	// cout << "Point Velocity = " << point_velocity << endl;
	CHECK_ARRAY_CLOSE (Vector3d (1., 2., 3.).data(), point_velocity.data(), 3, TEST_PREC);
}
