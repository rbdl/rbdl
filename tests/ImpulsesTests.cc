#include <UnitTest++.h>

#include <iostream>

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Contacts.h"
#include "rbdl/Dynamics.h"
#include "rbdl/Kinematics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

const double TEST_PREC = 1.0e-14;

struct ImpulsesFixture {
	ImpulsesFixture () {
		ClearLogOutput();
		model = new Model;

		model->gravity = Vector3d (0., -9.81, 0.);

		// base body
		base = Body (
				1.,
				Vector3d (0., 1., 0.),
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
				Vector3d (0., 1., 0.),
				Vector3d (1., 1., 1.)
				);
		child_id = model->AddBody (base_id, Xtrans (Vector3d (1., 0., 0.)), joint_rotzyx, child);

		Q = VectorNd::Zero(model->dof_count);
		QDot = VectorNd::Zero(model->dof_count);
		QDDot = VectorNd::Zero(model->dof_count);
		Tau = VectorNd::Zero(model->dof_count);

		contact_body_id = child_id;
		contact_point = Vector3d (0., 1., 0.);
		contact_normal = Vector3d (0., 1., 0.);

		ClearLogOutput();
	}
	
	~ImpulsesFixture () {
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

	constraint_set.v_plus[0] = 0.;
	constraint_set.v_plus[1] = 0.;
	constraint_set.v_plus[2] = 0.;

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

	constraint_set.v_plus[0] = 0.;
	constraint_set.v_plus[1] = 0.;
	constraint_set.v_plus[2] = 0.;

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

	constraint_set.v_plus[0] = 1.;
	constraint_set.v_plus[1] = 2.;
	constraint_set.v_plus[2] = 3.;

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
