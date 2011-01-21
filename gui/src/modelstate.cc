#include <QDebug>
#include <assert.h>

#include "modelstate.h"

#include "mathutils.h"
#include "Model.h"
#include "Contacts.h"
#include "Dynamics.h"
#include "Kinematics.h"

static Model* model = NULL;

unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
Body body_a, body_b, body_c, body_d;
Joint joint_a, joint_b, joint_c, joint_d;

cmlVector Q;
cmlVector QDot;
cmlVector QDDot;
cmlVector Tau;

unsigned int contact_body_id;
Vector3d contact_point;
Vector3d contact_normal;

void model_init () {
	model = new Model;
	model->Init();

	model->gravity.set (0., -9.81, 0.);

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

	body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

	body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
	joint_c = Joint (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

	Q = cmlVector(model->mBodies.size() - 1);
	QDot = cmlVector(model->mBodies.size() - 1);
	QDDot = cmlVector(model->mBodies.size() - 1);
	Tau = cmlVector(model->mBodies.size() - 1);

	Q.zero();
	QDot.zero();
	QDDot.zero();
	Tau.zero();

	contact_body_id = body_c_id;
	contact_point.set (0., 0., 0.);
	contact_normal.set (0., 1., 0.);

	model->AddContact(contact_body_id, contact_point, contact_normal);

	// we call model_update once to update the internal variables for the
	// state, etc.
	model_update (0.);
}

void model_update_contact (double delta_time) {
	ForwardDynamicsContacts (*model, Q, QDot, Tau, QDDot);

	Vector3d point_accel;
	CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point, point_accel);

	Vector3d point_velocity;
	CalcPointVelocity (*model, Q, QDot, contact_body_id, contact_point, point_velocity);

	Vector3d point_pos = model->GetBodyPointPosition (contact_body_id, contact_point);

	qDebug() << "accel =" << cml::dot(point_accel, contact_normal) 
		<< " point_accel =" << point_accel[0] << point_accel[1] << point_accel[2]
		<< " point_veloc =" << point_velocity[0] << point_velocity[1] << point_velocity[2];

	
	// << "state = " << Q[0] << Q[1] << Q[2] << "vel = " << QDot[0] << QDot[1] << QDot[2];

	delta_time *= 1.0e-1;
	int i;
	for (i = 0; i < Q.size(); i++) {
		Q[i] += delta_time * QDot[i];
		QDot[i] += delta_time * QDDot[i];
	}
}

void model_update (double delta_time) {
	model_update_contact (delta_time);
	return;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	int i;
	for (i = 0; i < Q.size(); i++) {
		Q[i] += delta_time * QDot[i];
		QDot[i] += delta_time * QDDot[i];
	}
}

Model* model_get() {
	assert (model);
	
	return model;
}

void model_destroy () {
	if (model)
		delete model;

	model = NULL;
}
