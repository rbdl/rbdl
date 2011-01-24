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

typedef cmlVector (rhs_func) (double, const cmlVector&);

cmlVector rk4_integrator (double t, double h, cmlVector &y0, rhs_func func) {
	cmlVector k1 (y0.size());
	cmlVector k2 (y0.size());
	cmlVector k3 (y0.size());
	cmlVector k4 (y0.size());

	k1 = func (t, y0);
	k2 = func (t + 0.5 * h, y0 + h * 0.5 * k1);
	k3 = func (t + 0.5 * h, y0 + h * 0.5 * k2);
	k4 = func (t + h, y0 + h * k3);

	return y0 + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.; 
}


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
	contact_point.set (1., 0., 0.);
	contact_normal.set (0., 1., 0.);

	model->AddContact(contact_body_id, contact_point, contact_normal);

	// we call model_update once to update the internal variables for the
	// state, etc.
	model_update (0.);
}

cmlVector rhs_contact (double t, const cmlVector &y) {
	unsigned int i;
	unsigned int size = Q.size();

	cmlVector q (size);
	cmlVector qdot (size);
	cmlVector qddot (size);

	for (i = 0; i < size; i++) {
		q[i] = y[i];
		qdot[i] = y[i + size];
	}

	ForwardDynamicsContacts (*model, q, qdot, Tau, qddot);

	cmlVector res (size * 2);
	for (i = 0; i < size; i++) {
		res[i] = qdot[i];
		res[i + size] = qddot[i];
	}

	return res;
}

cmlVector rhs_normal (double t, const cmlVector &y) {
	unsigned int i;
	unsigned int size = Q.size();

	cmlVector q (size);
	cmlVector qdot (size);
	cmlVector qddot (size);

	for (i = 0; i < size; i++) {
		q[i] = y[i];
		qdot[i] = y[i + size];
	}

	ForwardDynamics (*model, q, qdot, Tau, qddot);

	cmlVector res (size * 2);
	for (i = 0; i < size; i++) {
		res[i] = qdot[i];
		res[i + size] = qddot[i];
	}

	return res;
}


void model_update_contact (double delta_time) {
	unsigned int size = Q.size();
	unsigned int i;
	
	cmlVector y (size * 2);

	for (i = 0; i < size; i++) {
		y[i] = Q[i];
		y[i + size] = QDot[i];
	}

	cmlVector ynew (size * 2);
	ynew = rk4_integrator (0., delta_time, y, rhs_contact);

	for (i = 0; i < size; i++) {
		Q[i] += ynew[i];
		QDot[i] += ynew[i + size];
	}

	Vector3d point_accel;
	CalcPointAcceleration (*model, Q, QDot, QDDot, contact_body_id, contact_point, point_accel);

	Vector3d point_velocity;
	CalcPointVelocity (*model, Q, QDot, contact_body_id, contact_point, point_velocity);

	Vector3d point_pos = model->GetBodyPointPosition (contact_body_id, contact_point);

	qDebug() << "accel =" << cml::dot(point_accel, contact_normal) 
		<< " point_accel =" << point_accel[0] << point_accel[1] << point_accel[2]
		<< " point_veloc =" << point_velocity[0] << point_velocity[1] << point_velocity[2];
}

void model_update (double delta_time) {
//	model_update_contact (delta_time);
//	return;

	unsigned int size = Q.size();
	unsigned int i;
	
	cmlVector y (size * 2);

	for (i = 0; i < size; i++) {
		y[i] = Q[i];
		y[i + size] = QDot[i];
	}

	cmlVector ynew (size * 2);
	ynew = rk4_integrator (0., delta_time, y, rhs_normal);

	for (i = 0; i < size; i++) {
		Q[i] = ynew[i];
		QDot[i] = ynew[i + size];
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
