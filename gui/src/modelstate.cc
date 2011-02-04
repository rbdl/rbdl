#include <iostream>

#include <QDebug>
#include <assert.h>

#include "modelstate.h"

#include "mathutils.h"
#include "Model.h"
#include "Contacts.h"
#include "Dynamics.h"
#include "Kinematics.h"

using namespace std;

static Model* model = NULL;

unsigned int body_a_id, body_b_id, body_c_id, body_d_id;
Body body_a, body_b, body_c, body_d;
Joint joint_a, joint_b, joint_c, joint_d;

unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id,
	child_rot_z_id, child_rot_y_id, child_rot_x_id;

Body base_rot_z, base_rot_y, base_rot_x,
	child_rot_z, child_rot_y, child_rot_x;

Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
	joint_child_rot_z, joint_child_rot_y, joint_child_rot_x;

cmlVector Q;
cmlVector QDot;
cmlVector QDDot;
cmlVector Tau;

unsigned int contact_body_id;
Vector3d contact_point;
Vector3d contact_normal;

typedef cmlVector (rhs_func) (double, const cmlVector&);

cmlVector rk45_integrator (double t0, double tf, cmlVector &y0, rhs_func func, double error) {
	cmlVector k1 (y0.size());
	cmlVector k2 (y0.size());
	cmlVector k3 (y0.size());
	cmlVector k4 (y0.size());
	cmlVector k5 (y0.size());
	cmlVector k6 (y0.size());
	cmlVector rk4 (y0.size());
	cmlVector rk5 (y0.size());

	double s = 1.;
	double t = t0;
	double h0 = tf - t0;
	double h = h0;
	double h_min = 1.0e5;
	cmlVector h_min_y (y0);
	cmlVector h_min_rk5 (y0);
	cmlVector y (y0);
	int stepcount = 0;

	while (t < tf) {

		k1 = h * func (t, y);
		k2 = h * func (t + 0.25 * h, y + h * 0.25 * k1);
		k3 = h * func (t + 3./8. * h, y + h * 3./32. * k1 + 9. / 32. * k2);
		k4 = h * func (t + 12./13. * h, y + 1932./2197. * k1 - 7200./2197. * k2 + 7296./2197. * k3);
		k5 = h * func (t + h, y + 439./216. * k1 - 8. * k2 + 3680./513. * k3 - 845./4104. * k4);
		k6 = h * func (t + 0.5 * h, y - 8./27. * k1 + 2. * k2 - 3544./2565. * k3 + 1859./4104. * k4 - 11./40. * k5);

		rk4 = y + 25./216. * k1 + 1408./2565. * k3 + 2197./4104. * k4 - 1./5. * k5;
		rk5 = y + 16./135. * k1 + 6656./12825. * k3 + 28561./56430. * k4 - 9./50. * k5 + 2./55. * k6;

		double error_est = cml::length(rk5 - rk4);

//		cout << "error estimate = " << scientific << error_est << endl;

		if (error_est > error) {
//			s = sqrt (sqrt ( (error * h) / 2 * error_est) );
			s = s * 0.5;
			h = s * h;

			if (h < h_min) {
				h_min = h;
				h_min_y = y;
			}
//			cout << "decreasing setting stepsize to " << s * (tf - t0) << endl;
		} else {
			t = t + h;
			y = rk5;
			stepcount ++;
//			cout << "success" << endl;

			if (1.0e1 * error_est < error ) {
				// increasing stepsize
//				s = sqrt (sqrt ( (error * h) / 2 * error_est) );
				s = 2.;
				h = s*h;

//				cout << "increasing setting stepsize to " << s * (tf - t0) << " t = " << t << endl;
			}	
		}
	}

	cout << "used " << stepcount << " steps" << endl;
	cout << "h_min = " << h_min << " y_state = " << h_min_y << " rk5 = " << h_min_rk5 <<  endl;

	return y;
//	return y0 + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.; 
}

cmlVector rk4_integrator (double t0, double tf, cmlVector &y0, rhs_func func, double stepsize) {
	cmlVector k1 (y0.size());
	cmlVector k2 (y0.size());
	cmlVector k3 (y0.size());
	cmlVector k4 (y0.size());
	cmlVector y (y0);

	double t = t0;
	double h = stepsize;
	unsigned int stepcount = 0;

	if (h > tf - t0)
		h = tf - t0;

	while (t < tf) {
		k1 = func (t, y);
		k2 = func (t + 0.5 * h, y + h * 0.5 * k1);
		k3 = func (t + 0.5 * h, y + h * 0.5 * k2);
		k4 = func (t + h, y + h * k3);

		t += h;
		y = y + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.; 

		if (tf - t < stepsize) {
			h = tf - t + stepsize;
		}

		stepcount ++;
	}

	cout << "stepcount = " << stepcount << endl;
	return y;
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

	/*
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

	Q = cmlVector(model->mBodies.size() - 1);
	QDot = cmlVector(model->mBodies.size() - 1);
	QDDot = cmlVector(model->mBodies.size() - 1);
	Tau = cmlVector(model->mBodies.size() - 1);

	Q.zero();
	QDot.zero();
	QDDot.zero();
	Tau.zero();

	Q[0] = 0.1;
	Q[2] = 0.1;

	/*
	contact_body_id = child_rot_x_id;
	contact_point.set (1., 0., 0.);
	contact_normal.set (0., 1., 0.);

	model->AddContact(contact_body_id, contact_point, contact_normal);
	*/

	// we call model_update once to update the internal variables for the
	// state, etc.
	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
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
	ynew = rk4_integrator (0., delta_time, y, rhs_contact, delta_time);

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
//	ynew = rk45_integrator (0., delta_time, y, rhs_normal, 1.0e-3);
	ynew = rk4_integrator (0., delta_time, y, rhs_normal, delta_time);

	for (i = 0; i < size; i++) {
		Q[i] = ynew[i];
		QDot[i] = ynew[i + size];
	}

	cout << "y = " << ynew << endl;
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
