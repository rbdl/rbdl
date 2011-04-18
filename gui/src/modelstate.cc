#include <iostream>
#include <limits>

#include <QDebug>
#include <assert.h>

#include "modelstate.h"

#include "mathutils.h"
#include "Model.h"
#include "Contacts.h"
#include "Dynamics.h"
#include "Kinematics.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace SpatialAlgebra;
using namespace Experimental;

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

VectorNd Q;
VectorNd QDot;
VectorNd QDDot;
VectorNd Tau;

unsigned int contact_body_id;
Vector3d contact_point;
Vector3d contact_normal;

std::vector<ContactInfo> contact_data;

typedef VectorNd (rhs_func) (double, const VectorNd&);

VectorNd rk45_integrator (double t0, double tf, VectorNd &y0, rhs_func func, double error) {
	VectorNd k1 (y0.size());
	VectorNd k2 (y0.size());
	VectorNd k3 (y0.size());
	VectorNd k4 (y0.size());
	VectorNd k5 (y0.size());
	VectorNd k6 (y0.size());
	VectorNd rk4 (y0.size());
	VectorNd rk5 (y0.size());

	double s = 1.;
	double t = t0;
	double h0 = tf - t0;
	double h = h0;
	double h_min = 1.0e5;
	VectorNd h_min_y (y0);
	VectorNd h_min_rk5 (y0);
	VectorNd y (y0);
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

		double error_est = (rk5 - rk4).norm();

//		cout << "error estimate = " << scientific << error_est << endl;

		// if error is too big -> decrease stepsize
		if (error_est > error) {
//			s = sqrt (sqrt ( (error * h) / 2 * error_est) );
			s = s * 0.5;
			h = s * h;

			if (h < h_min) {
				h_min = h;
				h_min_y = y;
			}

			if (h < sqrt(numeric_limits<double>::epsilon())) {
				cerr << "Could not determine stepsize!" << endl;
				assert(0);
				abort();
			}
			cout << "decreasing setting stepsize to " << s * (tf - t0) << endl;
		} else {
			t = t + h;
			y = rk5;
			stepcount ++;
			cout << "success (" << tf - t << " still to go..." << endl;

			if (2.0e0 * error_est < error ) {
				// increasing stepsize
//				s = sqrt (sqrt ( (error * h) / 2 * error_est) );
				s = 2.;
				h = s*h;

				cout << "increasing setting stepsize to " << s * (tf - t0) << " t = " << t << endl;
			}	
		}
	}

	cout << "used " << stepcount << " steps" << endl;
	cout << "h_min = " << h_min << " y_state = " << h_min_y << " rk5 = " << h_min_rk5 <<  endl;

	return y;
//	return y0 + h * (k1 + 2 * k2 + 2 * k3 + k4) / 6.; 
}

VectorNd rk4_integrator (double t0, double tf, VectorNd &y0, rhs_func func, double stepsize) {
	VectorNd k1 (y0.size());
	VectorNd k2 (y0.size());
	VectorNd k3 (y0.size());
	VectorNd k4 (y0.size());
	VectorNd y (y0);

	double t = t0;
	double h = stepsize;
	unsigned int stepcount = 0;

	if (h > tf - t0)
		h = tf - t0;

//	cout << "t0 = " << t0 << " tf = " << tf << " h = " << h << endl;
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
//		cout << stepcount << " " << tf - t << endl;
	}

//	cout << "stepcount = " << stepcount << endl;
	return y;
}

VectorNd euler_integrator (double t0, double tf, VectorNd &y0, rhs_func func, double stepsize) {
	VectorNd y (y0);

	VectorNd ydot (y0);
	ydot = func (tf, y);

	y = y0 + (tf - t0) * ydot;

	return y;
}

void model_init () {
	model = new Model;
	model->Init();

	model->gravity = Vector3d (0., 0., 0.);

	// base body
	Body base (
			1.,
			Vector3d (0., 0., 0.),
			Vector3d (1., 1., 1.)
			);

	unsigned int base_body_id = model->SetFloatingBaseBody(base);

	Q = VectorNd::Constant (model->dof_count, 0.);
	QDot = VectorNd::Constant (model->dof_count, 0.);
	QDDot = VectorNd::Constant (model->dof_count, 0.);
	Tau = VectorNd::Constant (model->dof_count, 0.);

	model->SetBodyVisualizationSphere(
			base_body_id,
			Vector3d (0.7, 0.9, 0.7),
			Vector3d (0., 0., 0.),
			1.	
			);

	contact_body_id = base_body_id;
	contact_point = Vector3d (0., -1., 0.);
	contact_normal = Vector3d (0., 1., 0.);

	Q[1] = 1.;

	// We want the body to rotate around its contact point which is located
	// at (0, 0, 0). There it should have a negative unit rotation around the
	// Z-axis (i.e. rolling along the X axis). The spatial velocity of the
	// body at the contact point is therefore (0, 0, -1, 0, 0, 0).
	SpatialVector velocity_ground (0., 0., -1., 0., 0., 0.);

	// This has now to be transformed to body coordinates.
	SpatialVector velocity_body = Xtrans (Vector3d (0., 1., 0.)) * velocity_ground;

	// This has now to be shuffled such that it complies with the ordering of
	// the DoF in the generalized velocity vector.
	QDot[0] = velocity_body[3];
	QDot[1] = velocity_body[4];
	QDot[2] = velocity_body[5];
	QDot[3] = velocity_body[2];
	QDot[4] = velocity_body[1];
	QDot[5] = velocity_body[0];

	cout << "velocity_body = " << velocity_body << std::endl;
	cout << "Q = " << Q << std::endl;
	cout << "QDot = " << QDot << std::endl;

	// we call model_update once to update the internal variables for the
	// state, etc.
	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
}

VectorNd rhs_contact (double t, const VectorNd &y) {
	unsigned int i;
	unsigned int size = Q.size();

	VectorNd q (size);
	VectorNd qdot (size);
	VectorNd qddot (size);

	std::vector<ContactInfo> contact_data;
	contact_point = model->CalcBaseToBodyCoordinates (contact_body_id, Vector3d (Q[0], 0., Q[2]));
	
	Vector3d contact_point_world;
	contact_point_world = model->CalcBodyToBaseCoordinates(contact_body_id, contact_point);

	cout << "Q = " << Q << "\tCP = " << contact_point_world;

	contact_data.push_back(ContactInfo (contact_body_id, contact_point, Vector3d (1., 0., 0.), 0.));
	contact_data.push_back(ContactInfo (contact_body_id, contact_point, Vector3d (0., 1., 0.), 1.));
	contact_data.push_back(ContactInfo (contact_body_id, contact_point, Vector3d (0., 0., 1.), 0.));

	for (i = 0; i < size; i++) {
		q[i] = y[i];
		qdot[i] = y[i + size];
	}

	{
		_NoLogging nolog;
		ForwardDynamicsContactsLagrangian (*model, q, qdot, Tau, contact_data, qddot);
	}
	cout << "\tqdd = " << qddot ;

	Vector3d contact_point_world_vel;
	contact_point_world_vel = CalcPointVelocity (*model, q, qdot, contact_body_id, contact_point);
	cout << "\tCPvel = " << contact_point_world_vel;

	Vector3d contact_point_world_acc;
	contact_point_world_acc = CalcPointAcceleration (*model, q, qdot, qddot, contact_body_id, contact_point);
	cout << "\tCPacc = " << contact_point_world_acc;

	VectorNd res (size * 2);
	for (i = 0; i < size; i++) {
		res[i] = qdot[i];
		res[i + size] = qddot[i];
	}

//	cout << "     y = " << y << "    " << "res = " << res << endl;

	cout << endl;

	return res;
}

VectorNd rhs_normal (double t, const VectorNd &y) {
	unsigned int i;
	unsigned int size = Q.size();

	VectorNd q (size);
	VectorNd qdot (size);
	VectorNd qddot (size);

	for (i = 0; i < size; i++) {
		q[i] = y[i];
		qdot[i] = y[i + size];
	}

	ForwardDynamics (*model, q, qdot, Tau, qddot);
	contact_point = model->CalcBaseToBodyCoordinates (contact_body_id, Vector3d (Q[0], 0., Q[2]));

	Vector3d contact_point_world;
	contact_point_world = model->CalcBodyToBaseCoordinates(contact_body_id, contact_point);

	cout << "Q = " << Q << "\tCP = " << contact_point_world;

	Vector3d contact_point_world_vel;
	contact_point_world_vel = CalcPointVelocity (*model, q, qdot, contact_body_id, contact_point);
	cout << "\tCPvel = " << contact_point_world_vel;

	Vector3d contact_point_world_acc;
	contact_point_world_acc = CalcPointAcceleration (*model, q, qdot, qddot, contact_body_id, contact_point);
	cout << "\tCPacc = " << contact_point_world_acc << endl;

	assert (0);

	VectorNd res (size * 2);
	for (i = 0; i < size; i++) {
		res[i] = qdot[i];
		res[i + size] = qddot[i];
	}

	return res;
}

void model_update (double delta_time) {
	unsigned int size = Q.size();
	unsigned int i;
	
	VectorNd y (size * 2);

	for (i = 0; i < size; i++) {
		y[i] = Q[i];
		y[i + size] = QDot[i];
	}

	VectorNd ynew (size * 2);
//	delta_time = 0.02;
//	ynew = rk45_integrator (0., delta_time, y, rhs_normal, 1.0e-3);
//	ynew = rk4_integrator (0., delta_time, y, rhs_contact, 5.0e-2);
//	ynew = euler_integrator (0., delta_time, y, rhs_normal, 1.0e-3);
	ynew = euler_integrator (0., delta_time, y, rhs_contact, 1.0e-3);

//	cout << "        ynew = " << ynew << endl;

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
