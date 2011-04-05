#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"

using namespace SpatialAlgebra;

namespace RigidBodyDynamics {

/*
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param point_velocity cartesian velocity of the point in global frame (output)
 */
void CalcPointVelocity (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_velocity
		) {
	cmlVector cmlQ (Q.size());
	cmlVector cmlQDot (QDot.size());

	unsigned int i;
	for (i = 0; i < Q.size(); i++)
		cmlQ[i] = Q[i];

	for (i = 0; i < QDot.size(); i++)
		cmlQDot[i] = QDot[i];

	CalcPointVelocity( model, cmlQ, cmlQDot, body_id, point_position, point_velocity);
}

/*
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param QDot    velocity vector of the internal joints
 * \param QDDot    velocity vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param point_acceleration cartesian velocity of the point in global frame (output)
 */
void CalcPointAcceleration (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_acceleration
		) {
	cmlVector cmlQ (Q.size());
	cmlVector cmlQDot (QDot.size());
	cmlVector cmlQDDot (QDDot.size());

	unsigned int i;
	for (i = 0; i < Q.size(); i++) 
		cmlQ[i] = Q[i];

	for (i = 0; i < QDot.size(); i++)
		cmlQDot[i] = QDot[i];

	for (i = 0; i < QDDot.size(); i++)
		cmlQDDot[i] = QDDot[i];

	CalcPointAcceleration( model, cmlQ, cmlQDot, cmlQDDot, body_id, point_position, point_acceleration);
}

void CalcPointVelocity (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_velocity
	) {
	LOG << "-------- " << __func__ << " --------" << std::endl;
	unsigned int i;

	// this will contain the global velocities of the bodies
	std::vector<SpatialVector> global_velocities (
			model.mBodies.size() + 1,
			SpatialVector(0., 0., 0., 0., 0., 0.)
			);

	if (model.experimental_floating_base) {
		// set the transformation for the base body
		model.X_base[0] = XtransRotZYXEuler (Vector3d (Q[0], Q[1], Q[2]), Vector3d (Q[3], Q[4], Q[5]));
		model.X_lambda[0] = model.X_base[0];

		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase
		model.v[0].set (QDot[5], QDot[4], QDot[3], QDot[0], QDot[1], QDot[2]);

		model.v[0] = model.X_base[0].inverse() * model.v[0];
		
		global_velocities[0] = model.v[0];

		if (model.mBodies.size() > 1) {
			// Copy state values from the input to the variables in model
			for (i = 1; i < model.mBodies.size(); i++) {
				model.q.at(i) = Q[6 + i];
				model.qdot.at(i) = QDot[6 + i];
			}
		}
	} else {
		assert (body_id > 0 && body_id < model.mBodies.size());
		assert (model.q.size() == Q.size() + 1);
		assert (model.qdot.size() == QDot.size() + 1);

		// Reset the velocity of the root body
		model.v[0].zero();

		// Copy state values from the input to the variables in model
		for (i = 0; i < Q.size(); i++) {
			model.q.at(i+1) = Q[i];
			model.qdot.at(i+1) = QDot[i];
		}
	}
	
	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_J, model.S.at(i), v_J, c_J, model.q.at(i), model.qdot.at(i));
		LOG << "q(" << i << "):" << model.q.at(i) << std::endl;

		model.X_lambda.at(i) = X_J * model.X_T.at(i);

		if (lambda != 0)
			model.X_base.at(i) = model.X_lambda.at(i) * model.X_base.at(lambda);
		else
			model.X_base.at(i) = model.X_lambda.at(i);

		LOG << "X_J (" << i << "):" << X_J << std::endl;
		LOG << "v_J (" << i << "):" << v_J << std::endl;
		LOG << "X_base (" << i << "):" << model.X_base.at(i) << std::endl;
		model.v.at(i) = model.X_lambda.at(i) * model.v.at(lambda) + v_J;
		global_velocities.at(i) = global_velocities.at(lambda) + model.X_base.at(i).inverse() * v_J;
		LOG << "^0v (" << i << "): " << global_velocities.at(i) << std::endl;
	}

	LOG << std::endl;

	// First we need the rotation and translation of the body to compute the
	// global position of the point we are interested in
	// global position
	Matrix3d body_rotation;
	Vector3d body_translation;

	// the rotation is the transpose of the rotation part (upper left) of
	// X_base[i].
	body_rotation = model.X_base[body_id].get_rotation().transpose();
	// the translation is the bottom left part which still has to be transformed
	// into a global translation
	body_translation = body_rotation * model.X_base[body_id].get_translation() * -1.;

	Vector3d point_abs_pos = model.CalcBodyToBaseCoordinates(body_id, point_position); 

	LOG << "body_index     = " << body_id << std::endl;
	LOG << "point_pos      = " << point_position << std::endl;
	LOG << "global_velo    = " << global_velocities.at(body_id) << std::endl;
	LOG << "body_transf    = " << model.X_base[body_id] << std::endl;
	LOG << "body_rotation  = " << std::endl << body_rotation << std::endl;
	LOG << "body_translat  = " << body_translation << std::endl;
	LOG << "point_abs_ps   = " << point_abs_pos << std::endl;

	// Now we can compute the spatial velocity at the given point
	SpatialVector body_global_velocity (global_velocities.at(body_id));
	SpatialVector point_spatial_velocity = Xtrans (point_abs_pos) * body_global_velocity;

	point_velocity.set (
			point_spatial_velocity[3],
			point_spatial_velocity[4],
			point_spatial_velocity[5]
			);
	LOG << "point_velocity = " << point_velocity << std::endl;
}

void CalcPointAccelerationFeatherstone (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_acceleration
		)
{
	LOG << "-------- " << __func__ << " --------" << std::endl;
	unsigned int i;

	// Copy state values from the input to the variables in model
	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);
	assert (model.qddot.size() == QDDot.size() + 1);

	for (i = 0; i < Q.size(); i++) {
		model.q[i+1] = Q[i];
		model.qdot[i+1] = QDot[i];
		model.qddot[i+1] = QDDot[i];
	}

	// Reset the velocity of the root body
	model.v[0].zero();
	model.a[0].zero();

	if (model.experimental_floating_base) {
		// set the transformation for the base body
		model.X_base[0] = XtransRotZYXEuler (Vector3d (Q[0], Q[1], Q[2]), Vector3d (Q[3], Q[4], Q[5]));
		model.X_lambda[0] = model.X_base[0];

		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase.
		model.v[0].set (QDot[5], QDot[4], QDot[3], QDot[0], QDot[1], QDot[2]);
		model.a[0].set (QDDot[5], QDDot[4], QDDot[3], QDDot[0], QDDot[1], QDDot[2]);
		
		if (model.mBodies.size() > 1) {
			// Copy state values from the input to the variables in model
			for (i = 1; i < model.mBodies.size(); i++) {
				model.q.at(i) = Q[6 + i];
				model.qdot.at(i) = QDot[6 + i];
				model.qddot.at(i) = QDDot[6 + i];
			}
		}
	} else {
		assert (model.q.size() == Q.size() + 1);
		assert (model.qdot.size() == QDot.size() + 1);
		assert (model.qddot.size() == QDDot.size() + 1);

		// Reset the velocity of the root body
		model.v[0].zero();
		model.a[0].zero();

		// Copy state values from the input to the variables in model
		for (i = 0; i < Q.size(); i++) {
			model.q.at(i+1) = Q[i];
			model.qdot.at(i+1) = QDot[i];
			model.qddot.at(i+1) = QDDot[i];
		}
	}

	LOG << "qdot = ";
	for (i = 0; i < model.qdot.size(); i++) {
		LOG << model.qdot[i] << " ";
	}
	LOG << std::endl;

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_J, model.S.at(i), v_J, c_J, model.q.at(i), model.qdot.at(i));

		model.X_lambda.at(i) = X_J * model.X_T.at(i);

		if (lambda != 0) {
			model.X_base.at(i) = model.X_lambda.at(i) * model.X_base.at(lambda);
			model.v[i] = model.X_lambda[i] * model.v[lambda] + v_J;
			model.c.at(i) = c_J + model.v.at(i).crossm() * v_J;
			model.a[i] = model.X_lambda[i] * model.a[lambda] + model.c[i];
		}	else {
			model.X_base.at(i) = model.X_lambda.at(i);
			model.v[i] = v_J;
			model.c[i] = SpatialVectorZero;
			model.a[i] = SpatialVectorZero;
		}

		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];

		// c_i = c_J + v x v_J
		//     = c_J + v_i x S_i . qdot
		//     = \dot{S}_i . qdot
		//     (if S_i is moving with link i)

		// a_i = ^a_{i-1} + \dot{s}_i \dot{q}_i + s_i + \ddot{q}_i
		SpatialVector dot_si_qdot = model.v[i].crossm() * model.S[i] * model.qdot[i];
		SpatialVector si_qddot = model.S[i] * model.qddot[i];

		LOG << "a[" << i << "] = " << model.a[i] << std::endl;
	}

	LOG << std::endl;

	// computation of the global position of the point
	Vector3d point_abs_pos = model.CalcBodyToBaseCoordinates (body_id, point_position);
	LOG << "point_abs_ps = " << point_abs_pos << std::endl;

	// The whole computation looks in formulae like the following:
	SpatialVector body_global_velocity (model.X_base[body_id].inverse() * model.v[body_id]);
	SpatialVector body_global_acceleration (model.X_base[body_id].inverse() * model.a[body_id]);
	SpatialMatrix global_point_transform (Xtrans (point_abs_pos));
	SpatialMatrix local_point_transform (Xtrans (point_position));

	Matrix3d global_body_orientation_inv = model.GetBodyWorldOrientation (body_id).inverse();
	SpatialMatrix p_X_i = SpatialMatrix (global_body_orientation_inv, Matrix3dZero,
			Matrix3dZero, global_body_orientation_inv) * Xtrans (point_position);

	SpatialVector p_v_i = p_X_i * model.v[body_id];
	SpatialVector p_a_i = p_X_i * model.a[body_id];

	SpatialVector frame_acceleration = SpatialVector(0., 0., 0.,
			p_v_i[3], p_v_i[4], p_v_i[5]
			).crossm() * body_global_velocity;

	LOG << model.X_base[body_id] << std::endl;
//	LOG << "p_X_i              = " << p_X_i << std::endl;
	LOG << "p_v_i              = " << p_v_i << std::endl;
	LOG << "p_a_i              = " << p_a_i << std::endl;
	LOG << "body_global_vel    = " << body_global_velocity << std::endl;
	LOG << "frame_acceleration = " << frame_acceleration << std::endl;

	SpatialVector p_a_i_dash = p_a_i - frame_acceleration;

	point_acceleration.set(
			p_a_i_dash[3],
			p_a_i_dash[4],
			p_a_i_dash[5]
			);

	LOG << std::endl << "point_acceleration = " << point_acceleration << std::endl;
}

void CalcPointAccelerationDirect (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_acceleration
		)
{
	LOG << "-------- " << __func__ << " --------" << std::endl;

	unsigned int i;

	LOG << "Q = " << Q << std::endl;
	LOG << "QDot = " << QDot << std::endl;
	LOG << "QDDot = " << QDDot << std::endl;

	// this will contain the global velocities of the bodies
	std::vector<SpatialVector> global_velocities (
			model.mBodies.size() + 1,
			SpatialVector(0., 0., 0., 0., 0., 0.)
			);

	if (model.experimental_floating_base) {
		// set the transformation for the base body
		model.X_base[0] = XtransRotZYXEuler (Vector3d (Q[0], Q[1], Q[2]), Vector3d (Q[3], Q[4], Q[5]));
		model.X_lambda[0] = model.X_base[0];

		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase.
		model.v[0].set (QDot[5], QDot[4], QDot[3], QDot[0], QDot[1], QDot[2]);
		model.a[0].set (QDDot[5], QDDot[4], QDDot[3], QDDot[0], QDDot[1], QDDot[2]);
		
		global_velocities[0] = model.v[0];

		if (model.mBodies.size() > 1) {
			// Copy state values from the input to the variables in model
			for (i = 1; i < model.mBodies.size(); i++) {
				model.q.at(i) = Q[6 + i];
				model.qdot.at(i) = QDot[6 + i];
				model.qddot.at(i) = QDDot[6 + i];
			}
		}
	} else {
		assert (model.q.size() == Q.size() + 1);
		assert (model.qdot.size() == QDot.size() + 1);
		assert (model.qddot.size() == QDDot.size() + 1);

		// Reset the velocity of the root body
		model.v[0].zero();
		model.a[0].zero();

		// Copy state values from the input to the variables in model
		for (i = 0; i < Q.size(); i++) {
			model.q.at(i+1) = Q[i];
			model.qdot.at(i+1) = QDot[i];
			model.qddot.at(i+1) = QDDot[i];
		}
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_J, model.S.at(i), v_J, c_J, model.q.at(i), model.qdot.at(i));
		LOG << "q(" << i << "):" << model.q.at(i) << std::endl;

		model.X_lambda.at(i) = X_J * model.X_T.at(i);

		if (lambda != 0)
			model.X_base.at(i) = model.X_lambda.at(i) * model.X_base.at(lambda);
		else
			model.X_base.at(i) = model.X_lambda.at(i);

		LOG << "X_J (" << i << "):" << X_J << std::endl;
		LOG << "v_J (" << i << "):" << v_J << std::endl;
		LOG << "X_base (" << i << "):" << model.X_base.at(i) << std::endl;
		model.v.at(i) = model.X_lambda.at(i) * model.v.at(lambda) + v_J;

		// c_i = c_J + v x v_J
		//     = c_J + v_i x S_i . qdot
		//     = \dot{S}_i . qdot
		//     (if S_i is moving with link i)
		model.c.at(i) = c_J + model.v.at(i).crossm() * v_J;

		// we need to compute the global velocities as they may contribute to the
		// acceleration (e.g. in rotational motions where there is always an
		// acceleration towards the center
		global_velocities.at(i) = global_velocities.at(lambda) + model.X_base.at(i).inverse() * v_J;
		LOG << "^0v (" << i << "): " << global_velocities.at(i) << std::endl;

		// a_i = ^a_{i-1} + \dot{s}_i \dot{q}_i + s_i + \ddot{q}_i
		SpatialVector dot_si_qdot = model.v[i].crossm() * model.S[i] * model.qdot[i];
		LOG << "dot_si_qdot[" << i << "] = " << dot_si_qdot << std::endl;
		SpatialVector si_qddot = model.S[i] * model.qddot[i];
		LOG << "si_qddot   [" << i << "] = " << si_qddot << std::endl;

		model.a[i] = model.X_lambda[i] * model.a[lambda] + model.c[i] + model.S[i] * model.qddot[i];
	}

	LOG << std::endl;

	ClearLogOutput();

	// computation of the global position of the point
	Vector3d point_abs_pos = model.CalcBodyToBaseCoordinates (body_id, point_position);
	LOG << "point_abs_ps = " << point_abs_pos << std::endl;

	// The whole computation looks in formulae like the following:
	SpatialVector body_global_velocity (global_velocities.at(body_id));
	SpatialVector body_global_acceleration (model.X_base[body_id].inverse() * model.a[body_id]);
	SpatialMatrix point_transform (Xtrans (point_abs_pos));

	// The derivation for this formula can be found in
	// doc/notes/point_velocity_acceleration.tex

	// compute the linear velocity of the body (we could call
	// CalcPointVelocity, however we already have all the data here we need)
	SpatialVector point_spatial_velocity = Xtrans (point_abs_pos) * body_global_velocity;
	Vector3d point_lin_velocity (point_spatial_velocity[3], point_spatial_velocity[4], point_spatial_velocity[5]);

	//	CalcPointVelocity (model, Q, QDot, body_id, point_position, point_lin_velocity);

	LOG << "point_transform           = " << point_transform << std::endl;
	LOG << "point_lin_velocity        = " << point_lin_velocity << std::endl;
	LOG << "body_global_velocity      = " << body_global_velocity<< std::endl;
	LOG << "body_global_accelerations = " << body_global_acceleration << std::endl;
	LOG << "body_point_velocity       = " << point_transform * body_global_velocity << std::endl;
	LOG << "body_point_accelerations  = " << point_transform * body_global_acceleration << std::endl;

	SpatialVector frame_acceleration = SpatialVector (0., 0., 0., point_lin_velocity[0], point_lin_velocity[1], point_lin_velocity[2]).crossm() * body_global_velocity;
	LOG << "frame_acceleration        = " << frame_acceleration << std::endl;

	SpatialVector body_accel = point_transform * body_global_acceleration;
	LOG << "^p body_accel             = " << body_accel << std::endl;

	SpatialVector point_spatial_accel = (body_accel - frame_acceleration);
	LOG << "point_spatial_accel       = " << point_spatial_accel << std::endl;

	LOG << std::endl;

	point_acceleration.set (point_spatial_accel[3], point_spatial_accel[4], point_spatial_accel[5]);
	LOG << "point_acceleration        = " << point_acceleration <<  std::endl;
}

void CalcPointAcceleration (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_acceleration
		)
{
//	CalcPointAccelerationDirect (model, Q, QDot, QDDot, body_id, point_position, point_acceleration);
	CalcPointAccelerationFeatherstone (model, Q, QDot, QDDot, body_id, point_position, point_acceleration);
}

}
