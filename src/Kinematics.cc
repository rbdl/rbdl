#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"

using namespace SpatialAlgebra;

/** \brief Computes the velocity of a point on a body 
 *
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

/** \brief Computes the acceleration of a point on a body 
 *
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
	if (model.floating_base) {
		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase
		assert (0);

		// ForwardDynamicsFloatingBase(model, Q, QDot, Tau, QDDot);
		return;
	}

	unsigned int i;
	
	// Copy state values from the input to the variables in model
	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);

	for (i = 0; i < Q.size(); i++) {
		model.q[i+1] = Q[i];
		model.qdot[i+1] = QDot[i];
	}

	// Reset the velocity of the root body
	model.v[0].zero();

	// this will contain the global velocities of the bodies
	std::vector<SpatialVector> global_velocities (model.mBodies.size() + 1, SpatialVector(0., 0., 0., 0., 0., 0.));

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

	Vector3d point_abs_pos = body_translation + body_rotation * point_position;

	LOG << "body_index   = " << body_id << std::endl;
	LOG << "global_velo  = " << global_velocities.at(body_id) << std::endl;
	LOG << "body_transf  = " << model.X_base[body_id] << std::endl;
	LOG << "body_rotation= " << std::endl << body_rotation << std::endl;
	LOG << "body_tranlat = " << body_translation << std::endl;
	LOG << "point_abs_ps = " << point_abs_pos << std::endl;

	// Now we can compute the spatial velocity at the given point
	SpatialVector body_global_velocity (global_velocities.at(body_id));
	SpatialVector point_spatial_velocity = Xtrans (point_abs_pos * -1.) * body_global_velocity;

	point_velocity.set (
			point_spatial_velocity[3],
			point_spatial_velocity[4],
			point_spatial_velocity[5]
			);
	LOG << "point_velocity = " << point_velocity << std::endl;
}

void CalcPointAccelerationOld (
		Model &model,
		const cmlVector &Q,
		const cmlVector &QDot,
		const cmlVector &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
		Vector3d &point_acceleration
		) {
	if (model.floating_base) {
		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase
		assert (0);

		// ForwardDynamicsFloatingBase(model, Q, QDot, Tau, QDDot);
		return;
	}

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

	// this will contain the global accelerations of the bodies
	std::vector<SpatialVector> global_accelerations (
			model.mBodies.size() + 1,
			SpatialVector(0., 0., 0., 0., 0., 0.)
			);
	
	// this will contain the global velocities of the bodies
	std::vector<SpatialVector> global_velocities (
			model.mBodies.size() + 1,
			SpatialVector(0., 0., 0., 0., 0., 0.)
			);

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

		// we need to compute the global velocities as they may contribute to the
		// acceleration (e.g. in rotational motions where there is always an
		// acceleration towards the center
		global_velocities.at(i) = global_velocities.at(lambda) + model.X_base.at(i).inverse() * v_J;
		LOG << "^0v (" << i << "): " << global_velocities.at(i) << std::endl;

		// v_J = S_i * qdot
		global_accelerations.at(i) = global_accelerations.at(lambda) + model.X_base.at(i).inverse() * model.S.at(i) * model.qddot[i];
		LOG << "^0a (" << i << "): " << global_accelerations.at(i) << std::endl;

		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	LOG << std::endl;

	// we now compute the transformation from the local to the global frame of
	// the body. We split this up into the translation and rotation of the body
	
	// the rotation is the transpose of the rotation part (upper left) of
	// X_base[i].
	Matrix3d body_rotation (model.X_base[body_id].get_rotation().transpose());
	// the translation is the bottom left part which still has to be transformed
	// into a global translation
	Vector3d body_translation (body_rotation * model.X_base[body_id].get_translation() * -1.);

	// computation of the global position of the point
	Vector3d point_abs_pos = body_translation + body_rotation * point_position;
	LOG << "point_abs_ps = " << point_abs_pos << std::endl;

	// The whole computation looks in formulae like the following:
	SpatialVector body_global_velocity (global_velocities.at(body_id));
	SpatialVector body_global_acceleration (global_accelerations.at(body_id));
	SpatialMatrix point_transform (Xtrans (-point_abs_pos));

	// The derivation for this formula can be found in
	// doc/notes/point_velocity_acceleration.tex
	SpatialVector first = body_global_velocity.crossf() * (point_transform * body_global_velocity);
	SpatialVector second = point_transform * body_global_acceleration;
	SpatialVector point_spatial_accel = first + second;

	LOG << "first = " << first << std::endl;
	LOG << "scnd  = " << second << std::endl;
	LOG << "point_spatial_accel = " << point_spatial_accel << std::endl;
	
	point_acceleration.set (point_spatial_accel[3], point_spatial_accel[4], point_spatial_accel[5]);
	LOG << "point_acceleration = " << point_acceleration <<  std::endl;
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
	CalcPointAccelerationOld (model, Q, QDot, QDDot, body_id, point_position, point_acceleration);
	return;

	if (model.floating_base) {
		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase
		assert (0);

		// ForwardDynamicsFloatingBase(model, Q, QDot, Tau, QDDot);
		return;
	}

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

	// this will contain the global accelerations of the bodies
	std::vector<SpatialVector> global_accelerations (
			model.mBodies.size() + 1,
			SpatialVector(0., 0., 0., 0., 0., 0.)
			);
	
	// this will contain the global velocities of the bodies
	std::vector<SpatialVector> global_velocities (
			model.mBodies.size() + 1,
			SpatialVector(0., 0., 0., 0., 0., 0.)
			);

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

		// we need to compute the global velocities as they may contribute to the
		// acceleration (e.g. in rotational motions where there is always an
		// acceleration towards the center
		global_velocities.at(i) = global_velocities.at(lambda) + model.X_base.at(i).inverse() * v_J;
		LOG << "^0v (" << i << "): " << global_velocities.at(i) << std::endl;

		// v_J = S_i * qdot
		global_accelerations.at(i) = global_accelerations.at(lambda) + model.X_base.at(i).inverse() * model.S.at(i) * model.qddot[i];
		LOG << "^0a (" << i << "): " << global_accelerations.at(i) << std::endl;

		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	LOG << std::endl;

	// we now compute the transformation from the local to the global frame of
	// the body. We split this up into the translation and rotation of the body
	
	// the rotation is the transpose of the rotation part (upper left) of
	// X_base[i].
	Matrix3d body_rotation (model.X_base[body_id].get_rotation().transpose());
	// the translation is the bottom left part which still has to be transformed
	// into a global translation
	Vector3d body_translation (body_rotation * model.X_base[body_id].get_translation() * -1.);

	// computation of the global position of the point
	Vector3d point_abs_pos = body_translation + body_rotation * point_position;
	LOG << "point_abs_ps = " << point_abs_pos << std::endl;

	// The whole computation looks in formulae like the following:
	SpatialVector body_global_velocity (global_velocities.at(body_id));
	SpatialVector body_global_acceleration (global_accelerations.at(body_id));
	// new method
	SpatialMatrix point_trans = model.X_base[body_id];

	unsigned int j;
	for (i = 3; i < 6; i++) {
		for (j = 0; j < 3; j++) {
			point_trans(i,j) = 0.;
		}
	};
	LOG << "point_trans.            = " << point_trans << std::endl;
	point_trans.transpose();
	LOG << "point_trans.transpose() = " << point_trans << std::endl;
	point_trans = point_trans * Xtrans (point_position);

	SpatialVector pvi = point_trans * model.v[body_id];
	SpatialVector pai = point_trans * model.a[body_id];

	Vector3d omega (
			body_global_velocity[0],
			body_global_velocity[1],
			body_global_velocity[2]
			);
	SpatialVector point_spatial_velocity = Xtrans (point_abs_pos * -1.) * body_global_velocity;
	Vector3d point_velocity (
			point_spatial_velocity[3],
			point_spatial_velocity[4],
			point_spatial_velocity[5]
			);
//	CalcPointVelocity (model, Q, QDot, body_id, point_position, point_velocity);

	Vector3d bottom = cml::cross(omega, point_velocity);
	SpatialVector other (0., 0., 0.,
			bottom[0], bottom[1], bottom[2]);

	LOG << "pai pre = " << pai << std::endl;
	LOG << "other   = " << other << std::endl;
	pai += other;
	LOG << "pai pos = " << pai << std::endl;
	point_acceleration.set(pai[3], pai[4], pai[5]);
	LOG << "point_acceleration new = " << point_acceleration << std::endl;
}

