#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"

using namespace SpatialAlgebra;

namespace RigidBodyDynamics {

void ForwardKinematics (Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	unsigned int i;

	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);
	assert (model.qddot.size() == QDDot.size() + 1);

	if (model.experimental_floating_base) {
		assert (0 && !"ForwardKinematics not supported yet for experimental floating bases");
	}
	
	// positions
	for (i = 0; i < model.dof_count; i++) {
		model.q[i + 1] = Q[i];
	}

	// velocities
	for (i = 0; i < model.dof_count; i++) {
		model.qdot[i + 1] = QDot[i];
	}

	// accelerations
	for (i = 0; i < model.dof_count; i++) {
		model.qddot[i + 1] = QDDot[i];
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialMatrix X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints.at(i);
		unsigned int lambda = model.lambda.at(i);

		jcalc (model, i, X_J, model.S.at(i), v_J, c_J, model.q[i], model.qdot[i]);

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
	}
}

MatrixNd CalcPointJacobian (
		Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_position,
		bool update_kinematics
	) {
	LOG << "-------- " << __func__ << " --------" << std::endl;
	if (model.experimental_floating_base) {
		assert (0 && "CalcPointJacobian() not yet supported for experimental floating base models");
	};

	// update the Kinematics with zero acceleration
	if (update_kinematics) {
		VectorNd QDDot_zero (Q.size(), 0.);
		VectorNd QDot_zero (Q.size(), 0.);
		ForwardKinematics (model, Q, QDot_zero, QDDot_zero);
	}

	Vector3d point_base_pos = model.CalcBodyToBaseCoordinates(body_id, point_position);
	SpatialMatrix point_trans = Xtrans (point_base_pos);
	MatrixNd result (3, model.dof_count);

	unsigned int j;
	for (j = 1; j < model.mBodies.size(); j++) {
		SpatialVector S_base;
		S_base = point_trans * model.X_base[j].inverse() * model.S[j];

		result(0, j - 1) = S_base[3];
		result(1, j - 1) = S_base[4];
		result(2, j - 1) = S_base[5];
	}

	return result;
}

Vector3d CalcPointVelocity (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		unsigned int body_id,
		const Vector3d &point_position,
		bool update_kinematics
	) {
	LOG << "-------- " << __func__ << " --------" << std::endl;
	unsigned int i;

	if (model.experimental_floating_base) {
		// set the transformation for the base body
		model.X_base[0] = XtransRotZYXEuler (Vector3d (Q[0], Q[1], Q[2]), Vector3d (Q[3], Q[4], Q[5]));
		model.X_lambda[0] = model.X_base[0];

		// in this case the appropriate function has to be called, see
		// ForwardDynamicsFloatingBase
		model.v[0].set (QDot[5], QDot[4], QDot[3], QDot[0], QDot[1], QDot[2]);

		model.v[0] = model.X_base[0].inverse() * model.v[0];
		
		// global_velocities[0] = model.v[0];

		if (model.mBodies.size() > 1) {
			// Copy state values from the input to the variables in model
			for (i = 1; i < model.mBodies.size(); i++) {
				model.q[i] = Q[6 + i];
				model.qdot[i] = QDot[6 + i];
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
			model.q[i+1] = Q[i];
			model.qdot[i+1] = QDot[i];
		}
	}

	// update the Kinematics with zero acceleration
	if (update_kinematics) {
		VectorNd QDDot_zero (Q.size(), 0.);
		ForwardKinematics (model, Q, QDot, QDDot_zero);
	}

	Vector3d point_abs_pos = model.CalcBodyToBaseCoordinates(body_id, point_position); 

	LOG << "body_index     = " << body_id << std::endl;
	LOG << "point_pos      = " << point_position << std::endl;
//	LOG << "global_velo    = " << global_velocities.at(body_id) << std::endl;
	LOG << "body_transf    = " << model.X_base[body_id] << std::endl;
	LOG << "point_abs_ps   = " << point_abs_pos << std::endl;

	// Now we can compute the spatial velocity at the given point
//	SpatialVector body_global_velocity (global_velocities.at(body_id));
	SpatialVector point_spatial_velocity = Xtrans (point_abs_pos) * model.X_base[body_id].inverse() * model.v[body_id];

	LOG << "point_velocity = " <<	Vector3d (
			point_spatial_velocity[3],
			point_spatial_velocity[4],
			point_spatial_velocity[5]
			) << std::endl;

	return Vector3d (
			point_spatial_velocity[3],
			point_spatial_velocity[4],
			point_spatial_velocity[5]
			);
}

Vector3d CalcPointAcceleration (
		Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot,
		unsigned int body_id,
		const Vector3d &point_position,
			bool update_kinematics
	)
{
	LOG << "-------- " << __func__ << " --------" << std::endl;
	unsigned int i;

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
				model.q[i] = Q[6 + i];
				model.qdot[i] = QDot[6 + i];
				model.qddot[i] = QDDot[6 + i];
			}
		}
	} else {
		if (update_kinematics)
			ForwardKinematics (model, Q, QDot, QDDot);
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

	LOG << "point_acceleration = " <<	Vector3d (
			p_a_i_dash[3],
			p_a_i_dash[4],
			p_a_i_dash[5]
			) << std::endl;

	return Vector3d (
			p_a_i_dash[3],
			p_a_i_dash[4],
			p_a_i_dash[5]
			);
}

}
