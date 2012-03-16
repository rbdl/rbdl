/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <cstring>
#include <assert.h>

#include "rbdl_mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"

namespace RigidBodyDynamics {

using namespace Math;

void UpdateKinematics (Model &model,
		const VectorNd &Q,
		const VectorNd &QDot,
		const VectorNd &QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	unsigned int i;

	if (model.experimental_floating_base) {
		assert (0 && !"UpdateKinematics not supported yet for experimental floating bases");
	}
	
	// Copy positions, velocities, and accelerations while taking account for
	// fixed joints
	CopyDofVectorToModelStateVector (model, model.q, Q);
	CopyDofVectorToModelStateVector (model, model.qdot, QDot);
	CopyDofVectorToModelStateVector (model, model.qddot, QDDot);

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	model.a[0].setZero();
	//model.a[0] = spatial_gravity;

	for (i = 1; i < model.mBodies.size(); i++) {
		SpatialTransform X_J;
		SpatialVector v_J;
		SpatialVector c_J;
		Joint joint = model.mJoints[i];
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, X_J, model.S[i], v_J, c_J, model.q[i], model.qdot[i]);

		model.X_lambda[i] = X_J * model.X_T[i];

		if (lambda != 0) {
			model.X_base[i] = model.X_lambda[i] * model.X_base.at(lambda);
			model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
			model.c[i] = c_J + crossm(model.v[i],v_J);
		}	else {
			model.X_base[i] = model.X_lambda[i];
			model.v[i] = v_J;
			model.c[i].setZero();
		}
		
		model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];
		model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
	}

	for (i = 1; i < model.mBodies.size(); i++) {
		LOG << "a[" << i << "] = " << model.a[i].transpose() << std::endl;
	}

}

void UpdateKinematicsCustom (Model &model,
		const VectorNd *Q,
		const VectorNd *QDot,
		const VectorNd *QDDot
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	unsigned int i;

	if (model.experimental_floating_base) {
		assert (0 && !"UpdateKinematics not supported yet for experimental floating bases");
	}

	if (Q) {
		CopyDofVectorToModelStateVector (model, model.q, *Q);
	}

	if (QDot) {
		CopyDofVectorToModelStateVector (model, model.qdot, *QDot);
	}

	if (QDDot) {
		CopyDofVectorToModelStateVector (model, model.qddot, *QDDot);
	}

	if (Q) {
		for (i = 1; i < model.mBodies.size(); i++) {
			SpatialVector v_J;
			SpatialVector c_J;
			SpatialTransform X_J;
			Joint joint = model.mJoints[i];
			unsigned int lambda = model.lambda[i];

			jcalc (model, i, X_J, model.S[i], v_J, c_J, model.q[i], model.qdot[i]);

			model.X_lambda[i] = X_J * model.X_T[i];

			if (lambda != 0) {
				model.X_base[i] = model.X_lambda[i] * model.X_base.at(lambda);
			}	else {
				model.X_base[i] = model.X_lambda[i];
			}
		}
	}

	if (QDot) {
		for (i = 1; i < model.mBodies.size(); i++) {
			SpatialVector v_J;
			SpatialVector c_J;
			SpatialTransform X_J;
			Joint joint = model.mJoints[i];
			unsigned int lambda = model.lambda[i];

			jcalc (model, i, X_J, model.S[i], v_J, c_J, model.q[i], model.qdot[i]);

			if (lambda != 0) {
				model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + v_J;
				model.c[i] = c_J + crossm(model.v[i],v_J);
			}	else {
				model.v[i] = v_J;
				model.c[i].setZero();
			}
		}
	}

	if (QDDot) {
		for (i = 1; i < model.mBodies.size(); i++) {
			unsigned int lambda = model.lambda[i];

			if (lambda != 0) {
				model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];
			}	else {
				model.a[i].setZero();
			}

			model.a[i] = model.a[i] + model.S[i] * model.qddot[i];
		}
	}
}

Vector3d CalcBodyToBaseCoordinates (
		Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_body_coordinates,
		bool update_kinematics) {
	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	Matrix3d body_rotation = model.X_base[body_id].E.transpose();
	Vector3d body_position = model.X_base[body_id].r;

	return body_position + body_rotation * point_body_coordinates;
}

Vector3d CalcBaseToBodyCoordinates (
		Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_base_coordinates,
		bool update_kinematics) {
	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	Matrix3d body_rotation = model.X_base[body_id].E;
	Vector3d body_position = model.X_base[body_id].r;

	return body_rotation * point_base_coordinates - body_rotation * body_position;
}

Matrix3d CalcBodyWorldOrientation (
		Model &model,
		const VectorNd &Q,
		const unsigned int body_id,
		bool update_kinematics) 
{
	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	// We use the information from the X_base vector. In the upper left 3x3
	// matrix contains the orientation as a 3x3 matrix which we are asking
	// for.
	return model.X_base[body_id].E;
}

void CalcPointJacobian (
		Model &model,
		const VectorNd &Q,
		unsigned int body_id,
		const Vector3d &point_position,
		MatrixNd &G,
		bool update_kinematics
	) {
	LOG << "-------- " << __func__ << " --------" << std::endl;
	if (model.experimental_floating_base) {
		assert (0 && "CalcPointJacobian() not yet supported for experimental floating base models");
	};

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustom (model, &Q, NULL, NULL);
	}

	Vector3d point_base_pos = CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false);
	SpatialMatrix point_trans = Xtrans_mat (point_base_pos);

	assert (G.rows() == 3 && G.cols() == model.dof_count );

	G.setZero();

	// we have to make sure that only the joints that contribute to the
	// bodies motion also get non-zero columns in the jacobian.
	//VectorNd e = VectorNd::Zero(Q.size() + 1);

	unsigned int j = body_id;

	char e[(Q.size() + 1)];
	memset (&e[0], 0, Q.size() + 1);

	// e will contain 
	while (j != 0) {
		e[j] = 1;
		j = model.lambda[j];
	}

	for (j = 1; j < model.mBodies.size(); j++) {
		if (e[j] == 1) {
			SpatialVector S_base;
			S_base = point_trans * spatial_inverse(model.X_base[j].toMatrix()) * model.S[j];

			G(0, j - 1) = S_base[3];
			G(1, j - 1) = S_base[4];
			G(2, j - 1) = S_base[5];
		}
	}
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
		assert (0 && !"floating base not supported");
	}
		
	assert (body_id > 0 && body_id < model.mBodies.size());
	assert (model.q.size() == Q.size() + 1);
	assert (model.qdot.size() == QDot.size() + 1);

	// Reset the velocity of the root body
	model.v[0].setZero();

	// update the Kinematics with zero acceleration
	if (update_kinematics) {
		VectorNd QDDot_zero = VectorNd::Zero(Q.size());
		
		UpdateKinematics (model, Q, QDot, QDDot_zero);
	}

	Vector3d point_abs_pos = CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false); 

	LOG << "body_index     = " << body_id << std::endl;
	LOG << "point_pos      = " << point_position << std::endl;
//	LOG << "global_velo    = " << global_velocities.at(body_id) << std::endl;
	LOG << "body_transf    = " << model.X_base[body_id].toMatrix() << std::endl;
	LOG << "point_abs_ps   = " << point_abs_pos << std::endl;
	LOG << "X   = " << Xtrans_mat (point_abs_pos) * spatial_inverse(model.X_base[body_id].toMatrix()) << std::endl;
	LOG << "v   = " << model.v[body_id] << std::endl;

	// Now we can compute the spatial velocity at the given point
//	SpatialVector body_global_velocity (global_velocities.at(body_id));
	SpatialVector point_spatial_velocity = Xtrans_mat (point_abs_pos) * spatial_inverse(model.X_base[body_id].toMatrix()) * model.v[body_id];

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
	model.v[0].setZero();
	model.a[0].setZero();

	if (model.experimental_floating_base) {
		assert (0 && !"floating base not supported");
	}

	if (update_kinematics)
		UpdateKinematics (model, Q, QDot, QDDot);

	LOG << std::endl;

	// The whole computation looks in formulae like the following:
	SpatialVector body_global_velocity (spatial_inverse(model.X_base[body_id].toMatrix()) * model.v[body_id]);

	LOG << " orientation " << std::endl << CalcBodyWorldOrientation (model, Q, body_id, false) << std::endl;
	LOG << " orientationT " << std::endl <<  CalcBodyWorldOrientation (model, Q, body_id, false).transpose() << std::endl;

	Matrix3d global_body_orientation_inv = CalcBodyWorldOrientation (model, Q, body_id, false).transpose();
	SpatialMatrix p_X_i = SpatialMatrixZero;

	p_X_i.block<3,3>(0,0) = global_body_orientation_inv;
	p_X_i.block<3,3>(3,3) = global_body_orientation_inv;

	LOG << " p_X_i = " << std::endl << p_X_i << std::endl;
	LOG << " xtrans = " << std::endl << Xtrans (point_position) << std::endl;

	p_X_i *= Xtrans_mat (point_position);

	SpatialVector p_v_i = p_X_i * model.v[body_id];
	SpatialVector p_a_i = p_X_i * model.a[body_id];

	SpatialVector frame_acceleration = 
		crossm( SpatialVector(0., 0., 0., p_v_i[3], p_v_i[4], p_v_i[5]), (body_global_velocity));

	LOG << model.X_base[body_id] << std::endl;
	LOG << "v_i                = " << model.v[body_id] << std::endl;
	LOG << "a_i                = " << model.a[body_id] << std::endl;
	LOG << "p_X_i              = " << p_X_i << std::endl;
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

bool InverseKinematics (
		Model &model,
		const VectorNd &Qinit,
		const std::vector<unsigned int>& body_id,
		const std::vector<Vector3d>& body_point,
		const std::vector<Vector3d>& target_pos,
		VectorNd &Qres,
		double step_tol,
		double lambda,
		unsigned int max_iter
		) {

	assert (Qinit.size() == model.dof_count);
	assert (body_id.size() == body_point.size());
	assert (body_id.size() == target_pos.size());

	MatrixNd J = MatrixNd::Zero(3 * body_id.size(), model.dof_count);
	VectorNd e = VectorNd::Zero(3 * body_id.size());

	Qres = Qinit;

	for (int ik_iter = 0; ik_iter < max_iter; ik_iter++) {
		UpdateKinematicsCustom (model, &Qres, NULL, NULL);

		for (unsigned int k = 0; k < body_id.size(); k++) {
			MatrixNd G (3, model.dof_count);
			CalcPointJacobian (model, Qres, body_id[k], body_point[k], G, false);
			Vector3d point_base = CalcBodyToBaseCoordinates (model, Qres, body_id[k], body_point[k], false);
			LOG << "current_pos = " << point_base.transpose() << std::endl;

			for (unsigned int i = 0; i < 3; i++) {
				for (unsigned int j = 0; j < model.dof_count; j++) {
					unsigned int row = k * 3 + i;
					LOG << "i = " << i << " j = " << j << " k = " << k << " row = " << row << " col = " << j << std::endl;
					J(row, j) = G (i,j);
				}

				e[k * 3 + i] = target_pos[k][i] - point_base[i];
			}

			LOG << J << std::endl;

			// abort if we are getting "close"
			if (e.norm() < step_tol) {
				LOG << "Reached target close enough after " << ik_iter << " steps" << std::endl;
				return true;
			}
		}

		LOG << "J = " << J << std::endl;
		LOG << "e = " << e.transpose() << std::endl;

		MatrixNd JJTe_lambda2_I = J * J.transpose() + lambda*lambda * MatrixNd::Identity(e.size(), e.size());

		VectorNd z (body_id.size() * 3);
#ifndef RBDL_USE_SIMPLE_MATH
		z = JJTe_lambda2_I.colPivHouseholderQr().solve (e);
#else
		bool solve_successful = LinSolveGaussElimPivot (JJTe_lambda2_I, e, z);
		assert (solve_successful);
#endif

		LOG << "z = " << z << std::endl;

		VectorNd delta_theta = J.transpose() * z;
		LOG << "change = " << delta_theta << std::endl;

		Qres = Qres + delta_theta;
		LOG << "Qres = " << Qres.transpose() << std::endl;

		if (delta_theta.norm() < step_tol) {
			LOG << "reached convergence after " << ik_iter << " steps" << std::endl;
			return true;
		}

		VectorNd test_1 (z.size());
		VectorNd test_res (z.size());

		test_1.setZero();

		for (unsigned int i = 0; i < z.size(); i++) {
			test_1[i] = 1.;

			VectorNd test_delta = J.transpose() * test_1;

			test_res[i] = test_delta.squaredNorm();

			test_1[i] = 0.;
		}

		LOG << "test_res = " << test_res.transpose() << std::endl;
	}

	return false;
}

}
