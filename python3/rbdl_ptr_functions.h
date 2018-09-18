/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 *
 * This file defines functions that allows calling of the RBDL algorithms
 * by providing input and output as raw double arrays. It eliminates the
 * need of copying from Numpy values into temporary RBDL (C++) vectors and
 * matrices. However it requires C++11 and must be compiled with -std=c++11
 * (or -std=c++0x on older compilers).
 */

#include <rbdl/rbdl_math.h>
#include <rbdl/Dynamics.h>

namespace RigidBodyDynamics {

namespace Math {

// PTR_DATA_ROW_MAJOR :
// Specifies whether the data that is provided via raw double pointers is
// stored as row major. Eigen uses column major by default and therefore
// this has to be properly mapped.
#define PTR_DATA_ROW_MAJOR 1

#ifdef RBDL_USE_SIMPLE_MATH
	typedef VectorNd VectorNdRef;
	typedef MatrixNd MatrixNdRef;
#else
	typedef Eigen::Ref<Eigen::VectorXd> VectorNdRef;

#ifdef PTR_DATA_ROW_MAJOR
	typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixNdRowMaj;
	typedef Eigen::Ref<MatrixNdRowMaj> MatrixNdRef;
#else
	typedef Eigen::Ref<Eigen::MatrixXd> MatrixNdRef;
#endif

#endif

RBDL_DLLAPI inline VectorNdRef VectorFromPtr (double *ptr, unsigned int n) {
#ifdef RBDL_USE_SIMPLE_MATH
	return SimpleMath::Map<VectorNd> (ptr, n, 1);
#elif defined EIGEN_CORE_H
	return Eigen::Map<VectorNd> (ptr, n, 1);
#else
	std::cerr << __func__ << " not defined for used math library!" << std::endl;
	abort();
	return VectorNd::Constant (1,1./0.);
#endif
}

RBDL_DLLAPI inline MatrixNdRef MatrixFromPtr (double *ptr, unsigned int rows, unsigned int cols, bool row_major = true) {
#ifdef RBDL_USE_SIMPLE_MATH
	return SimpleMath::Map<MatrixNd> (ptr, rows, cols);
#elif defined EIGEN_CORE_H
#ifdef PTR_DATA_ROW_MAJOR
	return Eigen::Map<MatrixNdRowMaj> (ptr, rows, cols);
#else
	return Eigen::Map<MatrixNd> (ptr, rows, cols);
#endif
#else
	std::cerr << __func__ << " not defined for used math library!" << std::endl;
	abort();
	return MatrixNd::Constant (1,1, 1./0.);
#endif
}

}

RBDL_DLLAPI
void UpdateKinematicsCustomPtr (Model &model,
		const double *q_ptr,
		const double *qdot_ptr,
		const double *qddot_ptr
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	using namespace RigidBodyDynamics::Math;

	unsigned int i;

	if (q_ptr) {
		VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);

		for (i = 1; i < model.mBodies.size(); i++) {
			unsigned int lambda = model.lambda[i];

			VectorNd QDot_zero (VectorNd::Zero (model.q_size));

			jcalc (model, i, (Q), QDot_zero);

			model.X_lambda[i] = model.X_J[i] * model.X_T[i];

			if (lambda != 0) {
				model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
			}	else {
				model.X_base[i] = model.X_lambda[i];
			}
		}
	}

	if (qdot_ptr) {
		VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
		VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);

		for (i = 1; i < model.mBodies.size(); i++) {
			unsigned int lambda = model.lambda[i];

			jcalc (model, i, Q, QDot);

			if (lambda != 0) {
				model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
				model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
			}	else {
				model.v[i] = model.v_J[i];
				model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
			}
			// LOG << "v[" << i << "] = " << model.v[i].transpose() << std::endl;
		}
	}

	if (qddot_ptr) {
		VectorNdRef QDDot = VectorFromPtr(const_cast<double*>(qddot_ptr), model.q_size);

		for (i = 1; i < model.mBodies.size(); i++) {
			unsigned int q_index = model.mJoints[i].q_index;

			unsigned int lambda = model.lambda[i];

			if (lambda != 0) {
				model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i];
			}	else {
				model.a[i] = model.c[i];
			}

			if (model.mJoints[i].mDoFCount == 3) {
				Vector3d omegadot_temp ((QDDot)[q_index], (QDDot)[q_index + 1], (QDDot)[q_index + 2]);
				model.a[i] = model.a[i] + model.multdof3_S[i] * omegadot_temp;
			} else {
				model.a[i] = model.a[i] + model.S[i] * (QDDot)[q_index];
			}
		}
	}
}

RBDL_DLLAPI
void CalcPointJacobianPtr (
		Model &model,
		const double *q_ptr,
		unsigned int body_id,
		const Math::Vector3d &point_position,
		double * G_ptr,
		bool update_kinematics
	) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	using namespace RigidBodyDynamics::Math;

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustomPtr (model, q_ptr, NULL, NULL);
	}

	VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
	MatrixNdRef G = MatrixFromPtr(const_cast<double*>(G_ptr), 3, model.qdot_size);

	SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));

	assert (G.rows() == 3 && G.cols() == model.qdot_size );

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
	}

	unsigned int j = reference_body_id;

	// e[j] is set to 1 if joint j contributes to the jacobian that we are
	// computing. For all other joints the column will be zero.
	while (j != 0) {
		unsigned int q_index = model.mJoints[j].q_index;

		if (model.mJoints[j].mDoFCount == 3) {
			G.block(0, q_index, 3, 3) = ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block(3,0,3,3);
		} else {
			G.block(0,q_index, 3, 1) = point_trans.apply(model.X_base[j].inverse().apply(model.S[j])).block(3,0,3,1);
		}

		j = model.lambda[j];
	}
}

RBDL_DLLAPI
void CalcPointJacobian6DPtr (
		Model &model,
		const double *q_ptr,
		unsigned int body_id,
		const Math::Vector3d &point_position,
		double *G_ptr,
		bool update_kinematics
	) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	using namespace RigidBodyDynamics::Math;

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustomPtr (model, q_ptr, NULL, NULL);
	}

	VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
	MatrixNdRef G = MatrixFromPtr(const_cast<double*>(G_ptr), 6, model.qdot_size);

	SpatialTransform point_trans = SpatialTransform (Matrix3d::Identity(), CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false));

	assert (G.rows() == 6 && G.cols() == model.qdot_size );

	unsigned int reference_body_id = body_id;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
	}

	unsigned int j = reference_body_id;

	while (j != 0) {
		unsigned int q_index = model.mJoints[j].q_index;

		if (model.mJoints[j].mDoFCount == 3) {
			G.block(0, q_index, 6, 3) = ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block(0,0,6,3);
		} else {
			G.block(0,q_index, 6, 1) = point_trans.apply(model.X_base[j].inverse().apply(model.S[j])).block(0,0,6,1);
		}

		j = model.lambda[j];
	}
}

RBDL_DLLAPI
void CalcBodySpatialJacobianPtr (
		Model &model,
		const double *q_ptr,
		unsigned int body_id,
		double *G_ptr,
		bool update_kinematics
	) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	using namespace RigidBodyDynamics::Math;

	// update the Kinematics if necessary
	if (update_kinematics) {
		UpdateKinematicsCustomPtr (model, q_ptr, NULL, NULL);
	}

	MatrixNdRef G = MatrixFromPtr(const_cast<double*>(G_ptr), 6, model.q_size);

	assert (G.rows() == 6 && G.cols() == model.qdot_size );

	unsigned int reference_body_id = body_id;

	SpatialTransform base_to_body;

	if (model.IsFixedBodyId(body_id)) {
		unsigned int fbody_id = body_id - model.fixed_body_discriminator;
		reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
		base_to_body = model.mFixedBodies[fbody_id].mParentTransform * model.X_base[reference_body_id];
	} else {
		base_to_body = model.X_base[reference_body_id];
	}

	unsigned int j = reference_body_id;

	while (j != 0) {
		unsigned int q_index = model.mJoints[j].q_index;

		if (model.mJoints[j].mDoFCount == 3) {
			G.block(0,q_index,6,3) = (base_to_body * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j];
		} else {
			G.block(0,q_index,6,1) = base_to_body.apply(model.X_base[j].inverse().apply(model.S[j]));
		}

		j = model.lambda[j];
	}
}

RBDL_DLLAPI
void InverseDynamicsPtr (
		Model &model,
		const double *q_ptr,
		const double *qdot_ptr,
		const double *qddot_ptr,
		const double *tau_ptr,
		std::vector<Math::SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	using namespace RigidBodyDynamics::Math;

	VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
	VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);
	VectorNdRef QDDot = VectorFromPtr(const_cast<double*>(qddot_ptr), model.q_size);
	VectorNdRef Tau = VectorFromPtr(const_cast<double*>(tau_ptr), model.q_size);

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0].set (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, Q, QDot);

		if (lambda != 0) {
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
		} else {
			model.X_base[i] = model.X_lambda[i];
		}

		model.v[i] = model.X_lambda[i].apply(model.v[lambda]) + model.v_J[i];
		model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);

		if (model.mJoints[i].mDoFCount == 3) {
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i] + model.multdof3_S[i] * Vector3d (QDDot[q_index], QDDot[q_index + 1], QDDot[q_index + 2]);
		} else {
			model.a[i] = model.X_lambda[i].apply(model.a[lambda]) + model.c[i] + model.S[i] * QDDot[q_index];
		}

		if (!model.mBodies[i].mIsVirtual) {
			model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
		} else {
			model.f[i].setZero();
		}

		if (f_ext != NULL && (*f_ext)[i] != SpatialVector::Zero())
			model.f[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
	}

	for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
		if (model.mJoints[i].mDoFCount == 3) {
			Tau.block<3,1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f[i];
		} else {
			Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f[i]);
		}

		if (model.lambda[i] != 0) {
			model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}
}

RBDL_DLLAPI
void NonlinearEffectsPtr (
		Model &model,
		const double *q_ptr,
		const double *qdot_ptr,
		const double *tau_ptr
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	using namespace RigidBodyDynamics::Math;

	VectorNdRef Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
	VectorNdRef QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);
	VectorNdRef Tau = VectorFromPtr(const_cast<double*>(tau_ptr), model.q_size);

	SpatialVector spatial_gravity (0., 0., 0., -model.gravity[0], -model.gravity[1], -model.gravity[2]);

	// Reset the velocity of the root body
	model.v[0].setZero();
	model.a[0] = spatial_gravity;

	for (unsigned int i = 1; i < model.mJointUpdateOrder.size(); i++) {
		jcalc (model, model.mJointUpdateOrder[i], Q, QDot);
	}

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		if (model.lambda[i] == 0) {
			model.v[i] = model.v_J[i];
			model.a[i] = model.X_lambda[i].apply(spatial_gravity);
		}	else {
			model.v[i] = model.X_lambda[i].apply(model.v[model.lambda[i]]) + model.v_J[i];
			model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
			model.a[i] = model.X_lambda[i].apply(model.a[model.lambda[i]]) + model.c[i];
		}

		if (!model.mBodies[i].mIsVirtual) {
			model.f[i] = model.I[i] * model.a[i] + crossf(model.v[i],model.I[i] * model.v[i]);
		} else {
			model.f[i].setZero();
		}
	}

	for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
		if (model.mJoints[i].mDoFCount == 3) {
			Tau.block<3,1>(model.mJoints[i].q_index, 0) = model.multdof3_S[i].transpose() * model.f[i];
		} else {
			Tau[model.mJoints[i].q_index] = model.S[i].dot(model.f[i]);
		}

		if (model.lambda[i] != 0) {
			model.f[model.lambda[i]] = model.f[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.f[i]);
		}
	}
}

RBDL_DLLAPI
inline void CompositeRigidBodyAlgorithmPtr (
		Model& model,
		const double *q_ptr,
		double *H_ptr,
		bool update_kinematics = true
		) {
	using namespace RigidBodyDynamics::Math;

	VectorNdRef&& Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
	MatrixNdRef&& H = MatrixFromPtr(H_ptr, model.qdot_size, model.qdot_size);

	assert (H.rows() == model.dof_count && H.cols() == model.dof_count);

	for (unsigned int i = 1; i < model.mBodies.size(); i++) {
		if (update_kinematics) {
			jcalc_X_lambda_S (model, i, Q);
		}
		model.Ic[i] = model.I[i];
	}

	for (unsigned int i = model.mBodies.size() - 1; i > 0; i--) {
		if (model.lambda[i] != 0) {
			model.Ic[model.lambda[i]] = model.Ic[model.lambda[i]] + model.X_lambda[i].applyTranspose(model.Ic[i]);
		}

		unsigned int dof_index_i = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 3) {
			Matrix63 F_63 = model.Ic[i].toMatrix() * model.multdof3_S[i];
			H.block<3,3>(dof_index_i, dof_index_i) = model.multdof3_S[i].transpose() * F_63;

			unsigned int j = i;
			unsigned int dof_index_j = dof_index_i;

			while (model.lambda[j] != 0) {
				F_63 = model.X_lambda[j].toMatrixTranspose() * (F_63);
				j = model.lambda[j];
				dof_index_j = model.mJoints[j].q_index;

				if (model.mJoints[j].mDoFCount == 3) {
					Matrix3d H_temp2 = F_63.transpose() * (model.multdof3_S[j]);

					H.block<3,3>(dof_index_i,dof_index_j) = H_temp2;
					H.block<3,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
				} else {
					Vector3d H_temp2 = F_63.transpose() * (model.S[j]);

					H.block<3,1>(dof_index_i,dof_index_j) = H_temp2;
					H.block<1,3>(dof_index_j,dof_index_i) = H_temp2.transpose();
				}
			}
		} else {
			SpatialVector F = model.Ic[i] * model.S[i];
			H(dof_index_i, dof_index_i) = model.S[i].dot(F);

			unsigned int j = i;
			unsigned int dof_index_j = dof_index_i;

			while (model.lambda[j] != 0) {
				F = model.X_lambda[j].applyTranspose(F);
				j = model.lambda[j];
				dof_index_j = model.mJoints[j].q_index;

				if (model.mJoints[j].mDoFCount == 3) {
					Vector3d H_temp2 = (F.transpose() * model.multdof3_S[j]).transpose();

					LOG << F.transpose() << std::endl << model.multdof3_S[j] << std::endl;
					LOG << H_temp2.transpose() << std::endl;

					H.block<1,3>(dof_index_i,dof_index_j) = H_temp2.transpose();
 					H.block<3,1>(dof_index_j,dof_index_i) = H_temp2;
				} else {
					H(dof_index_i,dof_index_j) = F.dot(model.S[j]);
					H(dof_index_j,dof_index_i) = H(dof_index_i,dof_index_j);
				}
			}
		}
	}
}

RBDL_DLLAPI
void ForwardDynamicsPtr (
		Model &model,
		const double *q_ptr,
		const double *qdot_ptr,
		const double *tau_ptr,
		const double *qddot_ptr,
		std::vector<Math::SpatialVector> *f_ext
		) {
	LOG << "-------- " << __func__ << " --------" << std::endl;

	using namespace RigidBodyDynamics::Math;

	VectorNdRef&& Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
	VectorNdRef&& QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);
	VectorNdRef&& QDDot = VectorFromPtr(const_cast<double*>(qddot_ptr), model.q_size);
	VectorNdRef&& Tau = VectorFromPtr(const_cast<double*>(tau_ptr), model.q_size);

	SpatialVector spatial_gravity (0., 0., 0., model.gravity[0], model.gravity[1], model.gravity[2]);

	unsigned int i = 0;

	LOG << "Q          = " << Q.transpose() << std::endl;
	LOG << "QDot       = " << QDot.transpose() << std::endl;
	LOG << "Tau        = " << Tau.transpose() << std::endl;
	LOG << "---" << std::endl;

	// Reset the velocity of the root body
	model.v[0].setZero();

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int lambda = model.lambda[i];

		jcalc (model, i, Q, QDot);

		if (lambda != 0)
			model.X_base[i] = model.X_lambda[i] * model.X_base[lambda];
		else
			model.X_base[i] = model.X_lambda[i];

		model.v[i] = model.X_lambda[i].apply( model.v[lambda]) + model.v_J[i];

		/*
		LOG << "X_J (" << i << "):" << std::endl << X_J << std::endl;
		LOG << "v_J (" << i << "):" << std::endl << v_J << std::endl;
		LOG << "v_lambda" << i << ":" << std::endl << model.v.at(lambda) << std::endl;
		LOG << "X_base (" << i << "):" << std::endl << model.X_base[i] << std::endl;
		LOG << "X_lambda (" << i << "):" << std::endl << model.X_lambda[i] << std::endl;
		LOG << "SpatialVelocity (" << i << "): " << model.v[i] << std::endl;
		*/

		model.c[i] = model.c_J[i] + crossm(model.v[i],model.v_J[i]);
		model.I[i].setSpatialMatrix (model.IA[i]);

		model.pA[i] = crossf(model.v[i],model.I[i] * model.v[i]);

		if (f_ext != NULL && (*f_ext)[i] != SpatialVector::Zero()) {
			LOG << "External force (" << i << ") = " << model.X_base[i].toMatrixAdjoint() * (*f_ext)[i] << std::endl;
			model.pA[i] -= model.X_base[i].toMatrixAdjoint() * (*f_ext)[i];
		}
	}

// ClearLogOutput();

	LOG << "--- first loop ---" << std::endl;

	for (i = model.mBodies.size() - 1; i > 0; i--) {
		unsigned int q_index = model.mJoints[i].q_index;

		if (model.mJoints[i].mDoFCount == 3) {
			model.multdof3_U[i] = model.IA[i] * model.multdof3_S[i];
#ifdef EIGEN_CORE_H
			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse().eval();
#else
			model.multdof3_Dinv[i] = (model.multdof3_S[i].transpose() * model.multdof3_U[i]).inverse();
#endif
			Vector3d tau_temp (Tau[q_index], Tau[q_index + 1], Tau[q_index + 2]);

			model.multdof3_u[i] = tau_temp - model.multdof3_S[i].transpose() * model.pA[i];

//			LOG << "multdof3_u[" << i << "] = " << model.multdof3_u[i].transpose() << std::endl;
			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia = model.IA[i] - model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_U[i].transpose();
				SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.multdof3_U[i] * model.multdof3_Dinv[i] * model.multdof3_u[i];
#ifdef EIGEN_CORE_H
				model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
				model.IA[lambda] += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
				LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose() << std::endl;
			}
		} else {
			model.U[i] = model.IA[i] * model.S[i];
			model.d[i] = model.S[i].dot(model.U[i]);
			model.u[i] = Tau[q_index] - model.S[i].dot(model.pA[i]);
//			LOG << "u[" << i << "] = " << model.u[i] << std::endl;

			unsigned int lambda = model.lambda[i];
			if (lambda != 0) {
				SpatialMatrix Ia = model.IA[i] - model.U[i] * (model.U[i] / model.d[i]).transpose();
				SpatialVector pa = model.pA[i] + Ia * model.c[i] + model.U[i] * model.u[i] / model.d[i];
#ifdef EIGEN_CORE_H
				model.IA[lambda].noalias() += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda].noalias() += model.X_lambda[i].applyTranspose(pa);
#else
				model.IA[lambda] += model.X_lambda[i].toMatrixTranspose() * Ia * model.X_lambda[i].toMatrix();
				model.pA[lambda] += model.X_lambda[i].applyTranspose(pa);
#endif
				LOG << "pA[" << lambda << "] = " << model.pA[lambda].transpose() << std::endl;
			}
		}
	}

//	ClearLogOutput();

	model.a[0] = spatial_gravity * -1.;

	for (i = 1; i < model.mBodies.size(); i++) {
		unsigned int q_index = model.mJoints[i].q_index;
		unsigned int lambda = model.lambda[i];
		SpatialTransform X_lambda = model.X_lambda[i];

		model.a[i] = X_lambda.apply(model.a[lambda]) + model.c[i];
		LOG << "a'[" << i << "] = " << model.a[i].transpose() << std::endl;

		if (model.mJoints[i].mDoFCount == 3) {
			Vector3d qdd_temp = model.multdof3_Dinv[i] * (model.multdof3_u[i] - model.multdof3_U[i].transpose() * model.a[i]);
			QDDot[q_index] = qdd_temp[0];
			QDDot[q_index + 1] = qdd_temp[1];
			QDDot[q_index + 2] = qdd_temp[2];
			model.a[i] = model.a[i] + model.multdof3_S[i] * qdd_temp;
		} else {
			QDDot[q_index] = (1./model.d[i]) * (model.u[i] - model.U[i].dot(model.a[i]));
			model.a[i] = model.a[i] + model.S[i] * QDDot[q_index];
		}
	}

	LOG << "QDDot = " << QDDot.transpose() << std::endl;
}

RBDL_DLLAPI
void ForwardDynamicsConstraintsDirectPtr (
  Model &model,
  const double *q_ptr,
  const double *qdot_ptr,
  const double *tau_ptr,
  ConstraintSet &CS,
  double *qddot_ptr
) {
  LOG << "-------- " << __func__ << " --------" << std::endl;

  using namespace RigidBodyDynamics::Math;

  VectorNdRef&& Q = VectorFromPtr(const_cast<double*>(q_ptr), model.q_size);
  VectorNdRef&& QDot = VectorFromPtr(const_cast<double*>(qdot_ptr), model.q_size);
  VectorNdRef&& QDDot = VectorFromPtr(const_cast<double*>(qddot_ptr), model.q_size);
  VectorNdRef&& Tau = VectorFromPtr(const_cast<double*>(tau_ptr), model.q_size);

  // create copy of non-const accelerations
  VectorNd QDDot_dummy = QDDot;

  LOG << "Q          = " << Q.transpose() << std::endl;
  LOG << "QDot       = " << QDot.transpose() << std::endl;
  LOG << "Tau        = " << Tau.transpose() << std::endl;

  LOG << "QDDot      = " << QDDot.transpose() << std::endl;
  LOG << "QDDot_dummy      = " << QDDot_dummy.transpose() << std::endl;

  // calling non-pointer version
  ForwardDynamicsConstraintsDirect (
    model, Q, QDot, Tau, CS, QDDot_dummy
  );

  for (int i = 0; i < model.q_size; ++i) {
    QDDot[i] = QDDot_dummy[i];
  }
  LOG << "QDDot      = " << QDDot.transpose() << std::endl;
  LOG << "QDDot_dummy      = " << QDDot_dummy.transpose() << std::endl;
  LOG << "---" << std::endl;
}
}
