/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2015 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
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

RBDL_DLLAPI
void InverseDynamicsPtr (
		Model &model,
		const double* q_ptr,
		const double* qdot_ptr,
		const double* qddot_ptr,
		double* tau_ptr,
		std::vector<Math::SpatialVector> *f_ext = NULL
		) {
	Math::VectorNd &&Q = Math::VectorFromPtr(model.q_size, const_cast<double*>(q_ptr));
	Math::VectorNd &&QDot = Math::VectorFromPtr(model.qdot_size, const_cast<double*>(qdot_ptr));
	Math::VectorNd &&QDDot = Math::VectorFromPtr(model.qdot_size, const_cast<double*>(qddot_ptr));
	Math::VectorNd &&Tau = Math::VectorFromPtr(model.qdot_size, const_cast<double*>(tau_ptr));

	InverseDynamics (model, Q, QDot, QDDot, Tau, f_ext);
}

RBDL_DLLAPI
inline void CompositeRigidBodyAlgorithmPtr (
		Model& model,
		const double *q_ptr,
		double *H_ptr,
		bool update_kinematics = true
		) {
	Math::VectorNd &&Q = Math::VectorFromPtr(model.q_size, const_cast<double*>(q_ptr));
	Math::MatrixNd &&H = Math::MatrixFromPtr(model.qdot_size, model.qdot_size, H_ptr);

	CompositeRigidBodyAlgorithm (model, Q, H, update_kinematics);	
}

RBDL_DLLAPI
void ForwardDynamicsPtr (
		Model &model,
		const double* q_ptr,
		const double* qdot_ptr,
		const double* tau_ptr,
		double* qddot_ptr,
		std::vector<Math::SpatialVector> *f_ext = NULL
		) {
	Math::VectorNd &&Q = Math::VectorFromPtr(model.q_size, const_cast<double*>(q_ptr));
	Math::VectorNd &&QDot = Math::VectorFromPtr(model.qdot_size, const_cast<double*>(qdot_ptr));
	Math::VectorNd &&Tau = Math::VectorFromPtr(model.qdot_size, const_cast<double*>(tau_ptr));
	Math::VectorNd &&QDDot = Math::VectorFromPtr(model.qdot_size, const_cast<double*>(qddot_ptr));

	ForwardDynamics (model, Q, QDot, Tau, QDDot, f_ext);
}

}
