/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>

#include "rbdl/Logging.h"

#include "rbdl/Model.h"
#include "rbdl/Joint.h"

namespace RigidBodyDynamics {

	using namespace Math;

	RBDL_DLLAPI
		Math::SpatialTransform jcalc_XJ (
				Model &model,
				unsigned int joint_id,
				const Math::VectorNd &q) {
			// exception if we calculate it for the root body
			assert (joint_id > 0);

			if (model.mJoints[joint_id].mDoFCount == 1) {
				if (model.mJoints[joint_id].mJointType == JointTypeRevolute) {
					return Xrot (q[model.mJoints[joint_id].q_index], Vector3d (
								model.mJoints[joint_id].mJointAxes[0][0],
								model.mJoints[joint_id].mJointAxes[0][1],
								model.mJoints[joint_id].mJointAxes[0][2]
								));
				} else if (model.mJoints[joint_id].mJointType == JointTypePrismatic) {
					return Xtrans ( Vector3d (
								model.mJoints[joint_id].mJointAxes[0][3] * q[model.mJoints[joint_id].q_index],
								model.mJoints[joint_id].mJointAxes[0][4] * q[model.mJoints[joint_id].q_index],
								model.mJoints[joint_id].mJointAxes[0][5] * q[model.mJoints[joint_id].q_index]
								)
							);
				}
			} else if (model.mJoints[joint_id].mJointType == JointTypeSpherical) {
				return SpatialTransform ( model.GetQuaternion (joint_id, q).toMatrix(), Vector3d (0., 0., 0.));
			} else if (model.mJoints[joint_id].mJointType == JointTypeEulerZYX) {
				double q0 = q[model.mJoints[joint_id].q_index];
				double q1 = q[model.mJoints[joint_id].q_index + 1];
				double q2 = q[model.mJoints[joint_id].q_index + 2];

				double s0 = sin (q0);
				double c0 = cos (q0);
				double s1 = sin (q1);
				double c1 = cos (q1);
				double s2 = sin (q2);
				double c2 = cos (q2);

				return SpatialTransform ( Matrix3d (
							c0 * c1, s0 * c1, -s1,
							c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
							c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
							), Vector3d (0., 0., 0.));
			}

			std::cerr << "Error: invalid joint type!" << std::endl;
			abort();
			return SpatialTransform();
		}

	RBDL_DLLAPI
		void jcalc (
				Model &model,
				unsigned int joint_id,
				SpatialTransform &XJ,
				SpatialVector &v_J,
				SpatialVector &c_J,
				const VectorNd &q,
				const VectorNd &qdot
				) {
			// exception if we calculate it for the root body
			assert (joint_id > 0);

			if (model.mJoints[joint_id].mDoFCount == 1) {
				XJ = jcalc_XJ (model, joint_id, q);

				// Set the joint axis
				model.S[joint_id] = model.mJoints[joint_id].mJointAxes[0];

				// the velocity dependent spatial acceleration is != 0 only for rhenomic
				// constraints (see RBDA, p. 55)
				c_J.setZero();

				v_J = model.S[joint_id] * qdot[model.mJoints[joint_id].q_index];
			} else if (model.mJoints[joint_id].mJointType == JointTypeSpherical) {
				XJ = jcalc_XJ (model, joint_id, q);

				model.multdof3_S[joint_id].setZero();

				model.multdof3_S[joint_id](0,0) = 1.;
				model.multdof3_S[joint_id](1,1) = 1.;
				model.multdof3_S[joint_id](2,2) = 1.;

				Vector3d omega (qdot[model.mJoints[joint_id].q_index],
						qdot[model.mJoints[joint_id].q_index+1],
						qdot[model.mJoints[joint_id].q_index+2]);

				v_J = SpatialVector (
						omega[0], omega[1], omega[2],
						0., 0., 0.);

				c_J.setZero();
			} else if (model.mJoints[joint_id].mJointType == JointTypeEulerZYX) {
				double q0 = q[model.mJoints[joint_id].q_index];
				double q1 = q[model.mJoints[joint_id].q_index + 1];
				double q2 = q[model.mJoints[joint_id].q_index + 2];

				double s0 = sin (q0);
				double c0 = cos (q0);
				double s1 = sin (q1);
				double c1 = cos (q1);
				double s2 = sin (q2);
				double c2 = cos (q2);

				XJ.E = Matrix3d(
							c0 * c1, s0 * c1, -s1,
							c0 * s1 * s2 - s0 * c2, s0 * s1 * s2 + c0 * c2, c1 * s2,
							c0 * s1 * c2 + s0 * s2, s0 * s1 * c2 - c0 * s2, c1 * c2
							);

				model.multdof3_S[joint_id].setZero();

				model.multdof3_S[joint_id](0,0) = -s1;
				model.multdof3_S[joint_id](0,2) = 1.;

				model.multdof3_S[joint_id](1,0) = c1 * s2;
				model.multdof3_S[joint_id](1,1) = c2;

				model.multdof3_S[joint_id](2,0) = c1 * c2;
				model.multdof3_S[joint_id](2,1) = - s2;

				double qdot0 = qdot[model.mJoints[joint_id].q_index];
				double qdot1 = qdot[model.mJoints[joint_id].q_index + 1];
				double qdot2 = qdot[model.mJoints[joint_id].q_index + 2];

				v_J = model.multdof3_S[joint_id] * Vector3d (qdot0, qdot1, qdot2);

				c_J.set(
						- c1 * qdot0 * qdot1,
						-s1 * s2 * qdot0 * qdot1 + c1 * c2 * qdot0 * qdot2 - s2 * qdot1 * qdot2,
						-s1 * c2 * qdot0 * qdot1 - c1 * s2 * qdot0 * qdot2 - c2 * qdot1 * qdot2,
						0., 0., 0.
						);
			} else {
		// Only revolute joints supported so far
		assert (0);
	}
}

}
