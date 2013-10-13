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
		if (model.mJoints[joint_id].mJointType == JointTypeRevolute) {
			XJ = Xrot (q[model.mJoints[joint_id].q_index], Vector3d (
						model.mJoints[joint_id].mJointAxes[0][0],
						model.mJoints[joint_id].mJointAxes[0][1],
						model.mJoints[joint_id].mJointAxes[0][2]
						));

		} else if (model.mJoints[joint_id].mJointType == JointTypePrismatic) {
			XJ = Xtrans ( Vector3d (
						model.mJoints[joint_id].mJointAxes[0][3] * q[model.mJoints[joint_id].q_index],
						model.mJoints[joint_id].mJointAxes[0][4] * q[model.mJoints[joint_id].q_index],
						model.mJoints[joint_id].mJointAxes[0][5] * q[model.mJoints[joint_id].q_index]
						)
					);
		}

		// Set the joint axis
		model.S[joint_id] = model.mJoints[joint_id].mJointAxes[0];

		// the velocity dependent spatial acceleration is != 0 only for rhenomic
		// constraints (see RBDA, p. 55)
		c_J.setZero();

		v_J = model.S[joint_id] * qdot[model.mJoints[joint_id].q_index];
	} else {
		// Only revolute joints supported so far
		assert (0);
	}
}

}
