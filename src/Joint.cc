/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <assert.h>

#include "Logging.h"

#include "Model.h"
#include "Joint.h"

namespace RigidBodyDynamics {

using namespace Math;

void jcalc (
		const Model &model,
		const unsigned int &joint_id,
		SpatialTransform &XJ,
		SpatialVector &S,
		SpatialVector &v_J,
		SpatialVector &c_J,
		const double &q,
		const double &qdot
		) {
	// exception if we calculate it for the root body
	assert (joint_id > 0);

	// Calculate the spatial joint velocity
	v_J = model.S.at(joint_id);

	// Set the joint axis
	S = model.mJoints[joint_id].mJointAxes[0];

	// the velocity dependent spatial acceleration is != 0 only for rhenomic
	// constraints (see RBDA, p. 55)
	c_J.setZero();

	if (model.mJoints[joint_id].mJointType == JointTypeRevolute) {
		XJ = Xrot (q, Vector3d (
					model.mJoints[joint_id].mJointAxes[0][0],
					model.mJoints[joint_id].mJointAxes[0][1],
					model.mJoints[joint_id].mJointAxes[0][2]
					));
	} else if (model.mJoints[joint_id].mJointType == JointTypePrismatic) {
		XJ = Xtrans ( Vector3d (
					model.mJoints[joint_id].mJointAxes[0][3] * q,
					model.mJoints[joint_id].mJointAxes[0][4] * q,
					model.mJoints[joint_id].mJointAxes[0][5] * q
					)
				);
	} else {
		// Only revolute joints supported so far
		assert (0);
	}

	v_J = S * qdot;
}

}
