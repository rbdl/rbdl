#include <iostream>
#include <limits>
#include <assert.h>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Joint.h"

using namespace SpatialAlgebra;
namespace RigidBodyDynamics {

void jcalc (
		const Model &model,
		const unsigned int &joint_id,
		SpatialMatrix &XJ,
		SpatialVector &S,
		SpatialVector &v_J,
		SpatialVector &c_J,
		const double &q,
		const double &qdot
		) {
	// exception if we calculate it for the root body
	assert (joint_id > 0);

	Joint joint = model.mJoints[joint_id];

	// Calculate the spatial joint velocity
	v_J = model.S.at(joint_id);

	// Set the joint axis
	S = joint.mJointAxis;

	// the velocity dependent spatial acceleration is != 0 only for rhenomic
	// constraints (see RBDA, p. 55)
	c_J.zero();

	if (joint.mJointType == JointTypeFixed) {
		XJ = SpatialMatrixIdentity;
		v_J.zero();

		return;
	} else if (joint.mJointType == JointTypeRevolute) {
		// Only rotations around coordinate axes are supported so far!
		if (S == SpatialVector(1., 0., 0., 0., 0., 0.)) {
			XJ = Xrotx (q);
		} else if (S == SpatialVector(0., 1., 0., 0., 0., 0.)) {
			XJ = Xroty (q);
		} else if (S == SpatialVector(0., 0., 1., 0., 0., 0.)) {
			XJ = Xrotz (q);
		} else {
			assert (0 && !"Invalid joint axis!");
		}
	} else if (joint.mJointType == JointTypePrismatic) {
		XJ = Xtrans ( Vector3d (
					joint.mJointAxis[3] * q,
					joint.mJointAxis[4] * q,
					joint.mJointAxis[5] * q
					)
				);
	} else {
		// Only revolute joints supported so far
		assert (0);
	}

	v_J = S * qdot;
}

}
