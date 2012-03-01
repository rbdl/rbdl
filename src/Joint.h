/*
 * RBDL - Rigid Body Library
 * Copyright (c) 2011 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _JOINT_H
#define _JOINT_H

#include <rbdl_math.h>
#include <assert.h>
#include <iostream>
#include "Logging.h"

namespace RigidBodyDynamics {

class Model;

/** \brief General types of joints
 *
 * \todo add proper fixed joint handling
 */
enum JointType {
	JointTypeUndefined = 0,
	JointTypeFixed,
	JointTypeRevolute,
	JointTypePrismatic,

	JointType2DoF,
};

/** \brief Describes a joint relative to the predecessor body 
 *
 * This class contains all information required for one single joint. This
 * contains the joint type and the axis of the joint.
 */
struct Joint {
	Joint() :
		mJointAxes (NULL),
		mJointType (JointTypeUndefined),
		mDoFCount (0) {};
	Joint (const Joint &joint) :
		mJointType (joint.mJointType),
		mDoFCount (joint.mDoFCount) {
			mJointAxes = new Math::SpatialVector[mDoFCount];

			for (unsigned int i = 0; i < mDoFCount; i++)
				mJointAxes[i] = joint.mJointAxes[i];
		};
	Joint& operator= (const Joint &joint) {
		if (this != &joint) {
			if (mDoFCount > 0) {
				assert (mJointAxes);
				delete[] mJointAxes;
			}
			mJointType = joint.mJointType;
			mDoFCount = joint.mDoFCount;

			mJointAxes = new Math::SpatialVector[mDoFCount];

			for (unsigned int i = 0; i < mDoFCount; i++)
				mJointAxes[i] = joint.mJointAxes[i];
		}
		return *this;
	}
	~Joint() {
		if (mDoFCount) {
			assert (mJointAxes);
			delete[] mJointAxes;
			mJointAxes = NULL;
		}
	}

	/** \brief Constructs a joint from the given cartesian parameters
	 *
	 * This constructor creates all the required spatial values for the given
	 * cartesian parameters.
	 *
	 * \param joint_type whether the joint is revolute or prismatic
	 * \param joint_axis the axis of rotation or translation
	 */
	Joint (
			const JointType joint_type,
			const Math::Vector3d &joint_axis
			) {
		mDoFCount = 1;
		mJointAxes = new Math::SpatialVector[mDoFCount];

		// Some assertions, as we concentrate on simple cases
	
		// Only rotation around the Z-axis
		assert ( joint_type == JointTypeRevolute || joint_type == JointTypeFixed || joint_type == JointTypePrismatic );

		mJointType = joint_type;

		if (joint_type == JointTypeRevolute) {
			// make sure we have a unit axis
			assert (joint_axis.squaredNorm() == 1.);

			assert ( joint_axis == Math::Vector3d (1., 0., 0.)
					|| joint_axis == Math::Vector3d (0., 1., 0.)
					|| joint_axis == Math::Vector3d (0., 0., 1.));

			mJointAxes[0].set (
					joint_axis[0],
					joint_axis[1], 
					joint_axis[2], 
					0., 0., 0.
					);

		} else if (joint_type == JointTypeFixed) {
			mJointAxes[0].set (
					joint_axis[0],
					joint_axis[1], 
					joint_axis[2], 
					0., 0., 0.
					);
			mJointAxes[0].set (0., 0., 0., 0., 0., 0.);
		} else if (joint_type == JointTypePrismatic) {
			// make sure we have a unit axis
			assert (joint_axis.squaredNorm() == 1.);

			mJointAxes[0].set (
					0., 0., 0.,
					joint_axis[0],
					joint_axis[1],
					joint_axis[2]
					);
		}
	}

	/// \brief The spatial axis of the joint
	Math::SpatialVector* mJointAxes;
	/// \brief Type of joint (rotational or prismatic)
	JointType mJointType;
	unsigned int mDoFCount;
};

/** \brief Computes all variables for a joint model
 *
 *	By appropriate modification of this function all types of joints can be
 *	modeled. See RBDA Section 4.4 for details.
 *
 * \param model    the rigid body model
 * \param joint_id the id of the joint we are interested in (output)
 * \param XJ       the joint transformation (output)
 * \param S        motion subspace of the joint (output)
 * \param v_J      joint velocity (output)
 * \param c_J      joint acceleration for rhenomic joints (output)
 * \param q        joint state variable
 * \param qdot     joint velocity variable
 */
void jcalc (
		const Model &model,
		const unsigned int &joint_id,
		Math::SpatialTransform &XJ,
		Math::SpatialVector &S,
		Math::SpatialVector &v_J,
		Math::SpatialVector &c_J,
		const double &q,
		const double &qdot
		);

}

#endif /* _JOINT_H */
