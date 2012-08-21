/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
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

struct Model;

/** \brief General types of joints
 */
enum JointType {
	JointTypeUndefined = 0,
	JointTypeRevolute,
	JointTypePrismatic,
	JointTypeFixed,

	JointType1DoF,
	JointType2DoF,
	JointType3DoF,
	JointType4DoF,
	JointType5DoF,
	JointType6DoF
};

/** \brief Describes a joint relative to the predecessor body.
 *
 * This class contains all information required for one single joint. This
 * contains the joint type and the axis of the joint.
 */
struct Joint {
	Joint() :
		mJointAxes (NULL),
		mJointType (JointTypeUndefined),
		mDoFCount (0) {};
	Joint (JointType type) :
		mJointAxes (NULL),
		mJointType (type),
	  mDoFCount (0) {
			if (type != JointTypeFixed) {
				std::cerr << "Error: Invalid use of Joint constructor Joint(JointType type). Only allowed when type == JointTypeFixed." << std::endl;
				assert (0);
				abort();
			}
		}
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
		if (mJointAxes) {
			assert (mJointAxes);
			delete[] mJointAxes;
			mJointAxes = NULL;
			mDoFCount = 0;
		}
	}

	/** \brief Constructs a joint from the given cartesian parameters.
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
		assert ( joint_type == JointTypeRevolute || joint_type == JointTypePrismatic );

		mJointType = joint_type;

		if (joint_type == JointTypeRevolute) {
			// make sure we have a unit axis
			mJointAxes[0].set (
					joint_axis[0],
					joint_axis[1], 
					joint_axis[2], 
					0., 0., 0.
					);

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

	/** \brief Constructs a 1 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 */
	Joint (
			const Math::SpatialVector &axis_0
			) {
		mJointType = JointType1DoF;
		mDoFCount = 1;

		mJointAxes = new Math::SpatialVector[mDoFCount];
		mJointAxes[0] = axis_0;

		validate_spatial_axis (mJointAxes[0]);
	}

	/** \brief Constructs a 2 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 */
	Joint (
			const Math::SpatialVector &axis_0,
			const Math::SpatialVector &axis_1
			) {
		mJointType = JointType2DoF;
		mDoFCount = 2;

		mJointAxes = new Math::SpatialVector[mDoFCount];
		mJointAxes[0] = axis_0;
		mJointAxes[1] = axis_1;

		validate_spatial_axis (mJointAxes[0]);
		validate_spatial_axis (mJointAxes[1]);
	}

	/** \brief Constructs a 3 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 * \param axis_2 Motion subspace for axis 2
	 */
	Joint (
			const Math::SpatialVector &axis_0,
			const Math::SpatialVector &axis_1,
			const Math::SpatialVector &axis_2
			) {
		mJointType = JointType3DoF;
		mDoFCount = 3;

		mJointAxes = new Math::SpatialVector[mDoFCount];

		mJointAxes[0] = axis_0;
		mJointAxes[1] = axis_1;
		mJointAxes[2] = axis_2;

		validate_spatial_axis (mJointAxes[0]);
		validate_spatial_axis (mJointAxes[1]);
		validate_spatial_axis (mJointAxes[2]);
	}

	/** \brief Constructs a 4 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 * \param axis_2 Motion subspace for axis 2
	 * \param axis_3 Motion subspace for axis 3
	 */
	Joint (
			const Math::SpatialVector &axis_0,
			const Math::SpatialVector &axis_1,
			const Math::SpatialVector &axis_2,
			const Math::SpatialVector &axis_3
			) {
		mJointType = JointType4DoF;
		mDoFCount = 4;

		mJointAxes = new Math::SpatialVector[mDoFCount];

		mJointAxes[0] = axis_0;
		mJointAxes[1] = axis_1;
		mJointAxes[2] = axis_2;
		mJointAxes[3] = axis_3;

		validate_spatial_axis (mJointAxes[0]);
		validate_spatial_axis (mJointAxes[1]);
		validate_spatial_axis (mJointAxes[2]);
		validate_spatial_axis (mJointAxes[3]);
	}

	/** \brief Constructs a 5 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 * \param axis_2 Motion subspace for axis 2
	 * \param axis_3 Motion subspace for axis 3
	 * \param axis_4 Motion subspace for axis 4
	 */
	Joint (
			const Math::SpatialVector &axis_0,
			const Math::SpatialVector &axis_1,
			const Math::SpatialVector &axis_2,
			const Math::SpatialVector &axis_3,
			const Math::SpatialVector &axis_4
			) {
		mJointType = JointType5DoF;
		mDoFCount = 5;

		mJointAxes = new Math::SpatialVector[mDoFCount];

		mJointAxes[0] = axis_0;
		mJointAxes[1] = axis_1;
		mJointAxes[2] = axis_2;
		mJointAxes[3] = axis_3;
		mJointAxes[4] = axis_4;

		validate_spatial_axis (mJointAxes[0]);
		validate_spatial_axis (mJointAxes[1]);
		validate_spatial_axis (mJointAxes[2]);
		validate_spatial_axis (mJointAxes[3]);
		validate_spatial_axis (mJointAxes[4]);
	}

	/** \brief Constructs a 6 DoF joint with the given motion subspaces.
	 *
	 * The motion subspaces are of the format:
	 * \f[ (r_x, r_y, r_z, t_x, t_y, t_z) \f]
	 *
	 * \note So far only pure rotations or pure translations are supported.
	 *
	 * \param axis_0 Motion subspace for axis 0
	 * \param axis_1 Motion subspace for axis 1
	 * \param axis_2 Motion subspace for axis 2
	 * \param axis_3 Motion subspace for axis 3
	 * \param axis_4 Motion subspace for axis 4
	 * \param axis_5 Motion subspace for axis 5
	 */
	Joint (
			const Math::SpatialVector &axis_0,
			const Math::SpatialVector &axis_1,
			const Math::SpatialVector &axis_2,
			const Math::SpatialVector &axis_3,
			const Math::SpatialVector &axis_4,
			const Math::SpatialVector &axis_5
			) {
		mJointType = JointType6DoF;
		mDoFCount = 6;

		mJointAxes = new Math::SpatialVector[mDoFCount];

		mJointAxes[0] = axis_0;
		mJointAxes[1] = axis_1;
		mJointAxes[2] = axis_2;
		mJointAxes[3] = axis_3;
		mJointAxes[4] = axis_4;
		mJointAxes[5] = axis_5;

		validate_spatial_axis (mJointAxes[0]);
		validate_spatial_axis (mJointAxes[1]);
		validate_spatial_axis (mJointAxes[2]);
		validate_spatial_axis (mJointAxes[3]);
		validate_spatial_axis (mJointAxes[4]);
		validate_spatial_axis (mJointAxes[5]);
	}

	/** \brief Checks whether we have pure rotational or translational axis.
	 *
	 * This function is mainly used to print out warnings when specifying an
	 * axis that might not be intended.
	 */
	bool validate_spatial_axis (Math::SpatialVector &axis) {
		if (fabs(axis.norm() - 1.0) > 1.0e-8) {
			std::cerr << "Warning: joint axis is not unit!" << std::endl;
		}

		bool axis_rotational = false;
		bool axis_translational = false;

		Math::Vector3d rotation (axis[0], axis[1], axis[2]);
		Math::Vector3d translation (axis[3], axis[4], axis[5]);

		if (fabs(translation.norm()) < 1.0e-8)
			axis_rotational = true;

		if (fabs(rotation.norm()) < 1.0e-8)
			axis_translational = true;

		return axis_rotational || axis_translational;
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
