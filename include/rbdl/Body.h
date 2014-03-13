/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _BODY_H
#define _BODY_H

#include "rbdl/rbdl_math.h"
#include "rbdl/rbdl_mathutils.h"
#include <assert.h>
#include <iostream>
#include "rbdl/Logging.h"

namespace RigidBodyDynamics {

/** \brief Describes all properties of a single body 
 *
 * A Body contains information about mass, the location of its center of
 * mass, and the ineria tensor in the center of mass. This class is
 * designed to use the given information and transform it such that it can
 * directly be used by the spatial algebra.
 * 
 * The spatial inertia of the member variable Body::mSpatialInertia is
 * expressed at the origin of the coordinate frame.
 *
 */
struct RBDL_DLLAPI Body {
	Body() :
		mMass (1.),
		mCenterOfMass (0., 0., 0.),
		mInertia (Math::Matrix3d::Identity(3,3)),
		mSpatialInertia (Math::SpatialMatrix::Zero(6,6)),
		mIsVirtual (false)
	{ };
	Body(const Body &body) :
		mMass (body.mMass),
		mCenterOfMass (body.mCenterOfMass),
		mInertia (body.mInertia),
		mSpatialInertia (body.mSpatialInertia),
		mIsVirtual (body.mIsVirtual)
	{};
	Body& operator= (const Body &body) {
		if (this != &body) {
			mMass = body.mMass;
			mInertia = body.mInertia;
			mCenterOfMass = body.mCenterOfMass;
			mSpatialInertia = body.mSpatialInertia;
			mIsVirtual = body.mIsVirtual;
		}

		return *this;
	}

	/** \brief Constructs a body from mass, center of mass and radii of gyration 
	 * 
	 * This constructor eases the construction of a new body as all the
	 * required parameters can be specified as parameters to the
	 * constructor. These are then used to generate the spatial inertia
	 * matrix which is expressed at the origin.
	 * 
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param gyration_radii the radii of gyration at the center of mass of the body
	 */
	Body(const double &mass,
			const Math::Vector3d &com,
			const Math::Vector3d &gyration_radii) :
		mMass (mass),
		mCenterOfMass(com),
		mIsVirtual (false) {
			Math::Matrix3d com_cross (
					0., -com[2],  com[1],
					com[2],      0., -com[0],
					-com[1],  com[0],      0.
					);
			Math::Matrix3d parallel_axis;
			parallel_axis = mass * com_cross * com_cross.transpose();

			mInertia = Math::Matrix3d (
					gyration_radii[0], 0., 0.,
					0., gyration_radii[1], 0.,
					0., 0., gyration_radii[2]
					);

			Math::Matrix3d pa (parallel_axis);
			Math::Matrix3d mcc = mass * com_cross;
			Math::Matrix3d mccT = mcc.transpose();

			Math::Matrix3d inertia_O = mInertia + pa;

			mSpatialInertia.set (
					inertia_O(0,0), inertia_O(0,1), inertia_O(0,2), mcc(0, 0), mcc(0, 1), mcc(0, 2),
					inertia_O(1,0), inertia_O(1,1), inertia_O(1,2), mcc(1, 0), mcc(1, 1), mcc(1, 2),
					inertia_O(2,0), inertia_O(2,1), inertia_O(2,2), mcc(2, 0), mcc(2, 1), mcc(2, 2),
					mccT(0, 0), mccT(0, 1), mccT(0, 2), mass, 0., 0.,
					mccT(1, 0), mccT(1, 1), mccT(1, 2), 0., mass, 0.,
					mccT(2, 0), mccT(2, 1), mccT(2, 2), 0., 0., mass
					);
		}

	/** \brief Constructs a body from mass, center of mass, and a 3x3 inertia matrix 
	 * 
	 * This constructor eases the construction of a new body as all the
	 * required parameters can simply be specified as parameters to the
	 * constructor. These are then used to generate the spatial inertia
	 * matrix which is expressed at the origin.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param inertia_C the inertia at the center of mass
	 */
	Body(const double &mass,
			const Math::Vector3d &com,
			const Math::Matrix3d &inertia_C) :
		mMass (mass),
		mCenterOfMass(com),
		mInertia (inertia_C),
		mIsVirtual (false) {
			Math::Matrix3d com_cross (
					0., -com[2],  com[1],
					com[2],      0., -com[0],
					-com[1],  com[0],      0.
					);
			Math::Matrix3d parallel_axis = mass * com_cross * com_cross.transpose();

			LOG << "parrallel axis = " << std::endl << parallel_axis << std::endl;

			Math::Matrix3d pa (parallel_axis);
			Math::Matrix3d mcc = mass * com_cross;
			Math::Matrix3d mccT = mcc.transpose();

			mSpatialInertia.set (
					inertia_C(0,0) + pa(0, 0), inertia_C(0,1) + pa(0, 1), inertia_C(0,2) + pa(0, 2), mcc(0, 0), mcc(0, 1), mcc(0, 2),
					inertia_C(1,0) + pa(1, 0), inertia_C(1,1) + pa(1, 1), inertia_C(1,2) + pa(1, 2), mcc(1, 0), mcc(1, 1), mcc(1, 2),
					inertia_C(2,0) + pa(2, 0), inertia_C(2,1) + pa(2, 1), inertia_C(2,2) + pa(2, 2), mcc(2, 0), mcc(2, 1), mcc(2, 2),
					mccT(0, 0), mccT(0, 1), mccT(0, 2), mass, 0., 0.,
					mccT(1, 0), mccT(1, 1), mccT(1, 2), 0., mass, 0.,
					mccT(2, 0), mccT(2, 1), mccT(2, 2), 0., 0., mass
					);
		}
	
	/** \brief Joins inertial parameters of two bodies to create a composite
	 * body.
	 *
	 * This function can be used to joint inertial parameters of two bodies
	 * to create a composite body that has the inertial properties as if the
	 * two bodies were joined by a fixed joint.
	 *
	 * \note Both bodies have to have their inertial parameters expressed in
	 * the same orientation.
	 *
	 * \param transform The frame transformation from the origin of the
	 * original body to the origin of the added body
	 * \param other_body The other body that will be merged with *this.
	 */
	void Join (const Math::SpatialTransform &transform, const Body &other_body) {
		// nothing to do if we join a massles body to the current.
		if (other_body.mMass == 0. && other_body.mInertia == Math::Matrix3d::Zero()) {
			return;
		}

		double other_mass = other_body.mMass;
		double new_mass = mMass + other_mass;

		if (new_mass == 0.) {
			std::cerr << "Error: cannot join bodies as both have zero mass!" << std::endl;
			assert (false);
			abort();
		}

		Math::Vector3d other_com = transform.E.transpose() * other_body.mCenterOfMass + transform.r;
		Math::Vector3d new_com = (1 / new_mass ) * (mMass * mCenterOfMass + other_mass * other_com);

		LOG << "other_com = " << std::endl << other_com.transpose() << std::endl;
		LOG << "rotation = " << std::endl << transform.E << std::endl;

		// We have to transform the inertia of other_body to the new COM. This
		// is done in 4 steps:
		//
		// 1. Transform the inertia from other origin to other COM
		// 2. Rotate the inertia that it is aligned to the frame of this body
		// 3. Transform inertia of other_body to the origin of the frame of
		// this body
		// 4. Sum the two inertias
		// 5. Transform the summed inertia to the new COM

		Math::Matrix3d inertia_other = other_body.mSpatialInertia.block<3,3>(0,0);
		LOG << "inertia_other = " << std::endl << inertia_other << std::endl;

		// 1. Transform the inertia from other origin to other COM
		Math::Matrix3d other_com_cross = Math::VectorCrossMatrix(other_body.mCenterOfMass);
		Math::Matrix3d inertia_other_com = inertia_other - other_mass * other_com_cross * other_com_cross.transpose();
		LOG << "inertia_other_com = " << std::endl << inertia_other_com << std::endl;

		// 2. Rotate the inertia that it is aligned to the frame of this body
		Math::Matrix3d inertia_other_com_rotated = transform.E.transpose() * inertia_other_com * transform.E;
		LOG << "inertia_other_com_rotated = " << std::endl << inertia_other_com_rotated << std::endl;

		// 3. Transform inertia of other_body to the origin of the frame of this body
		Math::Matrix3d inertia_other_com_rotated_this_origin = Math::parallel_axis (inertia_other_com_rotated, other_mass, other_com);
		LOG << "inertia_other_com_rotated_this_origin = " << std::endl << inertia_other_com_rotated_this_origin << std::endl;

		// 4. Sum the two inertias
		Math::Matrix3d inertia_summed = Math::Matrix3d (mSpatialInertia.block<3,3>(0,0)) + inertia_other_com_rotated_this_origin;
		LOG << "inertia_summed  = " << std::endl << inertia_summed << std::endl;

		// 5. Transform the summed inertia to the new COM
		Math::Matrix3d new_inertia = inertia_summed - new_mass * Math::VectorCrossMatrix (new_com) * Math::VectorCrossMatrix(new_com).transpose();

		LOG << "new_mass = " << new_mass << std::endl;
		LOG << "new_com  = " << new_com.transpose() << std::endl;
		LOG << "new_inertia  = " << std::endl << new_inertia << std::endl;

		*this = Body (new_mass, new_com, new_inertia);
	}

	~Body() {};

	/// \brief The mass of the body
	double mMass;
	/// \brief The position of the center of mass in body coordinates
	Math::Vector3d mCenterOfMass;
	/// \brief Inertia matrix at the center of mass
	Math::Matrix3d mInertia;
	/// \brief The spatial inertia that contains both mass and inertia information
	Math::SpatialMatrix mSpatialInertia;

	bool mIsVirtual;
};

/** \brief Keeps the information of a body and how it is attached to another body.
 *
 * When using fixed bodies, i.e. a body that is attached to anothe via a
 * fixed joint, the attached body is merged onto its parent. By doing so
 * adding fixed joints do not have an impact on runtime.
 */
struct RBDL_DLLAPI FixedBody {
	/// \brief The mass of the body
	double mMass;
	/// \brief The position of the center of mass in body coordinates
	Math::Vector3d mCenterOfMass;
	/// \brief The spatial inertia that contains both mass and inertia information
	Math::SpatialMatrix mSpatialInertia;

	/// \brief Id of the movable body that this fixed body is attached to.
	unsigned int mMovableParent;
	/// \brief Transforms spatial quantities expressed for the parent to the
	// fixed body. 
	Math::SpatialTransform mParentTransform;
	Math::SpatialTransform mBaseTransform;

	static FixedBody CreateFromBody (const Body& body) {
		FixedBody fbody;

		fbody.mMass = body.mMass;
		fbody.mCenterOfMass = body.mCenterOfMass;
		fbody.mSpatialInertia = body.mSpatialInertia;

		return fbody;
	}
};

}

#endif /* _BODY_H */
