/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _BODY_H
#define _BODY_H

#include <rbdl_math.h>
#include <assert.h>
#include <iostream>
#include "Logging.h"

namespace RigidBodyDynamics {

/** \brief Describes all properties of a single body 
 *
 * A Body contains information about mass, the location of its center of
 * mass, and the ineria tensor in the center of mass. This class is
 * designed to use the given information and transform it such that it can
 * directly be used by the spatial algebra.
 */
struct Body {
	Body() :
		mMass (1.),
		mCenterOfMass (0., 0., 0.),
		mSpatialInertia (
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.
				)
	{ };
	Body(const Body &body) :
		mMass (body.mMass),
		mCenterOfMass (body.mCenterOfMass),
		mSpatialInertia (body.mSpatialInertia)
	{};
	Body& operator= (const Body &body) {
		if (this != &body) {
			mSpatialInertia = body.mSpatialInertia;
			mCenterOfMass = body.mCenterOfMass;
			mMass = body.mMass;
		}

		return *this;
	}

	/** \brief Constructs a body from mass, center of mass and radii of gyration 
	 *
	 * This constructor eases the construction of a new body as all the required
	 * parameters can simply be specified as parameters to the constructor.
	 * These are then used to generate the spatial inertia matrix.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param gyration_radii the radii of gyration at the center of mass of the body
	 */
	Body(const double &mass,
			const Math::Vector3d &com,
			const Math::Vector3d &gyration_radii) :
		mMass (mass),
		mCenterOfMass(com) {
			Math::Matrix3d com_cross (
					0., -com[2],  com[1],
					com[2],      0., -com[0],
					-com[1],  com[0],      0.
					);
			Math::Matrix3d parallel_axis;
			parallel_axis = mass * com_cross * com_cross.transpose();

			Math::Vector3d gr (gyration_radii);
			Math::Matrix3d pa (parallel_axis);
			Math::Matrix3d mcc = mass * com_cross;
			Math::Matrix3d mccT = mcc.transpose();

			mSpatialInertia.set (
					gr[0] + pa(0, 0), pa(0, 1), pa(0, 2), mcc(0, 0), mcc(0, 1), mcc(0, 2),
					pa(1, 0), gr[1] + pa(1, 1), pa(1, 2), mcc(1, 0), mcc(1, 1), mcc(1, 2),
					pa(2, 0), pa(2, 1), gr[2] + pa(2, 2), mcc(2, 0), mcc(2, 1), mcc(2, 2),
					mccT(0, 0), mccT(0, 1), mccT(0, 2), mass, 0., 0.,
					mccT(1, 0), mccT(1, 1), mccT(1, 2), 0., mass, 0.,
					mccT(2, 0), mccT(2, 1), mccT(2, 2), 0., 0., mass
					);
		}

	/** \brief Constructs a body from mass, center of mass, and a 3x3 inertia matrix 
	 *
	 * This constructor eases the construction of a new body as all the required
	 * parameters can simply be specified as parameters to the constructor.
	 * These are then used to generate the spatial inertia matrix.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param inertia_C the inertia at the center of mass
	 */
	Body(const double &mass,
			const Math::Vector3d &com,
			const Math::Matrix3d &inertia_C) :
		mMass (mass),
		mCenterOfMass(com) {
			Math::Matrix3d com_cross (
					0., -com[2],  com[1],
					com[2],      0., -com[0],
					-com[1],  com[0],      0.
					);
			Math::Matrix3d parallel_axis;
			Math::Matrix3d com_crossT = com_cross;
			com_crossT.transpose();
			parallel_axis = mass * com_cross * com_crossT;

			LOG << "parrallel axis = " << parallel_axis << std::endl;

			Math::Matrix3d pa (parallel_axis);
			Math::Matrix3d mcc = mass * com_cross;
			Math::Matrix3d mccT = mcc;
			mccT.transpose();

			mSpatialInertia.set (
					inertia_C(0,0) + pa(0, 0), inertia_C(0,1) + pa(0, 1), inertia_C(0,2) + pa(0, 2), mcc(0, 0), mcc(0, 1), mcc(0, 2),
					inertia_C(1,0) + pa(1, 0), inertia_C(1,1) + pa(1, 1), inertia_C(1,2) + pa(1, 2), mcc(1, 0), mcc(1, 1), mcc(1, 2),
					inertia_C(2,0) + pa(2, 0), inertia_C(2,1) + pa(2, 1), inertia_C(2,2) + pa(2, 2), mcc(2, 0), mcc(2, 1), mcc(2, 2),
					mccT(0, 0), mccT(0, 1), mccT(0, 2), mass, 0., 0.,
					mccT(1, 0), mccT(1, 1), mccT(1, 2), 0., mass, 0.,
					mccT(2, 0), mccT(2, 1), mccT(2, 2), 0., 0., mass
					);
		}

	/** \brief Constructs a body out of the given parameters
	 *
	 * This constructor eases the construction of a new body as all the required
	 * parameters can simply be specified as parameters to the constructor.
	 * These are then used to generate the spatial inertia matrix.
	 *
	 * \param mass the mass of the body
	 * \param com  the position of the center of mass in the bodies coordinates
	 * \param length the length of the segment (needed to compute the inertia at the CoM
	 * \param gyration_radii the radii of gyration at the center of mass of the body in percentage of the segment length
	 */
	Body(const double &mass,
			const Math::Vector3d &com,
			const double &length,
			const Math::Vector3d &gyration_radii) :
		mMass (mass),
		mCenterOfMass(com) {
			Math::Matrix3d com_cross (
					0., -com[2],  com[1],
					com[2],      0., -com[0],
					-com[1],  com[0],      0.
					);
			Math::Matrix3d parallel_axis;
			parallel_axis = mass * com_cross * com_cross.transpose();

			LOG << "parrallel axis = " << parallel_axis << std::endl;

			Math::Vector3d gr = mass * Math::Vector3d(
					gyration_radii[0] * gyration_radii[0] * length * length,
					gyration_radii[1] * gyration_radii[1] * length * length,
					gyration_radii[2] * gyration_radii[2] * length * length
					);
			Math::Matrix3d pa (parallel_axis);
			Math::Matrix3d mcc = mass * com_cross;
			Math::Matrix3d mccT = mcc.transpose();

			mSpatialInertia.set (
					gr[0] + pa(0, 0), pa(0, 1), pa(0, 2), mcc(0, 0), mcc(0, 1), mcc(0, 2),
					pa(1, 0), gr[1] + pa(1, 1), pa(1, 2), mcc(1, 0), mcc(1, 1), mcc(1, 2),
					pa(2, 0), pa(2, 1), gr[2] + pa(2, 2), mcc(2, 0), mcc(2, 1), mcc(2, 2),
					mccT(0, 0), mccT(0, 1), mccT(0, 2), mass, 0., 0.,
					mccT(1, 0), mccT(1, 1), mccT(1, 2), 0., mass, 0.,
					mccT(2, 0), mccT(2, 1), mccT(2, 2), 0., 0., mass
					);

			LOG << "spatial inertia = " << mSpatialInertia << std::endl;
		}


	~Body() {};

	/// \brief The mass of the body
	double mMass;
	/// \brief The position of the center of mass in body coordinates
	Math::Vector3d mCenterOfMass;
	/// \brief The spatial inertia that contains both mass and inertia information
	Math::SpatialMatrix mSpatialInertia;
};

}

#endif /* _BODY_H */
