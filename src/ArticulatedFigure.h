#ifndef ARTICULATEDFIGURE_H
#define ARTICULATEDFIGURE_H

#include <cmlwrapper.h>
#include <vector>
#include <assert.h>
#include <iostream>

enum JointType {
	JointTypeUndefined = 0,
	JointTypeFixed,
	JointTypeRevolute,
	JointTypePrismatic
};

/** \brief Describes all properties of a single body */
struct Body {
	Body() :
		mSpatialInertia (
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.,	
				0., 0., 0., 0., 0., 0.
				),
		mCenterOfMass (0., 0., 0.),
		mMass (1.) { };
	Body(const Body &body) :
		mSpatialInertia (body.mSpatialInertia),
		mCenterOfMass (body.mCenterOfMass),
		mMass (body.mMass) {};
	Body& operator= (const Body &body) {
		if (this != &body) {
			mSpatialInertia = body.mSpatialInertia;
			mCenterOfMass = body.mCenterOfMass;
			mMass = body.mMass;
		}
	}
	Body(const double &mass,
			const Vector3d &com,
			const Vector3d &gyration_radii) :
		mMass (mass),
		mCenterOfMass(com) {
			Matrix3d com_cross (
					0., -com[1],  com[2],
					com[1],      0., -com[0],
					-com[2],  com[0],      0.
					);
			Matrix3d parallel_axis;
			parallel_axis = mass * com_cross * cml::transpose(com_cross);

			Vector3d gr (gyration_radii);
			Matrix3d pa (parallel_axis);
			Matrix3d mcc = mass * com_cross;
			Matrix3d mccT = transpose(mcc);

			mSpatialInertia.set (
					gr[0] + pa(0, 0), pa(0, 1), pa(0, 2), mcc(0, 0), mcc(0, 1), mcc(0, 2),
					pa(1, 0), gr[1] + pa(1, 1), pa(1, 2), mcc(1, 0), mcc(1, 1), mcc(1, 2),
					pa(2, 0), pa(2, 1), gr[2] + pa(2, 2), mcc(2, 0), mcc(2, 1), mcc(2, 2),
					mccT(0, 0), mccT(0, 1), mccT(0, 2), mass, 0., 0.,
					mccT(1, 0), mccT(1, 1), mccT(1, 2), 0., mass, 0.,
					mccT(2, 0), mccT(2, 1), mccT(2, 2), 0., 0., mass
					);
		}

	~Body() {};

	SpatialMatrix mSpatialInertia;
	Vector3d mCenterOfMass;
	double mMass;
};

/** \brief Describes a joint relative to the predecessor body */
struct Joint {
	Joint() :
		mJointTransform (
				1., 0., 0., 0., 0., 0.,
				0., 1., 0., 0., 0., 0.,
				0., 0., 1., 0., 0., 0.,
				0., 0., 0., 1., 0., 0.,
				0., 0., 0., 0., 1., 0.,
				0., 0., 0., 0., 0., 1.
				),
		mJointAxis (
				0., 0., 0.,
				0., 0., 0.
				),
		mJointType (JointTypeUndefined) {};
	Joint (const Joint &joint) :
		mJointTransform (joint.mJointTransform),
		mJointAxis (joint.mJointAxis),
		mJointType (joint.mJointType) {};
	Joint& operator= (const Joint &joint) {
		if (this != &joint) {
			mJointTransform = joint.mJointTransform;
			mJointAxis = joint.mJointAxis;
			mJointType = joint.mJointType;
		}
		return *this;
	}

	// Special Constructors
	Joint (
			const JointType joint_type,
			const Vector3d &joint_axis,
			const Vector3d &parent_translation
			) {
		// Some assertions, as we concentrate on simple cases

		// Only rotation around the Z-axis
		assert ( joint_type == JointTypeRevolute || joint_type == JointTypeFixed
				);
		mJointType = joint_type;

		if (joint_type == JointTypeRevolute) {
			assert ( joint_axis == Vector3d (1., 0., 0.)
					|| joint_axis == Vector3d (0., 1., 0.)
					|| joint_axis == Vector3d (0., 0., 1.));

			mJointTransform = Xtrans(parent_translation * -1.);
			mJointAxis.set (
					joint_axis[0],
					joint_axis[1], 
					joint_axis[2], 
					0., 0., 0.
					);

		} else if (joint_type == JointTypeFixed) {
			mJointTransform = Xtrans(parent_translation * -1.);
			mJointAxis.set (
					joint_axis[0],
					joint_axis[1], 
					joint_axis[2], 
					0., 0., 0.
					);
			mJointAxis.set (0., 0., 0., 0., 0., 0.);
		}
	}

	/// \brief The transformation from the parents origin to the joint center (fixed in the parents frame!)
	SpatialMatrix mJointTransform;
	/// \brief The spatial axis of the joint
	SpatialVector mJointAxis;
	/// \brief Type of joint (rotational or prismatic)
	JointType mJointType;
};

/** \brief Contains all information of the model
 *
 */
struct Model {
	// Structural information

	/// \brief The id of the parents body
	std::vector<unsigned int> lambda;

	// State information

	/** \brief The joint position
	 * 
	 * Warning: to have an easier numbering in the algorithm the state vector
	 * has NDOF + 1 elements. However element with index 0 is not used!
	 * 
	 * q[0] - unused
	 * q[1] - joint 1
	 * q[2] - joint 2
	 * ...
	 * q[NDOF] - joint NDOF
	 *
	 */
	std::vector<double> q;
	/// \brief The joint velocity
	std::vector<double> qdot;
	/// \brief The joint acceleration
	std::vector<double> qddot;
	/// \brief The force / torque applied at joint i
	std::vector<double> tau;
	/// \brief The spatial velocity of body i
	std::vector<SpatialVector> v;
	/// \brief The spatial acceleration of body i
	std::vector<SpatialVector> a;

	////////////////////////////////////
	// Joints\t

	/// \brief All joints
	std::vector<Joint> mJoints;
	/// \brief The joint axis for joint i
	std::vector<SpatialVector> S;

	////////////////////////////////////
	// Dynamics variables

	/// \brief The velocity dependent spatial acceleration
	std::vector<SpatialVector> c;
	/// \brief The spatial inertia of body i
	std::vector<SpatialMatrix> IA;
	/// \brief The spatial bias force
	std::vector<SpatialVector> pA;
	/// \brief Temporary variable U_i (RBDA p. 130)
	std::vector<SpatialVector> U;
	/// \brief Temporary variable D_i (RBDA p. 130)
	std::vector<double> d;
	/// \brief Temporary variable u (RBDA p. 130)
	std::vector<double> u;


	////////////////////////////////////
	// Bodies

	/// \brief Transformation from the parent body to the current body
	std::vector<SpatialMatrix> X_lambda;
	/// \brief Transformation from the base to bodies reference frame
	std::vector<SpatialMatrix> X_base;

	/** \brief All bodies 0 ... N_B, including the base
	 * mBodies[0] - base body
	 * mBodies[1] - 1st movable body
	 * ...
	 * mBodies[N_B] - N_Bth movable body
	 */
	std::vector<Body> mBodies;
	std::vector<Matrix3d> mBodyOrientation;

	void Init ();
	void AddBody (
			const unsigned int parent_id,
			const Joint &joint,
			const Body &body
			);
};

void jcalc (
		const Model &model,
		const unsigned int &joint_id,
		SpatialMatrix &XJ,
		SpatialVector &S,
		SpatialVector &v_i,
		SpatialVector &c_i,
		const double &q,
		const double &qdot
		);

void ForwardDynamics (
		Model &model,
		const std::vector<double> &Q,
		const std::vector<double> &QDot,
		const std::vector<double> &Tau,
		std::vector<double> &QDDot
		);

#endif /* ARTICULATEDFIGURE_H */
