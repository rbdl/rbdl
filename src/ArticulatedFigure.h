#ifndef ARTICULATEDFIGURE_H
#define ARTICULATEDFIGURE_H

#include <cmlwrapper.h>
#include <vector>

enum JointType {
	JointTypeUndefined = 0,
	JointTypeFixed,
	JointTypeRevolute,
	JointTypePrismatic
};

/** \brief Describes all properties of a single body */
struct Body {
	Body() :
		mInertia (1., 1., 1.),
		mCenterOfMass (0., 0., 0.),
		mMass (1.) {};

	Vector3d mInertia;
	Vector3d mCenterOfMass;
	double mMass;
};

/** \brief Describes a joint relative to the predecessor body */
struct Joint {
	Joint() :
		mJointCenter (0., 0., 0.),
		mJointAxis (0., 0., 0.),
		mJointType (JointTypeUndefined) {};

	/// \brief The joint center in the predecessors frame
	Vector3d mJointCenter;
	/// \brief The axis of the joint
	Vector3d mJointAxis;
	/// \brief Type of joint (rotational or prismatic)
	JointType mJointType;
};

/** \brief Contains all information of the model */
struct ArticulatedFigure {
	/// \brief The id of the parents body
	std::vector<unsigned int> mParentId;
	/// \brief Type of joint i that connects body (i-1) with body i
	std::vector<JointType> mJointType;
	/// \brief The center of the joint that connects body (i-1) with body i in the predecessors frame
	std::vector<SpatialMatrix> mJointTransform;
	/// \brief The spatial inertia of body i
	std::vector<SpatialMatrix> mSpatialInertia;

	/// \brief The joint position
	std::vector<double> q;
	/// \brief The joint velocity
	std::vector<double> qdot;
	/// \brief The joint acceleration
	std::vector<double> qddot;
	/// \brief The force / torque applied at joint i
	std::vector<double> tau;

	/// \brief All joints
	std::vector<Joint> mJoints;

	/// \brief All bodies
	std::vector<Body> mBodies;
	std::vector<Matrix3d> mBodyOrientation;
	std::vector<Vector3d> mBodyPosition;
	std::vector<Vector3d> mBodyVelocity;

	void Init ();
	void AddBody (const unsigned int parent_id, const Joint &joint, const Body &body);
	void CalcVelocities ();
};

#endif /* ARTICULATEDFIGURE_H */
