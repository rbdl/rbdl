#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Body.h"
#include "ArticulatedFigure.h"
#include "Logging.h"

using namespace std;

struct ArticulatedFigureFixture {
	ArticulatedFigureFixture () {
		figure = new ArticulatedFigure;
		figure->Init();
	}
	~ArticulatedFigureFixture () {
		delete figure;
	}
	ArticulatedFigure *figure;
};

TEST_FIXTURE(ArticulatedFigureFixture, TestInit) {
	CHECK_EQUAL (1, figure->mParentId.size());
	CHECK_EQUAL (1, figure->mBodies.size());
	CHECK_EQUAL (0, figure->mJointType.size());
	CHECK_EQUAL (1, figure->mJointTransform.size());
	CHECK_EQUAL (1, figure->mSpatialInertia.size());
	CHECK_EQUAL (1, figure->mSpatialJointAxes.size());
	CHECK_EQUAL (1, figure->mSpatialVelocities.size());
	CHECK_EQUAL (0, figure->q.size());
	CHECK_EQUAL (0, figure->qdot.size());
	CHECK_EQUAL (0, figure->qddot.size());
	CHECK_EQUAL (0, figure->tau.size());
	CHECK_EQUAL (1, figure->mJoints.size());
	CHECK_EQUAL (1, figure->mBodyOrientation.size());
	CHECK_EQUAL (1, figure->mBodyPosition.size());
	CHECK_EQUAL (1, figure->mBodyVelocity.size());
}

TEST_FIXTURE(ArticulatedFigureFixture, TestAddBodyDimensions) {
	Body body;
	Joint joint;

	joint.mJointType = JointTypeRevolute;

	figure->AddBody(0, joint, body); 

	CHECK_EQUAL (2, figure->mParentId.size());
	CHECK_EQUAL (2, figure->mBodies.size());
	CHECK_EQUAL (1, figure->mJointType.size());
	CHECK_EQUAL (2, figure->mJointTransform.size());
	CHECK_EQUAL (2, figure->mSpatialInertia.size());
	CHECK_EQUAL (2, figure->mSpatialJointAxes.size());
	CHECK_EQUAL (2, figure->mSpatialVelocities.size());
	CHECK_EQUAL (1, figure->q.size());
	CHECK_EQUAL (1, figure->qdot.size());
	CHECK_EQUAL (1, figure->qddot.size());
	CHECK_EQUAL (1, figure->tau.size());
	CHECK_EQUAL (2, figure->mJoints.size());
	CHECK_EQUAL (2, figure->mBodyOrientation.size());
	CHECK_EQUAL (2, figure->mBodyPosition.size());
	CHECK_EQUAL (2, figure->mBodyVelocity.size());
}

/** \brief Tests whether the joint and body information stored in the ArticulatedFigure are computed correctly 
 */
TEST_FIXTURE(ArticulatedFigureFixture, TestAddBodySpatialValues) {
	Body body;
	Joint joint;

	joint.mJointType = JointTypeRevolute;
	joint.mJointAxis.set(0., 0., 1.);
	joint.mJointCenter.set(3., 7., 4.);

	figure->AddBody(0, joint, body); 

	SpatialVector spatial_joint_axis(0., 0., 1., 0., 0., 0.);
	CHECK_EQUAL (spatial_joint_axis, figure->mSpatialJointAxes.at(1));

	SpatialMatrix joint_transform(
			1.,  0.,  0.,  0.,  0.,  0.,
			0.,  1.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  4., -7.,  1.,  0.,  0.,
		 -4.,  0.,  3.,  0.,  1.,  0.,
			7., -3.,  0.,  0.,  0.,  1.
			);
	CHECK_EQUAL (joint_transform, figure->mJointTransform.at(1));

	// \Todo: Dynamic properties
}

TEST_FIXTURE(ArticulatedFigureFixture, TestBodyJointTransformation) {
	Body body;
	Joint joint;

	joint.mJointType = JointTypeRevolute;
	joint.mJointCenter.set(1., 0., 0.);

	figure->AddBody(0, joint, body); 

	SpatialMatrix joint_transformation (SpatialMatrixIdentity);

	joint_transformation(3, 1) =  joint.mJointCenter[2]; 
	joint_transformation(3, 2) = -joint.mJointCenter[1]; 

	joint_transformation(4, 0) = -joint.mJointCenter[2]; 
	joint_transformation(4, 2) =  joint.mJointCenter[0]; 

	joint_transformation(5, 0) =  joint.mJointCenter[1]; 
	joint_transformation(5, 1) = -joint.mJointCenter[0]; 

	CHECK_EQUAL (joint_transformation, figure->mJointTransform[1]);
}

TEST_FIXTURE(ArticulatedFigureFixture, TestJointComputeTransform) {
	Joint joint;
	Body body;

	joint.mJointType = JointTypeRevolute;
	joint.mJointAxis.set(0., 0., 1.);

	figure->AddBody (0, joint, body);

	SpatialMatrix test (
			1.,  0.,  0.,  0.,  0.,  0.,
			0.,  1.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  0.,  0.,  1.,  0.,  0.,
			0.,  0.,  0.,  0.,  1.,  0.,
			0.,  0.,  0.,  0.,  0.,  1.
			);

	figure->q.at(0) = 0.;
	figure->qdot.at(0) = 1.;

	CHECK_EQUAL (test, figure->JointComputeTransform(1));

	test.set (
			0., -1.,  0.,  0.,  0.,  0.,
			1.,  0.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  0.,  0.,  0., -1.,  0.,
			0.,  0.,  0.,  1.,  0.,  0.,
			0.,  0.,  0.,  0.,  0.,  1.
			);

	figure->q.at(0) = M_PI*0.5;
	CHECK (SpatialMatrixCompareEpsilon (test, figure->JointComputeTransform(1), 1.0e-16));
}

TEST_FIXTURE(ArticulatedFigureFixture, TestCalcVelocitiesSimple) {
	ClearLogOutput();

	Body body;
	Joint joint;

	joint.mJointType = JointTypeRevolute;
	joint.mJointCenter.set (0., 0., 0.);
	joint.mJointAxis.set(0., 0., 1.);

	Body endeffector;
	Joint fixed_joint;
	fixed_joint.mJointType = JointTypeFixed;
	fixed_joint.mJointCenter.set(1., 0., 0.);

	figure->AddBody(0, joint, body);
	figure->AddBody(1, fixed_joint, endeffector);

	figure->qdot.at(0) = 1.;
	figure->CalcVelocities();

	SpatialVector spatial_body_velocity (0., 0., 1., 0., 1., 0.);
	CHECK_EQUAL (spatial_body_velocity, figure->mSpatialVelocities.at(2));
//	std::cout << LogOutput.str() << std::endl;
	ClearLogOutput();

	figure->qdot.at(0) = -1.;
	figure->CalcVelocities();

	spatial_body_velocity.set (0., 0., -1., 0., -1., 0.);
	CHECK_EQUAL (spatial_body_velocity, figure->mSpatialVelocities.at(2));
	std::cout << LogOutput.str() << std::endl;
	ClearLogOutput();

}

	
