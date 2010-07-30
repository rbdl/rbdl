#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Body.h"
#include "ArticulatedFigure.h"

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
	CHECK_EQUAL (0, figure->mJointTransform.size());
	CHECK_EQUAL (1, figure->mSpatialInertia.size());
	CHECK_EQUAL (0, figure->q.size());
	CHECK_EQUAL (0, figure->qdot.size());
	CHECK_EQUAL (0, figure->qddot.size());
	CHECK_EQUAL (0, figure->tau.size());
	CHECK_EQUAL (0, figure->mJoints.size());
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
	CHECK_EQUAL (1, figure->q.size());
	CHECK_EQUAL (1, figure->qdot.size());
	CHECK_EQUAL (1, figure->qddot.size());
	CHECK_EQUAL (1, figure->tau.size());
	CHECK_EQUAL (1, figure->mJoints.size());
	CHECK_EQUAL (2, figure->mBodyOrientation.size());
	CHECK_EQUAL (2, figure->mBodyPosition.size());
	CHECK_EQUAL (2, figure->mBodyVelocity.size());
}

TEST_FIXTURE(ArticulatedFigureFixture, TestBodyJointTransformation) {
	Body body;
	Joint joint;

	joint.mJointType = JointTypeRevolute;

	figure->AddBody(0, joint, body); 

	Matrix3d bla, bla2;
	CHECK_EQUAL (bla, bla2);

	SpatialMatrix joint_transformation;
	SpatialMatrix test = figure->mJointTransform[1];

	CHECK_EQUAL (joint_transformation, test);
}

TEST_FIXTURE(ArticulatedFigureFixture, TestCalcVelocitiesSimple) {
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

	figure->q[1] = 1.;

	figure->CalcVelocities();

	Vector3d velocity (0., 1., 0.);
	CHECK_EQUAL (figure->mBodyVelocity[2], velocity);
}

	
