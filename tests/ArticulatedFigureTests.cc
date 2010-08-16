#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Body.h"
#include "ArticulatedFigure.h"
#include "Logging.h"

using namespace std;

struct ModelFixture {
	ModelFixture () {
		model = new Model;
		model->Init();
	}
	~ModelFixture () {
		delete model;
	}
	Model *model;
};

TEST_FIXTURE(ModelFixture, TestInit) {
	CHECK_EQUAL (1, model->lambda.size());

	CHECK_EQUAL (1, model->q.size());
	CHECK_EQUAL (1, model->qdot.size());
	CHECK_EQUAL (1, model->qddot.size());
	CHECK_EQUAL (1, model->tau.size());
	CHECK_EQUAL (1, model->v.size());
	
	CHECK_EQUAL (1, model->mJoints.size());
	CHECK_EQUAL (1, model->S.size());

	CHECK_EQUAL (1, model->mSpatialInertia.size());
	
	CHECK_EQUAL (1, model->X_lambda.size());
	CHECK_EQUAL (1, model->X_base.size());
	CHECK_EQUAL (1, model->mBodies.size());

	CHECK_EQUAL (1, model->mBodyOrientation.size());
}

TEST_FIXTURE(ModelFixture, TestAddBodyDimensions) {
	Body body;
	Joint joint (
			JointTypeRevolute,
			Vector3d(0., 0., 1.),
			Vector3d(0., 0., 0.)
			);

	model->AddBody(0, joint, body); 

	CHECK_EQUAL (2, model->lambda.size());

	CHECK_EQUAL (2, model->q.size());
	CHECK_EQUAL (2, model->qdot.size());
	CHECK_EQUAL (2, model->qddot.size());
	CHECK_EQUAL (2, model->tau.size());
	CHECK_EQUAL (2, model->v.size());
	
	CHECK_EQUAL (2, model->mJoints.size());
	CHECK_EQUAL (2, model->S.size());

	CHECK_EQUAL (2, model->mSpatialInertia.size());
	
	CHECK_EQUAL (2, model->X_lambda.size());
	CHECK_EQUAL (2, model->X_base.size());
	CHECK_EQUAL (2, model->mBodies.size());
}


/** \brief Tests whether the joint and body information stored in the Model are computed correctly 
 */
TEST_FIXTURE(ModelFixture, TestAddBodySpatialValues) {
	Body body;
	Joint joint (
		JointTypeRevolute,
		Vector3d(0., 0., 1.),
		Vector3d(3., 7., 4.)
		);

	model->AddBody(0, joint, body); 

	SpatialVector spatial_joint_axis(0., 0., 1., 0., 0., 0.);
	CHECK_EQUAL (spatial_joint_axis, joint.mJointAxis);

	SpatialMatrix joint_transform(
			1.,  0.,  0.,  0.,  0.,  0.,
			0.,  1.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  4., -7.,  1.,  0.,  0.,
		 -4.,  0.,  3.,  0.,  1.,  0.,
			7., -3.,  0.,  0.,  0.,  1.
			);
	CHECK_EQUAL (joint_transform, joint.mJointTransform);

	// \Todo: Dynamic properties
}

TEST_FIXTURE(ModelFixture, TestjcalcSimple) {
	Body body;
	Joint joint (
		JointTypeRevolute,
		Vector3d(0., 0., 1.),
		Vector3d(1., 0., 0.)
		);

	model->AddBody(0, joint, body);

	SpatialMatrix X_j;
	SpatialVector v_j;

	jcalc (*model, 1, X_j, v_j, 0., 1.);

	SpatialMatrix test_matrix (
			1.,  0.,  0.,  0.,  0.,  0.,
			0.,  1.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  0.,  0.,  1.,  0.,  0.,
			0.,  0.,  0.,  0.,  1.,  0.,
			0.,  0.,  0.,  0.,  0.,  1.
			);
	SpatialVector test_vector (
			0., 0., 1., 0., 0., 0.
			);

	CHECK (SpatialMatrixCompareEpsilon (test_matrix, X_j, 1.0e-16));
	CHECK (SpatialVectorCompareEpsilon (test_vector, v_j, 1.0e-16));

	jcalc (*model, 1, X_j, v_j, M_PI * 0.5, 1.);

	test_matrix.set (
			0., -1.,  0.,  0.,  0.,  0.,
			1.,  0.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  0.,  0.,  0., -1.,  0.,
			0.,  0.,  0.,  1.,  0.,  0.,
			0.,  0.,  0.,  0.,  0.,  1.
			);

	CHECK (SpatialMatrixCompareEpsilon (test_matrix, X_j, 1.0e-16));
	CHECK (SpatialVectorCompareEpsilon (test_vector, v_j, 1.0e-16));
}

/// \brief Checks the multiplication of a SpatialMatrix with a SpatialVector
TEST(TestSpatialMatrixTimesSpatialVector) {
	SpatialMatrix s_matrix (
			1., 0., 0., 0., 0., 7.,
			0., 2., 0., 0., 8., 0.,
			0., 0., 3., 9., 0., 0.,
			0., 0., 6., 4., 0., 0.,
			0., 5., 0., 0., 5., 0.,
			4., 0., 0., 0., 0., 6.
			);
	SpatialVector s_vector (
			1., 2., 3., 4., 5., 6.
			);

	SpatialVector result;
	result = s_matrix * s_vector;

	SpatialVector test_result (
			43., 44., 45., 34., 35., 40.
			);
	CHECK_EQUAL (test_result, result);
}

TEST_FIXTURE(ModelFixture, TestCalcVelocitiesSimple) {
	ClearLogOutput();

	Body body;
	Joint joint (
			JointTypeRevolute,
			Vector3d (0., 0., 1.),
			Vector3d (0., 0., 0.)
			);

	Body endeffector;
	Joint fixed_joint (
			JointTypeFixed,
			Vector3d (0., 0., 0.),
			Vector3d (1., 0., 0.)
			);

	model->AddBody(0, joint, body);
	model->AddBody(1, fixed_joint, endeffector);

	std::vector<double> Q;
	std::vector<double> QDot;
	std::vector<double> QDDot;
	std::vector<double> Tau;

	// Initialization of the input vectors
	Q.push_back(0.); Q.push_back(0.);
	QDot.push_back(0.); QDot.push_back(0.);
	QDDot.push_back(0.); QDDot.push_back(0.);
	Tau.push_back(0.); Tau.push_back(0.);

	QDot.at(0) = 1.;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	SpatialVector spatial_body_velocity (0., 0., 1., 0., 1., 0.);
	CHECK_EQUAL (spatial_body_velocity, model->v.at(2));
	// std::cout << LogOutput.str() << std::endl;
	ClearLogOutput();

	QDot.at(0) = -1.;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	spatial_body_velocity.set (0., 0., -1., 0., -1., 0.);
	CHECK_EQUAL (spatial_body_velocity, model->v.at(2));
	// std::cout << LogOutput.str() << std::endl;
	ClearLogOutput();

}
