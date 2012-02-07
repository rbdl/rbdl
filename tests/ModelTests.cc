#include <UnitTest++.h>

#include <iostream>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"
#include "Dynamics.h"

using namespace std;
using namespace SpatialAlgebra;
using namespace RigidBodyDynamics;

const double TEST_PREC = 1.0e-14;

struct ModelFixture {
	ModelFixture () {
		ClearLogOutput();
		model = new Model;
		model->Init();
		model->gravity = Vector3d (0., -9.81, 0.);
	}
	~ModelFixture () {
		delete model;
	}
	Model *model;
};

TEST_FIXTURE(ModelFixture, TestInit) {
	CHECK_EQUAL (1u, model->lambda.size());
	CHECK_EQUAL (1u, model->mu.size());
	CHECK_EQUAL (0u, model->dof_count);

	CHECK_EQUAL (1u, model->q.size());
	CHECK_EQUAL (1u, model->qdot.size());
	CHECK_EQUAL (1u, model->qddot.size());
	CHECK_EQUAL (1u, model->tau.size());
	CHECK_EQUAL (1u, model->v.size());
	CHECK_EQUAL (1u, model->a.size());
	
	CHECK_EQUAL (1u, model->mJoints.size());
	CHECK_EQUAL (1u, model->S.size());

	CHECK_EQUAL (1u, model->c.size());
	CHECK_EQUAL (1u, model->IA.size());
	CHECK_EQUAL (1u, model->pA.size());
	CHECK_EQUAL (1u, model->U.size());
	CHECK_EQUAL (1u, model->d.size());
	CHECK_EQUAL (1u, model->u.size());
	CHECK_EQUAL (1u, model->Ic.size());
	
	CHECK_EQUAL (1u, model->X_lambda.size());
	CHECK_EQUAL (1u, model->X_base.size());
	CHECK_EQUAL (1u, model->mBodies.size());
}

TEST_FIXTURE(ModelFixture, TestAddBodyDimensions) {
	Body body;
	Joint joint (
			JointTypeRevolute,
			Vector3d(0., 0., 1.)
			);

	unsigned int body_id = 0;
	body_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body); 

	CHECK_EQUAL (1u, body_id);
	CHECK_EQUAL (2u, model->lambda.size());
	CHECK_EQUAL (2u, model->mu.size());
	CHECK_EQUAL (1u, model->dof_count);

	CHECK_EQUAL (2u, model->q.size());
	CHECK_EQUAL (2u, model->qdot.size());
	CHECK_EQUAL (2u, model->qddot.size());
	CHECK_EQUAL (2u, model->tau.size());
	CHECK_EQUAL (2u, model->v.size());
	CHECK_EQUAL (2u, model->a.size());
	
	CHECK_EQUAL (2u, model->mJoints.size());
	CHECK_EQUAL (2u, model->S.size());

	CHECK_EQUAL (2u, model->c.size());
	CHECK_EQUAL (2u, model->IA.size());
	CHECK_EQUAL (2u, model->pA.size());
	CHECK_EQUAL (2u, model->U.size());
	CHECK_EQUAL (2u, model->d.size());
	CHECK_EQUAL (2u, model->u.size());
	CHECK_EQUAL (2u, model->Ic.size());

	SpatialVector spatial_zero;
	spatial_zero.setZero();
	
	CHECK_EQUAL (2u, model->X_lambda.size());
	CHECK_EQUAL (2u, model->X_base.size());
	CHECK_EQUAL (2u, model->mBodies.size());
}

TEST_FIXTURE(ModelFixture, TestExperimentalFloatingBodyDimensions) {
	Body body;

	model->experimental_floating_base = true;
	model->SetFloatingBaseBody(body);

	CHECK_EQUAL (1u, model->lambda.size());
	CHECK_EQUAL (1u, model->mu.size());
	CHECK_EQUAL (6u, model->dof_count);

	CHECK_EQUAL (7u, model->q.size());
	CHECK_EQUAL (7u, model->qdot.size());
	CHECK_EQUAL (7u, model->qddot.size());
	CHECK_EQUAL (7u, model->tau.size());
	CHECK_EQUAL (1u, model->v.size());
	CHECK_EQUAL (1u, model->a.size());
	
	CHECK_EQUAL (1u, model->mJoints.size());
	CHECK_EQUAL (1u, model->S.size());

	CHECK_EQUAL (1u, model->c.size());
	CHECK_EQUAL (1u, model->IA.size());
	CHECK_EQUAL (1u, model->pA.size());
	CHECK_EQUAL (1u, model->U.size());
	CHECK_EQUAL (1u, model->d.size());
	CHECK_EQUAL (1u, model->u.size());

	SpatialVector spatial_zero;
	spatial_zero.setZero();
	
	CHECK_EQUAL (1u, model->X_lambda.size());
	CHECK_EQUAL (1u, model->X_base.size());
	CHECK_EQUAL (1u, model->mBodies.size());
}

TEST_FIXTURE(ModelFixture, TestFloatingBodyDimensions) {
	Body body;

	model->SetFloatingBaseBody(body);

	CHECK_EQUAL (7u, model->lambda.size());
	CHECK_EQUAL (7u, model->mu.size());
	CHECK_EQUAL (6u, model->dof_count);

	CHECK_EQUAL (7u, model->q.size());
	CHECK_EQUAL (7u, model->qdot.size());
	CHECK_EQUAL (7u, model->qddot.size());
	CHECK_EQUAL (7u, model->tau.size());
	CHECK_EQUAL (7u, model->v.size());
	CHECK_EQUAL (7u, model->a.size());
	
	CHECK_EQUAL (7u, model->mJoints.size());
	CHECK_EQUAL (7u, model->S.size());

	CHECK_EQUAL (7u, model->c.size());
	CHECK_EQUAL (7u, model->IA.size());
	CHECK_EQUAL (7u, model->pA.size());
	CHECK_EQUAL (7u, model->U.size());
	CHECK_EQUAL (7u, model->d.size());
	CHECK_EQUAL (7u, model->u.size());

	SpatialVector spatial_zero;
	spatial_zero.setZero();
	
	CHECK_EQUAL (7u, model->X_lambda.size());
	CHECK_EQUAL (7u, model->X_base.size());
	CHECK_EQUAL (7u, model->mBodies.size());
}

/** \brief Tests whether the joint and body information stored in the Model are computed correctly 
 */
TEST_FIXTURE(ModelFixture, TestAddBodySpatialValues) {
	Body body;
	Joint joint (
		JointTypeRevolute,
		Vector3d(0., 0., 1.)
		);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body); 

	SpatialVector spatial_joint_axis(0., 0., 1., 0., 0., 0.);
	CHECK_EQUAL (spatial_joint_axis, joint.mJointAxis);

	// \Todo: Dynamic properties
}

TEST_FIXTURE(ModelFixture, TestAddBodyTestBodyName) {
	Body body;
	Joint joint (
		JointTypeRevolute,
		Vector3d(0., 0., 1.)
		);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body, "mybody"); 

	unsigned int body_id = model->GetBodyId("mybody");

	CHECK_EQUAL (1, body_id);
	CHECK_EQUAL (std::numeric_limits<unsigned int>::max(), model->GetBodyId("unknownbody"));
}

TEST_FIXTURE(ModelFixture, TestjcalcSimple) {
	Body body;
	Joint joint (
		JointTypeRevolute,
		Vector3d(0., 0., 1.)
		);

	model->AddBody(0, Xtrans(Vector3d(1., 0., 0.)), joint, body);

	SpatialTransform X_j;
	SpatialVector S;
	SpatialVector v_j;
	SpatialVector c;

	jcalc (*model, 1, X_j, S, v_j, c, 0., 1.);

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
	SpatialVector test_joint_axis (
			0., 0., 1., 0., 0., 0.
			);

	CHECK (SpatialMatrixCompareEpsilon (test_matrix, X_j.toMatrix(), 1.0e-16));
	CHECK (SpatialVectorCompareEpsilon (test_vector, v_j, 1.0e-16));
	CHECK_EQUAL (test_joint_axis, S);

	jcalc (*model, 1, X_j, S, v_j, c, M_PI * 0.5, 1.);

	test_matrix.set (
			0.,  1.,  0.,  0.,  0.,  0.,
		 -1.,  0.,  0.,  0.,  0.,  0.,
			0.,  0.,  1.,  0.,  0.,  0.,
			0.,  0.,  0.,  0.,  1.,  0.,
			0.,  0.,  0., -1.,  0.,  0.,
			0.,  0.,  0.,  0.,  0.,  1.
			);

	CHECK (SpatialMatrixCompareEpsilon (test_matrix, X_j.toMatrix(), TEST_PREC));
	CHECK (SpatialVectorCompareEpsilon (test_vector, v_j, TEST_PREC));
	CHECK_EQUAL (test_joint_axis, S);
}

TEST_FIXTURE(ModelFixture, TestCalcVelocitiesSimple) {
	Body body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	Body endeffector (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint fixed_joint (
			JointTypeFixed,
			Vector3d (0., 0., 0.)
			);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);
	model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), fixed_joint, endeffector);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Zero ((size_t) model->dof_count);
	VectorNd QDot = VectorNd::Zero ((size_t) model->dof_count);
	VectorNd QDDot = VectorNd::Zero ((size_t) model->dof_count);
	VectorNd Tau = VectorNd::Zero ((size_t) model->dof_count);

	QDot[0] = 1.;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	SpatialVector spatial_body_velocity (0., 0., 1., 0., 1., 0.);
	CHECK_EQUAL (spatial_body_velocity, model->v.at(2));
	// std::cout << LogOutput.str() << std::endl;
	ClearLogOutput();

	QDot[0] = -1.;
	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	spatial_body_velocity.set (0., 0., -1., 0., -1., 0.);
	CHECK_EQUAL (spatial_body_velocity, model->v.at(2));
	// std::cout << LogOutput.str() << std::endl;
}

TEST_FIXTURE ( ModelFixture, TestTransformBaseToLocal ) {
	Body body;
	unsigned int body_id = model->SetFloatingBaseBody (body);

	VectorNd q = VectorNd::Zero (model->dof_count);
	VectorNd qdot = VectorNd::Zero (model->dof_count);
	VectorNd qddot = VectorNd::Zero (model->dof_count);
	VectorNd tau = VectorNd::Zero (model->dof_count);

	Vector3d base_coords (0., 0., 0.);
	Vector3d body_coords;
	Vector3d base_coords_back;

	UpdateKinematics (*model, q, qdot, qddot);
	body_coords = CalcBaseToBodyCoordinates (*model, q, body_id, base_coords, false);
	base_coords_back = CalcBodyToBaseCoordinates (*model, q, body_id, body_coords, false);

	CHECK_ARRAY_CLOSE (base_coords.data(), base_coords_back.data(), 3, TEST_PREC);

	q[0] = 1.;
	q[1] = 0.2;
	q[2] = -2.3;
	q[3] = -2.3;
	q[4] = 0.03;
	q[5] = -0.23;

	UpdateKinematics (*model, q, qdot, qddot);
	body_coords = CalcBaseToBodyCoordinates (*model, q, body_id, base_coords, false);
	base_coords_back = CalcBodyToBaseCoordinates (*model, q, body_id, body_coords, false);

	CHECK_ARRAY_CLOSE (base_coords.data(), base_coords_back.data(), 3, TEST_PREC);
}
