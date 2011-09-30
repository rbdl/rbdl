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

	SpatialMatrix X_j;
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

	CHECK (SpatialMatrixCompareEpsilon (test_matrix, X_j, 1.0e-16));
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

	CHECK (SpatialMatrixCompareEpsilon (test_matrix, X_j, TEST_PREC));
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

TEST_FIXTURE(ModelFixture, TestCalcDynamicSingleChain) {
	Body body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDot = VectorNd::Constant  ((size_t) model->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant  ((size_t) model->dof_count, 0.);
	VectorNd Tau = VectorNd::Constant  ((size_t) model->dof_count, 0.);

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a[i] << endl;
	}

	CHECK_EQUAL (-4.905, QDDot[0]);
}

TEST_FIXTURE(ModelFixture, TestCalcDynamicSpatialInertiaSingleChain) {
	// This function checks the value for a non-trivial spatial inertia
	Body body(1., Vector3d (1.5, 1., 1.), Vector3d (1., 2., 3.));
	Joint joint (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint, body);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);


	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a[i] << endl;
	}

	CHECK_EQUAL (-2.3544, QDDot[0]);
}

TEST_FIXTURE(ModelFixture, TestCalcDynamicDoubleChain) {
	Body body_a (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint_a (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

	Body body_b (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint_b (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

//	cout << "--- Double Chain ---" << endl;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a[i] << endl;
	}

	//	cout << LogOutput.str() << endl;
	
	CHECK_CLOSE (-5.88600000000000E+00, QDDot[0], TEST_PREC);
	CHECK_CLOSE ( 3.92400000000000E+00, QDDot[1], TEST_PREC);
}

TEST_FIXTURE(ModelFixture, TestCalcDynamicTripleChain) {
	Body body_a (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint_a (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

	Body body_b (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint_b (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

	Body body_c (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint_c (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(2, Xtrans(Vector3d(1., 0., 0.)), joint_c, body_c);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

	// cout << "--- Triple Chain ---" << endl;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a[i] << endl;
	}

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE (-6.03692307692308E+00, QDDot[0], TEST_PREC);
	CHECK_CLOSE ( 3.77307692307692E+00, QDDot[1], TEST_PREC);
	CHECK_CLOSE ( 1.50923076923077E+00, QDDot[2], TEST_PREC);
}

TEST_FIXTURE(ModelFixture, TestCalcDynamicDoubleChain3D) {
	Body body_a (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint_a (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

	Body body_b (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
	Joint joint_b (
			JointTypeRevolute,
			Vector3d (0., 1., 0.)
			);

	model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

	// cout << "--- Double Chain 3D ---" << endl;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a[i] << endl;
	}

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE (-3.92400000000000E+00, QDDot[0], TEST_PREC);
	CHECK_CLOSE ( 0.00000000000000E+00, QDDot[1], TEST_PREC);
}

TEST_FIXTURE(ModelFixture, TestCalcDynamicSimpleTree3D) {
	Body body_a (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
	Joint joint_a (
			JointTypeRevolute,
			Vector3d (0., 0., 1.)
			);

	model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

	Body body_b1 (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
	Joint joint_b1 (
			JointTypeRevolute,
			Vector3d (0., 1., 0.)
			);

	model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b1, body_b1);

	Body body_c1 (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
	Joint joint_c1 (
			JointTypeRevolute,
			Vector3d (1., 0., 0.)
			);

	model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c1, body_c1);

	Body body_b2 (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
	Joint joint_b2 (
			JointTypeRevolute,
			Vector3d (0., 1., 0.)
			);

	model->AddBody(1, Xtrans(Vector3d(-0.5, 0., 0.)), joint_b2, body_b2);

	Body body_c2 (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
	Joint joint_c2 (
			JointTypeRevolute,
			Vector3d (1., 0., 0.)
			);

	model->AddBody(4, Xtrans(Vector3d(0., -0.5, 0.)), joint_c2, body_c2);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

	// cout << "--- SimpleTree ---" << endl;

	ForwardDynamics(*model, Q, QDot, Tau, QDDot);

	unsigned int i;
	for (i = 0; i < QDDot.size(); i++) {
		LOG << "QDDot[" << i << "] = " << QDDot[i] << endl;
	}

	for (i = 0; i < model->a.size(); i++) {
		LOG << "a[" << i << "]     = " << model->a[i] << endl;
	}

	// cout << LogOutput.str() << endl;

	CHECK_CLOSE (-1.60319066147860E+00, QDDot[0], TEST_PREC);
	CHECK_CLOSE (-5.34396887159533E-01, QDDot[1], TEST_PREC);
	CHECK_CLOSE ( 4.10340466926070E+00, QDDot[2], TEST_PREC);
	CHECK_CLOSE ( 2.67198443579767E-01, QDDot[3], TEST_PREC);
	CHECK_CLOSE ( 5.30579766536965E+00, QDDot[4], TEST_PREC);
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

	ForwardKinematics (*model, q, qdot, qddot);
	body_coords = model->CalcBaseToBodyCoordinates (body_id, base_coords);
	base_coords_back = model->CalcBodyToBaseCoordinates (body_id, body_coords);

	CHECK_ARRAY_CLOSE (base_coords.data(), base_coords_back.data(), 3, TEST_PREC);

	q[0] = 1.;
	q[1] = 0.2;
	q[2] = -2.3;
	q[3] = -2.3;
	q[4] = 0.03;
	q[5] = -0.23;

	ForwardKinematics (*model, q, qdot, qddot);
	body_coords = model->CalcBaseToBodyCoordinates (body_id, base_coords);
	base_coords_back = model->CalcBodyToBaseCoordinates (body_id, body_coords);

	CHECK_ARRAY_CLOSE (base_coords.data(), base_coords_back.data(), 3, TEST_PREC);
}

/*
TEST_FIXTURE ( ModelFixture, TestUpdate ) {
	Body body;

	unsigned int body_id = model->SetFloatingBaseBody (body);

	VectorNd q (model->dof_count);
	VectorNd qdot (model->dof_count);
	VectorNd qddot (model->dof_count);
	VectorNd tau (model->dof_count);

	VectorNd input (model->dof_count);
	VectorNd reference (model->dof_count + 1);

	reference[0] = 0;
	reference[1] = 0.;
	reference[2] = 1.;
	reference[3] = 2.;
	reference[4] = 3.;
	reference[5] = 4.;
	reference[6] = 5.;

	input[0] = 0.;
	input[1] = 1.;
	input[2] = 2.;
	input[3] = 3.;
	input[4] = 4.;
	input[5] = 5.;

	model->Update (input, input * 2, input * 4, input * 8);

	CHECK_ARRAY_CLOSE (reference.data(), model->q.data(), reference.size(), TEST_PREC);

	reference = reference * 2;
	CHECK_ARRAY_CLOSE (reference.data(), model->qdot.data(), reference.size(), TEST_PREC);

	reference = reference * 2;
	CHECK_ARRAY_CLOSE (reference.data(), model->qddot.data(), reference.size(), TEST_PREC);

	reference = reference * 2;
	CHECK_ARRAY_CLOSE (reference.data(), model->tau.data(), reference.size(), TEST_PREC);
}
*/
