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

TEST (TestForwardDynamicsLagrangian) {
	Model model;
	model.Init();
	Body base_body(1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));

	model.SetFloatingBaseBody(base_body);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);
	
	VectorNd QDDot_aba = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDDot_lagrangian = VectorNd::Constant ((size_t) model.dof_count, 0.);

	Q[0] = 1.1;
	Q[1] = 1.2;
	Q[2] = 1.3;
	Q[3] = 0.1;
	Q[4] = 0.2;
	Q[5] = 0.3;

	QDot[0] = 1.1;
	QDot[1] = -1.2;
	QDot[2] = 1.3;
	QDot[3] = -0.1;
	QDot[4] = 0.2;
	QDot[5] = -0.3;

	Tau[0] = 2.1;
	Tau[1] = 2.2;
	Tau[2] = 2.3;
	Tau[3] = 1.1;
	Tau[4] = 1.2;
	Tau[5] = 1.3;

	ForwardDynamics(model, Q, QDot, Tau, QDDot_aba);
	ForwardDynamicsLagrangian(model, Q, QDot, Tau, QDDot_lagrangian);

	CHECK_ARRAY_CLOSE (QDDot_aba.data(), QDDot_lagrangian.data(), QDDot_aba.size(), TEST_PREC);
}

/* 
 * A simple test for a model with 3 rotational dof. The reference value was
 * computed with Featherstones spatial_v1 code. This test was written
 * because my benchmark tool showed up inconsistencies, however this was
 * due to the missing gravity term. But as the test now works, I just leave
 * it here.
 */
TEST (TestForwardDynamics3DoFModel) {
	Model model;
	model.Init();

	model.gravity = Vector3d (0., -9.81, 0.);

	Body null_body (0., Vector3d(0., 0., 0.), 1., Vector3d (0., 0., 0.));
	Body base_body (1., Vector3d(0., 0.5, 0.), 1., Vector3d (1., 1., 1.));

	Joint joint_rot_z (JointTypeRevolute, Vector3d (0., 0., 1.));
	Joint joint_rot_y (JointTypeRevolute, Vector3d (0., 1., 0.));
	Joint joint_rot_x (JointTypeRevolute, Vector3d (1., 0., 0.));

	unsigned int base_id_rot_z, base_id_rot_y;
	// thes are the ids of the baseren with masses
	unsigned int base_id = std::numeric_limits<unsigned int>::max();

	// we can reuse both bodies and joints as they are copied
	base_id_rot_z = model.AddBody (0, Xtrans (Vector3d(0., 0., 0.)), joint_rot_z, null_body);
	base_id_rot_y = model.AddBody (base_id_rot_z, Xtrans (Vector3d(0., 0., 0.)), joint_rot_y, null_body);
	base_id = model.AddBody (base_id_rot_y, Xtrans (Vector3d(0., 0., 0.)), joint_rot_x, base_body);

	// Initialization of the input vectors
	VectorNd Q = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd Tau = VectorNd::Constant ((size_t) model.dof_count, 0.);
	
	VectorNd QDDot = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDDot_ref = VectorNd::Constant ((size_t) model.dof_count, 0.);

	Q[0] = 1.;

	ClearLogOutput();

	ForwardDynamics (model, Q, QDot, Tau, QDDot);

//	cout << LogOutput.str() << endl;

	QDDot_ref[0] = 3.301932144386186;
	
	CHECK_ARRAY_CLOSE (QDDot_ref.data(), QDDot.data(), QDDot.size(), TEST_PREC);
}

	
