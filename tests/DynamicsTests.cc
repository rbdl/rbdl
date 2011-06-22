#include <UnitTest++.h>

#include <iostream>
#include <limits>

#include "mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Kinematics.h"
#include "Dynamics.h"
#include "Contacts.h"

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

/*
 * Another simple 3 dof model test which showed some problems when
 * computing forward dynamics with the Lagrangian formulation. A proplem
 * occured as the CRBA does not update the kinematics of the model, hence
 * invalid body transformations and joint axis were used in the CRBA.
 * Running the CRBA after the InverseDynamics calculation fixes this. This
 * test ensures that the error does not happen when calling ForwardLagrangian.
 */
TEST (TestForwardDynamics3DoFModelLagrangian) {
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
	
	VectorNd QDDot_ab = VectorNd::Constant ((size_t) model.dof_count, 0.);
	VectorNd QDDot_lagrangian = VectorNd::Constant ((size_t) model.dof_count, 0.);

	Q[1] = 1.;
	ClearLogOutput();

	Q[0] = 0.;
	Q[1] = 1.;
	Q[2] = 0.;
	ForwardDynamicsLagrangian (model, Q, QDot, Tau, QDDot_lagrangian);
	Q[0] = 0.;
	Q[1] = 0.;
	Q[2] = 1.;
	ForwardDynamicsLagrangian (model, Q, QDot, Tau, QDDot_lagrangian);
	ForwardDynamics (model, Q, QDot, Tau, QDDot_ab);

//	cout << QDDot_lagrangian << endl;
//	cout << LogOutput.str() << endl;

	CHECK_ARRAY_CLOSE (QDDot_ab.data(), QDDot_lagrangian.data(), QDDot_ab.size(), TEST_PREC);
}

/* 
 * This is a test for a model where I detected incosistencies between the
 * Lagragian method and the ABA.
 */
TEST (TestForwardDynamicsTwoLegModelLagrangian) {
	Model *model = NULL;

	unsigned int hip_id,
							 upper_leg_right_id,
							 lower_leg_right_id,
							 foot_right_id,
							 upper_leg_left_id,
							 lower_leg_left_id,
							 foot_left_id;
	Body hip_body,
			 upper_leg_right_body,
			 lower_leg_right_body,
			 foot_right_body,
			 upper_leg_left_body,
			 lower_leg_left_body,
			 foot_left_body;

	Joint joint_rot_z, joint_rot_y, joint_rot_x;
	Joint joint_trans_z, joint_trans_y, joint_trans_x;

	std::vector<ContactInfo> contact_data_right;
	std::vector<ContactInfo> contact_data_left;
	std::vector<ContactInfo> contact_data_both;

	model = new Model();

	model->Init();

	model->gravity = Vector3d (0., -9.81, 0.);

	joint_rot_z = Joint (JointTypeRevolute, Vector3d (0., 0., 1.));
	joint_rot_y = Joint (JointTypeRevolute, Vector3d (0., 1., 0.));
	joint_rot_x = Joint (JointTypeRevolute, Vector3d (1., 0., 0.));

	joint_trans_z = Joint (JointTypePrismatic, Vector3d (0., 0., 1.));
	joint_trans_y = Joint (JointTypePrismatic, Vector3d (0., 1., 0.));
	joint_trans_x = Joint (JointTypePrismatic, Vector3d (1., 0., 0.));

	Body null_body (0., Vector3d (0., 0., 0.), Vector3d (0., 0., 0.));

	// hip
	hip_body = Body (1., Vector3d (0., 0., 0.), Vector3d (1., 1., 1.));

	// lateral right
	upper_leg_right_body = Body (1., Vector3d (0., -0.25, 0.), Vector3d (1., 1., 1.));
	lower_leg_right_body = Body (1., Vector3d (0., -0.25, 0.), Vector3d (1., 1., 1.));
	foot_right_body = Body (1., Vector3d (0.15, -0.1, 0.), Vector3d (1., 1., 1.));

	// lateral left
	upper_leg_left_body = Body (1., Vector3d (0., -0.25, 0.), Vector3d (1., 1., 1.));
	lower_leg_left_body = Body (1., Vector3d (0., -0.25, 0.), Vector3d (1., 1., 1.));
	foot_left_body = Body (1., Vector3d (0.15, -0.1, 0.), Vector3d (1., 1., 1.));

	// temporary value to store most recent body id
	unsigned int temp_id;

	// add hip to the model (planar, 3 DOF)
	temp_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_trans_x, null_body);
	temp_id = model->AddBody (temp_id, Xtrans (Vector3d (0., 0., 0.)), joint_trans_y, null_body);
	hip_id = model->AddBody (temp_id, Xtrans (Vector3d (0., 0., 0.)), joint_rot_z, hip_body);

	//
	// right leg
	//

	// add right upper leg
	temp_id = model->AddBody (hip_id, Xtrans (Vector3d(0., 0., 0.)), joint_rot_z, upper_leg_right_body);
	upper_leg_right_id = temp_id;

	// add the right lower leg (only one DOF)
	temp_id = model->AddBody (temp_id, Xtrans (Vector3d(0., -0.5, 0.)), joint_rot_z, lower_leg_right_body);
	lower_leg_right_id = temp_id;

	// add the right foot (1 DOF)
	temp_id = model->AddBody (temp_id, Xtrans (Vector3d(0., -0.5, 0.)), joint_rot_z, foot_right_body);
	foot_right_id = temp_id;

	//
	// left leg
	//

	// add left upper leg
	temp_id = model->AddBody (hip_id, Xtrans (Vector3d(0., 0., 0.)), joint_rot_z, upper_leg_left_body);
	upper_leg_left_id = temp_id;

	// add the left lower leg (only one DOF)
	temp_id = model->AddBody (temp_id, Xtrans (Vector3d(0., -0.5, 0.)), joint_rot_z, lower_leg_left_body);
	lower_leg_left_id = temp_id;

	// add the left foot (1 DOF)
	temp_id = model->AddBody (temp_id, Xtrans (Vector3d(0., -0.5, 0.)), joint_rot_z, foot_left_body);
	foot_left_id = temp_id;

	LOG << "--- model created (" << model->dof_count << " DOF) ---" << endl;
	
	// contact data
	ContactInfo right_x (foot_right_id, Vector3d (0., 0., 0.), Vector3d (1., 0., 0.));
	ContactInfo right_y (foot_right_id, Vector3d (0., 0., 0.), Vector3d (0., 1., 0.));
	ContactInfo right_z (foot_right_id, Vector3d (0., 0., 0.), Vector3d (0., 0., 1.));

	ContactInfo left_x (foot_left_id, Vector3d (0., 0., 0.), Vector3d (1., 0., 0.));
	ContactInfo left_y (foot_left_id, Vector3d (0., 0., 0.), Vector3d (0., 1., 0.));
	ContactInfo left_z (foot_left_id, Vector3d (0., 0., 0.), Vector3d (0., 0., 1.));

	// right contact
	contact_data_right.push_back (right_x);
	contact_data_right.push_back (right_y);
	//	contact_data_right.push_back (right_z);

	// left contact
	contact_data_left.push_back (left_x);
	contact_data_left.push_back (left_y);
	//	contact_data_left.push_back (left_z);

	// both contact
	contact_data_both.push_back (right_x);
	contact_data_both.push_back (right_y);
	contact_data_both.push_back (right_z);

	contact_data_both.push_back (left_x);
	contact_data_both.push_back (left_y);
	contact_data_both.push_back (left_z);

	VectorNd Q(model->dof_count);
	VectorNd QDot(model->dof_count);
	VectorNd Tau(model->dof_count);
	VectorNd QDDot(model->dof_count);
	VectorNd QDDotABA(model->dof_count);

	Q[0] = 0.8;
	Q[1] = -7.76326e-06;
	Q[2] = -1.58205e-07;
	Q[3] = 1.57391e-07;
	Q[4] = -1.03029e-09;
	Q[5] = 7.92143e-08;
	Q[6] = 1.57391e-07;
	Q[7] = -1.03029e-09;
	Q[8] = 7.92143e-08;

	QDot[0] = -1.77845e-06;
	QDot[1] = -0.00905283;
	QDot[2] = -0.000184484;
	QDot[3] = 0.000183536;
	QDot[4] = -1.20144e-06;
	QDot[5] = 9.23727e-05;
	QDot[6] = 0.000183536;
	QDot[7] = -1.20144e-06;
	QDot[8] = 9.23727e-05;

	Tau[0] = 0;
	Tau[1] = 0;
	Tau[2] = 0;
	Tau[3] = 0.1;
	Tau[4] = 0.1;
	Tau[5] = 0.1;
	Tau[6] = 0.1;
	Tau[7] = 0.1;
	Tau[8] = 0.1;

// QDDot =  6.31843e-07 -6.12442e-07  9.22595e-14   3.3712e-07  4.27368e-07 -7.91795e-07   3.3712e-07  4.27368e-07 -7.91795e-07
// QDDAB = -0.00192794    -9.81419        -0.2    0.198972 -0.00130243    0.100141    0.198972 -0.00130243    0.100141

	ForwardDynamics (*model, Q, QDot, Tau, QDDotABA);
	ClearLogOutput();
	ForwardDynamicsLagrangian (*model, Q, QDot, Tau, QDDot);

//	cout << LogOutput.str() << endl;

	// run it again to make sure the calculations give the same results and
	// no invalid state information lingering in the model structure is being used
	ForwardDynamics (*model, Q, QDot, Tau, QDDotABA);
	ClearLogOutput();
	ForwardDynamicsLagrangian (*model, Q, QDot, Tau, QDDot);

	CHECK_ARRAY_CLOSE (QDDotABA.data(), QDDot.data(), QDDotABA.size(), TEST_PREC);
}

