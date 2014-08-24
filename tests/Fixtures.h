#include "rbdl/rbdl.h"

struct FixedBase3DoF {
	FixedBase3DoF () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 *      Z
		 *      *---* 
		 *      |
		 *      |
		 *  Z   |
		 *  O---*
		 *      Y
		 */

		body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_a = Joint( SpatialVector(0., 0., 1., 0., 0., 0.));

		body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

		body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
		joint_b = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));

		body_b_id = model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

		body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
		joint_c = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

		body_c_id = model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		Tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

		point_position = Vector3d::Zero (3);
		point_acceleration = Vector3d::Zero (3);

		ref_body_id = 0;

		ClearLogOutput();
	}
	~FixedBase3DoF () {
		delete model;
	}
	
	RigidBodyDynamics::Model *model;

	unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
	RigidBodyDynamics::Body body_a, body_b, body_c;
	RigidBodyDynamics::Joint joint_a, joint_b, joint_c;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;

	RigidBodyDynamics::Math::Vector3d point_position, point_acceleration;
};

struct FixedBase6DoF {
	FixedBase6DoF () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;

		model->gravity = Vector3d  (0., -9.81, 0.);

		/* 3 DoF (rot.) joint at base
		 * 3 DoF (rot.) joint child origin
		 *
		 *          X Contact point (ref child)
		 *          |
		 *    Base  |
		 *   / body |
		 *  O-------*
		 *           \
		 *             Child body
		 */

		// base body (3 DoF)
		base_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_base_rot_z = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));
		base_rot_z_id = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_z, base_rot_z);

		base_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_base_rot_y = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));
		base_rot_y_id = model->AppendBody (Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_y, base_rot_y);

		base_rot_x = Body (
				1.,
				Vector3d (0.5, 0., 0.),
				Vector3d (1., 1., 1.)
				);
		joint_base_rot_x = Joint ( SpatialVector (1., 0., 0., 0., 0., 0.));
		base_rot_x_id = model->AddBody (base_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_base_rot_x, base_rot_x);

		// child body (3 DoF)
		child_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_z = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));
		child_rot_z_id = model->AddBody (base_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_rot_z, child_rot_z);

		child_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_y = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));
		child_rot_y_id = model->AddBody (child_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_y, child_rot_y);

		child_rot_x = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_rot_x = Joint ( SpatialVector (1., 0., 0., 0., 0., 0.));
		child_rot_x_id = model->AddBody (child_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_x, child_rot_x);

		Q = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		QDDot = VectorNd::Constant (model->mBodies.size() - 1, 0.);
		Tau = VectorNd::Constant (model->mBodies.size() - 1, 0.);

		contact_body_id = child_rot_x_id;
		contact_point = Vector3d  (0.5, 0.5, 0.);
		contact_normal = Vector3d  (0., 1., 0.);

		ClearLogOutput();
	}
	
	~FixedBase6DoF () {
		delete model;
	}
	RigidBodyDynamics::Model *model;

	unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id,
		child_rot_z_id, child_rot_y_id, child_rot_x_id,
		base_body_id;

	RigidBodyDynamics::Body base_rot_z, base_rot_y, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x;

	RigidBodyDynamics::Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;

	unsigned int contact_body_id;
	RigidBodyDynamics::Math::Vector3d contact_point;
	RigidBodyDynamics::Math::Vector3d contact_normal;
	RigidBodyDynamics::ConstraintSet constraint_set;
};

struct Human36 {
	RigidBodyDynamics::Model *model;
	RigidBodyDynamics::Model *model_emulated;
	RigidBodyDynamics::Model *model_3dof;

	RigidBodyDynamics::Math::VectorNd q;
	RigidBodyDynamics::Math::VectorNd qdot;
	RigidBodyDynamics::Math::VectorNd qddot;
	RigidBodyDynamics::Math::VectorNd tau;

	RigidBodyDynamics::Math::VectorNd qddot_emulated;
	RigidBodyDynamics::Math::VectorNd qddot_3dof;

	RigidBodyDynamics::ConstraintSet constraints_1B1C_emulated;
	RigidBodyDynamics::ConstraintSet constraints_1B4C_emulated;
	RigidBodyDynamics::ConstraintSet constraints_4B4C_emulated;

	RigidBodyDynamics::ConstraintSet constraints_1B1C_3dof;
	RigidBodyDynamics::ConstraintSet constraints_1B4C_3dof;
	RigidBodyDynamics::ConstraintSet constraints_4B4C_3dof;

	enum SegmentName {
		SegmentPelvis = 0,
		SegmentThigh,
		SegmentShank,
		SegmentFoot,
		SegmentMiddleTrunk,
		SegmentUpperTrunk,
		SegmentUpperArm,
		SegmentLowerArm,
		SegmentHand,
		SegmentHead,
		SegmentNameLast
	};

	double SegmentLengths[SegmentNameLast];
	double SegmentMass[SegmentNameLast];
	double SegmentCOM[SegmentNameLast][3];
	double SegmentRadiiOfGyration[SegmentNameLast][3];

	void initParameters () {
		SegmentLengths[SegmentPelvis     ] = 0.1457;
		SegmentLengths[SegmentThigh      ] = 0.4222;
		SegmentLengths[SegmentShank      ] = 0.4403;
		SegmentLengths[SegmentFoot       ] = 0.1037;
		SegmentLengths[SegmentMiddleTrunk] = 0.2155;
		SegmentLengths[SegmentUpperTrunk ] = 0.2421;
		SegmentLengths[SegmentUpperArm   ] = 0.2817;
		SegmentLengths[SegmentLowerArm   ] = 0.2689;
		SegmentLengths[SegmentHand       ] = 0.0862;
		SegmentLengths[SegmentHead       ] = 0.2429;

		SegmentMass[SegmentPelvis     ] = 0.8154;
		SegmentMass[SegmentThigh      ] = 10.3368;
		SegmentMass[SegmentShank      ] = 3.1609;
		SegmentMass[SegmentFoot       ] = 1.001;
		SegmentMass[SegmentMiddleTrunk] = 16.33;
		SegmentMass[SegmentUpperTrunk ] = 15.96;
		SegmentMass[SegmentUpperArm   ] = 1.9783;
		SegmentMass[SegmentLowerArm   ] = 1.1826;
		SegmentMass[SegmentHand       ] = 0.4453;
		SegmentMass[SegmentHead       ] = 5.0662;

		SegmentCOM[SegmentPelvis     ][0] = 0.;
		SegmentCOM[SegmentPelvis     ][1] = 0.;
		SegmentCOM[SegmentPelvis     ][2] = 0.0891;

		SegmentCOM[SegmentThigh      ][0] = 0.;
		SegmentCOM[SegmentThigh      ][1] = 0.;
		SegmentCOM[SegmentThigh      ][2] = -0.1729;

		SegmentCOM[SegmentShank      ][0] = 0.;
		SegmentCOM[SegmentShank      ][1] = 0.;
		SegmentCOM[SegmentShank      ][2] = -0.1963;

		SegmentCOM[SegmentFoot       ][0] = 0.1254;
		SegmentCOM[SegmentFoot       ][1] = 0.;
		SegmentCOM[SegmentFoot       ][2] = -0.0516;

		SegmentCOM[SegmentMiddleTrunk][0] = 0.;
		SegmentCOM[SegmentMiddleTrunk][1] = 0.;
		SegmentCOM[SegmentMiddleTrunk][2] = 0.1185;

		SegmentCOM[SegmentUpperTrunk ][0] = 0.;
		SegmentCOM[SegmentUpperTrunk ][1] = 0.;
		SegmentCOM[SegmentUpperTrunk ][2] = 0.1195;

		SegmentCOM[SegmentUpperArm   ][0] = 0.;
		SegmentCOM[SegmentUpperArm   ][1] = 0.;
		SegmentCOM[SegmentUpperArm   ][2] = -0.1626;

		SegmentCOM[SegmentLowerArm   ][0] = 0.;
		SegmentCOM[SegmentLowerArm   ][1] = 0.;
		SegmentCOM[SegmentLowerArm   ][2] = -0.1230;

		SegmentCOM[SegmentHand       ][0] = 0.;
		SegmentCOM[SegmentHand       ][1] = 0.;
		SegmentCOM[SegmentHand       ][2] = -0.0680;

		SegmentCOM[SegmentHead       ][0] = 0.;
		SegmentCOM[SegmentHead       ][1] = 0.;
		SegmentCOM[SegmentHead       ][2] = 1.1214;

		SegmentRadiiOfGyration[SegmentPelvis     ][0] = 0.0897;
		SegmentRadiiOfGyration[SegmentPelvis     ][1] = 0.0855;
		SegmentRadiiOfGyration[SegmentPelvis     ][2] = 0.0803;

		SegmentRadiiOfGyration[SegmentThigh      ][0] = 0.1389;
		SegmentRadiiOfGyration[SegmentThigh      ][1] = 0.0629;
		SegmentRadiiOfGyration[SegmentThigh      ][2] = 0.1389;

		SegmentRadiiOfGyration[SegmentShank      ][0] = 0.1123;
		SegmentRadiiOfGyration[SegmentShank      ][1] = 0.0454;
		SegmentRadiiOfGyration[SegmentShank      ][2] = 0.1096;

		SegmentRadiiOfGyration[SegmentFoot       ][0] = 0.0267;
		SegmentRadiiOfGyration[SegmentFoot       ][1] = 0.0129;
		SegmentRadiiOfGyration[SegmentFoot       ][2] = 0.0254;

		SegmentRadiiOfGyration[SegmentMiddleTrunk][0] = 0.0970;
		SegmentRadiiOfGyration[SegmentMiddleTrunk][1] = 0.1009;
		SegmentRadiiOfGyration[SegmentMiddleTrunk][2] = 0.0825;

		SegmentRadiiOfGyration[SegmentUpperTrunk ][0] = 0.1273;
		SegmentRadiiOfGyration[SegmentUpperTrunk ][1] = 0.1172;
		SegmentRadiiOfGyration[SegmentUpperTrunk ][2] = 0.0807;

		SegmentRadiiOfGyration[SegmentUpperArm   ][0] = 0.0803;
		SegmentRadiiOfGyration[SegmentUpperArm   ][1] = 0.0758;
		SegmentRadiiOfGyration[SegmentUpperArm   ][2] = 0.0445;

		SegmentRadiiOfGyration[SegmentLowerArm   ][0] = 0.0742;
		SegmentRadiiOfGyration[SegmentLowerArm   ][1] = 0.0713;
		SegmentRadiiOfGyration[SegmentLowerArm   ][2] = 0.0325;

		SegmentRadiiOfGyration[SegmentHand       ][0] = 0.0541;
		SegmentRadiiOfGyration[SegmentHand       ][1] = 0.0442;
		SegmentRadiiOfGyration[SegmentHand       ][2] = 0.0346;

		SegmentRadiiOfGyration[SegmentHead       ][0] = 0.0736;
		SegmentRadiiOfGyration[SegmentHead       ][1] = 0.0634;
		SegmentRadiiOfGyration[SegmentHead       ][2] = 0.0765;
	};

	RigidBodyDynamics::Body create_body (SegmentName segment) {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		Matrix3d inertia_C (Matrix3d::Zero());
		inertia_C(0,0) = pow(SegmentRadiiOfGyration[segment][0] * SegmentLengths[segment], 2) * SegmentMass[segment];
		inertia_C(1,1) = pow(SegmentRadiiOfGyration[segment][1] * SegmentLengths[segment], 2) * SegmentMass[segment];
		inertia_C(2,2) = pow(SegmentRadiiOfGyration[segment][2] * SegmentLengths[segment], 2) * SegmentMass[segment];

		return RigidBodyDynamics::Body (
				SegmentMass[segment],
				RigidBodyDynamics::Math::Vector3d (
					SegmentCOM[segment][0],
					SegmentCOM[segment][1],
					SegmentCOM[segment][2]
					),
				inertia_C);
	}

	void generate () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		Body pelvis_body = create_body (SegmentPelvis);
		Body thigh_body = create_body (SegmentThigh);
		Body shank_body = create_body (SegmentShank);
		Body foot_body = create_body (SegmentFoot);
		Body middle_trunk_body = create_body (SegmentMiddleTrunk);
		Body upper_trunk_body = create_body (SegmentUpperTrunk);
		Body upperarm_body = create_body (SegmentUpperArm);
		Body lowerarm_body = create_body (SegmentLowerArm);
		Body hand_body = create_body (SegmentHand);
		Body head_body = create_body (SegmentHead);

		Joint free_flyer (
				SpatialVector (0., 0., 0., 1., 0., 0.),
				SpatialVector (0., 0., 0., 0., 1., 0.),
				SpatialVector (0., 0., 0., 0., 0., 1.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (0., 0., 1., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.)
				);

		Joint rot_yxz_emulated (
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.),
				SpatialVector (0., 0., 1., 0., 0., 0.)
				);

		Joint rot_yxz_3dof = Joint(JointTypeEulerYXZ);

		Joint rot_yz (
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (0., 0., 1., 0., 0., 0.)
				);

		Joint rot_y (
				SpatialVector (0., 1., 0., 0., 0., 0.)
				);

		Joint fixed (JointTypeFixed);

		// Generate emulated model
		model_emulated->gravity = Vector3d (0., 0., -9.81);

		unsigned int pelvis_id = model_emulated->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), free_flyer, pelvis_body, "pelvis");

		// right leg
		model_emulated->AddBody (pelvis_id, Xtrans(Vector3d(0., -0.0872, 0.)), rot_yxz_emulated, thigh_body, "thigh_r");
		model_emulated->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_r");
		model_emulated->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_r");

		// left leg
		model_emulated->AddBody (pelvis_id, Xtrans(Vector3d(0., 0.0872, 0.)), rot_yxz_emulated, thigh_body, "thigh_l");
		model_emulated->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_l");
		model_emulated->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_l");

		// trunk
		model_emulated->AddBody (pelvis_id, Xtrans(Vector3d(0., 0., SegmentLengths[SegmentPelvis])), rot_yxz_emulated, middle_trunk_body, "middletrunk");
		unsigned int uppertrunk_id = model_emulated->AppendBody (Xtrans(Vector3d(0., 0., SegmentLengths[SegmentMiddleTrunk])), fixed, upper_trunk_body, "uppertrunk");

		// right arm
		model_emulated->AddBody (uppertrunk_id, Xtrans(Vector3d(0., -0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz_emulated, upperarm_body, "upperarm_r");
		model_emulated->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_r");
		model_emulated->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_r");

		// left arm
		model_emulated->AddBody (uppertrunk_id, Xtrans(Vector3d(0.,  0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz_emulated, upperarm_body, "upperarm_l");
		model_emulated->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_l");
		model_emulated->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_l");

		// head	
		model_emulated->AddBody (uppertrunk_id, Xtrans(Vector3d(0., 0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz_emulated, upperarm_body, "head");

		// Generate 3dof model
		model_3dof->gravity = Vector3d (0., 0., -9.81);

		pelvis_id = model_3dof->AddBody (0, Xtrans (Vector3d (0., 0., 0.)), free_flyer, pelvis_body, "pelvis");

		// right leg
		model_3dof->AddBody (pelvis_id, Xtrans(Vector3d(0., -0.0872, 0.)), rot_yxz_3dof, thigh_body, "thigh_r");
		model_3dof->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_r");
		model_3dof->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_r");

		// left leg
		model_3dof->AddBody (pelvis_id, Xtrans(Vector3d(0., 0.0872, 0.)), rot_yxz_3dof, thigh_body, "thigh_l");
		model_3dof->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentThigh])), rot_y, shank_body, "shank_l");
		model_3dof->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentShank])), rot_yz, foot_body, "foot_l");

		// trunk
		model_3dof->AddBody (pelvis_id, Xtrans(Vector3d(0., 0., SegmentLengths[SegmentPelvis])), rot_yxz_3dof, middle_trunk_body, "middletrunk");
		uppertrunk_id = model_3dof->AppendBody (Xtrans(Vector3d(0., 0., SegmentLengths[SegmentMiddleTrunk])), fixed, upper_trunk_body, "uppertrunk");

		// right arm
		model_3dof->AddBody (uppertrunk_id, Xtrans(Vector3d(0., -0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz_3dof, upperarm_body, "upperarm_r");
		model_3dof->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_r");
		model_3dof->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_r");

		// left arm
		model_3dof->AddBody (uppertrunk_id, Xtrans(Vector3d(0.,  0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz_3dof, upperarm_body, "upperarm_l");
		model_3dof->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentUpperArm])), rot_y, lowerarm_body, "lowerarm_l");
		model_3dof->AppendBody (Xtrans(Vector3d(0., 0., -SegmentLengths[SegmentLowerArm])), rot_yz, hand_body, "hand_l");

		// head	
		model_3dof->AddBody (uppertrunk_id, Xtrans(Vector3d(0., 0.1900, SegmentLengths[SegmentUpperTrunk])), rot_yxz_3dof, upperarm_body, "head");
	}

	void initConstraintSets () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		unsigned int foot_r_emulated = model_emulated->GetBodyId ("foot_r");
		unsigned int foot_l_emulated = model_emulated->GetBodyId ("foot_l");
		unsigned int hand_r_emulated = model_emulated->GetBodyId ("hand_r");
		unsigned int hand_l_emulated = model_emulated->GetBodyId ("hand_l");

		constraints_1B1C_emulated.AddConstraint (foot_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_1B1C_emulated.Bind (*model_emulated);	

		constraints_1B4C_emulated.AddConstraint (foot_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_1B4C_emulated.AddConstraint (foot_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_1B4C_emulated.AddConstraint (foot_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_1B4C_emulated.AddConstraint (foot_r_emulated, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_1B4C_emulated.Bind (*model_emulated);	

		constraints_4B4C_emulated.AddConstraint (foot_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_emulated.AddConstraint (foot_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_4B4C_emulated.AddConstraint (foot_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_4B4C_emulated.AddConstraint (foot_r_emulated, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

		constraints_4B4C_emulated.AddConstraint (foot_l_emulated, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_emulated.AddConstraint (foot_l_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_4B4C_emulated.AddConstraint (foot_l_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_4B4C_emulated.AddConstraint (foot_l_emulated, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

		constraints_4B4C_emulated.AddConstraint (hand_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_emulated.AddConstraint (hand_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_4B4C_emulated.AddConstraint (hand_r_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_4B4C_emulated.AddConstraint (hand_r_emulated, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

		constraints_4B4C_emulated.AddConstraint (hand_l_emulated, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_emulated.AddConstraint (hand_l_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_4B4C_emulated.AddConstraint (hand_l_emulated, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_4B4C_emulated.AddConstraint (hand_l_emulated, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_emulated.Bind (*model);

		unsigned int foot_r_3dof = model_3dof->GetBodyId ("foot_r");
		unsigned int foot_l_3dof = model_3dof->GetBodyId ("foot_l");
		unsigned int hand_r_3dof = model_3dof->GetBodyId ("hand_r");
		unsigned int hand_l_3dof = model_3dof->GetBodyId ("hand_l");

		constraints_1B1C_3dof.AddConstraint (foot_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_1B1C_3dof.Bind (*model_3dof);	

		constraints_1B4C_3dof.AddConstraint (foot_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_1B4C_3dof.AddConstraint (foot_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_1B4C_3dof.AddConstraint (foot_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_1B4C_3dof.AddConstraint (foot_r_3dof, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_1B4C_3dof.Bind (*model_3dof);	

		constraints_4B4C_3dof.AddConstraint (foot_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_3dof.AddConstraint (foot_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_4B4C_3dof.AddConstraint (foot_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_4B4C_3dof.AddConstraint (foot_r_3dof, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

		constraints_4B4C_3dof.AddConstraint (foot_l_3dof, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_3dof.AddConstraint (foot_l_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_4B4C_3dof.AddConstraint (foot_l_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_4B4C_3dof.AddConstraint (foot_l_3dof, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

		constraints_4B4C_3dof.AddConstraint (hand_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_3dof.AddConstraint (hand_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_4B4C_3dof.AddConstraint (hand_r_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_4B4C_3dof.AddConstraint (hand_r_3dof, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));

		constraints_4B4C_3dof.AddConstraint (hand_l_3dof, Vector3d (0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_3dof.AddConstraint (hand_l_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 1., 0.));
		constraints_4B4C_3dof.AddConstraint (hand_l_3dof, Vector3d (0.1, 0., -0.05), Vector3d (0., 0., 1.));
		constraints_4B4C_3dof.AddConstraint (hand_l_3dof, Vector3d (-0.1, 0., -0.05), Vector3d (1., 0., 0.));
		constraints_4B4C_3dof.Bind (*model);
	}

	Human36 () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		initParameters();
		model_emulated = new RigidBodyDynamics::Model();
		model_3dof = new RigidBodyDynamics::Model();
		model = model_emulated;
		generate();
		initConstraintSets();

		q = VectorNd::Zero (model_emulated->q_size);
		qdot = VectorNd::Zero (model_emulated->qdot_size);
		qddot = VectorNd::Zero (model_emulated->qdot_size);
		tau = VectorNd::Zero (model_emulated->qdot_size);

		qddot_emulated = VectorNd::Zero (model_emulated->qdot_size);
		qddot_3dof= VectorNd::Zero (model_emulated->qdot_size);
	};
	~Human36 () {
		delete model_emulated;
		delete model_3dof;
	}

};

struct FloatingBase12DoF {
	FloatingBase12DoF () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;

		model->gravity = Vector3d  (0., -9.81, 0.);

		/* 3 DoF (rot.) joint at base
		 * 3 DoF (rot.) joint child origin
		 *
		 *          X Contact point (ref child)
		 *          |
		 *    Base  |
		 *   / body |
		 *  O-------*
		 *           \
		 *             Child body
		 */

		base_rot_x = Body (
				1.,
				Vector3d (0.5, 0., 0.),
				Vector3d (1., 1., 1.)
				);
		base_rot_x_id = model->SetFloatingBaseBody(base_rot_x);

		// child body 1 (3 DoF)
		child_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_z = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));
		child_rot_z_id = model->AddBody (base_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_rot_z, child_rot_z);

		child_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_rot_y = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));
		child_rot_y_id = model->AddBody (child_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_y, child_rot_y);

		child_rot_x = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_rot_x = Joint ( SpatialVector (1., 0., 0., 0., 0., 0.));
		child_rot_x_id = model->AddBody (child_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_rot_x, child_rot_x);

		// child body (3 DoF)
		child_2_rot_z = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_2_rot_z = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));
		child_2_rot_z_id = model->AddBody (child_rot_x_id, Xtrans (Vector3d (1., 0., 0.)), joint_child_2_rot_z, child_2_rot_z);

		child_2_rot_y = Body (
				0.,
				Vector3d (0., 0., 0.),
				Vector3d (0., 0., 0.)
				);
		joint_child_2_rot_y = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));
		child_2_rot_y_id = model->AddBody (child_2_rot_z_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_2_rot_y, child_2_rot_y);

		child_2_rot_x = Body (
				1.,
				Vector3d (0., 0.5, 0.),
				Vector3d (1., 1., 1.)
				);
		joint_child_2_rot_x = Joint ( SpatialVector (1., 0., 0., 0., 0., 0.));
		child_2_rot_x_id = model->AddBody (child_2_rot_y_id, Xtrans (Vector3d (0., 0., 0.)), joint_child_2_rot_x, child_2_rot_x);

		Q = VectorNd::Constant (model->dof_count, 0.);
		QDot = VectorNd::Constant (model->dof_count, 0.);
		QDDot = VectorNd::Constant (model->dof_count, 0.);
		Tau = VectorNd::Constant (model->dof_count, 0.);

		ClearLogOutput();
	}
	
	~FloatingBase12DoF () {
		delete model;
	}
	RigidBodyDynamics::Model *model;

	unsigned int base_rot_z_id, base_rot_y_id, base_rot_x_id,
		child_rot_z_id, child_rot_y_id, child_rot_x_id,
		child_2_rot_z_id, child_2_rot_y_id,child_2_rot_x_id,
		base_body_id;

	RigidBodyDynamics::Body base_rot_z, base_rot_y, base_rot_x,
		child_rot_z, child_rot_y, child_rot_x,
		child_2_rot_z, child_2_rot_y, child_2_rot_x;

	RigidBodyDynamics::Joint joint_base_rot_z, joint_base_rot_y, joint_base_rot_x,
		joint_child_rot_z, joint_child_rot_y, joint_child_rot_x,
		joint_child_2_rot_z, joint_child_2_rot_y, joint_child_2_rot_x;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;
};

struct SimpleFixture {
	SimpleFixture () {
		ClearLogOutput();
		model = new RigidBodyDynamics::Model;
		model->gravity = RigidBodyDynamics::Math::Vector3d (0., -9.81, 0.);
	}
	~SimpleFixture () {
		delete model;
	}
	void ResizeVectors () {
		Q = RigidBodyDynamics::Math::VectorNd::Zero (model->dof_count);
		QDot = RigidBodyDynamics::Math::VectorNd::Zero (model->dof_count);
		QDDot = RigidBodyDynamics::Math::VectorNd::Zero (model->dof_count);
		Tau = RigidBodyDynamics::Math::VectorNd::Zero (model->dof_count);
	}

	RigidBodyDynamics::Model *model;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;
};

struct FixedJoint2DoF {
	FixedJoint2DoF () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 *      Z
		 *      *---* 
		 *      |
		 *      |
		 *  Z   |
		 *  O---*
		 *      Y
		 */

		body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_a = Joint( SpatialVector (0., 0., 1., 0., 0., 0.));

		body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

		body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
		joint_b = Joint (JointTypeFixed);

		body_b_id = model->AddBody(1, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

		body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
		joint_c = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

		body_c_id = model->AddBody(2, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		QDDot = VectorNd::Constant ((size_t) model->dof_count, 0.);

		point_position = Vector3d::Zero (3);
		point_acceleration = Vector3d::Zero (3);

		ref_body_id = 0;

		ClearLogOutput();
	}
	~FixedJoint2DoF () {
		delete model;
	}
	
	RigidBodyDynamics::Model *model;

	unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
	RigidBodyDynamics::Body body_a, body_b, body_c;
	RigidBodyDynamics::Joint joint_a, joint_b, joint_c;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;

	RigidBodyDynamics::Math::Vector3d point_position, point_acceleration;
};

/** \brief Fixture that contains two models of which one has one joint fixed.
 */
struct FixedAndMovableJoint {
	FixedAndMovableJoint () {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model_movable = new Model;

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 *      Z
		 *      *---* 
		 *      |
		 *      |
		 *  Z   |
		 *  O---*
		 *      Y
		 */

		body_a = Body (1., Vector3d (1., 0., 0.), Vector3d (1., 1., 1.));
		joint_a = Joint( SpatialVector (0., 0., 1., 0., 0., 0.));

		body_a_id = model_movable->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);

		body_b = Body (1., Vector3d (0., 1., 0.), Vector3d (1., 1., 1.));
		joint_b = Joint ( SpatialVector (0., 1., 0., 0., 0., 0.));

		body_b_id = model_movable->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);

		body_c = Body (1., Vector3d (0., 0., 1.), Vector3d (1., 1., 1.));
		joint_c = Joint ( SpatialVector (0., 0., 1., 0., 0., 0.));

		body_c_id = model_movable->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q = VectorNd::Constant ((size_t) model_movable->dof_count, 0.);
		QDot = VectorNd::Constant ((size_t) model_movable->dof_count, 0.);
		QDDot = VectorNd::Constant ((size_t) model_movable->dof_count, 0.);
		Tau = VectorNd::Constant ((size_t) model_movable->dof_count, 0.);
		C_movable = VectorNd::Zero ((size_t) model_movable->dof_count);
		H_movable = MatrixNd::Zero ((size_t) model_movable->dof_count, (size_t) model_movable->dof_count);

		// Assemble the fixed joint model
		model_fixed = new Model;

		body_a_fixed_id = model_fixed->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);
		Joint joint_fixed (JointTypeFixed);
		body_b_fixed_id = model_fixed->AddBody(body_a_fixed_id, Xtrans(Vector3d(1., 0., 0.)), joint_fixed, body_b);
		body_c_fixed_id = model_fixed->AddBody(body_b_fixed_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);

		Q_fixed = VectorNd::Constant ((size_t) model_fixed->dof_count, 0.);
		QDot_fixed = VectorNd::Constant ((size_t) model_fixed->dof_count, 0.);
		QDDot_fixed = VectorNd::Constant ((size_t) model_fixed->dof_count, 0.);
		Tau_fixed = VectorNd::Constant ((size_t) model_fixed->dof_count, 0.);
		C_fixed = VectorNd::Zero ((size_t) model_fixed->dof_count);
		H_fixed = MatrixNd::Zero ((size_t) model_fixed->dof_count, (size_t) model_fixed->dof_count);

		point_position = Vector3d::Zero (3);
		point_acceleration = Vector3d::Zero (3);

		ref_body_id = 0;

		ClearLogOutput();
	}

	~FixedAndMovableJoint () {
		delete model_movable;
		delete model_fixed;
	}
	RigidBodyDynamics::Math::VectorNd CreateDofVectorFromReducedVector (const RigidBodyDynamics::Math::VectorNd &q_fixed) {
		assert (q_fixed.size() == model_fixed->dof_count);

		RigidBodyDynamics::Math::VectorNd q_movable (model_movable->dof_count);

		q_movable[0] = q_fixed[0];
		q_movable[1] = 0.;
		q_movable[2] = q_fixed[1];

		return q_movable;
	}

	RigidBodyDynamics::Math::MatrixNd CreateReducedInertiaMatrix(const RigidBodyDynamics::Math::MatrixNd &H_movable) {
		assert (H_movable.rows() == model_movable->dof_count);
		assert (H_movable.cols() == model_movable->dof_count);
		RigidBodyDynamics::Math::MatrixNd H (model_fixed->dof_count, model_fixed->dof_count);

		H (0,0) = H_movable(0,0); H (0,1) = H_movable(0,2);
		H (1,0) = H_movable(2,0); H (1,1) = H_movable(2,2);

		return H;
	}
	
	RigidBodyDynamics::Model *model_fixed;
	RigidBodyDynamics::Model *model_movable;

	unsigned int body_a_id, body_b_id, body_c_id, ref_body_id;
	unsigned int body_a_fixed_id, body_b_fixed_id, body_c_fixed_id;

	RigidBodyDynamics::Body body_a, body_b, body_c;
	RigidBodyDynamics::Joint joint_a, joint_b, joint_c;

	RigidBodyDynamics::Math::VectorNd Q;
	RigidBodyDynamics::Math::VectorNd QDot;
	RigidBodyDynamics::Math::VectorNd QDDot;
	RigidBodyDynamics::Math::VectorNd Tau;
	RigidBodyDynamics::Math::VectorNd C_movable;
	RigidBodyDynamics::Math::MatrixNd H_movable;
	
	RigidBodyDynamics::Math::VectorNd Q_fixed;
	RigidBodyDynamics::Math::VectorNd QDot_fixed;
	RigidBodyDynamics::Math::VectorNd QDDot_fixed;
	RigidBodyDynamics::Math::VectorNd Tau_fixed;
	RigidBodyDynamics::Math::VectorNd C_fixed;
	RigidBodyDynamics::Math::MatrixNd H_fixed;

	RigidBodyDynamics::Math::Vector3d point_position, point_acceleration;
};

/** Model with two moving bodies and one fixed body
 */
struct RotZRotZYXFixed {
	RotZRotZYXFixed() {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;

		Joint joint_rot_z ( SpatialVector (0., 0., 1., 0., 0., 0.));

		Joint joint_rot_zyx (
				SpatialVector (0., 0., 1., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.)
				);

		Body body_a(1., RigidBodyDynamics::Math::Vector3d (1., 0.4, 0.4), RigidBodyDynamics::Math::Vector3d (1., 1., 1.));
		Body body_b(2., RigidBodyDynamics::Math::Vector3d (1., 0.4, 0.4), RigidBodyDynamics::Math::Vector3d (1., 1., 1.));
		Body body_fixed(10., RigidBodyDynamics::Math::Vector3d (1., 0.4, 0.4), RigidBodyDynamics::Math::Vector3d (1., 1., 1.));

		fixture_transform_a = Xtrans (RigidBodyDynamics::Math::Vector3d(1., 2., 3.));
		fixture_transform_b = Xtrans (RigidBodyDynamics::Math::Vector3d(4., 5., 6.));
		fixture_transform_fixed = Xtrans (RigidBodyDynamics::Math::Vector3d(-1., -2., -3.));

		body_a_id = model->AddBody (0, fixture_transform_a, joint_rot_z, body_a);
		body_b_id = model->AppendBody (fixture_transform_b, joint_rot_zyx, body_b);
		body_fixed_id = model->AppendBody (fixture_transform_fixed, Joint(JointTypeFixed), body_fixed);

		ClearLogOutput();
	}
	~RotZRotZYXFixed() {
		delete model;
	}
	
	RigidBodyDynamics::Model *model;

	unsigned int body_a_id, body_b_id, body_fixed_id;

	RigidBodyDynamics::Math::SpatialTransform fixture_transform_a;
	RigidBodyDynamics::Math::SpatialTransform fixture_transform_b;
	RigidBodyDynamics::Math::SpatialTransform fixture_transform_fixed;
};

struct TwoArms12DoF {
	TwoArms12DoF() {
		using namespace RigidBodyDynamics;
		using namespace RigidBodyDynamics::Math;

		ClearLogOutput();
		model = new Model;

		/* Basically a model like this, where X are the Center of Masses
		 * and the CoM of the last (3rd) body comes out of the Y=X=0 plane.
		 *
		 * *----O----*
		 * |         |
		 * |         |
		 * *         *
		 * |         |
		 * |         |
		 *
		 */

		Body body_upper = Body (1., Vector3d (0., -0.2, 0.), Vector3d (1.1, 1.3, 1.5));
		Body body_lower = Body (0.5, Vector3d(0., -0.15, 0.), Vector3d (0.3, 0.5, 0.2));

		Joint joint_zyx = Joint (
				SpatialVector (0., 0., 1., 0., 0., 0.),
				SpatialVector (0., 1., 0., 0., 0., 0.),
				SpatialVector (1., 0., 0., 0., 0., 0.)
				);

		right_upper_arm = model->AppendBody (Xtrans (Vector3d (0., 0., -0.3)), joint_zyx, body_upper, "RightUpper");
//		model->AppendBody (Xtrans (Vector3d (0., -0.4, 0.)), joint_zyx, body_lower, "RightLower");
		left_upper_arm = model->AddBody (0, Xtrans (Vector3d (0., 0., 0.3)), joint_zyx, body_upper, "LeftUpper");
//		model->AppendBody (Xtrans (Vector3d (0., -0.4, 0.)), joint_zyx, body_lower, "LeftLower");
				
		q = VectorNd::Constant ((size_t) model->dof_count, 0.);
		qdot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		qddot = VectorNd::Constant ((size_t) model->dof_count, 0.);
		tau = VectorNd::Constant ((size_t) model->dof_count, 0.);

		ClearLogOutput();
	}
	~TwoArms12DoF() {
		delete model;
	}
	
	RigidBodyDynamics::Model *model;

	RigidBodyDynamics::Math::VectorNd q;
	RigidBodyDynamics::Math::VectorNd qdot;
	RigidBodyDynamics::Math::VectorNd qddot;
	RigidBodyDynamics::Math::VectorNd tau;

	unsigned int right_upper_arm, left_upper_arm;

};


