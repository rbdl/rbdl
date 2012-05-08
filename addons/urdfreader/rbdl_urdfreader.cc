#include "rbdl.h"
#include "rbdl_urdfreader.h"

#include <assert.h>
#include <iostream>
#include <map>
#include <stack>

#include "urdf/model.h"

using namespace std;

namespace RigidBodyDynamics {

namespace Addons {

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr > URDFLinkMap;
typedef map<string, JointPtr > URDFJointMap;

bool construct_model (Model* rbdl_model, urdf::Model *urdf_model) {
	boost::shared_ptr<urdf::Link> urdf_root_link;

	URDFLinkMap link_map;
	link_map = urdf_model->links_;

	URDFJointMap joint_map;
	joint_map = urdf_model->joints_;

	stack<LinkPtr > link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	link_stack.push (link_map[(urdf_model->getRoot()->name)]);

	if (link_stack.top()->child_joints.size() > 0) {
		joint_index_stack.push(0);
	}

	while (link_stack.size() > 0) {
		LinkPtr cur_link = link_stack.top();
		int joint_idx = joint_index_stack.top();

		if (joint_idx < cur_link->child_joints.size()) {
			JointPtr cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push (joint_idx + 1);

			link_stack.push (link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			for (int i = 1; i < joint_index_stack.size() - 1; i++) {
				cout << "  ";
			}
			cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << " type = " << cur_joint->type << endl;
		} else {
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	/*

	unsigned int j;
	for (j = 0; j < joint_names.size(); j++) {
		URDFJoint urdf_joint = joints[joint_names[j]];
		URDFLink urdf_parent = links[urdf_joint.parent];
		URDFLink urdf_child = links[urdf_joint.child];

		// determine where to add the current joint and child body
		unsigned int rbdl_parent_id = 0;
		
		if (urdf_parent.name != "base_joint" && model->mBodies.size() != 1)
			rbdl_parent_id = model->GetBodyId (urdf_parent.name.c_str());

//		cout << "joint: " << urdf_joint.name << "\tparent = " << urdf_parent.name << " child = " << urdf_child.name << " parent_id = " << rbdl_parent_id << endl;

		// create the joint
		Joint rbdl_joint;
		if (urdf_joint.type == URDFJointTypeRevolute || urdf_joint.type == URDFJointTypeContinuous) {
			rbdl_joint = Joint (SpatialVector (urdf_joint.axis[0], urdf_joint.axis[1], urdf_joint.axis[2], 0., 0., 0.));
		} else if (urdf_joint.type == URDFJointTypePrismatic) {
			rbdl_joint = Joint (SpatialVector (0., 0., 0., urdf_joint.axis[0], urdf_joint.axis[1], urdf_joint.axis[2]));
		} else if (urdf_joint.type == URDFJointTypeFixed) {
			// todo: add fixed joint support to rbdl
			cerr << "Error while processing joint '" << urdf_joint.name << "': fixed joints not yet supported!" << endl;
			return false;
		} else if (urdf_joint.type == URDFJointTypeFloating) {
			// todo: what order of DoF should be used?
			rbdl_joint = Joint (
					SpatialVector (0., 0., 0., 1., 0., 0.),
					SpatialVector (0., 0., 0., 0., 1., 0.),
					SpatialVector (0., 0., 0., 0., 0., 1.),
					SpatialVector (1., 0., 0., 0., 0., 0.),
					SpatialVector (0., 1., 0., 0., 0., 0.),
					SpatialVector (0., 0., 1., 0., 0., 0.));
		} else if (urdf_joint.type == URDFJointTypePlanar) {
			// todo: which two directions should be used that are perpendicular
			// to the specified axis?
			cerr << "Error while processing joint '" << urdf_joint.name << "': planar joints not yet supported!" << endl;
			return false;
		}

		// compute the joint transformation
		SpatialTransform rbdl_joint_frame = Xrot (urdf_joint.origin_rot_r, Vector3d (1., 0., 0.))
				* Xrot (urdf_joint.origin_rot_p, Vector3d (0., 1., 0.))
				* Xrot (urdf_joint.origin_rot_y, Vector3d (0., 0., 1.))
				* Xtrans (Vector3d (
							urdf_joint.origin_x,
							urdf_joint.origin_y,
							urdf_joint.origin_z
							));

		// assemble the body
		Vector3d body_com (urdf_child.origin_x, urdf_child.origin_y, urdf_child.origin_z);
		Vector3d body_rpy (urdf_child.origin_rot_r, urdf_child.origin_rot_p, urdf_child.origin_rot_y);

		if (body_rpy != Vector3d (0., 0., 0.)) {
			cerr << "Error while processing joint '" << urdf_joint.name << "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
			return false;
		}

		Matrix3d inertia (
			urdf_child.ixx, urdf_child.ixy, urdf_child.ixz,
			urdf_child.ixy, urdf_child.iyy, urdf_child.iyz,
			urdf_child.ixz, urdf_child.iyz, urdf_child.izz
			);
		Body rbdl_body = Body (urdf_child.mass, body_com, inertia);

		model->AddBody (rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_child.name);
	}
	*/

	return false;
}

bool read_urdf_model (const char* filename, Model* model, bool verbose) {
	assert (model);

	model->Init();

	urdf::Model urdf_model;

	bool urdf_result = urdf_model.initFile (filename);
	if (!urdf_result) {
		cerr << "Error opening urdf file" << endl;
	}

	if (!construct_model (model, &urdf_model)) {
		cerr << "Error constructing model from urdf file." << endl;
		return false;
	}

	return true;
}

}

}
