#include <rbdl/rbdl.h>

#include "rbdl_urdfreader.h"

#include <assert.h>
#include <iostream>
#include <map>
#include <stack>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

using namespace std;

namespace RigidBodyDynamics {

namespace Addons {

using namespace Math;

typedef boost::shared_ptr<urdf::Link> LinkPtr;
typedef boost::shared_ptr<urdf::Joint> JointPtr;
typedef boost::shared_ptr<urdf::ModelInterface> ModelPtr;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr > URDFLinkMap;
typedef map<string, JointPtr > URDFJointMap;

bool construct_model (Model* rbdl_model, ModelPtr urdf_model, bool verbose) {
	boost::shared_ptr<urdf::Link> urdf_root_link;

	URDFLinkMap link_map;
	link_map = urdf_model->links_;

	URDFJointMap joint_map;
	joint_map = urdf_model->joints_;

	vector<string> joint_names;

	stack<LinkPtr > link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	link_stack.push (link_map[(urdf_model->getRoot()->name)]);

	// add the root body
	const boost::shared_ptr<const urdf::Link>& root = urdf_model->getRoot ();
	Vector3d root_inertial_rpy;
	Vector3d root_inertial_position;
	Matrix3d root_inertial_inertia;
	double root_inertial_mass;

	if (root->inertial)
	{
		root_inertial_mass = root->inertial->mass;

		root_inertial_position.set (
			root->inertial->origin.position.x,
			root->inertial->origin.position.y,
			root->inertial->origin.position.z);

		root_inertial_inertia(0,0) = root->inertial->ixx;
		root_inertial_inertia(0,1) = root->inertial->ixy;
		root_inertial_inertia(0,2) = root->inertial->ixz;

		root_inertial_inertia(1,0) = root->inertial->ixy;
		root_inertial_inertia(1,1) = root->inertial->iyy;
		root_inertial_inertia(1,2) = root->inertial->iyz;

		root_inertial_inertia(2,0) = root->inertial->ixz;
		root_inertial_inertia(2,1) = root->inertial->iyz;
		root_inertial_inertia(2,2) = root->inertial->izz;

		root->inertial->origin.rotation.getRPY (root_inertial_rpy[0], root_inertial_rpy[1], root_inertial_rpy[2]);
	}
	Body root_link = Body (root_inertial_mass,
	root_inertial_position,
	root_inertial_inertia);
	Joint root_joint = Joint (
		SpatialVector (0., 0., 0., 1., 0., 0.),
		SpatialVector (0., 0., 0., 0., 1., 0.),
		SpatialVector (0., 0., 0., 0., 0., 1.),
		SpatialVector (1., 0., 0., 0., 0., 0.),
		SpatialVector (0., 1., 0., 0., 0., 0.),
		SpatialVector (0., 0., 1., 0., 0., 0.));

	SpatialTransform root_joint_frame = SpatialTransform ();

	if (verbose) {
		cout << "+ Adding Root Body " << endl;
		cout << "  joint frame: " << root_joint_frame << endl;
		cout << "  joint dofs : " << root_joint.mDoFCount << endl;
		for (unsigned int j = 0; j < root_joint.mDoFCount; j++) {
			cout << "    " << j << ": " << root_joint.mJointAxes[j].transpose() << endl;
		}
		cout << "  body inertia: " << endl << root_link.mSpatialInertia << endl;
		cout << "  body mass   : " << root_link.mMass << endl;
		cout << "  body name   : " << root->name << endl;
	}

	rbdl_model->AppendBody(root_joint_frame,
		root_joint,
		root_link,
		root->name);

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

			if (verbose) {
				for (int i = 1; i < joint_index_stack.size() - 1; i++) {
					cout << "  ";
				}
				cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << "' type = " << cur_joint->type << endl;
			}

			joint_names.push_back(cur_joint->name);
		} else {
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	unsigned int j;
	for (j = 0; j < joint_names.size(); j++) {
		JointPtr urdf_joint = joint_map[joint_names[j]];
		LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
		LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

		// determine where to add the current joint and child body
		unsigned int rbdl_parent_id = 0;

		if (urdf_parent->name != "base_joint" && rbdl_model->mBodies.size() != 1)
			rbdl_parent_id = rbdl_model->GetBodyId (urdf_parent->name.c_str());

		if (rbdl_parent_id == std::numeric_limits<unsigned int>::max())
			cerr << "Error while processing joint '" << urdf_joint->name
				<< "': parent link '" << urdf_parent->name
				<< "' could not be found." << endl;

		//cout << "joint: " << urdf_joint->name << "\tparent = " << urdf_parent->name << " child = " << urdf_child->name << " parent_id = " << rbdl_parent_id << endl;

		// create the joint
		Joint rbdl_joint;
		if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) {
			rbdl_joint = Joint (SpatialVector (urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.));
		} else if (urdf_joint->type == urdf::Joint::PRISMATIC) {
			rbdl_joint = Joint (SpatialVector (0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
		} else if (urdf_joint->type == urdf::Joint::FIXED) {
			rbdl_joint = Joint (JointTypeFixed);
		} else if (urdf_joint->type == urdf::Joint::FLOATING) {
			// todo: what order of DoF should be used?
			rbdl_joint = Joint (
					SpatialVector (0., 0., 0., 1., 0., 0.),
					SpatialVector (0., 0., 0., 0., 1., 0.),
					SpatialVector (0., 0., 0., 0., 0., 1.),
					SpatialVector (1., 0., 0., 0., 0., 0.),
					SpatialVector (0., 1., 0., 0., 0., 0.),
					SpatialVector (0., 0., 1., 0., 0., 0.));
		} else if (urdf_joint->type == urdf::Joint::PLANAR) {
			// todo: which two directions should be used that are perpendicular
			// to the specified axis?
			cerr << "Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!" << endl;
			return false;
		}

		// compute the joint transformation
		Vector3d joint_rpy;
		Vector3d joint_translation;
		urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
		joint_translation.set (
				urdf_joint->parent_to_joint_origin_transform.position.x,
				urdf_joint->parent_to_joint_origin_transform.position.y,
				urdf_joint->parent_to_joint_origin_transform.position.z
				);
		SpatialTransform rbdl_joint_frame =
					Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
				* Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
				* Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
				* Xtrans (Vector3d (
							joint_translation
							));

		// assemble the body
		Vector3d link_inertial_position;
		Vector3d link_inertial_rpy;
		Matrix3d link_inertial_inertia = Matrix3d::Zero();
		double link_inertial_mass;

		// but only if we actually have inertial data
		if (urdf_child->inertial) {
			link_inertial_mass = urdf_child->inertial->mass;

			link_inertial_position.set (
					urdf_child->inertial->origin.position.x,
					urdf_child->inertial->origin.position.y,
					urdf_child->inertial->origin.position.z
					);
			urdf_child->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

			link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
			link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
			link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

			link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
			link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
			link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

			link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
			link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
			link_inertial_inertia(2,2) = urdf_child->inertial->izz;

			if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
				cerr << "Error while processing body '" << urdf_child->name << "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
				return false;
			}
		}

		Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);

		if (verbose) {
			cout << "+ Adding Body " << endl;
			cout << "  parent_id  : " << rbdl_parent_id << endl;
			cout << "  joint frame: " << rbdl_joint_frame << endl;
			cout << "  joint dofs : " << rbdl_joint.mDoFCount << endl;
			for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
				cout << "    " << j << ": " << rbdl_joint.mJointAxes[j].transpose() << endl;
			}
			cout << "  body inertia: " << endl << rbdl_body.mSpatialInertia << endl;
			cout << "  body mass   : " << rbdl_body.mMass << endl;
			cout << "  body name   : " << urdf_child->name << endl;
		}

		rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_child->name);
	}

	return true;
}

RBDL_DLLAPI bool read_urdf_model (const char* filename, Model* model, bool verbose) {
	assert (model);

	cerr << "Warning: this code (RigidBodyDynamics::Addons::" << __func__ << "()) is not properly tested as" << endl;
	cerr << "         I do not have a proper urdf model that I can use to validate the model loading." << endl;
	cerr << "         Please use with care." << endl;

	boost::shared_ptr<urdf::ModelInterface> urdf_model = urdf::parseURDFFile (filename);

	if (!urdf_model) {
		cerr << "Error opening urdf file" << endl;
	}

	if (!construct_model (model, urdf_model, verbose)) {
		cerr << "Error constructing model from urdf file." << endl;
		return false;
	}

	model->gravity.set (0., 0., -9.81);

	return true;
}

}

}
