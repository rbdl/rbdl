#include "rbdl.h"
#include "rbdl_urdfreader.h"

#include <assert.h>
#include <iostream>
#include <map>
#include "tinyxml/tinyxml.h"

using namespace std;

struct URDFLink {
	URDFLink () :
		name (""),
		origin_x (0.), origin_y (0.), origin_z (0.),
		origin_rot_r (0.), origin_rot_p (0.), origin_rot_y (0.),
		mass (0.),
		ixx(0.), ixy(0.), ixz(0.), iyy(0.), iyz(0.), izz(0.)
	{}

	string name;
	double origin_x, origin_y, origin_z;
	double origin_rot_r, origin_rot_p, origin_rot_y;
	double mass;
	double ixx, ixy, ixz, iyy, iyz, izz;

	void debug_print () {
		cout << "link '" << name << "'" << endl;
		cout << "  origin xyz = " << origin_x << " " << origin_y << " " << origin_z << endl;
		cout << "  origin_rot rpy = " << origin_rot_r << " " << origin_rot_p << " " << origin_rot_y << endl;
		cout << "  mass = " << mass << endl;
		cout << "  ixx = " << ixx << " ixy = " <<  ixy << " ixz = " << ixz << " iyy = " << iyy << " iyz = " << iyz << " izz = " << izz << endl;
	}
};

enum URDFJointType {
	URDFJointTypeUnknown = 0,
	URDFJointTypeRevolute,
	URDFJointTypeContinuous,
	URDFJointTypePrismatic,
	URDFJointTypeFixed,
	URDFJointTypeFloating,
	URDFJointTypePlanar,
	URDFJointTypeLast
};

struct URDFJoint {
	URDFJoint() :
		name (""),
		type (URDFJointTypeUnknown),
		origin_x (0.), origin_y (0.), origin_z (0.),
		origin_rot_r (0.), origin_rot_p (0.), origin_rot_y (0.),
		parent(""),
		child ("")
	{
		axis[0] = 0.;
		axis[1] = 0.;
		axis[2] = 0.;
	}

	string name;
	URDFJointType type;

	string parent;
	string child;

	double origin_x, origin_y, origin_z;
	double origin_rot_r, origin_rot_p, origin_rot_y;

	double axis[3];

	void debug_print () {
		cout << "joint '" << name << "'" << endl;
		cout << "  type = " << type << endl;
		cout << "  parent = " << parent << endl;
		cout << "  child = " << child << endl;
		cout << "  origin xyz = " << origin_x << " " << origin_y << " " << origin_z << endl;
		cout << "  origin_rot rpy = " << origin_rot_r << " " << origin_rot_p << " " << origin_rot_y << endl;
		cout << "  axis = " << axis[0] << " " << axis[1] << " " << axis[2] << endl;
	}};

typedef std::map<std::string, URDFLink> LinkMap;
typedef std::map<std::string, URDFJoint> JointMap;
typedef std::vector<std::string> JointNamesVector;

namespace RigidBodyDynamics {

class Model;

using namespace Math;

namespace Addons {

bool parse_double_triple (string str, double out[3]) {
	istringstream value_stream (str);

	if ((value_stream >> out[0]).bad())
		return false;

	if ((value_stream >> out[1]).bad())
		return false;

	if ((value_stream >> out[2]).bad())
		return false;

	return true;
}

bool read_single_link (TiXmlHandle handle, URDFLink &link) {
	string link_name;
	handle.ToElement()->QueryStringAttribute("name", &link_name);
	link.name = link_name;

	// read origin xyz and rpy
	TiXmlElement *origin_element = handle.FirstChild("inertial").FirstChild("origin").ToElement();
	if (origin_element) {
		string value_str;
		origin_element->QueryStringAttribute("xyz", &value_str);

		if (value_str.size() != 0) {
			double values[3];
			if (!parse_double_triple (value_str, values)) {
				cerr << "Unable to parse origin xyz: '" << value_str << "'" << endl;
				return false;
			}
			link.origin_x = values[0];
			link.origin_y = values[1];
			link.origin_z = values[2];
		}
		
		origin_element->QueryStringAttribute("rpy", &value_str);

		if (value_str.size() != 0) {
			double values[3];
			if (!parse_double_triple (value_str, values)) {
				cerr << "Unable to parse origin rpy: '" << value_str << "'" << endl;
				return false;
			}
			link.origin_rot_r = values[0];
			link.origin_rot_p = values[1];
			link.origin_rot_y = values[2];
		}
	}

	// read mass
	TiXmlElement *mass_element = handle.FirstChild("inertial").FirstChild("mass").ToElement();
	if (mass_element) {
		if (mass_element->QueryDoubleAttribute ("value", &(link.mass)) != TIXML_SUCCESS)
			link.mass = 0.;
	}

	// and inertia
	TiXmlElement *inertia_element = handle.FirstChild("inertial").FirstChild("inertia").ToElement();
	if (inertia_element) {
		if (inertia_element->QueryDoubleAttribute ("ixx", &(link.ixx)) != TIXML_SUCCESS) {
			link.ixx = 0.;
		}
		if (inertia_element->QueryDoubleAttribute ("ixy", &(link.ixy)) != TIXML_SUCCESS) {
			link.ixy = 0.;
		}
		if (inertia_element->QueryDoubleAttribute ("ixz", &(link.ixz)) != TIXML_SUCCESS) {
			link.ixz = 0.;
		}
		if (inertia_element->QueryDoubleAttribute ("iyy", &(link.iyy)) != TIXML_SUCCESS) {
			link.iyy = 0.;
		}
		if (inertia_element->QueryDoubleAttribute ("iyz", &(link.iyz)) != TIXML_SUCCESS) {
			link.iyz = 0.;
		}
		if (inertia_element->QueryDoubleAttribute ("izz", &(link.izz)) != TIXML_SUCCESS) {
			link.izz = 0.;
		}
	}

	return true;
}

bool read_links (TiXmlDocument &doc, LinkMap &links) {
	TiXmlHandle doc_handle (&doc);
	TiXmlHandle link_handle = doc_handle.FirstChild("robot").FirstChild("link");

	TiXmlElement *child = link_handle.ToElement();

	while (child) {
		if (child->ValueStr() != "link") {
			child = child->NextSiblingElement();
			continue;
		}

		URDFLink link;
		string link_name;
		child->QueryStringAttribute("name", &link_name);

		if (!read_single_link(child, link)) {
			return false;
		}

		links[link_name] = link;

		child = child->NextSiblingElement();
	}

	return true;
}

bool read_single_joint (TiXmlHandle handle, URDFJoint &joint) {
	string joint_name;
	handle.ToElement()->QueryStringAttribute("name", &joint_name);
	joint.name = joint_name;

	// type
	string joint_type_string;
	handle.ToElement()->QueryStringAttribute("type", &joint_type_string);
	if (joint_type_string == "revolute")
		joint.type = URDFJointTypeRevolute;
	else if (joint_type_string == "continuous")
		joint.type = URDFJointTypeContinuous;
	else if (joint_type_string == "prismatic")
		joint.type = URDFJointTypePrismatic;
	else if (joint_type_string == "fixed")
		joint.type = URDFJointTypeFixed;
	else if (joint_type_string == "Floating")
		joint.type = URDFJointTypeFloating;
	else if (joint_type_string == "Planar")
		joint.type = URDFJointTypePlanar;

	// read origin xyz and rpy
	TiXmlElement *origin_element = handle.FirstChild("origin").ToElement();
	if (origin_element) {
		string value_str;
		origin_element->QueryStringAttribute("xyz", &value_str);

		if (value_str.size() != 0) {
			double values[3];
			if (!parse_double_triple (value_str, values)) {
				cerr << "Unable to parse origin xyz: '" << value_str << "'" << endl;
				return false;
			}
			joint.origin_x = values[0];
			joint.origin_y = values[1];
			joint.origin_z = values[2];
		}
		
		origin_element->QueryStringAttribute("rpy", &value_str);

		if (value_str.size() != 0) {
			double values[3];
			if (!parse_double_triple (value_str, values)) {
				cerr << "Unable to parse origin rpy: '" << value_str << "'" << endl;
				return false;
			}
			joint.origin_rot_r = values[0];
			joint.origin_rot_p = values[1];
			joint.origin_rot_y = values[2];
		}
	}

	// read parent
	TiXmlElement *parent_element = handle.FirstChild("parent").ToElement();
	if (!parent_element) {
		cerr << "Could not find required parent element for joint '" << joint_name << endl;
		return false;
	}

	if (parent_element->QueryStringAttribute ("link", &(joint.parent)) != TIXML_SUCCESS) {
		cerr << "Could not find required parent element for joint '" << joint_name << endl;
		return false;
	}

	// read child
	TiXmlElement *child_element = handle.FirstChild("child").ToElement();
	if (!child_element) {
		cerr << "Could not find required child element for joint '" << joint_name << endl;
		return false;
	}

	if (child_element->QueryStringAttribute ("link", &(joint.child)) != TIXML_SUCCESS) {
		cerr << "Could not find required child element for joint '" << joint_name << endl;
		return false;
	}

	// axis
	joint.axis[0] = 1.;
	joint.axis[1] = 0.;
	joint.axis[2] = 0.;

	TiXmlElement *axis_element = handle.FirstChild("axis").ToElement();
	if (axis_element) {
		string value_str;
		axis_element->QueryStringAttribute("xyz", &value_str);

		if (value_str.size() != 0) {
			double values[3];
			if (!parse_double_triple (value_str, values)) {
				cerr << "Unable to parse origin xyz: '" << value_str << "'" << endl;
				return false;
			}
			joint.axis[0] = values[0];
			joint.axis[1] = values[1];
			joint.axis[2] = values[2];
		}
	}

//	joint.debug_print();

	return true;
}

bool read_joints (TiXmlDocument &doc, JointMap &joints, JointNamesVector &joint_names) {
	TiXmlHandle doc_handle (&doc);
	TiXmlHandle joint_handle = doc_handle.FirstChild("robot").FirstChild("joint");

	TiXmlElement *child = joint_handle.ToElement();

	while (child) {
		if (child->ValueStr() != "joint") {
			child = child->NextSiblingElement();
			continue;
		}

		URDFJoint joint;
		string joint_name;
		child->QueryStringAttribute("name", &joint_name);

		if (!read_single_joint(child, joint)) {
			return false;
		}

		joints[joint_name] = joint;
		joint_names.push_back(joint_name);

		child = child->NextSiblingElement();
	}

	return true;
}

bool construct_model (Model* model, LinkMap &links, JointMap &joints, JointNamesVector &joint_names) {
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

	return false;
}

bool read_urdf_model (const char* filename, Model* model, bool verbose) {
	assert (model);

	TiXmlDocument doc (filename);
	bool load_success = doc.LoadFile();

	if (!load_success) {
		return false;
	}

	LinkMap links;
	JointMap joints;
	JointNamesVector joint_names; // keeps track of the ordering of the joints

	if (!read_links(doc, links)) {
		return false;
	}

	if (verbose) {
		cout << "Read " << links.size() << " links." << endl;
	}

	if (!read_joints(doc, joints, joint_names)) {
		return false;
	}

	if (verbose) {
		cout << "Read " << joints.size() << " joints." << endl;
	}

	model->Init();
	construct_model (model, links, joints, joint_names);

	cout << "Created model with " << model->dof_count << " DoF" << endl;

	return false;
}

}

}
