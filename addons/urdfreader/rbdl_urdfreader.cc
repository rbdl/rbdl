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

namespace RigidBodyDynamics {

class Model;

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

bool read_joints (TiXmlDocument &doc, JointMap &joints) {
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

		child = child->NextSiblingElement();
	}

	return true;
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

	if (!read_links(doc, links)) {
		return false;
	}

	if (verbose) {
		cout << "Read " << links.size() << " links." << endl;
	}

	if (!read_joints(doc, joints)) {
		return false;
	}

	if (verbose) {
		cout << "Read " << joints.size() << " joints." << endl;
	}


	return false;
}

}

}
