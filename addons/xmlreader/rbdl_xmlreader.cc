#include "rbdl.h"
#include "rbdl_xmlreader.h"

#include <assert.h>
#include <iostream>
#include "tinyxml/tinyxml.h"

using namespace std;

namespace RigidBodyDynamics {

class Model;

namespace Addons {

bool read_xml_model (const char* filename, Model* model) {
	assert (model);

	std::vector<Body> xml_boides;
	std::vector<Joint> xml_joints;

	return false;
}

}

}
