#ifndef _RBDL_XMLREADER_H
#define _RBDL_XMLREADER_H

namespace RigidBodyDynamics {

class Model;

namespace Addons {
	bool read_xml_model (const char* filename, Model* model);
}

}

/* _RBDL_XMLREADER_H */
#endif
