#ifndef _RBDL_URDFREADER_H
#define _RBDL_URDFREADER_H

namespace RigidBodyDynamics {

class Model;

namespace Addons {
	bool read_urdf_model (const char* filename, Model* model, bool verbose = false);
}

}

/* _RBDL_URDFREADER_H */
#endif
