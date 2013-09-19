#ifndef _RBDL_URDFREADER_H
#define _RBDL_URDFREADER_H

#include <rbdl/rbdl_config.h>

namespace RigidBodyDynamics {

class Model;

namespace Addons {
	RBDL_DLLAPI bool read_urdf_model (const char* filename, Model* model, bool verbose = false);
}

}

/* _RBDL_URDFREADER_H */
#endif
