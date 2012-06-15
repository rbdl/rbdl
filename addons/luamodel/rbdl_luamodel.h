#ifndef _RBDL_LUAMODEL_H
#define _RBDL_LUAMODEL_H

namespace RigidBodyDynamics {

class Model;

namespace Addons {
	bool read_luamodel (const char* filename, Model* model, bool verbose = false);
}

}

/* _RBDL_LUAMODEL_H */
#endif
