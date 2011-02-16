#ifndef _MODELSTATE_H
#define _MODELSTATE_H

// Forward declaration for the model
namespace RigidBodyDynamics {
	class Model;
}

void model_init ();
void model_update (double delta_time);
RigidBodyDynamics::Model* model_get();
void model_destroy ();

#endif /* _MODELSTATE_H */
