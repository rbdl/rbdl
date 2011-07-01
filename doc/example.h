/** \file example.h 
 * \page SimpleExample A simple example
 *
 * Here is a simple example how one can create a meaningless model and
 * compute the forward dynamics for it:
 * 
 * \code
 *	#include <iostream>
 *
 *	#include <rbdl.h>
 *
 *	using namespace RigidBodyDynamics;
 *
 *	int main (int argc, char* argv[]) {
 *	Model* model = NULL;
 *
 *	unsigned int body_a_id, body_b_id, body_c_id;
 *	Body body_a, body_b, body_c;
 *	Joint joint_a, joint_b, joint_c;
 *
 *	model = new Model();
 *	model->Init();
 *
 *	model->gravity = Vector3d (0., -9.81, 0.);
 *
 *	body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
 *		joint_a = Joint(
 *		JointTypeRevolute,
 *		Vector3d (0., 0., 1.)
 *	);
 *	
 *	body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);
 *	
 *	body_b = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));
 *		joint_b = Joint (
 *		JointTypeRevolute,
 *		Vector3d (0., 0., 1.)
 *	);
 *	
 *	body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);
 *	
 *	body_c = Body (0., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
 *		joint_c = Joint (
 *		JointTypeRevolute,
 *		Vector3d (0., 0., 1.)
 *	);
 *	
 *	body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);
 *
 *	VectorNd Q = VectorNd::Zero (model->dof_count);
 *	VectorNd QDot = VectorNd::Zero (model->dof_count);
 *	VectorNd Tau = VectorNd::Zero (model->dof_count);
 *	VectorNd QDDot = VectorNd::Zero (model->dof_count);
 *
 * 	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
 *
 *	std::cout << QDDot.transpose() << std::endl;
 *
 *	delete model;
 *
 * 	return 0;
 *}
 * \endcode
 *
 * If the library itself is already created, one can compile this example
 * with:
 * \code
 * 	g++ example.cc -I<path to src folder> -lrbdl -L<path to librbdl.a> -o example
 * \endcode
 *
 * Additionally there is a CMakeLists.txt, that can be used to automatically
 * create the makefiles by using <a href="http://www.cmake.org">CMake</a>.
 * It uses the script FindRBDL.cmake which can be used to find the library
 * and include directory of the headers.
 *
 * The FindRBDL.cmake script can use the environment variables RBDL_PATH,
 * RBDL_INCLUDE_PATH, and RBDL_LIBRARY_PATH to find the required files.
 *
 */
