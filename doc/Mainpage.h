/** \file Mainpage.h 
 * \mainpage Main Page
 *
 * This is the documentation of a yet to be named rigid body simulation
 * code. So far the code supports the simulation of forward dynamics of
 * tree structured (i.e. no kinematic loops) rigid body models by using the
 * Articulated Body Algorithm (ABA) by Roy Featherstone. The code is
 * heavily inspired by the pseudo code of the book "Rigid Body Dynamics
 * Algorithms" of Featherstone.
 *
 * The library uses the configurable math library (which can be found here:
 * <a href="http://www.cmldev.net">http://www.cmldev.net</a>).
 *
 * Documentation of the functions can be found at the documentation page of
 * the namespace RigidBodyDynamics.
 *
 * \section Example An Example
 * Here is a simple example how one can create a model and compute the
 * forward dynamics for it:
 *
 * \code
 *	#include <iostream>
 *	
 *	#include "rbdl.h"
 *
 *	using namespace RigidBodyDynamics;
 *	
 *	int main (int argc, char* argv[]) {
 *		Model* model = NULL;
 *	
 *		unsigned int body_a_id, body_b_id, body_c_id;
 *		Body body_a, body_b, body_c;
 *		Joint joint_a, joint_b, joint_c;
 *	
 *		model = new Model();
 *		model->Init();
 *
 *		model->gravity.set (0., -9.81, 0.);
 *	
 *		body_a = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
 *		joint_a = Joint(
 *			JointTypeRevolute,
 *			Vector3d (0., 0., 1.)
 *		);
 *		
 *		body_a_id = model->AddBody(0, Xtrans(Vector3d(0., 0., 0.)), joint_a, body_a);
 *		
 *		body_b = Body (1., Vector3d (0., 0.5, 0.), Vector3d (1., 1., 1.));
 *		joint_b = Joint (
 *			JointTypeRevolute,
 *			Vector3d (0., 0., 1.)
 *		);
 *		
 *		body_b_id = model->AddBody(body_a_id, Xtrans(Vector3d(1., 0., 0.)), joint_b, body_b);
 *		
 *		body_c = Body (0., Vector3d (0.5, 0., 0.), Vector3d (1., 1., 1.));
 *		joint_c = Joint (
 *			JointTypeRevolute,
 *			Vector3d (0., 0., 1.)
 *		);
 *		
 *		body_c_id = model->AddBody(body_b_id, Xtrans(Vector3d(0., 1., 0.)), joint_c, body_c);
 *	
 *		cmlVector Q(3);
 *		cmlVector QDot(3);
 *		cmlVector QDDot(3);
 *		cmlVector Tau(3);
 *	
 *	 	ForwardDynamics (*model, Q, QDot, Tau, QDDot);
 *	
 *		std::cout << QDDot << std::endl;
 *	
 *	 	return 0;
 *	}
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
 * \section ModelConstruction Construction of Models
 *
 * The construction of models makes use of carefully designed constructors
 * of the classes Body and Joint to ease the process of creating bodies.
 * Adding bodies to the model is done by specifying the parent body by its
 * id, the transformation from the parent origin to the joint origin, the
 * joint specification as an object, and the body itself. These parameters
 * are then fed to the function Model::AddBody().
 */
