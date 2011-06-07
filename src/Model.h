#ifndef _MODEL_H
#define _MODEL_H

#include <mathwrapper.h>
#include <map>
#include <list>
#include <assert.h>
#include <iostream>
#include "Logging.h"

#include "Joint.h"
#include "Body.h"
#include "Visualization.h"

// std::vectors containing any objectst that have Eigen matrices or vectors
// as members need to have a special allocater. This can be achieved with
// the following macro.

#ifdef EIGEN_CORE_H
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Joint);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(RigidBodyDynamics::Body);
#endif

/** \brief Namespace for all structures of the RigidBodyDynamics library
 */
namespace RigidBodyDynamics {

namespace Visualization {
	class Primitive;
	class PrimitiveBox;
	class PrimitiveSphere;
}

/** \brief Contains all information about the rigid body model
 *
 * This class contains all information required to perform the forward
 * dynamics calculation. The variables in this class are also used for
 * storage of temporary values. It is designed for use of the Articulated
 * Rigid Body Algorithm (which is implemented in ForwardDynamics()) and
 * follows the numbering as described in Featherstones book.
 *
 * An important note is that body 0 is the root body and the moving bodies
 * start at index 1. Additionally the vectors for the states q, qdot, etc.
 * have \#Model::dof_count + 1 entries where always the first entry (e.g. q[0])
 * contains the value for the base (or "root" body. Thus the numbering might be
 * confusing as q[1] holds the position variable of the first degree of
 * freedom. This numbering scheme is very beneficial in terms of readability
 * of the code as the resulting code is very similar to the pseudo-code in
 * the RBDA book.
 */
struct Model {
	// Structural information

	/// \brief The id of the parents body
	std::vector<unsigned int> lambda;
	/// \brief Contains the ids of all the children of a given body
	std::vector<std::vector<unsigned int> >mu;

	/** \brief Use floating base extension as described in RBDA chapter 9.4
	 *
	 * \warning This function is experimental and produces wrong results. Do
	 * \warning not use!
	 */
	bool experimental_floating_base;

	/** \brief number of degrees of freedoms of the model
	 *
	 * This value contains the number of entries in the generalized state (q)
	 * velocity (qdot), acceleration (qddot), and force (tau) vector.
	 */
	unsigned int dof_count;

	/// \brief the cartesian vector of the gravity
	Vector3d gravity;

	// State information

	/** \brief The joint position
	 * 
	 * Warning: to have an easier numbering in the algorithm the state vector
	 * has NDOF + 1 elements. However element with index 0 is not used!
	 * 
	 * q[0] - unused <br>
	 * q[1] - joint 1 <br>
	 * q[2] - joint 2 <br>
	 * ... <br>
	 * q[NDOF] - joint NDOF <br>
	 *
	 */
	VectorNd q;
	/// \brief The joint velocity
	VectorNd qdot;
	/// \brief The joint acceleration
	VectorNd qddot;
	/// \brief The force / torque applied at joint i
	VectorNd tau;
	/// \brief The spatial velocity of body i
	std::vector<SpatialAlgebra::SpatialVector> v;
	/// \brief The spatial acceleration of body i
	std::vector<SpatialAlgebra::SpatialVector> a;

	////////////////////////////////////
	// Joints

	/// \brief All joints
	
	std::vector<Joint> mJoints;
	/// \brief The joint axis for joint i
	std::vector<SpatialAlgebra::SpatialVector> S;
	/// \brief Transformations from the parent body to the frame of the joint
	std::vector<SpatialAlgebra::SpatialMatrix> X_T;

	////////////////////////////////////
	// Dynamics variables

	/// \brief The velocity dependent spatial acceleration
	std::vector<SpatialAlgebra::SpatialVector> c;
	/// \brief The spatial inertia of body i
	std::vector<SpatialAlgebra::SpatialMatrix> IA;
	/// \brief The spatial bias force
	std::vector<SpatialAlgebra::SpatialVector> pA;
	/// \brief Temporary variable U_i (RBDA p. 130)
	std::vector<SpatialAlgebra::SpatialVector> U;
	/// \brief Temporary variable D_i (RBDA p. 130)
	VectorNd d;
	/// \brief Temporary variable u (RBDA p. 130)
	VectorNd u;
	/// \brief Internal forces on the body (used only InverseDynamics())
	std::vector<SpatialAlgebra::SpatialVector> f;
	/// \brief External forces acting on the body (in base coordinates)
	std::vector<SpatialAlgebra::SpatialVector> f_ext;
	/// \brief The spatial inertia of body i (used only in CompositeRigidBodyAlgorithm())
	std::vector<SpatialAlgebra::SpatialMatrix> Ic;

	////////////////////////////////////
	// Bodies

	/// \brief Transformation from the parent body to the current body
	std::vector<SpatialAlgebra::SpatialMatrix> X_lambda;
	/// \brief Transformation from the base to bodies reference frame
	std::vector<SpatialAlgebra::SpatialMatrix> X_base;

	/** \brief All bodies 0 ... N_B, including the base
	 *
	 * mBodies[0] - base body <br>
	 * mBodies[1] - 1st moveable body <br>
	 * ... <br>
	 * mBodies[N_B] - N_Bth moveable body <br>
	 */
	std::vector<Body> mBodies;

	/** \brief Connects a given body to the model
	 *
	 * When adding a body there are basically informations required:
	 * - what kind of body will be added?
	 * - where is the new body to be added?
	 * - by what kind of joint should the body be added?
	 *
	 * The first information "what kind of body will be added" is contained
	 * in the Body class that is given as a parameter.
	 *
	 * The question "where is the new body to be added?" is split up in two
	 * parts: first the parent (or successor) body to which it is added and
	 * second the transformation to the origin of the joint that connects the
	 * two bodies. With these two informations one specifies the relative
	 * positions of the bodies when the joint is in neutral position.gk
	 *
	 * The last question "by what kind of joint should the body be added?" is
	 * again simply contained in the Joint class.
	 *
	 * \param parent_id   id of the parent body
	 * \param joint_frame the transformation from the parent frame to the origin
	 *                    of the joint frame (represents X_T in RBDA)
	 * \param joint       specification for the joint that describes the connection
	 * \param body        specification of the body itself
	 *
	 * \returns id of the added body
	 */
	unsigned int AddBody (
			const unsigned int parent_id,
			const SpatialAlgebra::SpatialMatrix &joint_frame,
			const Joint &joint,
			const Body &body
			);

	/** \brief Specifies the dynamical parameters of the first body and
	 *  \brief assigns it a 6 DoF joint.
	 *
	 * The 6 DoF joint is simulated by adding 5 massless bodies at the base
	 * which are connected with joints. The body that is specified as a
	 * parameter of this function is then added by a 6th joint to the model.
	 *
	 * \param body Properties of the floating base body.
	 *
	 *  \returns id of the body with 6 DoF
	 */
	unsigned int SetFloatingBaseBody (
			const Body &body
			);
	
	/** \brief Returns the 3-D coordinate vector of the origin of a given body
	 *  \brief in base coordinates
	 *
	 *  \param body_id id of the body of intrest
	 *  
	 *  \returns 3-D coordinate vector of the origin of the body in base
	 *  \returns coordinates
	 */
	Vector3d GetBodyOrigin (const unsigned int body_id);

	/** \brief Returns the orientation of a given body as 3x3 matrix
	 *
	 *  \param body_id id of the body of intrest
	 *
	 *  \returns A 3x3 matrix that contains the rotation from base
	 *  \returns orientation to body orientation
	 */
	Matrix3d GetBodyWorldOrientation (const unsigned int body_id);

	/** \brief Returns the base coordinates of a point given in body coordinates
	 *
	 * \note The forward kinematics of the model have to be computed beforehand
	 * (e.g. with ForwardKinematics() or ForwardDynamics())!
	 *
	 * \param body_id id of the body for which the point coordinates are expressed
	 * \param body_point coordinates of the point in body coordinates
	 *
	 * \returns a 3-D vector with coordinates of the point in base coordinates
	 */
	Vector3d CalcBodyToBaseCoordinates (const unsigned int body_id, const Vector3d &body_point);

	/** \brief Returns the body coordinates of a point given in base coordinates
	 *
	 * \note The forward kinematics of the model have to be computed beforehand
	 * (e.g. with ForwardKinematics() or ForwardDynamics())!
	 * 
	 * \param body_id id of the body for which the point coordinates are expressed
	 * \param base_point coordinates of the point in body coordinates
	 *
	 * \returns a 3-D vector with coordinates of the point in body coordinates
	 */
	Vector3d CalcBaseToBodyCoordinates (const unsigned int body_id, const Vector3d &base_point);

	/// \brief Initializes the helper values for the dynamics algorithm
	void Init ();

	////////////////////////////////////
	// Visualization

	typedef std::list<Visualization::Primitive> VisualizationPrimitiveList;

#ifdef EIGEN_CORE_H
	typedef std::map<unsigned int, VisualizationPrimitiveList,
					std::less<unsigned int>, Eigen::aligned_allocator<std::pair<const unsigned int, VisualizationPrimitiveList> > > BodyVisualizationMap;
#else
	typedef std::map<unsigned int, VisualizationPrimitiveList> BodyVisualizationMap;
#endif
	BodyVisualizationMap mBodyVisualization;

	/** \brief Adds a Visualization::Primitive to the primitive list of a body
	 *
	 * \param body_id id of the body that should be drawn as a box
	 * \param primitive the primitive that should be associated with the body
	 *
	 * \todo Wrap primitives up in smart pointers.
	 */
	void AddBodyVisualizationPrimitive (unsigned int body_id, Visualization::Primitive primitive);
	
	/** \brief Returns the visualization primitive for a box
	 *
	 * \param body_id id of the body of intrest
	 *
	 * \returns A pointer to the primitive or NULL if none exists.
	 */
	VisualizationPrimitiveList GetBodyVisualizationPrimitiveList (unsigned int body_id);
};

}

#endif /* _MODEL_H */
