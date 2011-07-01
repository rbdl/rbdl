/** \file Mainpage.h 
 * \mainpage Rigid Body Dynamics Library
 *
 * This is the documentation of a yet to be named rigid body simulation
 * code. So far the code supports forward and inverse dynamics by using the
 * Articulated Body Algorathm and the Newton-Euler algorithm, respectively.
 * Additionally it also containes the Composite Rigid Body Algorithm that
 * computes the joint space inertia matrix.
 *
 * The code is written by <a
 * href="mailto:martin.felis@iwr.uni-heidelberg.de">Martin Felis
 * <martin.felis@iwr.uni-heidelberg.de></a> and heavily inspired by the
 * pseudo code of the book "Rigid Body Dynamics Algorithms" of <a
 * href="http://users.cecs.anu.edu.au/~roy/">Roy Featherstone</a>.
 * 
 * The library comes with version 3 of the the
 * <a href="http://eigen.tuxfamily.org/">Eigen</a> math library. More
 * information about it can be found here:
 * <a href="http://eigen.tuxfamily.org/">http://eigen.tuxfamily.org/</a>).
 *
 * Documentation of the functions can be found at the documentation page of
 * the namespace RigidBodyDynamics.
 *
 * A simple example can be found \ref SimpleExample "here".
 *
 * \section ModelConstruction Construction of Models
 *
 * The construction of models makes use of carefully designed constructors
 * of the classes Body and Joint to ease the process of creating bodies.
 * Adding bodies to the model is done by specifying the parent body by its
 * id, the transformation from the parent origin to the joint origin, the
 * joint specification as an object, and the body itself. These parameters
 * are then fed to the function RigidBodyDynamics::Model::AddBody().
 *
  * \todo [low] incorporate GiNaC (http://www.ginac.de) to generate code
 * \todo [low] serialization of the model?
 *
 * \subsection ToDo_done Done:
 * <ul>
 *   <li>[high] check impulse computation 2011-06-07</li>
 *   <li>[med] add specification for the visualization to the model</li>
 *   <li>[med] get replace std::vector<> vectors with proper math vectors in Model</li>
 * </ul>
 */
