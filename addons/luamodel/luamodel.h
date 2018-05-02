#ifndef _RBDL_LUAMODEL_H
#define _RBDL_LUAMODEL_H

#include <rbdl/rbdl_config.h>
#include <string>
#include <vector>

extern "C" {
  struct lua_State;
};

namespace RigidBodyDynamics {

struct Model;
struct ConstraintSet;

namespace Addons {
  
/** \page addon_luamodel_page Addon: rbdl_luamodel
* @{
*
* \section luamodel_introduction Lua Models
*
* The Addon LuaModel allows to load \link RigidBodyDynamics::Model Models\endlink
* that are specified as Lua scripts. <a href="http://www.lua.org">Lua</a> is
* an open-source light-weight scripting language (http://www.lua.org).
* This addon is not enabled by default and one has to enable it by
* setting BUILD_ADDON_LUAMODEL to true in when configuring the RBDL with
* CMake.
* 
* This addon comes with a standalone utility that can show various
* information of a lua model such as degree of freedom overview or model
* hierarchy. It is located in addons/luamodel/rbdl_luamodel_test. Use the -h
* switch to see available options.
* 
* Note: this addon is not even remotely as thoroughly tested as the RBDL
* itself so please use it with some suspicion.
* 
* \section luamodel_format Format Overview 
* 
* Models have to be specified as a specially formatted Lua table which must
* be returned by the script, i.e. if the model is specified in the table
* "model = { ... }" the script has to return this when executed. Within the
* returned table, rbdl_luamodel goes through the table "frames" and builds
* the model from the individual Frame Information Tables (see further down
* for more information about those).
* 
* A valid file could look like this:
* 
* \code
* model = {
*   frames = {
*     {
*       <frame 1 information table>
*     },
*     {
*       <frame 2 information table>
*     }
*   }
* }
* 
* return model
* \endcode
*
* Apart from the frames you can also specify gravity in the model file.
*
* Example:
* \code
* model = {
*   gravity = {0., 0., -9.81}
*
*   frames = {
*   ...
*   }
* }
* \endcode
* 
* Finally, several constraint sets can be defined for the model.
*
* Example:
* \code
* model = {
*   constraint_sets = {
*     constraint_set_1_name = {
*       {
*         ...
*       },
*       {
*         ...
*       },
*     },
*     constraint_set_2_name = {
*       ...
*     },
*   },  
* }
* \endcode
*
* \note The table frames must contain all Frame Information Tables as a list
* and individual tables must *not* be specified with a key, i.e.
* \code
*   frames = {
*     some_frame = {
*       ...
*     },
*     {
*       ..
*     }
*   }
* \endcode
* is not possible as Lua does not retain the order of the individual
* frames when an explicit key is specified.
* 
* \section luamodel_frame_table Frame Information Table
* 
* The Frame Information Table is searched for values needed to call
* Model::AddBody(). The following fields are used by rbdl_luamodel
* (everything else is ignored):
*
* \par name (required, type: string):
*     Name of the body that is being added. This name must be unique.
* 
* \par parent (required, type: string):
*     If the value is "ROOT" the parent frame of this body is assumed to be
*     the base coordinate system, otherwise it must be the exact same string
*     as was used for the "name"-field of the parent frame.
*   
* \par body (optional, type: table)
*     Specification of the dynamical parameters of the body. It uses the
*     values (if existing):
*     \code
*       mass (scalar value, default 1.),
*       com (3-d vector, default:  (0., 0., 0.))
*       inertia (3x3  matrix, default: identity matrix)
*     \endcode
* \par
*     to create a \ref RigidBodyDynamics::Body.
* 
* \par joint (optional, type: table)
*     Specifies the type of joint, fixed or up to 6 degrees of freedom. Each
*     entry in the joint table should be a 6-d that defines the motion
*     subspace of a single degree of freedom.
* \par
*     Default joint type is a fixed joint that attaches the body rigidly to
*     its parent.
* \par
*     3-DoF joints are described by using strings. The following 3-DoF
*     joints are available:
*
*     - "JointTypeSpherical": for singularity-free spherical joints
*     - "JointTypeEulerZYX": Euler ZYX joint
*     - "JointTypeEulerXYZ": Euler XYZ joint
*     - "JointTypeEulerYXZ": Euler YXZ joint
*     - "JointTypeTranslationXYZ": Free translational joint
*
* \par
*    Examples:
* \code    
*       joint_fixed = {}
*       joint_translate_x = { {0., 0., 0., 1., 0., 0.} }
*       joint_translate_xy = { 
*         {0., 0., 0., 1., 0., 0.},
*         {0., 0., 0., 0., 1., 0.}
*       }
*       joint_rotate_zyx = {
*         {0., 0., 1., 0., 0., 0.},
*         {0., 1., 0., 0., 0., 0.},
*         {1., 0., 0., 0., 0., 0.},
*       }
*       joint_rotate_euler_zyx = { "JointTypeEulerZYX" }
*   \endcode
*       
* \par  joint_frame (optional, type: table)
*     Specifies the origin of the joint in the frame of the parent. It uses
*     the values (if existing):
*   \code
*       r (3-d vector, default: (0., 0., 0.))
*       E (3x3 matrix, default: identity matrix)
*   \endcode
* \par
*    for which r is the translation and E the rotation of the joint frame
*    
* \section luamodel_constraint_table Constraint Information Table
* The Constraint Information Table is searched for values needed to call
* ConstraintSet::AddContactConstraint() or ConstraintSet::AddLoopConstraint().
* The employed fields are defined as follows. Please note that different fields
* may be used for different constraint types.
*
* \par constraint_type (required, type: string)
*     Specifies the type of constraints, either 'contact' or 'loop', other
*     values will cause a failure while reading the file.
* 
* \par name (optional, type: string)
*     Specifies a name for the constraint.
*
* The following fields are used exclusively for contact constraints:
*
* \par body (required, type: string)
*     The name of the body involved in the constraint.
*
* \par point(optional, type: table)
*     The position of the contact point in body coordinates. Defaults to
*     (0, 0, 0).
*
* \par normal(optional, type: table)
*     The normal along which the constraint acts in world coordinates.
*     Defaults to (0, 0, 0).
*
* \par normal_acceleration(optional, type: number)
*     The normal acceleration along the constraint axis. Defaults to 0.
*
* The following fields are used exclusively for loop constraints:
*
* \par predecessor_body (required, type: string)
*     The name of the predecessor body involved in the constraint.
*
* \par successor_body (required, type: string)
*     The name of the successor body involved in the constraint.
*
* \par predecessor_transform (optional, type: table)
*     The transform of the predecessor constrained frame with respect to 
*     the predecessor body frame. Defaults to the identity transform.
*
* \par sucessor_transform (optional, type: table)
*     The transform of the sucessor constrained frame with respect to 
*     the sucessor body frame. Defaults to the identity transform.
*
* \par axis (optional, type: table)
*     The 6d spatial axis along which the constraint acts, in coordinates
*     relative to the predecessor constrained frame. Defaults to (0,0,0,0,0,0).
*
* \par enable_stabilization(optional, type: bool, default: false)
*     Whether Baumgarte stabilization should be enabled or not.
*
* \par stabilization_parameter(optional, type: number)
*     The stabilization parameter T_stab for the Baumgarte stabilization (see
*     RBDL documentation on how this parameter is used). Defaults to 0.1.
*
* \section luamodel_example Example
* Here is an example of a model:
* \include samplemodel.lua
*
* \section luamodel_example_constraints Example with constraints
* Here is an example of a model including constraints:
* \include sampleconstrainedmodel.lua
*
*/

/** \brief Reads a model from a Lua file.
 *
 * \param filename the name of the Lua file.
 * \param model a pointer to the output Model structure.
 * \param verbose specifies wether information on the model should be printed
 * (default: true).
 *
 * \returns true if the model was read successfully.
 *
 * \note See \ref luamodel_introduction for information on how to define the
 * Lua model.
 */
RBDL_DLLAPI
bool LuaModelReadFromFile (
  const char* filename,
  Model* model,
  bool verbose = false);

/** \brief Reads a model file and returns the names of all constraint sets.
 */
RBDL_DLLAPI
std::vector<std::string> LuaModelGetConstraintSetNames(const char* filename);

/** \brief Reads a model and constraint sets from a Lua file.
 *
 * \param filename the name of the Lua file.
 * \param model a pointer to the output Model structure.
 * \param constraint_sets reference to a std::vector of ConstraintSet structures
 * in which to save the information read from the file.
 * \param constraint_set_names reference to a std::vector of std::string
 * specifying the names of the constraint sets to be read from the Lua file.
 * \param verbose specifies wether information on the model should be printed
 * (default: true).
 *
 * \returns true if the model and constraint sets were read successfully.
 *
 * \note constraint_sets and constraint_set_names are required to have the same
 * size. See \ref luamodel_introduction for information on how to define the
 * Lua model.
 */
RBDL_DLLAPI
bool LuaModelReadFromFileWithConstraints (
  const char* filename,
  Model* model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose = false);

/** \brief Reads a model from a lua_State.
 *
 * \param L a pointer to the lua_State.
 * \param model a pointer to the output Model structure.
 * \param verbose specifies wether information on the model should be printed
 * (default: true).
 *
 * \returns true if the model was read successfully.
 */
RBDL_DLLAPI
bool LuaModelReadFromLuaState (
  lua_State* L,
  Model* model,
  bool verbose = false);

/** @} */
}

}

/* _RBDL_LUAMODEL_H */
#endif
