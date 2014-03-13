#ifndef _RBDL_LUAMODEL_H
#define _RBDL_LUAMODEL_H

#include <rbdl/rbdl_config.h>

namespace RigidBodyDynamics {

class Model;

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
   * \section luamodel_example Example
	 * Here is an example of a model:
	 * \include samplemodel.lua
	 */

	RBDL_DLLAPI bool LuaModelReadFromFile (const char* filename, Model* model, bool verbose = false);

	/** @} */
}

}

/* _RBDL_LUAMODEL_H */
#endif
