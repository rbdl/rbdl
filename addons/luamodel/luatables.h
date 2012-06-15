/*
 * luatables
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 * 
 * (zlib license)
 * 
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * 
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 
 *    1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 
 *    2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 
 *    3. This notice may not be removed or altered from any source
 *    distribution.
 */

#ifndef _LUATABLES_H
#define _LUATABLES_H

#include <vector>
#include <string>

extern "C"
{
   #include <lua.h>
   #include <lauxlib.h>
   #include <lualib.h>
}

/** \brief Puts a lua value at a given path with optional index onto the stack.
 *
 * This function allows to navigate tables by specifying the path to an
 * element just as within lua, e.g. "model.bodies[2].inertia[2][3]". The
 * optional last parameter is used to ease iteration over multiple
 * elements.
 */
bool get_table_from_path (lua_State *L, const std::string &path_str, int index = -1);

bool value_exists (lua_State *L, const std::string &path_str, int index = -1);

std::string get_string (lua_State *L, const std::string &path_str, int index = -1);
double get_number (lua_State *L, const std::string &path_str, int index = -1);
const void* get_pointer (lua_State *L, const std::string &path_str, int index = -1);

size_t get_length (lua_State *L, const std::string &path_str, int index = -1);

std::vector<double> get_array (lua_State *L, const std::string &path_str, int index = -1);
std::vector<std::string> get_keys (lua_State *L, const std::string &path_str, int index = -1);

/* _LUATABLES_H */
#endif
