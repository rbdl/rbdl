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

#include "luatables.h"

#include <iostream>
#include <cstdlib>
#include <vector>
#include <sstream>

extern "C"
{
   #include <lua.h>
   #include <lauxlib.h>
   #include <lualib.h>
}

using namespace std;

void bail(lua_State *L, const char *msg){
	std::cerr << msg << lua_tostring(L, -1) << endl;
	abort();
}

/** \brief Puts a lua value at a given path with optional index onto the stack.
 *
 * This function allows to navigate tables by specifying the path to an
 * element just as within lua, e.g. "model.bodies[2].inertia[2][3]". The
 * optional last parameter is used to ease iteration over multiple
 * elements.
 */
bool get_table_from_path (lua_State *L, const char *path_str, const int index = -1);


/* Proxy functions for ltXXXAt() calls */
std::string ltGetString (lua_State *L, const char *path_str, const std::string &default_result) {
	return ltGetStringAt (L, path_str, -1, default_result);
}

double ltGetDouble (lua_State *L, const char *path_str, const double &default_result) {
	return ltGetDoubleAt (L, path_str, -1, default_result);
}

size_t ltGetLength (lua_State *L, const char *path_str) {
	return ltGetLengthAt (L, path_str, -1);
}

std::vector<std::string> ltGetKeys (lua_State *L, const char *path_str) {
	return ltGetKeysAt (L, path_str, -1);
}

std::vector<double> ltGetDoubleVector (lua_State *L, const char *path_str) {
	return ltGetDoubleVectorAt (L, path_str, -1);
}

bool ltGetDoubleArray (lua_State *L, const char *path_str, const unsigned int count, double *dest) {
	return ltGetDoubleArrayAt (L, path_str, count, dest, -1);
}

bool ltIsNumber (lua_State *L, const char *path_str) {
	return ltIsNumberAt (L, path_str, -1);
}

bool ltIsExisting (lua_State *L, const char *path_str) {
	return ltIsExistingAt (L, path_str, -1);
}

/* Actual interesting code */

bool ltOpenFile (lua_State **L, const char *filename) {
	*L = luaL_newstate();
	luaL_openlibs(*L);

	if (luaL_loadfile (*L, filename) || lua_pcall (*L, 0, 1, 0)) {
		bail (*L, "Error running file: ");
		return false;
	}

	return true;
}

void ltClose (lua_State **L) {
	lua_close(*L);
	*L = NULL;
}

std::string ltGetStringAt (lua_State *L, const char *path_str, const int index, const std::string &default_result) {
	std::string result;

	int stack_top = lua_gettop(L);

	if (!get_table_from_path(L, path_str, index)) 
		return default_result;

	if (!lua_isstring(L, -1)) {
		cout << "Error: value at " << path_str;
		if (index > 0)
			cout << "[" << index << "]";
		cout << " is not a string!" << endl;

		return default_result;
	}

	result = lua_tostring(L, -1);

	// clean up the stack
	lua_pop (L, lua_gettop(L) - stack_top);

	return result;
}

double ltGetDoubleAt (lua_State *L, const char *path_str, const int index, const double &default_result) {
	double result;
	
	int stack_top = lua_gettop(L);

	if (!get_table_from_path(L, path_str, index)) 
		return default_result;

	if (!lua_isnumber(L, -1)) {
		cout << "Error: value at " << path_str;
		if (index > 0)
			cout << "[" << index << "]";
		cout << " is not a number!" << endl;

		return default_result;
	}

	result = lua_tonumber(L, -1);

	// clean up the stack
	lua_pop (L, lua_gettop(L) - stack_top);

	return result;
}

size_t ltGetLengthAt (lua_State *L, const char *path_str, const int index) {
	size_t result = 0;
	
	int stack_top = lua_gettop(L);

	if (!get_table_from_path(L, path_str, index)) 
		return result;

	result = lua_objlen (L, -1);

	// clean up the stack
	lua_pop (L, lua_gettop(L) - stack_top);

	return result;
}

std::vector<std::string> ltGetKeysAt (lua_State *L, const char *path_str, const int index) {
	std::vector<string> result;
	
	int stack_top = lua_gettop(L);

	if (!get_table_from_path(L, path_str, index)) 
		return result;

	if (!lua_istable(L, -1)) {
		cout << "Error: value at " << path_str;
		if (index > 0)
			cout << "[" << index << "]";
		cout << " is not a table!" << endl;

		// clean up the stack
		lua_pop (L, lua_gettop(L) - stack_top);
		return result;
	}

	lua_pushnil(L);

	int i = 1;
	while (lua_next(L, -2) != 0) {
		if (lua_isnumber(L, -2)) {
			// if top value is a number we convert it to a string using lua
			// facilities. Note: lua_tostring modifies the value and can thus
			// confuse the call of lua_next()!
			lua_pushvalue(L, -2);
			result.push_back(lua_tostring(L, -1));
			lua_pop(L, 1);
		} else if (lua_isstring(L, -2)) {
			result.push_back (lua_tostring(L, -2));
		}
		else {
			cout << "Error: values at " << path_str;
			if (index > 0)
				cout << "[" << index << "]";
			cout << " is not a string!" << endl;

			// clean up the stack
			lua_pop (L, lua_gettop(L) - stack_top);
			return std::vector<std::string>();
		}

		lua_pop(L, 1);
		i++;
	}

	// clean up the stack
	lua_pop (L, lua_gettop(L) - stack_top);

	return result;
}

std::vector<double> ltGetDoubleVectorAt (lua_State *L, const char *path_str, const int index) {
	std::vector<double> result;
	
	int stack_top = lua_gettop(L);

	if (!get_table_from_path(L, path_str, index)) 
		return result;

	if (!lua_istable(L, -1)) {
		cout << "Error: value at " << path_str;
		if (index > 0)
			cout << "[" << index << "]";
		cout << " is not a table!" << endl;

		// clean up the stack
		lua_pop (L, lua_gettop(L) - stack_top);
		return result;
	}

	lua_pushnil(L);

	int i = 1;
	while (lua_next(L, -2)) {
		if (lua_isnumber (L, -1)) {
			result.push_back (lua_tonumber(L, -1));
		} else {
			cout << "Error: values at " << path_str;
			if (index > 0)
				cout << "[" << index << "]";
			cout << " are not numbers only!" << endl;

			// clean up the stack
			lua_pop (L, lua_gettop(L) - stack_top);
			return std::vector<double>();
		}
		lua_pop(L, 1);
		i++;
	}

	// clean up the stack
	lua_pop (L, lua_gettop(L) - stack_top);

	return result;
}

bool ltGetDoubleArrayAt (lua_State *L, const char *path_str, const unsigned int count, double *dest, const int index) {
	std::vector<double> result;
	
	int stack_top = lua_gettop(L);

	if (!get_table_from_path(L, path_str, index)) 
		return false;

	if (!lua_istable(L, -1)) {
		cout << "Error: value at " << path_str;
		if (index > 0)
			cout << "[" << index << "]";
		cout << " is not a table!" << endl;

		// clean up the stack
		lua_pop (L, lua_gettop(L) - stack_top);
		return false;
	}

	lua_pushnil(L);

	int i = 1;
	while (lua_next(L, -2)) {
		if (lua_isnumber (L, -1)) {
			result.push_back (lua_tonumber(L, -1));
		} else {
			cout << "Error: values at " << path_str;
			if (index > 0)
				cout << "[" << index << "]";
			cout << " are not numbers only!" << endl;

			// clean up the stack
			lua_pop (L, lua_gettop(L) - stack_top);
			return false;
		}
		lua_pop(L, 1);
		i++;
	}

	// clean up the stack
	lua_pop (L, lua_gettop(L) - stack_top);

	if (result.size() >= count) {
		for (unsigned int i = 0; i < count; i++) {
			dest[i] = result[i];
		}
		return true;
	}

	cout << "Error: Tried to read " << count << " values at " << path_str;
	if (index > 0)
		cout << "[" << index << "]";
	cout << " but only found " << result.size() << " elements!" << endl;

	return false;
}

bool ltIsNumberAt (lua_State *L, const char *path_str, const int index) {
	bool result = false;

	int stack_top = lua_gettop(L);

	if (!get_table_from_path(L, path_str, index)) 
		result = false;

	if (lua_isnumber(L, -1)) {
		result = true;
	}

	// clean up the stack
	lua_pop (L, lua_gettop(L) - stack_top);

	return result;
}

bool ltIsExistingAt (lua_State *L, const char *path_str, int index) {
	int stack_top = lua_gettop(L);

	if (!get_table_from_path(L, path_str, index)) {
		return false;
	}

	lua_pop (L, lua_gettop(L) - stack_top);
	return true;
}

/** \brief Extracts a single token from a path string */
static std::string path_get_next_token (std::string &path) {
	std::string token = path;

	bool is_index = false;
	bool have_bracket = false;

	if (token.find(".") != std::string::npos) {
		token = token.substr(0, token.find("."));
	}

	if (token.find("[") != std::string::npos) {
		have_bracket = true;

		if (token.find("[") == 0) {
			token = token.substr (token.find("[") + 1, token.find("]") - 1);
			path = path.substr (token.size() + 2, path.size());
		} else {
			token = token.substr (0, token.find("["));
			path = path.substr (token.size(), path.size());
		}
	} else {
		if (path.size() > token.size())
			path = path.substr (token.size() + 1, path.size());
		else
			path = "";
	}

	if (path[0] == '.')
		path = path.substr (1, path.size());

	return token;
}

/** \brief Puts a lua value at a given path with optional index onto the stack.
 *
 * This function allows to navigate tables by specifying the path to an
 * element just as within lua, e.g. "model.bodies[2].inertia[2][3]". The
 * optional last parameter is used to ease iteration over multiple
 * elements.
 */
bool get_table_from_path (lua_State *L, const char *path_str, const int index) {
	std::string path = path_str;
	std::string token = path;

	// backup of the current stack
	int stack_top = lua_gettop(L);

	do {
		token = path_get_next_token (path);

		istringstream convert (token);
		int token_int;
		if (!(convert >> token_int)) 
			lua_pushstring(L, token.c_str());
		else
			lua_pushnumber (L, token_int);

		lua_gettable (L, -2);

		if (path.size() == 0 && index > 0) {
			lua_pushnumber (L, index);
			lua_gettable (L, -2);
		}

		if (lua_isnil(L, -1)) {
			// clean up the stack
			lua_pop (L, lua_gettop(L) - stack_top);
			return false;
		}

	} while (path.size() > 0);

	return true;
}
