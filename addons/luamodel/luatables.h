/*
 * luatables
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>.
 * All rights reserved.
*
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef _LUATABLES_H
#define _LUATABLES_H

#include <vector>
#include <string>

#include <rbdl/rbdl_config.h>

// forward declaration so we do not need to #include lua.h
struct lua_State;

RBDL_DLLAPI bool ltOpenFile (lua_State **L, const char *filename);
RBDL_DLLAPI void ltClose (lua_State **L);

RBDL_DLLAPI std::string ltGetStringAt (lua_State *L, const char *path_str, const int index, const std::string &default_result = "");
RBDL_DLLAPI std::string ltGetString (lua_State *L, const char *path_str, const std::string &default_result = "");

RBDL_DLLAPI double ltGetDoubleAt (lua_State *L, const char *path_str, const int index, const double &default_result = 0.);
RBDL_DLLAPI double ltGetDouble (lua_State *L, const char *path_str, const double &default_result = 0.);

RBDL_DLLAPI size_t ltGetLengthAt (lua_State *L, const char *path_str, const int index);
RBDL_DLLAPI size_t ltGetLength (lua_State *L, const char *path_str);

RBDL_DLLAPI std::vector<std::string> ltGetKeysAt (lua_State *L, const char *path_str, const int index);
RBDL_DLLAPI std::vector<std::string> ltGetKeys (lua_State *L, const char *path_str);

RBDL_DLLAPI std::vector<double> ltGetDoubleVectorAt (lua_State *L, const char *path_str, const int index);
RBDL_DLLAPI std::vector<double> ltGetDoubleVector (lua_State *L, const char *path_str); 

RBDL_DLLAPI bool ltGetDoubleArrayAt (lua_State *L, const char *path_str, const unsigned int count, double *dest, const int index);
RBDL_DLLAPI bool ltGetDoubleArray (lua_State *L, const char *path_str, const unsigned int count, double *dest); 

RBDL_DLLAPI bool ltIsNumber (lua_State *L, const char *path_str);
RBDL_DLLAPI bool ltIsNumberAt (lua_State *L, const char *path_str, int index);

RBDL_DLLAPI bool ltIsExisting (lua_State *L, const char *path_str);
RBDL_DLLAPI bool ltIsExistingAt (lua_State *L, const char *path_str, int index);

/* _LUATABLES_H */
#endif
