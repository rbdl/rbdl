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

extern "C"
{
   #include <lua.h>
   #include <lauxlib.h>
   #include <lualib.h>
}

bool ltOpenFile (lua_State **L, const char *filename);
void ltClose (lua_State **L);

std::string ltGetStringAt (lua_State *L, const char *path_str, const int index, const std::string &default_result = "");
std::string ltGetString (lua_State *L, const char *path_str, const std::string &default_result = "");

double ltGetDoubleAt (lua_State *L, const char *path_str, const int index, const double &default_result = 0.);
double ltGetDouble (lua_State *L, const char *path_str, const double &default_result = 0.);

size_t ltGetLengthAt (lua_State *L, const char *path_str, const int index);
size_t ltGetLength (lua_State *L, const char *path_str);

std::vector<std::string> ltGetKeysAt (lua_State *L, const char *path_str, const int index);
std::vector<std::string> ltGetKeys (lua_State *L, const char *path_str);

std::vector<double> ltGetDoubleVectorAt (lua_State *L, const char *path_str, const int index);
std::vector<double> ltGetDoubleVector (lua_State *L, const char *path_str); 

bool ltGetDoubleArrayAt (lua_State *L, const char *path_str, const unsigned int count, double *dest, const int index);
bool ltGetDoubleArray (lua_State *L, const char *path_str, const unsigned int count, double *dest); 

bool ltIsNumber (lua_State *L, const char *path_str);
bool ltIsNumberAt (lua_State *L, const char *path_str, int index);

bool ltIsExisting (lua_State *L, const char *path_str);
bool ltIsExistingAt (lua_State *L, const char *path_str, int index);

/* _LUATABLES_H */
#endif
