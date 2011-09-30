/** \file example.h 
 * \page SimpleExample A simple example
 *
 * Here is a simple example how one can create a meaningless model and
 * compute the forward dynamics for it:
 *
 * \include example.cc
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
