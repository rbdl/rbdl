/** @page Casadi
 *
 *  RBDL comes in two flavors which only differ in what mathematical backend they use.
 *  Casadi is a very interesting backend, because it has some unique features such as alogrithmic
 *  differentiation. But due to its complexity it is not the default.
 *
 *  This page documents how to use the casadi version of RBDL. If you are unsure if you need casadi
 *  then you probably don't and should stick to the standart RBDL that uses Eigen as it's backend.
 *  For more information about casadi take a look at their <a href="https://web.casadi.org/"> website </a>.
 *
 *  The casadi_simple example shows how to build a simple application that uses casadi:
 *
 *  CMakeLists.txt: note that the FindCasadi.cmake file is also required to find the correct include paths for casadi
 *  \include casadi_simple/CMakeLists.txt
 *
 *  A simple rbdl example source file that works with casadi:
 *  \include casadi_simple/example.cc
 */
