#include "rbdl_tests.h"

#include "rbdl/Logging.h"
#include "rbdl/rbdl_math.h"
#include "rbdl/rbdl_mathutils.h"
#include <iostream>

const double TEST_PREC = 1.0e-14;

using namespace std;
using namespace RigidBodyDynamics::Math;

using namespace Catch::Matchers::Floating;

struct MathFixture {
};

TEST_CASE(__FILE__"_GaussElimPivot", "") {
  ClearLogOutput();

  MatrixNd A;
  A.resize(3,3);
  VectorNd b(3);
  VectorNd x(3);

  A(0,0) = 0; A(0,1) = 2; A(0,2) = 1;
  A(1,0) = 1; A(1,1) = 1; A(1,2) = 5;
  A(2,0) = 0; A(2,1) = 0; A(2,2) = 1;

  b[0] = 1;
  b[1] = 2;
  b[2] = 3;

  VectorNd test_result (3);

  test_result[0] = -12;
  test_result[1] = -1;
  test_result[2] = 3;

  LinSolveGaussElimPivot (A, b, x);

  REQUIRE_THAT (test_result, AllCloseVector(x));

  A(0,0) = 0; A(0,1) = -2; A(0,2) = 1;
  A(1,0) = 1; A(1,1) =  1; A(1,2) = 5;
  A(2,0) = 0; A(2,1) =  0; A(2,2) = 1;

  LinSolveGaussElimPivot (A, b, x);
  test_result[0] = -14;
  test_result[1] = 1;
  test_result[2] = 3;

  x[0] += 1.0e-13;

  REQUIRE_THAT (test_result, AllCloseVector(x));
}