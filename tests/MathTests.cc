#include <UnitTest++.h>

#include "rbdl/Logging.h"
#include "rbdl/rbdl_math.h"
#include "rbdl/rbdl_mathutils.h"

// The bundled SimpleMath header only compiles with NDEBUG defined: its
// debug-only Storage::resize() block references parameter names that the
// UNUSED macro renames. Guard the float quaternion coverage accordingly.
#ifdef NDEBUG
#include "rbdl/SimpleMath/SimpleMath.h"
#endif
#include <iostream>

const double TEST_PREC = 1.0e-14;

using namespace std;
using namespace RigidBodyDynamics::Math;

struct MathFixture {
};

TEST (GaussElimPivot) {
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

  CHECK_ARRAY_CLOSE (test_result.data(), x.data(), 3, TEST_PREC);

  A(0,0) = 0; A(0,1) = -2; A(0,2) = 1;
  A(1,0) = 1; A(1,1) =  1; A(1,2) = 5;
  A(2,0) = 0; A(2,1) =  0; A(2,2) = 1;

  LinSolveGaussElimPivot (A, b, x);
  test_result[0] = -14;
  test_result[1] = 1;
  test_result[2] = 3;

  CHECK_ARRAY_CLOSE (test_result.data(), x.data(), 3, TEST_PREC);
}

TEST (Dynamic_1D_initialize_value) {
  VectorNd myvector_10 = VectorNd::Constant ((size_t) 10, 12.);

  double *test_values = new double[10];
  for (unsigned int i = 0; i < 10; i++)
    test_values[i] = 12.;

  CHECK_ARRAY_EQUAL (test_values, myvector_10.data(), 10);
  delete[] test_values;
}

TEST (Dynamic_2D_initialize_value) {
  MatrixNd mymatrix_10x10 = MatrixNd::Constant (10, 10, 12.);

  double *test_values = new double[10 * 10];
  for (unsigned int i = 0; i < 10; i++)
    for (unsigned int j = 0; j < 10; j++)
      test_values[i*10 + j] = 12.;

  CHECK_ARRAY_EQUAL (test_values, mymatrix_10x10.data(), 10*10);
  delete[] test_values;
}

TEST (SpatialMatrix_Multiplication) {
  SpatialMatrix X_1 (
      1.,  2.,  3.,  4.,  5.,  6.,
      11., 12., 13., 14., 15., 16.,
      21., 22., 23., 24., 25., 26.,
      31., 32., 33., 34., 35., 36.,
      41., 42., 43., 44., 45., 46.,
      51., 52., 53., 54., 55., 56.
      );

  SpatialMatrix X_2 (X_1);

  X_2 *= 2.;

  SpatialMatrix correct_result (
      1442,    1484,    1526,    1568,    1610,    1652,
      4562,    4724,    4886,    5048,    5210,    5372,
      7682,    7964,    8246,    8528,    8810,    9092,
      10802,   11204,   11606,   12008,   12410,   12812,
      13922,   14444,   14966,   15488,   16010,   16532,
      17042,   17684,   18326,   18968,   19610,   20252
      );

  SpatialMatrix test_result = X_1 * X_2;

  CHECK_ARRAY_CLOSE (correct_result.data(), test_result.data(), 6 * 6, TEST_PREC);

  // check the *= operator:
  test_result = X_1;
  test_result *= X_2;

  CHECK_ARRAY_CLOSE (correct_result.data(), test_result.data(), 6 * 6, TEST_PREC);
}

TEST (QuaternionFromMatrixExactHalfTurn) {
  // Regression test: the trace-only implementation divided by w = 0 for
  // exact 180-degree rotations and returned NaNs.
  Matrix3d half_turns[3];
  half_turns[0] = Matrix3d::Identity();
  half_turns[0](1,1) = -1.;
  half_turns[0](2,2) = -1.;
  half_turns[1] = Matrix3d::Identity();
  half_turns[1](0,0) = -1.;
  half_turns[1](2,2) = -1.;
  half_turns[2] = Matrix3d::Identity();
  half_turns[2](0,0) = -1.;
  half_turns[2](1,1) = -1.;
  for (int i = 0; i < 3; i++) {
    Quaternion q = Quaternion::fromMatrix (half_turns[i]);
    CHECK (q.allFinite());
    CHECK_ARRAY_CLOSE (half_turns[i].data(), q.toMatrix().data(), 9,
                       TEST_PREC);
  }
}

TEST (QuaternionFromMatrixRoundtrip) {
  // fromMatrix must invert toMatrix, including for rotations with negative
  // trace, exact half turns, and tiny offsets around half turns, and keep
  // the w >= 0 hemisphere.
  for (int i = 0; i < 100; i++) {
    Vector3d axis = Vector3d (
        std::sin (i * 1.7), std::cos (i * 0.9), std::sin (i * 0.3 + 1.));
    double angle = M_PI * (i % 11) / 10.0;
    Quaternion q = Quaternion::fromAxisAngle (axis, angle);
    Quaternion r = Quaternion::fromMatrix (q.toMatrix());
    CHECK (r.allFinite());
    CHECK_ARRAY_CLOSE (q.toMatrix().data(), r.toMatrix().data(), 9,
                       TEST_PREC);
    CHECK (r[3] >= 0.);
  }
  for (int i = 0; i < 10; i++) {
    Vector3d axis = Vector3d (
        std::sin (i * 2.3 + 0.5), std::cos (i * 1.1),
        std::sin (i * 0.7 + 2.));
    double offsets[3] = {-1e-8, 0., 1e-8};
    for (int j = 0; j < 3; j++) {
      Quaternion q = Quaternion::fromAxisAngle (axis, M_PI + offsets[j]);
      Quaternion r = Quaternion::fromMatrix (q.toMatrix());
      CHECK (r.allFinite());
      CHECK_ARRAY_CLOSE (q.toMatrix().data(), r.toMatrix().data(), 9,
                         TEST_PREC);
      CHECK (r[3] >= 0.);
    }
  }
}

#ifdef NDEBUG
TEST (SimpleMathQuaternionFromMatrixHalfTurn) {
  // Same coverage for the float SimpleMath::GL::Quaternion variant, which
  // uses the transpose (standard) sign convention.
  SimpleMath::Matrix<float, 3, 3> half_turns[3];
  for (int i = 0; i < 3; i++) {
    half_turns[i] = SimpleMath::Matrix<float, 3, 3>::Identity();
    half_turns[i]((i + 1) % 3, (i + 1) % 3) = -1.f;
    half_turns[i]((i + 2) % 3, (i + 2) % 3) = -1.f;
  }
  for (int i = 0; i < 3; i++) {
    SimpleMath::GL::Quaternion q =
        SimpleMath::GL::Quaternion::fromMatrix (half_turns[i]);
    for (int k = 0; k < 4; k++) {
      CHECK (std::isfinite (q[k]));
    }
    SimpleMath::Matrix<float, 3, 3> r = q.toMatrix();
    for (int a = 0; a < 3; a++) {
      for (int b = 0; b < 3; b++) {
        CHECK_CLOSE (half_turns[i](a,b), r(a,b), 1.0e-5);
      }
    }
  }
  float offsets[3] = {-1e-3f, 0.f, 1e-3f};
  for (int i = 0; i < 5; i++) {
    SimpleMath::Matrix<float, 3, 1> axis;
    axis[0] = std::sin (i * 2.3f + 0.5f);
    axis[1] = std::cos (i * 1.1f);
    axis[2] = std::sin (i * 0.7f + 2.f);
    for (int j = 0; j < 3; j++) {
      SimpleMath::GL::Quaternion q = SimpleMath::GL::Quaternion (
          axis[0] * std::sin ((M_PI + offsets[j]) * 0.5) / axis.norm(),
          axis[1] * std::sin ((M_PI + offsets[j]) * 0.5) / axis.norm(),
          axis[2] * std::sin ((M_PI + offsets[j]) * 0.5) / axis.norm(),
          std::cos ((M_PI + offsets[j]) * 0.5));
      SimpleMath::GL::Quaternion r =
          SimpleMath::GL::Quaternion::fromMatrix (q.toMatrix());
      for (int k = 0; k < 4; k++) {
        CHECK (std::isfinite (r[k]));
      }
      CHECK (r[3] >= 0.f);
      SimpleMath::Matrix<float, 3, 3> m = r.toMatrix(), mq = q.toMatrix();
      for (int a = 0; a < 3; a++) {
        for (int b = 0; b < 3; b++) {
          CHECK_CLOSE (mq(a,b), m(a,b), 1.0e-4);
        }
      }
    }
  }
}
#endif  // NDEBUG
