#define CATCH_CONFIG_MAIN
#include <rbdl/rbdl.h>
#include <string>
#include <vector>
#include <cstring>
#include "rbdl_tests.h"
#include <rbdl/rbdl_utils.h>
#include "urdfreader.h"



using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;

using namespace std;

const double TEST_PREC = 1.0e-4;

std::string rbdlSourcePath = RBDL_URDFREADER_SOURCE_DIR;
TEST_CASE(__FILE__"_LoadURDF", "")
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/test.urdf");
  bool modelLoaded = URDFReadFromFile(modelFile.c_str(), &model, false);
  CHECK(modelLoaded);
}
TEST_CASE(__FILE__"_CenterOfMass", ""){
  // Checks model COM against known values
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/test.urdf");
  bool modelLoaded = URDFReadFromFile(modelFile.c_str(), &model, false);

  VectorNd q_zero (VectorNd::Zero (model.q_size));
  VectorNd qdot_zero (VectorNd::Zero (model.qdot_size));
  RigidBodyDynamics::UpdateKinematics (model, q_zero, qdot_zero, qdot_zero);


  SpatialRigidBodyInertia rbi_base = model.X_base[1].apply(model.I[1]);
  Vector3d body_com = rbi_base.h / rbi_base.m;
  CHECK_THAT(body_com.transpose()[0], IsClose(-0.0697169,TEST_PREC,TEST_PREC));
  CHECK_THAT(body_com.transpose()[1], IsClose(0.194009,TEST_PREC,TEST_PREC));
  CHECK_THAT(body_com.transpose()[2], IsClose(0.1,TEST_PREC,TEST_PREC));

  Vector3d model_com;
  double mass;
  RigidBodyDynamics::Utils::CalcCenterOfMass (model, q_zero, qdot_zero,
					      NULL, mass, model_com);
  CHECK_THAT(model_com[0], IsClose(0.2,TEST_PREC,TEST_PREC));
  CHECK_THAT(model_com[1], IsClose(0.0945087,TEST_PREC,TEST_PREC));
  CHECK_THAT(model_com[2], IsClose(-0.0597335,TEST_PREC,TEST_PREC));

  CHECK_THAT(mass, IsClose(1.5,TEST_PREC, TEST_PREC));
}
