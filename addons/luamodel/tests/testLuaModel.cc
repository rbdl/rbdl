/*
 * RBDL - Rigid Body Dynamics Library: Addon : muscle
 * Copyright (c) 2016 Matthew Millard <millard.matthew@gmail.com>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

//==============================================================================
// INCLUDES
//==============================================================================
#include <UnitTest++.h>

#include "luamodel.h"
#include "luatables.h"
#include <rbdl/rbdl.h>
#include <string>
#include <vector>
#include <cstring>

#ifdef RBDL_BUILD_ADDON_MUSCLE
#include "../muscle/Millard2016TorqueMuscle.h"
#endif


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons;



using namespace std;

const double TEST_PREC = 1.0e-11;

std::string rbdlSourcePath;
   
TEST(LoadLuaModel)
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  CHECK(modelLoaded);
}
TEST(LoadMotionCaptureMarkers)
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  std::vector< Point > updMarkerSet;
  bool markersLoaded = LuaModelReadMotionCaptureMarkers(modelFile.c_str(),
                                               &model, updMarkerSet,false);
  CHECK(updMarkerSet.size()==6);
  //The markers come out of order which makes testing a bit tricky.
  for(unsigned int i=0; i<updMarkerSet.size(); ++i){
    bool flag_found = false;
    if(std::strcmp(updMarkerSet[i].name.c_str(),"LASI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("pelvis"));
      CHECK_CLOSE(updMarkerSet[i].point_local[0], 0.047794,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[1], 0.200000,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[2], 0.070908,TEST_PREC);
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"RASI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("pelvis"));
      CHECK_CLOSE(updMarkerSet[i].point_local[0], 0.047794,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[1],-0.200000,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[2], 0.070908,TEST_PREC);
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"LPSI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("pelvis"));
      CHECK_CLOSE(updMarkerSet[i].point_local[0],-0.106106,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[1], 0.200000,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[2], 0.070908,TEST_PREC);
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"RPSI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("pelvis"));
      CHECK_CLOSE(updMarkerSet[i].point_local[0],-0.106106,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[1],-0.200000,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[2], 0.070908,TEST_PREC);
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"RTHI")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("thigh_right"));
      CHECK_CLOSE(updMarkerSet[i].point_local[0],-0.007376,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[1], 0.000000,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[2],-0.243721,TEST_PREC);
      flag_found = true;
    }
    if(std::strcmp(updMarkerSet[i].name.c_str(),"RKNE")==0){
      CHECK( updMarkerSet[i].body_id == model.GetBodyId("thigh_right"));
      CHECK_CLOSE(updMarkerSet[i].point_local[0],-0.011611,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[1], 0.000000,TEST_PREC);
      CHECK_CLOSE(updMarkerSet[i].point_local[2],-0.454494,TEST_PREC);
      flag_found = true;
    }
    CHECK(flag_found);
  }


}
TEST(LoadLocalFrames)
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  std::vector< LocalFrame > updLocalFrameSet;
  bool localFramesLoaded = LuaModelReadLocalFrames(modelFile.c_str(),&model,
                                                    updLocalFrameSet,false);

  CHECK(updLocalFrameSet.size()==2);

  unsigned int thighLeftId = model.GetBodyId("thigh_left");
  unsigned int thighRightId = model.GetBodyId("thigh_right");

  CHECK(std::strcmp("Pocket_L",updLocalFrameSet[0].name.c_str())==0);
  CHECK(updLocalFrameSet[0].body_id == thighLeftId);

  CHECK_CLOSE(updLocalFrameSet[0].r[0], 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[0].r[1], 0.2, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[0].r[2], 0.0, TEST_PREC);

  CHECK_CLOSE(updLocalFrameSet[0].E(0,0), 1.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[0].E(0,1), 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[0].E(0,2), 0.0, TEST_PREC);

  CHECK_CLOSE(updLocalFrameSet[0].E(1,0), 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[0].E(1,1), 1.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[0].E(1,2), 0.0, TEST_PREC);

  CHECK_CLOSE(updLocalFrameSet[0].E(2,0), 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[0].E(2,1), 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[0].E(2,2), 1.0, TEST_PREC);


  CHECK(std::strcmp("Pocket_R",updLocalFrameSet[1].name.c_str())==0);
  CHECK(updLocalFrameSet[1].body_id == thighRightId);
  CHECK_CLOSE(updLocalFrameSet[1].r[0], 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[1].r[1],-0.2, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[1].r[2], 0.0, TEST_PREC);

  CHECK_CLOSE(updLocalFrameSet[1].E(0,0), 1.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[1].E(0,1), 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[1].E(0,2), 0.0, TEST_PREC);

  CHECK_CLOSE(updLocalFrameSet[1].E(1,0), 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[1].E(1,1), 1.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[1].E(1,2), 0.0, TEST_PREC);

  CHECK_CLOSE(updLocalFrameSet[1].E(2,0), 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[1].E(2,1), 0.0, TEST_PREC);
  CHECK_CLOSE(updLocalFrameSet[1].E(2,2), 1.0, TEST_PREC);


}
TEST(LoadPoints)
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  std::vector< Point > updPointSet;
  bool pointsLoaded = LuaModelReadPoints(modelFile.c_str(),&model,
                                         updPointSet,false);
  CHECK(updPointSet.size()==4);
  
  unsigned int bodyId = model.GetBodyId("foot_right");

  CHECK( strcmp( updPointSet[0].name.c_str(),"Heel_Medial_L")      == 0);
  CHECK( strcmp( updPointSet[1].name.c_str(),"Heel_Lateral_L")     == 0);
  CHECK( strcmp( updPointSet[2].name.c_str(),"ForeFoot_Medial_L")  == 0);
  CHECK( strcmp( updPointSet[3].name.c_str(),"ForeFoot_Lateral_L") == 0);

  CHECK( updPointSet[0].body_id == bodyId );
  CHECK( updPointSet[1].body_id == bodyId );
  CHECK( updPointSet[2].body_id == bodyId );
  CHECK( updPointSet[3].body_id == bodyId );

  CHECK_CLOSE(updPointSet[0].point_local[0], -0.080, TEST_PREC);
  CHECK_CLOSE(updPointSet[0].point_local[1], -0.042, TEST_PREC);
  CHECK_CLOSE(updPointSet[0].point_local[2], -0.091, TEST_PREC);

  CHECK_CLOSE(updPointSet[1].point_local[0], -0.080, TEST_PREC);
  CHECK_CLOSE(updPointSet[1].point_local[1],  0.042, TEST_PREC);
  CHECK_CLOSE(updPointSet[1].point_local[2], -0.091, TEST_PREC);
  
  CHECK_CLOSE(updPointSet[2].point_local[0],  0.181788, TEST_PREC);
  CHECK_CLOSE(updPointSet[2].point_local[1], -0.054000, TEST_PREC);
  CHECK_CLOSE(updPointSet[2].point_local[2], -0.091000, TEST_PREC);

  CHECK_CLOSE(updPointSet[3].point_local[0],  0.181788, TEST_PREC);
  CHECK_CLOSE(updPointSet[3].point_local[1],  0.054000, TEST_PREC);
  CHECK_CLOSE(updPointSet[3].point_local[2], -0.091000, TEST_PREC);
}

TEST(LoadConstrainedLuaModel)
{
  RigidBodyDynamics::Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/sampleconstrainedmodel.lua");

  std::vector<std::string> constraintSetNames = 
    LuaModelGetConstraintSetNames(modelFile.c_str());
  std::vector<RigidBodyDynamics::ConstraintSet> constraintSets;

  constraintSets.resize(constraintSetNames.size());
  for(unsigned int i=0; i<constraintSetNames.size();++i){
    constraintSets[i] = ConstraintSet();
  }


  bool modelLoaded = LuaModelReadFromFileWithConstraints( modelFile.c_str(),
                                                          &model,
                                                          constraintSets,
                                                          constraintSetNames,
                                                          false);

  CHECK(modelLoaded);

  unsigned int baseId = model.GetBodyId("base");
  unsigned int rootId = model.GetBodyId("ROOT");
  unsigned int l12Id = model.GetBodyId("l12");
  unsigned int l22Id = model.GetBodyId("l22");

  unsigned int groupIndex = 0;


  // Contact Constraint X
  groupIndex = constraintSets[0].getGroupIndexByName("contactBaseX");
  CHECK(constraintSets[0].getGroupSize(groupIndex) == 1);
  CHECK(constraintSets[0].getGroupType(groupIndex) == ConstraintTypeContact);
  unsigned int userDefinedId = constraintSets[0].getGroupId(groupIndex);
  CHECK(userDefinedId == 2);

  std::vector<unsigned int> bodyIds =
      constraintSets[0].contactConstraints[0]->getBodyIds();
  CHECK(bodyIds.size() == 2);
  CHECK(bodyIds[0] == baseId);
  CHECK(bodyIds[1] == rootId);

  std::vector< Vector3d > normalVectors =
      constraintSets[0].contactConstraints[0]->getConstraintNormalVectors();

  // (all contact constraints between the same pair of bodies are grouped)
  CHECK(normalVectors.size()==1);
  CHECK_CLOSE(normalVectors[0][0], 1., TEST_PREC);
  CHECK_CLOSE(normalVectors[0][1], 0., TEST_PREC);
  CHECK_CLOSE(normalVectors[0][2], 0., TEST_PREC);

  //MM 17/5/2020
  //Contract constraints currently do not have the Baumgarte stabilization
  //parameter exposed: these kinds of constraints are so well numerically
  //behaved that this kind of constraint stabilization is normally not required.
  CHECK(constraintSets[0].isBaumgarteStabilizationEnabled(groupIndex)==false);

  // Contact Constraint YZ
  groupIndex = constraintSets[0].getGroupIndexByName("contactBaseYZ");
  CHECK(constraintSets[0].getGroupSize(groupIndex) == 2);
  CHECK(constraintSets[0].getGroupType(groupIndex) == ConstraintTypeContact);
  userDefinedId = constraintSets[0].getGroupId(groupIndex);
  CHECK(userDefinedId == 3);

  normalVectors =
        constraintSets[0].contactConstraints[1]->getConstraintNormalVectors();
  CHECK(normalVectors.size()==2);

  CHECK_CLOSE(normalVectors[0][0], 0., TEST_PREC);
  CHECK_CLOSE(normalVectors[0][1], 1., TEST_PREC);
  CHECK_CLOSE(normalVectors[0][2], 0., TEST_PREC);

  CHECK_CLOSE(normalVectors[1][0], 0., TEST_PREC);
  CHECK_CLOSE(normalVectors[1][1], 0., TEST_PREC);
  CHECK_CLOSE(normalVectors[1][2], 1., TEST_PREC);

  //MM 17/5/2020
  //Contract constraints currently do not have the Baumgarte stabilization
  //parameter exposed: these kinds of constraints are so well numerically
  //behaved that this kind of constraint stabilization is normally not required.
  CHECK(constraintSets[0].isBaumgarteStabilizationEnabled(groupIndex) == false);

  // Loop Constraint X
  groupIndex = constraintSets[0].getGroupIndexByName("loopL12L22Tx");

  CHECK(constraintSets[0].getGroupSize(groupIndex) == 1);
  CHECK(constraintSets[0].getGroupType(groupIndex) == ConstraintTypeLoop);
  userDefinedId = constraintSets[0].getGroupId(groupIndex);
  CHECK(userDefinedId == 1);

  bodyIds = constraintSets[0].loopConstraints[0]->getBodyIds();
  CHECK(bodyIds.size()==2);
  CHECK(bodyIds[0] == l12Id);
  CHECK(bodyIds[1] == l22Id);

  //Loop constraints often require stabilization so the Baumgarte
  //stabilization parameters are exposed
  CHECK(constraintSets[0].isBaumgarteStabilizationEnabled(groupIndex) == false);

  std::vector< SpatialVector > axis =
    constraintSets[0].loopConstraints[0]->getConstraintAxes();
  CHECK(axis.size()==1);
  CHECK_CLOSE( axis[0][0], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][1], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][2], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][3], 1., TEST_PREC);
  CHECK_CLOSE( axis[0][4], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][5], 0., TEST_PREC);

  // Loop Constraint Y
  groupIndex = constraintSets[0].getGroupIndexByName("loopL12L22Ty");
  CHECK(constraintSets[0].getGroupSize(groupIndex) == 1);
  CHECK(constraintSets[0].getGroupType(groupIndex) == ConstraintTypeLoop);
  userDefinedId = constraintSets[0].getGroupId(groupIndex);
  CHECK(userDefinedId == 2);

  axis =constraintSets[0].loopConstraints[1]->getConstraintAxes();
  CHECK(axis.size()==1);

  CHECK_CLOSE( axis[0][0], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][1], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][2], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][3], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][4], 1., TEST_PREC);
  CHECK_CLOSE( axis[0][5], 0., TEST_PREC);

  //Loop constraints often require stabilization so the Baumgarte
  //stabilization parameters are exposed
  CHECK(constraintSets[0].isBaumgarteStabilizationEnabled(groupIndex) == true);


  // Contact Constraint XYZ
  groupIndex = constraintSets[1].getGroupIndexByName("contactBaseXYZ");
  CHECK(constraintSets[1].getGroupSize(groupIndex) == 3);
  CHECK(constraintSets[1].getGroupType(groupIndex) == ConstraintTypeContact);
  CHECK(constraintSets[1].getGroupId(groupIndex) == 2);

  bodyIds = constraintSets[1].contactConstraints[0]->getBodyIds();
  CHECK(bodyIds.size()==2);
  CHECK(bodyIds[0] == baseId);
  CHECK(bodyIds[1] == rootId);

  normalVectors =
      constraintSets[1].contactConstraints[0]->getConstraintNormalVectors();
  CHECK(normalVectors.size()==3);
  CHECK_CLOSE(normalVectors[0][0], 1., TEST_PREC);
  CHECK_CLOSE(normalVectors[0][1], 0., TEST_PREC);
  CHECK_CLOSE(normalVectors[0][2], 0., TEST_PREC);

  CHECK_CLOSE(normalVectors[1][0], 0., TEST_PREC);
  CHECK_CLOSE(normalVectors[1][1], 1., TEST_PREC);
  CHECK_CLOSE(normalVectors[1][2], 0., TEST_PREC);

  CHECK_CLOSE(normalVectors[2][0], 0., TEST_PREC);
  CHECK_CLOSE(normalVectors[2][1], 0., TEST_PREC);
  CHECK_CLOSE(normalVectors[2][2], 1., TEST_PREC);

  CHECK(constraintSets[1].isBaumgarteStabilizationEnabled(groupIndex) == false);

  // Loop Constraint Tx Ty
  groupIndex = constraintSets[1].getGroupIndexByName("loopL12L22TxTy");
  CHECK(constraintSets[1].getGroupSize(groupIndex) == 2);
  CHECK(constraintSets[1].getGroupType(groupIndex) == ConstraintTypeLoop);
  CHECK(constraintSets[1].getGroupId(groupIndex) == 1);

  bodyIds = constraintSets[1].loopConstraints[0]->getBodyIds();
  CHECK(bodyIds.size()==2);
  CHECK(bodyIds[0] == l12Id);
  CHECK(bodyIds[1] == l22Id);

  axis =
    constraintSets[1].loopConstraints[0]->getConstraintAxes();
  CHECK(axis.size()==2);
  CHECK_CLOSE( axis[0][0], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][1], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][2], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][3], 1., TEST_PREC);
  CHECK_CLOSE( axis[0][4], 0., TEST_PREC);
  CHECK_CLOSE( axis[0][5], 0., TEST_PREC);

  CHECK_CLOSE( axis[1][0], 0., TEST_PREC);
  CHECK_CLOSE( axis[1][1], 0., TEST_PREC);
  CHECK_CLOSE( axis[1][2], 0., TEST_PREC);
  CHECK_CLOSE( axis[1][3], 0., TEST_PREC);
  CHECK_CLOSE( axis[1][4], 1., TEST_PREC);
  CHECK_CLOSE( axis[1][5], 0., TEST_PREC);

  CHECK(constraintSets[1].isBaumgarteStabilizationEnabled(groupIndex) == false);


}

#ifdef RBDL_BUILD_ADDON_MUSCLE
TEST(LoadMuscleTorqueGenerators)
{
  RigidBodyDynamics::Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodelwithtorquemuscles.lua");



  bool modelLoaded = LuaModelReadFromFile( modelFile.c_str(),
                                           &model,
                                           false);

  CHECK(modelLoaded);

  HumanMetaData humanData;
  bool humanDataLoaded =
      LuaModelReadHumanMetaData(modelFile.c_str(),humanData,false);
  CHECK(humanDataLoaded);

  CHECK(std::fabs(humanData.age - 35.0) < TEST_PREC);
  CHECK(std::fabs(humanData.height - 1.73) < TEST_PREC);
  CHECK(std::fabs(humanData.height - 1.73) < TEST_PREC);
  CHECK(std::strcmp(humanData.age_group.c_str(),"Young18To25")==0);
  CHECK(std::strcmp(humanData.gender.c_str(),"male")==0);


  std::vector < Muscle::Millard2016TorqueMuscle > mtgSet;
  std::vector< Millard2016TorqueMuscleInfo > mtgInfoSet;

  bool torqueMusclesLoaded = LuaModelReadMillard2016TorqueMuscleSets(
        modelFile.c_str(),model,humanData,mtgSet,mtgInfoSet,false);

  CHECK(torqueMusclesLoaded);
  CHECK(mtgSet.size() == 6);
  CHECK(mtgInfoSet.size() == 6);

  unsigned int i=0;

  //Check that the data is being loaded as it is written in the file for the
  //full right leg
  i=0;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"HipExtension_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - (-1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - ( 1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"thigh_right")== 0);
  CHECK(mtgInfoSet[i].joint_index - 1 == 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].passive_element_torque_scale) < TEST_PREC);
  i=1;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"HipFlexion_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - (-1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - (-1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"thigh_right")== 0);
  CHECK(mtgInfoSet[i].joint_index - 1 == 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].passive_element_torque_scale) < TEST_PREC);
  i=2;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"KneeExtension_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - ( 1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - (-1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"shank_right")== 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].passive_element_torque_scale) < TEST_PREC);
  i=3;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"KneeFlexion_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - ( 1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - ( 1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"shank_right")== 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].passive_element_torque_scale) < TEST_PREC);
  i=4;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"AnkleExtension_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - (-1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - ( 1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"foot_right")== 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].passive_element_torque_scale) < TEST_PREC);
  i=5;
  CHECK(std::strcmp(mtgInfoSet[i].name.c_str(),"AnkleFlexion_R")==0);
  CHECK(std::fabs(mtgInfoSet[i].angle_sign  - (-1)) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].torque_sign - (-1)) < TEST_PREC);
  CHECK(std::strcmp(mtgInfoSet[i].body.c_str(),"foot_right")== 0);
  CHECK(std::fabs(mtgInfoSet[i].activation_time_constant   - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].deactivation_time_constant - 0.05) < TEST_PREC);
  CHECK(std::fabs(mtgInfoSet[i].passive_element_torque_scale) < TEST_PREC);

}

#endif

int main (int argc, char *argv[])
{
    if(argc < 2){
      std::cerr << "The path to the rbdl/addons/luamodel directory must be "
                << "passed as an argument" << std::endl;
      assert(0);
      abort();                
    }

    //cout << argv[1] << endl;
    rbdlSourcePath.assign(argv[1]);
    return UnitTest::RunAllTests ();
}


