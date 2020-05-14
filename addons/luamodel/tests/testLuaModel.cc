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

using namespace RigidBodyDynamics;
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
}

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


