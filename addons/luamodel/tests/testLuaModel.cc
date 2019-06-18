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


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Addons;

using namespace std;

std::string rbdlSourcePath;
   
TEST(LoadLuaModel)
{
  Model model;
  std::string modelFile = rbdlSourcePath;
  modelFile.append("/samplemodel.lua");
  bool modelLoaded = LuaModelReadFromFile(modelFile.c_str(), &model, false);
  CHECK(modelLoaded);
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


