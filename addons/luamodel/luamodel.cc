#include "rbdl/rbdl.h"
#include "rbdl/rbdl_errors.h"
#include "luamodel.h"

#include <iostream>
#include <map>

#include "luatables.h"
#include "luatypes.h"

extern "C"
{
#include <lua5.1/lua.h>
#include <lua5.1/lauxlib.h>
#include <lua5.1/lualib.h>
}

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;



namespace RigidBodyDynamics
{

namespace Addons
{

//==============================================================================
bool LuaModelReadFromTable (LuaTable &model_table, Model *model, bool verbose);

//==============================================================================
bool LuaModelReadConstraintsFromTable (
  LuaTable &model_table,
  Model *model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
);

typedef map<string, unsigned int> StringIntMap;
StringIntMap body_table_id_map;

//==============================================================================
RBDL_DLLAPI
bool LuaModelReadFromLuaState (lua_State* L, Model* model, bool verbose)
{
  assert (model);

  LuaTable model_table = LuaTable::fromLuaState (L);

  return LuaModelReadFromTable (model_table, model, verbose);
}

//==============================================================================
RBDL_DLLAPI
bool LuaModelReadFromFile (const char* filename, Model* model, bool verbose)
{
  if(!model) {
    throw Errors::RBDLError("Model not provided.");
  }

  LuaTable model_table = LuaTable::fromFile (filename);

  return LuaModelReadFromTable (model_table, model, verbose);
}

//==============================================================================
RBDL_DLLAPI
std::vector<std::string> LuaModelGetConstraintSetNames(const char* filename)
{
  std::vector<std::string> result;

  LuaTable model_table = LuaTable::fromFile (filename);

  std::vector<LuaKey> constraint_keys;
  if (model_table["constraint_sets"].exists()) {
    constraint_keys = model_table["constraint_sets"].keys();
  }

  if (constraint_keys.size() == 0) {
    return result;
  }

  for (size_t ci = 0; ci < constraint_keys.size(); ++ci) {
    if (constraint_keys[ci].type != LuaKey::String) {
      throw Errors::RBDLFileParseError("Invalid constraint found in model.constraint_sets: no constraint set name was specified!");
    }

    result.push_back(constraint_keys[ci].string_value);
  }

  return result;
}

//==============================================================================
RBDL_DLLAPI
bool LuaModelReadFromFileWithConstraints (
  const char* filename,
  Model* model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
)
{
  if(!model) {
    throw Errors::RBDLError("Model not provided.");
  }
  if(constraint_sets.size() != constraint_set_names.size()) {
    throw Errors::RBDLFileParseError("Number of constraint sets different from the number of constraint set names.");
  }

  LuaTable model_table = LuaTable::fromFile (filename);
  bool modelLoaded = LuaModelReadFromTable (model_table, model, verbose);
  bool constraintsLoaded = LuaModelReadConstraintsFromTable (model_table, model
                           , constraint_sets, constraint_set_names, verbose);
  for(size_t i = 0; i < constraint_sets.size(); ++i) {
    constraint_sets[i].Bind(*model);
  }

  return modelLoaded && constraintsLoaded;
}

//==============================================================================
bool LuaModelReadFromTable (LuaTable &model_table, Model* model, bool verbose)
{
  if (model_table["gravity"].exists()) {
    model->gravity = model_table["gravity"].get<Vector3d>();

    if (verbose) {
      cout << "gravity = " << model->gravity.transpose() << endl;
    }
  }

  int frame_count = model_table["frames"].length();

  body_table_id_map["ROOT"] = 0;

  for (int i = 1; i <= frame_count; i++) {
    if (!model_table["frames"][i]["parent"].exists()) {
      throw Errors::RBDLError("Parent not defined for frame ");
    }

    string body_name = model_table["frames"][i]["name"].getDefault<string>("");
    string parent_name = model_table["frames"][i]["parent"].get<string>();
    unsigned int parent_id = body_table_id_map[parent_name];

    SpatialTransform joint_frame
      = model_table["frames"][i]["joint_frame"].getDefault(SpatialTransform());
    Joint joint
      = model_table["frames"][i]["joint"].getDefault(Joint(JointTypeFixed));
    Body body = model_table["frames"][i]["body"].getDefault<Body>(Body());

    unsigned int body_id
      = model->AddBody (parent_id, joint_frame, joint, body, body_name);
    body_table_id_map[body_name] = body_id;

    if (verbose) {
      cout << "==== Added Body ====" << endl;
      cout << "  body_name  : " << body_name << endl;
      cout << "  body id	: " << body_id << endl;
      cout << "  parent_id  : " << parent_id << endl;
      cout << "  joint dofs : " << joint.mDoFCount << endl;
      for (unsigned int j = 0; j < joint.mDoFCount; j++) {
        cout << "	" << j << ": " << joint.mJointAxes[j].transpose() << endl;
      }
      cout << "  joint_frame: " << joint_frame << endl;
    }
  }

  return true;
}

//==============================================================================
bool LuaModelReadConstraintsFromTable (
  LuaTable &model_table,
  Model *model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
)
{
  std::string conName;

  std::vector< Vector3d > normalSets;
  MatrixNd normalSetsMatrix;
  Vector3d normal;

  MatrixNd axisSetsMatrix;
  SpatialVector axis;
  std::vector< SpatialVector > axisSets;

  for(size_t i = 0; i < constraint_set_names.size(); ++i) {
    conName = constraint_set_names[i];
    if (verbose) {
      std::cout << "==== Constraint Set: " << conName << std::endl;
    }

    if(!model_table["constraint_sets"][conName.c_str()]
        .exists()) {
      ostringstream errormsg;
      errormsg << "Constraint set not existing: " << conName << "." << endl;
      throw Errors::RBDLFileParseError(errormsg.str());
    }

    size_t num_constraints = model_table["constraint_sets"]
                             [conName.c_str()]
                             .length();

    for(size_t ci = 0; ci < num_constraints; ++ci) {
      if (verbose) {
        std::cout << "== Constraint " << ci << "/" << num_constraints
                  << " ==" << std::endl;
      }

      if(!model_table["constraint_sets"]
          [conName.c_str()][ci + 1]["constraint_type"].exists()) {
        throw Errors::RBDLFileParseError("constraint_type not specified.\n");
      }

      string constraintType = model_table["constraint_sets"]
                              [conName.c_str()][ci + 1]["constraint_type"]
                              .getDefault<string>("");
      std::string constraint_name =
        model_table["constraint_sets"][conName.c_str()]
        [ci + 1]["name"].getDefault<string>("");

      //========================================================================
      //Contact
      //========================================================================
      if(constraintType == "contact") {
        if(!model_table["constraint_sets"][conName.c_str()]
            [ci + 1]["body"].exists()) {
          throw Errors::RBDLFileParseError("body not specified.\n");
        }

        unsigned int constraint_user_id=std::numeric_limits<unsigned int>::max();
        if(model_table["constraint_sets"][conName.c_str()]
            [ci + 1]["id"].exists()) {
          constraint_user_id = unsigned(int(
                                          model_table["constraint_sets"][conName.c_str()]
                                          [ci + 1]["id"].getDefault<double>(0.)));
        }


        unsigned int bodyId = model->GetBodyId(model_table["constraint_sets"]
                                               [conName.c_str()][ci + 1]["body"]
                                               .getDefault<string>("").c_str());

        Vector3d bodyPoint = model_table["constraint_sets"]
                             [conName.c_str()][ci + 1]
                             ["point"].getDefault<Vector3d>(Vector3d::Zero());

        normalSets.resize(0);
        normalSetsMatrix.resize(1,1);

        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["normal_sets"].exists()) {

          normalSetsMatrix =
            model_table["constraint_sets"][conName.c_str()]
            [ci + 1]["normal_sets"].getDefault< MatrixNd >(MatrixNd::Zero(1,1));

          if(normalSetsMatrix.cols() != 3 ) {
            std::ostringstream errormsg;
            errormsg << "The normal_sets field must be m x 3, the one read for "
                     << conName.c_str() << " has an normal_sets of size "
                     << normalSetsMatrix.rows() << " x " << normalSetsMatrix.cols()
                     << ". In addition the normal_sets field should resemble:"
                     << endl;
            errormsg << "  normal_sets = {{1.,0.,0.,}, " << endl;
            errormsg << "                 {0.,1.,0.,},}, " << endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }


          for(unsigned int r=0; r<normalSetsMatrix.rows(); ++r) {
            for(unsigned int c=0; c<normalSetsMatrix.cols(); ++c) {
              normal[c] = normalSetsMatrix(r,c);
            }
            normalSets.push_back(normal);
          }

        } else if(model_table["constraint_sets"][conName.c_str()]
                  [ci + 1]["normal"].exists()) {

          normal = model_table["constraint_sets"]
                   [conName.c_str()][ci + 1]
                   ["normal"].getDefault<Vector3d>(Vector3d::Zero());
          normalSets.push_back(normal);

        } else {
          std::ostringstream errormsg;
          errormsg << "The ContactConstraint must have either normal_sets field "
                   "(which is a m x 3 matrix) or an normal field. Neither of "
                   "these fields was found in "
                   << conName.c_str() << endl;
          throw Errors::RBDLFileParseError(errormsg.str());
        }

        std::string contactName = model_table["constraint_sets"]
                                  [conName.c_str()][ci + 1]
                                  ["name"].getDefault<string>("").c_str();

        for(unsigned int c=0; c<normalSets.size(); ++c) {
          constraint_sets[i].AddContactConstraint(bodyId,
                                                  bodyPoint,
                                                  normalSets[c],
                                                  contactName.c_str(),
                                                  constraint_user_id);
        }


        if(verbose) {
          cout  << "  type = contact" << endl;
          cout  << "  name = " << constraint_name << std::endl;
          cout  << "  body = "
                << model->GetBodyName(bodyId) << endl;
          cout  << "  body point = "
                << bodyPoint.transpose()
                << endl;
          cout  << "  world normal = " << endl;
          for(unsigned int c=0; c<normalSets.size(); ++c) {
            cout << normalSets[c].transpose() << endl;
          }
          cout << "  normal acceleration = DEPRECATED::IGNORED" << endl;
        }

        //========================================================================
        //Loop
        //========================================================================
      } else if(constraintType == "loop") {
        if(!model_table["constraint_sets"][conName.c_str()]
            [ci + 1]["predecessor_body"].exists()) {
          throw Errors::RBDLFileParseError("predecessor_body not specified.\n");
        }
        if(!model_table["constraint_sets"][conName.c_str()]
            [ci + 1]["successor_body"].exists()) {
          throw Errors::RBDLFileParseError("successor_body not specified.\n");
        }

        // Add the loop constraint as a non-stablized constraint and compute
        // and set the actual stabilization cofficients for the Baumgarte
        // stabilization afterwards if enabled.
        unsigned int constraint_id;

        bool enable_stabilization =
          model_table["constraint_sets"][conName.c_str()][ci + 1]
          ["enable_stabilization"].getDefault<bool>(false);
        double stabilization_parameter = 0.1;

        if (enable_stabilization) {
          stabilization_parameter =
            model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["stabilization_parameter"].getDefault<double>(0.1);
          if (stabilization_parameter <= 0.0) {
            std::stringstream errormsg;
            errormsg  << "Invalid stabilization parameter: "
                      << stabilization_parameter
                      << " must be > 0.0" << std::endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }
        }

        axisSetsMatrix.resize(1,1);
        axisSets.resize(0);
        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["axis_sets"].exists()) {
          axisSetsMatrix =
            model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["axis_sets"].getDefault< MatrixNd >( MatrixNd::Zero(1,1));

          if(axisSetsMatrix.cols() != 6 ) {
            std::stringstream errormsg;
            errormsg  << "The axis_sets field must be m x 6, the one read for "
                      << conName.c_str() << " has an axis_sets of size "
                      << axisSetsMatrix.rows() << " x " << axisSetsMatrix.cols()
                      << ". In addition the axis_sets field should resemble:"
                      << endl;
            errormsg  << "  axis_sets = {{0.,0.,0.,1.,0.,0.,}, " <<endl;
            errormsg  << "               {0.,0.,0.,0.,1.,0.,},}, " <<endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }

          for(unsigned int r=0; r<axisSetsMatrix.rows(); ++r) {
            for(unsigned int c=0; c<axisSetsMatrix.cols(); ++c) {
              axis[c] = axisSetsMatrix(r,c);
            }
            axisSets.push_back(axis);
          }

        } else if(model_table["constraint_sets"][conName.c_str()][ci + 1]
                  ["axis"].exists()) {
          axis = model_table["constraint_sets"][conName.c_str()][ci + 1]
                 ["axis"].getDefault< SpatialVector >( SpatialVector::Zero());

          axisSets.push_back(axis);

        } else {
          std::stringstream errormsg;
          errormsg  << "The LoopConstraint must have either axis_sets field "
                    "(which is a m x 6 matrix) or an axis field. Neither of "
                    "these fields was found in "
                    << conName.c_str() << endl;
          throw Errors::RBDLFileParseError(errormsg.str());
        }

        unsigned int constraint_user_id=std::numeric_limits<unsigned int>::max();
        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["id"].exists()) {
          constraint_user_id = unsigned(int(
                                          model_table["constraint_sets"][conName.c_str()]
                                          [ci + 1]["id"].getDefault<double>(0.)));
        }

        unsigned int idPredecessor =
          model->GetBodyId(model_table["constraint_sets"]
                           [conName.c_str()][ci + 1]["predecessor_body"]
                           .getDefault<string>("").c_str());

        unsigned int idSuccessor =
          model->GetBodyId(model_table["constraint_sets"]
                           [conName.c_str()][ci + 1]["successor_body"]
                           .getDefault<string>("").c_str());

        SpatialTransform Xp =
          model_table["constraint_sets"][conName.c_str()]
          [ci + 1]["predecessor_transform"]
          .getDefault<SpatialTransform>(SpatialTransform());

        SpatialTransform Xs =
          model_table["constraint_sets"][conName.c_str()]
          [ci + 1]["successor_transform"]
          .getDefault<SpatialTransform>(SpatialTransform());

        for(unsigned int r=0; r<axisSets.size(); ++r) {
          constraint_id = constraint_sets[i].AddLoopConstraint(
                            idPredecessor
                            , idSuccessor
                            , Xp
                            , Xs
                            , axisSets[r]
                            , enable_stabilization
                            , stabilization_parameter
                            , constraint_name.c_str()
                            , constraint_user_id);
        }

        if(verbose) {
          cout << "  type = loop" << endl;
          cout << "  name = " << constraint_name << std::endl;
          cout << "  predecessor body = "
               << model->GetBodyName(idPredecessor)<< endl;
          cout << "  successor body = "
               << model->GetBodyName(idSuccessor) << endl;
          cout << "  predecessor body transform = " << endl
               << Xp << endl;
          cout << "  successor body transform = " << endl
               << Xs << endl;
          cout << "  constraint axis (in predecessor frame) = " << endl;
          for(unsigned int c=0; c<axisSets.size(); ++c) {
            cout << axisSets[c].transpose() << endl;
          }
          cout << "  enable_stabilization = " << enable_stabilization
               << endl;
          if (enable_stabilization) {
            cout << "  stabilization_parameter = " << stabilization_parameter
                 << endl;
          }
          cout << "  constraint name = "
               << constraint_name.c_str() << endl;
        }
      } else {
        ostringstream errormsg;
        errormsg << "Invalid constraint type: " << constraintType << endl;
        throw Errors::RBDLFileParseError(errormsg.str());
      }
    }
  }

  return true;
}


bool LuaModelReadPoints (
      const char* filename,
      RigidBodyDynamics::Model *model,
      std::vector<Point>& updPointSet,
      bool verbose)
{

  LuaTable luaTable       = LuaTable::fromFile (filename);
  updPointSet.clear();
  unsigned int pointCount = unsigned(int(luaTable["points"].length()));

  if(pointCount > 0){
    updPointSet.resize(pointCount);
    Point point;

    for (unsigned int i = 1; i <= pointCount; ++i) {

      point = luaTable["points"][i];

      point.body_id   = model->GetBodyId (point.body_name.c_str());
      updPointSet[i-1]   = point;

      if (verbose) {
        cout  << "Point '"           << updPointSet[i-1].name
              << "' (PointName = "   << updPointSet[i-1].name << ")"    << endl;
        cout  << "  body        = "  << updPointSet[i-1].body_name
              << " (id = "           << updPointSet[i-1].body_id << ")" << endl;
        cout  << "  point_local  = '"       << updPointSet[i-1].point_local.transpose()
                                     << endl;
      }
    }
  }
  return true;  
}

bool LuaModelReadMotionCaptureMarkers (
      const char* filename,
      RigidBodyDynamics::Model *model,
      std::vector<Point>& updPointSet,
      bool verbose)
{

  LuaTable luaTable       = LuaTable::fromFile (filename);
  updPointSet.clear();

  if(luaTable["frames"].exists()){
    unsigned int frameCount = luaTable["frames"].length();
    std::vector<LuaKey> marker_keys;
    Point point;
    std::string body_name;
    unsigned int body_id;
    for(unsigned int i=1; i<frameCount; ++i){
      if(luaTable["frames"][i]["markers"].exists()){

        body_name = luaTable["frames"][i]["name"].getDefault<string>("");
        body_id = model->GetBodyId(body_name.c_str());
        marker_keys = luaTable["frames"][i]["markers"].keys();

        for(unsigned int j=0; j < marker_keys.size(); ++j){
          if (marker_keys[j].type != LuaKey::String) {
            throw Errors::RBDLFileParseError(
                    "Invalid marker found: missing name!");
          }
          point.name      = marker_keys[j].string_value;
          point.body_name = body_name;
          point.body_id   = body_id;
          point.point_local = luaTable["frames"][i]["markers"][point.name.c_str()]
                              .getDefault<Vector3d>(Vector3d::Zero());
          updPointSet.push_back(point);
        }
      }
    }
  }

  return true;
}


}
}
