#include "rbdl/rbdl.h"
#include "rbdl/rbdl_errors.h"
#include "luamodel.h"

#include <iostream>
#include <iomanip>
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

#ifdef RBDL_BUILD_ADDON_MUSCLE
#include "../muscle/Millard2016TorqueMuscle.h"
using namespace RigidBodyDynamics::Addons::Muscle;
#endif


namespace RigidBodyDynamics
{

namespace Addons
{

//==============================================================================
bool LuaModelReadFromTable (LuaTable &model_table, Model *model, bool verbose);

//==============================================================================
bool LuaModelReadConstraintsFromTable (
  LuaTable &model_table,
  const Model *model,
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
bool LuaModelReadLocalFrames (
      LuaTable &model_table,
      const RigidBodyDynamics::Model *model,
      std::vector<LocalFrame>& upd_local_frame_set,
      bool verbose);

RBDL_DLLAPI
bool LuaModelReadLocalFrames (
      const char* filename,
      const RigidBodyDynamics::Model *model,
      std::vector<LocalFrame>& upd_local_frame_set,
      bool verbose)
{
  LuaTable model_table       = LuaTable::fromFile (filename);
  return LuaModelReadLocalFrames(model_table,model,upd_local_frame_set,verbose);
}

//==============================================================================
bool LuaModelReadPoints (
      LuaTable &model_table,
      const RigidBodyDynamics::Model *model,
      std::vector<Point>& upd_point_set,
      bool verbose);

RBDL_DLLAPI
bool LuaModelReadPoints (
      const char* filename,
      const RigidBodyDynamics::Model *model,
      std::vector<Point>& upd_point_set,
      bool verbose)
{
  LuaTable model_table       = LuaTable::fromFile (filename);
  return LuaModelReadPoints(model_table,model,upd_point_set,verbose);
}

//==============================================================================
RBDL_DLLAPI
bool LuaModelReadFromFile (const char* filename, Model* upd_model, bool verbose)
{
  if(!upd_model) {
    throw Errors::RBDLError("Model not provided.");
  }

  LuaTable model_table = LuaTable::fromFile (filename);
  return LuaModelReadFromTable (model_table, upd_model, verbose);
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
      throw Errors::RBDLFileParseError(
            "Invalid constraint found in model.constraint_sets: "
            "no constraint set name was specified!");
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
    throw Errors::RBDLFileParseError(
          "Number of constraint sets different from"
          " the number of constraint set names.");
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
  const Model *model,
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

  std::vector< Point > pointSet;
  bool pointSetLoaded = LuaModelReadPoints(model_table,model,pointSet,verbose);

  std::vector< LocalFrame > localFrameSet;
  bool localFramesLoaded =
      LuaModelReadLocalFrames(model_table,model,localFrameSet,verbose);

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


      //========================================================================
      //Contact
      //========================================================================
      if(constraintType == "contact") {

        unsigned int constraint_user_id =
            std::numeric_limits<unsigned int>::max();

        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
                      ["id"].exists()) {
          constraint_user_id = unsigned(int(
            model_table["constraint_sets"][conName.c_str()][ci + 1]
                       ["id"].getDefault<double>(0.)));
        }

        //Go get the body id and the local coordinates of the point:
        //    If a named point is given, go through the point set
        //    Otherwise look for the explicit fields
        unsigned int bodyId;
        Vector3d bodyPoint;

        if(model_table["constraint_sets"][conName.c_str()][ci+1]
           ["point_name"].exists()){
          std::string pointName = model_table["constraint_sets"]
              [conName.c_str()][ci+1]["point_name"].getDefault<string>("");
          bool pointFound = false;
          unsigned int pi=0;
          while(pi < pointSet.size() && pointFound == false){
            if(std::strcmp(pointSet[pi].name.c_str(),pointName.c_str())==0){
              pointFound = true;
              bodyId = pointSet[pi].body_id;
              bodyPoint = pointSet[pi].point_local;
            }
          }
          if(pointFound == false){
            ostringstream errormsg;
            errormsg << "Could not find a point with the name: " << pointName
                     << " which is used in constraint set " << constraint_name
                     << endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }
        }else{
          if(!model_table["constraint_sets"][conName.c_str()]
              [ci + 1]["body"].exists()) {
            throw Errors::RBDLFileParseError("body not specified.\n");
          }
          bodyId = model->GetBodyId(model_table["constraint_sets"]
                                               [conName.c_str()][ci + 1]["body"]
                                               .getDefault<string>("").c_str());

          bodyPoint = model_table["constraint_sets"]
                               [conName.c_str()][ci + 1]
                               ["point"].getDefault<Vector3d>(Vector3d::Zero());
        }

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
                     << normalSetsMatrix.rows() << " x "
                     << normalSetsMatrix.cols()
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

        unsigned int constraint_user_id=std::numeric_limits<unsigned int>::max();
        if(model_table["constraint_sets"][conName.c_str()][ci + 1]
            ["id"].exists()) {
          constraint_user_id =
              unsigned(int(model_table["constraint_sets"][conName.c_str()]
                           [ci + 1]["id"].getDefault<double>(0.)));
        }

        //Get the local frames that this constraint will be applied to
        // If named local frames have been given, use them
        // Otherwise look for the individual fields
        unsigned int idPredecessor;
        unsigned int idSuccessor;
        SpatialTransform Xp;
        SpatialTransform Xs;

        if(model_table["constraint_sets"]
           [conName.c_str()][ci + 1]["predecessor_local_frame"].exists()){

          std::string localFrameName = model_table["constraint_sets"]
              [conName.c_str()][ci + 1]["predecessor_local_frame"]
              .getDefault<string>("");
          bool frameFound = false;
          unsigned int fi=0;
          while(fi < localFrameSet.size() && frameFound == false){
            if(std::strcmp(localFrameSet[fi].name.c_str(),
                           localFrameName.c_str())==0){
              frameFound = true;
              idPredecessor = localFrameSet[fi].body_id;
              Xp.r = localFrameSet[fi].r;
              Xp.E = localFrameSet[fi].E;
            }
          }
          if(frameFound == false){
            ostringstream errormsg;
            errormsg << "Could not find a local frame with the name: "
                     << localFrameName
                     << " which is used in constraint set " << constraint_name
                     << endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }
        }else{
          if(!model_table["constraint_sets"][conName.c_str()]
              [ci + 1]["predecessor_body"].exists()) {
            throw Errors::RBDLFileParseError(
                  "predecessor_body not specified.\n");
          }

          idPredecessor =
            model->GetBodyId(model_table["constraint_sets"]
                             [conName.c_str()][ci + 1]["predecessor_body"]
                             .getDefault<string>("").c_str());
          Xp = model_table["constraint_sets"][conName.c_str()]
                [ci + 1]["predecessor_transform"]
                .getDefault<SpatialTransform>(SpatialTransform());
        }

        if(model_table["constraint_sets"]
           [conName.c_str()][ci + 1]["successor_local_frame"].exists()){

          std::string localFrameName = model_table["constraint_sets"]
              [conName.c_str()][ci + 1]["successor_local_frame"]
              .getDefault<string>("");
          bool frameFound = false;
          unsigned int fi=0;
          while(fi < localFrameSet.size() && frameFound == false){
            if(std::strcmp(localFrameSet[fi].name.c_str(),
                           localFrameName.c_str())==0){
              frameFound = true;
              idSuccessor = localFrameSet[fi].body_id;
              Xs.r = localFrameSet[fi].r;
              Xs.E = localFrameSet[fi].E;
            }
            ++fi;
          }
          if(frameFound == false){
            ostringstream errormsg;
            errormsg << "Could not find a local frame with the name: "
                     << localFrameName
                     << " which is used in constraint set " << constraint_name
                     << endl;
            throw Errors::RBDLFileParseError(errormsg.str());
          }

        }else{

          if(!model_table["constraint_sets"][conName.c_str()]
              [ci + 1]["successor_body"].exists()) {
            throw Errors::RBDLFileParseError("successor_body not specified.\n");
          }

          idSuccessor =
            model->GetBodyId(model_table["constraint_sets"]
                             [conName.c_str()][ci + 1]["successor_body"]
                             .getDefault<string>("").c_str());


          Xs =  model_table["constraint_sets"][conName.c_str()]
                  [ci + 1]["successor_transform"]
                  .getDefault<SpatialTransform>(SpatialTransform());
        }



        // Add the loop constraint as a non-stablized constraint and compute
        // and set the actual stabilization cofficients for the Baumgarte
        // stabilization afterwards if enabled.

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

        unsigned int constraint_id;
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

//==============================================================================

bool LuaModelReadMotionCaptureMarkers (
      const char* filename,
      const RigidBodyDynamics::Model *model,
      std::vector<Point>& upd_point_set,
      bool verbose)
{

  LuaTable luaTable       = LuaTable::fromFile (filename);
  upd_point_set.clear();

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
          upd_point_set.push_back(point);
        }
      }
    }
  }

  return true;
}
//==============================================================================
bool LuaModelReadLocalFrames (
      LuaTable &model_table,
      const RigidBodyDynamics::Model *model,
      std::vector<LocalFrame>& upd_local_frame_set,
      bool verbose)
{
  //LuaTable luaTable       = LuaTable::fromFile (filename);
  upd_local_frame_set.clear();
  unsigned int localFrameCount =
      unsigned(int(model_table["local_frames"].length()));

  if(localFrameCount > 0){
    upd_local_frame_set.resize(localFrameCount);
    LocalFrame localFrame;

    for (unsigned int i = 1; i <= localFrameCount; ++i) {

      localFrame = model_table["local_frames"][signed(i)];

      localFrame.body_id     = model->GetBodyId (localFrame.body_name.c_str());
      upd_local_frame_set[i-1] = localFrame;

      if (verbose) {
        cout  << "LocalFrame '" << upd_local_frame_set[i-1].name
              << "' (name = "   << upd_local_frame_set[i-1].name << ")" << endl;
        cout  << "  body        = " << upd_local_frame_set[i-1].body_name
              << " (id = " << upd_local_frame_set[i-1].body_id << ")" << endl;
        cout  << "  r  = '" << upd_local_frame_set[i-1].r.transpose() << endl;
        cout  << "  E  = '" << upd_local_frame_set[i-1].E << endl;
      }
    }
  }
  return true;
}

//==============================================================================
bool LuaModelReadPoints (
      LuaTable &model_table,
      const RigidBodyDynamics::Model *model,
      std::vector<Point>& upd_point_set,
      bool verbose)
{

  upd_point_set.clear();
  unsigned int pointCount = unsigned(int(model_table["points"].length()));

  if(pointCount > 0){
    upd_point_set.resize(pointCount);
    Point point;

    for (unsigned int i = 1; i <= pointCount; ++i) {

      point = model_table["points"][i];

      point.body_id   = model->GetBodyId (point.body_name.c_str());
      upd_point_set[i-1]   = point;

      if (verbose) {
        cout  << "Point '"           << upd_point_set[i-1].name
              << "' (PointName = "   << upd_point_set[i-1].name << ")"    << endl;
        cout  << "  body        = "  << upd_point_set[i-1].body_name
              << " (id = "           << upd_point_set[i-1].body_id << ")" << endl;
        cout  << "  point_local  = '"       << upd_point_set[i-1].point_local.transpose()
                                     << endl;
      }
    }
  }
  return true;
}

//==============================================================================
bool LuaModelReadHumanMetaData(
  const std::string &filename,
  HumanMetaData &human_meta_data,
  bool verbose)
{

  LuaTable luaTable = LuaTable::fromFile (filename.c_str());
  unsigned int subjectCount = luaTable["human_meta_data"].length();

  if(subjectCount != 1){
    cerr  << "The lua file contains meta data for "
          << subjectCount
          << " but it should contain data for 1 subject"
          << endl;
    assert(0);
    abort();          
  }

  human_meta_data = luaTable["human_meta_data"][1];  

  return true;
}

//==============================================================================
#ifdef RBDL_BUILD_ADDON_MUSCLE
std::vector<unsigned int> getQIndex(
                  const RigidBodyDynamics::Model &model,
                  const char *childBodyName)
{

  unsigned int idChild = model.GetBodyId(childBodyName);
  unsigned int idJoint=idChild;
  while(idJoint > 1 && model.mBodies[idJoint-1].mIsVirtual){
    --idJoint;
  }

  std::vector<unsigned int> qIndex;

  unsigned int ccid;

  unsigned int id=idJoint;
  unsigned int dof = 0;

  while(id <= idChild){
    if(model.mJoints[id].mJointType == JointTypeCustom){
      ccid = model.mJoints[id].custom_joint_index;
      for(unsigned int i=0; i<model.mCustomJoints[ccid]->mDoFCount;++i){
        qIndex.push_back(model.mJoints[id].q_index+i);
      }
      dof = model.mCustomJoints[ccid]->mDoFCount;
    }else{
      for(unsigned int i=0; i<model.mJoints[id].mDoFCount;++i){
        qIndex.push_back(model.mJoints[id].q_index+i);
      }
      dof = model.mJoints[id].mDoFCount;
    }
    id += dof;
  }
  //To get the extra index for the quaternion joints
  for(id=idJoint;id<=idChild;++id){
    if(model.multdof3_w_index[id] > 0){
      qIndex.push_back(model.multdof3_w_index[id]);
    }
  }
  return qIndex;
}


unsigned int getMillard2016TorqueMuscleTypeId(std::string name)
{


  unsigned int i = 0;
  while (i != JointTorqueSet::Last) {
    if (name.find( std::string(JointTorqueSet.names[i]) )
        != std::string::npos ) {
      break;
    }
    ++i;
  }

  if (i == JointTorqueSet::Last) {
    cerr <<"Error: " << name << " does not contain the name of any registered"
         <<" muscle torque generator. For the full list look for the definition"
         <<" of JointTorqueSet::names[] in  "
         <<"addons/muscle/Millard2016TorqueMuscle.cc" << endl;
    assert(0);
    abort();
  }
  
  return i;

}

bool LuaModelReadMillard2016TorqueMuscleSets(
    const std::string &filename,
    const RigidBodyDynamics::Model &model,
    const HumanMetaData &human_meta_data,
    std::vector <Millard2016TorqueMuscle> &updMtgSet,
    std::vector <Millard2016TorqueMuscleInfo> &updMtgSetInfo,
    bool verbose)
{


  LuaTable     luaTable  = LuaTable::fromFile (filename.c_str());
  unsigned int mtgCount  = luaTable["millard2016_torque_muscles"].length();


  updMtgSet.resize(mtgCount);
  updMtgSetInfo.resize(mtgCount);
  Millard2016TorqueMuscleInfo mtgInfo;
  Millard2016TorqueMuscleInfo mtgInfoDefault;
  unsigned int id;

  for(unsigned int i = 1; i <= mtgCount; ++i){
    mtgInfo = luaTable["millard2016_torque_muscles"][i];
    id = i-unsigned(int(1));

    updMtgSetInfo[id] = mtgInfo;

    if (verbose) {
      if(i == 1){
        std::cout << "Millard2016TorqueMuscle" << std::endl;
        std::cout << std::setw(20) << "Name"
                  << std::setw(10)  << "Angle-Sign"
                  << std::setw(11)  << "Torque-Sign"
                  << std::endl;
      }
        std::cout   << std::setw(24) << mtgInfo.name
                    << std::setw(3)  << mtgInfo.angle_sign
                    << std::setw(3)  << mtgInfo.torque_sign
                    << std::endl;
    }

  }

  //Populate the subject information structure
  SubjectInformation subjectInfo;
  if (human_meta_data.gender == "male") {
    subjectInfo.gender = GenderSet::Male;
  }else if (human_meta_data.gender == "female") {
    subjectInfo.gender = GenderSet::Female;
  }else {
    cerr << "Unknown gender in subject metadata, " << 
      human_meta_data.gender << endl;
    abort();
  }
  
  if (human_meta_data.age_group == "Young18To25") {
    subjectInfo.ageGroup =
            AgeGroupSet::Young18To25;
  }else if (human_meta_data.age_group == "Middle55to65") {
    subjectInfo.ageGroup =
            AgeGroupSet::Middle55To65;
  }else if (human_meta_data.age_group == "SeniorOver65") {
    subjectInfo.ageGroup =
            AgeGroupSet::SeniorOver65;
  }else {
    cerr << "Unknown age group in subject metadata, " << 
      human_meta_data.age_group << endl;
    abort();
  }

  subjectInfo.heightInMeters  = human_meta_data.height;
  subjectInfo.massInKg        = human_meta_data.mass;

  DataSet::item mtgDataSet;

  for(unsigned int i = 0; i < mtgCount; ++i){
        
    if(updMtgSetInfo[i].data_set == "Gymnast"){
      mtgDataSet = DataSet::Gymnast;
    }else if(updMtgSetInfo[i].data_set == "Anderson2007"){
      mtgDataSet = DataSet::Anderson2007;
    }else{
      cerr << "Error: the data_set entry for "
           << updMtgSetInfo[i].name << " is "
           << updMtgSetInfo[i].data_set
           << "which neither Gymnast nor Anderson2007.";
      assert(0);
      abort();
    }

    if(updMtgSetInfo[i].body != mtgInfoDefault.body){
      unsigned int joint_offset = 0;
      if(updMtgSetInfo[i].joint_index != mtgInfoDefault.joint_index){
        joint_offset =updMtgSetInfo[i].joint_index;
      }

      //Go get the index of the first joint between the child body (given)
      //and its parent.
      std::vector<unsigned int> qIndices =
              getQIndex(model, updMtgSetInfo[i].body.c_str());

      updMtgSetInfo[i].q_index = qIndices[0] + joint_offset;
      updMtgSetInfo[i].qdot_index  = updMtgSetInfo[i].q_index;
      updMtgSetInfo[i].force_index = updMtgSetInfo[i].q_index;

    }
    if(updMtgSetInfo[i].activation_index == mtgInfoDefault.activation_index){
      updMtgSetInfo[i].activation_index = i;
    }

    updMtgSet[i] = Millard2016TorqueMuscle(
                          mtgDataSet,
                          subjectInfo,
                      int(getMillard2016TorqueMuscleTypeId(updMtgSetInfo[i].name)),
                          updMtgSetInfo[i].joint_angle_offset,
                          updMtgSetInfo[i].angle_sign,
                          updMtgSetInfo[i].torque_sign,
                          updMtgSetInfo[i].name);

    //Parameters for manual adjustment
    if (!std::isnan(updMtgSetInfo[i].passive_element_damping_coeff)) {
        updMtgSet[i].setNormalizedDampingCoefficient(
                    updMtgSetInfo[i].passive_element_damping_coeff);
    }

    if (!std::isnan(updMtgSetInfo[i].passive_element_torque_scale)) {
        updMtgSet[i].setPassiveTorqueScale(
                    updMtgSetInfo[i].passive_element_torque_scale);
    }

    if (!std::isnan(updMtgSetInfo[i].passive_element_angle_offset)) {
        updMtgSet[i].setPassiveCurveAngleOffset(
                    updMtgSetInfo[i].passive_element_angle_offset);
    }

    //Basic fitting of the passive curves
    if (!std::isnan(updMtgSetInfo[i].fit_passive_torque_scale[0]) &&
        !std::isnan(updMtgSetInfo[i].fit_passive_torque_scale[1])) {
        updMtgSet[i].fitPassiveTorqueScale(
                    updMtgSetInfo[i].fit_passive_torque_scale[0],
                    updMtgSetInfo[i].fit_passive_torque_scale[1]);
    }
    if (!std::isnan(updMtgSetInfo[i].fit_passive_torque_offset[0]) &&
        !std::isnan(updMtgSetInfo[i].fit_passive_torque_offset[1])) {

        updMtgSet[i].fitPassiveCurveAngleOffset(
                    updMtgSetInfo[i].fit_passive_torque_offset[0],
                    updMtgSetInfo[i].fit_passive_torque_offset[1]);
    }

    //Fitting parameters from the fitting method

    if(!std::isnan(updMtgSetInfo[i].max_isometric_torque)){
        updMtgSet[i].setMaximumActiveIsometricTorque(
                    updMtgSetInfo[i].max_isometric_torque);
    }

    if(!std::isnan(updMtgSetInfo[i].max_angular_velocity)){
        updMtgSet[i].setMaximumConcentricJointAngularVelocity(
                    updMtgSetInfo[i].max_angular_velocity);
    }
    if(!std::isnan(updMtgSetInfo[i].active_torque_angle_blending)) {
        updMtgSet[i].setActiveTorqueAngleCurveBlendingVariable(
                    updMtgSetInfo[i].active_torque_angle_blending);
    }
    if(!std::isnan(updMtgSetInfo[i].passive_torque_angle_blending)) {
        updMtgSet[i].setPassiveTorqueAngleCurveBlendingVariable(
                    updMtgSetInfo[i].passive_torque_angle_blending);
    }
    if(!std::isnan(updMtgSetInfo[i].torque_velocity_blending)) {
        updMtgSet[i].setTorqueAngularVelocityCurveBlendingVariable(
                    updMtgSetInfo[i].torque_velocity_blending);
    }
    //if(!std::isnan(updMtgSetInfo[i].angle_torque_angle_scaling)) {
    //    updMtgSet[i].setActiveTorqueAngleCurveAngleScaling(
    //                updMtgSetInfo[i].angle_torque_angle_scaling);
    //}


    
  }

  return true;
}
#endif

}
}
