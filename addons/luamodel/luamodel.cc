#include "rbdl/rbdl.h"
#include "luamodel.h"

#include <iostream>
#include <map>

#include "luatables.h"

extern "C"
{
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

template<>
Vector3d LuaTableNode::getDefault<Vector3d>(const Vector3d &default_value) { 
  Vector3d result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    if (vector_table.length() != 3) {
      cerr << "LuaModel Error: invalid 3d vector!" << endl;
      abort();
    }
    
    result[0] = vector_table[1];
    result[1] = vector_table[2];
    result[2] = vector_table[3];
  }

  stackRestore();

  return result;
}

template<>
SpatialVector LuaTableNode::getDefault<SpatialVector>(
  const SpatialVector &default_value
) {
  SpatialVector result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);
    
    if (vector_table.length() != 6) {
      cerr << "LuaModel Error: invalid 6d vector!" << endl;
      abort();
    }
    result[0] = vector_table[1];
    result[1] = vector_table[2];
    result[2] = vector_table[3];
    result[3] = vector_table[4];
    result[4] = vector_table[5];
    result[5] = vector_table[6];
  }

  stackRestore();

  return result;
}

template<>
Matrix3d LuaTableNode::getDefault<Matrix3d>(const Matrix3d &default_value) {
  Matrix3d result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);
    
    if (vector_table.length() != 3) {
      cerr << "LuaModel Error: invalid 3d matrix!" << endl;
      abort();
    }

    if (vector_table[1].length() != 3
        || vector_table[2].length() != 3
        || vector_table[3].length() != 3) {
      cerr << "LuaModel Error: invalid 3d matrix!" << endl;
      abort();
    }

    result(0,0) = vector_table[1][1];
    result(0,1) = vector_table[1][2];
    result(0,2) = vector_table[1][3];

    result(1,0) = vector_table[2][1];
    result(1,1) = vector_table[2][2];
    result(1,2) = vector_table[2][3];

    result(2,0) = vector_table[3][1];
    result(2,1) = vector_table[3][2];
    result(2,2) = vector_table[3][3];
  }

  stackRestore();

  return result;
}

template<>
SpatialTransform LuaTableNode::getDefault<SpatialTransform>(
  const SpatialTransform &default_value
) {
  SpatialTransform result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);
  
    result.r = vector_table["r"].getDefault<Vector3d>(Vector3d::Zero(3));
    result.E = vector_table["E"].getDefault<Matrix3d>(Matrix3d::Identity (3,3));
  }

  stackRestore();

  return result;
}

template<>
Joint LuaTableNode::getDefault<Joint>(const Joint &default_value) {
  Joint result = default_value;
  
  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    int joint_dofs = vector_table.length();

    if (joint_dofs == 1) {
      string dof_string = vector_table[1].getDefault<std::string>("");
      if (dof_string == "JointTypeSpherical") {
        stackRestore();
        return Joint(JointTypeSpherical);
      } else if (dof_string == "JointTypeEulerZYX") {
        stackRestore();
        return Joint(JointTypeEulerZYX);
      }
      if (dof_string == "JointTypeEulerXYZ") {
        stackRestore();
        return Joint(JointTypeEulerXYZ);
      }
      if (dof_string == "JointTypeEulerYXZ") {
        stackRestore();
        return Joint(JointTypeEulerYXZ);
      }
      if (dof_string == "JointTypeTranslationXYZ") {
        stackRestore();
        return Joint(JointTypeTranslationXYZ);
      }
    }

    if (joint_dofs > 0) {
      if (vector_table[1].length() != 6) {
        cerr << "LuaModel Error: invalid joint motion subspace description at " 
          << this->keyStackToString() << endl;
        abort();
      }
    }

    switch (joint_dofs) {
    case 0:
      result = Joint(JointTypeFixed);
      break;
    case 1:
      result = Joint (vector_table[1].get<SpatialVector>());
      break;
    case 2: 
      result = Joint(
        vector_table[1].get<SpatialVector>(),
        vector_table[2].get<SpatialVector>()
      );
      break;
    case 3:
      result = Joint(
        vector_table[1].get<SpatialVector>(),
        vector_table[2].get<SpatialVector>(),
        vector_table[3].get<SpatialVector>()
      );
      break;
    case 4:
      result = Joint(
        vector_table[1].get<SpatialVector>(),
        vector_table[2].get<SpatialVector>(),
        vector_table[3].get<SpatialVector>(),
        vector_table[4].get<SpatialVector>()
      );
      break;
    case 5:
      result = Joint(
        vector_table[1].get<SpatialVector>(),
        vector_table[2].get<SpatialVector>(),
        vector_table[3].get<SpatialVector>(),
        vector_table[4].get<SpatialVector>(),
        vector_table[5].get<SpatialVector>()
      );
      break;
    case 6:
      result = Joint(
        vector_table[1].get<SpatialVector>(),
        vector_table[2].get<SpatialVector>(),
        vector_table[3].get<SpatialVector>(),
        vector_table[4].get<SpatialVector>(),
        vector_table[5].get<SpatialVector>(),
        vector_table[6].get<SpatialVector>()
      );
      break;
    default:
      cerr << "Invalid number of DOFs for joint." << endl;
      abort();
      break;
    }
  }

  stackRestore();

  return result;
}

template<>
Body LuaTableNode::getDefault<Body>(const Body &default_value) {
  Body result = default_value;

  if (stackQueryValue()) {
    LuaTable vector_table = LuaTable::fromLuaState (luaTable->L);

    double mass = 0.;
    Vector3d com (Vector3d::Zero(3));
    Matrix3d inertia (Matrix3d::Identity(3,3));

    mass = vector_table["mass"];
    com = vector_table["com"].getDefault<Vector3d>(com);
    inertia = vector_table["inertia"].getDefault<Matrix3d>(inertia);

    result = Body (mass, com, inertia);
  }

  stackRestore();

  return result;
}

namespace RigidBodyDynamics {

namespace Addons {

bool LuaModelReadFromTable (LuaTable &model_table, Model *model, bool verbose);
bool LuaModelReadConstraintsFromTable (
  LuaTable &model_table,
  Model *model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
);

typedef map<string, unsigned int> StringIntMap;
StringIntMap body_table_id_map;

RBDL_DLLAPI
bool LuaModelReadFromLuaState (lua_State* L, Model* model, bool verbose) {
  assert (model);

  LuaTable model_table = LuaTable::fromLuaState (L);

  return LuaModelReadFromTable (model_table, model, verbose);
}

RBDL_DLLAPI
bool LuaModelReadFromFile (const char* filename, Model* model, bool verbose) {
  if(!model) {
    std::cerr << "Model not provided." << std::endl;
    assert(false);
    abort();
  }

  LuaTable model_table = LuaTable::fromFile (filename);

  return LuaModelReadFromTable (model_table, model, verbose);
}


RBDL_DLLAPI
std::vector<std::string> LuaModelGetConstraintSetNames(const char* filename) {
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
      std::cerr << "Invalid constraint found in model.constraint_sets: no constraint set name was specified!"
        << std::endl;
      abort();
    }

    result.push_back(constraint_keys[ci].string_value);
  }

  return result;
}

RBDL_DLLAPI
bool LuaModelReadFromFileWithConstraints (
  const char* filename,
  Model* model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
) {
  if(!model) {
    std::cerr << "Model not provided." << std::endl;
    assert(false);
    abort();
  }
  if(constraint_sets.size() != constraint_set_names.size()) {
    std::cerr << "Number of constraint sets different from the number of \
      constraint set names." << std::endl;
    assert(false);
    abort();
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


bool LuaModelReadFromTable (LuaTable &model_table, Model* model, bool verbose) {
  if (model_table["gravity"].exists()) {
    model->gravity = model_table["gravity"].get<Vector3d>();

    if (verbose)
      cout << "gravity = " << model->gravity.transpose() << endl;
  }

  int frame_count = model_table["frames"].length();

  body_table_id_map["ROOT"] = 0;

  for (int i = 1; i <= frame_count; i++) {
    if (!model_table["frames"][i]["parent"].exists()) {
      cerr << "Parent not defined for frame " << i << "." << endl;
      abort();
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
      cout << "  body id    : " << body_id << endl;
      cout << "  parent_id  : " << parent_id << endl;
      cout << "  joint dofs : " << joint.mDoFCount << endl;
      for (unsigned int j = 0; j < joint.mDoFCount; j++) {
        cout << "    " << j << ": " << joint.mJointAxes[j].transpose() << endl;
      }
      cout << "  joint_frame: " << joint_frame << endl;
    }
  }

  return true;
}

bool LuaModelReadConstraintsFromTable (
  LuaTable &model_table,
  Model *model,
  std::vector<ConstraintSet>& constraint_sets,
  const std::vector<std::string>& constraint_set_names,
  bool verbose
) { 
  for(size_t i = 0; i < constraint_set_names.size(); ++i) {
    if (verbose) {
      std::cout << "==== Constraint Set: " << constraint_set_names[i] << std::endl;
    }

    if(!model_table["constraint_sets"][constraint_set_names[i].c_str()]
      .exists()) {
      cerr << "Constraint set not existing: " << constraint_set_names[i] << "." 
        << endl;
      assert(false);
      abort();
    }

    size_t num_constraints = model_table["constraint_sets"]
      [constraint_set_names[i].c_str()]
      .length();

    for(size_t ci = 0; ci < num_constraints; ++ci) {
      if (verbose) {
        std::cout << "== Constraint " << ci << "/" << num_constraints << " ==" << std::endl;
      }

      if(!model_table["constraint_sets"]
        [constraint_set_names[i].c_str()][ci + 1]["constraint_type"].exists()) {
        cerr << "constraint_type not specified." << endl;
        assert(false);
        abort();
      }
      string constraintType = model_table["constraint_sets"]
        [constraint_set_names[i].c_str()][ci + 1]["constraint_type"]
        .getDefault<string>("");
      std::string constraint_name = model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
        ["name"].getDefault<string>("");

      if(constraintType == "contact") {
        if(!model_table["constraint_sets"][constraint_set_names[i].c_str()]
          [ci + 1]["body"].exists()) {
          cerr << "body not specified." << endl;
          assert(false);
          abort();
        }
        constraint_sets[i].AddContactConstraint
          (model->GetBodyId(model_table["constraint_sets"]
            [constraint_set_names[i].c_str()][ci + 1]["body"]
            .getDefault<string>("").c_str())
          , model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["point"].getDefault<Vector3d>(Vector3d::Zero())
          , model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["normal"].getDefault<Vector3d>(Vector3d::Zero())
          , model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["name"].getDefault<string>("").c_str()
          , model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["normal_acceleration"].getDefault<double>(0.));
        if(verbose) {
          cout << "  type = contact" << endl;
          cout << "  name = " << constraint_name << std::endl;
          cout << "  body = " 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["body"].getDefault<string>("") << endl;
          cout << "  body point = " 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["point"].getDefault<Vector3d>(Vector3d::Zero()).transpose() 
            << endl;
          cout << "  world normal = " 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["normal"].getDefault<Vector3d>(Vector3d::Zero()).transpose() 
            << endl;
          cout << "  normal acceleration = "
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["normal_acceleration"].getDefault<double>(0.) << endl;
        }
      }
      else if(constraintType == "loop") {
        if(!model_table["constraint_sets"][constraint_set_names[i].c_str()]
          [ci + 1]["predecessor_body"].exists()) {
          cerr << "predecessor_body not specified." << endl;
          assert(false);
          abort();
        }
        if(!model_table["constraint_sets"][constraint_set_names[i].c_str()]
          [ci + 1]["successor_body"].exists()) {
          cerr << "successor_body not specified." << endl;
          assert(false);
          abort();
        }

        // Add the loop constraint as a non-stablized constraint and compute
        // and set the actual stabilization cofficients for the Baumgarte
        // stabilization afterwards if enabled.
        unsigned int constraint_id;
        constraint_id = constraint_sets[i].AddLoopConstraint(model->GetBodyId
          (model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["predecessor_body"].getDefault<string>("").c_str())
          , model->GetBodyId(model_table["constraint_sets"]
            [constraint_set_names[i].c_str()][ci + 1]["successor_body"]
            .getDefault<string>("").c_str())
          , model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["predecessor_transform"].getDefault<SpatialTransform>(SpatialTransform())
          , model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["successor_transform"].getDefault<SpatialTransform>(SpatialTransform())
          , model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["axis"].getDefault<SpatialVector>(SpatialVector::Zero())
          , false
          , 0.0
          , constraint_name.c_str());

        bool enable_stabilization = model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["enable_stabilization"].getDefault<bool>(false);
        double stabilization_parameter = 0.0;
        if (enable_stabilization) {
          stabilization_parameter = model_table["constraint_sets"][constraint_set_names[i].c_str()][ci + 1]
            ["stabilization_parameter"].getDefault<double>(0.1);
          if (stabilization_parameter <= 0.0) {
            std::cerr << "Invalid stabilization parameter: " << stabilization_parameter
              << " must be > 0.0" << std::endl;
            abort();
          }
          double stabilization_coefficient = 1.0 / stabilization_parameter;
          constraint_sets[i].baumgarteParameters[i] = Vector2d(
              stabilization_coefficient, stabilization_coefficient);
        }

        if(verbose) {
          cout << "  type = loop" << endl;
          cout << "  name = " << constraint_name << std::endl;
          cout << "  predecessor body = " 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["predecessor_body"].getDefault<string>("") << endl;
          cout << "  successor body = " 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["successor_body"].getDefault<string>("") << endl;
          cout << "  predecessor body transform = " << endl 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["predecessor_transform"]
            .getDefault<SpatialTransform>(SpatialTransform()) << endl;
          cout << "  successor body transform = " << endl 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["successor_transform"]
            .getDefault<SpatialTransform>(SpatialTransform()) << endl;
          cout << "  constraint axis (in predecessor frame) = " 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["axis"].getDefault<SpatialVector>(SpatialVector::Zero())
            .transpose() << endl;
          cout << "  enable_stabilization = " << enable_stabilization 
            << endl; 
          if (enable_stabilization) {
            cout << "  stabilization_parameter = " << stabilization_parameter 
              << endl; 
          }
          cout << "  constraint name = " 
            << model_table["constraint_sets"][constraint_set_names[i].c_str()]
            [ci + 1]["name"].getDefault<string>("").c_str() << endl;
        }
      }
      else {
        cerr << "Invalid constraint type: " << constraintType << endl;
        abort();
      }
    }
  }

  return true;
}

}

}
