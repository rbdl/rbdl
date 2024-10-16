#include <rbdl/rbdl.h>

#include "urdfreader.h"

#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <stack>

#ifdef RBDL_USE_ROS_URDF_LIBRARY
  #include <urdf_model/model.h>
  #include <urdf_parser/urdf_parser.h>
  #include <boost/shared_ptr.hpp>

  typedef urdf::LinkSharedPtr LinkPtr;
  typedef const urdf::LinkConstSharedPtr ConstLinkPtr;
  typedef urdf::JointSharedPtr JointPtr;
  typedef urdf::ModelInterfaceSharedPtr ModelPtr;
  typedef urdf::Joint UrdfJointType;

  #define LINKMAP links_
  #define JOINTMAP joints_
  #define PARENT_TRANSFORM parent_to_joint_origin_transform
  #define RPY getRPY
#else
  #include <urdf/model.h>
  #include <urdf/link.h>
  #include <urdf/joint.h>

  typedef std::shared_ptr<urdf::Link> LinkPtr;
  typedef std::shared_ptr<urdf::Link> ConstLinkPtr;
  typedef std::shared_ptr<urdf::Joint> JointPtr;
  typedef std::shared_ptr<urdf::UrdfModel> ModelPtr;
  typedef urdf::JointType UrdfJointType;

  #define LINKMAP link_map
  #define JOINTMAP joint_map
  #define PARENT_TRANSFORM parent_to_joint_transform
  #define RPY getRpy
#endif

using namespace std;

namespace RigidBodyDynamics
{

  namespace Addons
  {

    using namespace Math;
    using namespace Errors;

    typedef vector<LinkPtr> URDFLinkVector;
    typedef vector<JointPtr> URDFJointVector;
    typedef map<string, LinkPtr> URDFLinkMap;
    typedef map<string, JointPtr> URDFJointMap;

// =============================================================================

std::string get_model_xml_string_from_file(const char *filename)
{
  ifstream model_file(filename);
  if (!model_file) {
    ostringstream error_msg;
    error_msg << "Error opening file '" << filename << "'." << endl;
    throw RBDLFileParseError(error_msg.str());
  }

  // reserve memory for the contents of the file
  string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());

  model_file.close();
  return model_xml_string;
}

Joint get_rbdl_joint(const JointPtr &urdf_joint)
{
  Joint rbdl_joint;
  if (urdf_joint->type == UrdfJointType::REVOLUTE ||
      urdf_joint->type == UrdfJointType::CONTINUOUS) {
    rbdl_joint = Joint(SpatialVector(urdf_joint->axis.x, urdf_joint->axis.y,
                                     urdf_joint->axis.z, 0., 0., 0.));
  } else if (urdf_joint->type == UrdfJointType::PRISMATIC) {
    rbdl_joint = Joint(SpatialVector(0., 0., 0., urdf_joint->axis.x,
                                     urdf_joint->axis.y, urdf_joint->axis.z));
  } else if (urdf_joint->type == UrdfJointType::FIXED) {
    rbdl_joint = Joint(JointTypeFixed);
  } else if (urdf_joint->type == UrdfJointType::FLOATING) {
    // todo: what order of DoF should be used?
    rbdl_joint = Joint(
      SpatialVector(0., 0., 0., 1., 0., 0.),
      SpatialVector(0., 0., 0., 0., 1., 0.),
      SpatialVector(0., 0., 0., 0., 0., 1.),
      SpatialVector(1., 0., 0., 0., 0., 0.),
      SpatialVector(0., 1., 0., 0., 0., 0.),
      SpatialVector(0., 0., 1., 0., 0., 0.));
  } else if (urdf_joint->type == UrdfJointType::PLANAR) {
    // todo: which two directions should be used that are perpendicular
    // to the specified axis?
    ostringstream error_msg;
    error_msg << "Error while processing joint '" << urdf_joint->name
              << "': planar joints not yet supported!" << endl;
    throw RBDLFileParseError(error_msg.str());
  }
  return rbdl_joint;
}

Body get_rbdl_body(ConstLinkPtr &urdf_link, bool is_root_link)
{
  // assemble the body
  urdf::Vector3 link_inertial_rpy_temp;
  Vector3d link_inertial_position;
  Vector3d link_inertial_rpy;
  Matrix3d link_inertial_inertia = Matrix3d::Zero();
  double link_inertial_mass = 0.;


  // but only if we actually have inertial data
#ifdef RBDL_USE_ROS_URDF_LIBRARY
  if (urdf_link->inertial) {
    auto I = urdf_link->inertial;
#else
  if (urdf_link->inertial.has_value()) {
    auto I = &urdf_link->inertial.value();
#endif
    link_inertial_mass = I->mass;

    link_inertial_position.set(
      I->origin.position.x,
      I->origin.position.y,
      I->origin.position.z);

    if(!is_root_link){
      I->origin.rotation.RPY(link_inertial_rpy_temp.x,
                             link_inertial_rpy_temp.y,
                             link_inertial_rpy_temp.z);
      link_inertial_rpy.set(link_inertial_rpy_temp.x,
                            link_inertial_rpy_temp.y,
                            link_inertial_rpy_temp.z);
    }

    link_inertial_inertia(0, 0) = I->ixx;
    link_inertial_inertia(0, 1) = I->ixy;
    link_inertial_inertia(0, 2) = I->ixz;

    link_inertial_inertia(1, 0) = I->ixy;
    link_inertial_inertia(1, 1) = I->iyy;
    link_inertial_inertia(1, 2) = I->iyz;

    link_inertial_inertia(2, 0) = I->ixz;
    link_inertial_inertia(2, 1) = I->iyz;
    link_inertial_inertia(2, 2) = I->izz;

    if(is_root_link)
    {
      if (link_inertial_mass == 0.
          && (I->ixx != 0 || I->ixy != 0 || I->ixz != 0 || I->iyy != 0
              || I->iyz != 0 || I->izz != 0)) {
        std::ostringstream error_msg;
        error_msg << "Error creating rbdl model! Urdf root link ("
                  << urdf_link->name
                  << ") has inertial but no mass!";
        throw RBDLFileParseError(error_msg.str());
      }
    }
    else
    {
      if (link_inertial_rpy_temp.x != 0 || link_inertial_rpy_temp.y != 0
          || link_inertial_rpy_temp.z != 0) {
        ostringstream error_msg;
        error_msg << "Error while processing body '" << urdf_link->name
                  << "': rotation of body frames not yet supported."
                  << " Please rotate the joint frame instead." << endl;
        throw RBDLFileParseError(error_msg.str());
      }
    }
  }

  Body rbdl_body = Body(link_inertial_mass, link_inertial_position,
                          link_inertial_inertia);

  return rbdl_body;
}

void add_joints_to_rbdl_model(Model *rbdl_model, const URDFLinkMap &link_map,
                              const URDFJointMap &joint_map,
                              const vector<string> &joint_names, bool verbose)
{
  unsigned int j;
  for (j = 0; j < joint_names.size(); j++) {
    JointPtr urdf_joint = joint_map.at(joint_names.at(j));
    LinkPtr urdf_parent = link_map.at(urdf_joint->parent_link_name);
    LinkPtr urdf_child = link_map.at(urdf_joint->child_link_name);

    // determine where to add the current joint and child body
    unsigned int rbdl_parent_id = 0;

    rbdl_parent_id = rbdl_model->GetBodyId(urdf_parent->name.c_str());


    if (rbdl_parent_id == std::numeric_limits<unsigned int>::max()) {
      ostringstream error_msg;
      error_msg << "Error while processing joint '" << urdf_joint->name
                << "': parent link '" << urdf_parent->name
                << "' could not be found." << endl;
      throw RBDLFileParseError(error_msg.str());
    }

    // create the joint
    Joint rbdl_joint = get_rbdl_joint(urdf_joint);

    // compute the joint transformation
    // Temp variable used for compatability between urdf functions
    // and potential Casadi symbolics.
    urdf::Vector3 joint_rpy_temp;

    Vector3d joint_rpy;
    Vector3d joint_translation;
    urdf_joint->PARENT_TRANSFORM.rotation.RPY(joint_rpy_temp.x,
                                              joint_rpy_temp.y,
                                              joint_rpy_temp.z);
    joint_rpy.set(joint_rpy_temp.x, joint_rpy_temp.y, joint_rpy_temp.z);
    joint_translation.set(
      urdf_joint->PARENT_TRANSFORM.position.x,
      urdf_joint->PARENT_TRANSFORM.position.y,
      urdf_joint->PARENT_TRANSFORM.position.z);
     SpatialTransform rbdl_joint_frame =
       Xrotx(joint_rpy[0])
       * Xroty(joint_rpy[1])
       * Xrotz(joint_rpy[2])
       * Xtrans(joint_translation);
     //rbdl_joint_frame = Xtrans(joint_translation);

    // assemble the body
    Body rbdl_body = get_rbdl_body(urdf_child, false);

    if(rbdl_model->mBodyNameMap.find(urdf_child->name) != rbdl_model->mBodyNameMap.end())
    {
      if (verbose) {
        cout << "+ Skipping Add Body: " << urdf_child->name << endl;
      }
      continue;
    }

    if (verbose) {
      cout << "+ Adding Body: " << urdf_child->name << endl;
      cout << "  parent_id  : " << rbdl_parent_id << endl;
      cout << "  joint names: " << joint_names[j] << endl;
      cout << "  joint frame: " << rbdl_joint_frame << endl;
      cout << "  joint dofs : " << rbdl_joint.mDoFCount << endl;
      for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
        cout << "    " << j << ": "
             << rbdl_joint.mJointAxes[j].transpose() << endl;
      }
      cout << "  body inertia: " << endl
           << rbdl_body.mInertia << endl;
      cout << "  body mass   : " << rbdl_body.mMass << endl;
      cout << "  body name   : " << urdf_child->name << endl;
    }

    if (urdf_joint->type == UrdfJointType::FLOATING) {
      Matrix3d zero_matrix = Matrix3d::Zero();
      Body null_body(0., Vector3d::Zero(), zero_matrix);
      Joint joint_txtytz(JointTypeTranslationXYZ);
      string trans_body_name = urdf_child->name + "_Translate";
      rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame,
                          joint_txtytz, null_body,
                          trans_body_name);

      Joint joint_euler_zyx(JointTypeEulerXYZ);
      rbdl_model->AppendBody(SpatialTransform(), joint_euler_zyx,
                             rbdl_body, urdf_child->name);
    } else {
      rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame, rbdl_joint,
                          rbdl_body, urdf_child->name);
    }
  }
}

void construct_model(Model *rbdl_model, ModelPtr urdf_model,
                     const string &root_link, bool floating_base, bool verbose) {

  LinkPtr urdf_root_link;

  URDFLinkMap link_map = urdf_model->LINKMAP;
  URDFJointMap joint_map = urdf_model->JOINTMAP;

  vector<string> joint_names;

  // Holds the links that we are processing in our depth first traversal
  // with the top element being the current link.
  stack<LinkPtr> link_stack;
  // Holds the child joint index of the current link
  stack<int> joint_index_stack;

  // Check if the parsed root link is a valid one or not
  if(link_map.count(root_link) == 0){
    ostringstream error_msg;
    error_msg << "Error the parsed root link: '" << root_link
              << "' could not be found." << endl;
    throw RBDLFileParseError(error_msg.str());
  }
  // add the bodies in a depth-first order of the model tree
  link_stack.push(link_map[root_link]);

  // add the root body
  ConstLinkPtr root = urdf_model->getLink(root_link);
  Body root_link_body = get_rbdl_body(root, true);

  Joint root_joint(JointTypeFixed);
  if (floating_base) {
    root_joint = JointTypeFloatingBase;
  }

  SpatialTransform root_joint_frame = SpatialTransform();

  if (verbose) {
    cout << "+ Adding Root Body " << endl;
    cout << "  joint frame: " << root_joint_frame << endl;
    if (floating_base) {
      cout << "  joint type : floating" << endl;
    } else {
      cout << "  joint type : fixed" << endl;
    }
    cout << "  body inertia: " << endl
         << root_link_body.mInertia << endl;
    cout << "  body mass   : " << root_link_body.mMass << endl;
    cout << "  body name   : " << root->name << endl;
  }

  rbdl_model->AppendBody(root_joint_frame,
                         root_joint,
                         root_link_body,
                         root->name);

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  while (link_stack.size() > 0) {
    LinkPtr cur_link = link_stack.top();

    unsigned int joint_idx = joint_index_stack.top();

    // Add any child bodies and increment current joint index if we still
    // have child joints to process.
    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // increment joint index
      joint_index_stack.pop();
      joint_index_stack.push(joint_idx + 1);

      link_stack.push(link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      if (verbose) {
        for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
          cout << "  ";
        }
        cout << "joint '" << cur_joint->name << "' child link '" <<
          link_stack.top()->name << "' type = " << cur_joint->type << endl;
      }

      joint_names.push_back(cur_joint->name);
    } else {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }

  add_joints_to_rbdl_model(rbdl_model, link_map, joint_map, joint_names,
                           verbose);
}
// =============================================================================

void construct_partial_model(Model *rbdl_model, ModelPtr urdf_model,
                             const string &root_link,
                             const vector<string> &tip_links,
                             bool floating_base, bool verbose) {
    LinkPtr urdf_root_link;

    URDFLinkMap link_map = urdf_model->LINKMAP;
    URDFJointMap joint_map = urdf_model->JOINTMAP;

    // Holds the links that we are processing in our depth first traversal
    // with the top element being the current link.
    stack<LinkPtr> link_stack;
    // Holds the child joint index of the current link
    stack<int> joint_index_stack;

    // Check if the parsed root link and tip links are valid or not
    if(link_map.count(root_link) == 0){
      ostringstream error_msg;
      error_msg << "Error the parsed root link: '" << root_link
                << "' could not be found." << endl;
      throw RBDLFileParseError(error_msg.str());
    }
    if(tip_links.empty()){
      ostringstream error_msg;
      error_msg << "Error the parsed tip links cannot be empty!" << endl;
      throw RBDLFileParseError(error_msg.str());
    }
    // add the bodies in a depth-first order of the model tree
    link_stack.push(link_map[root_link]);

    // add the root body
    ConstLinkPtr root = urdf_model->getLink(root_link);
    Body root_link_body = get_rbdl_body(root, true);

    Joint root_joint(JointTypeFixed);
    if (floating_base) {
      root_joint = JointTypeFloatingBase;
    }

    SpatialTransform root_joint_frame = SpatialTransform();

    if (verbose) {
      cout << "+ Adding Root Body " << endl;
      cout << "  joint frame: " << root_joint_frame << endl;
      if (floating_base) {
        cout << "  joint type : floating" << endl;
      } else {
        cout << "  joint type : fixed" << endl;
      }
      cout << "  body inertia: " << endl
           << root_link_body.mInertia << endl;
      cout << "  body mass   : " << root_link_body.mMass << endl;
      cout << "  body name   : " << root->name << endl;
    }

    rbdl_model->AppendBody(root_joint_frame,
                           root_joint,
                           root_link_body,
                           root->name);

    // depth first traversal: push the first child onto our joint_index_stack
    joint_index_stack.push(0);

    for(const std::string &tip_link : tip_links)
    {
        vector<string> joint_names;
        vector<string> local_joint_names;
        string parent_link = tip_link;
        vector<string> links_verbose;
        while(parent_link.compare(root_link) != 0)
        {
          ostringstream verbose_string;
          if (!(link_map[parent_link] && link_map[parent_link]->getParent() && 
              link_map[parent_link]->parent_joint) ) {
            ostringstream error_msg;
            error_msg << "Error while processing tip link '" << tip_link
                      << "' as reached root link is '" << parent_link
                      << "', couldn't find desired root link '"
                      << root_link << "' in the tree"  << endl;
            throw RBDLFileParseError(error_msg.str());
          }
          else{
          local_joint_names.push_back(
              link_map[parent_link]->parent_joint->name);
          }
          parent_link = link_map[parent_link]->getParent()->name;
          if (verbose) {
            verbose_string
                << "joint '" << link_map[parent_link]->parent_joint->name
                << "' child link '"
                << link_map[parent_link]->parent_joint->child_link_name
                << "' type = " << link_map[parent_link]->parent_joint->type
                << endl;
            links_verbose.push_back(verbose_string.str());
          }
        }
        reverse(local_joint_names.begin(), local_joint_names.end());
        reverse(links_verbose.begin(), links_verbose.end());
        joint_names.insert(joint_names.end(), local_joint_names.begin(),
                           local_joint_names.end());
        if(verbose)
        {
          for (unsigned int i = 0; i < links_verbose.size(); i++) {
          for (unsigned int j = 0; j < i; j++) {
            cout << "  ";
          }
            cout << links_verbose[i];
          }
        }
        add_joints_to_rbdl_model(rbdl_model, link_map, joint_map, joint_names,
                                verbose);
    }
}

RBDL_ADDON_DLLAPI bool URDFReadFromFile(const char *filename, Model *model,
                                        bool floating_base, bool verbose)
{
  const string model_xml_string = get_model_xml_string_from_file(filename);

  return URDFReadFromString(model_xml_string.c_str(), model, floating_base,
                            verbose);
}

// =============================================================================

RBDL_ADDON_DLLAPI bool URDFReadFromString(const char *model_xml_string,
                                          Model *model,
                                          bool floating_base,
                                          bool verbose)
{
    assert(model);

#ifdef RBDL_USE_ROS_URDF_LIBRARY
    ModelPtr urdf_model = urdf::parseURDF(model_xml_string);
#else
    ModelPtr urdf_model = urdf::UrdfModel::fromUrdfStr(model_xml_string);
#endif

    construct_model(model, urdf_model, urdf_model->getRoot()->name,
                    floating_base, verbose);

    model->gravity.set(0., 0., -9.81);

    return true;
}

RBDL_ADDON_DLLAPI bool PartialURDFReadFromFile(
    const char *filename,
    Model *model,
    const std::string &root_link,
    const std::vector<std::string> &tip_links,
    bool floating_base,
    bool verbose) {
  const string model_xml_string = get_model_xml_string_from_file(filename);

  return PartialURDFReadFromString(model_xml_string.c_str(), model, root_link,
                                   tip_links, floating_base, verbose);
}

RBDL_ADDON_DLLAPI bool PartialURDFReadFromString(
    const char *model_xml_string,
    Model *model,
    const std::string &root_link,
    const std::vector<std::string> &tip_links,
    bool floating_base,
    bool verbose) {
  assert(model);

#ifdef RBDL_USE_ROS_URDF_LIBRARY
  ModelPtr urdf_model = urdf::parseURDF(model_xml_string);
#else
  ModelPtr urdf_model = urdf::UrdfModel::fromUrdfStr(model_xml_string);
#endif

  if(tip_links.empty()){
      construct_model(model, urdf_model, root_link, floating_base, verbose);
  }
  else{
      construct_partial_model(model, urdf_model, root_link, tip_links,
                              floating_base, verbose);
  }

  model->gravity.set(0., 0., -9.81);

  return true;
}

} // namespace Addons

} // namespace RigidBodyDynamics
