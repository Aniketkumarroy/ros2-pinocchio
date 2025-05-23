#include "sthira.h"

namespace sthira {

void Sthira::loadPinocchioModelFromXML(const std::string &xml_stream) {

  pinocchio::urdf::buildModelFromXML(xml_stream, model_);
  std::cout << "[Sthira::loadPinocchioModelFromXML] "
               "Pinocchio model loaded with "
            << model_.njoints << " joints." << "\n";

  std::istringstream _xml_stream(xml_stream.c_str());
  if (_xml_stream.str().empty()) {
    const std::string exception_message(
        "error while converting std_msgs::msg::String to std::istringstream");
    throw std::invalid_argument(exception_message);
  }

  pinocchio::urdf::buildGeom(model_, _xml_stream, pinocchio::COLLISION,
                             collision_model_);

  _xml_stream.clear();
  _xml_stream.seekg(0, std::ios::beg);

  pinocchio::urdf::buildGeom(model_, _xml_stream, pinocchio::VISUAL,
                             visual_model_);

  collision_model_.addAllCollisionPairs();
  removeAdjacentCollisionPairs();

  std::cout << "[Sthira::loadPinocchioModelFromXML] "
               "Pinocchio collision model loaded sucessfully"
            << "\n";
  is_loaded_ = true;

  q_joints_.setZero(model_.nq);

  frame_transform_map_.clear();
  joint_transform_map_.clear();

  initializeModelData();
}

void Sthira::initializeModelData() {
  if (is_loaded_ == false || model_.njoints <= 1) {
    std::cout << "[Sthira::initializeModelData] model is not "
                 "loaded, no of joints is "
              << model_.njoints << "\n";
    return;
  }
  model_data_ = pinocchio::Data(model_);
  visual_data_ = pinocchio::GeometryData(visual_model_);
  collision_data_ = pinocchio::GeometryData(collision_model_);

  std::cout << "[Sthira::initializeModelData] "
               "Pinocchio model data initialized sucessfully"
            << "\n";
}

void Sthira::applyForwardKinematics(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints) {
  if (is_loaded_ == false || model_.njoints <= 1) {
    std::cout << "[Sthira::applyForwardKinematics] model is not "
                 "loaded, no of joints is "
              << model_.njoints << "\n";
    return;
  }

  pinocchio::forwardKinematics(model_, model_data_, q_joints);
  pinocchio::updateFramePlacements(model_, model_data_);
  pinocchio::updateGeometryPlacements(model_, model_data_, collision_model_,
                                      collision_data_);
  pinocchio::updateGeometryPlacements(model_, model_data_, visual_model_,
                                      visual_data_);

  std::cout << "[Sthira::applyForwardKinematics] "
               "sucessfully did forward kinematic"
            << "\n";
}

void Sthira::updateTransform() {
  if (is_loaded_ == false || model_.njoints <= 1) {
    std::cout << "[Sthira::updateTransform] model is not "
                 "loaded, no of joints is "
              << model_.njoints << "\n";
    return;
  }

  uint32_t _n_joints = model_.njoints;
  for (pinocchio::JointIndex i = 1; i < _n_joints; ++i) {
    pinocchio::SE3 _joint_transform = model_data_.oMi[i];
    joint_transform_map_[i].linear() = _joint_transform.rotation();
    joint_transform_map_[i].translation() = _joint_transform.translation();
  }

  uint32_t _n_frames = model_.nframes;
  for (pinocchio::FrameIndex i = 1; i < _n_frames; ++i) {
    const pinocchio::SE3 &_frame_transform = model_data_.oMf[i];
    frame_transform_map_[i].linear() = _frame_transform.rotation();
    frame_transform_map_[i].translation() = _frame_transform.translation();
  }

  std::cout << "[Sthira::updateTransform] "
               "sucessfully updated frames and joints transform"
            << "\n";
}

void Sthira::setQJoints(
    const std::unordered_map<std::string, Scalar> &joint_positions) {
  if (is_loaded_ == false || model_.njoints <= 1) {
    std::cout << "[Sthira::setQJoints] model is "
                 "not loaded, no of joints is "
              << model_.njoints << "\n";
    return;
  }
  if (static_cast<uint32_t>(model_.njoints - 1) !=
      static_cast<uint32_t>(joint_positions.size())) {
    std::cout << "[Sthira::setQJoints] Mismatch "
                 "in joint counts: model_.njoints - 1 ("
              << model_.njoints - 1
              << ") != "
                 "joint_position size ("
              << joint_positions.size() << ")" << "\n";

    return;
  }
  uint32_t _n_joints = model_.njoints;
  for (pinocchio::JointIndex i = 1; i < _n_joints; ++i) {
    const auto &_joint = model_.joints[i];
    const std::string &_joint_name = model_.names[i];

    const auto _it = joint_positions.find(_joint_name);
    if (_it == joint_positions.end()) {
      std::cout << "[Sthira::setQJoints] Joint" << _joint_name
                << " not found in joint_positions" << "\n";
      return;
    }
    auto idx_q = _joint.idx_q(); // Where to insert in q
    auto nq = _joint.nq();       // How many entries for this joint

    switch (nq) {
    case 1:
      q_joints_[idx_q + 0] = _it->second;
      break;

    case 2:
      q_joints_[idx_q + 0] = std::cos(_it->second / 2.0);
      q_joints_[idx_q + 1] = std::sin(_it->second / 2.0);
      break;
    default:
      std::cout << "[Sthira::setQJoints] Joint " << _joint_name
                << " has unsupported nq = " << nq << "; skipping" << "\n";
      break;
    }
  }
}

void Sthira::computeJacobian(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints,
    const uint32_t index, pinocchio::Data::Matrix6x &J, const Sthira::Type m) {
  if (is_loaded_ == false || model_.njoints <= 1) {
    std::cout << "[Sthira::computeJacobian] model is not "
                 "loaded, no of joints is "
              << model_.njoints << "\n";
    return;
  }
  uint32_t _N;
  if (m == Sthira::FRAME)
    _N = static_cast<uint32_t>(model_.nframes);
  else if (m == Sthira::JOINT)
    _N = static_cast<uint32_t>(model_.njoints);
  else {
    std::cout << "[Sthira::computeJacobian] wrong value for m provided, "
                 "possible values are "
                 "Sthira::FRAME or Sthira::JOINT"
              << "\n";
    return;
  }
  if (index >= _N) {
    std::cout << "[Sthira::computeJacobian] index " << index
              << " is out of bounds" << "\n";
    return;
  }
  if (m == Sthira::FRAME)
    pinocchio::computeFrameJacobian(model_, model_data_, q_joints, index, J);
  else if (m == Sthira::JOINT)
    pinocchio::computeJointJacobian(model_, model_data_, q_joints, index, J);
}

void Sthira::removeAdjacentCollisionPairs() {
  std::vector<pinocchio::CollisionPair> _filtered_pairs;

  for (const auto &_pair : collision_model_.collisionPairs) {
    const auto &_obj1 = collision_model_.geometryObjects[_pair.first];
    const auto &_obj2 = collision_model_.geometryObjects[_pair.second];

    // Check adjacency using parent joints
    pinocchio::JointIndex _joint1 = _obj1.parentJoint;
    pinocchio::JointIndex _joint2 = _obj2.parentJoint;

    if (model_.parents[_joint1] == _joint2 ||
        model_.parents[_joint2] == _joint1) {
      // Skip adjacent pairs
      continue;
    }

    _filtered_pairs.push_back(_pair);
  }

  collision_model_.collisionPairs = _filtered_pairs;
}

void Sthira::computeCollisions(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints,
    bool stop_at_first_collision) {

  // Compute the forward kinematics, update the geometry placements and calls
  // computeCollision for every active pairs of GeometryData.
  pinocchio::computeCollisions(model_, model_data_, collision_model_,
                               collision_data_, q_joints,
                               stop_at_first_collision);

  for (uint32_t i = 0; i < collision_model_.collisionPairs.size(); ++i) {
    const pinocchio::CollisionPair &_cp = collision_model_.collisionPairs[i];
    const hpp::fcl::CollisionResult &_cr = collision_data_.collisionResults[i];

    if (_cr.isCollision()) {
      std::string link1 = model_.frames[_cp.first].name;
      std::string link2 = model_.frames[_cp.second].name;
      std::cerr << "[Sthira::computeCollisions] link: " << link1
                << " is colliding with " << link2 << "\n";
    }
  }
}

bool Sthira::areColliding(const std::string &frame_1,
                          const std::string &frame_2) {

  pinocchio::FrameIndex _idxA = model_.getFrameId(frame_1);
  pinocchio::FrameIndex _idxB = model_.getFrameId(frame_2);
  int _geomIdA = -1, _geomIdB = -1;

  for (size_t i = 0; i < collision_model_.geometryObjects.size(); ++i) {
    const auto &_obj = collision_model_.geometryObjects[i];
    if (_geomIdA == -1 && _obj.parentFrame == _idxA)
      _geomIdA = static_cast<int>(i);
    if (_geomIdB == -1 && _obj.parentFrame == _idxB)
      _geomIdB = static_cast<int>(i);

    if (_geomIdA != -1 && _geomIdB != -1)
      break;
  }

  if (_geomIdA == -1 || _geomIdB == -1)
    return false;

  // for disabling compiler warnings
  uint32_t _idA = static_cast<uint32_t>(_geomIdA);
  uint32_t _idB = static_cast<uint32_t>(_geomIdB);

  int _pairIndex = -1;
  for (size_t i = 0; i < collision_model_.collisionPairs.size(); ++i) {
    const auto &_pair = collision_model_.collisionPairs[i];
    if ((_pair.first == _idA && _pair.second == _idB) ||
        (_pair.first == _idB && _pair.second == _idA)) {
      _pairIndex = static_cast<int>(i);
      break;
    }
  }

  if (_pairIndex == -1)
    return false;

  // This computes collision for the given pair
  bool collision = pinocchio::computeCollision(collision_model_,
                                               collision_data_, _pairIndex);
  return collision;
}

} // namespace sthira
