#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
// #include "pinocchio/collision/broadphase.hpp"
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>

namespace Sthira {

using Scalar = double; // if we switch it to other type then we might need to
// adjust datatype of pinocchio accordingly

class Sthira {
public:
  enum Type { JOINT, FRAME };

  void loadPinocchioModelFromXML(const std::string &xml_stream);

  bool isModelLoaded() { return (is_loaded_ && model_.njoints >= 1); }

  void setModelLoadStatus(bool status) { is_loaded_ = status; }

  uint32_t getNumOfJoints() { return static_cast<uint32_t>(model_.njoints); }

  uint32_t getNumOfFrames() { return static_cast<uint32_t>(model_.nframes); }

  const std::unordered_map<pinocchio::FrameIndex,
                           Eigen::Transform<Scalar, 3, Eigen::Isometry>> &
  getTransformsOfFrames() {
    return frame_transform_map_;
  }

  const std::unordered_map<pinocchio::JointIndex,
                           Eigen::Transform<Scalar, 3, Eigen::Isometry>> &
  getTransformsOfJoints() {
    return joint_transform_map_;
  }

  void applyForwardKinematics(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints);

  void applyForwardKinematics() { applyForwardKinematics(this->q_joints_); }

  void updateTransform();

  void
  setQJoints(const std::unordered_map<std::string, Scalar> &joint_positions);

  void computeJacobian(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints,
                       const uint32_t index, pinocchio::Data::Matrix6x &J,
                       const Type m);

  void computeJacobian(const uint32_t index, pinocchio::Data::Matrix6x &J,
                       const Type m) {
    computeJacobian(this->q_joints_, index, J, m);
  }

private:
  void initializeModelData();

  bool is_loaded_ = false;
  pinocchio::Model model_;
  pinocchio::GeometryModel visual_model_;
  pinocchio::GeometryModel collision_model_;

  pinocchio::Data model_data_;
  pinocchio::GeometryData visual_data_;
  pinocchio::GeometryData collision_data_;

  std::unordered_map<pinocchio::JointIndex,
                     Eigen::Transform<Scalar, 3, Eigen::Isometry>>
      joint_transform_map_;
  std::unordered_map<pinocchio::FrameIndex,
                     Eigen::Transform<Scalar, 3, Eigen::Isometry>>
      frame_transform_map_;

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q_joints_;
};

} // namespace Sthira
