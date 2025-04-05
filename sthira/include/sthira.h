#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
// #include "pinocchio/collision/broadphase.hpp"
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include<iostream>

namespace Sthira {

using Scalar = double; // if we switch it to other type then we might need to
// adjust datatype of pinocchio accordingly

class Sthira {
public:
  void loadPinocchioModelFromXML(const std::string &xml_stream);

  bool isModelLoaded() { return is_loaded_; }

  void setModelLoadStatus(bool status) { is_loaded_ = status; }

  void applyForwardKinematics(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints);

  void updateTransform();

  void
  setQJoints(const std::unordered_map<std::string, Scalar> &joint_positions);

private:
  void initializeModelData();

  bool is_loaded_ = false;
  pinocchio::Model model_;
  pinocchio::GeometryModel visual_model_;
  pinocchio::GeometryModel collision_model_;

  pinocchio::Data model_data_;
  pinocchio::GeometryData visual_data_;
  pinocchio::GeometryData collision_data_;

  std::unordered_map<std::string, Scalar> joint_position_map_;
  std::unordered_map<pinocchio::JointIndex,
                     Eigen::Transform<Scalar, 3, Eigen::Isometry>>
      joints_transform_map_;
  std::unordered_map<pinocchio::FrameIndex,
                     Eigen::Transform<Scalar, 3, Eigen::Isometry>>
      frame_transform_map_;

  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> q_joints_;
};

} // namespace Sthira
