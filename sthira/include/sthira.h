#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
// #include "pinocchio/collision/broadphase.hpp"
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>

namespace sthira {

using Scalar = double; // if we switch it to other type then we might need to
// adjust datatype of pinocchio accordingly

class Sthira {
public:
  enum Type { JOINT, FRAME };

  /**
   * @brief load the model from a string xml stream
   *
   * @param[in] xml_stream string stream
   */
  void loadPinocchioModelFromXML(const std::string &xml_stream);

  /**
   * @brief checks whether the pinocchio model is loaded
   *
   * @return true if loaded
   * @return false if not loaded
   */
  bool isModelLoaded() { return (is_loaded_ && model_.njoints >= 1); }

  /**
   * @brief reset the Model Load Status
   *
   * @param[in] status
   */
  void setModelLoadStatus(bool status) { is_loaded_ = status; }

  /**
   * @brief Get the Num Of Joints in the model
   *
   * @return uint32_t no. of joints
   */
  uint32_t getNumOfJoints() { return static_cast<uint32_t>(model_.njoints); }

  /**
   * @brief Get the Num Of Frames in the model
   *
   * @return uint32_t no. of frames
   */
  uint32_t getNumOfFrames() { return static_cast<uint32_t>(model_.nframes); }

  /**
   * @brief Get the Num Of degrees of freedom of model
   *
   * @return uint32_t no of dof
   */
  uint32_t getNumOfDOF() { return model_.nv; }

  /**
   * @brief Get the map of frame id and its corresponding transform
   *
   * @return const std::unordered_map<pinocchio::FrameIndex,
   * Eigen::Transform<Scalar, 3, Eigen::Isometry>>&
   */
  const std::unordered_map<pinocchio::FrameIndex,
                           Eigen::Transform<Scalar, 3, Eigen::Isometry>> &
  getTransformsOfFrames() {
    return frame_transform_map_;
  }

  /**
   * @brief Get the map of joint id and its corresponding transform
   *
   * @return const std::unordered_map<pinocchio::JointIndex,
   * Eigen::Transform<Scalar, 3, Eigen::Isometry>>&
   */
  const std::unordered_map<pinocchio::JointIndex,
                           Eigen::Transform<Scalar, 3, Eigen::Isometry>> &
  getTransformsOfJoints() {
    return joint_transform_map_;
  }

  /**
   * @brief apply forward kinematics on the model based on joints inputs
   * provided
   *
   * @param[in] q_joints joints input vector of size (model_dof, 1)
   */
  void applyForwardKinematics(
      const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints);

  /**
   * @brief apply forward kinematics on the model for its internal \c q_joints_
   *
   */
  void applyForwardKinematics() { applyForwardKinematics(this->q_joints_); }

  /**
   * @brief update the \c joint_transform_map_ and \c frame_transform_map_ after
   * applying forward kinematics
   *
   */
  void updateTransform();

  /**
   * @brief update the internal \c q_joints_ with the provided values mapping
   * the joint values with thier name
   *
   * @param[in] joint_positions joint positions with their name
   */
  void
  setQJoints(const std::unordered_map<std::string, Scalar> &joint_positions);

  /**
   * @brief computes jacobian at a specific frame or joint
   *
   * @param[in] q_joints new joint configuration for computing jacobian
   * @param[in] index index of the frame/joint for which jacobian is calculated
   * @param[out] J computed jacobian
   * @param[in] m if its Sthira::FRAME \c index will be treated as frame index,
   * if its Sthira::JOINT \c index will be treated as joint index
   */
  void computeJacobian(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints,
                       const uint32_t index, pinocchio::Data::Matrix6x &J,
                       const Type m);

  /**
   * @brief computes jacobian at a specific frame or joint
   *
   * @param[in] index index of the frame/joint for which jacobian is calculated
   * @param[out] J computed jacobian
   * @param[in] m if its Sthira::FRAME \c index will be treated as frame index,
   * if its Sthira::JOINT \c index will be treated as joint index
   */
  void computeJacobian(const uint32_t index, pinocchio::Data::Matrix6x &J,
                       const Type m) {
    computeJacobian(this->q_joints_, index, J, m);
  }

  /**
   * @brief turn off collision calculation for adjacent links
   *
   */
  void removeAdjacentCollisionPairs();

  /**
   * @brief computes collision for possible pairs of objects at \c q_joints
   * configuration
   *
   * @param[in] q_joints joint configuration
   * @param[in] stop_at_first_collision stop further calculation if found one
   * pair colliding
   */
  void
  computeCollisions(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints,
                    bool stop_at_first_collision = false);

  /**
   * @brief computes collision for possible pairs of objects for internal  \c
   * q_joints_ configuration
   *
   * @param[in] stop_at_first_collision stop further calculation if found one
   * pair colliding
   */
  void computeCollisions(bool stop_at_first_collision = false) {
    computeCollisions(this->q_joints_, stop_at_first_collision);
  }

  /**
   * @brief checkes whether two frames are colliding
   *
   * @param[in] frame_1 frame 1
   * @param[in] frame_2 frame 2
   * @return true if colliding
   * @return false if not colliding
   */
  bool areColliding(const std::string &frame_1, const std::string &frame_2);

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

} // namespace sthira
