#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
// #include "pinocchio/collision/broadphase.hpp"
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using Scalar = double; // if we switch it to other type then we might need to
                       // adjust datatype of pinocchio accordingly

class RobotDescripionSubscriber : public rclcpp::Node {
public:
  RobotDescripionSubscriber() : Node("robot_description_subscriber") {
    robot_des_topic_ = this->create_subscription<std_msgs::msg::String>(
        "/robot_description", 10,
        std::bind(&RobotDescripionSubscriber::robotDescSubs, this,
                  std::placeholders::_1));
  }

private:
  void loadPinocchioModelFromXML(const std_msgs::msg::String &msg) {
    pinocchio::urdf::buildModelFromXML(msg.data, model_);
    RCLCPP_INFO(this->get_logger(),
                "[RobotDescripionSubscriber::loadPinocchioModelFromXML] "
                "Pinocchio model loaded with %d joints.",
                model_.njoints);

    std::istringstream _xml_stream(msg.data.c_str());
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
    RCLCPP_INFO(this->get_logger(),
                "[RobotDescripionSubscriber::loadPinocchioModelFromXML] "
                "Pinocchio collision model loaded sucessfully");
    is_loaded_ = true;

    initializeModelData();
    findJointIds();
  }

  void findJointIds() {
    if (is_loaded_ == false || model_.njoints == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[RobotDescripionSubscriber::findJointIds] model is not "
                  "loaded, no of joints is %d",
                  model_.njoints);
      return;
    }
    joint_id_map_.clear();
    pinocchio::JointIndex _n_joints = model_.njoints;
    for (pinocchio::JointIndex i = 1; i < _n_joints; ++i) { // 0 universe
      std::cout << "Joint[" << i << "] : " << model_.names[i] << std::endl;
      joint_id_map_[model_.names[i]] = i;
    }
  }

  void initializeModelData() {
    if (is_loaded_ == false || model_.njoints == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[RobotDescripionSubscriber::initializeModelData] model is not "
                  "loaded, no of joints is %d",
                  model_.njoints);
      return;
    }

    model_data_ = pinocchio::Data(model_);
    visual_data_ = pinocchio::GeometryData(visual_model_);
    collision_data_ = pinocchio::GeometryData(collision_model_);

    RCLCPP_INFO(this->get_logger(),
                "[RobotDescripionSubscriber::initializeModelData] "
                "Pinocchio model data initialized sucessfully");
  }

  void
  forwardKinematics(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &q_joints) {

    if (is_loaded_ == false || model_.njoints == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[RobotDescripionSubscriber::forwardKinematics] model is not "
                  "loaded, no of joints is %d",
                  model_.njoints);
      return;
    }

    pinocchio::forwardKinematics(model_, model_data_, q_joints);
    pinocchio::updateFramePlacements(model_, model_data_);
    pinocchio::updateGeometryPlacements(model_, model_data_, collision_model_,
                                        collision_data_);
    pinocchio::updateGeometryPlacements(model_, model_data_, visual_model_,
                                        visual_data_);

    updateTransform();
  }

  void updateTransform() {

    if (is_loaded_ == false || model_.njoints == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[RobotDescripionSubscriber::updateTransform] model is not "
                  "loaded, no of joints is %d",
                  model_.njoints);
      return;
    }

    pinocchio::JointIndex _n_joints = model_.njoints;
    for (pinocchio::JointIndex i = 1; i < _n_joints; ++i) {
      pinocchio::SE3 _joint_transform = model_data_.oMi[i];
      joints_transform_[i].linear() = _joint_transform.rotation();
      joints_transform_[i].translation() = _joint_transform.translation();
    }
  }

  void robotDescSubs(const std_msgs::msg::String &msg) {
    if (!is_loaded_) {
      loadPinocchioModelFromXML(msg);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_des_topic_;
  bool is_loaded_ = false;
  pinocchio::Model model_;
  pinocchio::GeometryModel visual_model_;
  pinocchio::GeometryModel collision_model_;

  pinocchio::Data model_data_;
  pinocchio::GeometryData visual_data_;
  pinocchio::GeometryData collision_data_;

  std::unordered_map<std::string, pinocchio::JointIndex> joint_id_map_;
  std::unordered_map<pinocchio::JointIndex,
                     Eigen::Transform<Scalar, 3, Eigen::Isometry>>
      joints_transform_;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDescripionSubscriber>());
  rclcpp::shutdown();

  return 0;
}