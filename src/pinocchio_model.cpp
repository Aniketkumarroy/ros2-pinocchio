#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "<pinocchio/algorithm/jacobian.hpp>"
#include "<pinocchio/algorithm/joint-configuration.hpp>"
// #include "pinocchio/collision/broadphase.hpp"
#include <map>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

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
                "RobotDescripionSubscriber::loadPinocchioModelFromXML "
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
                "RobotDescripionSubscriber::loadPinocchioModelFromXML "
                "Pinocchio collision model loaded sucessfully");

    initializeModelData();
    findJointIds();
  }

  void findJointIds() {
    if (model_.njoints == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "RobotDescripionSubscriber::findJointIds model is not "
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
    model_data_ = pinocchio::Data(model_);
    visual_data_ = pinocchio::GeometryData(visual_model_);
    collision_data_ = pinocchio::GeometryData(collision_model_);

    RCLCPP_INFO(this->get_logger(),
                "RobotDescripionSubscriber::initializeModelData "
                "Pinocchio model data initialized sucessfully");
  }

  void robotDescSubs(const std_msgs::msg::String &msg) {
    if (!is_loaded_) {
      loadPinocchioModelFromXML(msg);
      is_loaded_ = true;
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

  std::map<std::string, pinocchio::JointIndex> joint_id_map_;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDescripionSubscriber>());
  rclcpp::shutdown();

  return 0;
}