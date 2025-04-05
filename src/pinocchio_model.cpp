#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
// #include "pinocchio/collision/broadphase.hpp"
#include "pinocchio/parsers/srdf.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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
    pinocchio::urdf::buildGeom(model_, msg.data, pinocchio::VISUAL,
                               visual_model_);
    pinocchio::urdf::buildGeom(model_, msg.data, pinocchio::COLLISION,
                               collision_model_);
  }
  void robotDescSubs(const std_msgs::msg::String &msg) {
    if (!is_loaded_) {
      loadPinocchioModelFromXML(msg);
      collision_model_.addAllCollisionPairs();
      is_loaded_ = true;
    }
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_des_topic_;
  bool is_loaded_ = false;
  pinocchio::Model model_;
  pinocchio::GeometryModel visual_model_;
  pinocchio::GeometryModel collision_model_;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDescripionSubscriber>());
  rclcpp::shutdown();

  return 0;
}