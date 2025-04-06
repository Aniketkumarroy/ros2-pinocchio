#include "sthira.h"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

class RobotDescripionSubscriber : public rclcpp::Node {
public:
  RobotDescripionSubscriber() : Node("robot_description_subscriber") {
    robot_des_sub_ = this->create_subscription<std_msgs::msg::String>(
        robot_des_topic_, 10,
        std::bind(&RobotDescripionSubscriber::robotDescSubCallback, this,
                  std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10,
        std::bind(&RobotDescripionSubscriber::jointStateSubCallback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono_literals::operator""ms(timer_period_),
        std::bind(&RobotDescripionSubscriber::timer_callback, this));
  }

private:
  void robotDescSubCallback(const std_msgs::msg::String &msg) {
    if (!robot_dyn_.isModelLoaded()) {
      robot_dyn_.loadPinocchioModelFromXML(msg.data);
    }
  }

  void jointStateSubCallback(const sensor_msgs::msg::JointState &msg) {
    if (!robot_dyn_.isModelLoaded()) {
      RCLCPP_INFO(this->get_logger(),
                  "[RobotDescripionSubscriber::jointStateSubCallback] model is "
                  "not loaded, no of joints is %d",
                  robot_dyn_.getNumOfJoints());
      return;
    }
    uint32_t _n_joints = msg.name.size();
    std::unordered_map<std::string, sthira::Scalar> _joint_positions;
    for (uint32_t i = 0; i < _n_joints; i++) {
      _joint_positions[msg.name[i]] = msg.position[i];
    }
    robot_dyn_.setQJoints(_joint_positions);
  }

  void timer_callback() {
    // robot_dyn_.applyForwardKinematics();
    // robot_dyn_.updateTransform();

    // auto _n_frames = robot_dyn_.getNumOfFrames();
    // Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor> J =
    //     Eigen::Matrix<double, 6, Eigen::Dynamic, Eigen::ColMajor>::Zero(
    //         6, robot_dyn_.getNumOfDOF());
    // for (uint32_t i = 1; i < _n_frames; i++) {
    //   robot_dyn_.computeJacobian(i, J, sthira::Sthira::FRAME);
    // }

    robot_dyn_.computeCollisions();
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_des_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  const std::string robot_des_topic_ = "/robot_description";
  const std::string joint_states_topic_ = "/joint_states";
  const uint32_t timer_period_ = 500; // ms

  sthira::Sthira robot_dyn_;
};

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDescripionSubscriber>());
  rclcpp::shutdown();

  return 0;
}