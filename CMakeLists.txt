cmake_minimum_required(VERSION 3.8)
project(ros2-pinocchio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(cpp_pub src/cpp_publisher.cpp)
ament_target_dependencies(cpp_pub rclcpp std_msgs)
add_executable(cpp_sub src/cpp_subscriber.cpp)
ament_target_dependencies(cpp_sub rclcpp std_msgs)

add_executable(pinocchio_model src/pinocchio_model.cpp)
ament_target_dependencies(pinocchio_model pinocchio rclcpp std_msgs sensor_msgs)

install(TARGETS
  cpp_pub
  cpp_sub
  pinocchio_model
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/py_publisher.py
  scripts/py_subscriber.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
