#include "sthira.h"

namespace Sthira {

void Sthira::loadPinocchioModelFromXML(const std::string &xml_stream) {

  pinocchio::urdf::buildModelFromXML(xml_stream, model_);
  std::cout << "[RobotDescripionSubscriber::loadPinocchioModelFromXML] "
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
  std::cout << "[RobotDescripionSubscriber::loadPinocchioModelFromXML] "
               "Pinocchio collision model loaded sucessfully"
            << "\n";
  is_loaded_ = true;

  q_joints_.setZero(model_.nq);

  initializeModelData();
}

void Sthira::initializeModelData() {
  if (is_loaded_ == false || model_.njoints <= 1) {
    std::cout
        << "[RobotDescripionSubscriber::initializeModelData] model is not "
           "loaded, no of joints is "
        << model_.njoints << "\n";
    return;
  }
  model_data_ = pinocchio::Data(model_);
  visual_data_ = pinocchio::GeometryData(visual_model_);
  collision_data_ = pinocchio::GeometryData(collision_model_);

  std::cout << "[RobotDescripionSubscriber::initializeModelData] "
               "Pinocchio model data initialized sucessfully"
            << "\n";
}
} // namespace Sthira
