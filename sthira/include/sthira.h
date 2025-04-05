#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
// #include "pinocchio/collision/broadphase.hpp"
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace sthira {

using Scalar = double; // if we switch it to other type then we might need to
// adjust datatype of pinocchio accordingly

class sthira {
private:
  /* data */
public:
  sthira(/* args */);
  ~sthira();
};

sthira::sthira(/* args */) {}

sthira::~sthira() {}

} // namespace sthira
