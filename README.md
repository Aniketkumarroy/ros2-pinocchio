# ros2-pinocchio
<h1 align="center">Payload visualization/metrics</h1>

## 🛠️ Sthira — Payload Visualization & Analysis tool for Robots

**Sthira** is a tool designed to evaluate and visualize a robot’s payload capabilities based on its kinematic and dynamic structure. it is built on top of [Pinocchio](https://github.com/stack-of-tasks/pinocchio), a state-of-the-art Rigid Body Algorithms for poly-articulated systems. Whether you’re designing manipulators, mobile robots, or humanoids — Sthira helps you understand **how much your robot can carry**, **where**, and **at what cost** (torque, power, stiffness).

> 🧠 Sthira is derived from word "*Sthirá* " meaning **firm, stable**, this tool empowers roboticists to design smarter, stronger systems.

---

## 🚀 Features

- 📦 **Payload Visualization** — View payload limits in specific configurations or across a workspace
- 🧠 **URDF-Aware** — Parses your robot’s URDF from its cml stream and processes its kinematic chain
- 🧭 **Forward Kinematics** — Compute end-effector position and oriantation for any given joint configuration
-  📐 **Jacobian Calculation** — Generate Jacobian matrices for velocity and force transformation
- 💥 **Collision Detection** — Check for self-collisions or environment contact while analyzing payload
<!-- - ⚙️ **Actuation Constraints** — Takes into account joint torque, velocity, stiffness limits
- 📊 **Joint Requirement Analysis** — Determine torque/power/stiffness required to achieve a desired payload
- 🗺️ **Workspace Mapping** — Visualize payload capability across your robot’s entire workspace -->
- 🔧 **Design Aid** — Helps you select actuators and refine robot design

---

<!-- ## 📸 Example Visuals

> _Replace with your plots or screenshots if available_
- Heatmap of max payload across workspace
- Torque requirements for various end-effector configurations
- Interactive 3D plots of payload vs configuration

--- -->


## 🔧 Installation

### 🧩 Prerequisites & Dependencies

* ### 🐢 ROS 2
to install ros2 humble in your machine head on to [ros2-humble](https://docs.ros.org/en/humble/Installation.html) and follow the steps or you can use the installation [script](ros_humble_install.sh) to install it
```bash
sudo chmod +x ros_humble_install.sh
sudo ./ros_humble_install.sh
```
after installation add this line at the end of your `~/.bashrc` to auto source the ros environment
```bash
source /opt/ros/humble/setup.bash
```
or run this to edit your `~/.bashrc`
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
> `Note`: make sure to source `setup.bash` if you are using bash and `setup.zsh` if you are using zsh, for example, if your shel is zsh then you will type
```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
```
* ## 🧍‍♂️ Pinocchio
to install pinocchio run
```bash
sudo apt install ros-humble-pinocchio
```
## 🛠️ Building
first create a ros workspace and navigate into it
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
clone the repository
```bash
git clone https://github.com/Aniketkumarroy/ros2-pinocchio.git
```
build the ros project
```bash
cd ~/ros2_ws
colcon build
```
before using the workspace be sure to source the workspace
```bash
source ~/ros2_ws/install/setup.bash
```
> `Note`:make sure to source `setup.bash` if you are using bash and `setup.zsh` if you are using zsh
---
## 🗂️ Directory Structure
```bash
.
├── CMakeLists.txt
├── include
│   └── ros2-pinocchio
├── package.xml
├── README.md
├── ros_humble_install.sh
├── scripts
│   ├── py_publisher.py
│   └── py_subscriber.py
├── src
│   ├── cpp_publisher.cpp
│   ├── cpp_subscriber.cpp
│   └── pinocchio_model.cpp
└── sthira
    ├── CMakeLists.txt
    ├── include
    │   └── sthira.h
    └── src
        └── sthira.cpp
```
`scripts`: contains example of a ros2 subscriber and publisher in python. you can run the subscriber/publisher by running
```bash
ros2 run ros2-pinocchio py_publisher.py
```
for publisher or
```bash
ros2 run ros2-pinocchio py_subscriber.py
```
for subscriber

`src`: contains a c++ subscriber(`cpp_subscriber.cpp`) and a publisher(`cpp_publisher.cpp`). you can run them using
```bash
ros2 run ros2-pinocchio cpp_pub
```
for publisher or
```bash
ros2 run ros2-pinocchio cpp_sub
```
for subscriber

here the name of the executable is different as compared to python because in case of c++ the executable name is defined in the root [CMakeList.txt](CMakeLists.txt)

`sthira`: this is the library which will parse our robot_description and will do the kinematic analysis on it or detect collision.
---
## 🕹️ Example
we will try a demo robot [TIAGo](https://github.com/pal-robotics/tiago_robot)
clone the repository in your workspace
```bash
cd ~/ros2_ws/src
git clone https://github.com/pal-robotics/tiago_robot.git
```
this package will need some dependencies from PAL Robotics, you can either git clone them and build them in the same ros workspace or just install the binaries

to install all the dependencies run
```bash
sudo apt install -y ros-humble-xacro \
    ros-humble-launch-param-builder \
    ros-humble-launch-pal \
    ros-humble-pal-urdf-utils \
    ros-humble-pmb2-description \
    ros-humble-pal-hey5-description \
    ros-humble-pal-gripper-description
```
build the package
```bash
cd ~/ros2_ws
colcon build
```
source the workspace
```bash
source ~/ros2_ws/install/setup.bash
```
> `Note`:make sure to source `setup.bash` if you are using bash and `setup.zsh` if you are using zsh
now open a terminal and run the `pinocchio_model` node
```bash
ros2 run ros2-pinocchio pinocchio_model
```
in another terminal run launch the TIAGo robot
```bash
ros2 launch tiago_description show.launch.py
```
play with the joint controller gui to move the robot arm and see what collision happens

you can go through the [pinocchio_model.cpp](src/pinocchio_model.cpp) to see what it does
---

## 🧩 Miscellaneous
youtube [video](https://youtu.be/iJdse5FIPO8) for installing ros humble

youtube [video](https://youtu.be/0tMFbH3Is9k) for running ros2 publisher and subscriber

youtube [video](https://youtu.be/FQ6-xSGKHmU) for running tiago robot visualization in rviz

---