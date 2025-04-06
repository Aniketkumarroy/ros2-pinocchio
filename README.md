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

## 📸 Example Visuals

> _Replace with your plots or screenshots if available_
- Heatmap of max payload across workspace
- Torque requirements for various end-effector configurations
- Interactive 3D plots of payload vs configuration

---


## 🔧 Installation

### 🧩 Prerequisites & Dependencies

* ### 🐢 ROS 2
to install ros2 humble in your machine head on to [ros2-humble](https://docs.ros.org/en/humble/Installation.html) and follow the steps or you can use the installation [script] to install it
```bash
chmod +x ros_humble_install.sh
./ros_humble_install.sh
```
* ## 🧍‍♂️ Pinocchio
to install pinocchio run
```bash
sudo apt install ros-humble-pinocchio
```

