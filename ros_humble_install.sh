#!/bin/bash

REMOVE_SCRIPT=
NO_GAZEBO=

while [[ "${#}" -gt 0 ]]; do
    case "${1}" in
        -r|--remove)
            REMOVE_SCRIPT="true"
            shift 1
            ;;

        -h|-help|--help)
            echo "$0 [-r|--remove]"
            echo "this scripts automates the process of installing ROS"
            echo "-r(optional): remove this script after instaalation of ros"
            exit 0
            ;;

        *)
            echo "unknown argument $1"
            exit 1
            ;;
    esac
done

apt-get update && apt-get install -y locales \
&& locale-gen en_US en_US.UTF-8 \
&& update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
&& export LANG=en_US.UTF-8

apt-get install -y software-properties-common \
&& add-apt-repository universe -y \
&& apt-get install curl -y \
&& curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt-get update && apt-get -y upgrade

apt-get install -y ros-humble-desktop

apt-get install -y python3-colcon-common-extensions

if [[ "${REMOVE_SCRIPT}" == "true" ]]; then
    rm -f "${0}"
fi