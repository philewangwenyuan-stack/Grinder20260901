#!/usr/bin/env bash
# Run as root on the Ubuntu 20.04 simulation host. Does not start a robot stack.
set -euo pipefail
source /etc/os-release
[[ "${ID}:${VERSION_ID}" == ubuntu:20.04 ]] || { echo 'Ubuntu 20.04 is required.' >&2; exit 1; }
[[ "${EUID}" == 0 ]] || { echo 'Run with sudo bash install_remote_sim_deps.sh' >&2; exit 1; }
export DEBIAN_FRONTEND=noninteractive
# Repair our own source before apt update when retrying an interrupted install.
if [[ -f /etc/apt/sources.list.d/grinder-ros-noetic.list ]]; then
  if [[ -f /usr/share/keyrings/grinder-ros-archive-keyring.asc ]]; then
    cp /usr/share/keyrings/grinder-ros-archive-keyring.asc /usr/share/keyrings/grinder-ros-archive-keyring.gpg
  fi
  printf '%s\n' 'deb [signed-by=/usr/share/keyrings/grinder-ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main' \
    > /etc/apt/sources.list.d/grinder-ros-noetic.list
fi
apt-get update
apt-get install -y ca-certificates curl gnupg
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  if [[ ! -s /usr/share/keyrings/grinder-ros-archive-keyring.gpg ]]; then
    curl --fail --location --retry 3 --connect-timeout 10 --max-time 60 \
      https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/grinder-ros-archive-keyring.gpg
  fi
  printf '%s\n' 'deb [signed-by=/usr/share/keyrings/grinder-ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main' \
    > /etc/apt/sources.list.d/grinder-ros-noetic.list
  apt-get update
fi
apt-get install -y build-essential cmake git rsync xauth mesa-utils \
  ros-noetic-desktop-full ros-noetic-navigation ros-noetic-libg2o \
  ros-noetic-gazebo-ros-control ros-noetic-robot-localization \
  ros-noetic-tf-conversions ros-noetic-eigen-conversions ros-noetic-tf2-sensor-msgs \
  ros-noetic-mbf-costmap-core ros-noetic-mbf-msgs ros-noetic-costmap-converter \
  libgoogle-glog-dev libgflags-dev libapr1-dev libsuitesparse-dev \
  libyaml-cpp-dev libusb-1.0-0-dev libudev-dev libpcap-dev \
  libprotobuf-dev protobuf-compiler pybind11-dev python3-dev python3-pip \
  python3-numpy python3-scipy python3-matplotlib python3-opencv python3-yaml \
  python3-serial python3-paho-mqtt ffmpeg
python3 -m pip install 'protobuf==4.25.1'
echo '[OK] Remote simulation dependencies installed.'
