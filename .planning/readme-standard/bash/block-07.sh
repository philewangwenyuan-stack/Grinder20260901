cd "$GRINDER_WS"
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic \
  --skip-keys="gtsam fast_gicp" -r -y
sudo apt install -y libboost-all-dev libeigen3-dev libmetis-dev libpcl-dev \
  libgoogle-glog-dev libgflags-dev libyaml-cpp-dev \
  python3-yaml python3-serial python3-protobuf python3-numpy python3-scipy \
  python3-matplotlib python3-opencv python3-dev pybind11-dev
