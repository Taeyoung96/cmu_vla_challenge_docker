FROM osrf/ros:noetic-desktop-full

# C++ 17 setting
ENV CXX_FLAGS="-std=c++17"
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test
RUN apt-get update
RUN apt-get install -y gcc-8 g++-8
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 800 --slave /usr/bin/g++ g++ /usr/bin/g++-8

# Install PCL & Eigen & essential libraries
RUN apt-get update && apt-get install -y cmake libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev libsuitesparse-dev libglew-dev wget unzip python3-pip
RUN pip3 install catkin_tools
RUN apt-get install -y ros-noetic-tf ros-noetic-cv-bridge ros-noetic-pcl-conversions ros-noetic-xacro ros-noetic-robot-state-publisher \
    ros-noetic-rviz ros-noetic-image-transport ros-noetic-image-transport-plugins ros-noetic-pcl-ros ros-noetic-eigen-conversions ros-noetic-tf-conversions \
    ros-noetic-catkin python3-osrf-pycommon python3-rosdep python3-catkin-tools python3-vcstool git libusb-dev python-yaml python-is-python3

WORKDIR /root/catkin_ws

# Load ROS environment at each run
COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]