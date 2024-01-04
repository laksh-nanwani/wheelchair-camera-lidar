# wheelchair-camera-lidar

git clone https://github.com/laksh-nanwani/wheelchair-camera-lidar.git

# Install ros-noetic from http://wiki.ros.org/noetic/Installation/Ubuntu

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl -y # if you haven't already installed curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y

sudo apt install python3-rosdep -y

sudo rosdep init

rosdep update

sudo apt-install ros-noetic-map-server

sudo apt install ros-noetic-ddynamic-reconfigure

# Install librealsense from https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide
sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev -y

sudo apt-get install git wget cmake build-essential -y

sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at -y

git clone https://github.com/IntelRealSense/librealsense.git

cd librealsense

./scripts/setup_udev_rules.sh

mkdir build && cd build

cmake ../

sudo make uninstall && make clean && make && sudo make install

sudo apt-get install liborocos-bfl-dev

sudo apt-get install ros-noetic-navigation

sudo apt-get install ros-noetic-geometry2

sudo apt-get install ros-noetic-geographic-info

sudo apt-get install ros-noetic-robot-navigation
