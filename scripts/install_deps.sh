#!/bin/bash
set -e

echo "Installing system dependencies "

DEPS_DIR="deps"

sudo apt-get install protobuf-compiler -y
sudo apt-get install libspdlog-dev -y
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libyaml-cpp-dev -y
sudo apt-get install libglib2.0-dev mesa-common-dev freeglut3-dev coinor-libipopt-dev libblas-dev liblapack-dev gfortran liblapack-dev libboost-all-dev libeigen3-dev -y
sudo apt install libhdf5-dev libqt5core5a libqt5gui5 libqt5svg5-dev libqt5widgets5 curl cmake-qt-gui g++ gcc libopencv-dev jq libqt5x11extras5-dev -y

sudo apt install ros-humble-image-transport* qml-module-qtquick-controls2 qtquickcontrols2-5-dev -y
sudo apt install ros-humble-rclpy ros-humble-nav-msgs ros-humble-sensor-msgs ros-humble-tf2-ros -y
sudo apt install qtcreator -y


sudo dpkg -i "$DEPS_DIR"/zsibot_common_*.deb
sudo dpkg -i "$DEPS_DIR"/ecal_*.deb
sudo dpkg -i "$DEPS_DIR"/mujoco_3.3.0_x86_64_Linux.deb
sudo dpkg -i "$DEPS_DIR"/onnx_1.51.0_x86_64_jammy_Linux.deb
sudo dpkg -i "$DEPS_DIR"/lcm_1.5.1-1_amd64.deb

sudo apt install -y qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    qml-module-qtquick-controls qml-module-qtquick-controls2 \
    qml-module-qtquick-layouts qml-module-qtgraphicaleffects \
    qml-module-qtqml-models2 qml-module-qtqml qml-module-qtquick-window2
sudo apt install fonts-noto-color-emoji -y

echo "System dependencies installed."
