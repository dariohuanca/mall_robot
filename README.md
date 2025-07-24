# Robot Mall

# Installation Guide

Follow these steps to set up the package on a fresh Ubuntu machine with **ROS 2 Humble**.

## 1. Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake
sudo apt-get install -y can-utils iproute2 linux-modules-extra-$(uname -r)
sudo apt-get install -y python3 python3-pip python3-pybind11 python3-setuptools
```

## 2. Install Python Packages

```bash
python3 -m pip install --upgrade pip
python3 -m pip install adafruit-circuitpython-ina260 python-can pyserial
```

## 3. Install Required ROS 2 Packages

```bash
sudo apt install -y ros-humble-joy
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
```

## 4. Install Intel® RealSense™ ROS Package

Follow **one** of these tutorials, depending on your hardware:

* Official ROS 2 wrapper: <https://github.com/IntelRealSense/realsense-ros>
* Jetson‑specific quick‑start (Jetson Orin): <https://jetsonhacks.com/2025/03/20/jetson-orin-realsense-in-5-minutes>

> **Tip:** Complete the firmware and udev‑rule steps in the tutorial before continuing.

## 5. Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## 6. Clone the Repositories

```bash
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/2b-t/myactuator_rmd.git
git clone https://github.com/2b-t/myactuator_rmd_ros.git
git clone --recursive https://github.com/dariohuanca/mall_robot.git
```

## 7. Build the Workspace

From the workspace root:

```bash
cd ~/ros2_ws
colcon build --cmake-args -D PYTHON_BINDINGS=on
# If you encounter missing‑dependency errors, run the command a second time:
# colcon build --cmake-args -D PYTHON_BINDINGS=on
source install/setup.bash
```

## 8. Set Up udev Rules for sllidar / RPLIDAR

sllidar_ros2 requires read/write access to the serial device (e.g. `/dev/ttyUSB0`).

### Create udev rule

You have to have the command colcon_cd in your system, if not, run the next commands

```bash
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
```

Verify if the file create_udev_rules.sh in sllidar_ros2/scripts has correctly name the package.
If it has something like this line 

```bash
colcon_cd rplidar_ros2
```

Correct it changing it to 

```bash
colcon_cd sllidar_ros2
```

Then run the script to create the udev rule

```bash
cd ~/ros2_ws/src/sllidar_ros2
source scripts/create_udev_rules.sh
```

Reconnect the LIDAR USB cable to apply the new rule.

---

You are now ready to launch the nodes