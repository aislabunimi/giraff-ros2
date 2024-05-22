# giraff-ros2

## Configure giraff
Install dependencies:
* Run the following commands: 
```shell
rosdep install --from-paths src -y --ignore-src
sudo apt install ros-humble-topic-tools ros-humble-rmw-cyclonedds-cpp ros-humble-nav2-* ros-humble-slam-toolbox ros-humble-nav2-lifecycle-manager ros-humble-magic-enum libuvc-dev libgoogle-glog-dev nlohmann-json3-dev xterm
echo  "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
colcon build --symlink-install
```
* Install the udev rules of cameras, laser, and rfid running the `install.sh` script inside the `giraff_ros2_driver/scripts` folder
* Set the usb_port number of the cameras inside the [giraff launch file](missions/launch/giraff_launch.py) using `sudo dmesg | grep usb`
* Give the permission to serial ports `sudo adduser $USER dialout`
* Reboot the system
* Check the port of the hokuyo, it is on of `/dev/ttyAMC[0-9]`