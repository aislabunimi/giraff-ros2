# giraff-ros2

## Configure giraff
Install dependencies:
* Run the following commands: 
```shell
rosdep install --from-paths src -y --ignore-src
sudo apt install ros-humble-magic-enum libuvc-dev libgoogle-glog-dev nlohmann-json3-dev xterm
colcon build --symlink-install
```
* Install the udev rules of cameras, laser, and rfid running the `install.sh` script inside the `giraff_ros2_driver/scripts` folder
* Give the permission to serial ports `sudo adduser $USER dialout`
* Reboot the system
