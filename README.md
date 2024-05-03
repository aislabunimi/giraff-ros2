# giraff-ros2

## Configure giraff
Install dependencies:
* Run the following commands: 
```shell
rosdep install --from-paths src -y --ignore-src
sudo apt install ros-humble-magic-enum libuvc-dev
colcon build --syml
```
* Install the udev rules of cameras running the `install.sh` script inside the astra_camera package