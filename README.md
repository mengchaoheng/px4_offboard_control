# px4_offboard_control

## Installation
1. mavros:
```
sudo apt-get install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
```
2. GeographicLib (opens new window)datasets
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```
3. offboard ros pkg.
```
cd ~/catkin_ws/src
git clone  https://github.com/mengchaoheng/px4_offboard_control.git
```

## Usage
1. Run PX4
```ConSole
cd <PX4 directory>
make px4_sitl gazebo
```
2. Run QGC.

3. Run mavros:
```
cd ~/catkin_ws
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
4. Run offboard node:
```
cd ~/catkin_ws
roslaunch my_offboard_node offb_node.launch
```

=============================
## Another way:

### Run on different computer:

1. On computer which ip is 192.168.3.146, run:
```ConSole
cd <PX4 directory>
make px4_sitl gazebo
```
(Optional) If run QGC on another computer, you can setup ip in `px4-rc.mavlink`:
```cpp
# GCS link
mavlink start -x -u $udp_gcs_port_local -r 4000000 -f -t 192.168.3.169  #  IP of QGC, in the same LAN

```


2. Run QGC, maybe on computer which ip is 192.168.3.169.

3. Run mavros:

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

(Optional) Or on computer different from the one running px4_sitl, run mavros by:
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.3.146:14557"
```
in which 192.168.3.146 is the ip of running `make px4_sitl gazebo`, and on that computer, you can setup ip in `px4-rc.mavlink`:
```cpp
# API/Offboard link
mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote -t 192.168.3.184 #  IP of mavros and offboard node
```
in which 192.168.3.184 is the ip of running mavros and offboard node.
If all running in the same computer, delete `-t 192.168.3.184` in mavlink start of API/Offboard link, and then run offboard node.

4.run offboard node
in the same computer which running mavros, run offboard node:
```
roslaunch my_offboard_node offb_node.launch
```
###  PX4 uer_guide:


Or just like PX4 uer_guide:
1. PX4 v1.14
```
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo-classic
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic

roslaunch px4 posix_sitl.launch
```
2.  mavros
```
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```

3. Offboard node
```
roslaunch my_offboard_node offb_node.launch
```