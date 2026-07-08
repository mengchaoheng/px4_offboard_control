This version adds finite-cycle mission termination.

New parameters:
- run_cycles:
  - 0.0 or smaller: continuous trajectory.
  - positive value: run this many Lissajous cycles.
- finish_hover_time:
  - seconds to hold the final trajectory point after the finite run.
- land_after_finish:
  - true: request land_mode after finish_hover_time.
  - false: hold the final point.
- land_mode:
  - PX4 mode string requested through /mavros/set_mode. Default: AUTO.LAND.

Recommended launch examples:

Continuous run:
<param name="run_cycles" value="0.0"/>

Run 3 cycles, hold 5 seconds, land:
<param name="run_cycles" value="3.0"/>
<param name="finish_hover_time" value="5.0"/>
<param name="land_after_finish" value="true"/>
<param name="land_mode" value="AUTO.LAND"/>

Run 3 cycles, hold final point:
<param name="run_cycles" value="3.0"/>
<param name="land_after_finish" value="false"/>

Copy files:
cp CircularTrajectory.h ~/catkin_ws/src/px4_offboard_control/include/my_offboard_node/CircularTrajectory.h
cp offb_node_pva.cpp ~/catkin_ws/src/px4_offboard_control/src/offb_node_pva.cpp
cp lissajous_pva.launch ~/catkin_ws/src/px4_offboard_control/launch/lissajous_pva.launch

Build:
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch px4_offboard_control lissajous_pva.launch
