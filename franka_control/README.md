In the Panda robot workstation:
```
export ROS_MASTER_URI=http://192.168.100.10:11311
roscore
```
```
cd $ROBOT_CODE
cd robot_ws/src
catkin make
source devel/setup.bash
```
```
roslaunch franka_example_controllers cartesian_velocity_controller.launch robot_ip=172.16.0.2 load_gripper:=false
```

In Luna Dell XPS laptop (inside Docker):
```
yarp conf 192.168.100.10 10000
yarpserver --ros
./four-dof-tracking
```
