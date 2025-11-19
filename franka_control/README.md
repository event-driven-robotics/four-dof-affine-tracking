In the Panda robot workstation:
```
export ROS_MASTER_URI=http://192.168.100.10:11311
roscore
```
```
cd $ROBOT_CODE
cd robot_ws/src
source devel/setup.bash
catkin build
```
```
roslaunch franka_example_controllers cartesian_velocity_controller.launch robot_ip:=172.16.0.2 load_gripper:=false
```
add into /etc/hosts 
```
iiticb005lw009u 192.168.100.164
```

In Luna's laptop (inside Docker):
```
export ROS_MASTER_URI=http://192.168.100.10:11311
yarp conf 192.168.100.164 10000
yarpserver --ros
./four-dof-tracking
```
