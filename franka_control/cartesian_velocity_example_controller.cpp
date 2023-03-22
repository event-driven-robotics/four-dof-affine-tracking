// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/cartesian_velocity_example_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>

#include <Eigen/Dense>

namespace franka_example_controllers {
ros::Subscriber sub_;
ros::Publisher pub_, pub2_;
void sharedDataCallback (const sharedmessage::ConstPtr &msg);
double delta_u_, delta_v_, delta_z_, delta_theta_;
double v_y_filtered = 0;
double v_x_filtered = 0;
double v_z_filtered = 0;
double v_yaw_filtered = 0;
std::unique_ptr< franka_hw::FrankaStateHandle > state_handle_;

bool CartesianVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ =
      robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr) {
    ROS_ERROR(
        "CartesianVelocityExampleController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "CartesianVelocityExampleController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("CartesianVelocityExampleController: Could not get state interface from hardware");
    return false;
  }
  state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  // try {
  //   auto state_handle = state_interface->getHandle(arm_id + "_robot");

  //   std::array<double, 7> q_start = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  //   for (size_t i = 0; i < q_start.size(); i++) {
  //     if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
  //       ROS_ERROR_STREAM(
  //           "CartesianVelocityExampleController: Robot is not in the expected starting position "
  //           "for running this example. Run `roslaunch franka_example_controllers "
  //           "move_to_start.launch robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` "
  //           "first.");
  //       return false;
  //     }
  //   }
  // } catch (const hardware_interface::HardwareInterfaceException& e) {
  //   ROS_ERROR_STREAM(
  //       "CartesianVelocityExampleController: Exception getting state handle: " << e.what());
  //   return false;
  // }

  sub_ = node_handle.subscribe("/foo2/sharedmessage", 1, sharedDataCallback);
  pub_ = node_handle.advertise<geometry_msgs::Pose>("/ee_pose", 1);
  pub2_ = node_handle.advertise<std_msgs::Float64MultiArray>("/errors_and_velocities", 1);

  return true;
}

void CartesianVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);

}

void sharedDataCallback (const sharedmessage::ConstPtr &msg)
{
  // ROS_INFO("x: [%.2f]", msg->content[0]);
  // ROS_INFO("y: [%.2f]", msg->content[1]);
  // ROS_INFO("vx: [%.2f]", msg->content[2]);
  // ROS_INFO("vy: [%.2f]", msg->content[3]);

  delta_u_ = - msg->content[0];
  delta_v_ = - msg->content[1];
  delta_theta_ = msg->content[2];
  delta_z_= msg -> content[3];

}
// void sharedDataCallback (const geometry_msgs::Pose msg)
// {
//   ROS_INFO("x: [%.2f]", msg.position.x);
// }

void CartesianVelocityExampleController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
  elapsed_time_ += period;

  // double time_max = 4.0;
  // double v_max = 0.05;
  // double angle = M_PI / 4.0;
  // double cycle = std::floor(
  //     pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max)) / time_max));
  // double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * elapsed_time_.toSec()));
  // double v_x = std::cos(angle) * v;
  // double v_z = -std::sin(angle) * v;

  double filter =.005;//.005
  double filter_z =.05;
  double filter_ang =1;

  double Kv_y = 1./100;
  double Kv_x = 1./200;
  double Kv_z = 1./20;
  double Kv_yaw = 1.2;

  double v_y = Kv_y * delta_u_; // v_y coorresponds to -v in the image plane
  double v_x= Kv_x * delta_v_; // v_x corresponds to +u in the image plane
  double v_z= Kv_z * delta_z_; // v_x corresponds to +u in the image plane
  double v_yaw = Kv_yaw * delta_theta_;

  v_y_filtered = filter*v_y +(1-filter)*v_y_filtered;
  v_x_filtered = filter*v_x +(1-filter)*v_x_filtered;
  v_z_filtered = filter_z*v_z +(1-filter)*v_z_filtered;
  v_yaw_filtered = filter_ang*v_yaw +(1-filter_ang)*v_yaw_filtered;
  // if (elapsed_time_.toSec()<5){
  //   v_y_filtered = filter*v_y +(1-filter)*v_y_filtered;
  // } else{
  //   v_y_filtered=v_y;
  // }

  v_yaw_filtered = (v_yaw_filtered*M_PI)/180;
  v_yaw = (v_yaw*M_PI)/180;

  // std::cout<<"delta scale"<<delta_z_;
  // std::cout<<"v_z_filtered: "<<v_y_filtered<<std::endl;
  // std::cout<<"v_z: "<<v_y<<std::endl;
  // std::cout<<"v_y_filtered: "<<v_y_filtered<<std::endl;
  // std::cout<<"v_y: "<<v_y<<std::endl;
  std::cout<<"v_yaw_filtered: "<<v_yaw_filtered<<std::endl;
  std::cout<<"v_yaw: "<<v_yaw<<std::endl;
  std::cout<<std::endl;

  std::array<double, 6> command = {{0.0, v_y_filtered, 0.0, 0.0, 0.0, 0.0}};
  velocity_cartesian_handle_->setCommand(command);

  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  geometry_msgs::Pose pose;

  pose.position.x = position.head(3)[0];
  pose.position.y = position.head(3)[1];
  pose.position.z = position.head(3)[2];

  pose.orientation.x = orientation.toRotationMatrix().eulerAngles(2, 1, 0)[0]; //yaw
  pose.orientation.y = orientation.toRotationMatrix().eulerAngles(2, 1, 0)[1]; //pitch
  pose.orientation.z = orientation.toRotationMatrix().eulerAngles(2, 1, 0)[2]; //roll

  pub_.publish(pose);

  std_msgs::Float64MultiArray deltas_msg;
  std_msgs::Float64MultiArray vel_msg;

  std::vector<double> vec1 = {delta_u_,delta_v_, delta_z_, delta_theta_, v_x_filtered, v_y_filtered, v_z_filtered, v_yaw_filtered, Kv_y, Kv_x, Kv_z, Kv_yaw, filter, filter_z, filter_ang};

  deltas_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  deltas_msg.layout.dim[0].size = vec1.size();
  deltas_msg.layout.dim[0].stride = 1;
  deltas_msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

  // copy in the data
  deltas_msg.data.clear();
  deltas_msg.data.insert(deltas_msg.data.end(), vec1.begin(), vec1.end());

  pub2_.publish(deltas_msg);

  // ros::spinOnce();
}

void CartesianVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerBase)
