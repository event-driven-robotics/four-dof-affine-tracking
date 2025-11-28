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
#include <Eigen/Geometry>

namespace franka_example_controllers {
ros::Subscriber sub_;
ros::Publisher pub_, pub2_;
void sharedDataCallback (const sharedmessage::ConstPtr &msg);
double x_object, y_object, z_object, roll_object, pitch_object, yaw_object;
double qx_object, qy_object, qz_object, qw_object;
double v_y_filtered = 0;
double v_x_filtered = 0;
double v_z_filtered = 0;
double v_yaw_filtered = 0;
std::unique_ptr< franka_hw::FrankaStateHandle > state_handle_;
Eigen::Matrix4d Tref;
Eigen::Matrix4d Tcam_ee;

Eigen::Matrix4d buildReferenceTransform(double z_des = 0.3) {
    Eigen::Matrix4d Tstar = Eigen::Matrix4d::Identity();
    Tstar(0,3) = 0.0;   // x
    Tstar(1,3) = 0.0;   // y
    Tstar(2,3) = -z_des; // z
    return Tstar;
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

  x_object = - msg->content[0];
  y_object = - msg->content[1];
  z_object = msg->content[2];
  // roll_object = msg -> content[3];
  // pitch_object = msg -> content[4];
  // yaw_object = msg -> content[5];

  qx_object = msg -> content[3];
  qy_object = msg -> content[4];
  qz_object = msg -> content[5];
  qw_object = msg -> content[6];

}
// void sharedDataCallback (const geometry_msgs::Pose msg)
// {
//   ROS_INFO("x: [%.2f]", msg.position.x);
// }

// Converts position + quaternion to 4x4 homogeneous transform
Eigen::Matrix4d poseToTransform_quaternion(
        double x, double y, double z,
        double qx, double qy, double qz, double qw)
{
    // Create Eigen quaternion
    Eigen::Quaterniond q(qw, qx, qy, qz); // Note: Eigen(qw, qx, qy, qz)
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();

    // Build homogeneous transform
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = R;
    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;

    return T;
}

Eigen::Matrix4f poseToTransform_euler(float x, float y, float z,
                                float roll, float pitch, float yaw)
{
    // Rotation matrices
    float cr = cos(roll);   float sr = sin(roll);
    float cp = cos(pitch);  float sp = sin(pitch);
    float cy = cos(yaw);    float sy = sin(yaw);

    // Rx(roll)
    Eigen::Matrix3f Rx;
    Rx << 1, 0, 0,
          0, cr, -sr,
          0, sr,  cr;

    // Ry(pitch)
    Eigen::Matrix3f Ry;
    Ry <<  cp, 0, sp,
            0, 1, 0,
          -sp, 0, cp;

    // Rz(yaw)
    Eigen::Matrix3f Rz;
    Rz << cy, -sy, 0,
          sy,  cy, 0,
           0,   0, 1;

    // Combined rotation: R = Rz * Ry * Rx
    Eigen::Matrix3f R = Rz * Ry * Rx;

    // Build homogeneous transform
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = R;
    T(0,3) = x;
    T(1,3) = y;
    T(2,3) = z;

    return T;
}


// Compute rotation vector (so3 logarithm)
Eigen::Vector3d so3Log(const Eigen::Matrix3d& R) {
    double cos_theta = (R.trace() - 1.0) * 0.5;
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));
    double theta = std::acos(cos_theta);

    if (theta < 1e-6)
        return Eigen::Vector3d::Zero();

    Eigen::Vector3d omega;
    omega << R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1);
    omega *= 0.5 / std::sin(theta);
    return theta * omega;
}

// PBVS with separate translation & rotation error
Eigen::Matrix<double,6,1> computePBVS_separate(
        const Eigen::Matrix4d& T_desired,
        const Eigen::Matrix4d& T_current,
        const Eigen::Matrix4d& T_root_ee,
        double Kt = 0.8, double Kr = 0.5)
{
    // Extract rotation and translation
    // Eigen::Matrix3d R_d = T_desired.block<3,3>(0,0);
    // Eigen::Matrix3d R_c = T_current.block<3,3>(0,0);

    // Eigen::Vector3d p_d = T_desired.block<3,1>(0,3);
    // Eigen::Vector3d p_c = T_current.block<3,1>(0,3);

    Eigen::Matrix4d T_error_cam;
    T_error_cam = T_current*T_desired.inverse();

    // // Translation error (desired - current)
    // Eigen::Vector3d e_t = p_c - p_d;

    // // Rotation error
    // Eigen::Matrix3d R_e = R_d.transpose() * R_c; // current w.r.t desired
    // Eigen::Vector3d e_r = so3Log(R_e);

    // Eigen::Matrix4d T_error_cam;
    // T_error_cam.block<3,1>(0,3) = e_t;
    // T_error_cam.block<3,3>(0,0) = R_e;

    // T_error_cam.block<1,4>(3,0) << 0, 0, 0, 1;

    Eigen::Matrix4d T_error_root = T_root_ee*T_error_cam;

    std::cout << "T_error_root" << std::endl <<T_error_root << std::endl << std::endl;
    std::cout << "T_error_cam" << std::endl <<T_error_cam << std::endl << std::endl;
    std::cout << "T_root_ee" << std::endl <<T_root_ee << std::endl << std::endl;

    Eigen::Vector3d p_root = T_error_root.block<3,1>(0,3);
    Eigen::Matrix3d R_root = T_error_root.block<3,3>(0,0);
    Eigen::Vector3d e_r_root = so3Log(R_root);

    // std::cout <<"tr error" <<p_root<<std::endl;
    // std::cout<< "rot error"<<e_r_root<<std::endl;

    // Apply separate gains
    Eigen::Matrix<double,6,1> xi;
    xi.segment<3>(0) = Kt * p_root; // linear velocity
    xi.segment<3>(3) = Kr * e_r_root; // angular velocity
    return xi;
}

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

  double rads = 180 * M_PI / 180.0;
  Eigen::Matrix4d Ry;
  Ry <<  cos(rads),0, sin(rads),0,
         0,1,0,0,
              -sin(rads),0,cos(rads),0,
              0,0,0,1;

  rads = -90 * M_PI / 180.0;
  Eigen::Matrix4d Rz;
  Rz << cos(rads), -sin(rads), 0, 0,
        sin(rads),  cos(rads), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Tcam_ee = Rz * Ry;

  Tref = buildReferenceTransform(0.3);

  sub_ = node_handle.subscribe("/foo2/sharedmessage", 1, sharedDataCallback);
  pub_ = node_handle.advertise<geometry_msgs::Pose>("/ee_pose", 1);
  pub2_ = node_handle.advertise<std_msgs::Float64MultiArray>("/errors_and_velocities", 1);

  // ///////////////////////////////////////////////
  // double lambda_t = 0.8;
  // double lambda_r = 0.5;

  // Eigen::Matrix4d Tee_root_init;
  // Tee_root_init <<  1,0,0,0,
  //                   0,1,0,0,
  //                   0,0,1,0,
  //                   0,0,0,1;


  // for (double x=-0.1; x<0.10; x+=0.02){

  //   Eigen::Matrix4d Tobject = poseToTransform_quaternion(0, 0, 0.5, 0, sin(x*0.5), 0, cos(x*0.5));

  //   Eigen::Matrix<double,6,1> v = computePBVS_separate(Tref, Tobject, Tee_root_init, lambda_t, lambda_r);

  //   std::cout << "x = " << x << std::endl;
  //   std::cout << "T = " << Tobject << std::endl<<std::endl;
  //   std::cout << "v = "<< v << std::endl;
  //   std::cout << "------------------" << std::endl;

  // }
  // return false;
  // ///////////////////////////////////////////////


  return true;
}

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

  // double filter =.005;//.005
  // double filter_z =.05;
  // double filter_ang =1;

  // double Kv_y = 1./100;
  // double Kv_x = 1./200;
  // double Kv_z = 1./20;
  // double Kv_yaw = 1.2;


  double lambda_t = 0.8;
  double lambda_r = 0.0;

  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Matrix4d T_root_ee = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());

  T_root_ee.block<3,1>(0,3) << 0,0,0;

  Eigen::Matrix4d T_root_cam = T_root_ee*Tcam_ee ;

  //std::cout << "T_root_ee="<<T_root_ee << std::endl;

  //std::cout << "T_root_cam="<<T_root_cam << std::endl;

  Eigen::Matrix4d Tobject = poseToTransform_quaternion(x_object, y_object, z_object, qx_object, qy_object, qz_object, qw_object);
  // Eigen::Matrix4d Tobject = poseToTransform_quaternion(0.01, 0, 0.3, 1, 0, 0, 0);
  Eigen::Matrix<double,6,1> v = computePBVS_separate(Tref, Tobject, T_root_cam, lambda_t, lambda_r);

  double vx = v(0);   // linear x
  double vy = v(1);   // linear y
  double vz = v(2);   // linear z

  double wx = v(3);   // angular x
  double wy = v(4);   // angular y
  double wz = v(5);   // angular z


  std::cout << "v="<<v<<std::endl;

  std::array<double, 6> command = {{vx, vy, vz, wx, wy, wz}};
  // std::array<double, 6> command = {{vz, -vx, -vy, wz, -wx, -wy}};
  velocity_cartesian_handle_->setCommand(command);

  // franka::RobotState robot_state = state_handle_->getRobotState();
  // Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  // Eigen::Vector3d position(transform.translation());
  // Eigen::Quaterniond orientation(transform.rotation());

  // geometry_msgs::Pose pose;

  // pose.position.x = position.head(3)[0];
  // pose.position.y = position.head(3)[1];
  // pose.position.z = position.head(3)[2];

  // pose.orientation.x = orientation.toRotationMatrix().eulerAngles(2, 1, 0)[0]; //yaw
  // pose.orientation.y = orientation.toRotationMatrix().eulerAngles(2, 1, 0)[1]; //pitch
  // pose.orientation.z = orientation.toRotationMatrix().eulerAngles(2, 1, 0)[2]; //roll

  // pub_.publish(pose);

  // std_msgs::Float64MultiArray deltas_msg;
  // std_msgs::Float64MultiArray vel_msg;

  // std::vector<double> vec1 = {delta_u_,delta_v_, delta_z_, delta_theta_, v_x_filtered, v_y_filtered, v_z_filtered, v_yaw_filtered, Kv_y, Kv_x, Kv_z, Kv_yaw, filter, filter_z, filter_ang};

  // deltas_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  // deltas_msg.layout.dim[0].size = vec1.size();
  // deltas_msg.layout.dim[0].stride = 1;
  // deltas_msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

  // // copy in the data
  // deltas_msg.data.clear();
  // deltas_msg.data.insert(deltas_msg.data.end(), vec1.begin(), vec1.end());

  // pub2_.publish(deltas_msg);

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