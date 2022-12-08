#include "pr_utils/pr_wheel.h"
#include "pr_utils/interp1d.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <fmt/format.h>
#include <unistd.h> 
#include <vector>


double rpm_to_rad_per_sec(const double rpm) {
  return rpm * 2*M_PI / 60.0;
}

int fl_enc_prev = 0.0;
int fr_enc_prev = 0.0;
int rl_enc_prev = 0.0;
int rr_enc_prev = 0.0;
ros::Time last_command_time;

// std::vector<float> rpm_vec = { 100.0,  125.0,  150.0,  200.0,  250.0, 300.0, 350.0};
// std::vector<float> t_vec   = {293.66, 233.92, 187.86, 138.88, 112.85, 94.42, 81.25};
std::vector<float> rpm_vec = { 200.0,  400.0,  600.0,  800.0, 1000.0, 1250.0, 1638.0};
std::vector<float> t_vec   = {703.30, 352.08, 211.36, 151.08, 117.39,  96.06,  70.47};
std::vector<float> spd_vec;

int main(int argc, char** argv) {
  ros::init(argc, argv, "pitranger");
  ros::NodeHandle nh;

  // TODO(Jordan): Should these come from the launch file?
  const std::string wheel_odom_frame_id = "odom";
  const std::string wheel_odom_child_frame_id = "base_link";
  const double wheel_rad_m = 0.0955;
  const double wheel_spacing_m = 0.30; // Morphin 2W = wheel-base

  // populate speed vector for interpolation
  for (int i = 0; i < t_vec.size(); i++) {
    // circumference (m) / time/rev (s)
    spd_vec.push_back((wheel_rad_m * M_PI * 2.0) / (t_vec[i] / 10.0));
  } 

  pr::WheelController wheels;

  // Attach a subscriber to set wheel velocities.
  auto wheel_cb = [&wheels, wheel_rad_m, wheel_spacing_m]
    (const geometry_msgs::TwistConstPtr& msg) {
    try {
      double lin = msg->linear.x;   // m/s
      double ang = msg->angular.z;  // rad/s

      double v_delta = ang*wheel_spacing_m;
      double l_mps = lin - v_delta;
      double r_mps = lin + v_delta;
      std::cout << "setting rpm: " << l_mps << ", " << r_mps << std::endl;

      // double l_rpm = (l_mps / (wheel_rad_m)) * (60.0 / (2.0 * M_PI));    // rad/s -> rpm
      // double r_rpm = (r_mps / (wheel_rad_m)) * (60.0 / (2.0 * M_PI));    // rad/s -> rpm

      double l_rpm = interp1d(spd_vec, rpm_vec, abs(l_mps));
      double r_rpm = interp1d(spd_vec, rpm_vec, abs(r_mps));

      if (!signbit(l_mps)) {l_rpm *= -1.0;}
      if (!signbit(r_mps)) {r_rpm *= -1.0;}

      // double l_rpm = 1250.0;
      // double r_rpm = 1250.0;
      wheels.set_right_rpm(r_rpm);
      wheels.set_left_rpm(-l_rpm);
      std::cout << "setting rpm: " << -l_rpm << ", " << r_rpm << std::endl;
    } catch(const std::exception& e) {
      fmt::print("WARN: pitranger node failed to set motor velocities.\n");
    }
  };
  // Attach a subscriber to set wheel velocities from CFS.
  auto cfs_wheel_cb = [&wheels, wheel_rad_m, wheel_spacing_m]
    (const sensor_msgs::JoyConstPtr& msg) {
    try {
      last_command_time = ros::Time::now();
      // std::cout << "joy time" << last_command_time.toSec() << "\n";
      // joy_msg * (0.0948 / 0.0654) * 590
      double factor_to_rpm = ((0.0948 / 0.0654) / 2) * (60 / (2*M_PI));
      // FL = RL & RL = RR
      double fl_rpm = msg->axes[1] * factor_to_rpm * 180;
      double fr_rpm = msg->axes[2] * factor_to_rpm * 180;
      double rl_rpm = msg->axes[3] * factor_to_rpm * 180;
      double rr_rpm = msg->axes[4] * factor_to_rpm * 180;

      std::cout << "time diff: " << ros::Time::now().toSec() - last_command_time.toSec() << std::endl;

      std::cout << "FL_RPM & FR_RPM: " << fl_rpm << ", " << rl_rpm << std::endl;  
      std::cout << "RL_RPM & RR_RPM: " << -fr_rpm << ", " << -rr_rpm << std::endl; 
      wheels.set_left_rpm(fl_rpm);
      wheels.set_right_rpm(-fr_rpm);
    } catch(const std::exception& e) {
      fmt::print("WARN: pitranger node failed to set motor velocities.\n");
    }
  };
  auto wheel_sub = nh.subscribe<geometry_msgs::Twist>("/pitranger/in/twist_cmd", 1, wheel_cb);
  auto wheel_vel_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 1, cfs_wheel_cb);

  // Create a publisher for the encoders.
  auto encoder_pub = nh.advertise<std_msgs::Float64MultiArray>("/encoders", 100);
  auto wheels_pub = nh.advertise<std_msgs::Int32MultiArray>("/wheels", 100);
  auto wheels_vel_pub = nh.advertise<std_msgs::Float32MultiArray>("/wheel_velocity", 100);

  // Track the robot state in x,y,yaw.
  double robot_x   = 0.0;
  double robot_y   = 0.0;
  double robot_yaw = 0.0;

  ros::WallTime prev_iter_time = ros::WallTime::now();
  ros::WallTime curr_iter_time = prev_iter_time;
  
  ros::Rate rate(10);
  while( ros::ok() ) {
    // Compute wheel odometry and publish it.
    try {
      // std::vector<int> rpms = wheels.get_rpm();
      // int fl_rpm = rpms[0];
      // int fr_rpm = rpms[1];
      // int rl_rpm = rpms[2];
      // int rr_rpm = rpms[3];

      std::vector<int> abs_encs = wheels.get_abs_encs();
      int fl_abs_enc = abs_encs[0];
      int fr_abs_enc = abs_encs[1] * -1.0;
      int rl_abs_enc = abs_encs[2];
      int rr_abs_enc = abs_encs[3] * -1.0;

      std::vector<int> rel_encs = wheels.get_rel_encs();
      int fl_rel_enc = rel_encs[0];
      int fr_rel_enc = rel_encs[1] * -1.0;
      int rl_rel_enc = rel_encs[2];
      int rr_rel_enc = rel_encs[3] * -1.0;

      /** Get elapsed time since previous iteration. **/
      curr_iter_time = ros::WallTime::now();
      const double dt = (curr_iter_time-prev_iter_time).toNSec() * 1.0e-9;
      prev_iter_time = curr_iter_time;

      // const int fr_enc = -fr_rpm / 60 * dt * 7;
      // const int fl_enc = fl_rpm / 60 * dt * 7;
      // const int rr_enc = -rr_rpm / 60 * dt * 7;
      // const int rl_enc = rl_rpm / 60 * dt * 7;

      /** Construct encoder message & populate it. **/
      std_msgs::Float64MultiArray enc_msg;
      enc_msg.data.push_back(curr_iter_time.sec);
      enc_msg.data.push_back(curr_iter_time.nsec);
      enc_msg.data.push_back(dt);
      enc_msg.data.push_back(fl_rel_enc); // Relative  
      enc_msg.data.push_back(fr_rel_enc);
      enc_msg.data.push_back(rl_rel_enc);
      enc_msg.data.push_back(rr_rel_enc);      
      enc_msg.data.push_back(fl_abs_enc); // Abs Encoder Ticks 
      enc_msg.data.push_back(fr_abs_enc);
      enc_msg.data.push_back(rl_abs_enc);
      enc_msg.data.push_back(rr_abs_enc);        

      /**  Wheels Encoder msg for CFS **/
      std_msgs::Int32MultiArray wheels_msg;
      wheels_msg.data.push_back(fl_rel_enc); // Encoder Ticks 
      wheels_msg.data.push_back(fr_rel_enc);
      wheels_msg.data.push_back(rl_rel_enc);
      wheels_msg.data.push_back(rr_rel_enc);
      std::cout << "Encoder Values --- : " << dt 
                // << " FL: " << fl_enc
                // << " FR: " << fr_enc   
                // << " RL:: " << rl_enc   
                // << " RR: " << rr_enc 
                // << " ---- ABS Encoder " 
                // << " FL: " << fl_abs_enc
                // << " FR: " << fr_abs_enc   
                // << " RL:: " << rl_abs_enc   
                // << " RR: " << rr_abs_enc 
                << " ---- REL Encoder " 
                << " FL: " << fl_rel_enc
                << " FR: " << fr_rel_enc   
                << " RL:: " << rl_rel_enc   
                << " RR: " << rr_rel_enc  
                << "\n";

      // Wheels Vel msg for CFS (just need it to exist)
      std_msgs::Float32MultiArray wheels_vel_msg;
      wheels_vel_msg.data.push_back(2 * M_PI * ((fl_rel_enc / dt) / 5281.0));
      wheels_vel_msg.data.push_back(2 * M_PI * ((fr_rel_enc / dt) / 5281.0));
      wheels_vel_msg.data.push_back(2 * M_PI * ((rl_rel_enc / dt) / 5281.0));
      wheels_vel_msg.data.push_back(2 * M_PI * ((rr_rel_enc / dt) / 5281.0));

      // Publish
      encoder_pub.publish(enc_msg);
      wheels_pub.publish(wheels_msg);
      wheels_vel_pub.publish(wheels_vel_msg);

      // If we recieve no new commands, we stop the motors
      // std::cout << ros::Time::now().toSec() - last_command_time.toSec() << std::endl;
      if (ros::Time::now().toSec() - last_command_time.toSec() > 5.1) {
        wheels.set_left_rpm(0.0);
        wheels.set_right_rpm(0.0);        
      } 
    } catch (const std::exception& e) {
      fmt::print("WARNING: {}", e.what());
    }

    ros::spinOnce();
    rate.sleep();
  }

  wheels.set_left_rpm(0.0);
  wheels.set_right_rpm(0.0);  
  return 0;
}
