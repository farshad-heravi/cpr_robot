#pragma once
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <maxon_epos_driver/EposManager.hpp>

// class CartesianRobotHW : public hardware_interface::RobotHW{
//     public:
//         CartesianRobotHW();
//         ~CartesianRobotHW();
//         bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
//             const std::vector<std::string> &motor_names);
//         void read(const ros::Time &time, const ros::Duration &period);
//         void write(const ros::Time &time, const ros::Duration &period);
//         void updateDiagnostics();

//     private:
//         void init_motors(ros::NodeHandle &root_nh_, ros::NodeHandle &robot_nh_,
//             const std::vector<std::string> &motor_names);

//         ros::NodeHandle root_nh_, robot_nh_;
//         EposManager epos_manager_;
//         // Instance of the Bus class that will be used to communicate with the firmware of the modules that are controlling the robot.
//         // cpr_robot::Bus m_Bus;
//         // Pointer to an array of instances of the Joint class. One entry per joint.
//         // cpr_robot::Joint** m_pJoints;

//         hardware_interface::JointStateInterface joint_state_interface_;
//         hardware_interface::PositionJointInterface position_joint_interface_;
//         hardware_interface::VelocityJointInterface velocity_joint_interface_;
//         hardware_interface::EffortJointInterface effort_joint_interface_;

//         std::vector<double> epos_joint_position_command_ = {0};
//         std::vector<double> epos_joint_velocity_command_ = {0};
//         std::vector<double> epos_joint_effort_command_ = {0};

//         std::vector<double> igus_joint_position_command_ = {0, 0};
//         std::vector<double> igus_joint_velocity_command_ = {0, 0};
//         std::vector<double> igus_joint_effort_command_ = {0, 0};

//         std::vector<double> epos_joint_position_ = {0};
//         std::vector<double> epos_joint_velocity_ = {0};
//         std::vector<double> epos_joint_current_ = {0};

//         std::vector<double> igus_joint_position_ = {0, 0};
//         std::vector<double> igus_joint_velocity_ = {0, 0};
//         std::vector<double> igus_joint_current_ = {0, 0};
// };