#ifndef CARTESIANROBOTHW_H
#define CARTESIANROBOTHW_H

#include <cpr_robot.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <maxon_epos_driver/EposManager.hpp>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Vector3.h>

namespace cpr_robot{
    class CartesianRobotHW : public hardware_interface::RobotHW{
        public:
            CartesianRobotHW(const std::vector<std::string> &motor_names, ros::NodeHandle &robot_nh);
            ~CartesianRobotHW();
            bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
                const std::vector<std::string> &motor_names);
            void read(const ros::Time &time, const ros::Duration &period);
            // void write(const ros::Time &time, const ros::Duration &period);
            void write(const std::vector<double> &pos);
            void updateDiagnostics();
            void doSwitch();
            void reference_igus_motors();
            void ik(const geometry_msgs::Vector3ConstPtr &msg);
            void gotoHandler(const geometry_msgs::Vector3ConstPtr &msg);
            ros::ServiceServer m_goto_sewing_point_srv_;
            ros::ServiceServer m_avtivate_srv_;
            bool is_active_;
            bool SewingPointHandler(std_srvs::TriggerRequest  &req, std_srvs::TriggerResponse &res);
            bool SetActiveStateHandler(std_srvs::SetBoolRequest  &req, std_srvs::SetBoolResponse &res);
            void ik_calculations(double &x, double &y, double &z, bool absolute=false);
            void update_rosparam_values();

        private:
            void init_motors(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
                const std::vector<std::string> &motor_names);

            std::vector<std::string> motor_names_;
            CartesianCPR igus_robot;
            ros::NodeHandle root_nh_, robot_nh_;
            EposManager epos_manager_;
            // Instance of the Bus class that will be used to communicate with the firmware of the modules that are controlling the robot.
            Bus m_Bus;
            // Pointer to an array of instances of the Joint class. One entry per joint.
            // CartesianJoint** m_pJoints;
            Joint** m_pJoints;

            hardware_interface::JointStateInterface joint_state_interface_;
            hardware_interface::PositionJointInterface position_joint_interface_;
            hardware_interface::VelocityJointInterface velocity_joint_interface_;
            hardware_interface::EffortJointInterface effort_joint_interface_;

            std::vector<double> epos_joint_position_command_ = {0};
            std::vector<double> epos_joint_velocity_command_ = {0};
            std::vector<double> epos_joint_effort_command_ = {0};

            std::vector<double> igus_joint_position_command_ = {0, 0};
            std::vector<double> igus_joint_velocity_command_ = {0, 0};
            std::vector<double> igus_joint_effort_command_ = {0, 0};

            std::vector<double> epos_joint_position_ = {0};
            std::vector<double> epos_joint_velocity_ = {0};
            std::vector<double> epos_joint_current_ = {0};

            std::vector<double> igus_joint_position_ = {0, 0};
            std::vector<double> igus_joint_velocity_ = {0, 0};
            std::vector<double> igus_joint_current_ = {0, 0};

            ros::Subscriber new_command_sub_;   // for sewing points
            ros::Subscriber new_gotoPoint_sub_; // for cartesian target point
            ros::Publisher joint_state_pub_;

            // config values
            std::vector<double>  needle_, home_, joint_ratios_, sewing_point_, offsets_;
            double third_link_radius_;
    };
}

#endif