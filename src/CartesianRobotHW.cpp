#include <boost/foreach.hpp>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <maxon_epos_driver/EposManager.hpp>
#include <cpr_robot.h>
#include <boost/foreach.hpp>


using namespace std;

namespace cpr_robot{
    CartesianRobotHW::CartesianRobotHW()
        : m_Bus("can0")
        {}
    CartesianRobotHW::~CartesianRobotHW(){}

    bool CartesianRobotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
                const std::vector<std::string> &motor_names)
    {
        // ! N.B.: Motor names order MATTERS! => [igus motor1's name, igus motor2's name, Maxon motor's name]
        root_nh_ = root_nh;
        robot_nh_ = robot_nh;

        // wait for URDF which contains transmission and limits information
        std::string urdf_str;

        // get cartesian_robot_name from rosparam
        std::string cartesian_robot_name;
        if(root_nh_.hasParam("cartesian_robot_name"))
        {
            root_nh_.param<std::string>("cartesian_robot_name", cartesian_robot_name, "cartesian_");
        }
        root_nh_.getParam("/"+cartesian_robot_name+"/robot_description", urdf_str);   // ? check if param name is correct for cartesian robot
        while (urdf_str.empty() && ros::ok()) {
            ROS_INFO_STREAM_ONCE("Waiting for robot_description");
            root_nh_.getParam("robot_description", urdf_str);
            ros::Duration(0.1).sleep();
        }

        // init igus parts
        igus_robot.Init(motor_names);

        // init maxon motor
        init_motors(root_nh_, robot_nh_, motor_names);

        // int i=0;
        // BOOST_FOREACH(const std::string &motor_name, motor_names)
        // {   
        //     if (i==0 || i==1){
        //         // igus motors
        //         // create joint_state_interface the motor
        //         hardware_interface::JointStateHandle joint_state_handle(motor_name, &igus_joint_position_[i], &igus_joint_velocity_[i], &igus_joint_current_[i]);
        //         joint_state_interface_.registerHandle(joint_state_handle);

        //         // Create position joint interface for the motor
        //         hardware_interface::JointHandle joint_position_handle(joint_state_handle, &igus_joint_position_command_[i]);
        //         position_joint_interface_.registerHandle(joint_position_handle);
        //         // Create velocity joint interface for the motor
        //         hardware_interface::JointHandle joint_velocity_handle(joint_state_handle, &igus_joint_velocity_command_[i]);
        //         velocity_joint_interface_.registerHandle(joint_velocity_handle);
        //         // Create effort joint interface for the motor
        //         hardware_interface::JointHandle joint_effort_handle(joint_state_handle, &igus_joint_effort_command_[i]);
        //         effort_joint_interface_.registerHandle(joint_effort_handle);
        //     } else {
        //         // Epos motor
        //         // create joint_state_interface the motor
        //         i = 0;
        //         hardware_interface::JointStateHandle joint_state_handle(motor_name, &epos_joint_position_[i], &epos_joint_velocity_[i], &epos_joint_current_[i]);
        //         joint_state_interface_.registerHandle(joint_state_handle);

        //         // Create position joint interface for the motor
        //         hardware_interface::JointHandle joint_position_handle(joint_state_handle, &epos_joint_position_command_[i]);
        //         position_joint_interface_.registerHandle(joint_position_handle);
        //         // Create velocity joint interface for the motor
        //         hardware_interface::JointHandle joint_velocity_handle(joint_state_handle, &epos_joint_velocity_command_[i]);
        //         velocity_joint_interface_.registerHandle(joint_velocity_handle);
        //         // Create effort joint interface for the motor
        //         hardware_interface::JointHandle joint_effort_handle(joint_state_handle, &epos_joint_effort_command_[i]);
        //         effort_joint_interface_.registerHandle(joint_effort_handle);
        //     }

        //     // TODO: joint limits and saturations also can be added; Do not remember to register them in the below section
        //     i++;
        // }

        // // register all joint interfaces
        // registerInterface(&joint_state_interface_);
        // registerInterface(&position_joint_interface_);
        // registerInterface(&velocity_joint_interface_);
        // registerInterface(&effort_joint_interface_);
        
        return true;
    }
    void CartesianRobotHW::read(const ros::Time &time, const ros::Duration &period){
        // read actutor states
    // epos_manager_.read(epos_joint_position_, epos_joint_velocity_, epos_joint_current_);
    // FIXME uncomment above

        // TEST
        // read igus motors
        igus_robot.Read(igus_joint_position_, igus_joint_velocity_, igus_joint_current_);
        ROS_INFO_STREAM("READ: " << igus_joint_position_[0]);
    }
    void CartesianRobotHW::write(const ros::Time &time, const ros::Duration &period){
        // TODO do some safety filter here [saturation, limit, etc]

        // write igus motors
        // TEST
        igus_robot.Write(igus_joint_position_command_, igus_joint_velocity_command_, igus_joint_effort_command_);

        // write epos motor
    // epos_manager_.write(epos_joint_position_command_, epos_joint_velocity_command_, epos_joint_effort_command_);
    }
    void CartesianRobotHW::updateDiagnostics(){}    // TODO
    void CartesianRobotHW::doSwitch(){}             // TODO

    void CartesianRobotHW::init_motors(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
                const std::vector<std::string> &motor_names)
    {
        // third motor will be handled by epos4
        // FIXME uncomment :
    //  std::vector< std::string > epos_motors = {motor_names[2]};
    //  epos_manager_.init(root_nh, robot_nh, epos_motors);
    }
}