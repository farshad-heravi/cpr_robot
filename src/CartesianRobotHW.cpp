#include <boost/foreach.hpp>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <maxon_epos_driver/EposManager.hpp>
#include "Bus.cpp"
#include "CartesianJoint.cpp"

using namespace std;

class CartesianRobotHW : public hardware_interface::RobotHW{
    public:
        CartesianRobotHW();
        ~CartesianRobotHW();
        bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
            const std::vector<std::string> &motor_names);
        void read(const ros::Time &time, const ros::Duration &period);
        void write(const ros::Time &time, const ros::Duration &period);
        void updateDiagnostics();

    private:
        void init_motors(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
            const std::vector<std::string> &motor_names);

        ros::NodeHandle root_nh_, robot_nh_;
        EposManager epos_manager_;
        // Instance of the Bus class that will be used to communicate with the firmware of the modules that are controlling the robot.
        cpr_robot::Bus m_Bus;
        // Pointer to an array of instances of the Joint class. One entry per joint.
        CartesianJoint** m_pJoints;

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
};

CartesianRobotHW::CartesianRobotHW(){}
CartesianRobotHW::~CartesianRobotHW(){}

bool CartesianRobotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
            const std::vector<std::string> &motor_names)
{
    // ! N.B.: Motor names order MATTERS! => [igus motor1's name, igus motor2's name, Maxon motor's name]
    root_nh_ = root_nh;
    robot_nh_ = robot_nh;

    // wait for URDF which contains transmission and limits information
    std::string urdf_str;
    root_nh_.getParam("robot_description", urdf_str);   // ? check if param name is correct for cartesian robot
    while (urdf_str.empty() && ros::ok()) {
        ROS_INFO_STREAM_ONCE("Waiting for robot_description");
        root_nh_.getParam("robot_description", urdf_str);
        ros::Duration(0.1).sleep();
    }

    // Extract Motors from urdf_str
    init_motors(root_nh_, robot_nh_, motor_names);

    int i=0;
    BOOST_FOREACH(const std::string &motor_name, motor_names)
    {   
        if (i==0 || i==1){
            // igus motors
            // create joint_state_interface the motor
            hardware_interface::JointStateHandle joint_state_handle(motor_name, &igus_joint_position_[i], &igus_joint_velocity_[i], &igus_joint_current_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create position joint interface for the motor
            hardware_interface::JointHandle joint_position_handle(joint_state_handle, &igus_joint_position_command_[i]);
            position_joint_interface_.registerHandle(joint_position_handle);
            // Create velocity joint interface for the motor
            hardware_interface::JointHandle joint_velocity_handle(joint_state_handle, &igus_joint_velocity_command_[i]);
            velocity_joint_interface_.registerHandle(joint_velocity_handle);
            // Create effort joint interface for the motor
            hardware_interface::JointHandle joint_effort_handle(joint_state_handle, &igus_joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(joint_effort_handle);
        } else {
            // Epos motor
            // create joint_state_interface the motor
            i = 0;
            hardware_interface::JointStateHandle joint_state_handle(motor_name, &epos_joint_position_[i], &epos_joint_velocity_[i], &epos_joint_current_[i]);
            joint_state_interface_.registerHandle(joint_state_handle);

            // Create position joint interface for the motor
            hardware_interface::JointHandle joint_position_handle(joint_state_handle, &epos_joint_position_command_[i]);
            position_joint_interface_.registerHandle(joint_position_handle);
            // Create velocity joint interface for the motor
            hardware_interface::JointHandle joint_velocity_handle(joint_state_handle, &epos_joint_velocity_command_[i]);
            velocity_joint_interface_.registerHandle(joint_velocity_handle);
            // Create effort joint interface for the motor
            hardware_interface::JointHandle joint_effort_handle(joint_state_handle, &epos_joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(joint_effort_handle);
        }

        // TODO: joint limits and saturations also can be added; Do not remember to register them in the below section
        i++;
    }

    // register all joint interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&effort_joint_interface_);
    
    return true;
}
void CartesianRobotHW::read(const ros::Time &time, const ros::Duration &period){}
void CartesianRobotHW::write(const ros::Time &time, const ros::Duration &period){}
void CartesianRobotHW::updateDiagnostics(){}

void CartesianRobotHW::init_motors(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
            const std::vector<std::string> &motor_names)
{
    // first two motor will be handled by igus
    m_pJoints = new CartesianJoint*[2];
    for(int i=0; i<2; i++) {
        m_pJoints[i] = new CartesianJoint(m_Bus,(unsigned int)(i+1), root_nh, robot_nh, motor_names[i]);
        m_pJoints[i]->init(robot_nh);
    } // TODO

    // third motor will be handled by epos4
    std::vector< std::string > epos_motors = {motor_names[2]};
    epos_manager_.init(root_nh, robot_nh, epos_motors);
}