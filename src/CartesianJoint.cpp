
#include "Joint.cpp"
#include "../include/cpr_robot.h"
#include "MotorModule.cpp"

class CartesianJoint : public cpr_robot::Joint {
    public:
        CartesianJoint(cpr_robot::Bus &canBus, const uint32_t moduleId, 
                        ros::NodeHandle &root_nh, ros::NodeHandle &robotnh,
                        const std::string &joint_name);
        ~CartesianJoint();
        void init(ros::NodeHandle &robot_nh);
        void Read(double &pos, double &vel, double &cur);
        void Write(double override);
        
    protected:
        //! Pointer to an instance of the MotorModule class which represents the module and firmware driving the motor for the joint.
        cpr_robot::MotorModule* m_pModule;
        //! The name used for this joint when communicating over ROS topics and services.
        // std::string m_JointName;
};

CartesianJoint::CartesianJoint(cpr_robot::Bus &canBus, const uint32_t moduleId, 
                        ros::NodeHandle &root_nh, ros::NodeHandle &robotnh,
                        const std::string &joint_name)
    : cpr_robot::Joint(canBus, moduleId)
    // , m_JointName(joint_name)
{}
CartesianJoint::~CartesianJoint(){}

void CartesianJoint::init(ros::NodeHandle &robot_nh)
{
        cpr_robot::Joint::OnInit();

        // create Joint ros node handle
        ros::NodeHandle joint_nh = ros::NodeHandle(robot_nh, get_JointName());
        // Set Joint parameters from the rosparam
        // TODO get the parameters and set them in the class using its own functions
}

void CartesianJoint::Read(double &pos, double &vel, double &cur)
{
    OnRead();
    pos = get_CurrentPosition();
    vel = get_CurrentVelocity();
    cur = get_CurrentEffort();
}
void CartesianJoint::Write(double override)
{
    ROS_INFO_STREAM("WRITE");
}