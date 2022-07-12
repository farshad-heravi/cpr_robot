#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <maxon_epos_driver/EposManager.hpp>
#include <cpr_robot.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <unistd.h>

using namespace std;

namespace cpr_robot{
    CartesianRobotHW::CartesianRobotHW(const std::vector<std::string> &motor_names, ros::NodeHandle &robot_nh)
        : m_Bus("can0")
        , motor_names_(motor_names)
        , is_active_(false)
        , test_with_hardware(true)  // will be removed later
        {
            int i=0;
            BOOST_FOREACH(const std::string &motor_name, motor_names)
            {   
                if (i==0 || i==1){
                    // igus motors
                    // create joint_state_interface the motor
                    hardware_interface::JointStateHandle joint_state_handle(motor_name, &igus_joint_position_[i], &igus_joint_velocity_[i], &igus_joint_current_[i]);
                    joint_state_interface_.registerHandle(joint_state_handle);

                    // // Create position joint interface for the motor
                    // hardware_interface::JointHandle joint_position_handle(joint_state_handle, &igus_joint_position_command_[i]);
                    // position_joint_interface_.registerHandle(joint_position_handle);
                    // // Create velocity joint interface for the motor
                    // hardware_interface::JointHandle joint_velocity_handle(joint_state_handle, &igus_joint_velocity_command_[i]);
                    // velocity_joint_interface_.registerHandle(joint_velocity_handle);
                    // // Create effort joint interface for the motor
                    // hardware_interface::JointHandle joint_effort_handle(joint_state_handle, &igus_joint_effort_command_[i]);
                    // effort_joint_interface_.registerHandle(joint_effort_handle);
                } else {
                    // Epos motor
                    // create joint_state_interface the motor
                    hardware_interface::JointStateHandle joint_state_handle(motor_name, &epos_joint_position_[0], &epos_joint_velocity_[0], &epos_joint_current_[0]);
                    joint_state_interface_.registerHandle(joint_state_handle);

                    // // Create position joint interface for the motor
                    // hardware_interface::JointHandle joint_position_handle(joint_state_handle, &epos_joint_position_command_[0]);
                    // position_joint_interface_.registerHandle(joint_position_handle);
                    // // Create velocity joint interface for the motor
                    // hardware_interface::JointHandle joint_velocity_handle(joint_state_handle, &epos_joint_velocity_command_[0]);
                    // velocity_joint_interface_.registerHandle(joint_velocity_handle);
                    // // Create effort joint interface for the motor
                    // hardware_interface::JointHandle joint_effort_handle(joint_state_handle, &epos_joint_effort_command_[0]);
                    // effort_joint_interface_.registerHandle(joint_effort_handle);
                }

                // TODO: joint limits and saturations also can be added; Do not remember to register them in the below section
                i++;
            }
            // register all joint interfaces
            registerInterface(&joint_state_interface_);
            // registerInterface(&position_joint_interface_);
            // registerInterface(&velocity_joint_interface_);
            // registerInterface(&effort_joint_interface_);
        }

    CartesianRobotHW::~CartesianRobotHW(){}

    bool CartesianRobotHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
                const std::vector<std::string> &motor_names)
    {
        // ! N.B.: Motor names order MATTERS! => [igus motor1's name, igus motor2's name, Maxon motor's name]
        root_nh_ = root_nh;
        robot_nh_ = robot_nh;
        update_rosparam_values();

        // wait for URDF which contains transmission and limits information
        std::string urdf_str;

        // get cartesian_robot_name from rosparam
        std::string cartesian_robot_name;
        root_nh_.param<std::string>("cartesian_robot_name", cartesian_robot_name, "cartesian_");
        
        root_nh_.getParam("/"+cartesian_robot_name+"/robot_description", urdf_str);
        while (urdf_str.empty() && ros::ok()) {
            ROS_INFO_STREAM("Waiting for robot_description");
            root_nh_.getParam("/"+cartesian_robot_name+"/robot_description", urdf_str);
            ros::Duration(0.1).sleep();
        }
        
        if (test_with_hardware)
            // init maxon motor
            init_motors(root_nh_, robot_nh_, motor_names);
        
        if (test_with_hardware)
            // init igus parts
            igus_robot.Init(motor_names);
        // reference_igus_motors();
        
        // initialize publisher and subscribers
        if(!root_nh_.hasParam("cartesian_traj_topicname"))
            ROS_ERROR_STREAM("cartesian_traj_topicname is not defined in rosparam.");
        std::string topicname;
        root_nh_.param<std::string>("cartesian_traj_topicname", topicname, "/cartesian_/TargetPoint");
        new_command_sub_ = root_nh_.subscribe(topicname, 10, &CartesianRobotHW::ik, this);
        new_gotoPoint_sub_ = root_nh_.subscribe("/"+cartesian_robot_name+"/goToPoint", 1, &CartesianRobotHW::gotoHandler, this);
        joint_state_pub_ = root_nh_.advertise<sensor_msgs::JointState>("/"+cartesian_robot_name+"/joint_states", 10);

        // initialize the services
        m_goto_sewing_point_srv_ = root_nh_.advertiseService("/CartesianGoToSewingPoint", &cpr_robot::CartesianRobotHW::SewingPointHandler, this);
        m_third_link_zero_srv_ = root_nh_.advertiseService("/Cartesian3rdLinkSetZero", &cpr_robot::CartesianRobotHW::ThirdLinkSetZero, this);
        m_avtivate_srv_ = root_nh_.advertiseService("/CartesianActivate", &cpr_robot::CartesianRobotHW::SetActiveStateHandler, this);
        return true;
    }
    void CartesianRobotHW::read(const ros::Time &time, const ros::Duration &period){
        sensor_msgs::JointState msg;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        msg.header = header;

        // read maxon motor state
        epos_manager_.read(epos_joint_position_, epos_joint_velocity_, epos_joint_current_);

        // read igus motors
        igus_robot.Read(igus_joint_position_, igus_joint_velocity_, igus_joint_current_);

        // fill in the joint state msg
        std::vector<double> pos_temp = {igus_joint_position_[0], igus_joint_position_[1], epos_joint_position_[0]/joint_ratios_[2]};
        std::vector<double> vel_temp = {igus_joint_velocity_[0], igus_joint_velocity_[1], epos_joint_velocity_[0]};
        // std::vector<double> eff_temp = {igus_joint_current_[0], igus_joint_current_[1], epos_joint_current_[0]};
        
        msg.name = motor_names_;
        msg.position = pos_temp;
        msg.velocity = vel_temp;
        // msg.effort = eff_temp;   // not required for now, always zero for igus motors
        joint_state_pub_.publish(msg); // uncomment
    }

    // void CartesianRobotHW::write(const ros::Time &time, const ros::Duration &period){
    //! @param pos: a double 3-element vector in radians
    void CartesianRobotHW::write(const std::vector<double> &pos){
        if (!is_active_)
            return;
        igus_joint_position_command_[0] = pos[0];
        igus_joint_position_command_[1] = pos[1];
        epos_joint_position_command_[0] = pos[2];
        // double e1 = pos[0] - igus_joint_position_[0];
        // double e2 = pos[1] - igus_joint_position_[1];
        // double e3 = pos[2] - epos_joint_position_[0];
        // ROS_INFO_STREAM("############################");
        // ROS_INFO_STREAM("Delta 1: " << e1);
        // ROS_INFO_STREAM("Delta 2: " << e2);
        // ROS_INFO_STREAM("Delta 3: " << e3);
        // TODO do some safety filter here [saturation, limit, etc]
        // set igus limits
        for(size_t i=0;i<2;i++){
            if (pos[i] < igus_robot.m_pJoints[i]->get_MinPosition()){
                ROS_WARN_STREAM(pos[0] << " is less than " << igus_robot.m_pJoints[i]->get_JointName() << " min position [" << igus_robot.m_pJoints[i]->get_MinPosition() << "]");
                igus_joint_position_command_[i] = igus_robot.m_pJoints[i]->get_MinPosition();
            } else if (pos[i] > igus_robot.m_pJoints[i]->get_MaxPosition()){
                ROS_WARN_STREAM(pos[0] << " is greater than " << igus_robot.m_pJoints[i]->get_JointName() << " max position [" << igus_robot.m_pJoints[i]->get_MaxPosition() << "]");
                igus_joint_position_command_[i] = igus_robot.m_pJoints[i]->get_MaxPosition();
            }
        }

        // TODO check third link limits

        // write igus motors
        ROS_DEBUG_STREAM("Target Joint Values: [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]");
        
        if (test_with_hardware)
        {
            // write igus motors
            igus_robot.Write(igus_joint_position_command_, igus_joint_velocity_command_, igus_joint_effort_command_);
            // write epos motor
            epos_manager_.write(epos_joint_position_command_, epos_joint_velocity_command_, epos_joint_effort_command_);
        }
    }

    void CartesianRobotHW::init_motors(ros::NodeHandle &root_nh, ros::NodeHandle &robot_nh,
                const std::vector<std::string> &motor_names)
    {
        // third motor will be handled by epos4 board
        std::vector< std::string > epos_motors = {motor_names[2]};
        epos_manager_.init(root_nh, robot_nh, epos_motors);
        epos_manager_.setZero(); // TODO UNCOMMENT
    }

    void CartesianRobotHW::updateDiagnostics(){}    // TODO
    void CartesianRobotHW::doSwitch(){}             // TODO

    void CartesianRobotHW::reference_igus_motors()
    {
        igus_robot.DoReferencing();
    }

    void CartesianRobotHW::update_rosparam_values()
    {
        // std::string cartesian_robot_name;
        // if (root_nh_.hasParam("cartesian_robot_name"))
        //     ROS_ERROR_STREAM("cartesian_robot_name not available in rosparam.");
        // root_nh_.param<std::string>("cartesian_robot_name", cartesian_robot_name, "cartesian_");
        
        // if (root_nh_.hasParam("/"+cartesian_robot_name+"/needle/x"))
        //     ROS_ERROR_STREAM("/"+cartesian_robot_name+"/needle/x" << " not available in rosparam.");
        // root_nh_.param<double>("/"+cartesian_robot_name+"/needle/x", needle_[0]);
        // if (root_nh_.hasParam("/"+cartesian_robot_name+"/needle/y"))
        //     ROS_ERROR_STREAM("/"+cartesian_robot_name+"/needle/y" << " not available in rosparam.");
        
        // if (root_nh_.hasParam("/"+cartesian_robot_name+"/home_point/x") || root_nh_.hasParam("/"+cartesian_robot_name+"/home_point/y") || root_nh_.hasParam("/"+cartesian_robot_name+"/home_point/z"))
        //     ROS_ERROR_STREAM("/"+cartesian_robot_name+"/home_point/..." << " not available in rosparam.");
        // root_nh_.param<double>("/"+cartesian_robot_name+"/home_point/x", home_[0]);
        // root_nh_.param<double>("/"+cartesian_robot_name+"/home_point/y", home_[1]);
        // root_nh_.param<double>("/"+cartesian_robot_name+"/home_point/z", home_[2]);
        
        // if (root_nh_.hasParam("/"+cartesian_robot_name+"/home_point/x") || root_nh_.hasParam("/"+cartesian_robot_name+"/home_point/y") || root_nh_.hasParam("/"+cartesian_robot_name+"/home_point/z"))
        //     ROS_ERROR_STREAM("/"+cartesian_robot_name+"/home_point/..." << " not available in rosparam.");
        // root_nh_.param<double>("/"+cartesian_robot_name+"/gear_ratios/joint1", joint_ratios_[0]);
        // root_nh_.param<double>("/"+cartesian_robot_name+"/gear_ratios/joint2", joint_ratios_[1]);
        // root_nh_.param<double>("/"+cartesian_robot_name+"/gear_ratios/joint3", joint_ratios_[2]);

        // root_nh_.param<double>("/"+cartesian_robot_name+"/sewing_start_point/x", sewing_point_[0]);
        // root_nh_.param<double>("/"+cartesian_robot_name+"/sewing_start_point/y", sewing_point_[1]);
        // root_nh_.param<double>("/"+cartesian_robot_name+"/sewing_start_point/z", sewing_point_[2]);

        // root_nh_.param<double>("/"+cartesian_robot_name+"/offsets/x", offsets_[0]);
        // root_nh_.param<double>("/"+cartesian_robot_name+"/offsets/y", offsets_[1]);
        // root_nh_.param<double>("/"+cartesian_robot_name+"/offsets/z", offsets_[2]);
        
        // root_nh_.param<double>("/"+cartesian_robot_name+"/third_link/radius", third_link_radius_);
        

        // CONFIG
        needle_ = {1.0, 0.215}; //{1.145, 0.215};
        home_ = {1.08,0.215,0.0};
        joint_ratios_ = {0.01114,0.01114,13.33};
        sewing_point_ = {1.08,0.215,-3.1415};//{0.35,0.215,0.0};
        offsets_ = {0.3,0,0};
        third_link_radius_ = 0;
    }

    void CartesianRobotHW::ik_calculations(double &x, double &y, double &z, bool absolute)
    {
        // calculate joint1, joint2, and joint3 positions and convert x, y, z to motor rotation angles in rad.

        if (absolute){
            ROS_INFO_STREAM("Asbolute");
            x = (x - offsets_[0]) / joint_ratios_[0];
            y = (y - offsets_[1]) / joint_ratios_[1];
            z = z * joint_ratios_[2];   // TODO offset in z? needed?
            return;
        }
        // if relative to needle position:
        if (z > M_PI/2)
        {
            z = z - 2 * M_PI;
        }
        z = z - M_PI/2;
        z = -z;
        z = z-M_PI;
        
        double calc_x = needle_[0] - (x * std::cos(z) - y * std::sin(z) );
        double calc_y = needle_[1] - (y * std::cos(z) + x * std::sin(z) );
        
        // add offsets
        calc_x = calc_x - offsets_[0];
        calc_y = calc_y - offsets_[1];
        
        x = calc_x / joint_ratios_[0];
        y = calc_y / joint_ratios_[1];
        z = z * joint_ratios_[2];
    }

    void CartesianRobotHW::ik(const geometry_msgs::Vector3ConstPtr &msg)
    {
        double x = msg->x, y = msg->y, z = msg->z;
        // ROS_INFO_STREAM("Received msg: " << x << ", " << y << ", " << z << "], [m, m, rad]");
        ik_calculations(x,y,z);
        // ROS_INFO_STREAM("after IK: " << x << ", " << y << ", " << z << " [rad]");

        // ///
        // sensor_msgs::JointState msg2;
        // std_msgs::Header header;
        // header.stamp = ros::Time::now();
        // msg2.header = header;

        // // // read maxon motor state
        // // epos_manager_.read(epos_joint_position_, epos_joint_velocity_, epos_joint_current_);
        // // // read igus motors
        // // igus_robot.Read(igus_joint_position_, igus_joint_velocity_, igus_joint_current_);

        // // fill in the joint state msg
        // std::vector<double> pos_temp = {x, y, z/joint_ratios_[2]};
        
        // msg2.name = motor_names_;
        // msg2.position = pos_temp;
        // joint_state_pub_.publish(msg2);
        // ///
        
        std::vector<double> joint_pos = {x, y, z};
        // write to hardware
        write(joint_pos);
    }

    void CartesianRobotHW::gotoHandler(const geometry_msgs::Vector3ConstPtr &msg)
    {
        double x = msg->x, y = msg->y, z = msg->z;
        ik_calculations(x,y,z,true);
        std::vector<double> joint_pos = {x, y, z};        
        // write to hardware
        write(joint_pos);
    }


    bool CartesianRobotHW::ThirdLinkSetZero(std_srvs::TriggerRequest  &req, std_srvs::TriggerResponse &res)
    {
        epos_manager_.setZero();
        res.success = true;
        res.message = "cartesian robot third link set to zero at this position.";
        return true;
    }

    bool CartesianRobotHW::SewingPointHandler(std_srvs::TriggerRequest  &req, std_srvs::TriggerResponse &res)
    {
        //temp :
        ros::Time time;
        ros::Duration period;
        read(time, period);
        double t1 = sewing_point_[0];
        double t2 = sewing_point_[1];
        double t3 = sewing_point_[2];
        ik_calculations(t1,t2,t3,true);     // goto absolute position
        std::vector<double> command = {t1, t2, epos_joint_position_[0]};
        write(command);
        ROS_INFO_STREAM("Now sending cartesian robot to : [" << t1 << ", " << t2 << "] [rad]");
        sleep(5);
        command = {t1, t2, t3};
        write(command);
        res.success = true;
        res.message = "cartesian robot has moved to sewing start point.";
        return true;
    }

    bool CartesianRobotHW::SetActiveStateHandler(std_srvs::SetBoolRequest  &req, std_srvs::SetBoolResponse &res)
    {
        if(req.data){
            is_active_ = req.data;
            res.success = true;
            res.message = "cartesian robot activated.";
            ROS_INFO_STREAM("cartesian robot activated.");
            return true;
        }else if (!req.data)
        {
            is_active_ = req.data;
            res.success = true;
            res.message = "cartesian robot deactivated.";
            ROS_INFO_STREAM("cartesian robot deactivated.");
            return true;
        } else
            return false;
    }
}






