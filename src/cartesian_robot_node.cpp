#include <ros/ros.h>
// #include <ros/console.h>
#include <controller_manager/controller_manager.h>
#include "CartesianRobotHW.cpp"

int main(int argc, char *argv[]){
    ros::init(argc, argv, "cartesian_robot_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::vector< std::string > motor_names;
    ros::removeROSArgs(argc, argv, motor_names);
    motor_names.erase(motor_names.begin()); // remove exec path
    
    CartesianRobotHW hardware;
    if (!hardware.init(nh, pnh, motor_names)) {
        ROS_FATAL("Failed to initialize motors");
        return 1;
    }
    ROS_INFO("Cartesian Robot Initialized");

    controller_manager::ControllerManager controllers(&hardware, nh);
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate control_rate(50);
    ros::Time last(ros::Time::now());
    while (ros::ok()) {
        const ros::Time now(ros::Time::now());
        const ros::Duration period(now - last);
        hardware.read(now, period);
        controllers.update(now, period);
        hardware.write(now, period);
    //   // hardware.updateDiagnostics(); // TODO
        last = now;
        control_rate.sleep();
    }
    return 1;
}