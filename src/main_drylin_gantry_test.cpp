#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include <cpr_robot.h>
#include <sstream>
#include <cmath>

double x_ref = 0.0;
double y_ref = 0.0;
double qx_ref = 0.0;
double qy_ref = 0.0;
double x_radius = 11.14;                // [mm]
double y_radius = 11.14;                // [mm]

void cb(const geometry_msgs::PoseStamped msg)
{
  x_ref = msg.pose.position.x;          // desired position in the task space
  y_ref = msg.pose.position.y;
  
  // Inverse Kinematics
  qx_ref = x_ref / x_radius;            // desired joint (= motor) position in radians
  qy_ref = y_ref / y_radius;
  
 auto x_ticks = robot.robot.m_pJoints[0].PositionToTicks(qx_ref);   // desired joint (= motor) position in encoder ticks
 auto y_ticks = robot.robot.m_pJoints[1].PositionToTicks(qy_ref);
  
    robot.m_pJoints[0].m_pModule.set_DesiredPosition(m_MotorPosition+m_MotorIncrement);
    robot.m_pJoints[0].m_pModule.Command_SetJoint(x_ticks, robot.m_pJoints[0].m_pModule.m_DOutputs);
      
  robot.m_pJoints[1].m_pModule.set_DesiredPosition(m_MotorPosition+m_MotorIncrement);
  robot.m_pJoints[1].m_pModule.Command_SetJoint(y_ticks, robot.m_pJoints[1].m_pModule.m_DOutputs);
}

int main(int argc, char **argv)
{
  // Robot class and relative node
  ros::init(argc, argv, "drylin_gantry");
  cpr_robot::drylin_gantry robot;
  robot.Init();
  
  // Subscriber node
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/sync_pos", 1000, cb);
    // ros::Rate rate(50);
    // while (ros::ok())
    // {
    //     robot.Read();
    //     robot.PublishState();
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    
  return 0;
}