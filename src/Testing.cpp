#include <cpr_robot.h>

namespace cpr_robot
{
    Testing::Testing() :
        Robot(1,0)
    {
        set_ModelName("Testing");
        set_GearRatio(0,528.2);
        set_JointName(0,"joint1");
        set_TicksPerMotorRotation(0,1440);
        set_MaxVelocity(0,30.0);
        set_MinPosition(0,-130.0);
        set_MaxPosition(0,130.0);
        set_MotorOffset(0,0);
        // define_Output(false,0,0,"Digital out 1");
    }
            
    Testing::~Testing()
    {

    }

}