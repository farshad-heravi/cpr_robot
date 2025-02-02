#ifndef CPRMOVER6_H
#define CPRMOVER6_H

namespace cpr_robot
{
	//! \class CPRMover6 CPRMover6.h <cpr_robot.h>
	//! \brief Class representing a Mover6 robot from Commonplace Robotics GmbH. 
	//!
	//! All model specific parameters are handled by this class
    class CPRMover6 : public Robot
    {
        public:
            CPRMover6();
            virtual ~CPRMover6();
    };
}

#endif