#ifndef TESTING_H
#define TESTING_H

namespace cpr_robot
{
	//! \class CPRMover6 CPRMover6.h <cpr_robot.h>
	//! \brief Class representing a Mover6 robot from Commonplace Robotics GmbH. 
	//!
	//! All model specific parameters are handled by this class
    class Testing : public Robot
    {
        public:
            Testing();
            virtual ~Testing();
    };
}

#endif