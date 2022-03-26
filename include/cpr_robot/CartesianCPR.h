#ifndef CARTESIANCPR_H
#define CARTESIANCPR_H

#include <vector>
#include <string>

namespace cpr_robot
{
	//! \class CartesianCPR CartesianCPR.h <cpr_robot.h>
	//! \brief Class representing a robolink 2DOF small version robot from igus. 
	//!
	//! All model specific parameters are handled by this class
    class CartesianCPR : public Robot
    {
        public:
            CartesianCPR();
            virtual ~CartesianCPR();
            void Init(const std::vector<std::string> &joint_names);
            void Read(std::vector<double> &positions, std::vector<double> &velocities, std::vector<double> &efforts);
            void Write(std::vector<double> &positions, std::vector<double> &velocities, std::vector<double> &efforts);
    };
}

#endif