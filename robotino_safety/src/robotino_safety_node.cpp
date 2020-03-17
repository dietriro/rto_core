/*
 * robotino_safety_node.cpp
 *
 *  Created on: Mar 21, 2012
 *      Author: indorewala@servicerobotics.eu
 */

#include "RobotinoSafety.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_safety_node");

	RobotinoSafety rs;
	rs.spin();

	return 0;
}
