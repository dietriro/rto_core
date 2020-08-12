#include "RobotinoMappingNode.h"

#include <ros/ros.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "robotino_mapping_node");
    RobotinoMappingNode rn;
    rn.spin();
    ros::spin();
	return 0;
}
