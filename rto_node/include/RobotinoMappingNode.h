#ifndef ROBOTINOMAPPINGNODE_H_
#define ROBOTINOMAPPINGNODE_H_

#include "ComROS.h"
#include "MappingROS.h"
#include "InitialPoseROS.h"
#include "NavGoalROS.h"
#include <ros/ros.h>

class RobotinoMappingNode
{
public:
	RobotinoMappingNode();
	~RobotinoMappingNode();

	bool spin();

private:
	ros::NodeHandle nh_;

	std::string hostname_;

	ComROS com_;
	MappingROS mappingRos_;
	InitialPoseROS initialPoseROS_;
	NavGoalROS navGoalROS_;

	void initModules();
};

#endif /* ROBOTINOMAPPINGNODE_H_ */
