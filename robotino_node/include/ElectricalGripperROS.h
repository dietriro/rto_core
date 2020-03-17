/*
 * ElectricalGripperROS.h
 *
 *  Created on: 09.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ELECTRICALGRIPPERROS_H_
#define ELECTRICALGRIPPERROS_H_

#include "rec/robotino/api2/ElectricalGripper.h"

#include <ros/ros.h>
#include "robotino_msgs/GripperState.h"
#include "robotino_msgs/SetGripperState.h"


class ElectricalGripperROS : public rec::robotino::api2::ElectricalGripper
{
public:
	ElectricalGripperROS();
	~ElectricalGripperROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Publisher gripper_pub_;

	ros::ServiceServer set_gripper_server_;

	robotino_msgs::GripperState gripper_msg_;

	ros::Time stamp_;

	bool setGripperStateCallback(
			robotino_msgs::SetGripperState::Request &req,
			robotino_msgs::SetGripperState::Response &res);

	void stateChangedEvent( int state );

};

#endif /* ELECTRICALGRIPPERROS_H_ */
