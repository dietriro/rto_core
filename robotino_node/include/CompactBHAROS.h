/*
 * CompactBHAROS.h
 *
 *  Created on: 08.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef COMPACTBHAROS_H_
#define COMPACTBHAROS_H_

#include "rec/robotino/api2/CompactBHA.h"

#include <ros/ros.h>
#include "robotino_msgs/BHAReadings.h"
#include "robotino_msgs/SetBHAPressures.h"

class CompactBHAROS : public rec::robotino::api2::CompactBHA
{
public:
	CompactBHAROS();
	~CompactBHAROS();

	void setTimeStamp(ros::Time stamp);

private:
	ros::NodeHandle nh_;

	ros::Subscriber bha_sub_;

	ros::Publisher bha_pub_;

	robotino_msgs::BHAReadings bha_msg_;

	ros::Time stamp_;

	void pressuresChangedEvent( const float* pressures, unsigned int size );
	void cablepullChangedEvent( const float* cablepull, unsigned int size );

	void setBHAPressuresCallback( const robotino_msgs::SetBHAPressuresConstPtr &msg);
};

#endif /* COMPACTBHAROS_H_ */
