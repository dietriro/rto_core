/*
 * RobotinoLocalMoveClient.h
 *
 *  Created on: 13.12.2011
 *      Author: indorewala@servicerobotics.eu
 */

#ifndef ROBOTINOLOCALMOVECLIENT_H_
#define ROBOTINOLOCALMOVECLIENT_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <robotino_local_move/LocalMoveAction.h>

typedef actionlib::SimpleActionClient<robotino_local_move::LocalMoveAction> Client;

class RobotinoLocalMoveClient
{
public:
	RobotinoLocalMoveClient();
	~RobotinoLocalMoveClient();

private:
	ros::NodeHandle nh_;

	Client client_;

	robotino_local_move::LocalMoveGoal goal_;

	float max_time_;

public:
	bool checkServer();
	void spin();
	void setMaxTime( const float& time );
	void sendGoal( const robotino_local_move::LocalMoveGoal& goal );

};

#endif /* ROBOTINOLOCALMOVECLIENT_H_ */
