//
// Created by bpeer on 17-10-10.
//

#ifndef BPEER_STATUS_MONITOR_BPEER_STATUS_MONITOR_H
#define BPEER_STATUS_MONITOR_BPEER_STATUS_MONITOR_H

#include <iostream>
#include "ros/ros.h"
#include <bprobot/msg_Q_status_monitor.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <signal.h>

using namespace std;

class StatusMonitor
{
public:
	StatusMonitor();
	~StatusMonitor(){
	};
	void status_process();
	void monitor_process_alive();
	bool process_is_ok( const char* node );

private:
	void process_receiveDataSpin( const ros::TimerEvent& e );
	ros::Timer timer_;
	int freq;

	ros::NodeHandle nh_;

	ros::Publisher status_Q_pub_;

	//sensor status
	ros::Subscriber laser_sub_;
	bool bLaser_alive;
	ros::Time mLaser_current_stamp;

	ros::Subscriber odom_sub_;
	bool bOdom_alive;
	ros::Time mOdom_current_stamp;

	//process status
	char planner_exe[128];  //进程名
	bool bPlanner_status;  //进程状态

	char amcl_exe[128];
	bool bAmcl_status;

	char returnCharging_exe[128];
	bool bReturnCharging_status;

	char identifyCharging_exe[128];
	bool bIdentifyCharging_status;

	char autoMapping_exe[128];
	bool bAutoMapping_status;

	char tf2topic_exe[128];
	bool bTf2Topic_status;
};


#endif //BPEER_STATUS_MONITOR_BPEER_STATUS_MONITOR_H
