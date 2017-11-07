//
// Created by bpeer on 17-10-10.
//

#ifndef BPEER_STATUS_MONITOR_BPEER_STATUS_MONITOR_H
#define BPEER_STATUS_MONITOR_BPEER_STATUS_MONITOR_H

#include <iostream>
#include "ros/ros.h"
#include <bprobot/msg_Q_status_monitor.h>
#include <bprobot/msg_A_STATUS_REPORT.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <signal.h>
#include <std_msgs/Int8.h>

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
	void process_reportStatusSpin( const ros::TimerEvent& e );

	ros::Timer timer1_;
	ros::Timer timer2_;

	int freq1;
	int freq2;

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
	bool mbPlanner_status;  //进程状态

	char amcl_exe[128];
	bool mbAmcl_status;

	char returnCharging_exe[128];
	bool mbReturnCharging_status;

	char identifyCharging_exe[128];
	bool mbIdentifyCharging_status;

	char autoMapping_exe[128];
	bool mbAutoMapping_status;

	char tf2topic_exe[128];
	bool mbTf2Topic_status;

	//系统模块状态量
	int mnStatus;
	ros::Publisher statusReport_pub_;

	ros::Subscriber autoCreateMap_sub_;
	ros::Subscriber autoCharge_sub_;
	ros::Subscriber localization_sub_;

};


#endif //BPEER_STATUS_MONITOR_BPEER_STATUS_MONITOR_H
