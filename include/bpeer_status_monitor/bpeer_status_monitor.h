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
#include <std_msgs/Bool.h>

#include <log4cplus/logger.h>
#include <log4cplus/fileappender.h>
#include <log4cplus/consoleappender.h>
#include <log4cplus/layout.h>
#include <loggingmacros.h>

using namespace log4cplus;
using namespace log4cplus::helpers;

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
	void process_reportLocalizationStatusSpin( const ros::TimerEvent& e );

	void judge_status_output_log( const bool laser_alive_, const bool odom_alive_ );

	ros::Timer timer1_;
	ros::Timer timer2_;
	ros::Timer timer3_;

	int freq1;
	int freq2;
	int freq3;

	ros::NodeHandle nh_;

	ros::Publisher status_Q_pub_;
	ros::Publisher status_Q_loc_pub_;

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

	SharedAppenderPtr _append;
	Logger _logger;

};


#endif //BPEER_STATUS_MONITOR_BPEER_STATUS_MONITOR_H
